"""Extract selected KITTI odometry sequences using CRC-verified byte ranges."""
import argparse
import concurrent.futures
import io
import http.client
import re
import struct
import time
import urllib.request
import zipfile
import zlib
from pathlib import Path

BASE = "https://avg-kitti.s3.eu-central-1.amazonaws.com/"


class RemoteZip(io.RawIOBase):
    def __init__(self, name):
        self.url = BASE + name
        with urllib.request.urlopen(urllib.request.Request(self.url, method="HEAD"), timeout=60) as r:
            self.size = int(r.headers["Content-Length"])
        self.pos = 0

    def seek(self, offset, whence=0):
        self.pos = (0, self.pos, self.size)[whence] + offset
        return self.pos

    def tell(self):
        return self.pos

    def fetch(self, offset, end):
        for attempt in range(7):
            try:
                url = self.url if attempt%2==0 else self.url.replace(BASE,"https://s3.eu-central-1.amazonaws.com/avg-kitti/")
                req = urllib.request.Request(url + f"?range={offset}-{end}&retry={attempt}", headers={"Range": f"bytes={offset}-{end-1}"})
                with urllib.request.urlopen(req, timeout=90) as r:
                    if r.headers.get("Content-Range") != f"bytes {offset}-{end-1}/{self.size}":
                        raise ValueError("Range response mismatch")
                    data = r.read()
                if len(data) != end-offset:
                    raise ValueError("Incomplete range")
                return data
            except (OSError, ValueError, http.client.HTTPException):
                if attempt == 6:
                    raise
                time.sleep(2**attempt)

    def read(self, n=-1):
        end = self.size if n < 0 else min(self.size, self.pos+n)
        if end <= self.pos:
            return b""
        data = self.fetch(self.pos,end)
        self.pos = end
        return data


def fetch(root, sequences=("05", "07")):
    wanted = "(?:" + "|".join(re.escape(s) for s in sequences) + ")"
    for archive, pattern in [
        ("data_odometry_poses.zip",rf"dataset/poses/{wanted}\.txt"),
        ("data_odometry_calib.zip",rf"dataset/sequences/{wanted}/(calib|times)\.txt")]:
        with urllib.request.urlopen(BASE+archive, timeout=90) as response:
            with zipfile.ZipFile(io.BytesIO(response.read())) as z:
                for name in z.namelist():
                    if re.fullmatch(pattern, name):
                        z.extract(name, root)
    remote=RemoteZip("data_odometry_velodyne.zip")
    with zipfile.ZipFile(remote) as z:
        files=sorted(z.infolist(),key=lambda f:f.header_offset)
        jobs=[]
        for i,f in enumerate(files):
            if not re.fullmatch(rf"dataset/sequences/{wanted}/velodyne/\d{{6}}\.bin", f.filename):
                continue
            p=root/f.filename
            if p.exists() and p.stat().st_size==f.file_size and zlib.crc32(p.read_bytes())==f.CRC:
                continue
            end=files[i+1].header_offset if i+1<len(files) else z.start_dir
            if not jobs or f.header_offset!=jobs[-1][-1][1] or end-jobs[-1][0][0].header_offset>8*1024**2:
                jobs.append([])
            jobs[-1].append((f,end))
    def unpack(group):
        origin=group[0][0].header_offset
        block=remote.fetch(origin,group[-1][1])
        for f,_ in group:
            start=f.header_offset-origin
            if block[start:start+4]!=b'PK\x03\x04':
                raise ValueError(f.filename)
            name,extra=struct.unpack_from("<HH",block,start+26)
            pos=start+30+name+extra
            data=block[pos:pos+f.compress_size]
            if f.compress_type==zipfile.ZIP_DEFLATED:
                data=zlib.decompress(data,-15)
            elif f.compress_type!=zipfile.ZIP_STORED:
                raise ValueError(f"Unsupported compression {f.compress_type}")
            if zlib.crc32(data)!=f.CRC or len(data)!=f.file_size:
                raise ValueError(f"CRC mismatch: {f.filename}")
            dest=root/f.filename
            dest.parent.mkdir(parents=True,exist_ok=True)
            tmp=dest.with_suffix('.part')
            tmp.write_bytes(data)
            tmp.replace(dest)
    print(f"{sum(map(len,jobs))} scans pending in {len(jobs)} blocks",flush=True)
    jobs.sort(key=lambda group:min(f.filename for f,_ in group))
    with concurrent.futures.ThreadPoolExecutor(max_workers=48) as pool:
        futures={pool.submit(unpack,job):job for job in jobs}
        failed=[]
        for i,future in enumerate(concurrent.futures.as_completed(futures),1):
            try:
                future.result()
            except (OSError,ValueError,http.client.HTTPException) as exc:
                print(f"Retry after other blocks: {type(exc).__name__}",flush=True)
                failed.append(futures[future])
            if i%25==0:
                print(f"{i}/{len(jobs)} blocks",flush=True)
        for job in failed:
            unpack(job)
    for seq in sequences:
        n=len((root/'dataset/poses'/f'{seq}.txt').read_text().splitlines())
        scans=root/'dataset/sequences'/seq/'velodyne'
        if sorted(p.name for p in scans.glob('*.bin'))!=[f'{i:06}.bin' for i in range(n)]:
            raise ValueError(f'Incomplete sequence {seq}')


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("root",type=Path)
    p.add_argument("--sequences", nargs="+", choices=("04","05","07"), default=("05","07"))
    args=p.parse_args()
    fetch(args.root, args.sequences)
