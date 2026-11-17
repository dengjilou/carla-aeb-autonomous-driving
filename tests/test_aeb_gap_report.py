import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scenarios'))
from report_aeb_gap import audit_run


class GapAuditTests(unittest.TestCase):
    def test_rejects_a_guard_flag_on_legacy_false_braking(self):
        name='equal_4_body_acc'
        with zipfile.ZipFile(ROOT/'docs/aeb_gap_traces.zip') as archive:
            run=json.loads(archive.read('equal-a/summary.json'))['runs'][name]
            trace=archive.read('equal-a/'+name+'.csv')
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            (folder/(name+'.csv')).write_bytes(trace)
            audit_run(folder,name,run)
            run['config']['aeb_closing_guard']=True
            with self.assertRaises(AssertionError):
                audit_run(folder,name,run)

    def test_rejects_distance_and_ttc_corruption(self):
        name='diagnosis_stress_body_baseline'
        with zipfile.ZipFile(ROOT/'docs/aeb_gap_traces.zip') as archive:
            run=json.loads(archive.read('a/summary.json'))['runs'][name]
            rows=list(csv.DictReader(io.StringIO(archive.read('a/'+name+'.csv').decode())))
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            def write():
                with (folder/(name+'.csv')).open('w',newline='') as output:
                    writer=csv.DictWriter(output,fieldnames=list(rows[0]))
                    writer.writeheader(); writer.writerows(rows)
            write()
            audit_run(folder,name,run)
            for key in ('aeb_distance_m','aeb_ttc_s'):
                original=rows[0][key]
                rows[0][key]='999'
                write()
                with self.subTest(key=key),self.assertRaises(AssertionError):
                    audit_run(folder,name,run)
                rows[0][key]=original
