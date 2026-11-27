"""Replay the held-out sequence's worst 100 m segment from local KITTI scans."""
import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


def grab_window(window_id, filename):
    from PyQt5.QtWidgets import QApplication
    app=QApplication.instance() or QApplication([])
    image=app.primaryScreen().grabWindow(window_id)
    if image.isNull() or not image.save(str(filename)):
        raise RuntimeError('Could not capture the selected RViz window')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path)
    parser.add_argument('npz',type=Path)
    parser.add_argument('--capture-window',type=lambda x:int(x,0),
                        help='X11 ID of the replay RViz window; save its last frame beside the NPZ')
    args=parser.parse_args()
    with np.load(args.npz) as replay:
        gt,lidar=(replay[key] for key in ('gt','lidar'))
        start,end=int(replay['start']),int(replay['end'])
    calib=(args.dataset/'sequences/07/calib.txt').read_text().splitlines()
    tr=np.eye(4)
    tr[:3]=np.fromstring(next(s[3:] for s in calib if s.startswith('Tr:')),sep=' ').reshape(3,4)
    truth=np.linalg.inv(tr)@gt@tr
    # Shared local origin for visibility, keeping the discrepancy between trajectories.
    segment=np.vstack((truth[start:end+1,:3,3],lidar[start:end+1,:3,3]))
    offset=(segment.min(axis=0)+segment.max(axis=0))/2
    rclpy.init()
    node=rclpy.create_node('kitti_07_replay')
    cloud=node.create_publisher(PointCloud2,'/kitti/points',2)
    routes=[node.create_publisher(RosPath,'/kitti/'+k,2) for k in ('truth','estimate')]
    try:
        deadline=time.monotonic()+45
        while any(p.get_subscription_count()==0 for p in [cloud,*routes]):
            if time.monotonic()>deadline: raise RuntimeError('RViz subscriptions unavailable')
            rclpy.spin_once(node,timeout_sec=.1)
        for i in range(start,end+1):
            header=Header(frame_id='map',stamp=node.get_clock().now().to_msg())
            for pub,mats in zip(routes,(truth,lidar)):
                msg=RosPath(header=header)
                for point in mats[start:i+1,:3,3]-offset:
                    pose=PoseStamped(header=header)
                    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point.tolist()
                    pose.pose.orientation.w=1.
                    msg.poses.append(pose)
                pub.publish(msg)
            scan=np.fromfile(args.dataset/'sequences/07/velodyne'/f'{i:06}.bin','<f4').reshape(-1,4)[::10,:3]
            points=scan@lidar[i,:3,:3].T+lidar[i,:3,3]-offset
            cloud.publish(create_cloud_xyz32(header,points.astype(np.float32)))
            rclpy.spin_once(node,timeout_sec=.005)
            time.sleep(.075)
        time.sleep(1)
        if args.capture_window:
            grab_window(args.capture_window,args.npz.with_name('rviz_07.png'))
        print(f'RViz subscribed to cloud and both paths; frames {start} through {end} sent.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
