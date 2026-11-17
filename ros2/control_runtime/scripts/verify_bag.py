#!/usr/bin/env python3
"""Replay only bagged inputs into a fresh node and compare regenerated commands."""
import argparse
import json
import signal
import subprocess
import time
from pathlib import Path

import rosbag2_py
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from control_runtime.msg import ControlState, ControlCommand, LqrState

FIELDS=('reference','throttle','brake','steer','latched_brake')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag,storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    expected={}
    states={}
    first_time=last_time=None
    while reader.has_next():
        topic,data,stamp=reader.read_next()
        if first_time is None:
            first_time=stamp
        last_time=stamp
        if topic=='/control/command':
            msg=deserialize_message(data,ControlCommand)
            assert msg.status==0
            expected[(msg.session,msg.sequence)]=msg
        elif topic in ('/control/state','/control/lqr_state'):
            msg=deserialize_message(data,LqrState if topic.endswith('lqr_state') else ControlState)
            state=msg.state if topic.endswith('lqr_state') else msg
            states[(state.session,state.sequence)]=msg
    assert states and set(states)==set(expected),'Incomplete recording'
    rclpy.init()
    node=Node('bag_verifier')
    received={}
    def command(msg):
        received[(msg.session,msg.sequence)]=msg
    node.create_subscription(ControlCommand,'/control/command',command,1000)
    # Discovery readiness uses a publisher but sends no state before bag play.
    publisher=node.create_publisher(ControlState,'/control/state',1)
    executable=Path(get_package_prefix('control_runtime'))/'lib/control_runtime/control_node'
    controller=subprocess.Popen([str(executable),'--ros-args','-p','use_sim_time:=true'])
    player=None
    try:
        deadline=time.monotonic()+10
        while (publisher.get_subscription_count()<1 or node.count_publishers('/control/command')<1):
            if time.monotonic()>deadline:
                raise TimeoutError('ROS graph discovery')
            rclpy.spin_once(node,timeout_sec=.05)
        player=subprocess.Popen(['ros2','bag','play',args.bag,'--rate','0.5','--topics',
                                 '/control/state','/control/lqr_state','/clock','/reference_path','/actual_path','/tf'])
        deadline=time.monotonic()+max(45,2*(last_time-first_time)/1e9+15)
        while len(received)<len(expected) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        assert set(received)==set(expected),(len(received),len(expected))
        maximum=0.
        for key,want in expected.items():
            actual=received[key]
            assert actual.status==0
            assert actual.stamp==want.stamp
            for field in FIELDS:
                maximum=max(maximum,abs(getattr(want,field)-getattr(actual,field)))
        assert maximum<=1e-9
        result=dict(matched_frames=len(received),command_abs_error_max=maximum,
                    command_tolerance=1e-9,playback_rate=.5,recorded_commands_republished=False,
                    first_sequence=min(k[1] for k in states),last_sequence=max(k[1] for k in states))
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
    finally:
        for process in (player,controller):
            if process and process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
