#!/usr/bin/env python3
"""Local NDJSON transport, simulation clock and standard ROS visualization."""
import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from control_runtime.msg import ControlState, ControlCommand

INPUTS=('dt','speed','cruise','gap','closing','heading','path_heading','cross_track','max_angle','aeb_request')
CONFIG=('gap_gain','relative_gain','standstill_gap','headway','stanley_gain','steer_limit','steer_alpha')
OUTPUTS=('reference','throttle','brake','steer','latched_brake')


class Bridge(Node):
    def __init__(self):
        super().__init__('control_bridge')
        self.publisher=self.create_publisher(ControlState,'/control/state',1)
        self.clock=self.create_publisher(Clock,'/clock',1)
        self.reference=self.create_publisher(Path,'/reference_path',1)
        self.actual=self.create_publisher(Path,'/actual_path',1)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(ControlCommand,'/control/command',self.command,1)
        self.condition=threading.Condition()
        self.reply=None
        self.pending=None
        self.path=Path()
        self.session=None
        self.server=socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.server.bind(('127.0.0.1',8766))
        self.server.listen(1)
        self.server.settimeout(.5)
        threading.Thread(target=self.serve,daemon=True).start()

    def command(self,msg):
        with self.condition:
            if self.pending==(msg.session,msg.sequence):
                self.reply=msg
                self.condition.notify_all()

    def publish_visuals(self,packet,stamp):
        if packet['session']!=self.session:
            self.session=packet['session']
            self.path=Path()
        if 'path' in packet:
            path=Path()
            path.header.frame_id='map'
            path.header.stamp=stamp
            for x,y in packet['path']:
                p=PoseStamped()
                p.header=path.header
                p.pose.position.x=float(x)
                p.pose.position.y=-float(y)
                p.pose.orientation.w=1.
                path.poses.append(p)
            self.reference.publish(path)
        if 'pose' in packet:
            pose=packet['pose']
            p=PoseStamped()
            p.header.frame_id='map'
            p.header.stamp=stamp
            p.pose.position.x=float(pose['x'])
            p.pose.position.y=-float(pose['y'])
            p.pose.position.z=float(pose['z'])
            p.pose.orientation.z=math.sin(-pose['yaw']/2)
            p.pose.orientation.w=math.cos(-pose['yaw']/2)
            transform=TransformStamped()
            transform.header=p.header
            transform.child_frame_id='ego'
            transform.transform.translation.x=p.pose.position.x
            transform.transform.translation.y=p.pose.position.y
            transform.transform.translation.z=p.pose.position.z
            transform.transform.rotation=p.pose.orientation
            self.tf.sendTransform(transform)
            self.path.header=p.header
            self.path.poses.append(p)
            self.actual.publish(self.path)

    def request(self,packet):
        state=ControlState()
        state.session=int(packet['session'])
        state.sequence=int(packet['sequence'])
        elapsed=state.sequence*packet['input']['dt']
        state.stamp=Time(sec=int(elapsed),nanosec=int(round((elapsed%1)*1e9)))
        if state.stamp.nanosec>=1000000000:
            state.stamp.sec+=1
            state.stamp.nanosec=0
        for key in INPUTS:
            setattr(state,key,float(packet['input'][key]))
        for key in CONFIG:
            setattr(state,key,float(packet['config'][key]))
        state.has_target=bool(packet['input']['has_target'])
        state.acc_enabled=bool(packet['input']['acc_enabled'])
        self.clock.publish(Clock(clock=state.stamp))
        self.publish_visuals(packet,state.stamp)
        with self.condition:
            self.pending=(state.session,state.sequence)
            self.reply=None
            self.publisher.publish(state)
            arrived=self.condition.wait_for(lambda:self.reply is not None,timeout=.8)
            self.pending=None
            if not arrived:
                raise TimeoutError('ROS control response exceeded 0.8 s')
            if self.reply.status:
                raise ValueError('Control node rejected frame: '+str(self.reply.status))
            return {key:getattr(self.reply,key) for key in OUTPUTS}

    def serve(self):
        while rclpy.ok():
            try:
                connection,_=self.server.accept()
            except socket.timeout:
                continue
            with connection,connection.makefile('rwb') as stream:
                while rclpy.ok():
                    line=stream.readline(1048576)
                    if not line:
                        break
                    try:
                        packet=json.loads(line)
                        response=dict(session=packet['session'],sequence=packet['sequence'],
                                      output=self.request(packet))
                    except (ValueError,KeyError,TypeError,TimeoutError) as exc:
                        response={'error':str(exc)}
                    try:
                        stream.write((json.dumps(response)+'\n').encode())
                        stream.flush()
                    except OSError:
                        break


def main():
    rclpy.init()
    node=Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.server.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
