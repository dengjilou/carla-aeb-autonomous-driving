"""Windows-side request/response transport to the ROS 2 C++ control node."""
import json
import math
import socket
import time
from control_native import default_config, validate_input, OUTPUT_FIELDS


class RosController:
    def __init__(self,gains=None,host='127.0.0.1',port=8766):
        self.config=default_config(gains)
        self.session=time.time_ns()
        self.sequence=0
        self.socket=socket.create_connection((host,port),timeout=1.)
        self.socket.settimeout(1.)
        self.stream=self.socket.makefile('rwb')
        self.scene={}

    def set_scene(self,pose,points):
        self.scene={'pose':dict(x=pose.location.x,y=pose.location.y,z=pose.location.z,
                                yaw=pose.rotation.yaw*3.141592653589793/180)}
        if self.sequence==0:
            self.scene['path']=points.tolist()

    def step(self,values):
        validate_input(values)
        packet=dict(self.scene,session=self.session,sequence=self.sequence,input=values,config=self.config)
        self.stream.write((json.dumps(packet)+'\n').encode())
        self.stream.flush()
        line=self.stream.readline(1048576)
        if not line:
            raise ConnectionError('ROS bridge disconnected')
        response=json.loads(line)
        if 'error' in response:
            raise RuntimeError(response['error'])
        if (response['session'],response['sequence'])!=(self.session,self.sequence):
            raise RuntimeError('ROS response has mismatched frame identity')
        result=response['output']
        if not all(key in result for key in OUTPUT_FIELDS):
            raise RuntimeError('Incomplete ROS command')
        if not all(math.isfinite(result[key]) for key in OUTPUT_FIELDS):
            raise RuntimeError('Nonfinite ROS command')
        if not (result['reference']>=0 and 0<=result['throttle']<=1 and 0<=result['brake']<=1
                and -1<=result['steer']<=1 and 0<=result['latched_brake']<=1):
            raise RuntimeError('Out of range ROS command')
        self.sequence+=1
        return result

    def reset(self):
        self.session=time.time_ns()
        self.sequence=0

    def close(self):
        if getattr(self,'stream',None):
            try:
                self.stream.close()
            except OSError:
                pass
            self.stream=None
        if getattr(self,'socket',None):
            self.socket.close()
            self.socket=None

    def __del__(self):
        self.close()
