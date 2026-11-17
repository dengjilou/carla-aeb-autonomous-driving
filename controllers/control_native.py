"""Matching Python/C++ stateful control interfaces; no simulator dependency."""
import ctypes as ct
import math
import os
from pathlib import Path
from acc import ACC, SpeedPID, arbitrate
from stanley_tracking import steering

ROOT = Path(__file__).resolve().parents[1]
CONFIG_FIELDS = ('gap_gain','relative_gain','standstill_gap','headway',
                 'stanley_gain','steer_limit','steer_alpha')
INPUT_FIELDS = ('dt','speed','cruise','gap','closing','heading','path_heading',
                'cross_track','max_angle','aeb_request')
OUTPUT_FIELDS = ('reference','throttle','brake','steer','latched_brake')


class Config(ct.Structure):
    _fields_ = [(name,ct.c_double) for name in CONFIG_FIELDS]


class Input(ct.Structure):
    _fields_ = [(name,ct.c_double) for name in INPUT_FIELDS]+[
        ('has_target',ct.c_int32),('acc_enabled',ct.c_int32)]


class Output(ct.Structure):
    _fields_ = [(name,ct.c_double) for name in OUTPUT_FIELDS]


def default_config(gains=None):
    config=dict(gap_gain=.3,relative_gain=.4,standstill_gap=8.,headway=1.8,
                stanley_gain=2.,steer_limit=.7,steer_alpha=.4)
    config.update(gains or {})
    return config


def validate_input(values):
    required = [v for k,v in values.items() if k not in ('gap','closing')]
    if not all(math.isfinite(v) for v in required):
        raise ValueError('Finite control input required')
    if values['has_target'] and not all(math.isfinite(values[k]) for k in ('gap','closing')):
        raise ValueError('Finite target input required')
    if (values['dt']<=0 or values['speed']<0 or values['cruise']<0 or values['max_angle']<=0
            or not 0<=values['aeb_request']<=1 or values['has_target'] not in (0,1)
            or values['acc_enabled'] not in (0,1)):
        raise ValueError('Invalid control input')


class PythonController:
    def __init__(self,gains=None):
        self.config=default_config(gains)
        self.reset()

    def reset(self):
        c=self.config
        self.acc=ACC(c['gap_gain'],c['relative_gain'],c['standstill_gap'],c['headway'])
        self.pid=SpeedPID()
        self.previous=0.
        self.latched=0.

    def step(self,values):
        validate_input(values)
        i,c=values,self.config
        reference=self.acc.target_speed(i['speed'],i['cruise'],
                    i['gap'] if i['has_target'] else None,i['closing']) if i['acc_enabled'] else i['cruise']
        throttle,brake=self.pid.update(reference,i['speed'],i['dt'])
        raw=steering(i['heading'],i['path_heading'],i['cross_track'],i['speed'],
                     i['max_angle'],c['stanley_gain'])
        self.previous=c['steer_alpha']*max(-c['steer_limit'],min(c['steer_limit'],raw))+(1-c['steer_alpha'])*self.previous
        throttle,brake,self.latched=arbitrate(throttle,brake,i['aeb_request'],self.latched)
        return dict(reference=reference,throttle=throttle,brake=brake,
                    steer=self.previous,latched_brake=self.latched)

    def close(self):
        pass


class NativeController:
    def __init__(self,gains=None,library=None):
        name='control_native.dll' if os.name=='nt' else 'libcontrol_native.so'
        default=ROOT/'output'/('native-win/Release' if os.name=='nt' else 'native-linux')/name
        self.library=ct.CDLL(str(library or os.environ.get('CONTROL_NATIVE_LIBRARY',default)))
        self.library.control_create.argtypes=[ct.POINTER(Config)]
        self.library.control_create.restype=ct.c_void_p
        self.library.control_step.argtypes=[ct.c_void_p,ct.POINTER(Input),ct.POINTER(Output)]
        self.library.control_step.restype=ct.c_int
        self.library.control_reset.argtypes=[ct.c_void_p]
        self.library.control_reset.restype=None
        self.library.control_destroy.argtypes=[ct.c_void_p]
        self.library.control_destroy.restype=None
        self.handle=self.library.control_create(ct.byref(Config(**default_config(gains))))
        if not self.handle:
            raise ValueError('Invalid native configuration or allocation failure')

    def step(self,values):
        if not self.handle:
            raise RuntimeError('Native controller is closed')
        input_value=Input(**values)
        output=Output()
        if self.library.control_step(self.handle,ct.byref(input_value),ct.byref(output)):
            raise ValueError('Invalid control input')
        return {name:getattr(output,name) for name in OUTPUT_FIELDS}

    def reset(self):
        if not self.handle:
            raise RuntimeError('Native controller is closed')
        self.library.control_reset(self.handle)

    def close(self):
        if getattr(self,'handle',None):
            self.library.control_destroy(self.handle)
            self.handle=None

    def __del__(self):
        self.close()
