#include <Python.h>
#include "structmember.h"
#include "pid_cpp.h"

typedef struct {
    PyObject_HEAD
    PID* pid;
} PidCppObject;

static void PidObject_dealloc(PidCppObject* self) {
    if (self->pid) {
        delete self->pid;
    }
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* PidObject_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    PidCppObject* self;
    self = (PidCppObject*)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->pid = nullptr;
    }
    return (PyObject*)self;
}

static int PidObject_init(PidCppObject* self, PyObject* args, PyObject* kwds) {
    double kp, ki, kd, setpoint;
    
    if (!PyArg_ParseTuple(args, "dddd", &kp, &ki, &kd, &setpoint)) {
        return -1;
    }
    
    self->pid = new PID(kp, ki, kd, setpoint);
    return 0;
}

static PyObject* PidObject_compute(PidCppObject* self, PyObject* args) {
    double actual, dt;
    
    if (!PyArg_ParseTuple(args, "dd", &actual, &dt)) {
        return NULL;
    }
    
    double result = self->pid->compute(actual, dt);
    return PyFloat_FromDouble(result);
}

static PyObject* PidObject_set_limits(PidCppObject* self, PyObject* args) {
    double min_val, max_val;
    
    if (!PyArg_ParseTuple(args, "dd", &min_val, &max_val)) {
        return NULL;
    }
    
    self->pid->set_output_limits(min_val, max_val);
    Py_RETURN_NONE;
}

static PyObject* PidObject_reset(PidCppObject* self) {
    self->pid->reset();
    Py_RETURN_NONE;
}

static PyObject* PidObject_set_setpoint(PidCppObject* self, PyObject* args) {
    double new_setpoint;
    
    if (!PyArg_ParseTuple(args, "d", &new_setpoint)) {
        return NULL;
    }
    
    self->pid->set_setpoint(new_setpoint);
    Py_RETURN_NONE;
}

static PyObject* PidObject_get_error(PidCppObject* self, void* closure) {
    return PyFloat_FromDouble(self->pid->get_error());
}

static PyObject* PidObject_get_kp(PidCppObject* self, void* closure) {
    return PyFloat_FromDouble(self->pid->get_kp());
}

static PyObject* PidObject_get_ki(PidCppObject* self, void* closure) {
    return PyFloat_FromDouble(self->pid->get_ki());
}

static PyObject* PidObject_get_kd(PidCppObject* self, void* closure) {
    return PyFloat_FromDouble(self->pid->get_kd());
}

static PyObject* PidObject_get_setpoint(PidCppObject* self, void* closure) {
    return PyFloat_FromDouble(self->pid->get_setpoint());
}

static PyMethodDef PidObject_methods[] = {
    {"compute", (PyCFunction)PidObject_compute, METH_VARARGS, "Compute PID output"},
    {"set_output_limits", (PyCFunction)PidObject_set_limits, METH_VARARGS, "Set output limits"},
    {"reset", (PyCFunction)PidObject_reset, METH_NOARGS, "Reset PID state"},
    {"set_setpoint", (PyCFunction)PidObject_set_setpoint, METH_VARARGS, "Set new target value"},
    {NULL}
};

static PyGetSetDef PidObject_getseters[] = {
    {"error", (getter)PidObject_get_error, NULL, "Current error", NULL},
    {"kp", (getter)PidObject_get_kp, NULL, "Proportional gain", NULL},
    {"ki", (getter)PidObject_get_ki, NULL, "Integral gain", NULL},
    {"kd", (getter)PidObject_get_kd, NULL, "Derivative gain", NULL},
    {"setpoint", (getter)PidObject_get_setpoint, NULL, "Target value", NULL},
    {NULL}
};

static PyTypeObject PidObjectType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "pid_cpp.PID",                   /* tp_name */
    sizeof(PidCppObject),            /* tp_basicsize */
    0,                               /* tp_itemsize */
    (destructor)PidObject_dealloc,   /* tp_dealloc */
    0,                               /* tp_print */
    0,                               /* tp_getattr */
    0,                               /* tp_setattr */
    0,                               /* tp_reserved */
    0,                               /* tp_repr */
    0,                               /* tp_as_number */
    0,                               /* tp_as_sequence */
    0,                               /* tp_as_mapping */
    0,                               /* tp_hash */
    0,                               /* tp_call */
    0,                               /* tp_str */
    0,                               /* tp_getattro */
    0,                               /* tp_setattro */
    0,                               /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,              /* tp_flags */
    "C++ PID Controller",            /* tp_doc */
    0,                               /* tp_traverse */
    0,                               /* tp_clear */
    0,                               /* tp_richcompare */
    0,                               /* tp_weaklistoffset */
    0,                               /* tp_iter */
    0,                               /* tp_iternext */
    PidObject_methods,               /* tp_methods */
    0,                               /* tp_members */
    PidObject_getseters,             /* tp_getset */
    0,                               /* tp_base */
    0,                               /* tp_dict */
    0,                               /* tp_descr_get */
    0,                               /* tp_descr_set */
    0,                               /* tp_dictoffset */
    (initproc)PidObject_init,        /* tp_init */
    0,                               /* tp_alloc */
    PidObject_new,                   /* tp_new */
};

static PyModuleDef pid_cpp_module = {
    PyModuleDef_HEAD_INIT,
    "pid_cpp",
    "C++ PID controller for CARLA longitudinal control",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit_pid_cpp(void) {
    PyObject* m;
    
    if (PyType_Ready(&PidObjectType) < 0)
        return NULL;
    
    m = PyModule_Create(&pid_cpp_module);
    if (m == NULL)
        return NULL;
    
    Py_INCREF(&PidObjectType);
    if (PyModule_AddObject(m, "PID", (PyObject*)&PidObjectType) < 0) {
        Py_DECREF(&PidObjectType);
        Py_DECREF(m);
        return NULL;
    }
    
    return m;
}
