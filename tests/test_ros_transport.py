"""Offline protocol rejection tests with a local one-response peer."""
import json
import socket
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controllers'))
from control_ros import RosController
from test_control_native import sample


class TransportTests(unittest.TestCase):
    def check_response(self,change,expected):
        server=socket.socket()
        server.bind(('127.0.0.1',0))
        server.listen(1)
        def peer():
            with server:
                connection,_=server.accept()
                with connection,connection.makefile('rwb') as stream:
                    request=json.loads(stream.readline())
                    response=dict(session=request['session'],sequence=request['sequence'],
                                  output=dict(reference=10.,throttle=0.,brake=0.,steer=0.,latched_brake=0.))
                    change(response)
                    stream.write((json.dumps(response)+'\n').encode())
                    stream.flush()
        worker=threading.Thread(target=peer)
        worker.start()
        controller=RosController(port=server.getsockname()[1])
        try:
            with self.assertRaisesRegex(RuntimeError,expected):
                controller.step(sample())
            self.assertEqual(controller.sequence,0)
        finally:
            controller.close()
            controller.close()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_wrong_frame(self):
        self.check_response(lambda r:r.update(sequence=r['sequence']+1),'mismatched')

    def test_nonfinite_command(self):
        self.check_response(lambda r:r['output'].update(steer=float('nan')),'Nonfinite')

    def test_incomplete_command(self):
        self.check_response(lambda r:r['output'].pop('brake'),'Incomplete')

    def test_out_of_range_command(self):
        self.check_response(lambda r:r['output'].update(throttle=2.),'Out of range')

    def test_explicit_timeout(self):
        self.check_response(lambda r:r.update(error='ROS control response exceeded 0.8 s'),'exceeded')


if __name__=='__main__':
    unittest.main()
