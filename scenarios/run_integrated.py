"""Run the frozen integrated experiment and generate its report with one command."""
import argparse
import subprocess
import sys
from pathlib import Path


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=root/'output/integrated-project')
    output=parser.parse_args().output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    for label,lateral,mode in [('legacy','legacy','both'),('stanley','stanley','both'),
                               ('repeat','stanley','both'),('lqr','lqr','acc')]:
        subprocess.run([sys.executable,str(root/'scenarios/run_longitudinal.py'),
                        '--lateral',lateral,'--mode',mode,'--output',str(output/label)],check=True)
    subprocess.run([sys.executable,str(root/'scenarios/run_cosim_aeb.py'),
                    '--output',str(output/'aeb')],check=True)
    subprocess.run([sys.executable,str(root/'scenarios/record_integrated.py'),
                    '--output',str(output/'video')],check=True)
    subprocess.run([sys.executable,str(root/'scenarios/report_integrated.py'),
                    '--input',str(output)],check=True)


if __name__=='__main__':
    main()
