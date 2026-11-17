"""Run curved-road comparisons, repeats, straight regression and a camera demo."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    repo=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=repo/'output/curved-release')
    root=parser.parse_args().output.resolve()
    root.mkdir(parents=True,exist_ok=False)
    base=[sys.executable,str(repo/'scenarios/run_longitudinal.py')]
    config=str(repo/'configs/curved.json')
    commands=[
        ['--config',config,'--mode','acc','--target-policy','corridor','--output',str(root/'corridor')],
        ['--config',config,'--mode','acc','--output',str(root/'route')],
        ['--config',config,'--mode','acc','--output',str(root/'repeat')],
    ]
    for label,policy in [('aeb-corridor','corridor'),('aeb-route','route'),('aeb-repeat','route')]:
        commands.append(['--config',config,'--case','holdout_extreme_brake','--mode','baseline',
                         '--target-policy',policy,'--output',str(root/label)])
    commands.append(['--output',str(root/'straight')])
    for command in commands:
        subprocess.run(base+command,check=True)
    subprocess.run([sys.executable,str(repo/'scenarios/record_integrated.py'),'--config',config,
                    '--output',str(root/'video')],check=True)
    subprocess.run([sys.executable,str(repo/'scenarios/report_curved.py'),'--input',str(root)],check=True)


if __name__=='__main__':
    main()
