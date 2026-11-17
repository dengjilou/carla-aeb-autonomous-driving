"""Record all six integrated ACC/AEB scene families with actual camera frames."""
import json
import argparse
import queue
from pathlib import Path
import imageio_ffmpeg
import pygame
from run_longitudinal import ROOT, load_config, run_case
from longitudinal_metrics import evaluate


def main():
    import carla
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'output/integrated-video')
    output=parser.parse_args().output
    output.mkdir(parents=True,exist_ok=False)
    config=load_config(ROOT/'configs/longitudinal.json')
    selected=['follow_holdout','slowdown_holdout','hard_brake_holdout',
              'cutin_holdout','cutout_holdout','adjacent_holdout']
    pygame.init()
    canvas=pygame.Surface((640,480))
    font=pygame.font.Font(None,25)
    writer=imageio_ffmpeg.write_frames(str(output/'integrated_demo.mp4'),(640,480),
                                      fps=20,codec='libx264',pix_fmt_in='rgb24',bitrate='600k')
    writer.send(None)
    evidence={}
    try:
        for name in selected:
            case=next(c for c in config['cases'] if c['name']==name)
            cfg=dict(case,step_s=config['step_s'],seed=config['seed'],lateral_controller='stanley')
            incoming=queue.Queue()
            def attach(world,ego):
                bp=world.get_blueprint_library().find('sensor.camera.rgb')
                bp.set_attribute('image_size_x','640')
                bp.set_attribute('image_size_y','360')
                camera=world.spawn_actor(bp,carla.Transform(carla.Location(x=-9,z=5),
                                         carla.Rotation(pitch=-15)),attach_to=ego)
                camera.listen(lambda img:incoming.put((img.frame,bytes(img.raw_data))))
                return camera
            def on_sample(row):
                while True:
                    frame,data=incoming.get(timeout=15)
                    if frame==row['frame']:
                        break
                    if frame>row['frame']:
                        raise RuntimeError('Camera skipped the recorded control frame')
                canvas.fill((17,25,36))
                canvas.blit(pygame.image.frombuffer(data,(640,360),'BGRA'),(0,38))
                labels=[(9,'CARLA-SUMO | '+name),(408,'t={:.2f}s  speed={:.1f}km/h  state={}'.format(
                        row['time_s'],row['ego_speed_mps']*3.6,row['control_state'])),
                        (434,'gap={:.1f}m  path error={:+.3f}m  brake={:.0%}'.format(
                        row['gap_m'],row['path_error_m'],row['brake'])),
                        (458,'SUMO actors={} | Stanley + ACC/PID + AEB'.format(row['sumo_background']))]
                for y,text in labels:
                    canvas.blit(font.render(text,True,(235,240,245)),(10,y))
                writer.send(pygame.image.tostring(canvas,'RGB'))
                if name=='cutin_holdout' and row['time_s']==1.5:
                    pygame.image.save(canvas,str(output/'integrated_demo.png'))
            attach.on_sample=on_sample
            rows,contacts=run_case(cfg,'acc',config['gains'],camera=attach,trace_path=output/(name+'.csv'))
            evidence[name]=dict(config=cfg,metrics=evaluate(rows,contacts,cfg),collisions=contacts)
            print(name,evidence[name]['metrics']['verdict'],flush=True)
    finally:
        writer.close()
        pygame.quit()
    (output/'capture.json').write_text(json.dumps(evidence,indent=2))
    print(output/'integrated_demo.mp4')


if __name__=='__main__':
    main()
