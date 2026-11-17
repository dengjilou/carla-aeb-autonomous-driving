"""CARLA adapter for the validated controllers and a fixed map-lane reference."""
import math
import numpy as np
from path_geometry import project
from stanley_tracking import steering
from lqr_tracking import LQR


class RouteTracking:
    def __init__(self, road, vehicle, mode, cruise):
        self.mode, self.previous = mode, 0.
        wp = road.get_waypoint(vehicle.get_location())
        history = wp.previous(8)
        wp = history[0] if history else wp
        points = []
        for _ in range(1001):
            points.append((wp.transform.location.x,wp.transform.location.y))
            options = wp.next(.5)
            if not options:
                break
            wp = sorted(options,key=lambda p:(p.road_id,p.lane_id,p.s))[0]
        if len(points) < 600:
            raise RuntimeError("Reference route must cover at least 300 m")
        self.points = np.array(points)
        self.vectors = np.diff(self.points,axis=0)
        self.lengths2 = np.sum(self.vectors**2,axis=1)
        self.stations = np.r_[0,np.cumsum(np.sqrt(self.lengths2))]
        inverse = np.array(vehicle.get_transform().get_inverse_matrix())
        wheels = vehicle.get_physics_control().wheels
        local = [inverse@np.array([w.position.x/100,w.position.y/100,w.position.z/100,1]) for w in wheels]
        self.front, self.rear = local[0][0], local[2][0]
        self.max_angle = math.radians(wheels[0].max_steer_angle)
        self.lqr = LQR(max(cruise,1),self.front-self.rear,position_weight=.2) if mode=='lqr' else None

    def locate(self, location):
        index,fraction,error = project(np.array([location.x,location.y]),self.points[:-1],self.vectors,self.lengths2)
        tangent = self.vectors[index]/math.sqrt(self.lengths2[index])
        return dict(error=error,station=float(self.stations[index]+fraction*math.sqrt(self.lengths2[index])),
                    tangent=tangent)

    def state(self, vehicle):
        return self.locate(vehicle.get_location())

    def nearest_target(self, ego, world, detection_range=50):
        origin = self.state(ego)
        ego_v = ego.get_velocity()
        ego_speed = ego_v.x*origin["tangent"][0]+ego_v.y*origin["tangent"][1]
        candidates=[]
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.id==ego.id or actor.get_location().distance(ego.get_location())>detection_range:
                continue
            state=self.state(actor)
            distance=state["station"]-origin["station"]
            if not 0 < distance < detection_range:
                continue
            forward=actor.get_transform().get_forward_vector()
            aligned=forward.x*state["tangent"][0]+forward.y*state["tangent"][1]
            width=ego.bounding_box.extent.y+actor.bounding_box.extent.y+.3
            if aligned<.5 or abs(state["error"])>width:
                continue
            velocity=actor.get_velocity()
            closing=ego_speed-(velocity.x*state["tangent"][0]+velocity.y*state["tangent"][1])
            candidates.append((distance,actor.id,float(closing)))
        return min(candidates) if candidates else (None,None,0.)

    def update(self, vehicle):
        import carla
        pose = vehicle.get_transform()
        speed = vehicle.get_velocity().length()
        heading = math.radians(pose.rotation.yaw)
        center = np.array([pose.location.x,pose.location.y])
        idx,_,error = project(center,self.points[:-1],self.vectors,self.lengths2)
        angle = heading-math.atan2(self.vectors[idx,1],self.vectors[idx,0])
        body_heading = math.atan2(math.sin(angle),math.cos(angle))
        axle = pose.transform(carla.Location(x=self.rear if self.mode=='lqr' else self.front))
        index,_,axle_error = project(np.array([axle.x,axle.y]),self.points[:-1],self.vectors,self.lengths2)
        tangent = math.atan2(self.vectors[index,1],self.vectors[index,0])
        if self.mode=='lqr':
            lo,hi = max(0,index-2),min(len(self.vectors)-1,index+2)
            delta = math.atan2(self.vectors[hi,1],self.vectors[hi,0])-math.atan2(self.vectors[lo,1],self.vectors[lo,0])
            curvature = math.atan2(math.sin(delta),math.cos(delta))/np.sqrt(self.lengths2[lo:hi]).sum()
            command = self.lqr.steering(axle_error,heading-tangent,curvature,self.max_angle)
        else:
            command = steering(heading,tangent,axle_error,speed,self.max_angle,2.)
        self.previous = .4*max(-.7,min(.7,command))+.6*self.previous
        return self.previous,error,body_heading
