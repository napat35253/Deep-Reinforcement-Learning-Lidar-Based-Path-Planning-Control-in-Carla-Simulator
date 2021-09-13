import glob
import os
import sys
import smrclib
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import random
import time
import numpy as np
import cv2
import open3d as o3d

pt_cloud = []

def process_lidar(raw):
    global j, raw_data, count
    points = np.frombuffer(raw.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))*np.array([1,-1,-1])
    pt_cloud.append(points)
        

actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    print('start')

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter('model3')[0]
    print(bp)

    spawn_point = carla.Transform(carla.Location(245,0,3),carla.Rotation(0,-90,0))
    spawn_point2 = carla.Transform(carla.Location(245,-80,3),carla.Rotation(0,0,0))

    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle2 = world.spawn_actor(bp, spawn_point2)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0))
    vehicle2.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    #vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

    actor_list.append(vehicle)
    actor_list.append(vehicle2)
    
    # get the blueprint for this sensor
    blueprint = blueprint_library.find('sensor.lidar.ray_cast')

    blueprint.set_attribute('points_per_second', '100000')
    blueprint.set_attribute('channels', '32')
    blueprint.set_attribute('range', '10000')
    blueprint.set_attribute('upper_fov', '60')
    blueprint.set_attribute('lower_fov', '-10')
    blueprint.set_attribute('rotation_frequency', '60')
    
    spawn_point = carla.Transform(carla.Location(x=0.0, z=0.0))

    sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
    actor_list.append(sensor)

    lidar = carla.LidarMeasurement
    sensor.listen(lambda data: process_lidar(data))

    time.sleep(10)
    
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')

vis = smrclib.Visualizer('kwy', 600, 600, pt_cloud[0], point_size=3)
for i in range(len(pt_cloud)-1):
    print(i)
    vis.update(pt_cloud[i])
    time.sleep(20/186)
