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

very_raw_data=[]
raw_data = b''
data = []
j = True
count=1

#vis = smrclib.Visualizer('kwy', 600, 600, np.array([1,2,3]), point_size=3)
def process_lidar(raw):
    global data, j, raw_data, count
    very_raw_data.append(raw)
    points = np.frombuffer(raw.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))
    #print(points)
    data.append(points)
    count+=1
actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    print('start')

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter('model3')[0]
    print(bp)

    spawn_point = carla.Transform(carla.Location(20,4,3),carla.Rotation(0,0,0))
    spawn_point2 = carla.Transform(carla.Location(100,4,3),carla.Rotation(0,90,0))

    vehicle = world.spawn_actor(bp, spawn_point)
    #vehicle2 = world.spawn_actor(bp, spawn_point2)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0))
    #vehicle2.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    #vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

    actor_list.append(vehicle)
    #actor_list.append(vehicle2)
    
    # get the blueprint for this sensor
    blueprint = blueprint_library.find('sensor.lidar.ray_cast')

    blueprint.set_attribute('points_per_second', '100000')
    #blueprint.set_attribute('channels', '')
    blueprint.set_attribute('range', '10000')
    #blueprint.set_attribute('upper_fov', '80')
    #blueprint.set_attribute('lower_fov', '-20')
    blueprint.set_attribute('rotation_frequency', '60')
    
    spawn_point = carla.Transform(carla.Location(x=0.0, z=0.0))

    sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)
    actor_list.append(sensor)

    lidar = carla.LidarMeasurement
    sensor.listen(lambda data: process_lidar(data))

    time.sleep(5)
    
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')

vis = smrclib.Visualizer('kwy', 600, 600, data[0], point_size=3)
for i in range(len(data)-1):
    vis.update(data[i],colors=None)
    print(i)
    time.sleep(20/count)
