import glob
import os
import sys
#import smrclib
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
#import open3d as o3d

very_raw_data=[]
raw_data = b''
data = []
j = True
count=0

##def bytestoFloat(m):
##    man = m[1:4]
##    arr = np.unpackbits(man.astype(np.uint8)) * np.arange(24)
##    arr = 1 / np.power(np.repeat(2, 24),arr)
##    arr = arr - (arr == 1)
##    e = (((m[0] & 127) << 1) | ((m[1] >> 7) & 1)) - 127
##    ret = (1 + arr.sum()) * (2 ** e) * ((-1) ** ((m[0] >> 7) & 1))
##    return ret

##a = np.array([0b01000011, 0b00100010, 0b11100101, 0b00000000])
##b = bytestoFloat(a)
##print(b)

##def process_lidar(raw):
##    global data, j, raw_data
##    very_raw_data.append(raw)
##    #lidar_data = np.array(raw.raw_data, dtype=np.float32)
##    raw_data += bytes(raw.raw_data)
##    #data.append(lidar_data)      
##
actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter('model3')[0]
    print(bp)

    spawn_point = carla.Transform(carla.Location(20,4,3),carla.Rotation(0,0,0))
    spawn_point2 = carla.Transform(carla.Location(100,4,3),carla.Rotation(0,90,0))

    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle2 = world.spawn_actor(bp, spawn_point2)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    vehicle2.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    #vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

    actor_list.append(vehicle)
    actor_list.append(vehicle2)
    
    # get the blueprint for this sensor
    blueprint = blueprint_library.find('sensor.lidar.ray_cast')

    blueprint.set_attribute('points_per_second', '70000')
    #blueprint.set_attribute('channels', '16')
    blueprint.set_attribute('range', '10000')
    #blueprint.set_attribute('upper_fov', '10')
    #blueprint.set_attribute('lower_fov', '-10')

    blueprint.set_attribute('rotation_frequency', '10')
    
    # Adjust sensor relative to vehicle
    spawn_point = carla.Transform(carla.Location(x=0.0, z=0.0))

    # spawn the sensor and attach to vehicle.
    sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)

    # add sensor to list of actors
    actor_list.append(sensor)
    points = np.frombuffer(sensor.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))
    lidar_data = np.array(points[:, :2])
    print(lidar_data)
    

    # do something with this sensor
    #lidar = carla.LidarMeasurement
    #sensor.listen(lambda data: process_lidar(data))
    #sensor.listen(lambda LidarMeasurement: LidarMeasurement.save_to_disk('_out/%06d.ply' % LidarMeasurement.frame_number))
    
    time.sleep(12)

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
##
##with open('log.txt', 'a') as f:
##    for i in raw_data:
##        f.write(str(i))
##        f.write('\t')


##raw_data = list(raw_data)
##raw_data = np.array(raw_data)
##raw_data = raw_data.reshape((-1,4))
##q = []
##for i in range(raw_data.shape[0]):
##    q.append(bytestoFloat(raw_data[i]))
##
##q = np.array(q)
##q = q.reshape((-1,3))
##vis = smrclib.Visualizer('kwy', 600, 600, q[0], point_size=4)
##for i in range(q.shape[0]):
##    vis.update(q[i],colors=None)
##    print(i)
