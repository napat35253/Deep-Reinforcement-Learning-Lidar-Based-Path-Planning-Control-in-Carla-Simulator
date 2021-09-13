import numpy as np
import random
import time
from IPython.display import clear_output

import glob
import os
import sys
#import smrclib
import math

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

#import cv2
#import open3d as o3d

SECONDS_PER_EPISODE = 12

def xxx():
    for actor in env.actor_list:
        actor.destroy()
        
class CarEnv:
    #BRAKE_AMT = 1.0

    actor_list = []
    collision_hist = []

    pt_cloud = []
    pt_cloud_filtered = []
    
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)

        self.world = self.client.get_world()

        blueprint_library = self.world.get_blueprint_library()

        self.model_3 = blueprint_library.filter('model3')[0]
        self.truck_2 = blueprint_library.filter('carlamotors')[0]
        
    def reset(self):
        self.collision_hist = []
        self.actor_list = []
        self.pt_cloud = []
        self.pt_cloud_filtered = []

        transform = carla.Transform(carla.Location(-120,120,3),carla.Rotation(0,-90,0))

        self.vehicle = self.world.spawn_actor(self.model_3, transform)
        
        self.actor_list.append(self.vehicle)

        self.lidar_sensor = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        self.lidar_sensor.set_attribute('points_per_second', '100000')
        self.lidar_sensor.set_attribute('channels', '32')
        self.lidar_sensor.set_attribute('range', '10000')
        self.lidar_sensor.set_attribute('upper_fov', '10')
        self.lidar_sensor.set_attribute('lower_fov', '-10')
        self.lidar_sensor.set_attribute('rotation_frequency', '60')

        transform = carla.Transform(carla.Location(x=0, z=1.9))
        self.sensor = self.world.spawn_actor(self.lidar_sensor, transform, attach_to=self.vehicle)

        self.actor_list.append(self.sensor)
        self.sensor.listen(lambda data: self.process_lidar(data))

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.7, brake=0.0))
        self.episode_start = time.time()


        time.sleep(4) # sleep to get things started and to not detect a collision when the car spawns/falls from sky.
        
        transform2 = carla.Transform(carla.Location(x=2.5, z=0.7))
        colsensor = self.world.get_blueprint_library().find('sensor.other.collision')
        self.colsensor = self.world.spawn_actor(colsensor, transform2, attach_to=self.vehicle)
        self.actor_list.append(self.colsensor)
        self.colsensor.listen(lambda event: self.collision_data(event))

        while self.distance_to_obstacle is None:
            time.sleep(0.01)

        self.episode_start = time.time()

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.7, brake=0.0))
        
        return int(self.distance_to_obstacle//1)

    def collision_data(self, event):
        self.collision_hist.append(event)

    def process_lidar(self, raw):
        points = np.frombuffer(raw.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))*np.array([1,-1,-1])
        lidar_data = points.astype(np.int32)
        self.pt_cloud.append(lidar_data)

        #screen points specifically -4<y<4 and 0<x<12
        pt = points[np.logical_and(points[:,0] > -3, points[:,0] < 3)]
        points_filter = pt[np.logical_and(pt[:,1] > 0, pt[:,1] < 50)]
        points_filter = points_filter[np.logical_and(points_filter[:,1] > 0, points_filter[:,1] < 50)]
        self.pt_cloud_filtered.append(points_filter)

        if len(points_filter) == 0:
            pass
        else:
            self.distance_to_obstacle = min(points_filter[:,1])-2.247148275375366

    def step(self, action):

        v = self.vehicle.get_velocity()
        kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        
        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer = 0.0))
        elif action == 1:
            while kmh != 0:
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer = 0.0))
                print(kmh)
                v = self.vehicle.get_velocity()
                kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
                print("distance_to_obstacle = ",self.distance_to_obstacle)
                
        # ถ้ารถชน หรือ รถหยุดแล้ว
        if kmh == 0 or len(self.collision_hist) != 0 :
            done = True
            print(self.distance_to_obstacle)
            if  0<= self.distance_to_obstacle <1 and len(self.collision_hist) != 0 :
                reward = -1
            elif  0<= self.distance_to_obstacle <1:
                reward = 0.1
            elif  1<= self.distance_to_obstacle <2:
                reward = 0.5
            elif  2<= self.distance_to_obstacle <3:
                reward = 0.45
            elif  3<= self.distance_to_obstacle <4:
                reward = 0.32
            elif  4<= self.distance_to_obstacle <5:
                reward = 0.20
            elif  5<= self.distance_to_obstacle <6:
                reward = -0.15
            elif  6<= self.distance_to_obstacle <7:
                reward = -0.8
            elif  7<= self.distance_to_obstacle <8:
                reward = -0.7
            elif  8<= self.distance_to_obstacle :
                reward = -1.5
        else:
            done = False
            reward = 0.5
        
        if self.episode_start + SECONDS_PER_EPISODE < time.time():
            done = True
            reward = -1

        return int(self.distance_to_obstacle//1), reward, done, None




#'------------------------main-----------------------'

env = CarEnv()

# action มี 2 อันคือ เบรคและเร่งเต็มที่, state มี 13 อัน คือ ชน, 0-1, 1-2, 2-3, 3-4, 4-5, 5-6, 6-7, 7-8, 8-9, 9-10, 10-11, >11
action_space_size = 2
state_space_size = 10
q_table = np.zeros((state_space_size, action_space_size))

num_episodes = 300
max_steps_per_episode = 100
learning_rate = 0.2
discount_rate = 0.99

exploration_rate = 1
max_exploration_rate = 1
min_exploration_rate = 0.01
exploration_decay_rate = 0.01

count = 0
rewards_all_episodes = []
action_all_episodes = []

for episode in range(num_episodes):
    state = env.reset()
      
    print('LOOP START')
    print(time.time()-env.episode_start)
    done = False
    rewards_current_episode = []
    action_current_episode = []
    count_step=0
    
    for step in range(max_steps_per_episode):
        exploration_rate_threshold = random.uniform(0, 1)
        if exploration_rate_threshold > exploration_rate:
            action = np.argmax(q_table[state,:]) 
        else:
            action = np.random.randint(0, high=2, size=None, dtype='int')
        action_current_episode.append(action)
        new_state, reward, done, info = env.step(action)
        print(new_state,reward,done,info)
        q_table[state, action] = q_table[state, action] * (1 - learning_rate) + \
            learning_rate * (reward + discount_rate * np.max(q_table[new_state, :]))
        print('UPDATED')
        count_step+=1
        state = new_state
        rewards_current_episode.append(reward)
                    
        if done == True:
            print(q_table)
            print('destroying actors')
            for actor in env.actor_list:
                actor.destroy()
            count+=1
            print('episode number = ',count)
            print('distance to obstacle = ',state)
            print(done)
            print('rewards_current_episode = ',rewards_current_episode)
            print('action_current_episode =', action_current_episode)
            print('count_step = ', count_step)
            break
        else :
            print(q_table)
            print('[INFO]')
            print('episode number = ',count)
            
        
    # Exploration rate decay
    exploration_rate = min_exploration_rate + \
          (max_exploration_rate - min_exploration_rate) * np.exp(-exploration_decay_rate*episode)
    print(exploration_rate)

    rewards_all_episodes.append(rewards_current_episode)
    action_all_episodes.append(action_current_episode)

 # Calculate and print the average reward per thousand episodes
rewards_per_thosand_episodes = np.split(np.array(rewards_all_episodes),num_episodes/100)
count = 10

print("********Average reward per thousand episodes********\n")
for r in rewards_per_thosand_episodes:
     print(count, ": ", str(sum(r/100)))
     count += 10

# Print updated Q-table
print("\n\n********Q-table********\n")
print(q_table)
