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

import numpy as np
#import cv2
#import open3d as o3d

SHOW_PREVIEW = False
SECONDS_PER_EPISODE = 12

class CarEnv:
    SHOW_CAM = SHOW_PREVIEW
    #BRAKE_AMT = 1.0

    actor_list = []

    front_camera = None
    collision_hist = []

    pt_cloud = []
    pt_cloud_filtered = []
    
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)

        # Once we have a client we can retrieve the world that is currently
        # running.
        self.world = self.client.get_world()

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = self.world.get_blueprint_library()

        # Now let's filter all the blueprints of type 'vehicle' and choose one
        # at random.
        #print(blueprint_library.filter('vehicle'))
        self.model_3 = blueprint_library.filter('model3')[0]
        self.model_3_2 = blueprint_library.filter('model3')[0]

    def reset(self):
        self.collision_hist = []
        self.actor_list = []
        self.pt_cloud = []
        self.pt_cloud_filtered = []

        transform = carla.Transform(carla.Location(20,7.2,3),carla.Rotation(0,0,0))
        transform2 = carla.Transform(carla.Location(50,7.2,3),carla.Rotation(0,90,0))

        self.vehicle = self.world.spawn_actor(self.model_3, transform)
        self.vehicle2 = self.world.spawn_actor(self.model_3_2, transform2)
        self.actor_list.append(self.vehicle)
        self.actor_list.append(self.vehicle2)

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

        self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0))
        self.vehicle2.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

        time.sleep(4) # sleep to get things started and to not detect a collision when the car spawns/falls from sky.

        transform2 = carla.Transform(carla.Location(x=2.5, z=0.7))
        colsensor = self.world.get_blueprint_library().find('sensor.other.collision')
        self.colsensor = self.world.spawn_actor(colsensor, transform2, attach_to=self.vehicle)
        self.actor_list.append(self.colsensor)
        self.colsensor.listen(lambda event: self.collision_data(event))

        while self.distance_to_obstacle is None:
            time.sleep(0.01)

        self.episode_start = time.time()

        self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0))
        self.vehicle2.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        
        return int(self.distance_to_obstacle//1)

    def collision_data(self, event):
        self.collision_hist.append(event)

    def process_lidar(self, raw):
        points = np.frombuffer(raw.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))*np.array([1,-1,-1])
        lidar_data = points.astype(np.int32)
        self.pt_cloud.append(lidar_data)
        
        pt = points[np.logical_and(points[:,0] > -4, points[:,0] < 4)]
        points_filter = pt[np.logical_and(pt[:,1] > 0, pt[:,1] < 12)]
        self.pt_cloud_filtered.append(points_filter)

        if len(points_filter) == 0:
            self.distance_to_obstacle = 13
        else:
            self.distance_to_obstacle = min(points_filter[:,1])* math.sin(80*2*math.pi/360)

    def step(self, action):

        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer=0.0))
        if action == 1:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer = 0.0))
            time.sleep(2)
        v = self.vehicle.get_velocity()
        kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        print('kmh = ',kmh)
        print('0-no crash,1-crash = ',len(self.collision_hist))

        #if car crash
        #if len(self.collision_hist) != 0:
        #    done = True
        #    reward = -100

        
        if kmh == 0 or len(self.collision_hist) != 0 :
            done = True
            if  0<=self.distance_to_obstacle <4 and len(self.collision_hist) != 0 :
                reward = -100
            if  self.distance_to_obstacle <2:
                reward = -100
            if  2<= self.distance_to_obstacle <3:
                reward = -0
            if  3<= self.distance_to_obstacle <4:
                reward = 100
            if  4<= self.distance_to_obstacle <5:
                reward = 200
            if  5<= self.distance_to_obstacle <6:
                reward = 170
            if  6<= self.distance_to_obstacle <7:
                reward = 140
            if  7<= self.distance_to_obstacle <8:
                reward =110
            if  8<= self.distance_to_obstacle <9:
                reward =80
            if  9<= self.distance_to_obstacle <10:
                reward =50
            if  10<= self.distance_to_obstacle <11:
                reward =20
            if  11<= self.distance_to_obstacle :
                reward =-10
        else:
            done = False
            reward = 0

        if self.episode_start + SECONDS_PER_EPISODE < time.time():
            done = True

        return int(self.distance_to_obstacle//1), reward, done, None




#'------------------------------------------------'

    
#env = gym.make("FrozenLake-v0")
env = CarEnv()

action_space_size = 2
state_space_size = 100

q_table = np.zeros((state_space_size, action_space_size))

num_episodes = 100
max_steps_per_episode = 100
learning_rate = 0.1
discount_rate = 0.99

exploration_rate = 1
max_exploration_rate = 1
min_exploration_rate = 0.01
exploration_decay_rate = 0.001

count = 0
rewards_all_episodes = []
# Q-learning algorithm
for episode in range(num_episodes):
    # initialize new episode params
    #for step in range(max_steps_per_episode): 
        # Exploration-exploitation trade-off
        # Take new action
        # Update Q-table
        # Set new state
        # Add new reward        

    # Exploration rate decay   
    # Add current episode reward to total rewards list
    
    state = env.reset()
    print('env.reset() = ',state)
    done = False
    rewards_current_episode = 0

    for step in range(max_steps_per_episode):
        # Exploration-exploitation trade-off
        exploration_rate_threshold = random.uniform(0, 1)
        if exploration_rate_threshold > exploration_rate:
            action = np.argmax(q_table[state,:]) 
        else:
            action = np.random.randint(q_table.shape[1], size=1)[0]

        new_state, reward, done, info = env.step(action)
        
        print('new_state = ', new_state)
        print('state = ', state)
        # Update Q-table for Q(s,a)
        q_table[state, action] = q_table[state, action] * (1 - learning_rate) + \
            learning_rate * (reward + discount_rate * np.max(q_table[new_state, :]))
        print(q_table)
        state = new_state
        rewards_current_episode += reward
        print(done) 
            
        if done == True:
            print('destroying actors')
            for actor in env.actor_list:
                actor.destroy()
            count+=1
            print('episode number = ',count)
            print('distance to obstacle = ',state)
            #print('done.')
            print(done)
            break
        
    # Exploration rate decay
    exploration_rate = min_exploration_rate + \
          (max_exploration_rate - min_exploration_rate) * np.exp(-exploration_decay_rate*episode)

    rewards_all_episodes.append(rewards_current_episode)

 # Calculate and print the average reward per thousand episodes
rewards_per_thosand_episodes = np.split(np.array(rewards_all_episodes),num_episodes/1000)
count = 1000

print("********Average reward per thousand episodes********\n")
for r in rewards_per_thosand_episodes:
     print(count, ": ", str(sum(r/1000)))
     count += 1000

# Print updated Q-table
print("\n\n********Q-table********\n")
print(q_table)
