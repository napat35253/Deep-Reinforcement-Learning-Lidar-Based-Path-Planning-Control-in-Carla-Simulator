from __future__ import print_function

import random
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
import glob
import os
import sys

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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
SECONDS_PER_EPISODE = 12
@profile
def xxx():
    env.world.wait_for_tick()
    for x in list(env.world.get_actors()):
        if x.type_id == 'vehicle.tesla.model3' or x.type_id == 'sensor.lidar.ray_cast' or x.type_id == 'sensor.other.collision':
            x.destroy()
            
def lidar_line(points,degree,width):
    angle = degree*(2*np.pi)/360
    points_l = points
    points_l = points_l[np.logical_and(points_l[:,2] > -1.75, points_l[:,2] < 1000)] #z
    points_l = points_l[np.logical_and(np.tan(angle)*points_l[:,0]+width*np.sqrt(1+np.tan(angle)**2)>=points_l[:,1], np.tan(angle)*points_l[:,0]-width*np.sqrt(1+np.tan(angle)**2)<=points_l[:,1])] #y
    if 180>degree >0:
        points_l = points_l[np.logical_and(points_l[:,1]>0, points_l[:,1]<1000)] #y>0
    if 180<degree<360:
        points_l = points_l[np.logical_and(points_l[:,1]<0, points_l[:,1] > -1000)] #x
    if degree == 0 or degree == 360:
        points_l = points_l[np.logical_and(points_l[:,0]>0,points_l[:,0] <1000 )] #x
    if degree == 180:
        points_l = points_l[np.logical_and(points_l[:,0] >-1000 , points_l[:,0]<0 )]
    return  points_l

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
        place=random.uniform(110,150)
        ##print('Location: ',str(place))
        #transform = carla.Transform(carla.Location(-120,place,3),carla.Rotation(0,-90,0))
        transform = carla.Transform(carla.Location(246,-36,3),carla.Rotation(0,-90,0))        
        self.flag = 0
        self.vehicle = self.world.spawn_actor(self.model_3, transform)
        self.flag = 1
        
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

        self.vehicle.apply_control(carla.VehicleControl(throttle=1, brake=0.0))
        self.episode_start = time.time()

        time.sleep(0.4) # sleep to get things started and to not detect a collision when the car spawns/falls from sky.
        
        transform2 = carla.Transform(carla.Location(x=2.5, z=0.7))
        colsensor = self.world.get_blueprint_library().find('sensor.other.collision')
        self.colsensor = self.world.spawn_actor(colsensor, transform2, attach_to=self.vehicle)
        self.actor_list.append(self.colsensor)
        self.colsensor.listen(lambda event: self.collision_data(event))

        while self.distance_to_obstacle_f is None:
            time.sleep(0.01)

        self.episode_start = time.time()

        self.vehicle.apply_control(carla.VehicleControl(throttle=1, brake=0.0))
        xx = self.distance_to_obstacle_f
        yy = self.distance_to_obstacle_r
        zz = self.distance_to_obstacle_l
        
        state_=np.array([xx,yy,zz])
        return state_

    def collision_data(self, event):
        self.collision_hist.append(event)

    def process_lidar(self, raw):
        points = np.frombuffer(raw.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))*np.array([1,-1,-1])
        
        lidar_f = lidar_line(points,90,2)
        lidar_r = lidar_line(points,45,2)
        lidar_l = lidar_line(points,135,2)

        if len(lidar_f) == 0:
            pass
        else:
            self.distance_to_obstacle_f = min(lidar_f[:,1])-2.247148275375366
        
        if len(lidar_r) == 0:
            pass
        else:
            self.distance_to_obstacle_r = np.sqrt(min(lidar_r[:,0]**2 + lidar_r[:,1]**2))
        
        if len(lidar_l) == 0:
            pass
        else:
            self.distance_to_obstacle_l = np.sqrt(min(lidar_l[:,0]**2 + lidar_l[:,1]**2))
    

    def step(self, action):
        sleepy=0.5
        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer = 0.3))
            time.sleep(sleepy)
            reward = 0.1
        elif action == 1:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer = -0.3))
            time.sleep(sleepy)
            reward =0.1

        
        if len(self.collision_hist) != 0:
            done = True
            reward = -10
        else :
            done=False
            reward=0.01
            
        if self.episode_start + SECONDS_PER_EPISODE < time.time():
            done = True
            
        xx = self.distance_to_obstacle_f
        yy = self.distance_to_obstacle_r
        zz = self.distance_to_obstacle_l
        state_=np.array([xx,yy,zz])
            
        return state_, reward, done, None

class Agent:

    def __init__(self, state_size, action_size, batch_size):
        self.state_size    = state_size
        self.action_size   = action_size
        self.batch_size    = batch_size
        self.memory        = deque(maxlen=2000)
        self.gamma         = 0.95   # discount rate
        self.epsilon       = 1.0    # exploration rate
        self.epsilon_min   = 0.01
        self.epsilon_decay = 0.995
        self.dqn           = self.build_model()
        
    def build_model(self):
        dqn = Sequential()
        dqn.add(Dense(24, input_dim=self.state_size, activation='tanh')) # input dimension = #states
        dqn.add(Dense(self.action_size, activation='linear'))            # output nodes = #action
        dqn.compile(loss='mse', optimizer=Adam(lr=0.01))                      
        print(dqn.summary())
        return dqn

    def act(self, state, explore):
        if explore and np.random.rand() <= self.epsilon: # explore/exploit tradeoff
            return random.randrange(self.action_size)
        act_values = self.dqn.predict(state)
        return np.argmax(act_values[0]) 

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train(self):
        if len(self.memory)<self.batch_size:
            print('YEAH')
            return

        X, dqnY = [], []
        minibatch = random.sample(self.memory, self.batch_size) 
        for state, action, reward, next_state, done in minibatch:
            X.append( state[0] )
            target = reward if done else reward + self.gamma * np.max(self.dqn.predict(next_state)[0])
            target_dqn = self.dqn.predict(state)
            target_dqn[0][action] = target
            dqnY.append( target_dqn[0] )

        self.dqn.train_on_batch( np.array(X), np.array(dqnY) )
        print('LOL')
 

        if self.epsilon > self.epsilon_min:    # gradually change from explore to exploit
            self.epsilon *= self.epsilon_decay

if __name__ == "__main__":
    env = CarEnv()
    state_size = 3
    action_size = 2
    print("{} actions, {}-dim state".format(action_size, state_size))
    agent = Agent(state_size, action_size, 32)
    
    emax = 5000
    for e in range(emax):
        while True:
            try:
                state = env.reset()
                state = state.reshape((1, state_size))
                if env.flag == 1:
                    break
            except RuntimeError as err:
                RuntimeError_count += 1
                print(err,' ',1)
                while True:
                    try:
                        time.sleep(15)  #wait for 15 sec
                        env.client = carla.Client('localhost', 2000) #reconnect to server
                        env.client.set_timeout(10.0)
                        env.world = env.client.get_world()  #if cannot reconnect, this line will cause an error ---> jump to the 'except' line
                        blueprint_library = env.world.get_blueprint_library()
                        env.model_3 = blueprint_library.filter('model3')[0]
                        xxx()
                        break
                    except RuntimeError as err:
                        RuntimeError_count += 1
                        print(err,' ',RuntimeError_count)
    
        
        for i in range(200):
            action = agent.act(state, True)
            next_state, reward, done, _ = env.step(action)
            next_state = next_state.reshape( (1,state_size) )
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            if done:
                print("episode: {}/{}, action: {}, e: {:.2}".format(e, emax, i, agent.epsilon))
                xxx()
                break
        agent.train()
        
