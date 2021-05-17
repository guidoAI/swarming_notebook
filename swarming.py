# -*- coding: utf-8 -*-
"""
Created on Mon May 17 16:11:16 2021

Simple script that simulates a swarm of agents with heading and velocity control.

@author: Guido de Croon, TU Delft
"""

import numpy as np
from matplotlib import pyplot as plt
from IPython import display
import time

class Environment:
    
    def __init__(self, width, height):
        
        # just a rectangular environment
        self.width = width
        self.height = height
        self.agents = []
        self.figure_handle = []
        
    def get_agents(self):
        
        return agents
    
    def add_agent(self, agent):
        
        self.agents.append(agent)

    def update_agents(self, dt):
        
        for a in self.agents:
            a.update(dt)
    
    def draw(self):
        
        if self.figure_handle == []:
            self.figure_handle = plt.figure('swarming')
        else:
            plt.figure('swarming')
        
        ax = self.figure_handle.add_subplot(111)
        plt.ion()
        self.figure_handle.canvas.draw()
        ax.clear()
        
        # draw the border:
        plt.plot([0, self.width, self.width, 0, 0], [0, 0, self.height, self.height, 0], 'k-')
        
        for a in self.agents:
            a.draw()
        
        plt.xlim([-1, self.width+1])
        plt.ylim([-1, self.height+1])
        
        display.clear_output(wait=True)
        display.display(plt.gcf())

class Agent:
    
    def __init__(self, environment):
        
        # an agent always needs to be embedded in an environment:
        self.environment = environment
        
        # random place and heading in the environment:
        self.x = np.random.rand() * environment.width
        self.y = np.random.rand() * environment.height
        self.heading = np.random.rand() * 2 * np.pi
        # zero velocity and commanded velocity:
        self.v = 0.0
        self.command_v = 0.0
        
        # heading rate and commanded rate
        self.rate = 0.0
        self.command_rate = 0.0
        
        # how quickly a command is satisfied depends on these low-pass factors:
        self.v_factor = 0.9
        self.rate_factor = 0.9
        
    
    def update(self, dt):
        
        # get command:
        self.set_command()
        
        # update velocities and rate
        self.v = (1-self.v_factor) * self.v + self.v_factor * self.command_v # make low-pass depend on dt?
        self.vx = np.cos(self.heading) * self.v
        self.vy = np.sin(self.heading) * self.v
        self.rate = (1-self.rate_factor) * self.rate + self.rate_factor * self.command_rate
        
        # update position and heading
        self.x += dt * self.vx
        self.y += dt * self.vy
        self.heading += dt * self.rate
        
        # respect bounds of the environment:
        if self.x > self.environment.width:
            self.x = self.environment.width
        elif self.x < 0:
            self.x = 0
        
        if self.y > self.environment.height:
            self.y = self.environment.height
        elif self.y < 0:
            self.y = 0
        
        
    def set_command(self):
        
        # standard function sets rate and velocity to zero - has to be overloaded
        self.command_v = 0.0
        self.command_rate = 0.0
        
    def draw(self, time_step = 3):
        
        plt.plot(self.x, self.y, 'bo')
        plt.plot([self.x, self.x + self.vx * time_step], [self.y, self.y + self.vy * time_step], 'r')
    
    def sense_nearest_neighbors(self, k = 3, max_range = 20):
        
        agents = self.environment.get_agents()
        n_agents = len(agents)
        if n_agents == 1:
            # that is the agent itself:
            return []
        
        # determine the distances to all agents:
        distances = np.zeros([n_agents, 1])
        for a in range(n_agents):
            agent = agents[a]
            dx = agent.x - self.x
            dy = agent.y - self.y
            distances[a] = np.sqrt(dx*dx + dy*dy)
            
        # get the k closest agents:
        closest_agents = np.argsort(distances)
        n_closest = len(closest_agents)
        # we start from 1, since the closest agent is the agent itself:
        if len(closest_agents) > k:
            closest_agents = closest_agents[1:k+1]
        else:
            closest_agents = closest_agents[1:]
        n_closest = len(closest_agents)
        
        # add the right agents
        neighbors = closest_agents
        delta_pos = []
        delta_v = []
        
        # make the rotation matrix for transforming to the body frame:
        rot_angle = -self.heading
        R = np.zeros([2,2])
        R[0,0] = np.cos(rot_angle)
        R[0,1] = -np.sin(rot_angle)
        R[1,0] = np.sin(rot_angle)
        R[1,1] = np.cos(rot_angle)

        R = np.zeros([2,2])
        for n in range(n_closest):
            agent = closest_agents[n]
            if(distances(agent) < max_range):
                # The agent is in range:
                
                # dx and dy in inertial frame
                d_agent = np.zeros([2,1])
                d_agent[0] = agent.x - self.x
                d_agent[1] = agent.y - self.y
                
                # convert to body frame:
                d_agent_body = R.dot(d_agent)

                # relative position to agent:
                delta_pos.append(d_agent_body)

                # dx and dy in inertial frame after 1 second:
                d_agent_next = np.zeros([2,1])
                d_agent_next[0] = agent.x + agent.vx - self.x - self.vx
                d_agent_next[1] = agent.y + agent.vy - self.y - self.vy
                
                # convert to body frame:
                d_agent_body_next = R.dot(d_agent_next)
                v_body = d_agent_body_next - d_agent_body 
                
                # relative velocity
                delta_v.append(v_body)

        return [neighbors, delta_pos, delta_v]
        

if __name__ == "__main__":    

    env = Environment(100, 100)
    for i in range(10):
        a = Agent(env)
        env.add_agent(a)
    
    dt = 0.1
    n_time_steps = 1000
    
    figure_handle = plt.figure('swarming')
    ax = figure_handle.add_subplot(111)
    plt.ion()
    figure_handle.canvas.draw()
    
    plt.figure()
    for t in range(n_time_steps):
        env.update_agents(dt)
        
        plt.figure('mountain_car')
        ax.clear()
        env.draw()
        display.clear_output(wait=True)
        display.display(plt.gcf())
        time.sleep(dt)
        


