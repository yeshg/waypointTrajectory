import pygame
from pygame.locals import *
from PID import PIDController

import math
import numpy as np


class Mouse:
    def __init__(self):
        self.px = 0
        self.py = 0
        self.vx = 0
        self.vy = 0
        self.radius = 0
        self.color = (100,200,100)

    def get_position(self):
        return (self.px, self.py)
    
    def get_velocity(self):
        return (self.vx, self.vy)
    
    def update(self, time_passed):
        prev_p = self.get_position()
        self.px, self.py  = pygame.mouse.get_pos()
        if time_passed > 0:
            self.vx = (self.px - prev_p[0]) / time_passed
            self.vy = (self.py - prev_p[1]) / time_passed
    
    def render(self, screen):
        pygame.draw.circle(screen, self.color, (self.px, self.py), self.radius)
    
class Robot:
    def __init__(self, trajectory, time_passed):
        self.px = trajectory[0][0]
        self.py = trajectory[0][1]

        # action space is forward velocity and heading
        self.vel = 0.0
        self.theta = 0.0 # radians
        self.thetas = []
        self.velocities = []

        # save previous and have hard limits for max velocity and angle change
        # self.prev_vel = 0.0
        # self.prev_theta = 0.0
        # self.max_dvel = 0.05
        # self.max_dtheta = 0.05

        # PID controllers for velocity and angle
        self.pid_vel = PIDController(3, 0, 0)
        self.pid_theta = PIDController(3, 0, 0)

        self.radius = 10
        self.color = (100,100,230)
        
        self.trajectory = trajectory
        self.prev_time = time_passed

        self.counter = 0
        self.count_inc = 1
    
    # def update(self,time_passed):
    #     t_idx = min(int(time_passed), len(self.trajectory) - 1)
    #     time_interval = time_passed - self.prev_time
    #     traj_position = self.trajectory[t_idx]

    #     # setpoint is current angle from robot to next point in trajectory
    #     # the robot's angle is the current value
    #     desired_angle = math.atan2(traj_position[1] - self.py, traj_position[0] - self.px)
    #     self.theta = math.atan2(self.py, self.px)
    #     self.omega = self.pid_theta.get_u(desired_angle, self.theta)

    #     # forward velocity should be proportional to euclidean distance from goal
    #     fvel = 0.3 * np.sqrt(np.power(traj_position[1] - self.py, 2) + np.power(traj_position[0] - self.px, 2))

    #     # change the vx and vy
    #     vx = np.sin(self.theta + self.omega) * fvel
    #     vy = np.cos(self.theta + self.omega) * fvel

    #     # # output of PID controller is a change in translational velocity (a vector)
    #     # self.vx, self.vy = self.pid_controller.get_u(traj_position, current_value)

    #     # # use this vector to find dtheta, and limit it to self.max_detha
    #     # theta = math.atan2(self.vy, self.vx)
    #     # dtheta = theta - self.prev_theta
    #     # if abs(dtheta) > self.max_dtheta:
    #     #     new_vx = self.vx * math.cos(theta - self.max_dtheta) - self.vy * math.sin(theta - self.max_dtheta)
    #     #     new_vy = self.vx * math.sin(theta - self.max_dtheta) + self.vy * math.cos(theta - self.max_dtheta)
    #     #     self.vx, self.vy = new_vx, new_vy
    #     #     theta = math.atan2(self.vy, self.vx)
    #     #     dtheta = abs(theta - self.prev_theta)
    #     # print(dtheta)

    #     # # get the magnitude of this vector and limit it to self.max_dvel
    #     # vel = np.sqrt(np.power(self.vx, 2) + np.power(self.vy, 2))
    #     # dvel = vel - self.prev_vel
    #     # if abs(dvel) > self.max_dvel:
    #     #     # divide vector components by as much is necessary to limit magnitude to 0.05
    #     #     self.vx = self.vx / vel * self.max_dvel
    #     #     self.vy = self.vy / vel * self.max_dvel

    #     # robot's action space is forward velocity and angle
    #     self.px = int(self.px + vx*time_interval)
    #     self.py = int(self.py + vy*time_interval)
    #     # print("{}, {}".format(self.px, self.py))
    #     self.prev_time = time_passed

    def update(self,time_passed):
        # t_idx = min(int(time_passed), len(self.trajectory) - 1)
        t_idx = self.counter
        time_interval = time_passed - self.prev_time
        traj_position = self.trajectory[self.counter]

        # setpoint is current angle from robot to next point in trajectory
        # the robot's angle is the current value
        desired_value = traj_position
        current_value = (self.px, self.py)
        self.vx = self.pid_vel.get_u(desired_value[0], current_value[0])
        self.vy = self.pid_vel.get_u(desired_value[1], current_value[1])

        self.velocity = np.sqrt(np.power(self.vx, 2) + np.power(self.vy, 2))

        # robot's action space is forward velocity and angle
        self.px = int(self.px + self.vx*time_interval)
        self.py = int(self.py + self.vy*time_interval)
        # print("{}, {}".format(self.px, self.py))
        self.prev_time = time_passed
        # if self.counter == 0 or self.counter == len(self.trajectory) - 1:
        #     self.count_inc *= -1

        # calculate angle and append to logging list
        self.theta = math.atan2(self.vy, self.vx)

        if self.counter < len(self.trajectory) - 1:
            self.counter += self.count_inc
            self.thetas.append(self.theta)
            self.velocities.append(self.velocity)
            # print(self.velocity)

    def return_info(self, px_2_m):

        # thetas are the yaw angle of the robot
        thetas_rotated = self.thetas # no rotation for now
        # center of mass position is x y position converted to meters, with constant z height
        positions_in_meters = np.array( [[self.trajectory[i][0] / px_2_m - self.trajectory[0][0] / px_2_m, self.trajectory[i][1] / px_2_m - self.trajectory[0][1] / px_2_m] for i in range(len(self.trajectory))] )
        velocities_in_meters = np.array( [self.velocities[i] / px_2_m for i in range(len(self.velocities))] )

        print("positions:\n{}\n\nvelocities:\n{}\n\norient:\n{}\n".format(positions_in_meters, velocities_in_meters, thetas_rotated))

        return positions_in_meters, velocities_in_meters, thetas_rotated

    def render(self,screen):
        pygame.draw.circle(screen,self.color,(self.px,self.py),self.radius)
        pygame.transform.rotate(screen, np.radians(self.theta))

class Waypoint:
    def __init__(self, mouse_position):
        self.px = mouse_position[0]
        self.py = mouse_position[1]
        self.radius = 5
        self.color = (100,200,100)

    def get_position(self):
        return (self.px, self.py)
    
    def render(self, screen):
        pygame.draw.circle(screen, self.color, (self.px, self.py), self.radius)

class Trajectory:
    def __init__(self, positions):
        self.positions = positions
        self.width = 2
        self.color = (100,200,100)
    
    def render(self, screen):
        for i in range(len(self.positions)):
            # pygame.draw.aaline(screen, self.color, self.positions[i-1], self.positions[i])
            # print(self.positions[i])
            pygame.draw.circle(screen, self.color, (int(self.positions[i][0]),int(self.positions[i][1])), self.width)