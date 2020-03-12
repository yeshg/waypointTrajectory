import pygame
from elements import Mouse, Robot, Waypoint, Trajectory

from scipy.interpolate import make_interp_spline, BSpline, interp1d, PchipInterpolator
from scipy.integrate import quad

import matplotlib.pyplot as plt
import numpy as np

import pickle

SCREEN_LENGTH = 580
SCREEN_WIDTH = 840
PX_2_M = 100 # X pixels = 1.0 meters
INTERP_NUM = PX_2_M // 10

class World:
    def __init__(self,screen_width,screen_height,px_2_m,interp_num=INTERP_NUM):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.px_2_m = px_2_m
        self.background_color = (0,0,0)
        self.mouse = Mouse()
        self.robot = None

        self.waypoints = []
        self.trajectory = None

        self.interp_num = interp_num

    def place_waypoint(self):
        mouse_position = self.mouse.get_position()
        self.waypoints.append(Waypoint(mouse_position))

    def waypointTrajectory(self):
        if len(self.waypoints) > 0:
            # xy points
            x = np.array([p.px for p in self.waypoints])
            y = np.array([p.py for p in self.waypoints])
            # parametrize by t
            t = np.linspace(0, 1, x.shape[0])
            # pchip interpolation
            spl = PchipInterpolator(t, np.c_[x,y])
            t_new = np.linspace(0, 1, num=self.interp_num*x.shape[0])
            x_new, y_new = spl(t_new).T
            points = [list(t) for t in zip(x_new, y_new)]
            print(points)
            self.trajectory = Trajectory(points)
            if self.robot is not None:
                self.robot.trajectory = self.trajectory.positions
            return t_new, x_new, y_new, spl
        return

    def initiate_robot(self, time_passed):
        self.robot = Robot(self.trajectory.positions, time_passed)

    def bias_robot(self):
        mouse_position = self.mouse.get_position()
        self.robot.px = mouse_position[0]
        self.robot.py = mouse_position[1]

    def update(self,time_passed):
        mouse_position = self.mouse.get_position()
        self.mouse.update(time_passed)
        if self.robot is not None:
            self.robot.update(time_passed)

    def render(self,screen):
        screen.fill(self.background_color)
        self.mouse.render(screen)
        if self.robot is not None:
            self.robot.render(screen)
        [x.render(screen) for x in self.waypoints]
        if self.trajectory is not None:
            self.trajectory.render(screen)

def pygame_main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_LENGTH), 0, 32)
    clock = pygame.time.Clock()
    world = World(SCREEN_WIDTH,SCREEN_LENGTH,PX_2_M)
    elapsed_time = 0.0
    done_with_waypoints = False
    # Game loop:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return world.robot.return_info(PX_2_M)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if not done_with_waypoints:
                    world.place_waypoint()
                    print("placed waypoint at {}".format(world.mouse.get_position()))
                else:
                    world.bias_robot()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    world.waypointTrajectory()
                    print("generated trajectory")
                    done_with_waypoints = True
                elif event.key == pygame.K_r:
                    elapsed_time = 0.0
                    world.initiate_robot(elapsed_time)
                    print("robot")

        time_passed_seconds = clock.tick() / 1000.0
        # time_passed_seconds = clock.tick() / 100.0
        elapsed_time += time_passed_seconds

        # print(elapsed_time)

        world.update(elapsed_time)
        world.render(screen)

        pygame.display.update()

def main():
    positions, velocity, yaw = pygame_main()

    command_trajectory = {"compos" : positions,
                          "speed"  : velocity,
                          "orient" : yaw}

    with open("command_trajectory.pkl", "wb") as f:
        pickle.dump(command_trajectory, f)
        print("wrote pickle file")

if __name__ == "__main__":
    main()