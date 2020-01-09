"""
Crowd Simulation
11/23/2019
Motion planning final project: Dynamic Exit Signs
Justin Ngo, Nathan Collins, Greg Berube
"""

import graphics  # uses John Zelle's python graphics library
import numpy as np
import gridmap as gm
import random

_INFLUENCE_TIMER = 1000  # this is the amount of time steps after making a decision to follow a new sign before we can change our mind again


class Particle(object):
    """
    Class to simulate a person in a crowd
    """

    def __init__(self, x, y, vx, vy, ax, ay, default=True):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ax = ax
        self.ay = ay
        self.force = 0
        self.decisiveness = 0
        self.saw_hazard = None
        self.dt = 0.2

        # TODO: add rotation for different graphics i.e. arrow
        # self.scale_pos = None
        self.graphic = None

        if default:
            self.density = 1
            self.mass = 1
            self.maxSpeed = 2  # grid sq/sec
            self.maxForce = 5
            self.radius = 7
            self.minSeparation = self.radius * 2  # depends on how fat or something corresponds to radius
            self.maxCohesion = self.radius * 0.5

            self.color = gm.colors[0]
            self.reached_goal = False
            self.flow_field_index = None
        # TODO: ADD RANDOM PEOPLE I.E. OLD PEOPLE ARE SLOW W/DIFFERENT COLOR

    def unit_graphics(self, window):
        """
        :param window: window to draw on
        :return:
        """

        person = graphics.Point(self.x, self.y)
        self.graphic = graphics.Circle(person, self.radius)
        self.graphic.setOutline(self.color)
        self.graphic.setFill(self.color)
        self.graphic.draw(window)

    def apply_force(self):
        # dx = cx - self.x
        # dy = cx - self.y
        # if dx == dy == 0:
        #     return
        # force = math.sqrt(dx ** 2 + dy ** 2)
        # self.ax += force ** -1 * self.mass ** -1 * dx
        # self.ay += force ** -1 * self.mass ** -1 * dy

        # force is more like acceleration
        self.vx += self.force[0] * self.dt
        self.vy += self.force[1] * self.dt

        # cap velocity
        speed = np.linalg.norm((self.vx, self.vy))
        if speed > self.maxSpeed:
            self.vx = self.vx * self.maxSpeed / speed
            self.vy = self.vy * self.maxSpeed / speed

        self.oldx = self.x
        self.oldy = self.y
        self.x = self.oldx + self.vx * self.dt
        self.y = self.oldy + self.vy * self.dt

        self.decisiveness = max(self.decisiveness - 1,
                                0)  # after every step, we get closer to thinking about following a new sign

    # def move_physics(self):
    #     self.oldx, self.oldy = self.x, self.y
    #
    #     self.vx += self.ax * self.dt
    #     self.vy += self.ay * self.dt
    #     self.x += self.vx * self.dt
    #     self.y += self.vy * self.dt

    def move_graphic(self):
        if self.graphic:
            dx = self.x - self.oldx
            dy = self.y - self.oldy
            self.graphic.move(dx, dy)

    def collideWithWorld(self, grid):
        """
        :param grid: Gridmap class
        :return:
        """
        pxmin = self.x - self.radius
        pxmax = self.x + self.radius
        pymin = self.y - self.radius
        pymax = self.y + self.radius

        list_of_states = [(x, y) for x in range(grid.rows) for y in range(grid.cols)]
        for obstacle in list_of_states:
            if not grid.is_obstacle(obstacle):
                continue
            bb = grid.get_state_bounding_box(state=obstacle)
            # bb['xmin'] -= self.radius
            # bb['ymin'] -= self.radius
            # bb['xmax'] += self.radius
            # bb['ymax'] += self.radius

            # inside_x = self.x >= bb['xmin'] and self.x <= bb['xmax']
            # inside_y = self.y >= bb['ymin'] and self.y <= bb['ymax']
            # if inside_x and inside_y:
            #     closest_x_side = 0
            #     closest_y_side = 0
            #     if self.x - bb['xmin'] <= bb['xmax'] - self.x:
            #         closest_x_side = bb['xmin']
            #     else:
            #         closest_x_side = bb['xmax']
            #     if self.y - bb['ymin'] <= bb['ymax'] - self.y:
            #         closest_y_side = bb['ymin']
            #     else:
            #         closest_y_side = bb['ymax']
            #
            #     if abs(closest_x_side - self.x) < abs(closest_y_side - self.y):
            #         self.x += closest_x_side - self.x
            #     else:
            #         self.y += closest_y_side - self.y

            bb = grid.get_state_bounding_box(state=obstacle)
            crossingX = (pxmin >= bb['xmin'] and pxmin <= bb['xmax']) or (pxmax >= bb['xmin'] and pxmax <= bb['xmax'])
            crossingY = (pymin >= bb['ymin'] and pymin <= bb['ymax']) or (pymax >= bb['ymin'] and pymax <= bb['ymax'])

            if crossingX and crossingY:
                vec_to_pointOnEdgeX = min(max(bb['xmin'], self.x), bb['xmax']) - self.x
                vec_to_pointOnEdgeY = min(max(bb['ymin'], self.y), bb['ymax']) - self.y

                dist = np.linalg.norm([vec_to_pointOnEdgeX, vec_to_pointOnEdgeY])

                vec_to_pointOnEdgeX /= dist
                vec_to_pointOnEdgeY /= dist

                dist -= self.radius

                if dist < 0.0:
                    self.x += (vec_to_pointOnEdgeX * dist)
                    self.y += (vec_to_pointOnEdgeY * dist)
                    pxmin = self.x - self.radius
                    pxmax = self.x + self.radius
                    pymin = self.y - self.radius
                    pymax = self.y + self.radius

    def can_be_influenced(self):
        return self.decisiveness <= 0

    def follow_sign(self, sign_key):
        self.flow_field_index = sign_key
        # set the flow field to follow
        self.decisiveness = _INFLUENCE_TIMER

    def following_sign(self):
        return self.flow_field_index is not None


""" 
****************** Crowd Mechanics ********************************************
"""


# TODO: INJURY
def reached_goal(me, resolution, dg):
    # get the position in the grid
    # check if its a goal
    # return true or false
    rows = len(dg[0])
    x1 = int(me.x // resolution)
    y1 = int(me.y // resolution)

    for i in range(len(dg)):
        # iterate through all policy values
        if 0 == dg[i][visgrid_2map((x1, y1), rows)]:
            # if this policies value is zero: just do this
            return True
    # TODO: add something to handle goals as a result of following signs.
    return False


# FLOCKING
def seek_goal(particle, goal):
    """
    particle: single Particle class
    goal: Tuple (x, y)
    """
    (dx, dy) = goal[0] - particle.x, goal[1] - particle.y
    magnitude = np.linalg.norm((dx, dy))

    # go at max speed
    desired_vel = (particle.maxSpeed * dx / magnitude,
                   particle.maxSpeed * dy / magnitude)

    delta_vel = (desired_vel[0] - particle.vx,
                 desired_vel[1] - particle.vy)

    scalar = (particle.maxForce / particle.maxSpeed)
    force = tuple([scalar * f for f in delta_vel])

    return force


def seperation(crowd, me):
    # TODO: SEPERATE
    """
    crowd: list of Particle class
    me:     single Particle class
    
    Don't enter my bubble
    """
    totalForce = (0, 0)
    neighbors = 0

    for p in crowd:
        if p != me:
            distance = np.linalg.norm((me.x - p.x, me.y - p.y))
            if distance < me.minSeparation and distance > 0:
                push = (me.x - p.x, me.y - p.y)
                totalForce = (totalForce[0] + push[0] / me.radius,
                              totalForce[1] + push[1] / me.radius)
                neighbors += 1

    if neighbors == 0:
        return (0, 0)

    # normalized force
    totalForce = (totalForce[0] / neighbors * me.maxForce,
                  totalForce[1] / neighbors * me.maxForce)

    return totalForce


def align(crowd, me):
    """
    crowd: list of Particle class
    me:     single Particle class
    
    Stay close to each other velocity dependent
    """
    avgDir = (0, 0)
    neighbors = 0

    for p in crowd:
        distance = np.linalg.norm((me.x - p.x, me.y - p.y))
        speed = np.linalg.norm((p.vx, p.vy))

        if distance < me.maxCohesion and speed > 0:
            avgDir = (avgDir[0] + p.vx / speed, avgDir[1] + p.vy / speed)
            neighbors += 1

    if neighbors == 0:
        return 0, 0

    avgDir = (avgDir[0] / neighbors, avgDir[1] / neighbors)

    desired_vel = (avgDir[0] * me.maxSpeed, avgDir[1] * me.maxSpeed)
    delta_vel = (desired_vel[0] - me.vx,
                 desired_vel[1] - me.vy)

    scalar = (me.maxForce / me.maxSpeed)
    force = tuple([scalar * f for f in delta_vel])

    return force


def cohesion(crowd, me):
    """
    crowd: list of Particle class
    me:     single Particle class
    
    Stay close to each other position dependent 
    """
    com = (me.x, me.y)  # Center of mass
    neighbors = 1

    # for a person in the crowd
    for p in crowd:
        if p != me:
            distance = np.linalg.norm((me.x - p.x, me.y - p.y))
            if distance < me.maxCohesion:
                com = (com[0] + p.x, com[1] + p.y)
                neighbors += 1

    # if I'm a neighbor to myself            
    if neighbors == 1:
        return 0, 0

        # average com
    com = (com[0] / neighbors, com[1] / neighbors)

    # head to com
    return seek_goal(me, com)


def choose_the_sign(me, resolution, base_dg, sign_dg, obedience=None):
    # get the position in the grid
    # look at all policies values at that position
    # choose a policy index based on value
    # Obedience: None or probability from 0 to 100
    rows = len(base_dg)
    x1 = int(me.x // resolution)
    y1 = int(me.y // resolution)
    map_pos = visgrid_2map((x1, y1), rows)

    if 0 == np.asarray(sign_dg[map_pos]):
        return True
    elif 0 == np.asarray(base_dg[map_pos]):
        return False
    if obedience is None:
        sign_weight = 1 / (sign_dg[map_pos] ** 4)
        base_weight = 1 / (sign_dg[map_pos] ** 4)
        # should now have a list of indeces:
        # the number of times each index appears is inv prop to policy value^2
        # print(index_weights)
        choice = random.choices([True, False], weights=[sign_weight, base_weight], k=1)
        # choice = random.choices([True, False], weights=[0.2, 0.8], k=1)
        return choice[0]
    else:
        return np.random.random()*100 <= obedience


def flowfieldfollow(me, resolution, ff, basic=True):
    # TODO: Later add resolution for x, y if I want non-squre maps
    """
    me: single Particle class
    """
    rows = len(ff)
    # this will always give bottom left of grid
    x1 = int(me.x // resolution)
    y1 = int(me.y // resolution)
    if basic:
        # basic follow , whats the grid vector im on right now
        vector = np.asarray(ff[visgrid_2map((x1, y1), rows)])
    else:
        # using Bilinear Interpolation for smoother.
        # see wikipedia, gives tuple(y,x)
        f00 = np.asarray(ff[visgrid_2map((x1, y1), rows)])
        f01 = np.asarray(ff[visgrid_2map((x1, y1 + 1), rows)])
        f10 = np.asarray(ff[visgrid_2map((x1 + 1, y1), rows)])
        f11 = np.asarray(ff[visgrid_2map((x1 + 1, y1 + 1), rows)])

        #    xw = me.x - x1*resolution
        #    yw = me.y - y1*resolution
        #    numer = f00*(1-xw) + f10*xw
        #    denom = f01*(1-xw) + f11*xw
        #    vector = numer*(1-yw) + denom*(yw)

        # interpolation
        x2, y2 = x1 + 1, y1 + 1
        x2 = x2 * resolution
        y2 = y2 * resolution
        x1 = x1 * resolution
        y1 = y1 * resolution
        # Note: x2 - x1 = 1 also y2 - y1 = 1
        # will be different if it wasn't square grid

        vector = (f00 * (x2 - me.x) * (y2 - me.y) +
                  f10 * (me.x - x1) * (y2 - me.y) +
                  f01 * (x2 - me.x) * (me.y - y1) +
                  f11 * (me.x - x1) * (me.y - y1)) * (resolution ** 2) ** -1

    # need to swap since fxx gives y,x
    vector[0], vector[1] = vector[1], vector[0]
    unit_v = vector / np.linalg.norm(vector)

    if np.isnan(unit_v).any():
        return np.array((0, 0))

    des_vel = unit_v * me.maxSpeed
    delta_v = des_vel - np.array((me.vx, me.vy))

    force = delta_v * (me.maxForce / me.maxSpeed)

    return force


def visgrid_2map(grid_state, rows):
    """
    grid_state: tuple(x,y) ==> r,c
    """
    r = (rows - 1) - grid_state[1]
    c = grid_state[0]

    return r, c
