"""
Crowd Simulation - gripmap.py
11/23/2019
Motion planning final project: Dynamic Exit Signs
Justin Ngo, Nathan Collins, Greg Berube

This class represents the map
"""

import math
import graphics  # uses John Zelle's python graphics library
import sys
import numpy as np
from particle import Particle


colors = ['blue', 'orange', 'red', 'green', 'brown', 'purple', 'cyan', 'black']
# state representation (y[row],x[col],t)
_X = 1
_Y = 0
_DEFAULT_SIGN_INFLUENCE = 1.5
_DEFAULT_HAZARD_VISIBILITY = 2

class GridMap(object):
    """
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell,
    x - occupied cell,
    g - goal location,
    i - initial location.
    upper case - Sign associated goal
    lower case - sign location
    z - random obstacles that can occur at start of simulation
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    """

    def __init__(self, winsize, map_path=None):
        """
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        """
        self.rows = None
        self.cols = None
        self.goal = []
        self.sign_goals = {}
        self.init_pos = []
        self.hazards = []
        self.occupancy_grid = None
        self.winsize = winsize

        if map_path is not None:
            self.read_map(map_path)

    def read_map(self, map_path):
        """
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        """
        map_file = open(map_path, 'r')
        lines = [l.rstrip() for l in map_file.readlines()]
        map_file.close()
        self.rows = len(lines)
        self.cols = max([len(l) for l in lines])
        self.occupancy_grid = np.zeros((self.rows, self.cols), dtype=np.bool)
        for r in range(self.rows):
            for c in range(self.cols):
                if lines[r][c] == 'x':
                    self.occupancy_grid[r][c] = True
                elif lines[r][c] == 'g':
                    self.goal.append((r, c))
                elif lines[r][c] == 'i':
                    self.init_pos.append((r, c))
                elif lines[r][c] == 'z':
                    #TODO: probabilitiy associated with hazards, fire have bigger radius you can tell etc i.e fire =2
                    # tuple (r, c, radius)
                    self.hazards.append((r,c, _DEFAULT_HAZARD_VISIBILITY))
                elif lines[r][c].isalpha():
                    char = lines[r][c]
                    char_key = char.lower()
                    if char_key not in self.sign_goals.keys():
                        self.sign_goals[char_key] = SignGoal(None,None,self.rows,self.cols, influence=_DEFAULT_SIGN_INFLUENCE)
                    if char.isupper():
                        self.sign_goals[char_key].goal = (r,c)
                    elif char.islower():
                        self.sign_goals[char_key].sign = (r,c)

    def is_obstacle(self, s):
        return self.occupancy_grid[s[0]][s[1]]

    def is_goal(self, s):
        """
        Test if a specifid state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        """
        return (s[_X] == self.goal[_X] and
                s[_Y] == self.goal[_Y])

    def transition(self, s, all_actions=False, corners=False):
        """
        s - tuple describing the state as (row, col) position on the grid.

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        returns the current state.

        Justin Ngo 10/26/2019 - Planning with uncertainity update
        Simulator = False (default)
            - this variable toggles 2 version
            - False: Returns a model of probability distributions a list of 2-tuples, each tuple holds
            a probability and state.
            - True: Returns a state according to transition probabilites
            the default is 80% making the correct attion and 10% chance in either direction perpendicular to commanded action

        Transition probabilites 'tp' = tuple(correct action, neighbor action, orthogonal action)
        i.e. neighbor action --> up-left, up-right for commanded action 'up'
        i.e. orthogonal aciton --> right or left given 'up' command
        """

        next_states = []
        list_a = self.probable_actions(s, all_actions, corners)
        for action in list_a:
            next_states.append(self.state_move(s, action))
        return next_states

    def state_move(self, s, a, check=False):
        """

        :param s: state tuple(r,c)
        :param a: actions string i.e. 'u', 'ne'
        :param check: bool to see if can pass corners obstacles
        :return: new state tuple(r,c) or bool
        """
        new_pos = list(s[:])
        # Ensure action stays on the board
        if a == 'u':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'd':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
        elif a == 'l':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'r':
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        # added diagonal moves Justin Ngo 9/7/2019
        elif a == 'ne':
            if s[_X] < self.cols - 1 and s[_Y] > 0:
                new_pos[_Y] -= 1
                new_pos[_X] += 1
        elif a == 'nw':
            if s[_X] > 0 and s[_Y] > 0:
                new_pos[_Y] -= 1
                new_pos[_X] -= 1
        elif a == 'sw':
            if s[_X] > 0 and s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
                new_pos[_X] -= 1
        elif a == 'se':
            if s[_X] < self.cols - 1 and s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
                new_pos[_X] += 1
        else:
            print('Unknown action:', str(a))

        if not check:
            # Test if new position is clear
            if self.occupancy_grid[new_pos[0], new_pos[1]]:
                s_prime = tuple(s)
            else:
                s_prime = tuple(new_pos)
            return s_prime
        else:
            if self.occupancy_grid[new_pos[0], new_pos[1]]:
                return True

    def probable_actions(self, s, all_actions, corners):
        """
        s: state tuple(r,c)
        all_actions: bool
        corners: bool
        Returns a list of probable actions
        """

        if not all_actions:
            probable_a = ['u', 'd', 'l', 'r']
        else:
            probable_a = ['u', 'd', 'l', 'r', 'ne', 'nw', 'sw', 'se']
            if corners:
                obstructed = set()
                for a in probable_a[:4]:
                    if self.state_move(s, a, check=True):
                        if a == 'u':
                            obstructed.add('ne')
                            obstructed.add('nw')
                        elif a == 'd':
                            obstructed.add('sw')
                            obstructed.add('se')
                        elif a == 'l':
                            obstructed.add('nw')
                            obstructed.add('sw')
                        elif a == 'r':
                            obstructed.add('ne')
                            obstructed.add('se')

                for obs in obstructed:
                    probable_a.remove(obs)

        return probable_a

    def make_particles(self, n, win):
        """
        n : number of people in crowd
        win: window to draw
        """

        # TODO: instead of the below, try a metaclass which supports iteration
        crowd = [Particle(0, 0, 0, 0, 0, 0) for _ in range(n)]

        # TODO: Determine where to spawn the crowd
        if len(self.init_pos) == 0:
            sys.exit("No place to spawn crowd")
        elif len(self.init_pos) > 1:
            # TODO: Find all neighbors and make into a big area but this will work for now
            n_spawn = n // len(self.init_pos)
            if n_spawn == 0:
                sys.exit("Increase crowd size or decrease spawn points")
            elif n % len(self.init_pos) != 0:
                print("Crowd size: ", n)
                print("Number of spawn points: ", len(self.init_pos))
                sys.exit("Crowd size must be divisible by number of spawn points")
            count = 0
            # break crowd evenly
            for each_pos in self.init_pos:
                pos = self.map2_visgrid(each_pos)
                for spawn in range(count, count + n_spawn):
                    self.spawnParticle(crowd[spawn], pos)
                count += n_spawn
        else:
            pos = self.map2_visgrid(self.init_pos[0])
            # TODO: make the random position in a given space with proper spacing
            for p in crowd:
                self.spawnParticle(p, pos)

        # Particle Graphic Initialization
        for p in crowd:
            p.unit_graphics(win)

        return crowd

    def spawnParticle(self, p, pos):
        """
        p: single Particle class
        pos: x,y tuple of grid
        """
        p.x = np.random.randint(pos[0] * self.stepsize + p.radius,
                                pos[0] * self.stepsize + self.stepsize - p.radius)
        p.y = np.random.randint(pos[1] * self.stepsize + p.radius,
                                pos[1] * self.stepsize + self.stepsize - p.radius)

    # def loc_to_state(self, x, y):
    #     """
    #     particle location: x,y ==> r,c
    #     """
    #     r = (self.rows - 1) - int(y // self.stepsize)
    #     c = int(x // self.stepsize)
    #     return (r,c)

    def map2_visgrid(self, state):
        """
        state: tuple(r,c) ==> y,x
        """
        y = (self.rows - 1) - state[0]
        x = state[1]

        return (x, y)

    def get_state_bounding_box(self, state=None, pos=None):
        """
        supply either state or pos, but not both; returns bounding
        box of the provided state/pos
        :param state: tuple(r,c) ==> y,x
        :param pos: x,y tuple of grid
        :return: dict with 'xmin','xmax','ymin','ymax'
        """
        if state is not None and pos is None:
            y = (self.rows - 1) - state[0]
            x = state[1]
        elif state is None and pos is not None:
            x = pos[0]
            y = pos[1]
        else:
            print("You didn't use this method right...")
        xmin = x * self.stepsize
        xmax = xmin + self.stepsize
        ymin = y * self.stepsize
        ymax = ymin + self.stepsize
        return {'xmin': xmin, 'xmax': xmax, 'ymin': ymin, 'ymax': ymax}

    def init_grid(self, win):
        """
        win: window to draw in
        """
        # TODO: any map size instead of just square
        try:
            if self.rows == self.cols:
                self.gridsize = self.rows
        except:
            sys.exit("Need square map")
        self.stepsize = self.winsize // self.gridsize  # want integer

        # draw grid lines
        for x in range(0, self.winsize + self.stepsize, self.stepsize):
            line = graphics.Line(graphics.Point(x, 0),
                                 graphics.Point(x, self.winsize))
            line.draw(win)
        for y in range(0, self.winsize + self.stepsize, self.stepsize):
            line = graphics.Line(graphics.Point(0, y),
                                 graphics.Point(self.winsize, y))
            line.draw(win)

        # draw goal
        for goal in self.goal:
            pos = self.map2_visgrid(goal)
            self.drawEnv(win, pos, colors[3])

        # draw initial
        for spawn_pt in self.init_pos:
            pos = self.map2_visgrid(spawn_pt)
            self.drawEnv(win, pos, colors[1])

        # draw obstacles
        for r in range(self.rows):
            for c in range(self.cols):
                if self.occupancy_grid[r, c] == True:
                    pos = self.map2_visgrid((r, c))
                    self.drawEnv(win, pos, colors[7]) # black

        # draw hazards
        for hazard in self.hazards:
            pos = self.map2_visgrid(hazard)
            # self.drawEnv(win, pos, graphics.color_rgb(130, 0, 130)) # red
            self.drawEnv(win, pos, "red4")  # red
            radius = hazard[2]*self.stepsize
            center = graphics.Point((pos[0]+0.5)*self.stepsize, (pos[1]+0.5)*self.stepsize)
            circle = graphics.Circle(center, radius)
            circle.setOutline("red4")
            circle.setWidth(3)
            circle.draw(win)

        # draw signs
        for sign in self.sign_goals.values():
            pos_sign = self.map2_visgrid(sign.sign)
            aCircle = graphics.Circle(graphics.Point((sign.sign[1]+0.5)*self.stepsize, (self.map2_visgrid(sign.sign)[1]+0.5)*self.stepsize),
                                      sign.sign_radius*self.stepsize)
            aCircle.setOutline(colors[6])
            aCircle.setWidth(3)
            aCircle.draw(win)
            pos_goal = self.map2_visgrid(sign.goal)

            self.drawEnv(win, pos_sign, colors[6])
            self.drawEnv(win, pos_goal, colors[5])

    def drawEnv(self, win, pos, color):
        """
        win: window to draw
        pos: center point of grid (tuple(x,y))
        color: string "color"
        """
        p1 = graphics.Point(pos[0] * self.stepsize, pos[1] * self.stepsize)
        p2 = graphics.Point(pos[0] * self.stepsize + self.stepsize,
                            pos[1] * self.stepsize + self.stepsize)
        square = graphics.Rectangle(p1, p2)
        square.draw(win)
        square.setFill(color)

    #        square.setFill("black")

    def DijkstraGrid(self, goal):
        """

        :param goal: tuple(r,c)
        :return: dijkstra grid
        """
        dg = np.full((self.rows, self.cols), -1.0)

        for r in range(self.rows):
            for c in range(self.cols):
                if self.occupancy_grid[r, c] == True:
                    dg[r, c] = math.inf

        # TODO: later be able to deal with wider goal then one block

        node_0 = Node(goal, 0)
        dg[goal] = 0
        visited = []
        frontier = []
        frontier.append(node_0)

        while len(frontier) > 0:
            # always in order of lowest cost (BFS)
            node_n = frontier.pop(0)

            if node_n.state not in visited:
                visited.append(node_n.state)
                neighbors = self.transition(node_n.state)
                for neighbor in neighbors:
                    if dg[neighbor] < 0 and neighbor not in visited:
                        distance = node_n.cost + 1
                        new_node = Node(neighbor, distance)
                        dg[neighbor] = distance
                        frontier.append(new_node)
        return dg

    def createFlowField(self, dg):
        """

        :param dg: dijkstra grid
        :return: flow field
        """
        ff = np.empty((self.rows, self.cols), dtype=np.dtype(object))

        for r in range(self.rows):
            for c in range(self.cols):
                if dg[r, c] == math.inf:
                    ff[r, c] = (0, 0)
                    continue

                neighbors = self.transition((r, c), all_actions=True, corners=True)

                minNeighbor = None
                minDist = 0

                for neighbor in neighbors:
                    r2, c2 = neighbor
                    dist = dg[r2, c2] - dg[r, c]
                    if dist < minDist:
                        minNeighbor = neighbor
                        minDist = dist

                if minNeighbor != None:
                    # translate to visual grid. i.e (0,0) is at bottom left corner
                    direction = tuple((r - minNeighbor[0], minNeighbor[1] - c))
                    normalize = tuple([x * np.linalg.norm((direction[1], direction[0])) ** -1 for x in direction])
                    ff[r, c] = normalize
                else:  # this is goal
                    ff[r, c] = (0, 0)

        return ff

    def visualize_dg(self, win, dg):
        """

        :param win: which window to draw
        :param dg: dijkstras grid
        :return:
        """
        for r in range(self.rows):
            for c in range(self.cols):
                if dg[r, c] != math.inf:
                    x, y = self.map2_visgrid((r, c))
                    label = graphics.Text(graphics.Point(x * self.stepsize + self.stepsize * 0.5,
                                                         y * self.stepsize + self.stepsize * 0.5), dg[r, c])
                    label.setTextColor('red')
                    #                    label.setSize(20)
                    #                    label.setStyle('italic')
                    label.draw(win)

    def visualize_flow(self, win, ff):
        """

        :param win: which window to draw in
        :param ff: flow field grid
        :return:
        """
        for r in range(self.rows):
            for c in range(self.cols):
                if ff[r, c] != (0, 0):
                    x, y = self.map2_visgrid((r, c))
                    # center
                    cx = x * self.stepsize + self.stepsize * 0.5
                    cy = y * self.stepsize + self.stepsize * 0.5

                    # vector
                    vector = ff[r, c]
                    vx = vector[1]
                    vy = vector[0]

                    p1 = graphics.Point(cx, cy)
                    p2 = graphics.Point(cx + 0.5 * vx * self.stepsize,
                                        cy + 0.5 * vy * self.stepsize)
                    arrow = graphics.Line(p1, p2)
                    arrow.setArrow("last")
                    arrow.draw(win)

    def hazard_encounter(self, state):
        """

        :param state: particle location on visual grid (x , y) in pixels
        :return: True or False if within hazard radius
        """
        for hazard in self.hazards:
            radius = hazard[2]
            x, y = self.map2_visgrid(hazard)
            encountered = np.linalg.norm([state[0] - (x + 0.5)*self.stepsize,
                                          state[1] - (y+0.5)*self.stepsize]) <= radius*self.stepsize
            if encountered:
                return encountered
        return False


class Node(object):
    def __init__(self, s, cost):
        """
        Class save the state and cost of a node in the GridMap class to generate dijkstra's grid and flow field
        :param s: state tuple(row, col)
        :param cost: cost
        """
        self.cost = cost
        self.state = s[:]


class SignGoal(object):
    def __init__(self, goal, sign, rows, cols, influence=5):
        self.sign = sign
        self.goal = goal
        self.sign_radius = influence
        self.dg = None
        self.ff = None
        self.rows = rows
        self.cols = cols

    def compute_dg_ff(self, map):
        return self._generate_dg(map) and self._generate_ff(map)

    def _generate_dg(self, map: GridMap):
        self.dg = map.DijkstraGrid(self.goal)
        return True

    def _generate_ff(self, map: GridMap):
        if self.dg is None:
            return False
        self.ff = map.createFlowField(self.dg)
        return True

    def map2_visgrid(self, state):
        """
        state: tuple(r,c) ==> y,x
        """
        y = (self.rows - 1) - state[0]
        x = state[1]
        return (x, y)

    def within_influence(self, state, stepsize):
        x, y = self.map2_visgrid(self.sign)
        return np.linalg.norm([state[0] - (x + 0.5) * stepsize,
                               state[1] - (y + 0.5) * stepsize]) <= self.sign_radius * stepsize

        # return np.linalg.norm([state[0] - self.sign[0], state[1] - self.sign[1]]) <= self.sign_radius

