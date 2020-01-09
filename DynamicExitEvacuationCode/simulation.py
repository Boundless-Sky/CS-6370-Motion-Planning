"""
Crowd Simulation
11/23/2019
Motion planning final project: Dynamic Exit Signs
Justin Ngo, Nathan Collins, Greg Berube
"""

import time as time
from graphics_routines import *  # uses John Zelle's python graphics library
from gridmap import GridMap
from particle import *

""" 
**************************** Simulation ***************************************
"""
colors = ['blue', 'orange', 'red', 'green', 'brown', 'purple', 'cyan', 'black']
_ODDS = [0.5, 0.5]


def simulation(n, steps, map_path, update_interval=1, fps=None, obedience=None, multi_sim=False):
    """

    :param n: crowd size
    :param steps: how many times to simulate
    :param map_path: what map file to use
    :param update_interval: how fast to update being drawn
    :param fps: caps frames per seconds to have smooth video
    :Param obediance: 0 to 100 percent of following sign. If none then don't overide base case
    :return:
    """
    winsize = 600
    # goal = (winsize // 1.2, winsize // 1.2)
    # update_interval is visualization update interval

    # obtain map. NOTE: be careful of hidden spaces and enters in map file.
    g = GridMap(winsize, map_path)

    # create graphics
    #    win, text = init_graphics(winsize)
    win = init_graphics(winsize)

    # create grid
    # Make grid to be square to be easier to work with
    g.init_grid(win)

    # create crowd
    crowd = g.make_particles(n, win)

    # Dijkstra's Algorithm
    dg = []  # values of policy
    ff = []  # direction vectors of policy

    for every_goal in g.goal:
        dg.append(g.DijkstraGrid(every_goal))
    for every_dg in dg:
        ff.append(g.createFlowField(every_dg))
    for every_sign_goal in g.sign_goals.values():
        every_sign_goal.compute_dg_ff(g)
        dg.append(every_sign_goal.dg)
        ff.append(every_sign_goal.ff)

    # g.visualize_dg(win, dg[0])
    # g.visualize_dg(win, dg)
    # TODO: try dynamic Fields later
    # g.visualize_flow(win, ff[0])d
    # g.visualize_flow(win, ff)

    # TODO: NATHAN add a list of SignGoals
    # TODO: NATHAN precompute dg and ff of sign goals

    if not multi_sim:
        # wait until you click on window to start simulation
        win.getMouse()
        # win.getKey()

    # simulate crowd
    start_t = time.time()
    # TODO: while simulation running, click to add obstacles
    step = 0
    dt = 0
    running = True
    # for step in range(steps):
    while running:
        running = False

        # decide which flow field to follow
        # should only do these if p is still in bounds
        for p in crowd:
            dt = p.dt
            # TODO: NATHAN change this so it actually does something with some random prob. distribution
            if not reached_goal(p, g.stepsize, dg):
                # if any agent hasn't reached any goal
                running = True  # continue running

            if winsize >= p.x >= 0 and winsize >= p.y >= 0:  # if im inside the building and not following a sign
                for key, sign_goal in g.sign_goals.items():
                    # TODO: Make the hazard a stand alone and not dependant on signs
                    if g.hazard_encounter((p.x, p.y)):
                        p.follow_sign(key)
                        p.saw_hazard = 1  # TODO: if multiple hazard ID them
                    else:
                        # if sign_goal.within_influence(g.loc_to_state(p.x, p.y)) or p.can_be_influenced():  # basically, if we're close enough to the sign for it to do something to us
                        if sign_goal.within_influence((p.x, p.y),g.stepsize) and  p.saw_hazard is None:
                            # TODO: Follow the sign previously ignored
                            # TODO: This should be going back to the sign last saw
                            # TODO: this is for people changing their mind (i.e. i haven't seen an exit in a while maybe I should follow exit sign)
                            if obedience is None:
                                follow = choose_the_sign(p, g.stepsize, dg[0],
                                                         sign_goal.dg)  # random.choices([True, False], _ODDS)[0]
                            else:
                                follow = choose_the_sign(p, g.stepsize, dg[0], sign_goal.dg, obedience)

                            if follow:
                                p.follow_sign(key)
                            else:
                                p.follow_sign(None)
                                # p.ff_index = flowfieldchoose(p, g.stepsize, dg) # TODO: change flowfieldchoose to take in list of SignGoal objects
                        
                        if p.can_be_influenced() and p.saw_hazard is None: #if im stuck in a crowd, perhaps follow sign
                            if choose_the_sign(p, g.stepsize, dg[0], sign_goal.dg, 50):
                                p.follow_sign(key)
                            
        # obtain force from map position, other particles, etc
        for p in crowd:
            if winsize >= p.x >= 0 and winsize >= p.y >= 0:
                if p.following_sign():
                    seek = 0.1 * flowfieldfollow(p, g.stepsize, g.sign_goals[p.flow_field_index].ff, basic=True)
                else:
                    seek = 0.1 * flowfieldfollow(p, g.stepsize, ff[0], basic=True)

                # seek = seek_goal(p, np.array(g.goal[0]) * g.stepsize * 1.5)
                seperate = seperation(crowd, p)
                # attachment = cohesion(crowd, p)
                # alignment = align(crowd, p)

                # scale cohesion so that people in front aren't too affected by being pulled back
                w_cohesion = 0.05

                # p.force = (seek[0] + seperate[0] + w_cohesion * attachment[0] + alignment[0],
                #            seek[1] + seperate[1] + w_cohesion * attachment[1] + alignment[1])
                p.force = (seek[0] + seperate[0],
                           seek[1] + seperate[1])
                # TODO: for each person in crowd apply force from other particles
                # TODO: instead of O^n search for each, try quadtree, cell binning, spatial index

            # move crowd
        for p in crowd:
            if winsize >= p.x >= 0 and winsize >= p.y >= 0:
                p.apply_force()

                # adjust particles when they're in collision with the world
        for p in crowd:
            if winsize >= p.x >= 0 and winsize >= p.y >= 0:
                p.collideWithWorld(g)

        # update visualization
        for p in crowd:
            p.move_graphic()
            if p.x > winsize or p.x < 0 or p.y > winsize or p.y < 0:
                p.graphic.undraw()
            if reached_goal(p, g.stepsize, dg):
                # if an agent reached any goal
                p.graphic.undraw()
                crowd.remove(p)

        if not multi_sim:
            #        update_step(win, text, step, update_interval)
            update_step(win, step, update_interval, fps)
        else:
            update_step(win, step, 1)

        step = step + 1
        # if step==800:
        #     # after 500 steps, switch to other map, which has blocked goals
        #     # update dg, ff according to new map
        #     map_path2 = './map1_2.txt'
        #     g = GridMap(winsize, map_path2)
        #
        #     # create grid
        #     g.init_grid(win)
        #     # draw obstacles
        #     for r in range(g.rows):
        #         for c in range(g.cols):
        #             if g.occupancy_grid[r, c]:
        #                 pos = g.map2_visgrid((r, c))
        #                 g.drawEnv(win, pos, colors[7])
        #
        #     # put the particles back on top of the grid squares
        #     for p in crowd:
        #         p.graphic.undraw()
        #         p.unit_graphics(win)
        #
        #
        #     # Dijkstra's Algorithm
        #     dg = [] # values of policy
        #     ff = [] # direction vectors of policy
        #
        #     for every_goal in g.goal:
        #         dg.append(g.DijkstraGrid(every_goal))
        #     for every_dg in dg:
        #         ff.append(g.createFlowField(every_dg))
        #     for every_sign_goal in g.sign_goals.values():
        #         every_sign_goal.compute_dg_ff(g)
        #         dg.append(every_sign_goal.dg)
        #         ff.append(every_sign_goal.ff)
        #     for p in crowd:
        #         p.flow_field_index = None
        #
        #     g.visualize_dg(win, dg[0])
        #     g.visualize_flow(win, ff[0])

    if not multi_sim:
        end_t = time.time()
        print('crowd simulation real time: {0} seconds'.format(end_t - start_t))
        print('steps:', step)
        print('simulation time (sec)', step * dt)

        escapeKey = None
        while escapeKey != 'q':
            escapeKey = win.getKey()
        win.close()
    else:
        end_t = time.time()
        win.close()
        real_time = end_t - start_t  # seconds
        sim_time = step * dt  # seconds
        n_steps = step

        return real_time, sim_time, n_steps


""" 
**************************** Main ********************************************
"""

if __name__ == "__main__":
    # Change Me
    multi_sim = False
    crowd_size = 24
    ntests = 30

    test_cases = ['./map_close.txt', './map_far.txt', './map_sign.txt']
    # test_cases = ['./map_sign.txt']
    steps = 30000  # deprecated, sim runs until everyone is out

    real_time = []  # seconds
    sim_time = []  # seconds
    n_steps = []  # integer

    if multi_sim is False:
        vis_update = 1  # update as fast as possible
        fps = 60  # limit update to have smooth video or None to have updates ASAP
        map_path = test_cases[2]
        simulation(crowd_size, steps, map_path, vis_update, fps, obedience=100, multi_sim=multi_sim)
    else:
        for test_case in test_cases:
            print('*********** TEST CASE: {0} ***********'.format(test_case))
            if test_case == './map_sign.txt':  # if its the sign test case
                for obedience in range(0, 110, 10):  # 0 to 110 since python doesn't include end
                    print('*********** OBEDIENCE {0}% *********** '.format(obedience))
                    for ntest in range(ntests):
                        rt, st, ns = simulation(crowd_size, steps, test_case, obedience=obedience, multi_sim=multi_sim)
                        real_time.append(rt)
                        n_steps.append(ns)
                        sim_time.append(st)  # sim time is steps * dt

                    # # real time
                    # avg_rt = sum(real_time) / len(real_time)
                    # print('*** REAL TIME (s) ***')
                    # print(*real_time, sep=", ")
                    # print('AVG RT (s):', avg_rt)
                    # real_time = []
                    #
                    # # steps
                    # avg_step = sum(n_steps) / len(n_steps)
                    # print('*** STEPS ***')
                    # print(*n_steps, sep=", ")
                    # print('AVG STEPS:', avg_step)
                    # n_steps = []

                    # sim time
                    avg_st = sum(sim_time) / len(sim_time)
                    print('*** SIMULATION TIME (s) ***')
                    for i in sim_time:
                        print("%.2f" % i)
                    print('AVG SIM: %.2f' % avg_st)
                    sim_time = []
            else:
                for ntest in range(ntests):
                    rt, st, ns = simulation(crowd_size, steps, test_case, multi_sim=multi_sim)
                    real_time.append(rt)
                    n_steps.append(ns)
                    sim_time.append(st)  # sim time is steps * dt

                # sim time
                avg_st = sum(sim_time) / len(sim_time)
                print('*** SIMULATION TIME (s) ***')
                for i in sim_time:
                    print("%.2f" % i)
                print('AVG SIM: %.2f' % avg_st)
                sim_time = []