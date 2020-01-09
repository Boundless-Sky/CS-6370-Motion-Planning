"""
Crowd Simulation - graphics_routines.py
11/23/2019
Motion planning final project: Dynamic Exit Signs
Justin Ngo, Nathan Collins, Greg Berube

This file holds functions that work with the graphics library
to render the motion planning problem to the screen.
"""

import graphics  # uses John Zelle's python graphics library

""" 
**************************** graphics *****************************************
"""


def init_graphics(winsize):
    """

    :param winsize: integer
    :return: window to draw in
    """
    # TODO: scale the size of particle to window size
    # Canvas Initialization
    window = graphics.GraphWin('Dynamic Exit Signs', winsize, winsize, autoflush=False)
    window.setBackground('white')
    window.setCoords(0, 0, winsize, winsize)  # origin at bottom left

    # Timestep Initialization
    #    text = graphics.Text(graphics.Point(winsize // 2, 10), 'step = 0')
    #    text.setSize(10)
    #    text.draw(window)

    #    return window, text
    return window


def update_step(win, step, update_interval, fps=None, text=None):
    """

    :param win: window to draw in
    :param step: how many times to run
    :param update_interval: how quick to update window being drawn. 1 for fast
    :param text: variable text to screen
    :return:
    """
    if update_interval and step % update_interval == 0:
        if text != None:
            format_str = 'step = {0}'
            text.setText(format_str.format(step))
        if fps is None:
            # win.update()
            graphics.update()  # optional argument to throttle at x frames per second
        else:
            graphics.update(fps)  # optional argument to throttle at x frames per second
