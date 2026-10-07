import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np

@dataclass
class State:
    xvel:float
    xpos:float
    ypos:float
    time:float

time_step = 0.01

def step (state:State) -> State:
    mass = 100
    radius = 0.216
    gear_ratio = 3
    max_torque = 180

    if state.time <= 7:
        driver_input = state.time/7

    elif state.time <= 22:
        driver_input = 1.0

    else:
        driver_input = 0.0

    command_torque = max_torque * driver_input
    force_at_wheels = (command_torque * gear_ratio)/radius
    acceleration = force_at_wheels/mass

    new_vel = state.xvel + acceleration * time_step
    new_xpos = state.xpos + state.xvel * time_step
    new_time = state.time + time_step

    newState = State{
        xvel = new_vel,
        xpos = new_xpos,
        ypos = 0,
        time = new_time
    }

    return newState

def animate (i):
    global x0
    so = step(x0)
    ax.clear()
    ax.scatter([x0.xpo],[x.ypos], s = 200, c = 'pink', marker = 's')
    ax.set_xlim(0,300)
    ax.set_ylim(0,10)
    return ax



fig = plt.figure(figsize=(3,3), dpi=150)
ax = fig.add_subplot(111)
ax.grid()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
# these lines are so the animation doesnt zoom in or out
plt.pause(3)
ani = animation.FuncAnimation(fig, animate, interval=0)
plt.show()