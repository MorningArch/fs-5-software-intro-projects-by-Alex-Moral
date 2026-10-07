import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np

#lines above ^ import relevant modules and packages

#must research where to initialize latvel variable
@dataclass
class State:
    xvel:float
    latvel:float
    xpos:float
    ypos:float
    time:float

#resolution of the simulation
time_step = 0.01

#defines step function to move simulation through time
def step (state:State) -> State:
    #important physical facts
    mass = 300
    radius = 0.216
    gear_ratio = 3
    max_torque = 180

    #initializing dynamic physical facts
    steer_angle = 0
    forward_speed = 15
    cornering_stiffness = 36000

    #driver pressing the pedal from 0 to 1 within 7 seconds
    if state.time <= 7:
        driver_input = state.time/7

    #pedal fully pressed until 22 seconds
    elif state.time <= 22:
        driver_input = 1.0

    #driver stops pressing pedal after 22 seconds
    else:
        driver_input = 0.0

    #part 1 physics equations
    command_torque = max_torque * driver_input
    force_at_wheels = (command_torque * gear_ratio)/radius
    acceleration = force_at_wheels/mass

    new_vel = state.xvel + acceleration * time_step
    new_xpos = state.xpos + state.xvel * time_step
    new_time = state.time + time_step

    #part 2 physics equations
    slip_angle = steer_angle - (latvel/forward_speed)
    lateral_force = cornering_stiffness * slip_angle
    lateral_acceleration = lateral_force/mass
    new_latvel = latvel + (lateral_acceleration * time_step)

    #updates variables of the simulaton state
    newState = State(
        xvel = new_vel,
        latvel = new_latvel,
        xpos = new_xpos,
        ypos = 0,
        time = new_time
    )

    return newState

#initializes car state as x0 at the origin point in space-time
x0 = State(
    xpos=0,
    ypos=0,
    xvel=0,
    time=0
)

#defines function which animates the car step by step through the graph
def animate (i):
    global x0
    x0 = step(x0)
    ax.clear()
    ax.scatter([x0.xpos],[x0.ypos], s = 200, c = 'pink', marker = 's')
    ax.set_xlim(0,300)
    ax.set_ylim(0,10)
    return ax


#draws the fixed simulation graph
fig = plt.figure(figsize=(3,3), dpi=150)
ax = fig.add_subplot(111)
ax.grid()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
# these lines are so the animation doesnt zoom in or out
plt.pause(3)
ani = animation.FuncAnimation(fig, animate, interval=0)
plt.show()