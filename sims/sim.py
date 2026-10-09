import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dataclasses import dataclass
import numpy as np

#lines above ^ import relevant modules and packages

@dataclass
class State:
    xvel:float
    xpos:float
    ypos:float
    time:float
    latvel:float
    dragvel:float
    motvel:float
    brakevel:float

#resolution of the simulation
time_step = 0.1

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

    #drag physical facts
    air_density = 1.2
    cross_sectional_area = 1.2
    drag_coefficient = 1.7
    drag_acceleration = 0
    rolling_resistance_coefficient = 0.015

    #motor model physical facts
    max_propulsion = 2000
    max_xvel = 27
    motor_driver_input = 0

    #braking physical facts
    maximum_braking_capacity = 1850

    #driver slams brakes after 2 seconds
    if state.time > 2:
        braking_input = 1

    #before 2 seconds, there are no brakes
    else:
        braking_input = 0
        state.brakevel = 25

    #pressing motor acceleration pedal over 3 seconds
    if state.time <= 3:
        motor_driver_input = state.time/3

    #holding the pedal for 20 seconds
    elif state.time <=  3 + 20:
        motor_driver_input = 1

    #accelerating car at 5m/s^2 for 10 seconds
    if state.time <= 10:
        drag_acceleration = 5

    #else:
    #    drag_acceleration = 0

    #turning steering wheel 5 degrees over 3 seconds
    if state.time <= 3:
        steer_angle = state.time * 5 / 3

    #holding steering wheel for another 7 seconds
    elif state.time <= 3 + 7:
        steer_angle = 5

    #steering wheel is released
    else:
        steer_angle = 0

    #driver pressing the pedal from 0 to 1 within 7 seconds
    if state.time <= 7:
        driver_input = state.time/7

    #pedal fully pressed until 22 seconds
    elif state.time <= 22:
        driver_input = 1.0

    #driver stops pressing pedal after 22 seconds
    else:
        driver_input = 0.0

    #Throttle physics equations
    command_torque = max_torque * driver_input
    force_at_wheels = (command_torque * gear_ratio)/radius
    acceleration = force_at_wheels/mass

    new_vel = state.xvel + acceleration * time_step
    new_xpos = state.xpos + state.xvel * time_step
    new_time = state.time + time_step

    #Traction physics equations
    slip_angle = (steer_angle * (3.14/180)) - (state.latvel/forward_speed)
    lateral_force = cornering_stiffness * slip_angle
    lateral_acceleration = lateral_force/mass
    new_latvel = state.latvel + (lateral_acceleration * time_step)

    #Drag physics equations - Note: dragvel and accel are not vel/accel of drag, but vel/accel
    #for the drag simulation specifically
    drag = 0.5 * cross_sectional_area * drag_coefficient * air_density * state.dragvel ** 2
    rolling_resistance = 9.81 * rolling_resistance_coefficient
    drag_net_acceleration = drag_acceleration - (drag/mass)
    new_dragvel = state.dragvel + (drag_net_acceleration * time_step) - rolling_resistance

    #Motor model physics equation
    propulsion = max_propulsion * motor_driver_input * (1-(state.motvel/max_xvel))
    motor_acceleration = propulsion / mass
    new_motvel = state.motvel + (motor_acceleration * time_step)

    #braking physics equations
    braking_force = braking_input * maximum_braking_capacity
    braking_acceleration = braking_force/mass
    new_brakevel = state.brakevel - (braking_acceleration * time_step)

    #updates variables of the simulaton state
    newState = State(
        xvel = new_vel,
        xpos = new_xpos,
        ypos = 0,
        time = new_time,
        latvel = new_latvel,
        dragvel = new_dragvel,
        motvel = new_motvel,
        brakevel = new_brakevel
    )

    return newState

#initializes car state as x0 at the origin point in space-time
x0 = State(
    xpos=0,
    ypos=0,
    xvel=0,
    time=0,
    latvel=0,
    dragvel=0,
    motvel=0,
    brakevel=0
)

#defines function which animates the car step by step through the graph
def animate (i):
    global x0
    x0 = step(x0)
    if x0.xpos < 300:
        throttle.set_offsets([[x0.xpos, x0.ypos]])
    #latforce.set_offsets([x0.time, x0.latvel])
    if x0.time < 15:
        ax1.plot(x0.time,x0.latvel, 'x', markeredgewidth=2)
    if (x0.dragvel >= 0.1):
    #    vel_with_drag.set_offsets([x0.time,x0.dragvel])
        ax2.plot(x0.time,x0.dragvel, 'x', markeredgewidth=2)
    #motor_velocity.set_offsets([x0.time,x0.motvel])
    ax3.plot(x0.time, x0.motvel, 'x', markeredgewidth=2)
    if (x0.brakevel >= 0):
    #    brake_velocity.set_offsets([x0.time,x0.brakevel])
        ax4.plot(x0.time,x0.brakevel, 'x', markeredgewidth=2)

#Not sure what below return still does anymore?  Artifacts from previous versions
    return ax, ax1, ax2, ax3, ax4


#draws the fixed simulation graph
fig = plt.figure(figsize=(3,3), dpi=80)
ax = fig.add_subplot(151)
ax.grid()
ax.set_xlim(0,300)
ax.set_ylim(0,10)
ax.set_xlabel("Car Position")
throttle = ax.scatter([x0.xpos],[x0.ypos], s = 200, c = 'pink', marker = 's')


#making traction graph
ax1 = fig.add_subplot(152)
ax1.grid()
ax1.set_xlim(0,15)
ax1.set_ylim(0,50)
ax1.set_xlabel("Lateral Vel / T")
#latforce = ax1.scatter([x0.time],[x0.latvel], s = 200, c = 'red', marker = 'D')
ax1.plot(x0.time,x0.latvel, 'x', markeredgewidth=2)

#making drag graph.  Note: still need to end simulation when dragvel = 0.1
ax2 = fig.add_subplot(153)
ax2.grid()
ax2.set_xlim(0,50)
ax2.set_ylim(0,50)
ax2.set_xlabel("Vel w/ Drag & Roll Resist. / T")
#vel_with_drag = ax2.scatter([x0.time],[x0.dragvel], s = 200, c = 'blue', marker = 'o')
ax2.plot(x0.time,x0.dragvel, 'x', markeredgewidth=2)

#making motor velocity graph.
ax3 = fig.add_subplot(154)
ax3.grid()
ax3.set_xlim(0,50)
ax3.set_ylim(0,50)
ax3.set_xlabel("Vel w/ Motor / T")
#motor_velocity = ax3.scatter([x0.time],[x0.motvel], s = 200, c = 'green', marker = 'o')
ax3.plot(x0.time, x0.motvel, 'x', markeredgewidth=2)

#making brake velocity graph
ax4 = fig.add_subplot(155)
ax4.grid()
ax4.set_xlim(0,15)
ax4.set_ylim(0,50)
ax4.set_xlabel("Vel w/ Brakes / T")
#brake_velocity = ax4.scatter([x0.time],[x0.brakevel], s = 200, c = 'green', marker = 's')
ax4.plot(x0.time,x0.brakevel, 'x', markeredgewidth=2)

# these lines are so the animation doesnt zoom in or out
plt.pause(10)
ani = animation.FuncAnimation(fig, animate, interval=10, frames = 10000)
plt.show()