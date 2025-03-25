import math

## Define constant
L = 2.5  # meters
w = 1.5  # meters

def ackermann_control(R, speed):
    # Compute turning angle
    delta = math.atan(L / R)

    # Compute radii for inner and outer rear wheels
    R_ri = abs(R - w / 2)
    R_ro = abs(R + w / 2)

    # Compute radii for inner and outer front wheels
    R_fi = math.sqrt(R_ri*R_ri+L*L)
    R_fo = math.sqrt(R_ro*R_ro+L*L)

    # Compute wheel speeds
    v_ri = speed * (R_ri / abs(R))
    v_ro = speed * (R_ro / abs(R))

    v_fi = speed * (R_fi / abs(R))
    v_fo = speed * (R_fo / abs(R))

    return delta, v_ri, v_ro, v_fi, v_fo

speed = 2.0  # m/s
traj = [2, 3, 4]

for radius in traj:

    delta, v_rleft, v_rright, v_fleft, v_fright = ackermann_control(radius, speed)
    print(f"Delta: {delta}, Left rear wheel speed: {v_rleft} m/s, Right rear wheel speed: {v_rright} m/s, Left front wheel speed: {v_fleft} m/s, Right front wheel speed: {v_fright} m/s")
