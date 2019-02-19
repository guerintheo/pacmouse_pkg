import numpy as np
import params as p

def motion_model(x, u, dt):
    """
    Given the previous state (x) and control input (u), calculate the
    predicted change in state at the next time step.
    
    x: previous state
    u: control input; NumPy array containing [angular_velocity_left_motor,
       angular_velocity_right_motor]
    dt: time step
    """
    # From robot kinematics. See LaTeX document for this math
    v = p.wheel_radius/2.0*((u[0]+u[1])/p.gear_ratio)
    psi_dot = (p.wheel_radius*p.wheel_dist_y)/(2*(p.wheel_dist_x**2 + p.wheel_dist_y**2))*((u[1]-u[0])/p.gear_ratio)
    # TODO: Handle the psi_dot = 0 case perhaps a bit better, to avoid division by zero
    if psi_dot == 0:
        psi_dot = 0.00001
    
    psi = x[2]
    a = x[4]
    
    dpsi = psi_dot*dt
    dv = a*dt
    
    dx = 1.0/(psi_dot**2.0)*((v+dv)*psi_dot*np.sin(psi + dpsi) - v*psi_dot*np.sin(psi) + a*np.cos(psi + dpsi) - a*np.cos(psi))
    dy = 1.0/(psi_dot**2.0)*((-v-dv)*psi_dot*np.cos(psi + dpsi) + v*psi_dot*np.cos(psi) + a*np.sin(psi + dpsi) - a*np.sin(psi))
    
    change_in_state = np.array([dx, dy, dpsi, dv, 0.0, 0.0])
    return change_in_state