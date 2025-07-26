'''This is an example for modeling
Cart-pole example
learn from https://tinympc.org/get-started/model/
'''

import autograd as AG
import autograd.numpy as np
mc = 0.2 # mass of the cart [kg]
mp = 0.1 # mass of the pole [kg]
l = 0.5  # distance to the center of mass [m]
g=9.81

def cartpole_dynamics(x,u):
    '''
    This function describes the continuous (nonlinear) dynamics of our system, i.e. 
    dx=f(x,u). 
    Before linearizing, we first discretize our continuous dynamics with an integrator. 
    You can write down whatever you like.
    '''
    r = x[0]
    theta = x[1]
    rd = x[2]
    theta_d =x[3]
    F = u[0]
    
    theta_dd=(g*np.sin(theta)+np.cos(theta)*((-F - mp*l*(theta_d**2)*\
             np.sin(theta))/(mc + mp)))/(l*(4/3-(mp*(np.cos(theta)**2))/(mc + mp)))
    rdd=(F + mp*l*((theta_d**2)*np.sin(theta) - theta_dd*np.cos(theta)))/(mc + mp)
    
    return np.array([rd, theta_d, rdd, theta_dd])

def cartpole_rk4(x,u,dt):
    f1=dt*cartpole_dynamics(x,u)
    f2=dt*cartpole_dynamics(x+f1/2,u)
    f3=dt*cartpole_dynamics(x+f2/2,u)
    f4=dt*cartpole_dynamics(x+f3,u)
    return x+(1/6)*(f1+2*f2+2*f3+f4)


xgoal = np.array([0.0, np.pi, 0.0, 0.0])
ugoal = np.array([0.0])

dt = 0.01

A = AG.jacobian(lambda x_: cartpole_rk4(x_, ugoal, dt))(xgoal)
B = AG.jacobian(lambda u_: cartpole_rk4(xgoal, u_, dt))(ugoal)

print(f"A:\n{A}")
print(f"B:\n{B}")