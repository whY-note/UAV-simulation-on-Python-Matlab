'''
learn from https://tinympc.org/get-started/model/
'''

import autograd as AG
import autograd.numpy as np

m = 1.05 # mass of the UAV [kg]
g = 9.81

Ixx=6.433e-3; # 转动惯量 x 轴 kg*m^2
Iyy=6.433e-3; # 转动惯量 y 轴 kg*m^2
Izz=1.182e-2; # 转动惯量 z 轴 kg*m^2


def UAV_dynamics(x,u):
    '''
    dx=f(x,u). 
    状态向量是12维 x = [px, py, pz,  # 位置(0,1,2)
                    vx, vy, vz,  # 速度(3,4,5)
                    φ, θ, ψ,     # 角度(6,7,8)
                    ωx, ωy, ωz   # 角速度(9,10,11)
                    ]
   控制向量是4维  u = [thrust,    # 推力
                    tau_x,     # 横滚力矩
                    tau_y,     # 俯仰力矩
                    tau_z      # 偏航力矩
                    ]
    '''
    T=u[0]
    tao_x=u[1]
    tao_y=u[2]
    tao_z=u[3]
    
    dx=x[3]
    dy=x[4]
    dz=x[5]
    
    ddx = 1/m*T*(np.cos(x[6]) * np.sin(x[7]) * np.cos(x[8]) + np.sin(x[8]) * np.sin(x[6]))
    ddy = 1/m*T*(np.cos(x[6]) * np.sin(x[7]) * np.sin(x[8]) - np.cos(x[8]) * np.sin(x[6])) # todo: make sure x[6]?
    ddz = 1/m*T * np.cos(x[6]) * np.cos(x[7]) - g
    
    dphi=x[9] # roll
    dtheta=x[10] # pitch
    dpsi=x[11] # yaw
    
    ddphi = 1/Ixx*(tao_x - x[10]*x[11]*(Iyy-Izz))
    ddtheta = 1/Iyy*(tao_y - x[11]*x[9]*(Izz-Ixx))
    ddpsi = 1/Izz*(tao_z - x[9]*x[10]*(Ixx-Iyy))
    
    return np.array([dx, dy, dz, ddx, ddy, ddz, dphi, dtheta, dpsi, ddphi, ddtheta, ddpsi])

def UAV_rk4(x,u,dt):
    f1=dt*UAV_dynamics(x,u)
    f2=dt*UAV_dynamics(x+f1/2,u)
    f3=dt*UAV_dynamics(x+f2/2,u)
    f4=dt*UAV_dynamics(x+f3,u)
    return x+(1/6)*(f1+2*f2+2*f3+f4)


xgoal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
ugoal = np.array([m*g, 0.0, 0.0, 0.0])

dt = 0.01

A = AG.jacobian(lambda x_: UAV_rk4(x_, ugoal, dt))(xgoal)
B = AG.jacobian(lambda u_: UAV_rk4(xgoal, u_, dt))(ugoal)

print(f"A:\n{A}")
print(f"B:\n{B}")

np.savetxt('modeling/A_matrix.txt', A, fmt='%.6f', delimiter=', ')
np.savetxt('modeling/B_matrix.txt', B, fmt='%.6f', delimiter=', ')