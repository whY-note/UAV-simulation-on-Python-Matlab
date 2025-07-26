'''在tinympc.py 的基础上进行封装'''

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# # tinympc的无人机 的参数与轨迹
# # 把N改为5
# from UAV_tinympc_500Hz.parameters.param_ABQR_500Hz import A,B,Q,R
# from UAV_tinympc_500Hz.reference_trajectory.traj_fig8 import XRef

# 选择参数
# from UAV_m105.parameters.param_ABQR_m105_50Hz import A,B,Q,R
from UAV_m105.parameters.param_ABQR_m105_100Hz import A,B,Q,R

# 选择轨迹
from UAV_Trajectory.traj_8 import XRef  # 8字型轨迹
# from UAV_Trajectory.hover_and_traj_8 import XRef # 先竖直上升，然后8字型轨迹
# from UAV_Trajectory.traj_8_song import XRef # 宋博的8字型轨迹

# from UAV_Trajectory.traj_spiral_upward import XRef
# from UAV_Trajectory.traj_hover1 import XRef

# 参数初始化
N = 30
Nu = 4
Nx = 12

mass=1.05

# Riccati方程相关变量初始化
class TinyMPC:
    def __init__(self,A,B,Q,R,XRef):
        self.A=A
        self.B=B
        
        self.Kk = np.zeros((Nu, Nx, N))
        self.Pk = np.zeros((Nx, Nx, N+1))
        self.dk = np.zeros((Nu, N))
        self.pk = np.zeros((Nx, N+1))

        # 线性成本参数
        self.rk = np.zeros((Nu, N))
        self.qk = np.zeros((Nx, N+1))

        # 松弛变量
        self.zk = np.zeros((Nx, N+1))
        self.vk = np.zeros((Nu, N))
        self.zk_new = np.zeros((Nx, N+1))
        self.vk_new = np.zeros((Nu, N))

        # 对偶变量
        self.yk = np.zeros((Nx, N+1))
        self.gk = np.zeros((Nu, N))

        # 状态和输入变量
        self.xk = np.zeros((Nx, N+1))
        self.uk = np.zeros((Nu, N))

        # 约束条件
        self.xmax_vec = np.array([20,20,200,200,200,200,200,200,200,200,200,200])
        self.xmin_vec = -self.xmax_vec
        self.xmax = np.tile(self.xmax_vec.reshape(-1,1), (1, N+1))
        self.xmin = np.tile(self.xmin_vec.reshape(-1,1), (1, N+1))

        self.umax_vec = np.array([200,200,200,200])
        self.umin_vec = -self.umax_vec

        # # 电机转速限制，转化为u的限制
        # self.umax_vec = np.array([32,1.87,1.87,0.2079])
        # self.umin_vec = np.array([0,-1.87,-1.87,-0.2079])

        # self.umax_vec = np.array([32,1.0,1.0,0.2079])
        # self.umin_vec = np.array([0,-1.0,-1.0,-0.2079])
        self.umax = np.tile(self.umax_vec.reshape(-1,1), (1, N))
        self.umin = np.tile(self.umin_vec.reshape(-1,1), (1, N))

        self.XRef = XRef.T  # 转置以匹配MATLAB结构
        # print(self.XRef)

        self.max_iter = 100
        self.rho = 0.1

        self.k_steps = self.XRef.shape[1] - N - 2
        self.abs_pri_tol = 1e-3
        self.abs_dual_tol = 1e-3
        
        # 假设初值接近0，但不是0
        self.xk[:,0]=np.array([0, 0, 0.008, 0, 0, 0, 0, 0, 0, 0, 0, 0]).T
        # self.xk[:,0]=np.array([0.000015281127888,0.000000002675643,0.238336952171659,
        #                        0.00015161938543,-0.000000170073603,0.154748336713113,
        #                        0.000000026731694,0.000331273616864,-0.000001380604917,
        #                        -0.000018439794034,0.004420032250926,-0.00001407826788
        #                         ]).T

        # self.xk[:,0] = np.array([ 6.67234061e-14, -2.55803853e-05, -8.63969374e-04 , 5.24655329e-12,
        #                 2.92112142e-03, -8.98402531e-02 , 3.20250065e-04 , 3.79039217e-14,
        #                 -3.05471764e-12 ,-3.68491502e-02 , 2.26141700e-12, -9.49590632e-11],dtype=np.float32).T
        
        # # 用参考轨迹的第一个作为初值
        # self.xk[:, 0] = XRef[0,:]
        
        self.xk_history=[]
        self.uk_history=[]
        self.xk_before_solve_history=[]
        
        self.xk_calc = np.zeros((Nx, self.k_steps))
        self.uk_calc = np.zeros((Nu, self.k_steps))
        self.motor_speeds=np.zeros((4,self.k_steps))

        self.Q_rtile = Q + self.rho * np.eye(Nx)
        self.R_rtile = R + self.rho * np.eye(Nu)
        self.Pk[:, :, N] = self.Q_rtile
        self.iters = np.zeros(self.k_steps)

        # 反向传播计算,Kk和Pk
        for n in range(N-1, -1, -1):
            self.Kk[:, :, n] = np.linalg.inv(self.R_rtile + B.T @ self.Pk[:, :, n+1] @ B) @ (B.T @ self.Pk[:, :, n+1] @ A)
            self.Pk[:, :, n] = self.Q_rtile + self.Kk[:, :, n].T @ self.R_rtile @ self.Kk[:, :, n] + \
                        (A - B @ self.Kk[:, :, n]).T @ self.Pk[:, :, n+1] @ (A - B @ self.Kk[:, :, n])


    def solve(self,i):
        # self.xk_before_solve_history.append(self.xk[:,0])
        # print(f"xk_before_solve: {self.xk[:,0]}")
        A=self.A
        B=self.B
        
        xref = self.XRef[:, i:i+N+1]
        self.yk.fill(0)
        self.gk.fill(0)
        
        # j 是内循环次数
        for j in range(self.max_iter):
            # 前向传播
            for k in range(N):
                self.uk[:, k] = -self.Kk[:, :, k] @ self.xk[:, k] - self.dk[:, k]
                self.xk[:, k+1] = A @ self.xk[:, k] + B @ self.uk[:, k]
            
            # 更新松弛变量
            self.znew = np.clip(self.xk + self.yk, self.xmin, self.xmax)
            self.vnew = np.clip(self.uk + self.gk, self.umin, self.umax)
            
            # 更新对偶变量
            self.yk += self.xk - self.znew
            self.gk += self.uk - self.vnew
            
            # 更新线性成本参数
            self.rk = self.rho * (self.gk - self.vnew)
            self.qk = -self.Q_rtile @ xref + self.rho * (self.yk - self.znew)
            
            # 计算残差
            primal_residual_state = np.max(np.abs(self.xk - self.znew))
            dual_residual_state = self.rho * np.max(np.abs(self.zk - self.znew))
            primal_residual_input = np.max(np.abs(self.uk - self.vnew))
            dual_residual_input = self.rho * np.max(np.abs(self.vk - self.vnew))
            
            if (primal_residual_state < self.abs_pri_tol and 
                dual_residual_state < self.abs_dual_tol and
                primal_residual_input < self.abs_pri_tol and
                dual_residual_input < self.abs_dual_tol):
                self.xk = self.znew.copy()
                self.uk = self.vnew.copy()
                break
            
            self.vk = self.vnew.copy()
            self.zk = self.znew.copy()
            self.pk[:, N] = self.qk[:, N]
            
            # 反向传播更新
            for n in range(N-1, -1, -1):
                self.dk[:, n] = np.linalg.inv(self.R_rtile + B.T @ self.Pk[:, :, n+1] @ B) @ \
                        (B.T @ self.pk[:, n+1] + self.rk[:, n])
                self.pk[:, n] = self.qk[:, n] + (A - B @ self.Kk[:, :, n]).T @ \
                        (self.pk[:, n+1] - self.Pk[:, :, n+1] @ B @ self.dk[:, n]) + \
                        self.Kk[:, :, n].T @ (self.R_rtile @ self.dk[:, n] - self.rk[:, n])

        # 已经达到收敛要求 或者 达到最大迭代次数
        # 对u的第一项加回重力
        self.uk[0,:]+=mass*9.81 # 加回重力

        return j
    

    def main_loop(self):
        # 主控制循环

        for i in range(self.k_steps):
            iter_time=self.solve(i)
            print(f"{i}step: iter {iter_time}")
            
            self.iters[i] = iter_time
            
            # 记录当前的xk，uk，用于画图
            self.xk_calc[:, i] = self.xk[:, 0]
            self.xk_history.append(self.xk[:, 0])
            # print(f"xk: {self.xk_calc[:, i]}")
            
            self.uk_calc[:, i] = self.uk[:, 0]
            self.uk_history.append(self.uk[:, 0])
            # print(f"uk: {self.uk_calc[:,i]}")

            # 计算转速
            curr_motor_speeds=self.thrust_to_motor_speeds(self.uk[0, 0],self.uk[1, 0],self.uk[2, 0],self.uk[3, 0])
            self.motor_speeds[:,i]=np.array(curr_motor_speeds)

            # 更新xk0
            self.xk[:,0]=self.xk[:,1]
        
        df_uk_history=pd.DataFrame(self.uk_history)
        df_uk_history.to_excel("tinympc_class_data/uk_history.xlsx")
        df_xk_history=pd.DataFrame(self.xk_history)
        df_xk_history.to_excel("tinympc_class_data/xk_history.xlsx")
        # df_xk_before_solve_history=pd.DataFrame( self.xk_before_solve_history)
        # df_xk_before_solve_history.to_excel("tinympc_class_data/xk_before_solve_history.xlsx")

    
    def plot(self):
        k_steps=self.k_steps
        xk_calc=self.xk_calc
        uk_calc=self.uk_calc
        motor_speeds=self.motor_speeds
        iters=self.iters
        XRef=self.XRef
        
        # 绘图部分
        t = np.arange(k_steps)
        plt.figure()
        plt.plot(t, xk_calc[0, :], label="real x")
        plt.plot(t, XRef[0, :k_steps], label="ref x")
        plt.legend()

        plt.figure()
        plt.plot(t, xk_calc[1, :], label="real y")
        plt.plot(t, XRef[1, :k_steps], label="ref y")
        plt.legend()

        plt.figure()
        plt.plot(t, xk_calc[2, :], label="real z")
        plt.plot(t, XRef[2, :k_steps], label="ref z")
        plt.legend()
        
        plt.figure()
        plt.plot(t,xk_calc[6,:],label="real $\phi$")
        plt.plot(t,XRef[6,:k_steps],label="real $\phi$")
        plt.legend()
        
        plt.figure()
        plt.plot(t,xk_calc[7,:],label="real $\\theta$")
        plt.plot(t,XRef[7,:k_steps],label="real $\\theta$")
        plt.legend()
        
        plt.figure()
        plt.plot(t, xk_calc[8, :], label="real yaw")  # Python索引从0开始，对应MATLAB第9个元素
        plt.plot(t, XRef[8, :k_steps], label="ref yaw")
        plt.legend()

        delta_x = xk_calc - XRef[:, :k_steps]
        plt.figure()
        plt.plot(t,delta_x[0,:k_steps],label="delta x")
        plt.plot(t,delta_x[1,:k_steps],label="delta y")
        plt.plot(t,delta_x[2,:k_steps],label="delta z")
        plt.legend()

        plt.figure()
        plt.plot(t,uk_calc[0,:],label="T")
        plt.plot(t,uk_calc[1,:],label="$tau_x$")
        plt.plot(t,uk_calc[2,:],label="$tau_y$")
        plt.plot(t,uk_calc[3,:],label="$tau_z$")
        plt.legend()
        
        plt.figure()
        plt.plot(t,motor_speeds[0,:],label='w1')
        plt.plot(t,motor_speeds[1,:],label='w2')
        plt.plot(t,motor_speeds[2,:],label='w3')
        plt.plot(t,motor_speeds[3,:],label='w4')
        plt.legend()

        plt.figure()
        plt.plot(t,iters,label="iters")
        plt.legend()
    
    def plot_trajectory(self):
        '''画轨迹和参考轨迹'''
        xk_calc=self.xk_calc.T
        x_ref=self.XRef.T
        fig=plt.figure(figsize=(10,8))
        
        ax1 = fig.add_subplot(111, projection='3d')
        ax1.plot(xk_calc[:,0], xk_calc[:,1], xk_calc[:,2],
                 'b-', linewidth=1,
                 label="real"
                 )
        # ax1.scatter(0, 0, 0, c='r', s=100, label='Start (0,0,0)')
        # ax2 = fig.add_subplot(122,projection='3d')
        ax1.plot(x_ref[:,0], x_ref[:,1], x_ref[:,2],
                 color='r', linestyle='--', linewidth=1,
                 label="reference"
                 )

        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_zlabel('Z Position (m)')
        plt.legend()
        plt.show()
        
        

        
    def thrust_to_motor_speeds(self, thrust, tau_x, tau_y, tau_z):
        '''
        X字型无人机:
        3   1
          X
        2   4
        '''
        self.Ct = 8.297e-6                   # 单桨推力系数Ct=motor_constant [N/(rad/s)^2]
        self.Cm = 1.076e-7                   # 单桨力矩系数Cm=Ct*moment_constant [N·m/(rad/s)^2]
        self.d =0.165                        # 机身半径 d [m]
        self.max_rpm = 983.84      # 电机最大转速 [RPM]
        # 混控参数配置
        Ct = self.Ct    # 单桨推力系数Ct [N/(rad/s)^2]
        Cm = self.Cm    # 单桨力矩系数Cm [N·m/(rad/s)^2]
        d = self.d              # 机身半径 d [m]
        max_rpm = self.max_rpm       # 电机最大转速 [RPM]

        cc=np.sin(np.pi/4)

        # 定义混控矩阵
        #书上
        mix_matrix = np.array([ 
            [Ct ,  Ct,  Ct,  Ct],  
            [-cc*d*Ct,  cc*d*Ct,  cc*d*Ct, -cc*d*Ct],  
            [cc*d*Ct, -cc*d*Ct,  cc*d*Ct,  -cc*d*Ct],   
            [ Cm, Cm,  -Cm, -Cm] 
        ])
        # mix_matrix = np.zeros((4,4))
        # mix_matrix[0,:]=Ct

        # 计算电机转速
        u= np.array([
            thrust,    # 推力项
            tau_x,     # 横滚力矩
            tau_y,     # 俯仰力矩
            tau_z      # 偏航力矩
        ])
        
        motor_speeds_squared=np.linalg.pinv(mix_matrix) @ u # 求转速平方
        # 确保转速平方非负 (电机不可反转)
        motor_speeds_squared = np.maximum(motor_speeds_squared, 0) 

        # 计算实际转速 [rad/s]
        motor_speeds=np.sqrt(motor_speeds_squared)

        # max_rpm的单位转换：从RPM转换为rad/s 
        max_speed = max_rpm * 2*np.pi/60  # RPM转rad/s: 1RPM = 2π/60 rad/s

        # 应用转速限制
        motor_speeds = np.clip(motor_speeds,0, max_speed)

        # 返回4个电机的转速
        # 与仿真中的实际电机顺序相对应
        return [motor_speeds[2], motor_speeds[1], 
                motor_speeds[3], motor_speeds[0]] 

if __name__=="__main__":

    mpc=TinyMPC(A,B,Q,R,XRef)
    mpc.main_loop()
    mpc.plot()
    mpc.plot_trajectory()
    plt.axis('equal')
    plt.show()