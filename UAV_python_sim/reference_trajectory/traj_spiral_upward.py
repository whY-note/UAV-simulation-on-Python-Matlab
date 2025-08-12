# import scipy

# data = scipy.io.loadmat('param.mat')

# XRef=data['XRef']
# print(XRef.shape)
# print(type(XRef))

# import numpy as np
# import matplotlib.pyplot as plt

# # ====== 参数配置 ======
# duration = 20.0         # 总时长（秒）
# time_step = 0.01         # 时间步长（秒）
# radius = 1.0            # 半径（米）
# target_height = 5.0     # 目标飞行高度（米）
# omega=1


# # ====== 生成时间序列 ======
# t = np.arange(0, duration, time_step)

# # ====== 轨迹方程 ======
# x=radius*np.sin(omega*t)
# y=radius*np.cos(omega*t)-1
# z=target_height/duration*t

# vx=radius*omega*np.cos(omega*t)*np.ones(t.shape)
# vy=-radius*omega*np.sin(omega*t)*np.ones(t.shape)
# vz=target_height/duration*np.ones(t.shape)

# roll=np.zeros(t.shape)
# pitch=np.zeros(t.shape)
# yaw= np.pi/4*np.ones(t.shape)

# p=np.zeros(t.shape)
# q=np.zeros(t.shape)
# r=np.zeros(t.shape)

# # ====== 组合状态量 ======
# # 状态量顺序：[x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
# states = np.vstack([x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]).T
# print(states.shape)
# # ====== 可视化 ======
# plt.figure(figsize=(14, 10))

# # 3D轨迹
# ax1 = plt.subplot(221, projection='3d')
# ax1.plot(x, y, z, label='Flight Path')
# ax1.scatter(x[0], y[0], z[0], c='r', s=50, label='Start (Ground)')
# ax1.set_xlabel('X (m)')
# ax1.set_ylabel('Y (m)')
# ax1.set_zlabel('Z (m)')
# ax1.set_title('3D Trajectory from Ground')
# ax1.legend()

# # 高度变化
# plt.subplot(222)
# plt.plot(t, z, label='Z Position')
# plt.plot(t, vz, label='Vertical Speed (Vz)')
# plt.xlabel('Time (s)')
# plt.ylabel('Height (m) / Speed (m/s)')
# plt.title('Vertical Motion (Takeoff)')
# plt.legend()

# # 水平轨迹
# plt.subplot(223)
# plt.plot(x, y, label='Horizontal Path')
# plt.scatter(x[0], y[0], c='r', s=50, label='Start')
# plt.xlabel('X (m)')
# plt.ylabel('Y (m)')
# plt.title('Horizontal Circle Trajectory')
# plt.axis('equal')
# plt.legend()

# # 姿态角
# plt.subplot(224)
# plt.plot(t, np.degrees(yaw), label='Yaw')
# plt.plot(t, np.degrees(roll), label='Roll')
# plt.plot(t, np.degrees(pitch), label='Pitch')
# plt.xlabel('Time (s)')
# plt.ylabel('Angle (deg)')
# plt.title('Attitude Angles')
# plt.legend()

# plt.tight_layout()
# plt.show()

'''=============================生成轨迹, 写入txt文件=============================='''
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_trajectory(trajectory):
    """绘制3D轨迹图 和 2D俯视图"""
    fig = plt.figure(figsize=(12, 8))
    
    # 3D轨迹
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2], 'b-', linewidth=1)
    ax1.scatter(0, 0, 0, c='r', s=100, label='Start (0,0,0)')
    # ax1.scatter(0, -1, 0, c='g', s=100, label='Circle Center (0, -1 ,0)')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_zlabel('Z Position (m)')
    ax1.set_title('3D Trajectory')
    ax1.legend()
    ax1.grid(True)
    
    # 2D俯视图（验证圆方程）
    ax2 = fig.add_subplot(122)
    ax2.plot(trajectory[:,0], trajectory[:,1], 'b-', linewidth=1)
    ax2.add_patch(plt.Circle((0,-1), 1, fill=False, color='r', linestyle='--', label='Target Circle'))
    ax2.scatter(0, 0, c='r', s=100, label='Start (0,0)')
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Y Position (m)')
    ax2.set_title('2D Top View\n')
    ax2.axis('equal')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()

    plt.show()


# 参数设置

total_time = 20.0          # 总时间（秒）
time_step = 0.01  # 时间步长[s]
num_points = int(total_time/time_step)          # 总数据点数
target_height = 5

# 螺旋参数
spiral_radius = 1.0       # 螺旋半径（米）
vz = target_height/total_time       # 螺旋上升速率
angular_vel = 3  # 角速度
centre_x= 0
centre_y= -1

# 初始化轨迹数组
trajectory = np.zeros((num_points, 12))  # 12个状态变量

for i in range(num_points):
    t = i * time_step
    
    # 位置 (x, y, z)
    theta = angular_vel * t
    trajectory[i, 0] = centre_x + spiral_radius * math.sin(theta)  # x
    trajectory[i, 1] = centre_y + spiral_radius * math.cos(theta)  # y
    trajectory[i, 2] = vz * t                # z
    # trajectory[i, 0] = centre_x + math.sin(0.03*i)  # x
    # trajectory[i, 1] = centre_y + math.cos(0.03*i)  # y
    # trajectory[i, 2] = 0.2*i              # z
    
    # 速度 (dx, dy, dz)
    trajectory[i, 3] =  spiral_radius * math.cos(theta) * angular_vel  # dx
    trajectory[i, 4] =  -spiral_radius * math.sin(theta) * angular_vel  # dy
    trajectory[i, 5] =  vz                                  # dz
    
    # 欧拉角 (roll, pitch, yaw) 
    trajectory[i, 6] = 0.0                     # roll
    trajectory[i, 7] = 0.0                     # pitch
    trajectory[i, 8] = np.pi/4 #theta                    # yaw
    
    # 欧拉角导数 (droll, dpitch, dyaw)
    trajectory[i, 9] = 0.0                     # droll
    trajectory[i, 10] = 0.0                    # dpitch
    trajectory[i, 11] = 0 #angular_vel            # dyaw

# 写入TXT文件
with open('spiral_trajectory.txt', 'w') as f:
    # 写入表头
    f.write("# x(m) y(m) z(m) dx(m/s) dy(m/s) dz(m/s) roll(rad) pitch(rad) yaw(rad) droll(rad/s) dpitch(rad/s) dyaw(rad/s)\n")
    
    # 写入数据
    for point in trajectory:
        line = ', '.join(['%.6f' % val for val in point])
        f.write(line + ',\n')

print("轨迹已生成并保存到 spiral_trajectory.txt")
plot_trajectory(trajectory)

XRef=trajectory