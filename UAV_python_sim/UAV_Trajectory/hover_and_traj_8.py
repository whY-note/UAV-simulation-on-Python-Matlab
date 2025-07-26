import numpy as np
import matplotlib.pyplot as plt

# ====== 参数配置 ======
duration = 80.0         # 总时长（秒）
time_step = 0.02         # 时间步长（秒）
radius = 1.0            # 8字形半径（米）
target_height = 2.0     # 目标飞行高度（米）
climb_time = 5.0        # 爬升到目标高度的时间（秒）
omega = 0.5             # 角速度（控制轨迹速度）

num_points = int( duration/time_step )
climb_num_points = int( climb_time/time_step )

# 初始化轨迹数组
trajectory = np.zeros((num_points, 12))  # 12个状态变量

for i in range(num_points):
    if i<climb_num_points:
        # 悬停轨迹
        trajectory[i,2]=target_height
        
    else:
        # 8字型轨迹
        
        # 画 8 字型的时间
        t_fig8=time_step*i-climb_time
        
        # --- 水平方向（X-Y平面） ---
        trajectory[i,0]= radius * np.sin(omega * t_fig8)  # X轴位置
        trajectory[i,1]= 0.5 * radius * np.sin(2 * omega * t_fig8)     # Y轴位置

        vx = radius * omega * np.cos(omega * t_fig8)      # X方向速度
        vy = 0.5 * radius * 2 * omega * np.cos(2 * omega * t_fig8)  # Y方向速度
        trajectory[i,3] = vx
        trajectory[i,4] = vy
        
        # Z方向
        trajectory[i,2]=target_height
        
        trajectory[i,8] =np.pi/4  #yaw

# 可选
trajectory[:,3:]=0
    
    # ====== 可视化 ======
plt.figure(figsize=(14, 10))

x=trajectory[:,0]
y=trajectory[:,1]
z=trajectory[:,2]
vx=trajectory[:,3]
vy=trajectory[:,4]
vz=trajectory[:,5]
roll=trajectory[:,6]
pitch=trajectory[:,7]
yaw=trajectory[:,8]

t = np.linspace(0,duration, num_points)

# 3D轨迹
ax1 = plt.subplot(221, projection='3d')
ax1.plot(x, y, z, label='Flight Path')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Trajectory from Ground')
ax1.legend()

# 高度变化
plt.subplot(222)
plt.plot(t, z, label='Z Position')
plt.plot(t, vz, label='Vertical Speed (Vz)')
plt.xlabel('Time (s)')
plt.ylabel('Height (m) / Speed (m/s)')
plt.title('Vertical Motion (Takeoff)')
plt.legend()

# 水平轨迹
plt.subplot(223)
plt.plot(x, y, label='Horizontal Path')
plt.scatter(x[0], y[0], c='r', s=50, label='Start')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Horizontal 8-Shaped Trajectory')
plt.axis('equal')
plt.legend()

# 姿态角
plt.subplot(224)
plt.plot(t, np.degrees(yaw), label='Yaw')
plt.plot(t, np.degrees(roll), label='Roll')
plt.plot(t, np.degrees(pitch), label='Pitch')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Attitude Angles')
plt.legend()

plt.tight_layout()
plt.show()

with open('hover_fig8_trajectory.txt', 'w') as f:
    # 写入表头
    f.write("# x(m) y(m) z(m) dx(m/s) dy(m/s) dz(m/s) roll(rad) pitch(rad) yaw(rad) droll(rad/s) dpitch(rad/s) dyaw(rad/s)\n")
    
    # 写入数据
    for point in trajectory:
        line = ', '.join(['%.6f' % val for val in point])
        f.write(line + ',\n')

XRef= trajectory