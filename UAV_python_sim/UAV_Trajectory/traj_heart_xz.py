import numpy as np
import matplotlib.pyplot as plt

# # 参数设置
# x0 , z0 =0,0 # 起点
# s = 0.08  # 缩放因子
# x_center, z_center = 0 + x0,  17*s + z0 # 中心点坐标
# theta = np.linspace(-np.pi, np.pi, 1000)  # 参数范围[0, 2π]

# # 爱心方程
# x=x_center + s*(16*(np.sin(theta))**3)
# z=z_center + s*(13*np.cos(theta) - 5*np.cos(2*theta) - 2*np.cos(3*theta) -np.cos(4*theta))

# # 在X-Z平面绘图
# plt.figure(figsize=(8, 8))
# plt.plot(x, z, color='red', linewidth=3)
# plt.scatter(x[0],z[0], color = 'b', label = 'Start point')
# plt.axis('equal')
# plt.title('Heart Curve in X-Z Plane')
# plt.xlabel('X-axis')
# plt.ylabel('Z-axis')
# plt.grid(True, alpha=0.3)
# plt.show()


'''----------------------生成3D轨迹----------------------'''
# ====== 参数配置 ======
duration = 40.0         # 总时长（秒）
time_step = 0.005         # 时间步长（秒）

omega = 0.5             # 角速度（控制轨迹速度）
x0 ,y0, z0 =0,0,0 # 起点
s = 0.08  # 缩放因子

# ====== 生成时间序列 ======
t = np.arange(0, duration, time_step)

# ====== 轨迹方程 ======
theta = omega*t - np.pi

x_center, z_center = 0 + x0,  17*s + z0 +0.2  # 中心点坐标 （稍微高一点，避免撞地）

# x-z平面上的爱心方程
x=x_center + s*(16*(np.sin(theta))**3)
z=z_center + s*(13*np.cos(theta) - 5*np.cos(2*theta) - 2*np.cos(3*theta) -np.cos(4*theta))
y=np.zeros_like(t)

vx = np.zeros_like(t)
vy = np.zeros_like(t)
vz = np.zeros_like(t)

# ====== 姿态与角速度 ======
# 假设无人机始终朝向速度方向（简化处理）
yaw = np.zeros_like(t)                     # 偏航角（基于水平速度方向）

# Roll/Pitch 保持水平（设为0）
roll = np.zeros_like(t)
pitch = np.zeros_like(t)

# 角速度（姿态导数）
p = np.zeros_like(t)                         # Roll角速度
q = np.zeros_like(t)                         # Pitch角速度
r = np.zeros_like(t)                         # Yaw角速度

# ====== 组合状态量 ======
# 状态量顺序：[x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
states = np.vstack([x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]).T


# ====== 可视化 ======
plt.figure(figsize=(14, 10))
ax1 = plt.subplot(221, projection='3d')
ax1.plot(x, y, z, label='Reference Path')
ax1.scatter(x[0], y[0], z[0], c='r', s=50)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
# ax1.set_title('3D Trajectory from Ground')
ax1.set_box_aspect([-1,1,1])
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

with open('heart_xz_trajectory.txt', 'w') as f:
    # 写入表头
    f.write("# x(m) y(m) z(m) dx(m/s) dy(m/s) dz(m/s) roll(rad) pitch(rad) yaw(rad) droll(rad/s) dpitch(rad/s) dyaw(rad/s)\n")
    
    # 写入数据
    for point in states:
        line = ', '.join(['%.6f' % val for val in point])
        f.write(line + ',\n')

XRef= states

