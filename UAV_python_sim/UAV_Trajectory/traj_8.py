import numpy as np
import matplotlib.pyplot as plt

# # ====== 参数配置 ======
# duration = 80.0         # 总时长（秒）
# time_step = 0.02         # 时间步长（秒）
# radius = 1.0            # 8字形半径（米）
# target_height = 2.0     # 目标飞行高度（米）
# climb_time = 5.0        # 爬升到目标高度的时间（秒）
# omega = 0.5             # 角速度（控制轨迹速度）

# ====== 参数配置 ======
duration = 40.0         # 总时长（秒）
time_step = 0.005         # 时间步长（秒）
radius = 1.0            # 8字形半径（米）
target_height = 2.0     # 目标飞行高度（米）
climb_time = 5.0        # 爬升到目标高度的时间（秒）
omega = 0.5             # 角速度（控制轨迹速度）

# ====== 生成时间序列 ======
t = np.arange(0, duration, time_step)

# ====== 轨迹方程 ======
# --- 水平方向（X-Y平面） ---
x = radius * np.sin(omega * t)               # X轴位置
y = 0.5 * radius * np.sin(2 * omega * t)     # Y轴位置

vx = radius * omega * np.cos(omega * t)      # X方向速度
vy = 0.5 * radius * 2 * omega * np.cos(2 * omega * t)  # Y方向速度

# --- 垂直方向（Z轴） ---
z = np.zeros_like(t)                         # 初始高度为0
vz = np.zeros_like(t)                        # 初始垂直速度为0

# 爬升阶段（使用平滑的余弦过渡）
climb_mask = t <= climb_time
z[climb_mask] = target_height * (1 - np.cos(np.pi * t[climb_mask] / climb_time)) / 2
vz[climb_mask] = (target_height * np.pi / (2 * climb_time)) * np.sin(np.pi * t[climb_mask] / climb_time)

# z[climb_mask] = target_height/climb_time * t[climb_mask] 

# 平飞阶段（保持目标高度）
z[~climb_mask] = target_height
vz[~climb_mask] = 0

# ====== 姿态与角速度 ======
# 假设无人机始终朝向速度方向（简化处理）
yaw = np.arctan2(vy, vx)                     # 偏航角（基于水平速度方向）
yaw = np.unwrap(yaw)                         # 解除角度跳变

# Roll/Pitch 保持水平（设为0）
roll = np.zeros_like(t)
pitch = np.zeros_like(t)

# 角速度（姿态导数）
p = np.zeros_like(t)                         # Roll角速度
q = np.zeros_like(t)                         # Pitch角速度
r = np.gradient(yaw, t)                      # Yaw角速度


# ====== 组合状态量 ======
# 状态量顺序：[x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
states = np.vstack([x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]).T


states[:,3:]=0

print(states.shape)
# ====== 可视化 ======
plt.figure(figsize=(14, 10))

# 3D轨迹
ax1 = plt.subplot(221, projection='3d')
ax1.plot(x, y, z, label='Flight Path')
ax1.scatter(x[0], y[0], z[0], c='r', s=50, label='Start (Ground)')
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

with open('fig8_trajectory.txt', 'w') as f:
    # 写入表头
    f.write("# x(m) y(m) z(m) dx(m/s) dy(m/s) dz(m/s) roll(rad) pitch(rad) yaw(rad) droll(rad/s) dpitch(rad/s) dyaw(rad/s)\n")
    
    # 写入数据
    for point in states:
        line = ', '.join(['%.6f' % val for val in point])
        f.write(line + ',\n')

XRef= states

last_climb_idx = len(climb_mask) - 1 - np.argmax(climb_mask[::-1])
XRef_noUpward =states[last_climb_idx:]
print(XRef_noUpward)