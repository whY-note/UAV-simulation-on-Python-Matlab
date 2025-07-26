import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from UAV_Trajectory.traj_8 import XRef  # 8字型轨迹
from UAV_Trajectory.traj_hover1 import XRef
df = pd.read_excel("draw_traj_ROS/xk_history_tinympc_hover.xlsx",index_col=0)
data_xyz = np.array(df.iloc[:,:3])

x=data_xyz[:,0]
y=data_xyz[:,1]
z=data_xyz[:,2]
# # 绘制散点图（蓝色点）
# scatter = ax.scatter(x, y, z, c='b', marker='-', alpha=0.8, s=20)

# 连接数据点形成轨迹（红色线段）
# for i in range(len(x) - 1):
#     ax.plot([x[i], x[i+1]], [y[i], y[i+1]], [z[i], z[i+1]], 'r-', alpha=0.5)


fig=plt.figure(figsize=(10,8))
    
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z,
            'b-', linewidth=1,
            label="real"
            )
x_ref=XRef  
ax.plot(x_ref[:,0], x_ref[:,1], x_ref[:,2],
                 color='r', linestyle='--', linewidth=1,
                 label="reference"
                 )

# 添加标签和标题
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title('3D Visualization of Table Data')


# 添加网格和视图调整
ax.grid(True, linestyle='--', alpha=0.5)


plt.show()

delta_z=z -  x_ref[:2001,2]
print(delta_z)
t = np.arange(len(delta_z))
plt.plot(t,delta_z)
plt.ylabel("delta z (m)")
plt.grid(True, linestyle='--', alpha=0.5)
# plt.title("$\z_{real} - \z_{ref}$")
plt.show()