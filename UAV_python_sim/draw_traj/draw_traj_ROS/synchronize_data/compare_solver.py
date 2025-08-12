import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
# 推荐画图用: tinympc_uart_100Hz_time_vary_2m_2, tinympc_uart_100Hz_time_vary_y30dg_x30dg_5

FILE_ROOT="draw_traj_ROS/synchronize_data/"

# 
df_real_tinympc = pd.read_csv(FILE_ROOT + "tinympc_uart_100Hz_time_vary_heart_xz/xk_history.csv")
df_real_qpOASES = pd.read_csv(FILE_ROOT + "qpOASES_uart_100Hz_time_vary_heart_xz/xk_history.csv")
df_real_ours=pd.read_csv(FILE_ROOT + "CODMPC_uart_100Hz_time_vary_heart_xz/xk_history.csv")

# df_real_tinympc = pd.read_csv(FILE_ROOT + "tinympc_uart_100Hz_time_vary/xk_history.csv")
# df_real_qpOASES = pd.read_csv(FILE_ROOT + "qpOASES_uart_100Hz_time_vary/xk_history.csv")
# df_real_ours=pd.read_csv(FILE_ROOT + "CODMPC_uart_100Hz_time_vary/xk_history.csv")

df_ref = pd.read_csv(FILE_ROOT + "CODMPC_uart_100Hz_time_vary_heart_xz_2/xk_ref.csv")
print(df_ref)

'''
solver_name:
- tinympc
- qpOASES
- ours
'''
df_time_tinympc = pd.read_csv(FILE_ROOT + "tinympc_uart_100Hz_time_vary_heart_xz/uk_history.csv")
df_time_ours = pd.read_csv(FILE_ROOT + "CODMPC_uart_100Hz_time_vary_heart_xz_2/uk_history.csv")

start_time=0
end_time = 80 # unit:s
start_time_tick = 0
end_time_tick =8000

#设定 一圈的开始与结束
# fig8_single_start=5471
# fig8_single_end=7972 # df_real_tinympc.shape[0]
fig8_single_start= 0
fig8_single_end= 7972

# q取出一圈的轨迹
df_real_xyz_tinympc=df_real_tinympc.iloc[fig8_single_start: fig8_single_end,:3]
df_real_xyz_qpOASES=df_real_qpOASES.iloc[:45,:3]
df_real_xyz_ours=df_real_ours.iloc[fig8_single_start: fig8_single_end,:3] 

df_ref_xyz=df_ref.iloc[fig8_single_start: fig8_single_end,:3]

# 取出一圈的时间
df_time_tinympc = df_time_tinympc.iloc[fig8_single_start: fig8_single_end,4]
# df_time_qpOASES = df_time_qpOASES.iloc[fig8_single_start: fig8_single_end,:]
df_time_ours = df_time_ours.iloc[fig8_single_start: fig8_single_end,4]

'''_______________________________画2D轨迹图_______________________________'''
plt.plot(df_real_xyz_tinympc.iloc[:, 0],df_real_xyz_tinympc.iloc[:, 1],color='mediumseagreen',label="TinyMPC")
# plt.plot(df_real_xyz_qpOASES.iloc[:, 0],df_real_xyz_qpOASES.iloc[:, 1],color='dodgerblue',label="qpOASES")
plt.plot(df_real_xyz_ours.iloc[:, 0],df_real_xyz_ours.iloc[:, 1],'r-',label="COD-MPC")

plt.plot(df_ref_xyz.iloc[:, 0],df_ref_xyz.iloc[:, 1], color='black',linestyle='--', label= "Reference")

plt.xlabel("X(m)")
plt.ylabel("Y(m)")

plt.legend(loc='upper right')
plt.show()

'''_______________________________画3D轨迹图_______________________________'''
fig=plt.figure(figsize=(10,8))
    
ax = fig.add_subplot(111, projection='3d')
ax.plot(df_real_xyz_tinympc.iloc[:,0], df_real_xyz_tinympc.iloc[:,1], df_real_xyz_tinympc.iloc[:,2], 
            color='mediumseagreen',label="TinyMPC"
            )

ax.plot(df_real_xyz_qpOASES.iloc[:,0], df_real_xyz_qpOASES.iloc[:,1], df_real_xyz_qpOASES.iloc[:,2],
            color="dodgerblue",label="qpOASES"
            )

ax.plot(df_real_xyz_ours.iloc[:,0], df_real_xyz_ours.iloc[:,1], df_real_xyz_ours.iloc[:,2],
            'r-',label="COD-MPC"
            )


ax.plot(df_ref_xyz.iloc[:,0], df_ref_xyz.iloc[:,1], df_ref_xyz.iloc[:,2],
            color='black',linestyle='--', label= "Reference"
            )

# 添加标签和标题
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_box_aspect([-1, 1, 1]) 
ax.set_ylim(-0.2,0.2)
# ax.set_zlim(0, 0.7)
ax.grid(True, linestyle='--', alpha=0.5)
plt.legend(loc='upper right')
plt.show()


'''_______________________________画轨迹误差图__________________________________'''
fig=plt.figure(figsize=(10,8))

df_diff_tinympc=df_real_xyz_tinympc-df_ref_xyz
df_diff_tinympc['magnitude']=np.linalg.norm(np.array(df_diff_tinympc),axis=1)

# df_diff_qpOASES=df_real_xyz_qpOASES-df_ref_xyz
# df_diff_qpOASES['magnitude']=np.linalg.norm(np.array(df_diff_qpOASES),axis=1)

df_diff_ours = df_real_xyz_ours-df_ref_xyz
df_diff_ours['magnitude']=np.linalg.norm(np.array(df_diff_ours),axis=1)

df_diff_tinympc['magnitude'].plot(color='mediumseagreen',linewidth=1, alpha=0.8, label="TinyMPC")
# df_diff_qpOASES['magnitude'].plot(color='dodgerblue',linewidth=1, alpha=0.8,label="qpOASES")
df_diff_ours['magnitude'].plot(color='r',linewidth=1, alpha=0.8,label="COD-MPC")

plt.ylabel("Error(m)")
plt.xlabel("Time(s)")

time_ticks=np.linspace(start_time_tick,end_time_tick,11)
print(time_ticks)
time_ticks_label=list( str(round(i,2)) for i in np.linspace(start_time,end_time,11) )
print(time_ticks_label)
plt.xticks(time_ticks, time_ticks_label,
           rotation=45)

plt.title("Trajectory Error")
plt.legend(loc='right')
plt.show()

print("===========Trajectory Error============")
print("TinyMPC\n")
print(df_diff_tinympc.describe())

# print("qpOASES\n")
# print(df_diff_qpOASES.describe())

print("COD-MPC\n")
print(df_diff_ours.describe())

'''________________________________画计算时间图_____________________________________'''
fig=plt.figure(figsize=(10,8))
df_time_tinympc.plot(color='g',linewidth=1,alpha=0.8, label="TinyMPC")
# df_time_qpOASES[0].plot(color='royalblue',linewidth=1,alpha=0.8,label="qpOASES")
df_time_ours.plot(color='r',linewidth=1,alpha=0.8,label="COD-MPC")

plt.ylabel("Solving Time($\mu s$)")
plt.xlabel("Time(s)")

time_ticks=np.linspace(start_time_tick,end_time_tick,11)
print(time_ticks)
time_ticks_label=list( str(round(i,2)) for i in np.linspace(start_time,end_time,11) )
print(time_ticks_label)
plt.xticks(time_ticks, time_ticks_label,
           rotation=45)

plt.title("Solving Time")
plt.legend(loc='right')

plt.show()

print("===========Solving Time============")
print("TinyMPC\n")
print(df_time_tinympc.describe())

# print("qpOASES\n")
# print(df_time_qpOASES.describe())

print("Ours\n")
print(df_time_ours.describe())