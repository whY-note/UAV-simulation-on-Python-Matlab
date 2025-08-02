import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df_real_tinympc = pd.read_csv("draw_traj_ROS/synchronize_data/tinympc_uart_100Hz/xk_history.csv")
df_real_qpOASES = pd.read_csv("draw_traj_ROS/synchronize_data/qpOASES_uart_100Hz/xk_history.csv")
# Todo: ours 待测
# df_real_ours=

df_ref = pd.read_csv("draw_traj_ROS/synchronize_data/qpOASES_uart_100Hz/xk_ref.csv")

'''
solver_name:
- tinympc
- qpOASES
- ours
'''

# df_time_tinympc=read_time_file("draw_traj_vitis/fig8_real_traj/tinympc_fig8_N20_o2_Time.txt")
# df_time_qpOASES=read_time_file("draw_traj_vitis/fig8_real_traj/qpOASES_fig8_N20_o2_Time.txt")
# df_time_ours=read_time_file("draw_traj_vitis/fig8_real_traj/ours_fig8_N20_o2_Time.txt")

# 设定 一圈的开始与结束
fig8_single_start=5471
fig8_single_end= df_real_qpOASES.shape[0]

df_real_xyz_tinympc=df_real_tinympc.iloc[fig8_single_start: fig8_single_end,:3]
df_real_xyz_qpOASES=df_real_qpOASES.iloc[fig8_single_start: fig8_single_end,:3]
# df_real_xyz_ours=df_real_ours.iloc[fig8_single_start: fig8_single_end,:3] # todo

df_ref_xyz=df_ref.iloc[fig8_single_start: fig8_single_end,:3]

# df_time_tinympc = df_time_tinympc.iloc[fig8_single_start: fig8_single_end,:]
# df_time_qpOASES = df_time_qpOASES.iloc[fig8_single_start: fig8_single_end,:]
# df_time_ours = df_time_ours.iloc[fig8_single_start: fig8_single_end,:]

'''_______________________________画轨迹图_______________________________'''
plt.plot(df_real_xyz_tinympc.iloc[:, 0],df_real_xyz_tinympc.iloc[:, 1],color='mediumseagreen',label="TinyMPC")
plt.plot(df_real_xyz_qpOASES.iloc[:, 0],df_real_xyz_qpOASES.iloc[:, 1],color='dodgerblue',label="qpOASES")
# plt.plot(df_real_xyz_ours.iloc[:, 0],df_real_xyz_ours.iloc[:, 1],'r-',label="Ours") # todo

plt.plot(df_ref_xyz.iloc[:, 0],df_ref_xyz.iloc[:, 1], color='black',linestyle='--', label= "Reference")

plt.xlabel("X(m)")
plt.ylabel("Y(m)")

plt.legend(loc='upper right')
plt.show()


'''_______________________________画轨迹误差图__________________________________'''
df_diff_tinympc=df_real_xyz_tinympc-df_ref_xyz
df_diff_tinympc['magnitude']=np.linalg.norm(np.array(df_diff_tinympc),axis=1)

df_diff_qpOASES=df_real_xyz_qpOASES-df_ref_xyz
df_diff_qpOASES['magnitude']=np.linalg.norm(np.array(df_diff_qpOASES),axis=1)

# df_diff_ours = df_real_xyz_ours-df_ref_xyz
# df_diff_ours['magnitude']=np.linalg.norm(np.array(df_diff_ours),axis=1)

df_diff_tinympc['magnitude'].plot(color='mediumseagreen',linewidth=1, alpha=0.8, label="TinyMPC")
df_diff_qpOASES['magnitude'].plot(color='dodgerblue',linewidth=1, alpha=0.8,label="qpOASES")
# df_diff_ours['magnitude'].plot(color='r',linewidth=1, alpha=0.8,label="Ours")

plt.ylabel("Error(m)")
plt.xlabel("Time(s)")

time_ticks=np.linspace(fig8_single_start,fig8_single_end,10)
print(time_ticks)
time_ticks_label=list( str(round(i,2)) for i in np.linspace(0,6.3,10) )
print(time_ticks_label)
plt.xticks(time_ticks, time_ticks_label,
           rotation=45)

plt.title("Trajectory Error")
plt.legend(loc='right')
plt.show()

print("TinyMPC\n")
print(df_diff_tinympc.describe())

print("qpOASES\n")
print(df_diff_qpOASES.describe())

# print("Ours\n")
# print(df_diff_ours.describe())

# '''________________________________画计算时间图_____________________________________'''
# df_time_tinympc[0].plot(color='g',linewidth=1,alpha=0.8, label="TinyMPC")
# df_time_qpOASES[0].plot(color='royalblue',linewidth=1,alpha=0.8,label="qpOASES")
# df_time_ours[0].plot(color='r',linewidth=1,alpha=0.8,label="Ours")

# plt.ylabel("Solving Time($\mu s$)")
# plt.xlabel("Time(s)")

# time_ticks=np.linspace(fig8_single_start,fig8_single_end,10)
# print(time_ticks)
# time_ticks_label=list( str(round(i,2)) for i in np.linspace(0,6.3,10) )
# print(time_ticks_label)
# plt.xticks(time_ticks, time_ticks_label,
#            rotation=45)

# plt.title("Solving Time")
# plt.legend(loc='right')

# plt.show()

# print("TinyMPC\n")
# print(df_time_tinympc.describe())

# print("qpOASES\n")
# print(df_time_qpOASES.describe())

# print("Ours\n")
# print(df_time_ours.describe())