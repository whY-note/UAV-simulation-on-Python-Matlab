import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def read_real_traj_file(file_path):
    # df=pd.read_csv(file_path)
    # df=df.applymap(lambda x: float(str(x).replace(' ','')) if isinstance(x,str) else x)
    data_array = np.loadtxt(file_path)
    
    # 转换为Pandas DataFrame
    df = pd.DataFrame(data_array)
    
    # 删除空行
    df=df.dropna()
    df=df.reset_index(drop=True)
    
    return df

def read_ref_traj_file(file_path):
    """
    读取参考轨迹数据文件，返回 DataFrame
    文件格式：
    # x(m) y(m) z(m) dx(m/s) dy(m/s) dz(m/s) roll(rad) pitch(rad) yaw(rad) droll(rad/s) dpitch(rad/s) dyaw(rad/s)
    
    """
    # 读取文件，跳过注释行
    df = pd.read_csv(file_path, comment='#', header=None)
    
    # 去除可能的尾随逗号（如果存在）
    df = df.applymap(lambda x: float(str(x).replace(',', '')) if isinstance(x, str) else x)
    
    return df

def read_time_file(file_path):
    """
    读取计算时间数据文件，返回 DataFrame
    """
    df = pd.read_csv(file_path, header=None)
    
    # 删除空行
    df=df.dropna()
    df=df.reset_index(drop=True)
    return df

'''
solver_name:
- tinympc
- qpOASES
- ours
'''
# solver_name="ours"
df_real_tinympc=read_real_traj_file("draw_traj_vitis/fig8_real_traj/tinympc_fig8_N20_o2.txt")  
df_real_qpOASES=read_real_traj_file("draw_traj_vitis/fig8_real_traj/qpOASES_fig8_N20_o2.txt")  
df_real_ours=read_real_traj_file("draw_traj_vitis/fig8_real_traj/ours_fig8_N20_o2.txt")  

df_ref=read_ref_traj_file("draw_traj_vitis/fig8_trajectory.txt")

df_time_tinympc=read_time_file("draw_traj_vitis/fig8_real_traj/tinympc_fig8_N20_o2_Time.txt")
df_time_qpOASES=read_time_file("draw_traj_vitis/fig8_real_traj/qpOASES_fig8_N20_o2_Time.txt")
df_time_ours=read_time_file("draw_traj_vitis/fig8_real_traj/ours_fig8_N20_o2_Time.txt")

# 设定 一圈的开始与结束
fig8_single_start=3338
fig8_single_end= 3968 #df_real.shape[0]

df_real_xyz_tinympc=df_real_tinympc.iloc[fig8_single_start: fig8_single_end,:3]
df_real_xyz_qpOASES=df_real_qpOASES.iloc[fig8_single_start: fig8_single_end,:3]
df_real_xyz_ours=df_real_ours.iloc[fig8_single_start: fig8_single_end,:3]

df_ref_xyz=df_ref.iloc[fig8_single_start: fig8_single_end,:3]

df_time_tinympc = df_time_tinympc.iloc[fig8_single_start: fig8_single_end,:]
df_time_qpOASES = df_time_qpOASES.iloc[fig8_single_start: fig8_single_end,:]
df_time_ours = df_time_ours.iloc[fig8_single_start: fig8_single_end,:]

'''_______________________________画轨迹图_______________________________'''
plt.plot(df_real_xyz_tinympc.iloc[:, 0],df_real_xyz_tinympc.iloc[:, 1],'g-',label="TinyMPC")
plt.plot(df_real_xyz_qpOASES.iloc[:, 0],df_real_xyz_qpOASES.iloc[:, 1],color='royalblue',label="qpOASES")
plt.plot(df_real_xyz_ours.iloc[:, 0],df_real_xyz_ours.iloc[:, 1],'r-',label="Ours")

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

df_diff_ours = df_real_xyz_ours-df_ref_xyz
df_diff_ours['magnitude']=np.linalg.norm(np.array(df_diff_ours),axis=1)

df_diff_tinympc['magnitude'].plot(color='g',linewidth=1, alpha=0.8, label="TinyMPC")
df_diff_qpOASES['magnitude'].plot(color='royalblue',linewidth=1, alpha=0.8,label="qpOASES")
df_diff_ours['magnitude'].plot(color='r',linewidth=1, alpha=0.8,label="Ours")

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

print("Ours\n")
print(df_diff_ours.describe())

'''________________________________画计算时间图_____________________________________'''
df_time_tinympc[0].plot(color='g',linewidth=1,alpha=0.8, label="TinyMPC")
df_time_qpOASES[0].plot(color='royalblue',linewidth=1,alpha=0.8,label="qpOASES")
df_time_ours[0].plot(color='r',linewidth=1,alpha=0.8,label="Ours")

plt.ylabel("Solving Time($\mu s$)")
plt.xlabel("Time(s)")

time_ticks=np.linspace(fig8_single_start,fig8_single_end,10)
print(time_ticks)
time_ticks_label=list( str(round(i,2)) for i in np.linspace(0,6.3,10) )
print(time_ticks_label)
plt.xticks(time_ticks, time_ticks_label,
           rotation=45)

plt.title("Solving Time")
plt.legend(loc='right')

plt.show()

print("TinyMPC\n")
print(df_time_tinympc.describe())

print("qpOASES\n")
print(df_time_qpOASES.describe())

print("Ours\n")
print(df_time_ours.describe())