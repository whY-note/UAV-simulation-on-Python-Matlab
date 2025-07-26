import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def read_trajectory_file(file_path):
    """
    读取轨迹数据文件，返回 DataFrame
    文件格式：
    # x(m) y(m) z(m) dx(m/s) dy(m/s) dz(m/s) roll(rad) pitch(rad) yaw(rad) droll(rad/s) dpitch(rad/s) dyaw(rad/s)
    0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
    ...
    """
    # 读取文件，跳过注释行
    df = pd.read_csv(file_path, comment='#', header=None)
    
    # 去除可能的尾随逗号（如果存在）
    df = df.applymap(lambda x: float(str(x).replace(',', '')) if isinstance(x, str) else x)
    
    return df

df_ref_traj=read_trajectory_file('draw_traj_ROS/fig8_trajectory_song.txt')

print(df_ref_traj)

df_real_traj=pd.read_excel("draw_traj_ROS/xk_history_song5.xlsx",index_col=0)
print(df_real_traj)


plt.plot(df_real_traj.iloc[:, 0], df_real_traj.iloc[:, 1], 'b-', label='Real Trajectory') 
plt.plot(df_ref_traj.iloc[:, 0], df_ref_traj.iloc[:, 1], 'r-', label='Reference Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()


# 计算误差
df_real_traj_xyz=df_real_traj.iloc[:,:3]
df_ref_traj_xyz =df_ref_traj.iloc[:,:3]


from scipy.spatial.distance import directed_hausdorff


def calculate_trajectory_errors_xy(df_real,df_ref):
    # 计算x y 的坐标
    # 确保数据对齐
    n_points = min(len(df_real), len(df_ref))
    real = df_real.iloc[:n_points].values
    ref = df_ref.iloc[:n_points].values
    # 逐点欧氏距离误差
    pointwise_errors = np.linalg.norm(real - ref, axis=1)
    
    # 关键误差指标
    error_metrics = {
        'xy MAE': np.mean(pointwise_errors),
        'xy RMSE': np.sqrt(np.mean(pointwise_errors**2)),
        'xy Max Error': np.max(pointwise_errors),
        'xy Std Dev': np.std(pointwise_errors),
        'xy Hausdorff Distance': max(directed_hausdorff(real, ref)[0], 
                                 directed_hausdorff(ref, real)[0])
    }
    return error_metrics

def calculate_trajectory_errors(df_real, df_ref):
    
    """
    计算x y z 坐标之间的误差
    输入:
        df_real: 实际轨迹DataFrame [x,y,z]
        df_ref: 参考轨迹DataFrame [x,y,z]
    返回:
        error_metrics: 包含所有误差指标的字典
        error_df: 逐点误差DataFrame
    """
    # 确保数据对齐
    n_points = min(len(df_real), len(df_ref))
    real = df_real.iloc[:n_points].values
    ref = df_ref.iloc[:n_points].values
    
    # 逐点欧氏距离误差
    pointwise_errors = np.linalg.norm(real - ref, axis=1)
    
    # 关键误差指标
    error_metrics = {
        'MAE': np.mean(pointwise_errors),
        'RMSE': np.sqrt(np.mean(pointwise_errors**2)),
        'Max Error': np.max(pointwise_errors),
        'Std Dev': np.std(pointwise_errors),
        'Hausdorff Distance': max(directed_hausdorff(real, ref)[0], 
                                 directed_hausdorff(ref, real)[0]),
        'Mean Z Error': np.mean(np.abs(real[:,2] - ref[:,2])),
        'Horizontal Error': np.mean(np.linalg.norm(real[:,:2] - ref[:,:2], axis=1))
    }
    
    # 创建逐点误差DataFrame
    error_df = pd.DataFrame({
        'Point_Index': range(n_points),
        'X_Error': real[:,0] - ref[:,0],
        'Y_Error': real[:,1] - ref[:,1],
        'Z_Error': real[:,2] - ref[:,2],
        'Total_Error': pointwise_errors,
        'Horizontal_Error': np.linalg.norm(real[:,:2] - ref[:,:2], axis=1)
    })
    
    return error_metrics, error_df

# 选取一圈
# real_start=1366
# real_end=2000

real_start= 0
real_end=1000

# 画一圈的轨迹
plt.plot(df_real_traj_xyz.iloc[real_start:real_end,0],df_real_traj_xyz.iloc[real_start:real_end,1], 'b-', label='Real Trajectory') 
plt.plot(df_ref_traj.iloc[real_start:real_end, 0], df_ref_traj.iloc[real_start:real_end, 1], 'r-', label='Reference Trajectory')
plt.legend()
plt.grid(True)
plt.title('1 circle')
plt.axis('equal')
plt.show()

# 计算误差
error_metrics_xy=calculate_trajectory_errors_xy(df_real_traj_xyz.iloc[real_start:real_end,:2],df_ref_traj_xyz.iloc[real_start:real_end,:2])

error_metrics, error_df = calculate_trajectory_errors(df_real_traj_xyz.iloc[real_start:real_end,:], df_ref_traj_xyz.iloc[real_start:real_end,:])

# 打印关键指标
print("=== Trajectory Error Metrics ===")
for metric, value in error_metrics_xy.items():
    print(f"{metric}: {value:.4f}")
print("=== Trajectory Error Metrics ===")
for metric, value in error_metrics.items():
    print(f"{metric}: {value:.4f}")