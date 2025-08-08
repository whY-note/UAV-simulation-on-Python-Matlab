import pandas as pd
import matplotlib.pyplot as plt

solver_name =  'tinympc' #'CODMPC'
df_xk_ref=pd.read_csv("draw_traj_ROS/synchronize_data/"+solver_name+"_uart_100Hz_time_vary_y30dg_x30dg/xk_ref.csv")
df_xk = pd.read_csv("draw_traj_ROS/synchronize_data/"+solver_name+"_uart_100Hz_time_vary_y30dg_x30dg/xk_history.csv")
df_uk = pd.read_csv("draw_traj_ROS/synchronize_data/"+solver_name+"_uart_100Hz_time_vary_y30dg_x30dg/uk_history.csv")

print(df_xk_ref)
print(df_xk)
print(df_uk)

title_name = df_xk_ref.columns
print(title_name)

# for name in title_name:
#     df_xk[name].plot(color = 'b', label='real')
#     df_xk_ref[name].plot(color = 'r', label='reference')
#     plt.title(str(name))
#     plt.show()




df_uk['thrust'].plot(label = 'thrust')
plt.show()

df_uk.plot(y= ['tau_x','tau_y','tau_z'], 
           color=["indianred","goldenrod","teal"])

plt.legend()
plt.show()

# 选择前n行数据
df_uk = df_uk.head(4000)
df_uk['delta_tau_x'] = df_uk['tau_x'].diff()
df_uk['delta_tau_x'].plot(color="indianred",label='delta_tau_x')


df_xk = df_xk.head(4000)
df_uk['delta_roll'] = df_xk['roll'].diff()
df_uk['delta_roll'].plot(color="seagreen",label='delta_roll')
plt.legend()
plt.show()