import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# 測定データ
acc_des = np.array([0, 0, 1, 1, 1, 1, 1])
acc_act = np.array([0, 0, 0, 0.3, 0.6, 0.8, 1])
time_t = np.array([0, 1, 2, 3, 4, 5, 6])

A = np.vstack((acc_des, np.ones(len(acc_des)))).T
v = acc_act.T

print(A)
print(v)
# 最小二乗近似
u = np.linalg.inv(A.T @ A) @ A.T @ v
print(u)

estimate = acc_des * u[0] + u[1]

def evalueate(ref,data):
    mse = mean_squared_error(ref,data)
    return mse

time_constant = 1.0

error_max = 10000
delay_index = 0
for i in range(3):
    size = 3
    #　最小二乗
    u = 0.3
    # 遅延、time constantを反映させた値の評価
    error = evalueate(acc_act[i:3+i],acc_des[0:3]*u)
    # error、遅延、時定数の更新
    if(error_max > error):
        delay_index = i
        time_constant = u
    print(error,delay_index)


for i in range(10):
    error = evalueate(acc_act,acc_des*time_constant)
    print(error)

# 近似式のプロット
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.set_xlabel('acc_des')
ax.set_ylabel('acc_act')
ax.scatter(time_t, acc_act, label='measure data')
ax.plot(time_t, estimate, 'r', label='least square')
ax.legend(loc='best')

plt.show()