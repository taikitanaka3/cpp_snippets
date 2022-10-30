import numpy as np
from sklearn.metrics import mean_squared_error

# y_trueが真の値、y_predが予測値
y_true = np.array([1.0, 1.1, 1.3, 1.4, 1.6, 1.7, 1.9, 2.0])
y_pred = np.array([1.0, 1.2, 1.3, 1.5, 1.5, 1.9, 1.9, 2.0])

# scikit-learn で計算する場合
rmse = np.sqrt(mean_squared_error(y_true, y_pred))
print(rmse)

# numpy で計算する場合

for i in range(10):
    rmse = np.sqrt(np.mean((y_true - y_pred*i) ** 2))
    print(rmse)