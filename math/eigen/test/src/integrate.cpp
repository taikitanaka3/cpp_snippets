#include <bits/stdc++.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;
using namespace std;
#define print(x) std::cout << x << "\n"

class Vehicle {
private:
  /**
   * @brief constructor with initialization
   * @param x initial state
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param C coefficient matrix of x for measurement model
   * @param Q covariace matrix for process model
   * @param R covariance matrix for measurement model
   * @param P initial covariance of estimated state
   */
  enum IDX {
    XX = 0,
    YY = 1,
    YAW = 2,
    YAWB = 3,
    VX = 4,
    WZ = 5,
  };
  MatrixXd A_;
  MatrixXd B_;
  MatrixXd C_;
  MatrixXd D_;
  MatrixXd X_;
  MatrixXd NL_;
  MatrixXd V_;
  MatrixXd S_;
  MatrixXd X_true_;
  MatrixXd X_pred_;
  MatrixXd X_obs_;
  int dim_x_;
  double v1 = 1.0;
  double v2 = 0;
  double l = 2.0;
  double dt_ = 0;

public:
  Vehicle() : dim_x_(6) {
    A_ = MatrixXd::Zero(6, 6);
    B_ = MatrixXd::Zero(6, 1);
    X_ = MatrixXd::Zero(6, 1);
    X_true_ = MatrixXd::Zero(5, 1);
    NL_ = MatrixXd::Zero(5, 3);
    V_ = MatrixXd::Ones(3, 1);
    S_ = MatrixXd::Zero(6, 1);
  }

  void initialize() {
    X_(IDX::XX) = 0;
    X_(IDX::YY) = 0;
    X_(IDX::YAW) = 0;
  }
  void integrate() {
    /*  == NonLinear model ==
     * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
     * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
     * yaw_{k+1} = yaw_k + tan(phi)*vx_/l + (wz_k) * dt
     * phi {k+1} = phi_k
     * b_{k+1}   = b_k
     */
    //
    NL_(0, 0) = std::cos(S_(3)) * dt_;     // x
    NL_(1, 0) = std::sin(S_(3)) * dt_;     // y
    NL_(2, 0) = std::tan(S_(4)) * dt_ / l; // th
    NL_(3, 1) = 1;                         // phi
    NL_(4, 2) = 1;                         // bias

    X_true_ += NL_ * V_;
    print(X_true_);
  }
  void predict(){

  };
  void update() {
    integrate();
    predict();
  };
};

void Main() {
  Matrix3d A_(3, 3); //係数行列A
  A_ << 1, 1, 0.5, 0, 1, 1, 0, 0, 1;
  std::cout << "A_ = \n" << A_ << std::endl;

  Matrix3d B_(3, 3); //係数行列B
  B_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  std::cout << "B_ = \n" << B_ << std::endl;
  ;

  Vector3d Ct_; //係数行列C  列ベクトル
  Ct_ << 1, 0, 0;
  std::cout << "Ct_  = \n" << Ct_ << std::endl;
  MatrixXd C_ = Ct_.transpose(); //係数行列C  行ベクトル
  std::cout << "C_  = \n" << C_ << std::endl;

  Matrix3d Q_(3, 3); //状態ノイズ
  Q_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  std::cout << "Q_ = \n" << Q_ << std::endl;

  MatrixXd P_(3, 3); //事前誤差共分散行列
  MatrixXd G_(3, 1); //カルマンゲイン
  double R_ = 1.0;   //システムノイズ
  double y[10] = {2, 4, 6, 8, 10, 12, 14, 16};

  Matrix3d I_(3, 3); //単位行列
  I_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Vector3d X_;
  X_ << 1, 1, 1;

  P_ = MatrixXd::Ones(3, 3);

  // calc next state
  X_ = A_ * X_; //	事前状態推定
  print("X[1]= \n" << X_);

  P_ = A_ * P_ * A_.transpose() + B_ * Q_ * B_.transpose();
  print("P[1]= \n" << P_); //	事前誤差共分散行列
  G_ = P_ * C_.transpose() / ((C_ * P_ * C_.transpose())(0, 0) + R_);
  // double a=INV;
  print("G[1]= \n" << G_); //	事前誤差共分散行列

  MatrixXd X_1 = X_ + G_ * (y[0] - (C_ * X_)(0, 0));
  print("X1[1]= \n" << X_1); //	事前誤差共分散行列
  MatrixXd P_1 = (I_ - G_ * C_) * P_;
  print("P1[1]= \n" << P_1); //	事前誤差共分散行列
  Vehicle vehicle;
  vehicle.initialize();
  print("===start===");
  for (int i = 0; i < 10; i++) {
    print("===update===");
    vehicle.update();
  }
  return;
}

int main() {
  std::cin.tie(0);
  std::ios_base::sync_with_stdio(false);
  std::cout << std::fixed << std::setprecision(15);
  Main();
}
