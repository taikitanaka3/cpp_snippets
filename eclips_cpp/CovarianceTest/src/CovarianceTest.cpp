#include <array>
#include <iostream>
#include <math.h>
#include <numeric>
#include <vector>
using namespace std;

float set_variance(array<float, 3> data) {
  float num = data.size();
  float avg = std::accumulate(data.begin(), data.end(), 0) / num;
  float variance = 0.f;
  for (int i = 0; i < data.size(); i++) {
    variance += pow(data[i] - avg, 2);
  }
  variance /= num;
  cout << "var" << variance << endl;
  return variance;
}
float SetCovarianceComponent(vector<float> &x, vector<float> &y) {
  if (x.size() != y.size()) {
    cout << "different array size" << endl;
  }
  float len = x.size();
  float avgx = std::accumulate(x.begin(), x.end(), 0) / len;
  float avgy = std::accumulate(y.begin(), y.end(), 0) / len;
  // cout << "average x"<<avgx<<endl;
  // cout << "average y"<<avgy<<endl;
  float coefficient = 0.f;
  for (int i = 0; i < len; i++) {
    coefficient += (x[i] - avgx) * (y[i] - avgy) / len;
  }
  coefficient /= len;
  // cout<<"coeffcient"<<coefficient<<endl;
  return coefficient;
}

vector<float> set_covariance(array<float, 6> x) {
  vector<float> covarianceMatrix(36, 0);
  // covarianceMatrix[0][0]=CoSetCovarianceComponent()

  return covarianceMatrix;
}

int main() {

  vector<float> x = {1, 2, 3};
  vector<float> y = {10, 20, 30};
  array<float, 3> z = {100, 200, 300};

  array<float, 4> covarianceMatrix = {0, 0, 0, 0};

  covarianceMatrix[0] = SetCovarianceComponent(x, x);
  covarianceMatrix[1] = covarianceMatrix[2] = SetCovarianceComponent(x, y);
  covarianceMatrix[3] = SetCovarianceComponent(y, y);
  cout << covarianceMatrix[3] << "!!!Hello World!!!"
       << endl; // prints !!!Hello World!!!
  return 0;
}
