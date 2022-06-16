#include <bits/stdc++.h>
using namespace std;
using ll = long long int;
using ld = long double;
using VI = vector<ll>;
using VD = vector<double>;
using VVI = vector<VI>;
using VC = vector<char>;
using VB = vector<bool>;
using VVC = vector<VC>;
using VS = vector<string>;
using PLL = pair<ll, ll>;
using PLD = pair<ld, ld>;
using VPLL = vector<PLL>;
#define print(x) std::cout << x << "\n"
#define rep(i, n) for (ll i = 0; i < (ll)(n); i++)
#define repd(i, n) for (ll i = (ll)(n)-1; i >= 0; i--)
#define ALL(x) (x).begin(), (x).end()
#define ALLR(x) (x).rbegin(), (x).rend()
#define SZ(x) ((ll)(x).size())
#define MAX(x) *max_element((x).begin(), (x).end())
#define MIN(x) *min_element((x).begin(), (x).end())
#define SORTR(x) sort((x).rbegin(), (x).rend())
#define SORT(x) sort((x).begin(), (x).end())
#define SUM(x) accumulate((x).begin(), (x).end(), 0)
#define FILL(x, a) fill(x.begin(), x.end(), a)
#define EACH(i, x)                                                             \
  for (typeof((x).begin()) i = (x).begin(); i != (x).end(); ++i)
#define EXIST(v, x) (std::find(v.begin(), v.end(), x) != v.end())

const ll INF = 1e18;
const ld EPS = 1e-10;
const int MOD = int(1e9) + 7;
template <class T> bool chmax(T &a, const T &b) {
  if (a < b) {
    a = b;
    return 1;
  }
  return 0;
}
template <class T> bool chmin(T &a, const T &b) {
  if (b < a) {
    a = b;
    return 1;
  }
  return 0;
}
template <class BidirectionalIterator>
bool next_partial_permutation(BidirectionalIterator first,
                              BidirectionalIterator middle,
                              BidirectionalIterator last) {
  reverse(middle, last);
  return next_permutation(first, last);
}
ll gcd(ll x, ll y) { return (x % y) ? gcd(y, x % y) : y; }
ll lcm(ll x, ll y) { return x / gcd(x, y) * y; }
ll GCD(VI v) {
  ll a = v[0];
  for (ll i = 1; i < SZ(v); i++) {
    a = gcd(a, v[i]);
  }
  return a;
}
ll LCM(VI v) {
  ll a = v[0];
  for (ll i = 1; i < SZ(v); i++) {
    a = lcm(a, v[i]);
  }
  return a;
}
VI Bit2Vector(const ll bit, ll n) {
  VI s;
  rep(i, n) if (bit & (1 << i)) s.push_back(i);
  return s;
}
#define Vprint(x)                                                              \
  for (int i = 0; i < size(x); i++) {                                          \
    print(x[i]);                                                               \
  }

double average(VD arr) {
  if (arr.size() == 0)
    return 0;
  return accumulate((arr).begin(), (arr).end(), 0) / arr.size();
}

VD normalization(VD arr) {
  double min = *std::min_element(arr.begin(), arr.end());
  double max = *std::max_element(arr.begin(), arr.end());
  for (auto &v : arr) {
    v = min + (v - min) * (1 - min) / (max - min);
  }
  return arr;
}

VD standardization(VD val) {
  double variance = 0;
  double ave = average(val);
  for (const auto v : val) {
    variance += std::pow(v - ave, 2);
  }
  double stddev = std::sqrt(variance / SZ(val));
  for (auto &v : val) {
    v = (v - ave) / stddev;
  }
  return val;
}

template <class T> void applyAveragingToVector(T &arr) {
  if (arr.size() == 0)
    return;
  double ave = std::accumulate((arr).begin(), (arr).end(), 0.0) / arr.size();
  for (auto &v : arr) {
    v -= ave;
  }
  Vprint(arr);
  return;
}

static double sequentialAverage(int &cnt, double &avg, const double val) {
  avg = (cnt * avg + val) / (cnt + 1);
  cnt++;
  return avg;
}

static double sequentialStddev(int &cnt, double &avg, double &variance,
                               const double val) {
  double old_avg = avg;
  avg = (cnt * avg + val) / (cnt + 1);
  variance = (cnt * (variance + std::pow(old_avg, 2)) + std::pow(val, 2)) /
                 (double)(cnt + 1) -
             std::pow(avg, 2);
  cnt++;
  return std::sqrt(variance);
}

static double sequentialCovariance(int &cnt, double &avg_x, double &avg_y,
                                   double &cov, const double val_x,
                                   const double val_y) {
  if (cnt == 0) {
    cnt++;
    return 0;
  }
  cov = (cnt * cov +
         (val_x - avg_x) * (val_y - avg_y) * ((cnt + 1.0) / (double)cnt)) /
        (double)(cnt + 1.0);
  cnt++;
  return cov;
}

template <class T> int getMaximumIndex(const T x) {
  const auto iter = std::max_element(x.begin(), x.end());
  int index = std::distance(x.begin(), iter);
  return index;
}
template <class T> double getAverageFromVector(const T &arr) {
  if (arr.size() == 0)
    return 0;
  return std::accumulate((arr).begin(), (arr).end(), (double)0.0) /
         (double)arr.size();
}

template <class T> double getStddevFromVector(const T &arr) {
  if (arr.size() == 0)
    return 0;
  double average = getAverageFromVector(arr);
  double diff = 0;
  for (const auto &v : arr) {
    diff += std::pow(v - average, 2);
  }
  return std::sqrt(diff / (double)arr.size());
}

template <class T> double getCovarianceFromVector(const T &x, const T &y) {
  int sz = x.size();
  double cov = 0;
  if (sz == 0) {
    return 0;
  }
  double avg_x = getAverageFromVector(x);
  double avg_y = getAverageFromVector(y);
  for (int i = 0; i < x.size(); i++) {
    cov += (x[i] - avg_x) * (y[i] - avg_y);
  }
  return cov / sz;
}

template <class T>
double getCorrelationCoefficientFromVector(const T &x, const T &y) {
  int sz = x.size();
  double coeff = 0;
  if (sz == 0)
    return 0;
  double avg_x = getAverageFromVector(x);
  double avg_y = getAverageFromVector(y);
  double x_stddev = getStddevFromVector(x);
  double y_stddev = getStddevFromVector(y);
  for (int i = 0; i < x.size(); i++) {
    coeff += (x[i] - avg_x) * (y[i] - avg_y);
  }
  coeff /= (x_stddev * y_stddev * sz);
  return coeff;
}
void Main() {
  int cnt = 0;
  int cnt_stddev = 0;
  int cnt_avg_x = 0;
  int cnt_avg_y = 0;
  int cnt_y = 0;
  int cnt_x = 0;
  int cnt_cov = 0;
  double avg = 0;
  double val;

  double var = 0;
  vector<double> x;
  vector<double> y;

  double avg_x;
  double avg_y;
  double xy_cov;

  for (int i = 0; i < 4; ++i) {
    x.emplace_back(i);
    y.emplace_back(i * 2);
    double val_x = i;
    double val_y = i * 2;
    double seq_stddev = sequentialStddev(cnt_stddev, avg, var, val_x);
    double normal_stddev = getStddevFromVector(x);
    double avg_x = sequentialAverage(cnt_avg_x, avg_x, val_x);
    double avg_y = sequentialAverage(cnt_avg_y, avg_y, val_y);
    double seq_cov =
        sequentialCovariance(cnt_cov, avg_x, avg_y, xy_cov, val_x, val_y);
    double normal_cov = getCovarianceFromVector(x, y);
    print("avg: " << avg << " var: " << var << " stddev: " << seq_stddev
                  << " normal: " << normal_stddev << " cov: " << seq_cov
                  << " normal_cov: " << normal_cov);
  }
  return;
}

double calcSequantialCrrelation() {
  int cnt_stddev = 0;
  int cnt_avg_x = 0;
  int cnt_avg_y = 0;
  int cnt_stddev_x = 0;
  int cnt_stddev_y = 0;
  int cnt_y = 0;
  int cnt_x = 0;
  int cnt_cov = 0;
  vector<double> x;
  vector<double> y;
  double avg_x;
  double avg_y;
  double var_x = 0;
  double var_y = 0;
  double xy_cov;

  for (int i = 0; i < 4; ++i) {
    x.emplace_back(i);
    y.emplace_back(i * 2);
    double val_x = i;
    double val_y = i * 2;
    double avg_x = sequentialAverage(cnt_avg_x, avg_x, val_x);
    double avg_y = sequentialAverage(cnt_avg_y, avg_y, val_y);
    double stddev_x = sequentialStddev(cnt_stddev_x, avg_x, var_x, val_x);
    double stddev_y = sequentialStddev(cnt_stddev_y, avg_y, var_y, val_y);
    double seq_cov =
        sequentialCovariance(cnt_cov, avg_x, avg_y, xy_cov, val_x, val_y);
    double seq_corr = seq_cov / (stddev_x * stddev_y);
    double normal_cov = getCovarianceFromVector(x, y);
    print(" cov: " << seq_cov << " normal_cov: " << normal_cov << "seq corr"
                   << seq_corr);
  }
}

void Main2() {
  int cnt = 0;
  int cnt_stddev = 0;
  int cnt_avg_x = 0;
  int cnt_avg_y = 0;
  int cnt_y = 0;
  int cnt_x = 0;
  int cnt_cov = 0;
  double avg = 0;
  double val;

  double var = 0;
  vector<double> x;
  vector<double> y;

  double avg_x;
  double avg_y;
  double xy_cov;

  for (int i = 0; i < 4; ++i) {
    x.emplace_back(i);
    y.emplace_back(i * 2);
    double val_x = i;
    double val_y = i * 2;
    double seq_stddev = sequentialStddev(cnt_stddev, avg, var, val_x);
    double normal_stddev = getStddevFromVector(x);
    double avg_x = sequentialAverage(cnt_avg_x, avg_x, val_x);
    double avg_y = sequentialAverage(cnt_avg_y, avg_y, val_y);
    double seq_cov =
        sequentialCovariance(cnt_cov, avg_x, avg_y, xy_cov, val_x, val_y);
    double normal_cov = getCovarianceFromVector(x, y);
    print("avg: " << avg << " var: " << var << " stddev: " << seq_stddev
                  << " normal: " << normal_stddev << " cov: " << seq_cov
                  << " normal_cov: " << normal_cov);
  }
  return;
}

int main() {
  std::cin.tie(0);
  std::ios_base::sync_with_stdio(false);
  std::cout << std::fixed << std::setprecision(15);
  calcSequantialCrrelation();
}
