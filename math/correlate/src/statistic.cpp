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

template <class T> int getMaximumIndex(const T x) {
  const auto iter = std::max_element(x.begin(), x.end());
  int index = std::distance(x.begin(), iter);
  return index;
}
template <class T> double getAverageFromVector(const T &arr) {
  if (arr.size() == 0)
    return 0;
  return std::accumulate((arr).begin(), (arr).end(), 0.0) / (double)arr.size();
}

template <class T> double getStddevFromVector(const T &arr) {
  if (arr.size() == 0)
    return 0;
  double average = getAverageFromVector(arr);
  double stddev = 0;
  for (const auto &v : arr) {
    stddev += std::sqrt(std::pow(v - average, 2) / (double)arr.size());
  }
  return stddev;
}

template <class T>
double getCorrelationCoefficientFromVector(const T &x, const T &y) {
  int sz = x.size();
  double coeff = 0;
  if (sz == 0)
    return 0;
  double x_avg = getAverageFromVector(x);
  double y_avg = getAverageFromVector(y);
  double x_stddev = getStddevFromVector(x);
  double y_stddev = getStddevFromVector(y);
  for (int i = 0; i < x.size(); i++) {
    coeff += (x[i] - x_avg) * (y[i] - y_avg) / sz;
  }
  coeff /= (x_stddev * y_stddev);
  return coeff;
}
void Main() {

  vector<double> a(5, 3);
  a[3] = 10;
  double ave = average(a);
  // print(ave);
  // VD stddev=standardization(a);
  // Vprint(stddev);

  std::vector<double> x, y;
  for (int i = 0; i < 10; i++) {
    x.push_back(i);
    y.push_back(i * i * i);
  }
  y[5] = -2000;
  double coeff = getCorrelationCoefficientFromVector(x, y);
  print(coeff);
  // applyAveragingToVector(x);
  // int index=getMaximumIndex(x);
  // print(index);
  // std::vector<double>::iterator iter = std::max_element(x.begin(), x.end());
  // size_t index = std::distance(x.begin(), iter);
  // std::cout << "max element:" << x[index] << std::endl;
  return;
}

int main() {
  std::cin.tie(0);
  std::ios_base::sync_with_stdio(false);
  std::cout << std::fixed << std::setprecision(15);
  Main();
}
