#include <bits/stdc++.h>
#
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace std;
using ll = long long int;
using ld = long double;
using VI = vector<ll>;
using VD = vector<ld>;
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

using namespace Eigen;

void Main() {
  Vector3i a(2, -1, 0);
  Vector3i b(3, 1, 4);
  Vector3i sub = 3 * a - 2 * b;
  print("a - b = \n" << sub);

  double inner_prod = a.dot(b);
  print("a \u2022 b = " << inner_prod);

  Vector3i cross_prod = a.cross(b);
  print("a X b = \n" << cross_prod);

  float norm = a.norm();
  print("norm a = " << norm);

  MatrixXi A(3, 2);
  A << 3, 1, 8, 3, 1, 0;
  MatrixXi B(3, 2);
  B << 1, 5, -2, 3, 4, -1;
  MatrixXi C = 2 * A - 3 * B;
  print("a X b = \n" << C);

  Vector3f v;
  v = Vector3f::Identity();

  // (0, 0, 0)
  v = Vector3f::Zero();

  // (1, 1, 1)
  v = Vector3f::Ones();

  // (1, 0, 0)
  v = Vector3f::UnitX();

  // (0, 1, 0)
  v = Vector3f::UnitY();

  // (0, 0, 1)
  v = Vector3f::UnitZ();

  v = Vector3f::Constant(0.5f);

  // 各要素への読み書き
  float x, y, z;
  x = v.x();
  y = v.y();
  z = v.z();

  v.x() = x;
  v.y() = y;
  v.z() = z;

  v(0) = x;
  v(1) = y;
  v(2) = z;

  x = v[0];
  y = v[1];
  z = v[2];

  v << 0.5f, 1.2f, -2.0f;

  // cp@y
  Vector3f v1;
  v1 = v;

  // vector * const
  v1 = v * 2.0f;
  v1 = v / 2.0f;
}

int main() {
  std::cin.tie(0);
  std::ios_base::sync_with_stdio(false);
  std::cout << std::fixed << std::setprecision(15);
  Main();
}
