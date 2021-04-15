#include <bits/stdc++.h>
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
#define Vprint(x)                                                              \
  for (int i = 0; i < size(x); i++) {                                          \
    print(x[i]);                                                               \
  }
#define VVprint(x)                                                             \
  for (int i = 0; i < size(x); i++) {                                          \
    print(x[i]);                                                               \
  }

double lowpassFilter(double curr, double prev, double cutoff,
                     double delta_time) {
  double tau = 1 / (2 * M_PI * cutoff);
  double a = tau / (delta_time + tau);
  double b = delta_time / (delta_time + tau);
  print("cutoff: " << cutoff << " tau/(T+tau): " << a << " T/(T+tau): " << b);
  return prev * a + (1 - a) * curr;
}

/**
 * @param : arr vector like contener
 * @return : norm normalized arr vector
 */
template <class T> T getNormalizedVector(const T &arr) {
  T norm = arr;
  double min = *std::min_element(arr.begin(), arr.end());
  double max = *std::max_element(arr.begin(), arr.end());
  for (auto &v : norm) {
    // normalize range -1 ~ 1
    v = (2 * v - (max + min)) / (max - min);
  }
  return norm;
}

void Main() {
  ll n, m, l;
  ll res = 0;
  string s, t, u;
  string sres = "No or NO";

  VD a(10);
  for (int i = 0; i < 10; i++) {
    a[i] = (double)i * i;
  }
  double val = lowpassFilter(3, 4, 7, 0.033);
  for (int i = 3; i < 30; i++) {
    lowpassFilter(1, 2, i, 0.033);
  }

  // auto b=getNormalizedVector(a);
  // Vprint(b);

  return;
}

int main() {
  std::cin.tie(0);
  std::ios_base::sync_with_stdio(false);
  std::cout << std::fixed << std::setprecision(4);
  Main();
}
