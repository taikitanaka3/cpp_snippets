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

/**
 *
 * @param in : input signal
 * @param out : output signal
 * @return
 */
VD crossCorrelation(VD in, VD out) {
  VD corr(SZ(in) + SZ(out) - 1);
  /**
   * Direct Cross Correlation Method
   * Rx1x2 = E[x1x2(t-T)]
   */
  for (int i = 0; i < SZ(in); ++i) {
    for (int j = 0; j < SZ(out); ++j) {
      if (i + j + 1 < SZ(in) + SZ(out) - 1)
        corr[i + j + 1] += in[i] * out[j];
    }
  }
  return corr;
}

/**
 * @param corr
 * @return max index of correlation peak
 */
int peakDetector(VD corr) {
  double max = numeric_limits<double>::min();
  int max_index = 0;
  for (int i = 0; i < SZ(corr); ++i) {
    if (chmax<double>(max, corr[i])) {
      max_index = i;
    }
  }
  return max_index;
}

void Main() {
  ll n, m, l;
  ll res = 0;
  string s, t, u;
  string sres = "No or NO";
  VD in_ = {1, 4, 1, 0};
  VD out_ = {0, 0, 2, 1};
  VD in = {1, 4, 1, 0, 8, 9, 0, 0, 0, 0};        //{1,4,1,0};
  VD out = {0, 0, 2, 1, 5, 6, 6, 8, 9, 0, 0, 0}; //{0,0,2,1,};
  for (int j = 0; j < 1; j++) {
    VD corr = crossCorrelation(in, out);
    Vprint(corr);
    int peak = peakDetector(corr);
    int estimate_delay = peak + 1 - SZ(in);
    print(estimate_delay);
    in.emplace_back(in_[j % 4]);
    out.emplace_back(out_[j % 4]);
    in.erase(in.begin());
    out.erase(out.begin());
  }
  return;
}

int main() {
  std::cin.tie(0);
  std::ios_base::sync_with_stdio(false);
  std::cout << std::fixed << std::setprecision(15);
  Main();
}
