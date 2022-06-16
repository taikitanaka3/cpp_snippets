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
#define VPRINT(x)                                                              \
  for (int i = 0; i < x.size(); i++) {                                         \
    PRINT(x[i]);                                                               \
  }
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

namespace navigation_msgs {
struct MapMetaData {
  float resolution;
  int width;
  int height;

  MapMetaData(const float &resolution = 0.f, const int &width = 0,
              const int &height = 0) {
    this->resolution = resolution;
    this->width = width;
    this->height = height;
  }
};

struct OccupancyGrid {
  MapMetaData info;
  vector<int> data = {};

  OccupancyGrid(const MapMetaData &info = MapMetaData(),
                const vector<int> &data = vector<int>()) {
    this->info = info;
    this->data = data;
  }
};
}; // namespace navigation_msgs

template <class T> void Vprint(T vec) {
  for (const auto &v : vec) {
    std::cout << v;
  }
  std::cout << std::endl;
}
template <class T> void Vprint(T vec, int w) {
  for (int i = 0; i < vec.size(); ++i) {
    if (i % w == 0 && i != 0)
      std::cout << std::endl;
    std::cout << vec[i] << "\t";
  }
  std::cout << std::endl;
}
template <class T> T averagePooling(T occ_grid) {
  Vprint(occ_grid.data, occ_grid.info.width);
  navigation_msgs::OccupancyGrid new_occ_grid;
  auto &old_g = occ_grid.data;
  auto &new_g = new_occ_grid.data;
  auto &new_w = new_occ_grid.info.width;
  auto &new_h = new_occ_grid.info.height;
  new_h = occ_grid.info.height / 2;
  new_w = occ_grid.info.width / 2;
  new_g.resize(new_w * new_h);
  new_occ_grid.info.resolution = occ_grid.info.resolution * 2;

  for (int i = 0; i < new_w; ++i) {
    for (int j = 0; j < new_h; ++j) {
      // top left corner
      int old_idx = i * 2 + j * 2;
      new_g[i * new_w + j] = (old_g[old_idx] + old_g[old_idx + 1] +
                              old_g[old_idx + occ_grid.info.width] +
                              old_g[old_idx + occ_grid.info.width + 1]) *
                             0.25;
    }
  }
  Vprint(new_g, new_w);
  return new_occ_grid;
}

template <class T> T maxPooling(T occ_grid) {
  Vprint(occ_grid.data, occ_grid.info.width);
  navigation_msgs::OccupancyGrid new_occ_grid;
  auto &old_g = occ_grid.data;
  auto &new_g = new_occ_grid.data;
  auto &new_w = new_occ_grid.info.width;
  auto &new_h = new_occ_grid.info.height;
  new_h = occ_grid.info.height / 2;
  new_w = occ_grid.info.width / 2;
  new_g.resize(new_w * new_h);
  new_occ_grid.info.resolution = occ_grid.info.resolution * 2;

  for (int i = 0; i < new_w; ++i) {
    for (int j = 0; j < new_h; ++j) {
      // top left corner
      int old_idx = i * 2 + j * 2;
      std::vector<int> pertition = {old_g[old_idx], old_g[old_idx + 1],
                                    old_g[old_idx + occ_grid.info.width],
                                    old_g[old_idx + occ_grid.info.width + 1]};
      new_g[i * new_w + j] =
          *std::max_element(pertition.begin(), pertition.end());
    }
  }
  Vprint(new_g, new_w);
  return new_occ_grid;
}

void Main() {
  navigation_msgs::OccupancyGrid occ_grid;
  /* make grid map with some unknown cell
      | 0 | 1 | 2 | 3 |
    0 | ? | ? |100|   |
    1 | ? | ? |100|   |
    2 |100|100|100|   |
    3 |   |   |   |   |
  */
  occ_grid.data = {50,  50,  100, 0, 50, 50, 100, 0,
                   100, 100, 100, 0, 0,  0,  0,   0};
  occ_grid.info.height = 4;
  occ_grid.info.width = 4;
  occ_grid.info.resolution = 0.5;
  // std::vector<double> g={1,2,3};
  // g.resize(1);

  navigation_msgs::OccupancyGrid average_pooling = averagePooling(occ_grid);
  navigation_msgs::OccupancyGrid max_pooling = maxPooling(occ_grid);
  return;
}

int main() {
  std::cin.tie(0);
  std::ios_base::sync_with_stdio(false);
  std::cout << std::fixed << std::setprecision(15);
  Main();
}
