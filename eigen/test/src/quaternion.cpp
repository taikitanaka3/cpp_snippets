#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;
using namespace std;
using ll=long long int;
using ld=long double;
using VI=vector<ll>;
using VD=vector<ld>;
using VVI=vector<VI>;
using VC=vector<char>;
using VB=vector<bool>;
using VVC=vector<VC>;
using VS=vector<string>;
using PLL =pair<ll,ll>;
using PLD=pair<ld,ld>;
using VPLL=vector<PLL>;
#define print(x) std::cout<<x<<"\n"
#define rep(i,n) for(ll i=0;i<(ll)(n);i++)
#define repd(i,n) for(ll i=(ll)(n)-1;i>=0;i--)
#define ALL(x) (x).begin(),(x).end()
#define ALLR(x) (x).rbegin(),(x).rend()
#define SZ(x) ((ll)(x).size())
#define MAX(x) *max_element((x).begin(),(x).end())
#define MIN(x) *min_element((x).begin(),(x).end())
#define SORTR(x) sort((x).rbegin(),(x).rend())
#define SORT(x) sort((x).begin(),(x).end())
#define SUM(x) accumulate((x).begin(),(x).end(), 0)
#define FILL(x,a) fill(x.begin(),x.end(),a)
#define EACH(i,x) for(typeof((x).begin()) i=(x).begin(); i!=(x).end(); ++i)
#define EXIST(v, x) (std::find(v.begin(), v.end(), x) != v.end())

const ll INF = 1e18;
const ld EPS   = 1e-10;
const int MOD  = int(1e9)+7;
template<class T>bool chmax(T &a, const T &b) { if (a<b) { a=b; return 1; } return 0; }
template<class T>bool chmin(T &a, const T &b) { if (b<a) { a=b; return 1; } return 0; }
template <class BidirectionalIterator>
bool next_partial_permutation(BidirectionalIterator first, BidirectionalIterator middle,BidirectionalIterator last){reverse(middle, last); return next_permutation(first , last);}
ll gcd(ll x, ll y) { return (x % y)? gcd(y, x % y): y; }
ll lcm(ll x, ll y) { return x / gcd(x, y) * y; }
ll GCD(VI v){ll a = v[0]; for (ll i = 1; i<SZ(v); i++) {a = gcd(a, v[i]);} return a;}
ll LCM(VI v){ll a = v[0]; for (ll i = 1; i<SZ(v); i++) {a = lcm(a, v[i]);} return a;}
VI Bit2Vector(const ll bit, ll n) {	VI s;	rep(i,n) if (bit & (1 << i)) s.push_back(i); return s;}


void Main()
{
	// 変数を定義。値は未定義
	Quaternionf q1;

	// コンストラクタで値(w, x, y, z)を渡して初期化
	Quaternionf q2(1.0f, 0.0f, 0.25f, 0.5f);
	std::cout << q2.w() << "," << q2.x() << "," << q2.y() << "," << q2.z() << std::endl;

	// コンストラクタで角度とベクトルを渡して初期化
	Quaternionf q3(AngleAxisf(0.1f, Vector3f::UnitY()));

	// ２つのベクトルからクオータニオンを求める
	Quaternionf q4 = Quaternionf::FromTwoVectors(Vector3f::UnitX(), Vector3f::UnitZ());

	// 単位クオータニオン(wが1で他が0)
	q4 = Quaternionf::Identity();
	std::cout << q4.w() << "," << q4.x() << "," << q4.y() << "," << q4.z() << std::endl;

	// 乗算
	Quaternionf q_mul = q2 * q3;

	// 逆クオータニオン
	Quaternionf q_inv = q4.inverse();

	// 共役クオータニオンを求める
	Quaternionf q_conj = q4.conjugate();

	// 内積
	float dot = q3.dot(q4);

	// 回転ベクトルの長さ
	float norm = q3.norm();

	// 正規化
	q3.normalize();
	Quaternionf q_normalized = q4.normalized();

	// 球面線形補間
	// q3→q4を t[0, 1.0] で補間する
	float t = 0.5f;
	Quaternionf q_slerp = q3.slerp(t, q4);
	return;
}

int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(15);
	Main();
}
