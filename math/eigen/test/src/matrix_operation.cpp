//http://blog.livedoor.jp/tek_nishi/archives/8623876.html
#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
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
	using namespace Eigen;

	Matrix3f m;

	// あらかじめ用意された値
	m = Matrix3f::Identity();
	m = Matrix3f::Zero();
	m = Matrix3f::Ones();
	m = Matrix3f::Random();
	m = Matrix3f::Constant(0.25f);

	// 行列の要素へのアクセス
	// matrix(列, 行)
	// 「行」が横方向のインデックス
	// 「列」が縦方向のインデックス
	// m2[0, 1] や m2[0][1] などの書き方はできない
	m(2, 0) = 1.0f;
	float value = m(0, 2);

	Matrix3f m1;
	m1 << 0.0f, 0.0f, 1.0f,
			0.0f, 0.0f, 1.0f,
			0.0f, 1.0f, 0.0f;

	// 加算・減算
	Matrix3f m2;
	m2 = m + m1;
	m2 = m - m1;

	// 乗算・減算
	m2 = m * 2.0f;
	m2 = m / 2.0f;

	// 逆行列
	m2 = m.inverse();

	// 共役行列
	m2 = m.conjugate();

	{
		// 4x4行列の一部を切り取って3x3行列へコピー
		// | a b c d |    | a b c |
		// | e f g h | -> | e f g |
		// | i j k l |    | i j k |
		// | m n o p |
		Matrix4f m_in;
		m_in <<  0.0f,  1.0f,  2.0f,  3.0f,
				4.0f,  5.0f,  6.0f,  7.0f,
				8.0f,  9.0f, 10.0f, 11.0f,
				12.0f, 13.0f, 14.0f, 15.0f;

		Matrix3f m_out = m_in.block(0, 0, 3, 3);

		// 4x4行列の一部を切り取ってベクトルへコピー
		// | a b c d |
		// | e f g h | -> | g k o |
		// | i j k l |
		// | m n o p |
		Vector3f v_out = m_in.block(1, 2, 3, 1);
	}

	// 転置行列
	// | a b c |    | a d g |
	// | d e f | -> | b e h |
	// | g h i |    | c f i |
	m2 = m.transpose();

	// 4行3列の行列を定義
	Matrix<float, 4, 3> l;

	// メンバ関数rowsが行、colsが列を返す
	std::cout << "Rows:" << l.rows() << " Cols:" << l.cols() << std::endl;

	return;
}

int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(15);
	Main();
}
