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
	Matrix3d A_(3, 3);//係数行列A
	A_ << 1, 1, 0.5,
			0, 1, 1,
			0, 0, 1;
	std::cout << "A_ = \n" << A_ << std::endl;

	Matrix3d B_(3, 3);//係数行列B
	B_ << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
	std::cout << "B_ = \n" << B_ << std::endl;;

	Vector3d Ct_; //係数行列C  列ベクトル
	Ct_ << 1, 0, 0;
	std::cout << "Ct_  = \n" << Ct_ << std::endl;
	MatrixXd C_ = Ct_.transpose();//係数行列C  行ベクトル
	std::cout << "C_  = \n" << C_ << std::endl;

	Matrix3d Q_(3, 3);//状態ノイズ
	Q_ << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
	std::cout << "Q_ = \n" << Q_ << std::endl;

	MatrixXd P_(3,3);//事前誤差共分散行列
	MatrixXd G_(3,1);//カルマンゲイン
	double R_ = 1.0;//システムノイズ
	double y[10] = { 2,4,6,8,10,12,14,16 };

	Matrix3d I_(3, 3);//単位行列
	I_ << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
	Vector3d X_;
	X_<<1,1,1;

	P_=MatrixXd::Ones(3, 3);

	//calc next state
	X_=A_*X_;//	事前状態推定
	print("X[1]= \n"<<X_);

	P_=A_*P_*A_.transpose()+B_*Q_*B_.transpose();
	print("P[1]= \n"<<P_);//	事前誤差共分散行列
	G_=P_*C_.transpose()/((C_*P_*C_.transpose())(0,0)+R_);
	//double a=INV;
	print("G[1]= \n"<<G_);//	事前誤差共分散行列

	MatrixXd X_1=X_+G_*(y[0]-(C_*X_)(0,0));
	print("X1[1]= \n"<<X_1);//	事前誤差共分散行列
	MatrixXd P_1=(I_-G_*C_)*P_;
	print("P1[1]= \n"<<P_1);//	事前誤差共分散行列
	return;
}

int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(15);
	Main();
}
