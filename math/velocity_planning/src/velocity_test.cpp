#include <bits/stdc++.h>
#include <Trajectory.h>
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

// curr_traj_vel[m/s]
double calcSafeVelocity(double curr_traj_vel){
	struct Scene {
		const double kmh2ms=1.0/3.6;
		const double ms2kmh=1.0/3.6;
		double national_highway = 60.0*kmh2ms;
		double public_road = 40.0*kmh2ms;
		double private_road = 30.0*kmh2ms;
		double pedestrian_road = 25.0*kmh2ms;
	};
	Scene scene;
	double safe_vel=curr_traj_vel;
	// ignore high speed way
	if(curr_traj_vel>scene.national_highway) {
		return curr_traj_vel;
	}
	else if(curr_traj_vel>scene.public_road) {
		safe_vel=35.0*scene.ms2kmh;
	}
	else if(curr_traj_vel>scene.private_road) {
		safe_vel=30.0*scene.ms2kmh;
	}
	else if(curr_traj_vel>scene.pedestrian_road) {
		safe_vel=20.0*scene.ms2kmh;
	}
	else{
		safe_vel=5.0*scene.ms2kmh;
	}
	return safe_vel;
}

void Main()
{
	autoware_planning_msgs::Trajectory safe_trajectory;
	for (int i = 0; i < 70; i++) {
		double v=i*1.0/3.6;
		autoware_planning_msgs::TrajectoryPoint point;
		auto &traj_v=point.twist.linear.x;
		traj_v=calcSafeVelocity(v);
		safe_trajectory.points.emplace_back(point);
		print("v: "<<v*3.6<<"\t"<<"safev: "<<traj_v*3.6);
	}
	return;
}

int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(15);
	Main();
}
