#define _USE_MATH_DEFINES
#include <bits/stdc++.h>
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



struct Point{
    double x;
    double y;
    double z;
};
struct Rpy{
    double r;
    double p;
    double y;
};

struct Pose{
    Point position;
    Rpy rpy;
};

struct Lane{
    vector<Pose> poses;
};

bool distance_threshold(double positional_dist,double posture_dist){
    const double posisitional_threshold=2.0;
    const double posture_threshold=M_PI*0.1;
    if(positional_dist>posisitional_threshold) return false;
    if(posture_dist>posture_threshold) return false;
    return true;
}

void Main()
{
	ll n,m,l; ll res=0;
	string s,t,u; string sres="No or NO";

    vector<Pose> vp;
    vp.emplace_back(Pose{Point{1,2,0},Rpy{0,0,0}});
    vp.emplace_back(Pose{Point{2,2,0},Rpy{0,0,0}});
    vp.emplace_back(Pose{Point{3,2,0},Rpy{0,0,0}});
    Lane lane1={vp};
    vp.clear();
    vp.emplace_back(Pose{Point{1,1,0},Rpy{0,0,M_PI_2}});
    vp.emplace_back(Pose{Point{2,2,0},Rpy{0,0,M_PI_2}});
    vp.emplace_back(Pose{Point{3,3,0},Rpy{0,0,M_PI_2}});
    Lane lane2={vp};
    vp.clear();
    vp.emplace_back(Pose{Point{0,2,0},Rpy{0,0,M_PI}});
    vp.emplace_back(Pose{Point{-1,2,0},Rpy{0,0,M_PI}});
    vp.emplace_back(Pose{Point{-2,2,0},Rpy{0,0,M_PI}});
    Lane lane3={vp};
    //lanes is lane vector
    vector<Lane> lanes={lane1,lane2,lane3};
    //target lane to find
    Lane nearest_lane;
    //assume vehicle is near to first lane
    Pose vehicle_pose=Pose{Point{1,3,0},Rpy{0,0,0}};
    double min_cost=INF;
    for(auto &lane:lanes){
        Pose lane_top=lane.poses[0];
        double dx=vehicle_pose.position.x-lane_top.position.x;
        double dy=vehicle_pose.position.x-lane_top.position.x;
        double dth=vehicle_pose.rpy.y-lane_top.rpy.y;
        double positional_dist=hypot(dx,dy);
        double posture_dist=abs(dth);
        print("d_pos: "<<positional_dist<<" d_rad: "<<posture_dist);
        if(!distance_threshold(positional_dist,posture_dist)) continue;
        double cost=positional_dist/2.0+posture_dist/(M_PI*0.1);
        if(chmin(min_cost,cost)){
            nearest_lane=lane;
        }
    }

    print("x y z");
    for(auto p:nearest_lane.poses){
        print(p.position.x<<" "<<p.position.y<<" "<<p.position.z);
    }
    return;
}

/*
d_pos: 0.00000 d_rad: 0.00000
d_pos: 0.00000 d_rad: 1.57080
d_pos: 1.41421 d_rad: 3.14159
x y z
1.00000 2.00000 0.00000
2.00000 2.00000 0.00000
3.00000 2.00000 0.00000
 */
int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(5);
	Main();
}
