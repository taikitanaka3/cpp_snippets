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
using PLL =pair<ll, ll>;
using PLD=pair<ld, ld>;
using VPLL=vector<PLL>;
#define print(x) std::cout<<x<<"\n"
#define rep(i, n) for(ll i=0;i<(ll)(n);i++)
#define repd(i, n) for(ll i=(ll)(n)-1;i>=0;i--)
#define ALL(x) (x).begin(),(x).end()
#define ALLR(x) (x).rbegin(),(x).rend()
#define SZ(x) ((ll)(x).size())
#define MAX(x) *max_element((x).begin(),(x).end())
#define MIN(x) *min_element((x).begin(),(x).end())
#define SORTR(x) sort((x).rbegin(),(x).rend())
#define SORT(x) sort((x).begin(),(x).end())
#define SUM(x) accumulate((x).begin(),(x).end(), 0)
#define FILL(x, a) fill(x.begin(),x.end(),a)
#define EACH(i, x) for(typeof((x).begin()) i=(x).begin(); i!=(x).end(); ++i)
#define EXIST(v, x) (std::find(v.begin(), v.end(), x) != v.end())

const ll INF = 1e18;

const ld EPS = 1e-10;

const int MOD = int(1e9)+7;

template<class T> bool chmax(T& a, const T& b)
{
	if (a<b) {
		a = b;
		return 1;
	}
	return 0;
}
template<class T> bool chmin(T& a, const T& b)
{
	if (b<a) {
		a = b;
		return 1;
	}
	return 0;
}
template<class BidirectionalIterator>
bool next_partial_permutation(BidirectionalIterator first, BidirectionalIterator middle, BidirectionalIterator last)
{
	reverse(middle, last);
	return next_permutation(first, last);
}
ll gcd(ll x, ll y) { return (x%y) ? gcd(y, x%y) : y; }
ll lcm(ll x, ll y) { return x/gcd(x, y)*y; }
ll GCD(VI v)
{
	ll a = v[0];
	for (ll i = 1; i<SZ(v); i++) { a = gcd(a, v[i]); }
	return a;
}
ll LCM(VI v)
{
	ll a = v[0];
	for (ll i = 1; i<SZ(v); i++) { a = lcm(a, v[i]); }
	return a;
}
VI Bit2Vector(const ll bit, ll n)
{
	VI s;
	rep(i, n) if (bit & (1 << i)) s.push_back(i);
	return s;
}

/*
struct PossibleCollision
{
	lanelet::BasicPoint2d obstacle;
	lanelet::BasicPoint2d collision;
	lanelet::ConstLanelet obstacle_lanelet;
	double ego_distance;
	double obstacle_distance;
	double obstacle_max_vel;
	bool vehicle_obstacle;  // if false, assume a bicycle instead

	PossibleCollision(
			lanelet::BasicPoint2d obstacle_, lanelet::BasicPoint2d collision_,
			lanelet::ConstLanelet obstacle_lanelet_, double ego_distance_, double obstacle_distance_,
			double obstacle_max_vel_, bool vehicle_obstacle_)
			: obstacle(obstacle_),
			  collision(collision_),
			  obstacle_lanelet(obstacle_lanelet_),
			  ego_distance(ego_distance_),
			  obstacle_distance(obstacle_distance_),
			  obstacle_max_vel(obstacle_max_vel_),
			  vehicle_obstacle(vehicle_obstacle_)
	{
	}
};*/

struct PlannerParam {
	double min_road_obstacle_vel;  // m/s
	double sidewalk_obstacle_vel;  // m/s
	double max_decel;              // m/s^2
	double max_blindspot_dist;     // m
	double min_ego_velocity;       // m/s
	double vehicle_width;          // m
	double vehicle_length;         // m
	bool filter_passable_collisions;
	bool filter_lanelets_with_traffic_lights;
	double safety_time_buffer;     // s
	bool road_obstacles_enabled;
	bool sidewalk_obstacles_enabled;
	bool other_obstacles_enabled;
};

struct PossibleCollision {
	double dist_to_ego;

};

inline double calculateSafeVelocity(double curr_ego_distance, double max_decel)
{
	return std::sqrt(2.0*-max_decel*curr_ego_distance);
}

inline double calculateSafeVelocity(double curr_traj_vel,double curr_ego_distance, double max_decel)
{
	struct Scene {
		const double kmh2ms=1.0/3.6;
		double national_highway = 60.0*kmh2ms;
		double public_road = 40.0*kmh2ms;
		double private_road = 30.0*kmh2ms;
		double pedestrian_road = 25.0*kmh2ms;
	};
	Scene scene;
	double safe_velocity=0;
	// ignore high speed way
	if(curr_traj_vel>scene.national_highway) return curr_traj_vel;
	else if(curr_traj_vel>scene.public_road) {
		safe_velocity=curr_traj_vel=35.0;
	}
	else if(curr_traj_vel>scene.private_road) {
		safe_velocity=curr_traj_vel=25.0;
	}
	else if(curr_traj_vel>scene.private_road) {
		safe_velocity=curr_traj_vel=20.0;
	}
	else if(curr_traj_vel>scene.pedestrian_road) {
		safe_velocity=curr_traj_vel=5.0;
	}
	double dist_with_previous_point=10.0;
	double min_vel = curr_traj_vel+dist_with_previous_point/curr_traj_vel*max_decel;
	// ensure we only lower the original velocity (and do not increase it)
	safe_velocity = std::min(safe_velocity, curr_traj_vel);
	return safe_velocity;
}

autoware_planning_msgs::Trajectory applySafeVelocity(
		const autoware_planning_msgs::Trajectory& trajectory,
		const std::vector<PossibleCollision>& possible_collisions, int curr_traj_point_index,
		const PlannerParam& param)
{
	// Apply safe velocities

	autoware_planning_msgs::Trajectory safe_trajectory;
	//safe_trajectory.header.frame_id = trajectory.header.frame_id;
	//safe_trajectory.header.stamp = trajectory.header.stamp;
	// copy points up to and including the current one
	int i(0);
	for (; i<std::max(1, curr_traj_point_index); ++i)
		safe_trajectory.points.emplace_back(trajectory.points[i]);
	// adjust the velocity of the remaining trajectory points
	/*
	double dist_along_traj(0);
	for (; i<trajectory.points.size(); ++i) {
		const double dist_with_previous_point =
				distance(trajectory.points[i-1].pose, trajectory.points[i].pose);
		dist_along_traj += dist_with_previous_point;
		double safe_vel = trajectory.points[i].twist.linear.x;
		for (const auto& possible_collision : possible_collisions) {
			const double dist_to_collision = possible_collision.ego_distance-dist_along_traj-param.vehicle_length;
			if (dist_to_collision>=0) {
				const double collision_avoidance_vel = std::max(
						param.min_ego_velocity,
						calculateSafeVelocity(dist_to_collision, param.max_decel));
				safe_vel = std::min(safe_vel, collision_avoidance_vel);
			}
		}
		double prev_vel = safe_trajectory.points.back().twist.linear.x;
		if (prev_vel>0) {
			// minimum velocity calculated by applying the maximum deceleration (v = v0 + Δt * a)
			double min_vel = prev_vel+dist_with_previous_point/prev_vel*param.max_decel;
			safe_vel = std::max(safe_vel, min_vel);
			// ensure we only lower the original velocity (and do not increase it)
			safe_vel = std::min(safe_vel, trajectory.points[i].twist.linear.x);
		}
		autoware_planning_msgs::TrajectoryPoint safe_traj_point(trajectory.points[i]);
		safe_traj_point.twist.linear.x = safe_vel;
		safe_trajectory.points.push_back(safe_traj_point);
	}*/
	return safe_trajectory;
}

double velocity_planning()
{
	double dist_along_traj(0);
	double original_trajectory_vel = 20;
	const double dist_with_previous_point = 10;
	const double possible_collision_ego_distance = 15;
	PlannerParam param;

	/**
	 * min_ego_vel
	 *
	 *
	 *
	 */


	param.vehicle_length = 4.0;
	param.min_ego_velocity = 5.0;
	param.max_decel = 2.0;
	double nearest_trajectory_vel = 15.0;

	dist_along_traj += dist_with_previous_point;
	double safe_vel = original_trajectory_vel;
	const double dist_to_collision = possible_collision_ego_distance-dist_along_traj-param.vehicle_length;
	// only if collision dist > 0
	if (dist_to_collision>=0) {
		const double collision_avoidance_vel = std::max(
				param.min_ego_velocity,
				calculateSafeVelocity(dist_to_collision, param.max_decel));
		safe_vel = std::min(safe_vel, collision_avoidance_vel);
	}
	// only use for forward move
	if (nearest_trajectory_vel>0) {
		// minimum velocity calculated by applying the maximum deceleration (v = v0 + Δt * a)
		double min_vel = nearest_trajectory_vel+dist_with_previous_point/nearest_trajectory_vel*param.max_decel;
		safe_vel = std::max(safe_vel, min_vel);
		// ensure we only lower the original velocity (and do not increase it)
		safe_vel = std::min(safe_vel, original_trajectory_vel);
	}
	return safe_vel;
}

void Main()
{

	return;
}

int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(15);
	Main();
}
