#!/bin/bash

cd $(dirname $0)
dir=$(pwd)

echo $dir

contest=$1
number=$2

echo "Create contest directory..."
mkdir $contest
cd $contest

echo "Create contest number directory..."
mkdir $number
cd $number

echo "Create files."

# cpp file template
template=$(
    cat <<EOS
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


void Main()
{
	ll n,m,l; ll res=0;
	string s,t,u; string sres="No or NO";

	cin>>n;
	VI a(n),b(n);
	rep(i, n) cin >> a[i];

	ll h=100,w=100;

	VVI	mp(h,VI(w,0));
	VVC	grid(h,VC(w,'.'));
	rep(j, h){
		rep(i,w){
			grid[j][i]='#';
		}
	}

	cout << res << "\n";
	return;
}

int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(15);
	Main();
}


EOS
)

filelist=("a" "b" "c" "d" "e" "f")

for file in ${filelist[@]}; do
    touch ${file}.cpp
    echo "$template" >${file}.cpp
    echo "created file: ${file}.cpp"
done

echo "Add executable into CMakeLists.txt"

cd ..

# Write settings into CMakeLists.txt
echo "" >>./CMakeLists.txt
echo "cmake_minimum_required(VERSION 3.10)" >>./CMakeLists.txt
#echo "project($contest)">> ./CMakeLists.txt
echo "set(CMAKE_CXX_STANDARD 17)" >>./CMakeLists.txt

echo "# Contest $contest $number" >>./CMakeLists.txt
for file in ${filelist[@]}; do
    echo "add_executable(${number}_${file} $number/${file}.cpp)" >>./CMakeLists.txt
done
