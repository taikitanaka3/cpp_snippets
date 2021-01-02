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


class UniquePointer{
private:
	unique_ptr<int> unique_pointer_;
	unique_ptr<int> reset_uptr;
public:
	UniquePointer(int val):unique_pointer_(new int(val)){}
	void reset_value(int val){reset_uptr.reset(new int(val));}
	int get_value()const{return *unique_pointer_;}
	void set_value(int val){unique_pointer_.reset(new int(val));}
};

void simple_unique_pointer(){
	unique_ptr<int> unique_pointer(new int(10));
	rep(i,10){
		*unique_pointer+=i;
		print("current i="<<*unique_pointer);
	}
	print("unique_pointer="<<*unique_pointer);
	unique_ptr<string> ptr(new string("aaa"));
	bool b=ptr.operator bool();// if it has ownership
	print(b);
	std::unique_ptr<std::string> pStr(new std::string("print"));
	print(*pStr);
	unsigned int StrSize = pStr->size();
	
	unique_ptr<int[]> ptrArray(new int[10]);
	rep(i,10){
		ptrArray[i]=i;
		print("array i:"<<i);
	}

}

int main(){
	simple_unique_pointer();
	UniquePointer uniquePointer(999);
	UniquePointer uniquePointer2(move(uniquePointer));
	print("unique ptr="<<uniquePointer2.get_value());
	uniquePointer2.set_value(100);
	print("unique ptr="<<uniquePointer2.get_value());
	try {
		uniquePointer.get_value();
		//UniquePointer uniquePointer2(uniquePointer);
	}
	catch(exception e){
		print("Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)");
	}

	return 0;
}//ディストラクタ内で、自動的にptrの中身をdelete
