#include <chrono>
#include <iostream>
using std::cout;
#include <ratio>
#include <thread>
using std::chrono::time_point;

void f() { std::this_thread::sleep_for(std::chrono::seconds(1)); }

int main() {
  std::chrono::system_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  std::chrono::system_clock::time_point t0 =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> tm = t0 - t1;

  f();
  auto t2 = std::chrono::high_resolution_clock::now();

  // floating-point duration: no duration_cast needed
  std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

  // integral duration: requires duration_cast
  auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
  // cout<<typeid(int_ms).name;
  //  converting integral duration to integral duration of shorter divisible
  //  time unit: no duration_cast needed
  std::chrono::duration<long, std::micro> int_usec = int_ms;

  std::cout << "f() took " << fp_ms.count() << " ms, "
            << "or " << int_ms.count() << " whole milliseconds "
            << "(which is " << int_usec.count() << " whole microseconds)"
            << std::endl;
}
