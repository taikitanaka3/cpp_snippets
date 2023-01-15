
#include <iostream>
#include <math.h>
#include <vector>
using std::vector;
using namespace std;

namespace car_size {
const float x = 4.f;
const float y = 2.f;
} // namespace car_size

struct Vector2 {
  float x;
  float y;
  Vector2(float x, float y) {
    this->x = x;
    this->y = y;
  }
};

struct Rect {
  float r = 0.f;
  float l = 0.f;
  float f = 0.f;
  float b = 0.f;
  Rect(float r = 0.f, float l = 0.f, float f = 0.f, float b = 0.f) {
    this->r = r;
    this->l = l;
    this->f = f;
    this->b = b;
  }
  Vector2 GetCenter() {
    float cx = (r + l) * 0.5f;
    float cy = (f + b) * 0.5f;
    return Vector2(cx, cy);
  }
};

void Search(Rect &rect1, Rect &rect2) {
  float distx = abs(abs(rect1.GetCenter().x) - abs(rect2.GetCenter().x));
  float disty = abs(abs(rect1.GetCenter().y) - abs(rect2.GetCenter().y));

  if (distx < car_size::x && disty < car_size::y) {
    cout << "距離x:" << distx << "距離x:" << disty << endl;
    cout << "当たってますよ" << endl;
  }
}

void Process(vector<Rect> &rects) {
  for (int i = 0; i < rects.size(); i++) {
    for (int j = 0; j < rects.size(); j++) {
      if (i == j || i > j) {
        // cout << "もうすでに判定しているのでパス" << endl;
      } else {
        cout << "id:" << i << "と"
             << "id:" << j << "で判定します" << endl;
        Search(rects[i], rects[j]);
      }
    }
  }
}

int main() {
  vector<Rect> rects = {Rect(7.f, 11.f, 1.5f, -0.5f), Rect(6.f, 2.f, 4.f, 2.f),
                        Rect(8.f, 4.f, 3.f, 1.f), Rect(13.f, 9.f, 4.f, 2.f)};
  Process(rects);
  cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
  return 0;
}
