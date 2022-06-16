//#include <time.h>//乱数初期化用

#include <ctime>
#include <iostream>
#include <random>
#include <vector>

using namespace std;

class NumberPublisher;

/**
 *  @brief オブザーバー用抽象クラス
 **/
class Subscriber {
public:
  //オブザーバにNotifyする時に呼ばれる関数
  virtual void Update(NumberPublisher *publisher) = 0;
};

/**
 *  @brief 数字作成の抽象クラス
 **/
class NumberPublisher {
public:
  /**
   *  @brief Observerを追加する関数
   */
  void AddSubscriber(Subscriber *Subscriber) {
    observers_.push_back(Subscriber);
  }

  /**
   *  @brief 登録されたObserverに通知を実施する関数
   */
  void Notify(void) {
    int nobserver = observers_.size();
    for (int io = 0; io < nobserver; io++) {
      //登録されたすべてのオブザーバのUpdate関数を呼ぶ
      observers_[io]->Update(this);
    }
  }

  virtual int GetNum(void) = 0;
  virtual void Publish(void) = 0;

private:
  vector<Subscriber *> observers_; //オブザーバのポインタのリスト
};

/**
 *  @brief 数字を表示するオブザーバー
 */
class DigitObserver : public Subscriber {
public:
  /**
   *  @brief Updateされた時に数字として表示する関数
   */
  void Update(NumberPublisher *publisher) {
    cout << "DigitObserver:" << publisher->GetNum() << endl;
  }

private:
};

/**
 *  @brief 数字をスターで表示するオブザーバー
 */
class StarObserver : public Subscriber {
public:
  /**
   *  @brief Updateされた時に*として表示する関数
   */
  void Update(NumberPublisher *publisher) {
    cout << "DigitObserver:";
    for (int i = 0; i < publisher->GetNum(); i++) {
      cout << "*";
    }
    cout << endl;
  }

private:
};

/**
 *  @brief 乱数発生クラス
 **/
class RandomNumberPublisher : public NumberPublisher {
public:
  RandomNumberPublisher(void) {}

  /**
   *  @brief 生成された乱数を返す関数
   */
  int GetNum(void) { return number_; }

  /**
   *  @brief 乱数を10個発生させ、オブザーバに通知する関数
   */
  void Publish(void) {
    // メルセンヌ・ツイスター法による擬似乱数生成器を、
    // ハードウェア乱数をシードにして初期化
    mt19937 engine_(static_cast<unsigned int>(time(nullptr)));
    uniform_int_distribution<int> dist(0, 100);
    for (int i = 0; i < 10; i++) {
      number_ = 1;
      Notify(); //登録されたオブザーバーに通知
    }
  }

private:
  int number_;     //生成した値
  mt19937 engine_; //乱数発生エンジン
};

int main(void) {
  cout << "Subscriber Pattern Sample Start!!" << endl;

  NumberPublisher *publisher = new RandomNumberPublisher();
  Subscriber *ob1 = new DigitObserver();
  Subscriber *ob2 = new StarObserver();

  //オブザーバーの登録
  publisher->AddSubscriber(ob1);
  publisher->AddSubscriber(ob2);

  //数字の発生
  publisher->Publish();

  return 0;
}
