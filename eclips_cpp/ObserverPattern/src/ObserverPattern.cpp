//#include <time.h>//乱数初期化用

#include <iostream>
#include <vector>
#include <random>
#include <ctime>

using namespace std;

class TopicGenerator;

/**
 *  @brief オブザーバー用抽象クラス
 **/
class Observer{
  public:
    //オブザーバにNotifyする時に呼ばれる関数
    virtual void Update(TopicGenerator *topic)=0;
};

/**
 *  @brief 数字作成の抽象クラス
 **/
class TopicGenerator{
  public:
   /**
    *  @brief Observerを追加する関数
    */
    void AddObserver(Observer *observer){
      observers_.push_back(observer);
    }

   /**
    *  @brief 登録されたObserverに通知を実施する関数
    */
    void Notify(void){
      int nobserver=observers_.size();
      for(int io=0;io<nobserver;io++){
        //登録されたすべてのオブザーバのUpdate関数を呼ぶ
        observers_[io]->Update(this);
      }
    }

    virtual int GetNum(void)=0;
    virtual void Publish(void)=0;

  private:
    vector<Observer*> observers_;//オブザーバのポインタのリスト
};

/**
 *  @brief 数字を表示するオブザーバー
 */
class DigitObserver:public Observer{
  public:
   /**
    *  @brief Updateされた時に数字として表示する関数
    */
    void Update(TopicGenerator *topic){
      cout<<"DigitObserver:"<<topic->GetNum()<<endl;
    }
  private:
};

/**
 *  @brief 数字をスターで表示するオブザーバー
 */
class StarObserver:public Observer{
  public:
   /**
    *  @brief Updateされた時に*として表示する関数
    */
    void Update(TopicGenerator *topic){
      cout<<"DigitObserver:";
      for(int i=0;i<topic->GetNum();i++){
        cout<<"*";
      }
      cout<<endl;
    }
  private:
};


/**
 *  @brief 乱数発生クラス
 **/
class NumberInfoGenerator:public TopicGenerator{
  public:
    NumberInfoGenerator(void){}

   /**
    *  @brief 生成された乱数を返す関数
    */
    int GetNum(void){return number_;}

   /**
    *  @brief 乱数を10個発生させ、オブザーバに通知する関数
    */
    void Publish(void){
      // メルセンヌ・ツイスター法による擬似乱数生成器を、
      // ハードウェア乱数をシードにして初期化
      for(int i=0;i<10;i++){
        number_++;
        Notify();//登録されたオブザーバーに通知
      }
    }

  private:
    int number_;//生成した値
};


int main(void){
  cout<<"Observer Pattern Sample Start!!"<<endl;

  TopicGenerator* topic=new NumberInfoGenerator();
  Observer *ob1=new DigitObserver();
  Observer *ob2=new StarObserver();

  //オブザーバーの登録
  topic->AddObserver(ob1);
  topic->AddObserver(ob2);

  //数字の発生
  topic->Publish();

  return 0;
}
