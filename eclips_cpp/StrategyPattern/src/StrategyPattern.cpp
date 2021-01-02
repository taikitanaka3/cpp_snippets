#include <iostream>
#include <vector>

using namespace std;


//制御用抽象クラス
class AbstractControl
{
  public:
    virtual void Control(int c){};
};

//ストラテジーの1つ
class LinearFeedback : public AbstractControl
{
  public:
    void Control(int c)
    {
        return ;
    }
};
//ストラテジーの1つ
class PurePursuit : public AbstractControl
{
  public:
    void Control(int c)
    {
        cout << "ppc" << endl;
        return ;
    }
};
//制御用クラス 制御内容はStrategy Pattern で変更可能
class ControlStrategy
{
  public:
    ControlStrategy(AbstractControl *control)
    {
        this->control_= control;
    }
    void Control(int c)
    {
        this->control_->Control(c);
    }

  private:
    AbstractControl *control_;
};

int main(void)
{
    cout << "Strategy Method Pattern Sample Start!!" << endl;

    ControlStrategy ctr(new PurePursuit());
    ctr.Control(1);

    return 0;
}
