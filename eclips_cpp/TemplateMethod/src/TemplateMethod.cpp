#include <iostream>
using namespace std;

/**
 *  @brief
 処理フローのテンプレートクラス
 */
class AbstractState
{
  public:
    //処理の始まり
    virtual void Start(void){};
    //1度のみ処理のの実行
    virtual void SpinOnce(void){};
    //状態取得
    virtual void GetState(void){};
    //状態設定
    virtual void SetState(void){};
    virtual void OverrideErrorFunc(){};
    /**
     *  @brief 処理実行関数
     */

    void Update(void)
    {
        GetState();
        SpinOnce();
        SetState();
        cout<<"s"<<bInitialized;
        bInitialized=false;
    }
    public:
    bool bInitialized=true;
};

/**
 *  @brief シンプルな処理機能クラス
 */
class IMU :public AbstractState
{
  public:
    IMU(){
            cout<<"this init"<<bInitialized<<endl;
    }

    void OverrideErrorFunc() override;
    void Start(void)
    {
        cout << "Start+";
        cout << "+" << endl;
        this->bInitialized=true;
    }

    void GetState(void)
    {
        cout << "Get" << "|" << endl;
    }

    void SetState(void)
    {
        cout << "Set" << "|" << endl;
    }
    void Update();
  private:
    bool bInitialized=false ;
};

void IMU::OverrideErrorFunc(){
	return;
}


int main(void)
{
    cout << "TemplateMethod Pattern Sample Start!!" << endl;

    //２つのサブクラスのポインタをスーパークラスに代入する
    AbstractState *sim = new IMU();
    AbstractState *sim2 = new IMU();
    //sim->Start();
    sim->Update();
    sim2->Update();
    //cout<<"super"<<sim->bInitialized;
    return 0;
}
