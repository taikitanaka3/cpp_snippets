#pragma once
#include <iostream>

/**
 *  @brief
 処理フローのテンプレートクラス
 */
class TemplateMethod
{
public:
    virtual ~TemplateMethod() = default;
    //1度のみ処理のの実行
    virtual void SpinOnce(void){};
    //状態取得
    virtual void GetState(void){};
    //状態設定
    virtual void SetState(void){};
    virtual void Start(void){};
    /**
     *  @brief 処理実行関数
    */
    virtual void Update(void) final
    {
        if (!bInitialized)
        {
            Start();
            std::cout << "Initialized At Begin Play" << std::endl;
            bInitialized = true;
        }
        GetState();
        SpinOnce();
        SetState();
    }

private:
    bool bInitialized = false;
};