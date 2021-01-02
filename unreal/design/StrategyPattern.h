#pragma once
#include <iostream>

//制御用抽象クラス
class AbstractControl
{
public:
    virtual void ControlMethod(Geometry geometry){};
};

//制御用クラス 制御内容はStrategy Pattern で変更可能
class ControlStrategy
{
public:
    ControlStrategy(AbstractControl *controlMethod)
    {
        this->controlMethod = controlMethod;
    }
    void ControlMethod(Geometry geometry)
    {
        this->controlMethod->ControlMethod(geometry);
    }

private:
    AbstractControl *controlMethod;
};

class LinearFeedback : public AbstractControl
{
public:
    void Control(Geometry geometry)
    {
        //float d = geometry.twist.linear.x * pos2Goal.y - geometry.twist.linear.y * pos2Goal.x;
        //steering = Gp_steer * d + Gd_steer * (d - d_old) / header.deltaTime;
        //d_old = d;
    }

private:
    float Gp_steer = 50.0f;
    float Gd_steer = 15.0f;
    float d_old = 0.f;
};