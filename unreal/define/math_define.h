#pragma once
#define _USE_MATH_DEFINES
#include "define/core_define.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
#include <random>
#include <chrono> //for std clock
#include <algorithm>

using std::cout;
using std::endl;
using std::map;
using std::vector;

const static float DEG2RAD = M_PI / 180.f;
const static float RAD2DEG = 180.f * M_1_PI;
const static float PI_2 = M_PI * 2.f;

struct Math
{
    static Vector3 Derivative(const Vector3 current, const Vector3 old, const float dt)
    {
        Vector3 deri = Vector3::Div(Vector3::Sub(current, old), dt);
        return deri;
    }
    static float Deg2Rad(float deg) { return deg * M_PI / 180.f; }
    static float Rad2Deg(float rad) { return rad * 180.f / M_PI; }

    static float AngleRangeCorrector(float angle)
    {
        if (angle > M_PI)
        {
            while (angle > M_PI)
            {
                angle -= 2 * M_PI;
            }
        }
        else if (angle < -M_PI)
        {
            while (angle < -M_PI)
            {
                angle += 2 * M_PI;
            }
        }
        return angle;
    }

    template <typename T>
    static int sgn(T val) { return (T(0) < val) - (val < T(0)); }

    template <typename T>
    static T Threashold(T val, T min, T max)
    {
        if (val < min || max < val)
            val = 0;
        return val;
    }

    template <typename T>
    static T Saturation(T val, T min, T max)
    {
        if (val < min)
        {
            val = min;
        }
        else if (max < val)
        {
            val = max;
        }
        return val;
    }

    template <typename T>
    static T Clamp(T val, T limmit)
    {
        if (val > limmit)
            val = limmit;
        return val;
    }

    static void VectorMinMaxNormalize(std::vector<float> &values)
    {
        float min = *std::min_element(values.begin(), values.end());
        float max = *std::max_element(values.begin(), values.end());
        for (int i = 0; i < values.size(); i++)
        {
            if (max - min == 0)
            {
                values[i] = 0;
            }
            else
            {
                values[i] = (values[i] - min) / (max - min);
            }
        }
    }

    static float RandomNoise(float x, const float mean = 0.f, const float stddev = 0.1f)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::normal_distribution<float> dist(mean, stddev);
        x = x + dist(generator);
        return x;
    }

    static vector<float> SetCovarianceMatrix(vector<vector<float>> data)
    {
        vector<float> covarianceMatrix(data.size() * data.size(), 0);
        cout << "    covariance matrix>>>>" << endl;
        cout << data.size() << endl;
        for (int i = 0; i < data.size(); i++)
        {
            for (int j = 0; j < data.size(); j++)
            {
                int index = i + data.size() * i;
                covarianceMatrix[index] = SetCovarianceComponent(data[i], data[j]);
                cout << covarianceMatrix[index] << endl;
            }
        }
        cout << "    <<<<covariance matrix" << endl;
        return covarianceMatrix;
    }

    static float SetCovarianceComponent(vector<float> &x, vector<float> &y)
    {
        if (x.size() != y.size())
        {
            cout << "different array size" << endl;
        }
        float len = x.size();
        float avgx = std::accumulate(x.begin(), x.end(), 0) / len;
        float avgy = std::accumulate(y.begin(), y.end(), 0) / len;
        //cout << "average x"<<avgx<<endl;
        //cout << "average y"<<avgy<<endl;
        float coefficient = 0.f;
        for (int i = 0; i < len; i++)
        {
            coefficient += (x[i] - avgx) * (y[i] - avgy) / len;
        }
        coefficient /= len;
        //cout<<"coeffcient"<<coefficient<<endl;
        return coefficient;
    }
};
