
#pragma once
#define _USE_MATH_DEFINES
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <math.h>
#include <time.h>
#include <array>
using std::array;
using std::cout;
using std::endl;
using std::map;
using std::vector;



struct State
{
public:
    float x = 0.f;
    float y = 0.f;
    float th = 0.f;
    float phi = 0.f;
    State(float x = 0.f, float y = 0.f, float th = 0.f, float phi = 0.f)
    {
        this->x = x;
        this->y = y;
        this->th = th;
        this->phi = phi;
    }
};
struct Locus
{
public:
    vector<State> states = {};
    Locus(const vector<State> states = vector<State>())
    {
        this->states = states;
    }
    ~Locus(){};
};
struct Loci
{
    Header header;
    vector<Locus> loci;
    Loci(const Header header = Header(),
         const vector<Locus> loci = vector<Locus>())
    {
        this->header = header;
        this->loci = loci;
    }
    void Debug()
    {
        cout << "= id =" << header.frame_id << "==" << endl;
        cout << "seq" << header.seq << "stamp" << header.stamp << endl;
        cout << "num loci" << loci.size() << endl;
    }
};
