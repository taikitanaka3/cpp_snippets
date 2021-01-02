#pragma once
#include <iostream>
#include <map>
#include <string>
using std::string;
/*
    *他の人が車両の諸元を見て入力する設定
    *あくまでも車両の設定のみを記入すること
    
*/
namespace vehicle
{
const float MAX_ACCELERATION = 1.f;
const float MAX_ANG_ACCELERATION = float(30 * M_PI / 180);

const float MAX_LINEAR_VELOCITY = 30.f;
const float MAX_ANGULAR_VELOCITY = float(30 * M_PI / 180);

const float LIMIT_MAX_ANG_VELO = float(M_PI * 45 / 180);
const float LIMIT_MIN_ANG_VELO = float(-M_PI * 45 / 180);
const float LIMIT_MAX_VELO = 40.f;
const float LIMIT_MIN_VELO = 0.f;
const float CAR_LENGTH = 2.4f;

namespace limmit
{
const float STEERING_LIMMIT = 0.7f;
const float THROTTLE_LIMMIT = 0.85f;
} // namespace limmit

} // namespace vehicle

namespace safety
{
static std::map<string, const float> auto_cruise =
    {
        {"ennable", 15.f},
        {"follow", 10.f},
        {"disable", 5.f}};

static std::map<string, const float> distance =
    {
        {"Detected", 15.f},
        {"Warning", 6.f},
        {"Danger", 4.f},
        {"Crash", 2.f},
        };
//ref https://www.roads.maryland.gov/OPPEN/VISSIM%20Modeling%20Guidance%209-12-2017.pdf

//const param
const float STANDSTILL_DISTANCE = 6.5f; //m
const float ADDITIVESAFETY_DISTANCE = 2.f;
const float FOLLOWING_VARIATION = 2.f; //how much the lagging vehicle intensionally move close to
const float LANE_CHANGE_DURATION = 2.f;
const float TIME_TO_COLLISION = 2.f;

} // namespace safety

/*
速度 	空走距離 	制動距離 	停止距離
２０キロメートル毎時 	６ｍ 	３ｍ 	９ｍ
３０キロメートル毎時 	８ｍ 	６ｍ 	１４ｍ
４０キロメートル毎時 	１１ｍ 	１１ｍ 	２２ｍ
５０キロメートル毎時 	１４ｍ 	１８ｍ 	３２ｍ
６０キロメートル毎時 	１７ｍ 	２７ｍ 	４４ｍ
７０キロメートル毎時 	１９ｍ 	３９ｍ 	５８ｍ
８０キロメートル毎時 	２２ｍ 	５４ｍ 	７６ｍ
９０キロメートル毎時 	２５ｍ 	６８ｍ 	９３ｍ
１００キロメートル毎時 	２８ｍ 	８４ｍ 	１１２ｍ
*/