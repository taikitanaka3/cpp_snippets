#pragma once
#define _USE_MATH_DEFINES
#include "design/TemplateMethod.h"
#include "define/math_define.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"
#include "define/trajectory_define.h"
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

struct CollisionCheck
{
private:
    CollisionCheck();
    ~CollisionCheck();

public:
    static ObjPose calc_nearest_obs(const Pose &pose, ObjPose &allObj, float collisionDist = 5)
    {
        ObjPose nearObj;
        for (int i = 0; i < allObj.obj.poses.size(); i++)
        {
            float dis2obs = hypot(
                pose.position.x - allObj.obj.poses[i].position.x,
                pose.position.y - allObj.obj.poses[i].position.y);
            if (dis2obs < collisionDist)
            {
                nearObj.obj.poses.emplace_back(allObj.obj.poses[i].position);
            }
        }
        return nearObj;
    }
};


namespace cost{
const float WEIGHT_ANGLE = 0.15f;
const float WEIGHT_VELOCITY = 0.14f;
const float WEIGHT_OBSTACLE = 0.15f;
}

struct CostFunction
{
private:
    CostFunction();
    ~CostFunction();

public:
    static float score_heading_angle(Locus &locus, GetPlan &getPlan)
    {
        int pathLength=locus.states.size();
        float angle_to_goal = atan2(getPlan.goal.position.y - locus.states[pathLength].x, getPlan.goal.position.x - locus.states[pathLength].y);
        float score_angle = angle_to_goal - locus.states[pathLength].th;
        score_angle = abs(Math::AngleRangeCorrector(score_angle));
        score_angle = M_PI - score_angle;
        return score_angle;
    }

    static float score_heading_velo(float velocity)
    {
        return velocity;
    }

    static float score_obstcle(Locus &locus, ObjPose &nearObj)
    {
        float score_obstacle = 2;
        float temp_dis_to_obs = 0.0;
        for (int j = 0; j < locus.states.size(); j++)
        { //for i in range(len(locus.x)):
            for (int i = 0; i < nearObj.obj.poses.size(); i++)
            {
                //for obs in nearest_obs:
                temp_dis_to_obs = hypot(
                    locus.states[j].x - nearObj.obj.poses[i].position.x,
                    +locus.states[j].x - nearObj.obj.poses[i].position.y);
                //最大スコアよりも小さいスコアならば入れ替え
                if (temp_dis_to_obs < score_obstacle)
                {
                    score_obstacle = temp_dis_to_obs;
                }
                //ここは改善の余地あり
                //ぶつかったら即終了
                if (temp_dis_to_obs < 1.f)
                {
                    score_obstacle = -1;
                    return score_obstacle;
                }
            }
        }
        return score_obstacle;
    }
    static Locus evaluate(Loci &loci,GetPlan &getPlan, Pose &pose,ObjPose &ObjPose)
    {
        CollisionCheck::calc_nearest_obs(pose, ObjPose);
        //ObjPose nearObj={};
        std::vector<float> score_heading_angles;
        std::vector<float> score_heading_velos;
        std::vector<float> score_obstacles;
        for (int i = 0; i < loci.loci.size(); i++)
        {
            //# (1) heading_angle
            score_heading_angles.emplace_back(CostFunction::score_heading_angle(loci.loci[i], getPlan));
            //# (2) heading_velo
            score_heading_velos.emplace_back(CostFunction::score_heading_velo(0.f));
            //# (3) obstacle
            //score_obstacles.emplace_back(CostFunction::score_obstcle(loci.loci[i], nearObj));
        }
        //printf("score:%lf\n", score_obstacles[0]);
        Math::VectorMinMaxNormalize(score_heading_angles);
        Math::VectorMinMaxNormalize(score_heading_velos);
        Math::VectorMinMaxNormalize(score_obstacles);
        int candidate = 0;
        float current_score = 0;
        float best_score = 0.0;
        float angle_score = 0.0;
        float velo_score = 0.0;
        float obs_score = 0.0;
        for (int i = 0; i < loci.loci.size(); i++)
        {
            //if (useHeading_angle)
                angle_score = cost::WEIGHT_ANGLE * score_heading_angles[i];
            //if (useHeading_velo)
                velo_score = cost::WEIGHT_VELOCITY * score_heading_velos[i];
            //if (useObstacle)
                obs_score = cost::WEIGHT_OBSTACLE * score_obstacles[i];
            current_score = angle_score + velo_score + obs_score;
            if (current_score > best_score)
            {
                candidate = i;
                best_score = current_score;
            }
        }
        angle_score = score_heading_angles[candidate];
        velo_score = score_heading_velos[candidate];
        obs_score = score_obstacles[candidate];
        Locus optPath = loci.loci[candidate];
        //printf("score:%lf omega%lf velo%lf obs%lf\n", best_score, angle_score, velo_score, obs_score);
    }
};