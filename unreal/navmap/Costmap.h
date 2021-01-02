#pragma once
#include "define/core_define.h"
#include "define/math_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include <vector>
using std::vector;
#include <math.h>
#include <stdio.h>

namespace costmap{
    const int LIMMIT_INDEX = 100;
    const int CRASH_INDEX = 99;
    const int WARNING_INDEX = 97;
    const int MINIMUM_INDEX = 1;
    const float CRASH_SIZE = 2.0f;
    const float CLEARANCE_SIZE = 8.0f;
}

class Costmap
{
private:
    Costmap();
    ~Costmap();

public:
    static void CalculateHeightMap(OccupancyGrid &costmap, int x, int y)
    {
        int xmin = Math::Saturation(x - int(costmap::CLEARANCE_SIZE / costmap.info.resolution), 0, costmap.info.width);
        int xmax = Math::Saturation(x + int(costmap::CLEARANCE_SIZE / costmap.info.resolution), 0, costmap.info.width);
        int ymin = Math::Saturation(y - int(costmap::CLEARANCE_SIZE / costmap.info.resolution), 0, costmap.info.height);
        int ymax = Math::Saturation(y + int(costmap::CLEARANCE_SIZE / costmap.info.resolution), 0, costmap.info.height);

        for (int j = ymin; j < ymax; j++)
        {
            for (int i = xmin; i < xmax; i++)
            {
                int index = TF::Grid2Array(costmap.info.width, i, j);
                //if(data<1 is ok too!)
                if (costmap.Data[index] == 0)
                {
                    int newData = costmap.Data[index];
                    float dist = hypotf(x - i, y - j);
                    if (dist < costmap::CRASH_SIZE)
                    {
                        newData = costmap::LIMMIT_INDEX - 1;
                    }
                    else if (dist < costmap::CLEARANCE_SIZE)
                    {
                        newData = 1 + int(float(costmap::WARNING_INDEX / dist));
                    }
                    else
                    {
                    }
                    costmap.Data[index] = newData;
                }
            }
        }
        //objectcの位置
        int objectIndex = TF::Grid2Array(costmap.info.width, x, y);
        costmap.Data[objectIndex] = 100;
    }
};
