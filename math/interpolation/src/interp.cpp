/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "interpolate.h"

/*
 * linear interpolation
 */

bool LinearInterpolate::interpolate(
        const std::vector<double> &base_index, const std::vector<double> &base_value,
        const double &return_index, double &return_value) {
    auto isIncrease = [](const std::vector<double> &x) {
        for (int i = 0; i < (int) x.size() - 1; ++i) {
            double x1=x[i+1];
            double x0=x[i];
            if (x[i+1] < x[i]) {
                std::cout<<"decrease"<<std::endl;
                return false;
            }
        }
        return true;
    };

    if (base_index.size() == 0 || base_value.size() == 0) {
        printf(
                "[interpolate] some vector size is zero: base_index.size() = %lu, base_value.size() = %lu",
                base_index.size(), base_value.size());
        return false;
    }

    // check if inputs are valid
    if (
            !isIncrease(base_index) || return_index < base_index.front() ||
            base_index.back() < return_index || base_index.size() != base_value.size()) {
        std::cerr << "[isIncrease] bad index, return false" << std::endl;
        bool b1 = !isIncrease(base_index);
        bool b3 = return_index < base_index.front();
        bool b4 = base_index.back() < return_index;
        bool b5 = base_index.size() != base_value.size();
        printf("%d, %d, %d, %d\n", b1, b3, b4, b5);
        printf("base_index = [%f, %f]\n", base_index.front(), base_index.back());
        printf(
                "base_index.size() = %lu, base_value.size() = %lu\n", base_index.size(), base_value.size());
        printf("base_index: [");
        for (int i = 0; i < base_index.size(); ++i) {
            printf("%f, ", base_index.at(i));
        }
        printf("]\n");
        printf("base_value: [");
        for (int i = 0; i < base_value.size(); ++i) {
            printf("%f, ", base_value.at(i));
        }
        printf("]\n");
        printf("return_index = %f\n", return_index);
        return false;
    }

    // calculate linear interpolation
    int i = 0;
    if (base_index[i] == return_index) {
        return_value = base_value[i];
        return true;
    }
    while (base_index[i] < return_index) {
        ++i;
    }
    if (i <= 0 || (int) base_index.size() - 1 < i) {
        std::cerr << "? something wrong. skip this return_index." << std::endl;
        return false;
    }

    const double dist_base_return_index = base_index[i] - base_index[i - 1];
    const double dist_to_forward = base_index[i] - return_index;
    const double dist_to_backward = return_index - base_index[i - 1];
    if (dist_to_forward < 0.0 || dist_to_backward < 0.0) {
        std::cerr << "?? something wrong. skip this return_index. base_index[i - 1] = "
                  << base_index[i - 1] << ", return_index = " << return_index
                  << ", base_index[i] = " << base_index[i] << std::endl;
        return false;
    }

    const double value = (dist_to_backward * base_value[i] + dist_to_forward * base_value[i - 1]) /
                         dist_base_return_index;
    return_value = value;

    return true;
}


int main() {
    //const double target = -1.4;
    //const double current = -1.5;
    const double target = 1.4;
    const double current = 1.5;
    double voltage=0;
    LinearInterpolate linear_interp;
    std::vector<double> steer_angle_vels_interp;
    std::vector<double> vel_index_ = {-2, -1, 0, 1, 2};
    std::vector<double> voltage_index_ = {-4, -2, 0, 2, 4};
    std::vector<std::vector<double>> eps_map_ =
            {{-1.5, -2.0, -2.5, -2.0, -1.5},
             {-1,   -1.5, -2.0, -1.5, -1},
             {0,    0,    0,    0,    0},
             {1,    1.5,  2,    1.5,  1},
             {1.5,  2,    2.5,  2,    1.5}
            };

    for (std::vector<double> steer_angle_vels : eps_map_) {
        double steer_angle_vel_interp;
        linear_interp.interpolate(vel_index_, steer_angle_vels, current, steer_angle_vel_interp);
        steer_angle_vels_interp.push_back(steer_angle_vel_interp);
        std::cout<<steer_angle_vel_interp<<std::endl;
        //return 1;
    }
    linear_interp.interpolate(steer_angle_vels_interp, voltage_index_, target, voltage);
    std::cout<<"voltage: "<<voltage<<std::endl;
    return 0;
}
