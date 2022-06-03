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

#include "csv_loader.h"

CSVLoader::CSVLoader(std::string csv_path) { csv_path_ = csv_path; }

CSVLoader::~CSVLoader() {}

bool CSVLoader::readCSV(std::vector<std::vector<std::string>> & result, const char delim)
{
    std::ifstream ifs(csv_path_);
    if (!ifs.is_open()) {
        return false;
    }

    std::string buf;
    while (std::getline(ifs, buf)) {
        std::vector<std::string> tokens;

        std::istringstream iss(buf);
        std::string token;
        while (std::getline(iss, token, delim)) {
            tokens.push_back(token);
        }

        if (tokens.size() != 0) {
            result.push_back(tokens);
        }
    }

    return true;
}

bool readEPSMapFromCSV(const std::string & csv_path,
                       std::string & vehicle_name, std::vector<double> & vel_index,
                       std::vector<double> & voltage_index, std::vector<std::vector<double>> & eps_map)
{
    CSVLoader csv(csv_path);
    std::vector<std::vector<std::string>> table;

    if (!csv.readCSV(table))
    {
        return false;
    }

    if (table[0].size() < 2)
    {
        return false;
    }

    vehicle_name = table[0][0];
    for (unsigned int i = 1; i < table[0].size(); ++i)
    {
        vel_index.push_back(std::stod(table[0][i]));
    }

    for (unsigned int i = 1; i < table.size(); ++i)
    {
        if (table[0].size() != table[i].size())
        {
            return false;
        }
        voltage_index.push_back(std::stod(table[i][0]));
        std::vector<double> steer_angle_vels;
        for (unsigned int j = 1; j < table[i].size(); ++j)
        {
            steer_angle_vels.push_back(std::stod(table[i][j]));
            std::cout<<std::stod(table[i][j]);
        }
        std::cout<<std::endl;
        eps_map.push_back(steer_angle_vels);
    }

    return true;
}
bool LinearInterpolate::interpolate(
        const std::vector<double> &base_index, const std::vector<double> &base_value,
        const double &return_index, double &return_value) {
    auto isIncrease = [](const std::vector<double> &x) {
        for (int i = 0; i < (int) x.size() - 1; ++i) {
            double x1=x[i+1];
            double x0=x[i];
            if (x[i+1] < x[i]) {
                std::cout<<"decrease: "<<i<<std::endl;
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

#include <bits/stdc++.h>
int main() {
    std::cout << std::fixed << std::setprecision(5);
    std::string map_path="/home/ros1-melodic/workspace/cpp_snippets/math/interpolation/config/map2.csv";
    std::vector<double> steer_angle_vels_interp;
    std::vector<double> vel_index_;
    std::vector<double> voltage_index_;
    std::vector<std::vector<double>> eps_map_;
    std::string name="aaa";
    readEPSMapFromCSV(map_path,name,vel_index_,voltage_index_,eps_map_);
    //const double target = -1.4;
    //const double current = -1.5;
    double voltage=0;
    double target = -0.5;
    double current = -0.9;
    for(int i=-4;i<4;i++) {
        for (int j = -4; j < 4; j++) {
            double target = i*0.1;
            double current = j*0.1;
            LinearInterpolate linear_interp;
            steer_angle_vels_interp.clear();
            if (current < vel_index_.front())
            {
                current = vel_index_.front();
            }
            else if (vel_index_.back() < current)
            {
                current = vel_index_.back();
            }
            for (std::vector<double> steer_angle_vels : eps_map_) {
                double steer_angle_vel_interp;
                linear_interp.interpolate(vel_index_, steer_angle_vels, current, steer_angle_vel_interp);
                steer_angle_vels_interp.push_back(steer_angle_vel_interp);
            }
            if (target < steer_angle_vels_interp.front())
            {
                target = steer_angle_vels_interp.front();
            }
            else if (steer_angle_vels_interp.back() < target)
            {
                target = steer_angle_vels_interp.back();
            }
            linear_interp.interpolate(steer_angle_vels_interp, voltage_index_, target, voltage);
            std::cout<<"(target_vel,current_vel)\t"<<"("<<target<<","<<current<<")"<<"\t\t"<<"voltage: "<<voltage<<std::endl;
        }
    }
        return 0;
}
