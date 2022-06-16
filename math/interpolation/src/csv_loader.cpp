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

#include "csv_loader.h"

CSVLoader::CSVLoader(std::string csv_path) { csv_path_ = csv_path; }

CSVLoader::~CSVLoader() {}

bool CSVLoader::readCSV(std::vector<std::vector<std::string>> &result,
                        const char delim) {
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

bool readEPSMapFromCSV(const std::string &csv_path, std::string &vehicle_name,
                       std::vector<double> &vel_index,
                       std::vector<double> &voltage_index,
                       std::vector<std::vector<double>> &eps_map) {
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  if (table[0].size() < 2) {
    return false;
  }

  vehicle_name = table[0][0];
  for (unsigned int i = 1; i < table[0].size(); ++i) {
    vel_index.push_back(std::stod(table[0][i]));
  }

  for (unsigned int i = 1; i < table.size(); ++i) {
    if (table[0].size() != table[i].size()) {
      return false;
    }
    voltage_index.push_back(std::stod(table[i][0]));
    std::vector<double> steer_angle_vels;
    for (unsigned int j = 1; j < table[i].size(); ++j) {
      steer_angle_vels.push_back(std::stod(table[i][j]));
      std::cout << std::stod(table[i][j]);
    }
    std::cout << std::endl;
    eps_map.push_back(steer_angle_vels);
  }

  return true;
}

int main() {
  std::string map_path = "/home/ros1-melodic/workspace/cpp_snippets/math/"
                         "interpolation/config/map.csv";
  double voltage = 0;
  std::vector<double> steer_angle_vels_interp;
  std::vector<double> vel_index_;
  std::vector<double> voltage_index_;
  std::vector<std::vector<double>> eps_map_;
  std::string name = "aaa";
  readEPSMapFromCSV(map_path, name, vel_index_, voltage_index_, eps_map_);
}
