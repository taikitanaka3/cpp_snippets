// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest_practice/math/range.hpp"
#include "gtest_practice/risk_predictive_braking.hpp"

#include <gtest/gtest.h>

#include <vector>

// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest_practice/risk_predictive_braking.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(vector,vector_insert) {
  std::vector<int> a={0,2,4};
  std::vector<int> b={1,3,5};
  for(int i = 0;i < b.size();i++){
    // i=0 0->1 -> {0 1 2 4}
    // i=1 
    a.insert(a.begin()+i*2+1,b.at(i));
  }
  for(auto i:a){
      //std::cout<<i<<std::endl;
  }
}
