#!/bin/bash

cd $(dirname $0)
dir=$(pwd)

echo $dir

package_name=$1

echo "Create package_name directory..."
mkdir $package_name
cd $package_name

echo "Create package_name directory..."
mkdir $package_name
cd $package_name

echo "Create files."

# hpp file template
template_hpp=$(
    cat <<EOS
#include <bits/stdc++.h>
EOS
)

# cpp file template
template_cpp=$(
    cat <<EOS
#include <bits/stdc++.h>
#include <$package_name/$package_name.hpp>

int main()
{
  return 0;
}

EOS
)

# test file template
template_test=$(
    cat <<EOS
#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <$package_name/$package_name.hpp>
TEST(test, nominal) {
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


EOS
)

# cmake file template
template_cmake=$(
    cat <<EOS
cmake_minimum_required(VERSION 3.5)
project($package_name)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_library($package_name SHARED
  src/$package_name/$package_name.cpp
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_gtest(test_$package_name \${test_files})
  target_link_libraries(test_$package_name
    $package_name
  )
endif()
ament_auto_package()


EOS
)

# xml template
template_xml=$(
    cat <<EOS
<?xml version="1.0"?>
<package format="2">
  <name>$package_name</name>
  <version>0.1.0</version>
  <maintainer email="tanaka3@gmail.com">tanaka3</maintainer>
  <description>The gtest package</description>
  <license>Apache License 2.0</license>
  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

EOS
)

filelist=($package_name)

for file in ${filelist[@]}; do
    mkdir -p src/${file}
    mkdir -p include/${file}
    mkdir test
    echo "$template_test" >test/test_${file}.cpp
    echo "$template_hpp" >include/${file}/${file}.hpp
    echo "$template_cpp" >src/${file}/${file}.cpp
    echo "$template_xml" >package.xml
    echo "$template_cmake" >CMakeLists.txt
    echo "created file: ${file}.cpp"
done
