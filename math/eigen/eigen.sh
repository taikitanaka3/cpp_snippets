#!/bin/bash

cd $(dirname $0)
dir=$(pwd)

echo $dir

contest=$1
number=$2

echo "Create contest directory..."
mkdir $contest
cd $contest

echo "Create contest number directory..."
mkdir $number
cd $number

echo "Create files."

# cpp file template
template=$(
    cat <<EOS
#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;
using namespace std;


void Main()
{
	Vector3i a(2, -1,0);
	Vector3i b(3, 1,4);
	Vector3i sub = 3*a-2*b;
	print("a - b = \n" << sub);

	double inner_prod = a.dot(b);
	print("a \u2022 b = " << inner_prod);

	Vector3i cross_prod = a.cross(b);
	print("a X b = \n" << cross_prod);

	float norm = a.norm();
	print("norm a = " << norm);


	MatrixXi A(3, 2);
	A <<
			3, 1,
			8, 3,
			1, 0;
	MatrixXi B(3, 2);
	B <<
			1, 5,
			-2, 3,
			4, -1;
	MatrixXi C=2*A-3*B;
	print("a X b = \n"<<C);

	return;
}

int main()
{
	std::cin.tie(0);
	std::ios_base::sync_with_stdio(false);
	std::cout << std::fixed << std::setprecision(15);
	Main();
}


EOS
)

filelist=("a" "b" "c" "d" "e" "f")

for file in ${filelist[@]}; do
    touch ${file}.cpp
    echo "$template" >${file}.cpp
    echo "created file: ${file}.cpp"
done

echo "Add executable into CMakeLists.txt"

cd ..

# Write settings into CMakeLists.txt
echo "" >>./CMakeLists.txt
echo "cmake_minimum_required(VERSION 3.10)" >>./CMakeLists.txt
#echo "project($contest)">> ./CMakeLists.txt
echo "set(CMAKE_CXX_STANDARD 17)" >>./CMakeLists.txt

echo "# Contest $contest $number" >>./CMakeLists.txt
for file in ${filelist[@]}; do
    echo "add_executable(${number}_${file} $number/${file}.cpp)" >>./CMakeLists.txt
done
