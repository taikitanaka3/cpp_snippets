#include <bits/stdc++.h>
#include <cv_practice/cv_practice.hpp>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

TEST(test, fillpoly) {
  std::vector<std::vector<cv::Point>> cv_polygons;

  // generate overwrapping polygon
  cv_polygons.emplace_back(std::vector<cv::Point>{
      cv::Point(0, 0), cv::Point(80, 80), cv::Point(0, 80)});
  cv_polygons.emplace_back(std::vector<cv::Point>{
      cv::Point(0, 0), cv::Point(20, 20), cv::Point(0, 20)});

  cv::Mat fill_poly_image(100, 100, CV_8UC1, cv::Scalar(0));
  cv::Mat fill_convex_poly_image(100, 100, CV_8UC1, cv::Scalar(0));

  cv::fillPoly(fill_poly_image, cv_polygons, cv::Scalar(255));
  for (auto p : cv_polygons) {
    cv::fillConvexPoly(fill_convex_poly_image, p, cv::Scalar(255));
  }
  cv::namedWindow("fill_poly_image", cv::WINDOW_NORMAL);
  cv::imshow("fill_poly_image", fill_poly_image);
  cv::namedWindow("fill_convex_poly_image", cv::WINDOW_NORMAL);
  cv::imshow("fill_convex_poly_image", fill_convex_poly_image);
  cv::waitKey(150000);
  cv::waitKey(150000);
}

TEST(test, erode) {
  cv::Mat image;
  cv::Mat cv_image(100, 100, CV_8UC1, cv::Scalar(0));
  cv::rectangle(cv_image, cv::Point(100, 30), cv::Point(53, 31),
                cv::Scalar(100), -1);
  cv::rectangle(cv_image, cv::Point(100, 37), cv::Point(53, 53),
                cv::Scalar(100), -1);
  cv::rectangle(cv_image, cv::Point(100, 53), cv::Point(53, 55),
                cv::Scalar(255), -1);
  cv::namedWindow("original", cv::WINDOW_NORMAL);
  cv::imshow("original", cv_image);
  cv::Mat erode_image(100, 100, CV_8UC1, cv::Scalar(0));
  cv::erode(cv_image, erode_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::erode(cv_image, cv_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::namedWindow("erode", cv::WINDOW_NORMAL);
  cv::imshow("erode", cv_image);
  cv::waitKey(5000);
}

TEST(test, erode_add) {
  cv::Mat image;
  cv::Mat cv_image(100, 100, CV_8UC1, cv::Scalar(0));
  cv::Mat cv_image2(100, 100, CV_8UC1, cv::Scalar(0));
  cv::rectangle(cv_image, cv::Point(100, 0), cv::Point(53, 31), cv::Scalar(100),
                -1);
  cv::rectangle(cv_image2, cv::Point(100, 30), cv::Point(53, 31),
                cv::Scalar(255), -1);

  cv::namedWindow("original", cv::WINDOW_NORMAL);
  cv::imshow("original", cv_image);
  cv_image += cv_image2;
  cv::namedWindow("add", cv::WINDOW_NORMAL);
  cv::imshow("add", cv_image);
  cv::waitKey(5000);
}

TEST(test, nominal) {
  cv::Mat image;
  cv::Mat cv_image(100, 100, CV_8UC1, cv::Scalar(255));
  cv::namedWindow("original", cv::WINDOW_NORMAL);
  cv::imshow("original", cv_image);
  cv::waitKey(1);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
