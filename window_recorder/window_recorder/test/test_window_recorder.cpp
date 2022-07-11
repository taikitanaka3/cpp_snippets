#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <window_recorder/window_recorder.hpp>

using namespace std;
using namespace cv;

TEST(test, imread) {
  cv::VideoCapture capture("/home/$USER/cpp_snippets/window_recorder/"
                           "window_recorder/test/opencv-logo001.png",
                           cv::CAP_IMAGES);
  if (!capture.isOpened()) {
  }
  cv::Mat image;
  capture >> image;
  while (true) {

    // Capture Image from File ( e.g. input_030.jpg, input_031.jpg, ... )
    capture >> image;
    if (image.empty()) {
      std::cout << "em" << std::endl;
      break;
    }
    /* Image Processing */
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::imshow("image", image);
    cv::waitKey(30000);
    if (cv::waitKey(3000) >= 0) {
      break;
    }
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
