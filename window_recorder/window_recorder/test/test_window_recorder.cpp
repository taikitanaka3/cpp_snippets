#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <window_recorder/window_recorder.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

TEST(test, record) {
  time_t start, end;
  //////////////////// Added Part
  VideoCapture vcap(0);
  if (!vcap.isOpened()) {
      cout << "Error opening video stream or file" << endl;
  }
  int frame_width = vcap.get(CAP_PROP_FRAME_WIDTH);
  int frame_height = vcap.get(CAP_PROP_FRAME_HEIGHT);
  VideoWriter video("/tmp/test.mp4", VideoWriter::fourcc('X','V','I','D'), 10, Size(frame_width, frame_height), true);
  time(&start);
  for (;;){
    Mat frame;
    vcap >> frame;
    video.write(frame);
    imshow("Frame", frame);
    char c = (char)waitKey(33);
    if (c == 27) break;
    time(&end);
    double dif = difftime(end, start);
    printf("Elasped time is %.2lf seconds.", dif);
    if (dif==10)
    {
        std::cout << "DONE" << dif<< std::endl;
        break;
    }
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
