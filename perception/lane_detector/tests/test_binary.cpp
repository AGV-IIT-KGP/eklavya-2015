#include <gtest/gtest.h>
#include "../src/detector/getLaneBinary.cpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include <opencv2/opencv.hpp>

// This test function checks whether the image obtained from the applyThreshold() function is binary or not

TEST (LaneDetectorTest, Binary)
{  
  
  const int _debug = 1;
  cv::Mat image = cv::imread(ros::package::getPath("lane_detector") + "/tests/Test1.png",CV_LOAD_IMAGE_COLOR);
  cv::Mat binaryImage(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
  binaryImage = applyThreshold(image, _debug);
  for (int i = 0; i < binaryImage.rows; ++i)
  {
  	for (int j = 0; j < binaryImage.cols; ++j)
  	{
  		ASSERT_TRUE(((int)binaryImage.at<uchar>(i,j)==0)||((int)binaryImage.at<uchar>(i,j)==255));
  	}
  }
}

int main(int argc, char **argv)
{

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}