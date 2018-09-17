#include <iostream>
#include "svo.h"

void svo::drawMatches(cv::Mat img1, 
					  cv::Mat img2,
					  std::vector<cv::Point2f> corners1,
					  std::vector<cv::Point2f> corners2,
					  std::string window1,
					  std::string window2) {
	cv::Mat img1_out, img2_out;
	cv::cvtColor(img1, img1_out, CV_GRAY2BGR);
	cv::cvtColor(img2, img2_out, CV_GRAY2BGR);
	for (int i = 0; i < corners1.size(); i++) {
		cv::circle(img1_out, corners1[i], 4, CV_RGB(255, 0, 0), -1, 8, 0);
		cv::circle(img2_out, corners2[i], 4, CV_RGB(255, 0, 0), -1, 8, 0);
	}
	imshow(window1, img1_out);
	cv::waitKey(-1);
	imshow(window2, img2_out);
	cv::waitKey(-1);
}