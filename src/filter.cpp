#include <iostream>
#include "svo.h"

void svo::filterByStatus(std::vector<uchar> status,
		   				 std::vector<cv::Point2f> &corners1,
						 std::vector<cv::Point2f> &corners2) {
	size_t j = 0;
	for(size_t i = 0; i < status.size(); i++) {
		if((int)status[i] == 0 ||
		   corners2[i].x < 0 || corners2[i].y < 0 ||
		   corners2[i].x > cols || corners2[i].y > rows) {
		   	continue;
		} 
		corners1[j] = corners1[i];
		corners2[j] = corners2[i];
		j++;
	}
	corners1.resize(j);
	corners2.resize(j); 	
}

void svo::filterByStatus(std::vector<uchar> status,
		   				 std::vector<cv::Point2f> &corners1,
						 std::vector<cv::Point2f> &corners2,
						 std::vector<cv::Point2f> &corners1_) {
	size_t j = 0;
	for(size_t i = 0; i < status.size(); i++) {
		if((int)status[i] == 0 ||
		   corners2[i].x < 0 || corners2[i].y < 0 ||
		   corners2[i].x > cols || corners2[i].y > rows) {
		   	continue;
		} 
		corners1[j] = corners1[i];
		corners2[j] = corners2[i];
		corners1_[j] = corners1_[i];
		j++;
	}
	corners1.resize(j);
	corners2.resize(j); 	
	corners1_.resize(j);
}

void svo::filterByStatus(std::vector<uchar> status,
		   				 std::vector<cv::Point2f> &corners1,
						 std::vector<cv::Point2f> &corners2,
						 std::vector<cv::Point2f> &corners1_,
						 std::vector<cv::Point2f> &corners2_) {
	size_t j = 0;
	for(size_t i = 0; i < status.size(); i++) {
		if((int)status[i] == 0 ||
		   corners2[i].x < 0 || corners2[i].y < 0 ||
		   corners2[i].x > cols || corners2[i].y > rows) {
		   	continue;
		} 
		corners1[j] = corners1[i];
		corners2[j] = corners2[i];
		corners1_[j] = corners1_[i];
		corners2_[j] = corners2_[i];
		j++;
	}
	corners1.resize(j);
	corners2.resize(j); 	
	corners1_.resize(j);
	corners2_.resize(j);
}

void svo::filterForStereoMatching(std::vector<cv::Point2f> &corners_l,
								  std::vector<cv::Point2f> &corners_r,
								  std::vector<cv::Point2f> &corners_l_) {
	size_t j = 0;
	for(size_t i = 0; i < corners_l.size(); i++) {
		if(abs(corners_l[i].y - corners_r[i].y)>=0.01) {
		   	continue;
		} 
		corners_l[j] = corners_l[i];
		corners_r[j] = corners_r[i];
		corners_l_[j] = corners_l_[i];
		j++;
	}
	corners_l.resize(j);
	corners_r.resize(j); 
	corners_l_.resize(j);
}

void svo::filterForStereoMatching(std::vector<cv::Point2f> &corners_l,
								  std::vector<cv::Point2f> &corners_r,
								  std::vector<cv::Point2f> &corners_l_,
								  std::vector<cv::Point2f> &corners_r_) {
	size_t j = 0;
	for(size_t i = 0; i < corners_l.size(); i++) {
		if(abs(corners_l[i].y - corners_r[i].y)>=0.01) {
		   	continue;
		} 
		corners_l[j] = corners_l[i];
		corners_r[j] = corners_r[i];
		corners_l_[j] = corners_l_[i];
		corners_r_[j] = corners_r_[i];
		j++;
	}
	corners_l.resize(j);
	corners_r.resize(j); 
	corners_l_.resize(j);
	corners_r_.resize(j);
}