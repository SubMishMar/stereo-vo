#include <iostream>
#include <fstream>
#include <iomanip>
#include "svo.h"

void svo::generate3DLmks(std::vector<cv::Point2f> corners_l,
                         std::vector<cv::Point2f> corners_r,
                         std::vector<cv::Point3f> &lmks3d) {
	lmks3d.clear();
	float x, y, z;
	for (int i = 0; i < corners_l.size(); i++) {

		float x_l = corners_l[i].x;
		float y_l = corners_l[i].y;

		float x_r = corners_r[i].x;
		float disparity = x_l - x_r;

		z = baseline*focal_length/(disparity);
		x = x_l*z/focal_length;
		y = y_l*z/focal_length;

		lmks3d.push_back(cv::Point3f(x, y, z));
	}
}

void svo::readImages(int id, cv::Mat &left_image, cv::Mat &right_image) {
    undistort(imread(filenames_left[id], CV_8UC1),
            left_image, K_l, cv::noArray(), K_l);
    undistort(imread(filenames_right[id], CV_8UC1),
            right_image, K_r, cv::noArray(), K_r);
}

void svo::continousOperation () {
    uint database_id = 0;
    uint query_id = database_id + 1;

    // Step 1: Capture 2 stereo image pairs
    readImages(database_id, db_img_l, db_img_r);
    readImages(query_id, qry_img_l, qry_img_r);
    // Step 1: Ends

    // Step 2: Extract and Match features b/w two adjascent left images
    detectGoodFeatures(db_img_l, 
					   db_corners_l);
    status.clear();
    status = calculateOpticalFlow(db_img_l, 
    							  qry_img_l,
    							  db_corners_l,
    							  qry_corners_l);
    filterByStatus(status,
    			   db_corners_l,
    			   qry_corners_l); 
    status.clear();
    // Step 2: Ends

    // Step 3: Triangulate Matched Features
	status = calculateOpticalFlow(db_img_l, 
    					 		  db_img_r, 
					              db_corners_l,
			                      db_corners_r);
	filterByStatus(status,
		   		   db_corners_l,
				   db_corners_r,
				   qry_corners_l);
	status.clear();
	std::cout << db_corners_l.size() << "\t" << db_corners_r.size() << "\t" << qry_corners_l.size() << std::endl;
	filterForStereoMatching(db_corners_l, db_corners_r, qry_corners_l);
	std::cout << db_corners_l.size() << "\t" << db_corners_r.size() << "\t" << qry_corners_l.size() << std::endl;

	status = calculateOpticalFlow(qry_img_l,
								  qry_img_r,
								  qry_corners_l,
								  qry_corners_r);
	filterByStatus(status,
		   		   qry_corners_l,
				   qry_corners_r,
				   db_corners_l,
				   db_corners_r);
	status.clear();	
	std::cout << db_corners_l.size() << "\t" << db_corners_r.size() 
				<< "\t" << qry_corners_l.size() << "\t" << qry_corners_r.size() << std::endl;
	filterForStereoMatching(qry_corners_l, qry_corners_r, db_corners_l, db_corners_r);
	std::cout << db_corners_l.size() << "\t" << db_corners_r.size() 
				<< "\t" << qry_corners_l.size() << "\t" << qry_corners_r.size() << std::endl;

	generate3DLmks(db_corners_l, db_corners_r, db_lmks3d);
	generate3DLmks(qry_corners_l, qry_corners_r, qry_lmks3d);
    // drawMatches(db_img_l, db_img_r, db_corners_l, db_corners_r, "db_img_l", "db_img_r");
    // drawMatches(qry_img_l, qry_img_r, qry_corners_l, qry_corners_r, "qry_img_l", "qry_img_r");
	std::cout << db_lmks3d << std::endl << std::endl;
	std::cout << qry_lmks3d << std::endl;
    // Step 3: Ends

	// generate3DLmks(db_corners_l, db_corners_r, db_lmks3d);

	// for(int i = 0; i < lmks3d.size(); i++) {
	// 	std::cout << lmks3d[i].x << "\t" << lmks3d[i].y << "\t" << lmks3d[i].z << std::endl;
	// }
}