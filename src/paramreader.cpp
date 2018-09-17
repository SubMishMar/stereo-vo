#include <iostream>
#include "svo.h"

void svo::readParams(int argc, char** argv) {
    folder_left = argv[1];
    cv::glob(folder_left, filenames_left);
    folder_right = argv[2];
    cv::glob(folder_right, filenames_right);
    config_file = argv[3];
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "Failed to Open" << std::endl;
    } else {
        fsSettings["height"] >> rows;
        fsSettings["width"] >> cols;
        fsSettings["P0"] >> P_l;
        fsSettings["P1"] >> P_r;

        fsSettings["maxCorners"] >> maxCorners;
        fsSettings["qualityLevel"] >> qualityLevel;
        fsSettings["minDistance"] >> minDistance;
        fsSettings["blockSize"] >> blockSize;
        fsSettings["useHarrisDetector"] >> useHarrisDetector;
        fsSettings["k"] >> k;
        fsSettings["winSizeGFTT"] >> winSizeGFTT;

        std::cout << "Parameters Loaded" << std::endl;
    }
}