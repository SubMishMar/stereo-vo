#include <eigen3/Eigen/Eigen>


#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

class svo {
private:
    //for GFTT
    int maxCorners;
    double qualityLevel;
    double minDistance;
    int blockSize;
    bool useHarrisDetector;
    double k;
    int winSizeGFTT;

    //Calibration Matrix
    cv::Mat P_l, K_l;
    cv::Mat P_r, K_r;

    //Folder and filenames
    std::vector<cv::String> filenames_left;
    std::vector<cv::String> filenames_right;
    cv::String folder_left;
    cv::String folder_right;
    std::string config_file;

    //Image
    cv::Mat db_img_l, db_img_r, qry_img_l, qry_img_r;

    // other vars
    int rows, cols;

    // utility variables
    cv::Mat db_mask_mat_l, db_mask_mat_r;
    cv::Mat qry_mask_mat_l, qry_mask_mat_r;
    std::vector<cv::Point2f> db_corners_l;
    std::vector<cv::Point2f> db_corners_r;
    std::vector<cv::Point2f> qry_corners_l;
    std::vector<cv::Point2f> qry_corners_r;
    std::vector<cv::Point3f> db_lmks3d;
    std::vector<cv::Point3f> qry_lmks3d;
    double baseline, focal_length;
    std::vector<uchar> status;

public:
    //constructor
    svo(int argc, char **argv) {
        readParams(argc, argv);
        K_l = P_l(cv::Range(0, 3), cv::Range(0, 3));
        K_r = P_r(cv::Range(0, 3), cv::Range(0, 3));
        baseline = -P_r.at<double>(0, 3)/K_l.at<double>(0, 0);
        focal_length = K_l.at<double>(0, 0);
    }

    // func for reading params
    void readParams(int, char**);

    // detect GFTT
    void detectGoodFeatures(cv::Mat img,
                            std::vector<cv::Point2f> &corners);

    // calculating optical flow
    std::vector<uchar> calculateOpticalFlow(cv::Mat img1, cv::Mat img2, 
                                              std::vector<cv::Point2f> &corners1,
                                              std::vector<cv::Point2f> &corners2);
    //func for continous operation
    void continousOperation();

    // draw matches
    void drawMatches(cv::Mat img1, 
                      cv::Mat img2,
                      std::vector<cv::Point2f> corners1,
                      std::vector<cv::Point2f> corners2,
                      std::string window1,
                      std::string window2);

    // filter by status
    void filterByStatus(std::vector<uchar> status,
                        std::vector<cv::Point2f> &corners1,
                        std::vector<cv::Point2f> &corners2); 

    // filter for Stereo Matching
    void filterForStereoMatching(std::vector<cv::Point2f> &corners_l,
                                 std::vector<cv::Point2f> &corners_r,
                                 std::vector<cv::Point2f> &corners_l_);

    // generate 3d landmarks
    void generate3DLmks(std::vector<cv::Point2f> corners_l,
                        std::vector<cv::Point2f> corners_r,
                        std::vector<cv::Point3f> &lmks3d);

    // read database images
    void readImages(int id, cv::Mat &left_image, cv::Mat &right_image);

    // find stereo matches
    void getStereoMatches(cv::Mat left_img, 
                          cv::Mat right_img,
                          std::vector<cv::Point2f> &features_left,
                          std::vector<cv::Point2f> &features_right,
                          std::vector<cv::Point2f> &features_left_);
    
    // overloaded filter status fn
    void filterByStatus(std::vector<uchar> status,
                         std::vector<cv::Point2f> &corners1,
                         std::vector<cv::Point2f> &corners2,
                         std::vector<cv::Point2f> &corners1_);
    void filterByStatus(std::vector<uchar> status,
                         std::vector<cv::Point2f> &corners1,
                         std::vector<cv::Point2f> &corners2,
                         std::vector<cv::Point2f> &corners1_,
                         std::vector<cv::Point2f> &corners2_);
    
    //overloaded filter for stereo matching
    void filterForStereoMatching(std::vector<cv::Point2f> &corners_l,
                                  std::vector<cv::Point2f> &corners_r,
                                  std::vector<cv::Point2f> &corners_l_,
                                  std::vector<cv::Point2f> &corners_r_);

    //overloaded stereomatiching fn
    void getStereoMatches(cv::Mat left_img, 
                           cv::Mat right_img,
                           std::vector<cv::Point2f> &features_left,
                           std::vector<cv::Point2f> &features_right,
                           std::vector<cv::Point2f> &features_left_,
                           std::vector<cv::Point2f> &features_right_);
};