//
//  Proc.h
//  
//  Copyright © 2019 Songyou Peng. All rights reserved.
//




#ifndef header_h
#define header_h


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


class Proc
{
private:
    cv::VideoCapture cap;// Define camera object
    std::vector <std::vector<double> > pt_nextpose;// 2D vector for points of the next pose
    cv::Mat frame, frame_save;
    std::vector<std::string> imageList;// Extract the path of previous iamges
    std::string curr_image_path;
    cv::Mat intrinsics, distCoeffs;
    cv::Mat translate;
    cv::Size boardSize;
    float squareSize;
    // std::string nextpose_path;
    // std::string output_path;
    // std::string nextpose_extr_paths;
    std::vector<cv::Point3f> corner_points;
    
    bool found;
    int counting = 0;
    std::vector<cv::Point2f> pointBuf;
    
public:
    std::string base_path;
    std::string imagelist_path;
    Proc();// Constructor
    ~Proc();// Destructor
    void extract_calibPara();
    void resetImagePath();
    bool readImageList();
    void openCamera();
    void showFrame();// Show current video frame
    bool controlFrame(int);
    void showNextPose();// Show next pose
    void extract_points();// Extract 2D points of next pose
    int update_captureIndex(); // Update the current new captured image index
    void captureImage(std::string , int); // Capture an image
    void drawCircle(cv::Mat, cv::Point);// Draw circle for showNextPose
    void addImagePath(const int&); // Add the new captured image to the image list
    bool plotGuide(bool );
    void build_3Dpoints();
    void showChessboard();
    void extractInfo(std::string);
};



#endif /* header_h */
