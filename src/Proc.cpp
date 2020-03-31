//
//  Proc.cpp
//  
//  Copyright © 2019 Songyou Peng. All rights reserved.
//

#include "Proc.h"

Proc::Proc()
{}

void Proc::openCamera()
{
    // set the size of frame to 640*480
    if(cap.open(0))
    {
        cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    }
}

Proc::~Proc(){cap.release();}

bool Proc::controlFrame(int keyNum)
{
    cap >> frame;
    frame.copyTo(frame_save); // copy the current frame for the capturing purpose later
    
    if(keyNum == 27 || frame.empty()) return false; // end of video stream or press ESC
    else return true;
}
void Proc::drawCircle(cv::Mat img, cv::Point center)
{
    int thickness = -1;
    int lineType = 8;
    cv::circle(img,
               center,
               3.0,
               cv::Scalar(0, 0, 255),
               thickness,
               lineType
               );
}

void Proc::showNextPose()
{
    int width = pt_nextpose[0].size();
    cv::Point2f prev, curr;

    curr.x = pt_nextpose[0][0];
    curr.y = pt_nextpose[1][0];
    drawCircle(frame,curr);
    for (int i = 1; i < width; ++i)
    {
        prev = curr;
        curr.x = pt_nextpose[0][i];
        curr.y = pt_nextpose[1][i];
        drawCircle(frame,curr);
        cv::line(frame, prev, curr, cv::Scalar(255, 255, 255), 2, 8);
    }
    
}

void Proc::build_3Dpoints()
{
    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            corner_points.push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));
    
}

void Proc::extractInfo(std::string input_path)
{
    cv::FileStorage fs(input_path, cv::FileStorage::READ);
    cv::FileNode n = fs.getFirstTopLevelNode();

    n["BoardSize_Width"] >> boardSize.width;
    n["BoardSize_Height"] >> boardSize.height;
    n["Square_Size"] >> squareSize;
    fs.release();
}

void Proc::extract_calibPara()
{
    // Read intrinsic matrix
    cv::FileStorage fs(base_path + "out_camera_data.xml", cv::FileStorage::READ);
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs.release();
}

void Proc::extract_points()
{
    std::ifstream infile;
    infile.open(base_path + "nextpose_points.txt");
    
    while (infile)
    {
        // read the entile line into a string
        std::string s;
        if (!getline(infile, s)) break;
        
        // Now we use a stringstream to seperate the fileds out of the line
        std::istringstream ss(s);
        std::vector <double> record;
        while (ss)
        {
            std::string s;
            if (!getline(ss, s, ',')) break;
            std::stringstream fs(s);
            double f = 0.0;
            fs >> f;
            record.push_back(f);
        }
        pt_nextpose.push_back(record);
    }
    if (!infile.eof())
    {
        std::cerr << "Cannot find the file containing next pose!\n";
    }
    infile.close();
}

void Proc::resetImagePath()
{
    cv::FileStorage fs(imagelist_path, cv::FileStorage::READ);
    if (fs.isOpened()) // Image list xml file exists, remove it
    {
        char path[2000];
        std::strcpy(path, imagelist_path.c_str());
        std::remove(path);
    }
    cv::FileStorage fst(imagelist_path, cv::FileStorage::WRITE);
    fst.release();
    fs.release();
}

bool Proc::readImageList()
{
    imageList.clear();
    cv::FileStorage fs(imagelist_path, cv::FileStorage::READ);
    if( !fs.isOpened())
    {
        fs.release();
        return false;
    }
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
    {
        fs.release();
        return false;
    }
    
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        imageList.push_back((std::string)*it);
    fs.release();
    return true;
}

int Proc::update_captureIndex()
{
    std::string index_path = base_path + "out_idx_path.txt";
    std::ifstream input(index_path);
    // File already exists
    if (input.is_open())
    {
        std::string number;
        int idx;
        getline(input,number); //read number
        idx = atoi(number.c_str()); //convert to integer
        ++idx;
        input.close();
        std::ofstream output(index_path);
        output << idx;
        output.close();
        return idx;
    }
    else
    {
        int idx = 0;
        std::ofstream my_file(index_path);
        my_file << idx;
        my_file.close();
        return idx;
    }
}

void Proc::captureImage(std::string path, int idx)
{
    char imagename[200];
    sprintf(imagename,"%.3d.jpg", idx);
    curr_image_path = path + (std::string)imagename;
    cv::imwrite( curr_image_path, frame_save);
    std::cout << "Capture image " << idx << " successfully." << std::endl;

}

void Proc::showFrame()
{
    cv::imshow("Calibration Wizard", frame);
}

void Proc::addImagePath(const int& mode)
{
    if (mode == 1)
    {
        if(!readImageList())
            std::cerr << "The image list is invalid";
    }
    
    cv::FileStorage fs(imagelist_path, cv::FileStorage::WRITE);

    imageList.push_back(curr_image_path);
    fs << "images" << "[";
    std::vector<std::string>::iterator it = imageList.begin();
    for (;it != imageList.end(); ++it)
    {
        fs << *(it);
    }
    fs << "]";
    fs.release();
    
}

// Find the chessboard and plot the guide to the goal (guide_flag = 1, plot; 0, no)
bool Proc::plotGuide(bool guide_flag)
{
    if (counting++ % 5 == 0)
    {
        found = findChessboardCorners( frame_save, boardSize, pointBuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK);
    }

        // Draw the corners.
        drawChessboardCorners( frame,boardSize, cv::Mat(pointBuf), found);

        return found;
}




void Proc::showChessboard()
{
    bool found;
    std::vector<cv::Point2f> pointBuf;
    
    found = findChessboardCorners( frame, boardSize, pointBuf,
                                  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    // Draw the corners.
    drawChessboardCorners( frame,boardSize, cv::Mat(pointBuf), found);
}

