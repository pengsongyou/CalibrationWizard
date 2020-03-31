//
//  Calib.h
//
//  Copyright © 2019 Songyou Peng. All rights reserved.
//

#ifndef Calib_h
#define Calib_h
#include "setting.h"

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
class Calib
{
private:
    Settings s;
    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int numIntr;
    int mode;
    clock_t prevTimestamp = 0;
    Scalar RED, GREEN;
    const char ESC_KEY = 27;
    
    // Print camera parameters to the output file
    void saveCameraParams( Settings&, Size&, Mat&, Mat&,
                          const vector<Mat>&, const vector<Mat>&,
                          const vector<float>&, const vector<vector<Point2f> >&,
                          double);

    bool runCalibrationAndSave(Settings&, Size, Mat& , Mat& ,vector<vector<Point2f> >);
    static bool runCalibration( Settings&, Size& , Mat& , Mat&,
                               vector<vector<Point2f> > , vector<Mat>&, vector<Mat>&,
                               vector<float>&, double&);
    static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>&, Settings::Pattern patternType /*= Settings::CHESSBOARD*/);
    static double computeReprojectionErrors( const vector<vector<Point3f> >&,
                                            const vector<vector<Point2f> >&,
                                            const vector<Mat>&, const vector<Mat>&,
                                            const Mat&, const Mat&,
                                            vector<float>&);
    
public:
    std::string base_path;
    Calib();
//    ~Calib();
    bool readSettings(Settings);
    void cameraCalib();
};

#endif /* Calib_h */
