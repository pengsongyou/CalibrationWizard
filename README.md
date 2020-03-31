# Calibration Wizard
[**Paper**](http://openaccess.thecvf.com/content_ICCV_2019/papers/Peng_Calibration_Wizard_A_Guidance_System_for_Camera_Calibration_Based_on_ICCV_2019_paper.pdf) | [**Video**](https://www.youtube.com/watch?v=my3jocjpD0U&feature=youtu.be&t=398) <br>
<img src="https://pengsongyou.github.io/media/wizard0.jpg" width="270"/> <img src="https://pengsongyou.github.io/media/wizard1.jpg" width="270"/> <img src="https://pengsongyou.github.io/media/wizard2.jpg" width="270"/>

This repository contains the implementation of the paper:

Calibration Wizard: A Guidance System for Camera Calibration Based on Modelling Geometric and Corner Uncertainty  
[Songyou Peng](http://pengsongyou.github.io/) and [Peter Sturm](https://team.inria.fr/steep/people/peter-sturm/)  
**ICCV 2019** (Oral)  

If you find our code or paper useful, please consider citing
```
@inproceedings{peng2019iccv,
 author =  {Songyou Peng and Peter Sturm},
 title = {Calibration Wizard: A Guidance System for Camera Calibration Based on Modelling Geometric and Corner Uncertainty},
 booktitle = {IEEE International Conference on Computer Vision (ICCV)},
 year = {2019},
}
```
## Installation
### Requirements
* CMake
* OpenCV
* MATLAB

### Build
```sh
git clone https://github.com/pengsongyou/CalibrationWizard
cd CalibrationWizard
mkdir build
cd build
cmake ..
make
```
Tested working on Mac OSX, OpenCV 2.4.11 and MATLAB R2015b.

## Usage

* **First step**: run binary `./bin/CalibrationWizard` and choose `mode=0` to capture images freely for initial calibration.
Press `space` to capture one image. After capturing, press `ESC` and the calibration is automatically done.  
* **Second step**: run `src_matlab/main_estimate.m` to estimate the next best pose.  
* **Third step**: run binary `./bin/CalibrationWizard` again and choose `mode=1`. The estimated next pose should be displayed. You can try to overlay the checkerboard with the new pose, and press `space` to capture the image.

Loop over the second and third step until you are satisfied, or tired :)

#### Command Line Options
* `mode=0` captures images for initial camera calibration
* `mode=1` shows the next best pose for capturing

In addition, we provide `mode=2` if you only want to perform calibration on all the captured images listed in `out/images/image_list.xml`.

#### TODO
* [ ] Release codes of the unified system in C++ which integrates the next best pose estimation.

#### Extreme Poses
Sometimes the next pose is extreme so the checkerboard cannot be detected. Please consider: 
* Check if the values in the uncertainty map are low. If yes, then the calibration has more or less converged.
* Consider autocorrelation matrix, simply set `autoCorr_flag=1` in `src_matlab/main_estimate.m`.
* Re-run `src_matlab/main_estimate.m`.
