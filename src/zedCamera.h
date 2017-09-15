// Author: Akash Bapat <akash@cs.unc.edu>
//BSD 3-Clause License

//Copyright (c) 2017, The University of North Carolina at Chapel Hill
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:

//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.

//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.

//* Neither the name of the copyright holder nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//file created for simple api for zed camera without using the zed sdk
#ifndef ZEDCAMMERA_H
#define ZEDCAMMERA_H
#include"datatypes.h"
#include<opencv2/opencv.hpp>
#include "confParser.h"
#include "stereo/opencvstereo.h"

/*! \namespace nfs
 *  \brief This library was developed for autonomous car, hence name nfs
 *
 */
namespace nfs{


/*! \namespace vision
 *  \brief This library was developed for autonomous car,  this namespace takes care of the vision part of autonomous car
 *
 */
namespace vision{

/*!
 * \brief The zedCamera class this class is a wrapper for using zed camera without zed sdk. This class uses opencv for camera access.
 */
class zedCamera{
public:


    /*!
     * \brief The RESOLUTION enum describes the available resolutions
     *        WVGA : 1344x 376  =  2*WxH
     *        HD   : 2560x 720  =  2*WxH
     *        FHD  : 3840x1080  =  2*WxH
     *        RES2K: 4416x1242  =  2*WxH
     */
    enum class RESOLUTION{
        WVGA,HD,FHD,RES2K
    };


    /*!
     * \brief The SIDE enum describes which camera side you want to access, left or right
     */
    enum class SIDE{
        LEFT, RIGHT
    };


    /*!
     * \brief The IMAGE enum specifies which image is requested
     *          RAW         : raw image data which sufferes from radial distortin and the left-right images are not inplane
     *          UNDISTORTED : undistorted image has the radial distortion removed, needs the calibration data to do so, and the left-right images are not inplane
     *          RECTIFIED   : rectified image has the radial distortion removed, needs the calibration data to do so, and the left-right images ARE inplane
     *
     */
    enum class IMAGE{
        RAW,RECTIFIED, UNDISTORTED
    };


    /*!
     * \brief The IMAGE_MEASURE enum the requested measurement type, requires calibration information for computation and gpu support
     *          DEPTH     : depth values in meters
     *          DISPARITY : pixel disparities, conceptually inverse of depth,  depth = f*baseline/disparity
     */
    enum class IMAGE_MEASURE{
        DEPTH,DISPARITY
    };


    /*!
     * \brief The ERRORCODE enum reflects the process status
     *          SUCCESS         : success! everything is fine
     *          CAM_OPEN_FAILED : failed to oepn camera, check connections or multiple cameras
     *          INVALID_FPS     : input fps is unsupported, must be 15,30,60 or 100
     *          NO_GPU          : no nVidia gpu found
     *          DEFAULTING      : using default values for supported fps for input resolution
     *          NO_CALIBRATION  : no calibration data provided, but function call requires it or failed to read calibration data
     *          MULTIPLE_ERRORS : multiple errors occured
     */
    enum class ERRORCODE{
        SUCCESS, CAM_OPEN_FAILED,INVALID_FPS, NO_GPU, DEFAULTING, NO_CALIBRATION, MULTIPLE_ERRORS

    };



    /*!
     * \brief operator +  addition operator for ERRORCODE enum
     * \param l left operand
     * \param r right operand
     * \return addition of ERRORCODEs
     */
    friend  inline ERRORCODE operator+(ERRORCODE l, ERRORCODE r)  {

        if(l==ERRORCODE::SUCCESS && r == ERRORCODE::SUCCESS)
            return ERRORCODE::SUCCESS;

        else if(l==ERRORCODE::SUCCESS && r != ERRORCODE::SUCCESS)
            return r;

        else if(l!=ERRORCODE::SUCCESS && r == ERRORCODE::SUCCESS)
            return l;

        else return ERRORCODE::MULTIPLE_ERRORS;
    }





    //---------------------------------------------------------------------------------------------------------------------------
    // class  starts


    /*!
     * \brief zedCamera simple constructor, can access only raw feed using this constructor
     * \param camName name of the camera
     * \param res   resolution demanded by user, not guaranteed to be available
     * \param fps   frame rate demanded by user, not guaranteed to be available
     */
    zedCamera(const std::string camName, const RESOLUTION res, const int fps);


    /*!
     * \brief zedCamera   complex constructor, can access all the feeds inlcuding depth, disparity, raw, rectified and undistorted
     * \param camName name of the camera
     * \param path2CalibrationFile path to calibration data file, must use full path
     * \param res  resolution demanded by user, not guaranteed to be available
     * \param fps  frame rate demanded by user, not guaranteed to be available
     * \param enableDisparityFlag   if true, disparity image is also computed
     * \param useGPU flag to use GPU for stereo estimation
     */
    zedCamera(const std::string camName, const std::string path2CalibrationFile, const RESOLUTION res, const int fps, const bool enableDisparityFlag, const bool useGPU =false);

    /*!
     * \brief zedCamera   complex constructor, can access all the feeds inlcuding depth, disparity, raw, rectified and undistorted
     * \param cam_id Index of the camera
     * \param path2CalibrationFile path to calibration data file, must use full path
     * \param res  resolution demanded by user, not guaranteed to be available
     * \param fps  frame rate demanded by user, not guaranteed to be available
     * \param enableDisparityFlag   if true, disparity image is also computed
     * \param useGPU flag to use GPU for stereo estimation
     */
    zedCamera(const int camera_id, const std::string path2CalibrationFile, const RESOLUTION res, const int fps, const bool enableDisparityFlag, const bool useGPU =false);


    /*!
     * \brief zedCamera simple constructor, can access only raw feed using this constructor
     * \param res   resolution demanded by user, not guaranteed to be available
     * \param fps   frame rate demanded by user, not guaranteed to be available
     */
    zedCamera(const RESOLUTION res, const int fps);


    /*!
     * \brief zedCamera   complex constructor, can access all the feeds inlcuding depth, disparity, raw, rectified and undistorted
     * \param path2CalibrationFile path to calibration data file, must use full path
     * \param res    resolution demanded by user, not guaranteed to be available
     * \param fps   frame rate demanded by user, not guaranteed to be available
     * \param enableDisparityFlag   if true, disparity image is also computed
     * \param useGPU flag to use GPU for stereo estimation
     */
    zedCamera(const std::string path2CalibrationFile, const RESOLUTION res, const int fps, const bool enableDisparityFlag, const bool useGPU = false);

    /*!
     * \brief ~zedCamera destructor for zed camera, only does something when gpu is in use
     *
     */
    ~zedCamera();


    /*!
     * \brief init actually starts the camera, must init befire doing anything, evfen accessing width and height..
     * \return success or any failure flags
     */
    ERRORCODE init();


    /*!
     * \brief step grabs an image from the camera and sets up the internal member images for processing
     * \return result of reading image from the camera
     */
    bool step();





    /*!
     * \brief getImage returns imageType IMAGE of SIDE side
     * \param side  requested camera side, SIDE::LEFT or SIDE::RIGHT
     * \param imageType requested image type, can be IMAGE::RAW, IMAGE::RECTIFIED, IMAGE::UNDISTORTED
     * \return   returns the appropriate image after grabbing and processing it (if necessary)
     */
    colorImage getImage(const SIDE side,const IMAGE imageType);




    /*!
     * \brief getImageNormalize returns measurement image, i.e.  IMAGE_MEASURE::DEPTH or IMAGE_MEASURE::DISPARITY and normalizes it
     * \param meas image measure type, can be IMAGE_MEASURE::DEPTH or IMAGE_MEASURE::DISPARITY
     * \param minDisparity minimum pixel disparity, increase if stereo camreas are far apart
     * \param numDisparity range of depth calculated and complexity(time) increases proportionally
     * \param scale_min  minimum measure value, if not defined automatically computed and hence is not constant for each image
     * \param scaleD_max  maximum measure value, if not defined automatically computed and hence is not constant for each image
     * \return  return image is either single channel float depth image or single channel float disparity image, depth values are in meters
     */
    measureImage getImageNormalize(const IMAGE_MEASURE meas, const int minDisparity, const int numDisparity, const float scale_min = 0, const float scale_max = 0);



    /*!
     * \brief getImage returns measurement image, i.e.  IMAGE_MEASURE::DEPTH or IMAGE_MEASURE::DISPARITY
     * \param meas image measure type, can be IMAGE_MEASURE::DEPTH or IMAGE_MEASURE::DISPARITY
     * \param minDisparity minimum pixel disparity, increase if stereo camreas are far apart
     * \param numDisparity range of depth calculated and complexity(time) increases proportionally
     * \return  return image is either single channel float depth image or single channel float disparity image,, depth values are in meters
     */
    measureImage getImage(const IMAGE_MEASURE meas, const int minDisparity, const int numDisparity);


    /*!
     * \brief getPointCloudImage returns a 3D image transforming the disparity into a x,y,z image, invalid depth is 10000
     * \return 3D pointcloud image
     */
    pointcloudImage getPointCloudImage();

    //void calibrate();



    /*!
     * \brief getWidth returns image width of one side
     * \return returns image width of one side
     */
    int getWidth()const;

    /*!
     * \brief getHeight returns image height, same for both sides
     * \return returns image height, same for both sides
     */
    int getHeight()const;


    /*!
     * \brief getFPS returns frame rate of capture
     * \return  returns frame rate of capture
     */
    int getFPS()const;


    /*!
     * \brief printError processes and prints an error message
     * \param er ERRORCODE to be processed
     * \return returns error msg
     */
    static std::string printError(ERRORCODE er);


    /*!
     * \brief resetDefault resets to WVGA resolution and 30 fps, do not use in normal code, use only when you are lazy
     */
    void resetDefault();

    /*!
     * \brief calibrate calibrates the camera for all the resolutions and does stereo calibration \todo
     * \param path2outCalibFile writes the calibratin data to this path
     */
    static void calibrate(std::string path2outCalibFile);



    /*!
     * \brief getIntrinsicMatrix returns the intrinsic matrix for the rectified images
     * \param side defines left or right camera
     * \return intrisic matrix cv::Mat(3, 3, CV_64F);
     */
    cv::Mat getIntrinsicMatrix(SIDE side) const;

    /*!
     * \brief getRectifiedIntrinsicMatrix
     * \param side defines left or right camera
     * \return rectified intrisic matrix cv::Mat(3, 3, CV_64F);
     */
    cv::Mat getRectifiedIntrinsicMatrix(SIDE side) const;


    /*!
     * \brief getRadialDistCoeffs returns the radial distrotion parameters for side
     * \param side defines left or right camera
     * \return cv::Mat::zeros(4, 1, CV_64F) radial distortion where last two parameters are zero
     */
    cv::Mat getRadialDistCoeffs(SIDE side) const;

    /*!
     * \brief getBaselineCM returns baseline in centimeters
     * \return baseline in centimeters
     */
    float getBaselineCM()const;


    /*!
     * \brief savePointcloud saves the previously computed depth map as color point cloud
     * \param fname name of the file including path if not relative, do not add extension
     * \param binFlag not used currently
     * \param format pointcloudtype, current supported are PLY, PCD, XYZ (non-colored), by default PLY is saved
     */
    void savePointcloud(std::string fname, bool binFlag, std::string format = "PLY");

private:
    const std::string m_path2Calibration;
    const RESOLUTION m_res;
    const bool m_disparityFlag;
    const bool m_useGPU;
    //video or image params
    int m_width;
    int m_height;
    int m_fps;
    cv::VideoCapture m_cam;

    //previously captured images
    colorImage m_rawImage;
    measureImage m_disparity;

    //calibration data
    cv::Mat m_KLeft;
    cv::Mat m_KRight;
    cv::Mat m_radDistLeft;
    cv::Mat m_radDistRight;
    cv::Mat m_R;
    cv::Mat m_T;
    float m_baseline;

    //after rectification
    cv::Mat m_rect_KLeft;
    cv::Mat m_rect_KRight;


    //saved maps for undistortion
    lookupMap m_xLeftUndist;
    lookupMap m_yLeftUndist;
    lookupMap m_xRightUndist;
    lookupMap m_yRightUndist;

    //saved maps for undistortion + rectification
    lookupMap m_xLeftRectify;
    lookupMap m_yLeftRectify;
    lookupMap m_xRightRectify;
    lookupMap m_yRightRectify;

    cv::Mat m_disp2depthTransform;

    stereoMatcherInterface* m_cpuStereo;

    ERRORCODE loadCalibrationData(std::string path2CalibrationFile);

    //gpu objects
#ifdef USE_GPU
    stereoMatcherInterface* m_gpuStereo;
#endif

    void construct();
    static std::string resolution2String(RESOLUTION res);

};

}}
#endif //ZEDCAMMERA_H
