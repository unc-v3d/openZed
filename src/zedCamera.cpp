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
#include "zedCamera.h"

//#define INVALID_DEPTH 10000
#define MAX_DISPARITY_GPU 128
#define MIN_DISPARITY_GPU 8
#define MAX_DEPTH MAX_DISPARITY_GPU
#define  WINDOW_SIZE_GPU 11

#include <stdlib.h>   //abs function
#include<exception>

namespace nfs{ namespace vision{

zedCamera::zedCamera(const std::string camName, const RESOLUTION res, const int fps):
    m_path2Calibration(""),m_res(res), m_disparityFlag(false),m_useGPU(false),m_fps(fps), m_cam(cv::VideoCapture(camName)){

    construct();

}

zedCamera::zedCamera(const std::string camName, const std::string path2CalibrationFile, const  RESOLUTION res, const int fps, const  bool enableDisparityFlag, const bool useGPU):
    m_path2Calibration(path2CalibrationFile),m_res(res), m_disparityFlag(enableDisparityFlag),m_useGPU(useGPU),m_fps(fps), m_cam(cv::VideoCapture(camName)){

    construct();
}



zedCamera::zedCamera(const RESOLUTION res, const int fps):
    m_path2Calibration(""),m_res(res), m_disparityFlag(false),m_useGPU(false),m_fps(fps), m_cam(cv::VideoCapture(0)){

    construct();

}

zedCamera::zedCamera(const std::string path2CalibrationFile, const  RESOLUTION res, const int fps, const  bool enableDisparityFlag, const bool useGPU):
    m_path2Calibration(path2CalibrationFile),m_res(res), m_disparityFlag(enableDisparityFlag),m_useGPU(useGPU), m_fps(fps), m_cam(cv::VideoCapture(0)){

    construct();
}



void zedCamera::construct(){

    m_width = 0;
    m_height = 0;

    m_KLeft = cv::Mat::eye(3, 3, CV_64F);
    m_KRight = cv::Mat::eye(3, 3, CV_64F);

    m_rect_KLeft = cv::Mat::eye(3, 3, CV_64F);
    m_rect_KRight = cv::Mat::eye(3, 3, CV_64F);

    m_radDistLeft = cv::Mat::zeros(4, 1, CV_64F);
    m_radDistRight = cv::Mat::zeros(4, 1, CV_64F);
    m_R = cv::Mat::eye(3, 3, CV_64F);
    m_T = cv::Mat::zeros(3, 1, CV_64F);
    m_baselineCM =1;// fake baseline

#ifdef USE_GPU
    m_stereoMatcher = nullptr;
#endif
}


zedCamera::~zedCamera(){
#ifdef USE_GPU
    if(m_stereoMatcher)
        delete m_stereoMatcher;
#endif
}



zedCamera::ERRORCODE zedCamera::init(){

    if(!m_cam.isOpened())
        return ERRORCODE::CAM_OPEN_FAILED;


    if(m_fps != 15 && m_fps != 30 && m_fps != 60 && m_fps != 100)
        return ERRORCODE::INVALID_FPS;


    ERRORCODE retVal = ERRORCODE::SUCCESS;

    //decode the height and width for different resoultion and set appropriate fps
    //assert the input  paramters and downgrade if not available,

    if(m_res == RESOLUTION::RES2K){
        m_cam.set(CV_CAP_PROP_FRAME_HEIGHT, 1242); //4416x1242
        m_cam.set(CV_CAP_PROP_FRAME_WIDTH, 4416); //left and right image

        //only available fps is 15
        m_cam.set(CV_CAP_PROP_FPS, 15);
    }
    else if(m_res == RESOLUTION::FHD){
        m_cam.set(CV_CAP_PROP_FRAME_HEIGHT, 1080); //3840x1080
        m_cam.set(CV_CAP_PROP_FRAME_WIDTH,3840); //left and right image

        //only available fps is 15 and 30
        if(m_fps > 30){
            m_fps = 30;
            m_cam.set(CV_CAP_PROP_FPS, 30);
            retVal = ERRORCODE::DEFAULTING;

        } else m_cam.set(CV_CAP_PROP_FPS, m_fps);

    }
    else if(m_res == RESOLUTION::HD){
        m_cam.set(CV_CAP_PROP_FRAME_HEIGHT, 720);//2560x720
        m_cam.set(CV_CAP_PROP_FRAME_WIDTH, 2560); //left and right image

        //only available fps is 15, 30,60
        if(m_fps > 60){
            m_fps = 60;
            m_cam.set(CV_CAP_PROP_FPS, 60);
            retVal = ERRORCODE::DEFAULTING;

        } else m_cam.set(CV_CAP_PROP_FPS, m_fps);


    }
    else {//wvga
        m_cam.set(CV_CAP_PROP_FRAME_HEIGHT, 376);//1344x376
        m_cam.set(CV_CAP_PROP_FRAME_WIDTH, 1344); //left and right image

        //  available fps is 15, 30, 60 and 100

        m_cam.set(CV_CAP_PROP_FPS, m_fps);

    }



    //recollect the fps and resolution from the camera
    m_height = m_cam.get(CV_CAP_PROP_FRAME_HEIGHT);
    m_width = m_cam.get(CV_CAP_PROP_FRAME_WIDTH)/2;
    m_fps = m_cam.get(CV_CAP_PROP_FPS);






    cv::Size outImSize =  cv::Size(m_width, m_height);
    //load calibration data
    if(!m_path2Calibration.empty())
        retVal = retVal + loadCalibrationData(m_path2Calibration);

    if(retVal == ERRORCODE::SUCCESS){ //initialize the undistort and rectification maps

        //initialise undistortion maps
        cv::initUndistortRectifyMap(m_KLeft, m_radDistLeft,cv::Mat(),  m_KLeft,outImSize,CV_32FC1, m_xLeftUndist,m_yLeftUndist);
        cv::initUndistortRectifyMap(m_KRight, m_radDistRight,cv::Mat(),  m_KRight, outImSize,CV_32FC1, m_xRightUndist,m_yRightUndist);


        //initialise undistortion + rectification maps
        cv::Mat R1,R2,P1,P2;
        cv::stereoRectify(m_KLeft,m_radDistLeft,m_KRight,m_radDistRight,outImSize,m_R,m_T,R1,R2,P1,P2,m_disp2depthTransform,cv::CALIB_ZERO_DISPARITY,0);

        P1(cv::Rect(0, 0, 3, 3)).copyTo(m_rect_KLeft);
        P2(cv::Rect(0, 0, 3, 3)).copyTo(m_rect_KRight);


        cv::initUndistortRectifyMap(m_KLeft, m_radDistLeft, R1,  P1, outImSize,CV_32FC1, m_xLeftRectify,m_yLeftRectify);
        cv::initUndistortRectifyMap(m_KRight, m_radDistRight,R2,  P2, outImSize,CV_32FC1, m_xRightRectify,m_yRightRectify);





#ifdef USE_GPU
        if(m_useGPU){
            int deviceCount;
            cudaError_t e = cudaGetDeviceCount(&deviceCount);
            if(e != cudaSuccess){
                retVal = retVal + ERRORCODE::NO_GPU;
                return retVal;
            }

            //
            // set up our matcher
            //


            cv::Mat tmp;
            cv::initUndistortRectifyMap(m_KLeft, m_radDistLeft, R1,  P1, outImSize,CV_32FC2, m_leftMapGPU,tmp);
            cv::initUndistortRectifyMap(m_KRight, m_radDistRight,R2,  P2, outImSize,CV_32FC2, m_rightMapGPU,tmp);

            float baseline = cv::norm(m_T);

            float focal_length = m_rect_KLeft.at<double>(0,0);

            const StereoMatcher::Options options = {
                (unsigned int)m_width,(unsigned int) m_height,
                WINDOW_SIZE_GPU,MIN_DISPARITY_GPU, MAX_DISPARITY_GPU,baseline ,focal_length , 1.0f, MAX_DEPTH};

            m_stereoMatcher = new StereoMatcher(options);

            m_stereoMatcher->initUndistortRectifyMaps(m_leftMapGPU.data, m_rightMapGPU.data);
        }
#endif

    }




    return retVal;
}



bool zedCamera::step(){

    return  m_cam.read(m_rawImage);

}



zedCamera::colorImage zedCamera::getImage(const SIDE side,const IMAGE imageType){

    colorImage retImage = colorImage(m_height, m_width, CV_8UC3);

    try{
        // throw exception for invalid input parameters
        if(m_path2Calibration.empty() && (imageType == IMAGE::RECTIFIED ||imageType == IMAGE::UNDISTORTED)  )
            throw ERRORCODE::NO_CALIBRATION;


        //get raw image previously grabbed
        colorImage rawColorImage = colorImage(m_height, m_width, CV_8UC3); // color image

        if(side == SIDE::LEFT){
            rawColorImage = m_rawImage(cv::Rect(0,0,m_width,m_height));

        } else{ //if( side == SIDE::RIGHT){
            rawColorImage = m_rawImage(cv::Rect(m_width,0,m_width,m_height));
        }





        if(imageType == IMAGE::RAW){ // copy data and return
            retImage =  rawColorImage.clone();
        }
        else if(imageType == IMAGE::UNDISTORTED){ // undistort according to calibration data and return the image

            if(side == SIDE::LEFT)
                cv::remap(rawColorImage ,   retImage, m_xLeftUndist , m_yLeftUndist, cv::INTER_AREA);
            else cv::remap(rawColorImage ,   retImage, m_xRightUndist , m_yRightUndist, cv::INTER_AREA);


        } else{ //imageType == IMAGE::RECTIFIED // undistort and rectify according to calibration data and return the image

            if(side == SIDE::LEFT)
                cv::remap(rawColorImage ,   retImage, m_xLeftRectify , m_yLeftRectify, cv::INTER_AREA);
            else cv::remap(rawColorImage ,   retImage, m_xRightRectify , m_yRightRectify, cv::INTER_AREA);
        }


    } catch(ERRORCODE& err){
        printError( err);
    }

    return retImage;

}


zedCamera::measureImage zedCamera::getImage(const IMAGE_MEASURE meas, const int minDisparity, const int numDisparity){



    colorImage left = m_rawImage(cv::Rect(0,0,m_width,m_height));


    colorImage right = m_rawImage(cv::Rect(m_width,0,m_width,m_height));


    cv::Mat imLeftGrey,imRightGrey;


    cv::cvtColor(left,imLeftGrey ,CV_BGR2GRAY);
    cv::cvtColor(right,imRightGrey,CV_BGR2GRAY);




#ifdef USE_GPU
    if(m_useGPU){
        //call the gpu code
        //  m_stereoMatcher->setMaxDisparity(minDisparity + numDisparity);
        m_disparity = measureImage(m_height, m_width, CV_32F);
        // stereo-rectified intensity images
        m_stereoMatcher->init_frame(imLeftGrey.data, imRightGrey.data);
        //find disparity
        m_stereoMatcher->match();


        /*  cv::Mat tmp = measureImage(m_height, m_width, CV_32FC1);
       m_stereoMatcher->download_depth(tmp.data);

{ double _min, _max;
      cv::minMaxIdx(tmp, &_min, &_max);
std::cout<<_min<<" " <<_max<<std::endl;

return tmp;

} */
        //even if the passed param is depth, it always returns disparity
        m_stereoMatcher->download_disparity(m_disparity.data);

    }else{
#else
    //rectify images
    cv::Mat imLeftGreyUndistortedRectified,imRightGreyUndistortedRectified;
    cv::remap(imLeftGrey,imLeftGreyUndistortedRectified,m_xLeftRectify,m_yLeftRectify,cv::INTER_AREA);
    cv::remap(imRightGrey,imRightGreyUndistortedRectified,m_xRightRectify,m_yRightRectify,cv::INTER_AREA);

    //    cv::Ptr<cv::StereoBM> stereoBlockMatcher = cv::StereoBM::create(numDisparity);

    //   stereoBlockMatcher->setMinDisparity(minDisparity);    // large (30cm) baseline and 36 pixel offset toward

    /* cv::Ptr<cv::StereoSGBM> stereoBlockMatcher = cv::StereoSGBM::create(1,    //int minDisparity
                                                          192,     //int numDisparities
                                                          5,      //int SADWindowSize
                                                          600,    //int P1 = 0
                                                          2400,   //int P2 = 0
                                                          10,     //int disp12MaxDiff = 0
                                                          4,     //int preFilterCap = 0
                                                          1,      //int uniquenessRatio = 0
                                                          150,    //int speckleWindowSize = 0
                                                          2,     //int speckleRange = 0
                                                          true);  //bool fullDP = false

*/


    int SADWindowSize =11;
    cv::Ptr<cv::StereoSGBM> stereoBlockMatcher = cv::StereoSGBM::create(minDisparity,    //int minDisparity
                                                                        numDisparity,     //int numDisparities
                                                                        SADWindowSize,      //int SADWindowSize
                                                                        8*SADWindowSize*SADWindowSize,    //int P1 = 0
                                                                        32*SADWindowSize*SADWindowSize,   //int P2 = 0
                                                                        1,     //int disp12MaxDiff = 0
                                                                        61,     //int preFilterCap = 0
                                                                        8,      //int uniquenessRatio = 0
                                                                        71,    //int speckleWindowSize = 0
                                                                        1,     //int speckleRange = 0
                                                                        false);  //bool fullDP = false

    cv::Mat tmpDisparity;
    stereoBlockMatcher->compute( imLeftGreyUndistortedRectified, imRightGreyUndistortedRectified, tmpDisparity);

    //! NOTE: apparently opencv stereo::SGBM has disparity values multiplied by 16, so finnally divide by it

    tmpDisparity.convertTo(m_disparity,CV_32FC1,1.0/16);


#endif


#ifdef USE_GPU
}
#endif

if(meas == IMAGE_MEASURE::DISPARITY){

    return m_disparity;

}else{ //calculate depth and return

pointcloudImage depth3DImage;
cv::reprojectImageTo3D(  m_disparity, depth3DImage, m_disp2depthTransform, true ); //sets for invalid disparity, depth = 10000
//extract the depth data

std::vector<measureImage> channels(3);

cv::split(depth3DImage, channels);

//last value is depth
return channels[2];


}


             }



zedCamera::measureImage zedCamera::getImageNormalize(const IMAGE_MEASURE meas,const  int minDisparity,const  int numDisparity,
                                                     const float scale_min, const float scale_max){



    measureImage mImage =  getImage(meas, minDisparity,  numDisparity);

    double local_min = scale_min;
    double local_max = scale_max;



    if(std::abs(scale_min) < FLOAT_EPSILON && std::abs(scale_max) <FLOAT_EPSILON){
        cv::minMaxIdx(mImage, &local_min, &local_max);

    }


    measureImage scaledImage ;
    mImage.convertTo(scaledImage, CV_32FC1, 1.0f/(local_max-local_min), - local_min/(local_max - local_min));
    return scaledImage;

}



int zedCamera::getWidth()const{
    return m_width;

}

int zedCamera::getHeight()const{
    return m_height;
}

int zedCamera::getFPS()const{
    return m_fps;
}




zedCamera::pointcloudImage zedCamera::getPointCloudImage()const{
    pointcloudImage depth3DImage;
    cv::reprojectImageTo3D(  m_disparity, depth3DImage, m_disp2depthTransform, true ); //sets for invalid disparity, depth = 10000
    return depth3DImage;
}

void zedCamera::calibrate(std::string path2outCalibFile){

    //  zedCamera zedCamera(RESOLUTION::WVGA, 30);
    //! \todo write code for calibrating the camera for all resolutions and save it in the same format as zed gives

}

std::string zedCamera::printError(ERRORCODE er) {

    std::string msg;

    switch(er){
    case ERRORCODE::SUCCESS:
        msg = "Successfully opened the camera" ;
        break;

    case ERRORCODE::CAM_OPEN_FAILED:
        msg = "Camera open failed";
        std::cout << msg << std::endl;
        std::exit(EXIT_FAILURE);
        break;

    case ERRORCODE::INVALID_FPS:
        msg = "Input fps is invalid, must be 15, 30, 60 or 100" ;
        break;

    case ERRORCODE::NO_GPU:
        msg = "No GPU found";
        break;

    case ERRORCODE::DEFAULTING:
        msg = "Using default compatible values for esolution and fps";
        break;

    case ERRORCODE::NO_CALIBRATION:
        msg =  "No calibration data found to process images" ;
        break;

    case ERRORCODE::MULTIPLE_ERRORS:
        msg =  "Multiple errors occured" ;
        break;

    default:
        msg =  "Unknown error";
        break;
    }


    std::cout << msg << std::endl;

    return msg;

}


void zedCamera::resetDefault(){

    //wvga
    m_cam.set(CV_CAP_PROP_FRAME_HEIGHT, 376);//1344x376
    m_cam.set(CV_CAP_PROP_FRAME_WIDTH, 1344); //left and right image

    //default fps is 30
    m_cam.set(CV_CAP_PROP_FPS, 30);

    //recollect the fps and resolution from the camera
    m_height = m_cam.get(CV_CAP_PROP_FRAME_HEIGHT);
    m_width = m_cam.get(CV_CAP_PROP_FRAME_WIDTH)/2;
    m_fps = m_cam.get(CV_CAP_PROP_FPS);


}


cv::Mat zedCamera::getIntrinsicMatrix(SIDE side)const{
    if(side == SIDE::LEFT)
        return m_KLeft;
    else
        return m_KRight;
}

cv::Mat zedCamera::getRectifiedIntrinsicMatrix(SIDE side) const{
    if(side == SIDE::LEFT)
        return m_rect_KLeft;
    else
        return m_rect_KRight;
}

cv::Mat zedCamera::getRadialDistCoeffs(SIDE side)const{
    if(side == SIDE::LEFT)
        return m_radDistLeft;
    else
        return m_radDistRight;
}


float zedCamera::getBaselineCM()const{
    return m_baselineCM;
}


zedCamera::ERRORCODE zedCamera::loadCalibrationData(std::string path2CalibrationFile){

    confParser calibFile(path2CalibrationFile);
    std::string resolution = resolution2String(m_res);

    std::vector<std::string> cameraNames;
    cameraNames.push_back("LEFT_CAM_");
    cameraNames.push_back("RIGHT_CAM_");


    float val;
    bool successFlag = true;
    // get intrinsic matrix for left camera
    successFlag &=  calibFile.getValue(cameraNames[0] + resolution +"/fx",&val);
    m_KLeft.at<double>(0, 0) =val;

    successFlag &=   calibFile.getValue(cameraNames[0] + resolution +"/fy",&val);
    m_KLeft.at<double>(1, 1) = val;

    successFlag &=   calibFile.getValue(cameraNames[0] + resolution +"/cx",&val);
    m_KLeft.at<double>(0, 2) = val;

    successFlag &=   calibFile.getValue(cameraNames[0] + resolution +"/cy",&val);
    m_KLeft.at<double>(1, 2) = val;


    // get intrinsic matrix for right camera
    successFlag &=    calibFile.getValue(cameraNames[1] + resolution +"/fx",&val);
    m_KRight.at<double>(0, 0) = val;

    successFlag &=   calibFile.getValue(cameraNames[1] + resolution +"/fy",&val);
    m_KRight.at<double>(1, 1) = val;

    successFlag &=    calibFile.getValue(cameraNames[1] + resolution +"/cx",&val);
    m_KRight.at<double>(0, 2) = val;

    successFlag &=   calibFile.getValue(cameraNames[1] + resolution +"/cy",&val);
    m_KRight.at<double>(1, 2) = val;


    // get radial distortion parameters for left camera
    successFlag &=    calibFile.getValue(cameraNames[0] + resolution +"/k1",&val);
    m_radDistLeft.at<double>(0) = val;

    successFlag &=   calibFile.getValue(cameraNames[0] + resolution +"/k2",&val);
    m_radDistLeft.at<double>(1) = val;



    // get radial distortion parameters for right camera
    successFlag &=   calibFile.getValue(cameraNames[1] + resolution +"/k1",&val);
    m_radDistRight.at<double>(0) = val;

    successFlag &=   calibFile.getValue(cameraNames[1] + resolution +"/k2",&val);
    m_radDistRight.at<double>(1) = val;


    //load stereo calibration data
    successFlag &=   calibFile.getValue("STEREO/Baseline",&val);
    m_T.at<double>(0,0) = -val/1000.0;// in meters
    m_baselineCM = val/10.0f;
    float rx,ry,rz;


    successFlag &=   calibFile.getValue("STEREO/RX_" + resolution,&rx);
    successFlag &=   calibFile.getValue("STEREO/RZ_" + resolution,&rz);
    successFlag &=   calibFile.getValue("STEREO/CV_" + resolution,&ry);


    cv::Mat rVec = cv::Mat(3, 1, CV_64F);
    rVec.at<double>(0,0) = rx;
    rVec.at<double>(1,0) = ry;
    rVec.at<double>(2,0) = rz;
    cv::Rodrigues(rVec,m_R);





    if(!successFlag)
        return  ERRORCODE::NO_CALIBRATION;
    else
        return ERRORCODE::SUCCESS;


}



std::string zedCamera::resolution2String(RESOLUTION res){

    std::string resStr;

    switch(res){
    case  RESOLUTION::WVGA:
        resStr = "VGA";
        break;

    case RESOLUTION::HD:
        resStr = "HD";
        break;
    case  RESOLUTION::FHD:
        resStr = "FHD";
        break;
    case RESOLUTION::RES2K:
        resStr = "2K";
        break;

    default:
        resStr = "";

    }

    return resStr;
}




void zedCamera::savePointcloud(std::string fname,bool binFlag, std::string format)const{

    if(binFlag){ //open file as binary

    }

    colorImage left = m_rawImage(cv::Rect(0,0,m_width,m_height));

    colorImage rectLeft;

    cv::remap(left,rectLeft,m_xLeftRectify,m_yLeftRectify,cv::INTER_AREA);
    pointcloudImage pcIm = zedCamera::getPointCloudImage();




    //cycle through everything and check for invalid z

    int invalidPts = 0;
    for(int r = 0; r < pcIm.rows; r++){
        const float* ptPtr = pcIm.ptr<float>(r);
        for(int c = 0; c< pcIm.cols; c++){

            if((int) ptPtr[3*c+2] == 10000)
                invalidPts++;

        }
    }





    //for now doing ascii

    if(format.compare("PLY")==0){
        std::ofstream outFile(fname+".ply");

        if (outFile.is_open())
        {

            outFile << "ply" <<std::endl;
            outFile << "format ascii 1.0" <<std::endl;
            outFile << "element vertex "<< pcIm.rows * pcIm.cols -invalidPts <<std::endl; //    element vertex 6607
            outFile << "property float x" <<std::endl;
            outFile << "property float y" <<std::endl;
            outFile << "property float z" <<std::endl;
            outFile << "property uchar red" <<std::endl;
            outFile << "property uchar green" <<std::endl;
            outFile << "property uchar blue" <<std::endl;
            outFile << "end_header" <<std::endl;


            for(int r = 0; r < pcIm.rows; r++){
                const float* ptPtr = pcIm.ptr<float>(r);
                const uchar* colorPtr = rectLeft.ptr<uchar>(r);
                for(int c = 0; c< pcIm.cols; c++){

                    if((int) ptPtr[3*c+2] == 10000)
                        continue;
                    float x  =  ptPtr[3*c];
                    float y  =  ptPtr[3*c+1];
                    float z  =  ptPtr[3*c+2];

                    float b = colorPtr[3*c] ;
                    float g = colorPtr[3*c+1] ;
                    float r = colorPtr[3*c+2] ;

                    outFile <<x <<" " << y <<" " << z<<" "<<r <<" " << g <<" " << b<<std::endl;
                }
            }


            outFile.close();
        }
        else throw std::runtime_error("failed to create file at " + fname);

    } else if(format.compare("XYZ")==0){
        std::ofstream outFile(fname+".xyz");

        if (outFile.is_open())
        {

            outFile << "# pointcloud generated by openZed library" <<std::endl;

            for(int r = 0; r < pcIm.rows; r++){
                const float* ptPtr = pcIm.ptr<float>(r);

                for(int c = 0; c< pcIm.cols; c++){
                    if((int) ptPtr[3*c+2] == 10000)
                        continue;

                    outFile << ptPtr[3*c] <<" " << ptPtr[3*c+1] <<" " << ptPtr[3*c+2] <<std::endl;
                }
            }


            outFile.close();
        }
        else throw std::runtime_error("failed to create file at " + fname);
    } else if(format.compare("PCD")==0){//!PCD is untested\todo
        std::ofstream outFile(fname+".pcd");

        if (outFile.is_open())
        {

            outFile << "# .PCD v.7 - Point Cloud Data file format" <<std::endl;
            outFile << "VERSION .7" <<std::endl;
            outFile << "FIELDS x y z rgb "  <<std::endl; //    element vertex 6607
            outFile << "SIZE 4 4 4 4" <<std::endl;
            outFile << "TYPE F F F F" <<std::endl;
            outFile << "property float z" <<std::endl;
            outFile << "COUNT 1 1 1 1" <<std::endl;
            outFile << "WIDTH "<<pcIm.rows * pcIm.cols -invalidPts <<std::endl;
            outFile << "HEIGHT "<<1 <<std::endl;
            outFile << "VIEWPOINT 0 0 0 1 0 0 0" <<std::endl;
            outFile << "POINTS "<<pcIm.cols*pcIm.rows <<std::endl;
            outFile << "DATA ascii" <<std::endl;


            for(int r = 0; r < pcIm.rows; r++){
                const float* ptPtr = pcIm.ptr<float>(r);
                const uchar* colorPtr = rectLeft.ptr<uchar>(r);
                for(int c = 0; c< pcIm.cols; c++){

                    if((int) ptPtr[3*c+2] == 10000)
                        continue;

                    unsigned int r = colorPtr[3*c+2];
                    unsigned int g = colorPtr[3*c+1];
                    unsigned int b = colorPtr[3*c];
                    float colorFloat = (r <<16) &  (g << 8) &  b;

                    outFile << ptPtr[3*c] <<" " << ptPtr[3*c+1] <<" " << ptPtr[3*c+2] <<" "<< colorFloat<<std::endl;
                }
            }


            outFile.close();
        }
        else throw std::runtime_error("failed to create file at " + fname);

    }    else throw std::runtime_error("Unkown pointcloud format: " + format);



}




}}
