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


#include"opencvstereo.h"
#include<exception>
 
namespace nfs{ namespace vision{


opencvStereo::opencvStereo(const stereoMatcherInterface::stereoParams params):stereoMatcherInterface(params){}


opencvStereo::~opencvStereo(){}


void opencvStereo::loadOptions(const std::string path2File){
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
    int minDisparity= 1;
    int numDisparity = 128;


    m_stereoBlockMatcher = cv::StereoSGBM::create(minDisparity,    //int minDisparity
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


    m_depth = cv::Mat::zeros(m_params.height,m_params.width, CV_32FC1);

}

void opencvStereo::initUndistortRectify(const lookupMap xLeftMap, const lookupMap yLeftMap,const lookupMap xRightMap,const lookupMap yRightMap){

    m_xLeftMap = xLeftMap;
    m_yLeftMap = yLeftMap;


    m_xRightMap = xRightMap;
    m_yRightMap = yRightMap;
}

colorImage opencvStereo::undistortRectify(const colorImage image, const bool isLeft){
    colorImage retImage;

    if(isLeft){
        cv::remap(image ,   retImage, m_xLeftMap , m_yLeftMap, cv::INTER_AREA);
    } else{
        cv::remap(image ,   retImage, m_xRightMap, m_yRightMap, cv::INTER_AREA);
    }
    return retImage;
}

void opencvStereo::undistortRectify(colorImage rawLeftImage, colorImage rawRightImage){

    m_rectifiedLeft = undistortRectify(rawLeftImage, true);
    m_rectifiedRight = undistortRectify(rawRightImage, false);
}

colorImage opencvStereo::getRectifiedImage(const bool isLeft) const {

    if(isLeft){
        return m_rectifiedLeft;
    } else{
        return m_rectifiedRight;
    }
}

void opencvStereo::match(){

    cv::Mat imLeftGrey, imRightGrey;
    cv::Mat tmpDisparity;

    cv::cvtColor(m_rectifiedLeft,imLeftGrey ,CV_BGR2GRAY);
    cv::cvtColor(m_rectifiedRight,imRightGrey,CV_BGR2GRAY);

    m_stereoBlockMatcher->compute( imLeftGrey, imRightGrey, tmpDisparity);

    tmpDisparity.convertTo(m_disparity,CV_32FC1,1.0/16);

}

measureImage opencvStereo::getDisparity(){
    return m_disparity;
}

measureImage opencvStereo::getDepth(){


    const float fb = m_params.focalLength * m_params.focalLength;
    for(  int r  = 0; r <m_disparity.rows; r++){

        const float* dataPtr = m_disparity.ptr<measureType>(r);
        float* depthPtr = m_depth.ptr<measureType>(r);

        for(  int c  = 0; c <m_disparity.cols; c++){

            const measureType disparityValue = dataPtr[c];

            if(std::abs(disparityValue) < FLOAT_EPSILON){
                depthPtr[c] = INVALID_DEPTH;
                continue;
            }
            depthPtr[c] = fb/disparityValue;
        }
    }
      
    return m_depth;
}

pointcloudImage opencvStereo::getPointCloudImage(){

    pointcloudImage depth3DImage;
 
    cv::reprojectImageTo3D(  m_disparity, depth3DImage, m_params.disparity2depth, true ); //sets for invalid disparity, depth = 10000
    return depth3DImage;
}

}}
