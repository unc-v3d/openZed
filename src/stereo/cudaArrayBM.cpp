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


#include"cudaArrayBM.h"
#include<exception>
#define MAX_DISPARITY_GPU 128
#include<cuda.h>
namespace nfs{ namespace vision{


cudaArrayBM::cudaArrayBM(const stereoMatcherInterface::stereoParams params):stereoMatcherInterface(params),m_stereoBlockMatcher(nullptr){

  //  m_xyLeftMap = cv::Mat::zeros(params.height,params.width,CV_32FC2);
  //   m_xyRightMap = cv::Mat::zeros(params.height,params.width,CV_32FC2);

                int deviceCount;
                cudaError_t e = cudaGetDeviceCount(&deviceCount);
                if(e != cudaSuccess)
                    throw std::runtime_error("No Cuda enabled GPU found");

}


cudaArrayBM::~cudaArrayBM(){
    if(m_stereoBlockMatcher != nullptr)
        delete m_stereoBlockMatcher;
}

void cudaArrayBM::loadOptions(const std::string path2File){


   unsigned int SADWindowSize =11;
   unsigned int minDisparity= 1;
   unsigned int numDisparity = 128;



    const ext_stereo::StereoMatcher::Options options = {
                             (unsigned int)m_params.width,(unsigned int) m_params.height,
                              SADWindowSize,minDisparity, numDisparity,m_params.baseline ,m_params.focalLength , 1.0f, MAX_DISPARITY_GPU};


     m_stereoBlockMatcher = new ext_stereo::StereoMatcher(options);

}

void cudaArrayBM::initUndistortRectify(const lookupMap xLeftMap, const lookupMap yLeftMap,const lookupMap xRightMap,const lookupMap yRightMap){

    m_xLeftMap = xLeftMap;
    m_yLeftMap = yLeftMap;


    m_xRightMap = xRightMap;
    m_yRightMap = yRightMap;

    //copy data to combined xy maps
    std::vector<lookupMap> xyLeftMapVec;
    xyLeftMapVec.push_back(m_xLeftMap);
    xyLeftMapVec.push_back(m_yLeftMap);
    cv::merge(xyLeftMapVec, m_xyLeftMap);

    std::vector<lookupMap> xyRightMapVec;
    xyRightMapVec.push_back(m_xRightMap);
    xyRightMapVec.push_back(m_yRightMap);
    cv::merge(xyRightMapVec, m_xyRightMap);

    m_stereoBlockMatcher->initUndistortRectifyMaps(m_xyLeftMap.data, m_xyRightMap.data);

}

colorImage cudaArrayBM::undistortRectify(const colorImage image, const bool isLeft){
    colorImage retImage;
    //!\todo Akash:not using GPU, convert it
    if(isLeft){
        cv::remap(image ,   retImage, m_xLeftMap , m_yLeftMap, cv::INTER_AREA);
    } else{
        cv::remap(image ,   retImage, m_xRightMap, m_yRightMap, cv::INTER_AREA);
    }
    return retImage;
}

void cudaArrayBM::undistortRectify(colorImage rawLeftImage, colorImage rawRightImage){

    m_rectifiedLeft = undistortRectify(rawLeftImage, true);
    m_rectifiedRight = undistortRectify(rawRightImage, false);


    cv::Mat imLeftGrey, imRightGrey;

    cv::cvtColor(m_rectifiedLeft,imLeftGrey ,CV_BGR2GRAY);
    cv::cvtColor(m_rectifiedRight,imRightGrey,CV_BGR2GRAY);

    m_stereoBlockMatcher->init_frame(imLeftGrey.data, imRightGrey.data);
}

colorImage cudaArrayBM::getRectifiedImage(const bool isLeft) const {

    if(isLeft){
        return m_rectifiedLeft;
    } else{
        return m_rectifiedRight;
    }
}

void cudaArrayBM::match(){

    m_stereoBlockMatcher->match();
    m_stereoBlockMatcher->download_disparity(m_disparity.data);

}

measureImage cudaArrayBM::getDisparity(){
    return m_disparity;
}

measureImage cudaArrayBM::getDepth(){
    //!\todo Akash:not using GPU, convert it
    measureImage depth = cv::Mat::zeros(m_disparity.rows,m_disparity.cols, CV_32FC1);
    const float fb = m_params.focalLength * m_params.focalLength;
    for(  int r  = 0; r <m_disparity.rows; r++){

        const float* dataPtr = m_disparity.ptr<measureType>(r);
        float* depthPtr = depth.ptr<measureType>(r);

        for(  int c  = 0; c <m_disparity.cols; c++){

            const measureType disparityValue = dataPtr[c];

            if(std::abs(disparityValue) < FLOAT_EPSILON){
                depthPtr[c] = INVALID_DEPTH;
                continue;
            }
            depthPtr[c] = fb/disparityValue;
        }
    }
    return depth;
}

pointcloudImage cudaArrayBM::getPointCloudImage(){
    //!\todo Akash:not using GPU, convert it
    pointcloudImage depth3DImage;
    cv::reprojectImageTo3D(  m_disparity, depth3DImage, m_params.disparity2depth, true ); //sets for invalid disparity, depth = 10000
    return depth3DImage;
}

}}
