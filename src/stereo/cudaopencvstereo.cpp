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

#include"cudaopencvstereo.h"
#include<exception>

#include <opencv2/cudaarithm.hpp> // for divide in getDepth
#include<opencv2/cudawarping.hpp> // for remap
#include<opencv2/cudaimgproc.hpp> // for cvtColor in depthmap compute

namespace nfs{ namespace vision{

opencvStereoGPU::opencvStereoGPU(const stereoMatcherInterface::stereoParams params,const METHOD methodType, const int gpuID, const bool verbose):stereoMatcherInterface(params),m_method(methodType), m_verbose(verbose){

    const int numberOfGPUs = cv::cuda::getCudaEnabledDeviceCount();

    if(numberOfGPUs == 0)
        throw std::runtime_error("No Cuda enabled GPU found");

    if(gpuID <= numberOfGPUs-1)
        cv::cuda::setDevice(gpuID);



    const int gpuIDUsed = cv::cuda::getDevice();

    if(m_verbose)
        std::cout<<"Using GPU: " << gpuIDUsed <<std::endl;


}

opencvStereoGPU::~opencvStereoGPU(){

}

void opencvStereoGPU::loadOptions(const std::string path2File) {

    int SADWindowSize =11;
    //   int minDisparity= 1;
    int numDisparity = 128;


   switch(m_method){
    case METHOD::BLOCK_MATCHING:
       m_stereoBlockMatcher = cv::cuda::createStereoBM(numDisparity,SADWindowSize);
        break;

   case METHOD::BELIEF_PROPAGATION:
       m_bp = cv::cuda::createStereoBeliefPropagation(numDisparity);
    break;

   case METHOD::CONSTANT_SPACE_BP:
       m_csbp = cv::cuda::createStereoConstantSpaceBP(numDisparity);
   break;
   }

   m_filter = cv::cuda::createDisparityBilateralFilter(numDisparity,15,10); //5 = filter size



//median filtering to denoise disparity image, then applying a bilateral filter to smooth

}

void opencvStereoGPU::initUndistortRectify(const lookupMap xLeftMap, const lookupMap yLeftMap,const lookupMap xRightMap,const lookupMap yRightMap){

    m_xLeftMap = xLeftMap;
    m_yLeftMap = yLeftMap;

    m_xRightMap = xRightMap;
    m_yRightMap = yRightMap;

    m_xLeftMapGPU = cv::cuda::GpuMat(m_xLeftMap);
    m_yLeftMapGPU = cv::cuda::GpuMat(m_yLeftMap);

    m_xRightMapGPU = cv::cuda::GpuMat(m_xRightMap);
    m_yRightMapGPU = cv::cuda::GpuMat(m_yRightMap);

}

colorImage opencvStereoGPU::undistortRectify(const colorImage image, const bool isLeft){


    cv::cuda::GpuMat imageGPU = cv::cuda::GpuMat(image);
    cv::cuda::GpuMat retImageGPU = cv::cuda::GpuMat(image.rows,image.cols,image.type());

    if(isLeft){
        cv::cuda::remap(imageGPU ,   retImageGPU, m_xLeftMapGPU , m_yLeftMapGPU, cv::INTER_LINEAR);
    } else{
        cv::cuda::remap(imageGPU ,   retImageGPU, m_xRightMapGPU, m_yRightMapGPU, cv::INTER_LINEAR);
    }


    colorImage retImage;
    retImageGPU.download(retImage);
    return retImage;

}

void opencvStereoGPU::undistortRectify(colorImage rawLeftImage, colorImage rawRightImage){
    //not using function undistortRectify(const colorImage image, const bool isLeft) as done in CPU code for efficiency

    //upload to GPU
    cv::cuda::GpuMat rawLeftImageGPU = cv::cuda::GpuMat(rawLeftImage);
    cv::cuda::GpuMat rawRightImageGPU = cv::cuda::GpuMat(rawRightImage);

    //remap
    cv::cuda::remap(rawLeftImageGPU ,   m_leftGPU, m_xLeftMapGPU , m_yLeftMapGPU, cv::INTER_LINEAR);
    cv::cuda::remap(rawRightImageGPU ,   m_rightGPU, m_xRightMapGPU, m_yRightMapGPU, cv::INTER_LINEAR);

}

colorImage opencvStereoGPU::getRectifiedImage(const bool isLeft) const{
    //transform the previously computed GPU memory to CPU and return

    colorImage retImage;

    if(isLeft){
        m_leftGPU.download(retImage);
    } else{
        m_rightGPU.download(retImage);
    }

    return retImage;
}


void opencvStereoGPU::match(){

    cv::cuda::GpuMat imLeftGrey, imRightGrey, tmpDis;

    cv::cuda::cvtColor(m_leftGPU,imLeftGrey, CV_BGR2GRAY);
    cv::cuda::cvtColor(m_rightGPU,imRightGrey, CV_BGR2GRAY);


    switch(m_method){
     case METHOD::BLOCK_MATCHING:
       m_stereoBlockMatcher->compute(imLeftGrey, imRightGrey, tmpDis);
         break;

    case METHOD::BELIEF_PROPAGATION:
        m_bp->compute(imLeftGrey, imRightGrey, tmpDis);
     break;

    case METHOD::CONSTANT_SPACE_BP:
         m_csbp->compute(imLeftGrey, imRightGrey, tmpDis);
    break;
    }

m_filter->apply(tmpDis,m_leftGPU,m_disparityGPU);
    //donot download
}

measureImage opencvStereoGPU::getDisparity(){
    cv::Mat tmpDisparity;
    m_disparityGPU.download(tmpDisparity);

    measureImage cpuDisparity;

    tmpDisparity.convertTo(cpuDisparity, CV_32FC1);
    return cpuDisparity;

}
measureImage opencvStereoGPU::getDepth(){

    //transform disparity into depth, download and return

    const float fb = m_params.focalLength*m_params.baseline;

    cv::cuda::GpuMat depthGPU;
    cv::cuda::divide(m_disparityGPU,fb,depthGPU);

    cv::Mat tmpDepth;
    depthGPU.download(tmpDepth);


    measureImage cpuDepth;
    tmpDepth.convertTo(cpuDepth, CV_32FC1);


    for(  int r  = 0; r <cpuDepth.rows; r++){

        float* depthPtr = cpuDepth.ptr<measureType>(r);


        for(  int c  = 0; c <cpuDepth.cols; c++){

            measureType depth = depthPtr[c];

            if(std::isinf(depth)){
                depthPtr[c] = INVALID_DEPTH;
            }

        }
    }




    return cpuDepth;
}

pointcloudImage opencvStereoGPU::getPointCloudImage(){

    cv::cuda::GpuMat pcGPU;

    cv::cuda::reprojectImageTo3D(m_disparityGPU, pcGPU,m_params.disparity2depth,3); // 3channel output

    pointcloudImage retPC;

    pcGPU.download(retPC);



    for(  int r  = 0; r <retPC.rows; r++){

        float* depthPtr = retPC.ptr<measureType>(r);

        for(  int c  = 0; c <retPC.cols; c++){

            const   measureType x = depthPtr[3*c];
            const   measureType y = depthPtr[3*c+1];
            const   measureType z = depthPtr[3*c+2];

            if(std::isinf(x) || std::isinf(y) || std::isinf(z)){
                depthPtr[3*c+2]     = INVALID_DEPTH;

            }

        }
    }


    return retPC;






}


 

}}

