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
#ifndef CUDA_OPENCV_STEREO_H
#define CUDA_OPENCV_STEREO_H

#include<opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>

#include"stereointerface.h"

 
namespace nfs{

namespace vision{

class opencvStereoGPU : public stereoMatcherInterface {
public:

    enum class METHOD{
        BLOCK_MATCHING,
        BELIEF_PROPAGATION,
        CONSTANT_SPACE_BP
    };

    opencvStereoGPU(const stereoMatcherInterface::stereoParams params, const METHOD methodType = METHOD::BLOCK_MATCHING, const int gpuID = 0, const bool verbose = false);
    ~opencvStereoGPU();

    void loadOptions(const std::string path2File) ;
    void initUndistortRectify(const lookupMap xLeftMap, const lookupMap yLeftMap,const lookupMap xRightMap,const lookupMap yRightMap);
    colorImage undistortRectify(const colorImage image, const bool isLeft);
    void undistortRectify(colorImage rawLeftImage, colorImage rawRightImage);
    colorImage getRectifiedImage(const bool isLeft) const;
    void match();
    measureImage getDisparity();
    measureImage getDepth();
    pointcloudImage getPointCloudImage();


private:
    const METHOD m_method;
    const bool m_verbose;

    cv::Ptr<cv::cuda::StereoBM> m_stereoBlockMatcher;
    cv::Ptr<cv::cuda::StereoBeliefPropagation> m_bp;
    cv::Ptr<cv::cuda::StereoConstantSpaceBP> m_csbp;
    cv::Ptr<cv::cuda::DisparityBilateralFilter> m_filter;

    cv::cuda::GpuMat m_disparityGPU;
    cv::cuda::GpuMat m_depthGPU;

    cv::cuda::GpuMat m_rightGPU;
    cv::cuda::GpuMat m_leftGPU;

    cv::cuda::GpuMat m_xLeftMapGPU;
    cv::cuda::GpuMat m_yLeftMapGPU;

    cv::cuda::GpuMat m_xRightMapGPU;
    cv::cuda::GpuMat m_yRightMapGPU;

};

 

}}
#endif //CUDA_OPENCV_STEREO_H
