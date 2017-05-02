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
#ifndef STEREOINTERFACE_H
#define STEREOINTERFACE_H
#include<string>
#include<opencv2/opencv.hpp>
#include"../datatypes.h"
namespace nfs{


/*! \namespace vision
 *  \brief This library was developed for autonomous car,  this namespace takes care of the vision part of autonomous car
 *
 */
namespace vision{


/*!
 * \brief The stereoMatcher class this is a interface clas every stereoMatching class should derive from if you want to change the stereomatching algorithm
 */
class stereoMatcherInterface{
public:

    typedef struct stereoParams{
        //    cv::Mat leftK;
        //  cv::Mat rightK;
        // cv::Mat leftRadialParams;
        // cv::Mat rightRadialParams;
        //cv::Mat rightCameraPose;
        cv::Mat disparity2depth;
        float baseline;
        float focalLength;
        int width;
        int height;
    }stereoParams;



    stereoMatcherInterface(const stereoParams params): m_params(params){}
    virtual ~stereoMatcherInterface(){}

    virtual void loadOptions(const std::string path2File) = 0;
    virtual void initUndistortRectify(const lookupMap xLeftMap, const lookupMap yLeftMap,const lookupMap xRightMap,const lookupMap yRightMap) = 0;
    virtual colorImage undistortRectify(const colorImage image, const bool isLeft) = 0;
    virtual void undistortRectify(colorImage rawLeftImage, colorImage rawRightImage) = 0;
    virtual colorImage getRectifiedImage(const bool isLeft) const = 0;
    virtual void match() = 0;
    virtual measureImage getDisparity() = 0;
    virtual measureImage getDepth() = 0;
    virtual pointcloudImage getPointCloudImage() = 0;

    //   virtual colorImage d_getOriginalImage(const bool isLeft) = 0; //typically used for debugging GPU code
protected:
    measureImage m_disparity;
    lookupMap m_xLeftMap;
    lookupMap m_yLeftMap;

    lookupMap m_xRightMap;
    lookupMap m_yRightMap;
    colorImage m_rectifiedLeft;
    colorImage m_rectifiedRight;
    const stereoParams m_params;

};

}}
#endif //STEREOINTERFACE_H
