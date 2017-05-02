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
#ifndef DATATYPESZED_H
#define DATATYPESZED_H
#include<opencv2/opencv.hpp>
#define FLOAT_EPSILON 0.000001
#define INVALID_DEPTH 10000
//#define USE_GPU 1
namespace nfs{


/*! \namespace vision
 *  \brief This library was developed for autonomous car,  this namespace takes care of the vision part of autonomous car
 *
 */
namespace vision{

using measureType = float;
using pointcloudImage = cv::Mat_<cv::Vec3f>;
using measureImage = cv::Mat_<measureType>;// generic image to support both depth and disparity images
using colorImage   = cv::Mat_<cv::Vec<uchar,3> >;
using lookupMap   = cv::Mat_<float>;



}}
#endif //DATATYPESZED_H
