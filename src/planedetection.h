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

//series of plane fitting methods to the depth estimates from zed camera
#ifndef PLANEDETECTION_H
#define PLANEDETECTION_H
#include"datatypes.h"



namespace nfs{ namespace vision{

/*!
 * \brief The planeDetectBasic class This is a static class for single plane fitting to the point cloud obtained from the zed camera. The intension is to find ground plane
 */
class planeDetectBasic{
public:

    /*!
     * \brief detect detects the ground plane using the depth data, disparity is not allowed
     * \param depthMap depth values in float cv::Mat
     * \param intrinsicMatrix intrisic matrix cv::Mat<double> of the rectified left frame
     * \param camHtInMeters distance (+ve) of camera above the ground in meters
     * \param [out] maskImage Output mask indicating the points filtered by the input filter settings, if RANSAC is not used, also indicates points used to estimate plane, if RANSAC is used a subset of theres points is used
     * \param tolerance tolerance in meters around the plane
     * \param useRANSAC flag to use RANSAC
     * \param inlierProbability if RANSAC is used this values indicates inlier probability to be used
     * \return equation of plane in 3D aX + bY + cZ +d =0 as  [a, b, c, d]
     */
    static cv::Scalar detect(measureImage const &  depthMap, const cv::Mat intrinsicMatrix,
                             const float camHtInMeters , cv::Mat &maskImage, const float tolerance = 0.05f, bool useRANSAC = false, const float inlierProbability = 0.7f);
private:

    /*!
     * \brief planeDetectBasic empty private constructor to avoid initialization
     */
    planeDetectBasic(){}


    /*!
 * \brief filterPoints filters points around the defined height below the camera and allows points which are within the tolerance
 * \param depthMap depth values in float cv::Mat
 * \param intrinsicMatrix  intrisic matrix cv::Mat<double> of the rectified left frame
 * \param camHtInMeters distance (+ve) of camera above the ground in meters
 * \param maskImage [out] maskImage Output mask indicating the points filtered by the input filter settings, if RANSAC is not used, also indicates points used to estimate plane, if RANSAC is used a subset of theres points is used
 * \param tolerance tolerance in meters around the plane
 * \return vector of filtered 3D points, also sets the mask
 */
    static std::vector<cv::Point3d> filterPoints(measureImage const &  depthMap,cv::Mat const intrinsicMatrix,
                                                 const float camHtInMeters, cv::Mat& maskImage, const float tolerance);


    /*!
 * \brief fitPlane given a set of N 3D homogeneous points, fits a plane to it using all N points
 * \param homoPtsMat Nx4 cv::Mat of N homogeneous 3D points
 * \return fitted plane as cv::Mat
 */
    static cv::Mat fitPlane(cv::Mat homoPtsMat);



    /*!
 * \brief evalFitPlane function to evaluate how well the plane fits the data
 * \param model model of the plane
 * \param data  Nx4 cv::Mat of N homogeneous 3D points
 * \param [out] supportIdx indices of all the points which support the plane model within threshold
 * \param threshold theshold for support
 * \return number of points which support the plane model
 */
    static   unsigned int evalFitPlane(cv::Mat model, cv::Mat data,cv::Mat supportIdx, const float threshold);
};




/*!
 * \brief The RANSAC class generic class to apply RANSAC for robust fitting of model. It uses mathematically correct number of iterations, given the inlierProbability.
 *          Needs the functions to evaluate the fit of a model and a separate function to estimate a model
 */
class RANSAC{

public :
    /*!
     * \brief RANSAC constructor to set up the mathematically correct number of iterations, given the inlierProbability.
     * \param inlierProbability probability that a point is inlier, higher value indicats cleaner data
     * \param threshold observations inside threshold from the model are acceptable
     * \param numSample number of observations required to estimate the model
     */
    RANSAC(const float inlierProbability, const float threshold, const unsigned int numSample);


    /*!
     * \brief fit estimates a fitting model according to fittingFunction to the data
     * \param data cv::Mat NxM matrix, where threes are N observations
     * \param supportIdx Nx1 matrix, 255 indicates the support of the observation to the model
     * \param fittingFunction function pointer to a function with data as input and output as model
     * \param evalFit function pointer to a function with first arg as model, second as data NxM, thrid as supportIdx and the last as threshold  for support to the model, returns th number of supporting obsrevations
     * \return best fitting robust estimate of the model
     */
    cv::Mat fit(const cv::Mat data, cv::Mat& supportIdx, cv::Mat (*fittingFunction)(cv::Mat),  unsigned int (*evalFit)(cv::Mat, cv::Mat, cv::Mat, const float));
private:

    const float  m_inlierProb; //!< input, inlier probability
    const float  m_threshold; //!< input, threshold for support to the model
    const unsigned int  m_nS; //!< input, number of observations required to estimate the model
    const unsigned int m_nIter;//!< computed internally, number of iterations RANSAC must run to get the model which guarantees that the correct inlier observations were sampled in RANSAC loop at least once

};

}}
#endif // PLANEDETECTION_H
