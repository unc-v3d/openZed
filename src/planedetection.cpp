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
#include <exception>
#include<random>
#include "planedetection.h"
namespace nfs{ namespace vision{

cv::Scalar planeDetectBasic::detect(zedCamera::measureImage const &  depthMap,cv::Mat const intrinsicMatrix,
                                    const float camHtInMeters, cv::Mat& maskImage, const float tolerance , bool useRANSAC, const float inlierProbability){

    cv::Scalar plane = cv::Scalar(0,0,0,0);
    cv::Mat model ;

    std::vector<cv::Point3d> pts = filterPoints(  depthMap,intrinsicMatrix,camHtInMeters, maskImage,tolerance );


    if(pts.empty())
        return plane;

    cv::Mat ptsMat = cv::Mat(pts).reshape(1); // changes the number of channels into column
    cv::Mat homoPtsMat;
    cv::Mat oneVec = cv::Mat::zeros(ptsMat.rows,1, CV_64F) + 1;

    cv::hconcat(ptsMat, oneVec, homoPtsMat);


    if(useRANSAC){ //use a RANSAC, pick 3 points  create a plane , find support do itrerations for inlier ratio of inlierRatio
        //using 4 samples so that the fitting is easier
        RANSAC ransac(  inlierProbability, 0.1f, 4);
        cv::Mat supportIdx;
        model = ransac.fit( homoPtsMat,  supportIdx,   fitPlane, evalFitPlane);




    } else{// by default uses all the points which pass the filter and does an linear best fit for all the points
        //create a mat out of the vector



        model = fitPlane(homoPtsMat);
    }


    plane[0] = model.at<double>(0) ;
    plane[1] = model.at<double>(1) ;
    plane[2] = model.at<double>(2) ;
    plane[3] = model.at<double>(3);



    return plane;
}


std::vector<cv::Point3d> planeDetectBasic::filterPoints(zedCamera::measureImage const &  depthMap,cv::Mat const intrinsicMatrix,
                                                        const float camHtInMeters, cv::Mat& maskImage, const float tolerance ){
    maskImage = cv::Mat::zeros(depthMap.rows,depthMap.cols,CV_8UC1);

    std::vector<cv::Point3d> pts;

    const double fx = intrinsicMatrix.at<double>(0,0);
    const double fy = intrinsicMatrix.at<double>(1,1);

    const double cx = intrinsicMatrix.at<double>(0,2);
    const double cy = intrinsicMatrix.at<double>(1,2);

    // In 3D $aX + bY + cZ +d = 0$

    //selectively use points in a tolerance of += 1 cm around the give height to fit the plane.

    for(int r = 0; r < depthMap.rows; r++){
        float const*  depthPtr = depthMap.ptr<float>(r);
        uchar* maskPtr = maskImage.ptr<uchar>(r);
        for( int c = 0; c < depthMap.cols; c++ ){
            //z is in meters
            float z = depthPtr[c];

            if( std::abs(z) < FLOAT_EPSILON ||  std::abs(z - 10000)  <  FLOAT_EPSILON ) // invalid depth flag for opencv
                continue;


            float  y = (r-cy)*z/fy;
            float x = (c-cx)*z/fx;
            //check for  zero z
            //         if( ((camHtInMeters - tolerance)* fy/ z + cy < r &&  (camHtInMeters + tolerance)* fy/ z + cy > r ) ||  (r > 500 && r < 600) ){

            //check for  zero z
            if(  y > (camHtInMeters - tolerance) && y < (camHtInMeters + tolerance) ){
                pts.push_back(cv::Point3d(x,y,z));
                maskPtr[c] = 255;
            }

        }
    }
    return pts;
}



cv::Mat planeDetectBasic::fitPlane(cv::Mat homoPtsMat){

    cv::Mat sol;
    cv::SVD::solveZ(homoPtsMat,sol);
    return sol;

}

unsigned int planeDetectBasic::evalFitPlane(cv::Mat model, cv::Mat data, cv::Mat supportIdx , const float threshold){
    //fit the model, see support and return number of supporting points
    //data is assumed to be homogeneous

    supportIdx = abs(data*model) < threshold;

    return static_cast<unsigned int>(cv::countNonZero(supportIdx));
}





std::vector<cv::Point3d> filterPointCloud::filterLIDARLike(zedCamera::measureImage const &  depthMap, const cv::Mat intrinsicMatrix,cv::Scalar groundPlaneEqn,
                                                           const float metersAbovePlane, const float tolerance){



    std::vector<cv::Point3d> pts;
    cv::Point3d normal(groundPlaneEqn[0],groundPlaneEqn[1],groundPlaneEqn[2]);
    const double divideNorm = cv::norm(normal);
    cv::Scalar  newPlane = groundPlaneEqn/divideNorm + cv::Scalar(0,0,0,-metersAbovePlane);

    const double fx = intrinsicMatrix.at<double>(0,0);
    const double fy = intrinsicMatrix.at<double>(1,1);

    const double cx = intrinsicMatrix.at<double>(0,2);
    const double cy = intrinsicMatrix.at<double>(1,2);


    for(int r = 0; r < depthMap.rows; r++){
        float const*  depthPtr = depthMap.ptr<float>(r);

        for( int c = 0; c < depthMap.cols; c++ ){
            //z is in meters
            float z = depthPtr[c];

            if( std::abs(z) < FLOAT_EPSILON ||  std::abs(z - 10000)  <  FLOAT_EPSILON ) // invalid depth flag for opencv
                continue;


            float  y = (r-cy)*z/fy;
            float x = (c-cx)*z/fx;


            if(  std::abs(newPlane[0]*x + newPlane[1]*y + newPlane[2]*z + newPlane[3]) < tolerance ){
                pts.push_back(cv::Point3d(x,y,z));

            }

        }
    }
    return pts;


}




cv::Mat filterPointCloud::transformPointCloud(std::vector<cv::Point3d> pointcloud,cv::Mat transform){
    assert(transform.cols == 4 && transform.rows == 4); // transform must be 4x4 matrix

    //assumes


    cv::Mat ptsMat = cv::Mat(pointcloud).reshape(1); // changes the number of channels into column
    cv::Mat homoPtsMat;
    cv::Mat oneVec = cv::Mat::zeros(ptsMat.rows,1, CV_64FC1) + 1;

    cv::hconcat(ptsMat, oneVec, homoPtsMat);

    cv::Mat homoTransformedPts = homoPtsMat *transform;


    cv::Mat transformedPts;
    homoTransformedPts(cv::Rect2i(0,0,3,homoTransformedPts.rows)).copyTo(transformedPts);


    return transformedPts;


}



cv::Mat filterPointCloud::topView(std::vector<cv::Point3d> pointcloud,cv::Scalar const& groundPlaneEqn,const float heightFromGroundPlane){
    //construct transform to get a top view where (x,y,z) -> up, right, forward
    //height is height from the origin of the original system


    cv::Point3d normal(groundPlaneEqn[0],groundPlaneEqn[1],groundPlaneEqn[2]);
    const double divideNorm = cv::norm(normal);

    normal = normal/divideNorm; //normal  is -z in new
    cv::Point3d newZ = -normal;
    cv::Point3d newX =  cv::Point3d(0,0,1);
    cv::Point3d newY =  newX.cross(newZ);
    newY = newY/ cv::norm(newY);

    newX = newY.cross(newZ);
    newX = newX/cv::norm(newX);

    //build transform
    cv::Mat t = cv::Mat::zeros(3,1,CV_64FC1);
    t.at<double>(2) = -heightFromGroundPlane;

    // see https://www.fastgraph.com/makegames/3drotation/
    //    The rows of R represent the coordinates in the original space of unit vectors along the coordinate axes of the rotated space. (Figure 1).
    //    The columns of R represent the coordinates in the rotated space of unit vectors along the axes of the original space
    // new{X,Y,Z} are orientation of unit vectors of rotatied space in original space

    double array[9] = { newX.x,newX.y,newX.z,
                       newY.x,newY.y,newY.z,
                       newZ.x,newZ.y,newZ.z};

    cv::Mat rot = cv::Mat(3, 3, CV_64F, array);

    t= rot*t;

    cv::Mat transform =cv::Mat::zeros(4, 4, CV_64F);

    rot.copyTo(transform(cv::Rect(0,0,3,3)));
    t.copyTo(transform(cv::Rect(3,0,1,3)));

    transform.at<double>(3,3) = 1;

    //transform the pointcloud

  return  filterPointCloud::transformPointCloud( pointcloud, transform);
}





RANSAC::RANSAC(const float inlierProbability, const float threshold, const unsigned int numSample):
    m_inlierProb(inlierProbability),m_threshold(threshold),m_nS(numSample),m_nIter(static_cast<unsigned int>(std::ceil(std::log(1-inlierProbability)/std::log(1- std::pow((1 - std::exp(1)) ,numSample)  ) ) )){

}


cv::Mat RANSAC::fit(cv::Mat const data, cv::Mat& supportIdx, cv::Mat (*fittingFunction)(cv::Mat minimalData), unsigned int (*evalFit)(cv::Mat, cv::Mat, cv::Mat,  const float)){

    //sample m_nS rows from data, use the fitting fuction to fit the model, use evalFit to check whether the model was any good
    unsigned int iterCount = 0;
    cv::Mat model;
    bool isModelGood = false;

    unsigned int supportNum = 0;
    //initialize
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,data.rows-1);
    //create m_nS random numbers





    cv::Mat minData = cv::Mat(static_cast<int>(m_nS),data.cols,data.type());
    supportIdx = cv::Mat(data.rows,1,CV_8UC1); // 255 indicate support, 0 indicates not

    //sample observations
    for(unsigned int i = 0; i < m_nS; i++){
        int rowIdx = distribution(generator);
        data.row(rowIdx).copyTo(minData.row(static_cast<int>(i)));
    }

    model = fittingFunction(minData);
    supportNum = evalFit(model,data,supportIdx, m_threshold);

    while(iterCount < m_nIter && !isModelGood){
        //sample m_nS points

        //sample observations
        for(unsigned int i = 0; i < m_nS; i++){
            int rowIdx = distribution(generator);
            data.row(rowIdx).copyTo(minData.row(static_cast<int>(i)));
        }

        cv::Mat  modelNew = fittingFunction(minData);

        unsigned int newSupport = evalFit(modelNew,data,supportIdx, m_threshold);

        if(newSupport > supportNum){
            modelNew.copyTo(model);
            supportNum = newSupport;
        }

        iterCount++;
    }

    return model;

}


}}

