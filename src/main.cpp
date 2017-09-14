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

#include"zedCamera.h"
#include"planedetection.h"



int main(int argc, char** argv){

    if(argc <3){
        std::cout<<"program path2camera"<<std::endl;
        return -1;
    }

    std::string camName(argv[1]);
    std::string calibFilePath(argv[2]);
    double dist;
    sscanf(argv[3],"%lf",&dist);

    std::cout<<"Using camera at "<< camName<<std::endl;
    std::cout<<"Using calibration file at "<<calibFilePath<<std::endl;

    nfs::vision::zedCamera zcam(camName,calibFilePath,nfs::vision::zedCamera::RESOLUTION::HD, 30,true,true);



    nfs::vision::zedCamera::printError(zcam.init());
    bool validIm = true;
    char keypress = 'r';





    cv::Mat K = zcam.getRectifiedIntrinsicMatrix(nfs::vision::zedCamera::SIDE::LEFT);

    while(keypress != 'q' && validIm){
        validIm = zcam.step();

        if(validIm){
            cv::Mat mask;
            cv::Mat disparityImage  =  zcam.getImage( nfs::vision::zedCamera::IMAGE_MEASURE::DEPTH, 1,128);

            cv::Scalar plane = nfs::vision::planeDetectBasic::detect(disparityImage,K,dist,mask,0.1 , true, 0.8);


        //      std::cout << " Plane eqn is : "<< plane << std::endl;
            cv::Mat disp;


            disparityImage.convertTo(disp, CV_8UC1,0.25);
            cv::imshow("Disparity", disp);
            cv::imshow("Mask", mask);
            keypress = (char)cv::waitKey(1);

            if(keypress == 's'){
                //zcam.savePointcloud("pc",true,"XYZ");zcam.savePointcloud("pc",true,"PCD");
                std::vector<cv::Point3d>  vec =   nfs::vision::filterPointCloud::filterLIDARLike(   disparityImage,  K  ,  plane,  0.5, 0.02);
                cv::Mat  vecMat =   nfs::vision::filterPointCloud::topView(vec,plane,1);

                std::cout<<vecMat;


            }


        }
    }

}
