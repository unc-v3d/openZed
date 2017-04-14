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

//file created for simple loading of .conf calibration files provided by zed camera

#ifndef CONFPARSER_H
#define CONFPARSER_H
#include <string>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <cstring>

namespace nfs{ namespace vision{


/*!
 * \brief The confParser class  utility functions to parse .conf files
 *        supports only conf files, assumes multiple lines per category, no nesting of category
 */
class confParser{

public:


 /*!
 * \brief confParser default constructor, does nothing
 */
confParser();


/*!
 * \brief confParser given path to .conf file containing the calibration data, loads it
 * \param path2File path to .conf file containing the calibration data
 */
confParser(std::string path2File){
	loadFile(path2File);
}



/*!
 * \brief loadFile loads all the calibration data into hashmaps
 * \param path2File path to .conf file containing calibration data
 * \return returns true if successful, otherwise false
 */
bool loadFile(std::string path2File){
//open file and start parsing, if something fails return false

//readline line by line 
std::ifstream calibFile(path2File, std::ifstream::in);

std::string headerLine;
	if(calibFile.is_open()){
	 
		while(getline(calibFile,headerLine)){
			if(headerLine.size() > 0){
				if(headerLine[0] =='['){ //create new hashmap
					
					 std::size_t idx = headerLine.find("]");
					  if (idx == std::string::npos)
  					 	return false;
					
					//add all the params to hashmap
					std::unordered_map<std::string,float> cameraParams; 
					std::string paramLine;
					while(getline(calibFile,paramLine)){
						if(paramLine.empty())
							break;
							
						if(paramLine.back() == '\r'){//if windows line ending, delete \r
							paramLine.pop_back();
						}

						std::size_t paramIdx = paramLine.find("=");

 						 if (paramIdx==std::string::npos ||  paramIdx == paramLine.size()-1)
	return false;

						std::string paramName =  paramLine.substr(0,paramIdx);
 						
						float paramValue = std::stof(paramLine.substr(paramIdx+1));
						

						 
						cameraParams.insert({paramName, paramValue});
					}
					
					std::string cameraName =  headerLine.substr(1,idx-1);					
				  	m_data.insert({cameraName, cameraParams});
										
				}	
			}
		}
	
	} else{
		std::cout <<  "Error: " << strerror(errno) << std::endl;
		std::cout <<  "Failed to open file at " << path2File << std::endl;
		calibFile.close();
		return false;
	}
//printMap();
 return true;
}





/*!
 * \brief getValue seraches the hashmap to get the value of the variable
 * \param variable  string name of variable, eg LEFT_CAM_HD/fx will give the fx for left camera
 * \param valuePtr [out] pointer where the value is written to
 * \return returns true if the string variable can be traversed successfully, false of no such value is in the calibration file
 */
bool getValue(std::string variable, float* valuePtr){
//expects a slash in the middle
// default write value is 0
	*valuePtr = 0;

 std::size_t idx = variable.find("/");

  if (idx==std::string::npos ||  idx == variable.size()-1)
   	return false;

std::string cameraType =  variable.substr(0,idx);
 
std::string param =  variable.substr(idx+1);
 
std::unordered_map<std::string, std::unordered_map<std::string,float> > ::iterator  i = m_data.find(cameraType);

if( i != m_data.end()) {

	//look for param 
	std::unordered_map<std::string,float>  paramMap = i->second;
	std::unordered_map<std::string,float>::iterator j = paramMap.find(param);
	
	if( j != paramMap.end()){
		*valuePtr = j->second; 
		return true;
	} else return false;

	
}
else return false;

}



/*!
 * \brief printMap prints all the contents of all hashmaps
 */
void printMap()const{


for(std::unordered_map<std::string, std::unordered_map<std::string,float> >::const_iterator it = m_data.begin(); it != m_data.end(); ++it){
  std::cout << it->first << std::endl;
for(   std::unordered_map<std::string,float>::const_iterator itParam = it->second.begin(); itParam != it->second.end(); ++itParam){
    std::cout << "\t" << itParam->first << " " << itParam->second <<std::endl;
}
 }

}
private:
std::unordered_map<std::string, std::unordered_map<std::string,float> > m_data; //!< contains all the calibration data, first key is the name eg LEFT_CAM_HD, second key is fx,or fy and so on

};

}}
#endif //CONFPARSER_H
