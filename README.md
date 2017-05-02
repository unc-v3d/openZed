This is a simple interface to use zed camera (https://www.stereolabs.com/).
It has a GPU and CPU version for stereo

The main GPU stereo code can be found at https://github.com/unc-v3d/real_time_stereo
This code has been adapted to use with zed camera

This GPU code requires cudaArray (https://github.com/trueprice/cudaArray). This is automatically downloaded using git and cmake

This code has been tested under ubuntu

src/a.conf is an example calibration file which comes with the zed camera
To use it,  open a terminal 
git clone https://github.com/unc-v3d/openZed.git  \
cd openZed \
mkdir build \
cd build \
cmake .. \
make install
