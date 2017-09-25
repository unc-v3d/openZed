#include "openzed_camera.h"

 #include <QDebug>
#include <opencv2/opencv.hpp>

namespace nfs {
namespace apps {

OpenzedCamera::OpenzedCamera(const vision::zedCamera::IMAGE& image_type,const vision::zedCamera::RESOLUTION& image_res,const int& fps,const QString path_to_camera,
                             const QString path_to_config_file, QObject* parent)
    : image_type_(image_type), image_res_(image_res),fps_(fps),path_to_camera_(path_to_camera),
      path_to_config_file_(path_to_config_file),
      QObject(parent) {

}

int OpenzedCamera::GetHeight() { return camera_->getHeight(); }

int OpenzedCamera::GetWidth() { return camera_->getWidth(); }

cv::Mat OpenzedCamera::GetFrame() {
  const int width = GetWidth();
  const int height = GetHeight();

  cv::Mat LR(height, width * 2, CV_8UC3, 1);

  is_image_valid_ = camera_->step();
  if (is_image_valid_) {
    cv::Mat Limage(height, width, CV_8UC3, 1);
    zedImage left = camera_->getImage(vision::zedCamera::SIDE::LEFT, image_type_);

    memcpy(Limage.data, left.data, width * height * 3 * sizeof(uchar));

    cv::Mat Rimage(height, width, CV_8UC3, 1);
    zedImage right =
        camera_->getImage(vision::zedCamera::SIDE::RIGHT, image_type_);

    memcpy(Rimage.data, right.data, width * height * 3 * sizeof(uchar));

    hconcat(Limage, Rimage, LR);
  }
  return LR;
}

bool OpenzedCamera::IsImageGood() const { return is_image_valid_; }

void OpenzedCamera::ProcessERRCODE(nfs::vision::zedCamera::ERRORCODE zed_error) {
  QString msg;
  msg.fromStdString(vision::zedCamera::printError(zed_error));

  if (!msg.isEmpty()) {
    qDebug() << msg;
  }
}

void OpenzedCamera::ConstructCamera() {
bool convert_to_int;
const int camera_id  = path_to_camera_.toInt(&convert_to_int);
if(convert_to_int){   
// This means a value was inserted, useful in windows.
camera_.reset(nullptr);
camera_.reset(new vision::zedCamera(
      camera_id, path_to_config_file_.toStdString(),
      image_res_, fps_, true, true));


} else {
camera_.reset(nullptr);
  camera_.reset(new vision::zedCamera(
      path_to_camera_.toStdString(), path_to_config_file_.toStdString(),
      image_res_, fps_, true, true));
 }
 ProcessERRCODE(camera_->init());
//qDebug() << "Opening camera at " << path_to_camera_ << " with  config info at " << path_to_config_file_ << " at fps " << fps_ << " at res " << int(image_res_) << " of image type " << int(image_type_) ;
}

void OpenzedCamera::SetFps(const int fps) {
  fps_ = fps;
  ConstructCamera();
}
void OpenzedCamera::SetImageResolution(
    const vision::zedCamera::RESOLUTION resolution) {
  image_res_=resolution;
  ConstructCamera();
}
void OpenzedCamera::SetImageType(const vision::zedCamera::IMAGE image_type) {
  image_type_ = image_type;
  ConstructCamera();
}

void OpenzedCamera::SetCameraPath(const QString camera_path) {
  path_to_camera_ = camera_path;
  ConstructCamera();
}
void OpenzedCamera::SetCamerConfigPath(const QString camera_config_path) {
  path_to_config_file_ = camera_config_path;
  ConstructCamera();
}
 

void OpenzedCamera::Initialize(){
  ConstructCamera();
}
}  // namespace apps
}  // namespace nfs
