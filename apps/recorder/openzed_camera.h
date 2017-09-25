#ifndef APPS_RECORDER_OPENZED_CAMERA_H_
#define APPS_RECORDER_OPENZED_CAMERA_H_

#include "zedCamera.h"

#include <memory>

#include <QObject>
#include <QString>

namespace nfs {
namespace apps {

class OpenzedCamera : public QObject {
Q_OBJECT
  using zedImage = vision::colorImage;

 public:
  OpenzedCamera(const vision::zedCamera::IMAGE& image_type,const vision::zedCamera::RESOLUTION& image_res,const int& fps, const QString path_to_camera, const QString path_to_config_file,
                QObject* parent = nullptr);
  int GetHeight();
  int GetWidth();
  cv::Mat GetFrame();
  bool IsImageGood() const;
 void Initialize();
 public slots:
  void SetFps(const int fps);
  void SetImageResolution(const vision::zedCamera::RESOLUTION resolution);
  void SetImageType(const vision::zedCamera::IMAGE image_type);
  void SetCameraPath(const QString camera_path);
  void SetCamerConfigPath(const QString camera_config_path);
//signals:
//void Intialized();
 private:
  QString path_to_camera_;
  QString path_to_config_file_;
  std::unique_ptr<vision::zedCamera> camera_;
 
  bool is_image_valid_;
  vision::zedCamera::IMAGE image_type_ ;
  vision::zedCamera::RESOLUTION image_res_;
  int fps_;

  void ConstructCamera();
  static void ProcessERRCODE(vision::zedCamera::ERRORCODE zed_error);
};

}  // namespace apps
}  // namespace nfs

#endif  // APPS_RECORDER_OPENZED_CAMERA_H_
