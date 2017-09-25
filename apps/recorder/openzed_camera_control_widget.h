#ifndef APPS_RECORDER_OPENZED_CAMERA_CONTROL_WIDGET_H_
#define APPS_RECORDER_OPENZED_CAMERA_CONTROL_WIDGET_H_

#include <QDockWidget>
#include <QHBoxLayout>

#include "zedCamera.h"

namespace nfs {
namespace apps {

class OpenzedCameraControlWidget : public QDockWidget {
  Q_OBJECT
 public:
  OpenzedCameraControlWidget(QWidget* parent = nullptr);
 static const QString kPathToCamera;
 static const QString kPathToCameraConfig;
static const int  kFps ;
static const vision::zedCamera::IMAGE kImageType ; 
static const vision::zedCamera::RESOLUTION kResolution;
 public slots:
  void SetResolution(const int combo_box_current_idx);
  void SetFps(const int combo_box_current_idx);
  void SetCameraPath(const QString camera_path);
  void SetCameraConfigPath(const QString camera_config_path);
 void SetImageType( const int combo_box_current_idx);
 void UpdateWithCameraConfigPath();
void UpdateWithCameraPath();
 signals:
  void ResolutionChanged(const vision::zedCamera::RESOLUTION res);
  void FpsChanged(const int fps);
  void CameraPathChanged(const QString camera_path);
  void CameraConfigChanged(const QString camera_config_path);
  void ImageTypeChanged(const vision::zedCamera::IMAGE image_type);
private:
QString partial_camera_path_;
QString partial_camera_config_path_;
};

}  // namespace apps
}  // namespace nfs

#endif  // APPS_RECORDER_OPENZED_CAMERA_CONTROL_WIDGET_H_
