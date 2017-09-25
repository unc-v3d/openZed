#include "openzed_camera_control_widget.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QDebug>

namespace nfs {
namespace apps {

 const int  OpenzedCameraControlWidget::kFps = 30;
 const vision::zedCamera::IMAGE OpenzedCameraControlWidget::kImageType = vision::zedCamera::IMAGE::RAW;    
 const vision::zedCamera::RESOLUTION OpenzedCameraControlWidget::kResolution = vision::zedCamera::RESOLUTION::WVGA;
const QString OpenzedCameraControlWidget::kPathToCamera = "0";
  const QString OpenzedCameraControlWidget::kPathToCameraConfig = "./a.conf";


OpenzedCameraControlWidget::OpenzedCameraControlWidget(QWidget* parent)
    : QDockWidget("Camera control panel", parent) {
  this->setFeatures(QDockWidget::NoDockWidgetFeatures);

  partial_camera_path_ =kPathToCamera;
  partial_camera_config_path_ = kPathToCameraConfig;


  QHBoxLayout* horizontal_layout = new QHBoxLayout();
 
QComboBox* image_type = new QComboBox(this);

  image_type->addItem("Raw");
  image_type->addItem("Undistorted");
  image_type->addItem("Rectified");
  image_type->setEditable(false);
  horizontal_layout->addWidget(image_type);


 QComboBox* resolution_options = new QComboBox(this);

  resolution_options->addItem("VGA");
  resolution_options->addItem("HD");
  resolution_options->addItem("Full HD");
  resolution_options->addItem("2K");
  resolution_options->setEditable(false);
  horizontal_layout->addWidget(resolution_options);

  QComboBox* fps_options = new QComboBox(this);

  fps_options->addItem("15");
  fps_options->addItem("30");
  fps_options->addItem("60");
  fps_options->addItem("100");
  fps_options->setEditable(false);
  horizontal_layout->addWidget(fps_options);
 fps_options->setCurrentIndex(1);

  QLineEdit* path_to_camera_line_edit = new QLineEdit(kPathToCamera);
  horizontal_layout->addWidget(path_to_camera_line_edit);

  QLineEdit* path_to_camera_config_line_edit = new QLineEdit(kPathToCameraConfig);
  horizontal_layout->addWidget(path_to_camera_config_line_edit);

  QWidget* multi_widget = new QWidget();
  multi_widget->setLayout(horizontal_layout);
  this->setWidget(multi_widget);

  // Connect all the signals and slots.
 bool success = false;
success = QObject::connect(image_type, SIGNAL(currentIndexChanged(int)),
                          this, SLOT(SetImageType(const int)));
assert(success);
 success = QObject::connect(resolution_options,  SIGNAL(currentIndexChanged(int)),
                          this, SLOT(SetResolution(const int)));
assert(success);

  success = QObject::connect(fps_options, SIGNAL(currentIndexChanged(int)), this,
                          SLOT(SetFps(const int)));
assert(success);

 success =  QObject::connect(path_to_camera_line_edit, SIGNAL(textChanged(const QString&)), this,
                         SLOT(SetCameraPath(const QString)));
assert(success);
  success = QObject::connect(path_to_camera_config_line_edit,
                          SIGNAL(textChanged(const QString&)), this,
                           SLOT(SetCameraConfigPath(const QString)));
assert(success);

 success =  QObject::connect(path_to_camera_line_edit, SIGNAL(editingFinished()), this,  SLOT(UpdateWithCameraPath()));
assert(success);
  success = QObject::connect(path_to_camera_config_line_edit,
                          SIGNAL(editingFinished()), this,
                           SLOT(UpdateWithCameraConfigPath()));
assert(success);




}

void OpenzedCameraControlWidget::SetResolution(
    const int combo_box_current_idx) {
qDebug()<<"combo_box_current_idx is "<< combo_box_current_idx ;
  vision::zedCamera::RESOLUTION res;
  switch (combo_box_current_idx) {
    case 0: {
      res = vision::zedCamera::RESOLUTION::WVGA;
      break;
    }
    case 1: {
      res = vision::zedCamera::RESOLUTION::HD;
      break;
    }
    case 2: {
      res = vision::zedCamera::RESOLUTION::FHD;
      break;
    }
    case 3: {
      res = vision::zedCamera::RESOLUTION::RES2K;
      break;
    }
    default: {
      res = kResolution;
      break;
    }
  }

  emit ResolutionChanged(res);
}

void OpenzedCameraControlWidget::SetFps(const int combo_box_current_idx) {
qDebug()<<"combo_box_current_idx is "<< combo_box_current_idx ;
  int fps;
  switch (combo_box_current_idx) {
    case 0: {
      fps = 15;
      break;
    }
    case 1: {
      fps = 30;
      break;
    }
    case 2: {
      fps = 60;
      break;
    }
    case 3: {
      fps = 100;
      break;
    }
    default: {
      fps = kFps;
      break;
    }
  }
  emit FpsChanged(fps);
}

void OpenzedCameraControlWidget::SetCameraPath(const QString camera_path) {
partial_camera_path_  =camera_path;
}
void OpenzedCameraControlWidget::SetCameraConfigPath(
    const QString camera_config_path) {
qDebug()<<"camera_config_path is "<< camera_config_path ;
partial_camera_config_path_  =camera_config_path;

}

void OpenzedCameraControlWidget::UpdateWithCameraPath( ) {
qDebug()<<"camera_path is "<< partial_camera_path_ ;

  emit CameraPathChanged(partial_camera_path_);
}
void OpenzedCameraControlWidget::UpdateWithCameraConfigPath( ) {
qDebug()<<"camera_config_path is "<< partial_camera_config_path_ ;

emit CameraConfigChanged(partial_camera_config_path_);
}


 void OpenzedCameraControlWidget::SetImageType( const int combo_box_current_idx){
qDebug()<<"combo_box_current_idx is "<< combo_box_current_idx ;
 vision::zedCamera::IMAGE image_type;
  switch (combo_box_current_idx) {
    case 0: {
      image_type = vision::zedCamera::IMAGE::RAW;
      break;
    }
    case 1: {
      image_type = vision::zedCamera::IMAGE::UNDISTORTED;
      break;
    }
    case 2: {
      image_type = vision::zedCamera::IMAGE::RECTIFIED;
      break;
    }
    default: {
      image_type = kImageType;
      break;
    }
  }
  emit ImageTypeChanged(image_type);


}

}  // namespace apps
}  // namespace nfs
