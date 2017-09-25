#include "producer.h"

#include <QThread>
#include<QDebug>

namespace nfs {
namespace apps {

ProduceRGBImage::ProduceRGBImage(std::unique_ptr<OpenzedCamera> camera, ImageBuffer *rgb_buffer, QObject *parent)
    : camera_(std::move(camera)), rgb_buffer_(rgb_buffer), QObject(parent) {
  stop_producing_ = false;
}

void ProduceRGBImage::StopProducing() { stop_producing_ = true;
 }

void ProduceRGBImage::Produce() {
  while (!stop_producing_ && camera_.get() != nullptr) {
    cv::Mat rgb_image = camera_->GetFrame();
    if (camera_->IsImageGood()) {
  rgb_buffer_->AddImage(rgb_image);
      QThread::usleep(1);
    } else {
      QThread::usleep(500);
    }

  } 
qDebug() << "finished producing."; 
    emit FinishedProducing();
  
}

}  // namespace apps
}  // namespace nfs
