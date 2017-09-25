#include "consumer.h"

#include <QDebug>
#include <QDir>
#include<QThread>

#include <opencv2/opencv.hpp>

namespace nfs {
namespace apps {

Consumer::Consumer(const QString& save_path, const QString image_extension,
                   ImageBuffer* image_buffer, QObject* parent)
    : save_path_(save_path),
      image_extension_(image_extension),
      image_buffer_(image_buffer),
      QObject(parent) {done_capture_ = false;}

void Consumer::StartConsuming() {
  while (!done_capture_ ) {
    const cv::Mat color_image = image_buffer_->GetImage();
    const QString number = QString("%1").arg(image_idx_, 5, 10, QChar('0'));
    const QString file_full_path = QDir(save_path_).
                                           filePath(QString(number
                                            + "." + image_extension_));
    cv::imwrite(file_full_path.toStdString(), color_image);
    ++image_idx_;
QThread::usleep(1);
}

// Flush the buffer and then quit.
bool valid_image;
  cv::Mat color_image = image_buffer_->FlushGetImage(&valid_image);
while(valid_image){
 
    const QString number = QString("%1").arg(image_idx_, 5, 10, QChar('0'));
    const QString file_full_path = QDir(save_path_).
                                           filePath(QString(number
                                            + "." + image_extension_));
    cv::imwrite(file_full_path.toStdString(), color_image);
    ++image_idx_;
QThread::usleep(1);
  color_image = image_buffer_->FlushGetImage(&valid_image);

}

    qDebug() << "Saving finished.";
    emit Finished();
  
}

void Consumer::StopCapture() { done_capture_ = true; }

}  // namespace apps
}  // namespace nfs
