#ifndef APPS_RECORDER_PRODUCER_H_
#define APPS_RECORDER_PRODUCER_H_

#include <QObject>
#include <QString>

#include <memory>

#include "image_buffer.h"
#include "openzed_camera.h"

namespace nfs {
namespace apps {

class ProduceRGBImage : public QObject {
Q_OBJECT
 public:
  explicit ProduceRGBImage(std::unique_ptr<OpenzedCamera> camera, ImageBuffer *rgb_buffer, QObject *parent = NULL);

 public slots:
  void StopProducing();
  void Produce();

 signals:
  void FinishedProducing();

 private:
  std::unique_ptr<OpenzedCamera> camera_;
  ImageBuffer *rgb_buffer_;
  bool stop_producing_;
};

}  // namespace apps
}  // namespace nfs

#endif  // APPS_RECORDER_PRODUCER_H_
