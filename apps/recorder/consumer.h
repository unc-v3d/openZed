#ifndef APPS_RECORDER_CONSUMER_H_
#define APPS_RECORDER_CONSUMER_H_

#include <QObject>

#include "image_buffer.h"

namespace nfs {
namespace apps {

class Consumer : public QObject {
Q_OBJECT
 public:
  Consumer(const QString& save_path, const QString image_extension,
           ImageBuffer* image_buffer, QObject* parent = nullptr);

 public slots:
  void StopCapture();
  void StartConsuming();

 signals:
 void Finished();

 private:
  ImageBuffer* image_buffer_;
  const QString save_path_;
  const QString image_extension_;
  bool done_capture_;
  int image_idx_ = 0;
};

}  // namespace apps
}  // namespace nfs

#endif  // APPS_RECORDER_CONSUMER_H_
