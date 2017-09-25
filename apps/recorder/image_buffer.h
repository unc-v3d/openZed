#ifndef APPS_RECORDER_IMAGE_BUFFER_H_
#define APPS_RECORDER_IMAGE_BUFFER_H_

#include <vector>
#include<memory>

#include <QSemaphore>

#include <opencv2/opencv.hpp>

#define NFS_APPS_IMAGE_BUFFER_SIZE 1000

namespace nfs {
namespace apps {

class ImageBuffer {
 public:
  ImageBuffer();
  void AddImage(const cv::Mat& image);
  cv::Mat GetImage();
cv::Mat FlushGetImage(bool* valid);
 private:
  std::vector<cv::Mat> buffer_;
  std::unique_ptr<QSemaphore> free_frames_;
  std::unique_ptr<QSemaphore> used_frames_;
  int consume_idx_ = 0;
  int produce_idx_ = 0;
};

}  // namespace apps
}  // namespace nfs

#endif  // APPS_RECORDER_IMAGE_BUFFER_H_
