#include "image_buffer.h"

namespace nfs {
namespace apps {

ImageBuffer::ImageBuffer()
    : free_frames_(new QSemaphore(NFS_APPS_IMAGE_BUFFER_SIZE)),
      used_frames_(new QSemaphore(0)) {
  buffer_.resize(NFS_APPS_IMAGE_BUFFER_SIZE);
}

void ImageBuffer::AddImage(const cv::Mat& image) {
  free_frames_->acquire();
  image.copyTo(buffer_[produce_idx_ % buffer_.size()]);
  produce_idx_++;
  used_frames_->release();
}

cv::Mat ImageBuffer::GetImage() {
  cv::Mat ret_image;

  used_frames_->acquire();
  buffer_[consume_idx_ % buffer_.size()].copyTo(ret_image);
  consume_idx_++;
  free_frames_->release();

  return ret_image;
}

cv::Mat ImageBuffer::FlushGetImage(bool* valid) {
  cv::Mat ret_image;
  if(consume_idx_ == produce_idx_){
*valid = false;
return ret_image;
}

*valid = true;
return GetImage();
}

}  // namespace apps
}  // namespace nfs
