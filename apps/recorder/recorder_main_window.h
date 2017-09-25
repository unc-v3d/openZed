#ifndef APPS_RECORDER_RECORDER_MAIN_WINDOW_H_
#define APPS_RECORDER_RECORDER_MAIN_WINDOW_H_

#include <memory>

#include <QLabel>
#include <QMainWindow>
#include <QTimer>
#include <QToolBar>

#include "image_buffer.h"
#include "openzed_camera.h"
#include "openzed_camera_control_widget.h"

namespace Ui {
class MainWindow;
}

namespace nfs {
namespace apps {

class RecorderMainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit RecorderMainWindow(QWidget* parent = 0);
  ~RecorderMainWindow();
 public slots:
  void Play();
  void Record();
  void DisplayFrame();
  void FinishedRecordingStartPlaying();
  void StopRecording();
 signals:
  void RecordingStopRequested();

 private:
  void CreateToolbar();
  QString GetSaveDirPathFromDialog();

  Ui::MainWindow* ui_;
  std::unique_ptr<QToolBar> toolbar_;
  std::unique_ptr<OpenzedCamera> camera_;
  QLabel* image_display_;
  QTimer* timer_;
  std::unique_ptr<ImageBuffer> rgb_buffer_;
  bool finished_recording_;
};

}  // namespace apps
}  // namespace nfs

#endif  // APPS_RECORDER_RECORDER_MAIN_WINDOW_H_
