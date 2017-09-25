#include "recorder_main_window.h"

#include <QAction>
#include <QComboBox>
#include <QDebug>
#include <QDir>
#include <QImage>
#include <QInputDialog>
#include <QMessageBox>
#include <QThread>

#include "consumer.h"
#include "producer.h"
#include "ui_recorder_main_window.h"

#include "openzed_camera_control_widget.h"

namespace nfs {
namespace apps {

RecorderMainWindow::RecorderMainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::MainWindow) {
  ui_->setupUi(this);
  this->setWindowTitle("Openzed Recorder");
  image_display_ = new QLabel(this);
  image_display_->setScaledContents(true);
  this->setCentralWidget(image_display_);

  CreateToolbar();

  OpenzedCameraControlWidget* camera_control =
      new OpenzedCameraControlWidget(this);

  this->addDockWidget(Qt::TopDockWidgetArea, camera_control);
  camera_ = std::unique_ptr<OpenzedCamera>(new OpenzedCamera(
      OpenzedCameraControlWidget::kImageType,
      OpenzedCameraControlWidget::kResolution, OpenzedCameraControlWidget::kFps,
      OpenzedCameraControlWidget::kPathToCamera,
      OpenzedCameraControlWidget::kPathToCameraConfig));

  // Connect all the control signal and slots to camera_.
  bool success = QObject::connect(
      camera_control,
      SIGNAL(ResolutionChanged(const vision::zedCamera::RESOLUTION)),
      camera_.get(),
      SLOT(SetImageResolution(const vision::zedCamera::RESOLUTION)));
  assert(success);

  success = QObject::connect(camera_control, SIGNAL(FpsChanged(const int)),
                             camera_.get(), SLOT(SetFps(const int)));
  assert(success);

  success =
      QObject::connect(camera_control, SIGNAL(CameraPathChanged(const QString)),
                       camera_.get(), SLOT(SetCameraPath(const QString)));
  assert(success);

  success = QObject::connect(
      camera_control, SIGNAL(CameraConfigChanged(const QString)), camera_.get(),
      SLOT(SetCamerConfigPath(const QString)));
  assert(success);

  success = QObject::connect(
      camera_control, SIGNAL(ImageTypeChanged(const vision::zedCamera::IMAGE)),
      camera_.get(), SLOT(SetImageType(const vision::zedCamera::IMAGE)));
  assert(success);

  timer_ = new QTimer(this);
  timer_->stop();
  success =
      QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(DisplayFrame()));
  assert(success);

  // this->showMaximized();
  finished_recording_ = true;
  this->show();
}

void RecorderMainWindow::CreateToolbar() {
  toolbar_ = std::unique_ptr<QToolBar>(new QToolBar("Quick Access", this));

  const QIcon play_icon =
      QApplication::style()->standardIcon(QStyle::SP_MediaPlay);
  QAction* play_action = new QAction(play_icon, tr("Play"), this);
  play_action->setStatusTip(tr("Play"));
  connect(play_action, &QAction::triggered, this, &RecorderMainWindow::Play);
  toolbar_->addAction(play_action);

  const QIcon record_icon =
      QApplication::style()->standardIcon(QStyle::SP_DialogNoButton);
  QAction* record_action = new QAction(record_icon, tr("Record"), this);
  record_action->setStatusTip(tr("Record"));
  connect(record_action, &QAction::triggered, this,
          &RecorderMainWindow::Record);
  toolbar_->addAction(record_action);

  const QIcon stop_record_icon =
      QApplication::style()->standardIcon(QStyle::SP_MediaStop);
  QAction* stop_record_action =
      new QAction(stop_record_icon, tr("Stop recording"), this);
  stop_record_action->setStatusTip(tr("Stop recording"));
  connect(stop_record_action, &QAction::triggered, this,
          &RecorderMainWindow::StopRecording);
  toolbar_->addAction(stop_record_action);

  this->addToolBar(toolbar_.get());
}

RecorderMainWindow::~RecorderMainWindow() { delete ui_; }

void RecorderMainWindow::Play() {
  if (camera_.get() == nullptr) {
    camera_ = std::unique_ptr<OpenzedCamera>(
        new OpenzedCamera(OpenzedCameraControlWidget::kImageType,
                          OpenzedCameraControlWidget::kResolution,
                          OpenzedCameraControlWidget::kFps,
                          OpenzedCameraControlWidget::kPathToCamera,
                          OpenzedCameraControlWidget::kPathToCameraConfig));
  }

  camera_->Initialize();

  timer_->start();
}

void RecorderMainWindow::DisplayFrame() {
  //  Get the frame from the camera and display it.
  const cv::Mat frame_bgr = camera_->GetFrame();
  // qDebug() << "Grabbed frame of size : " << frame_bgr.cols << " "  <<
  // frame_bgr.cols;
  cv::Mat frame_rgb;
  cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);
  QImage qimage = QImage(frame_rgb.data, frame_rgb.cols, frame_rgb.rows,
                         frame_rgb.step, QImage::Format_RGB888)
                      .copy();
  image_display_->setPixmap(QPixmap::fromImage(qimage));
}

void RecorderMainWindow::Record() {
  if (finished_recording_) {
    const QString save_dir_path = GetSaveDirPathFromDialog();

    timer_->stop();
    rgb_buffer_.reset(new ImageBuffer());
    ProduceRGBImage* producer_rgb =
        new ProduceRGBImage(std::move(camera_), rgb_buffer_.get());

    // Create thread for producing frames.
    QThread* producer_thread = new QThread;

    producer_rgb->moveToThread(producer_thread);
    bool success = false;
    success = QObject::connect(producer_thread, SIGNAL(started()), producer_rgb,
                               SLOT(Produce()));
    assert(success);

    success = QObject::connect(producer_rgb, SIGNAL(FinishedProducing()),
                               producer_thread, SLOT(quit()));
    assert(success);

    success = QObject::connect(producer_rgb, SIGNAL(FinishedProducing()),
                               producer_rgb, SLOT(deleteLater()));
    assert(success);

    success = QObject::connect(producer_thread, SIGNAL(finished()),
                               producer_thread, SLOT(deleteLater()));
    assert(success);

    // Hook up producer with stop signal.
    success = QObject::connect(this, SIGNAL(RecordingStopRequested()),
                               producer_rgb, SLOT(StopProducing()),Qt::DirectConnection);
    assert(success);

    // Create thread for consuming.
    Consumer* color_frame_consumer =
        new Consumer(save_dir_path, "png", rgb_buffer_.get());

    QThread* consumer_thread = new QThread;

    color_frame_consumer->moveToThread(consumer_thread);

    success = QObject::connect(consumer_thread, SIGNAL(started()),
                               color_frame_consumer, SLOT(StartConsuming()));
    assert(success);

    success = QObject::connect(color_frame_consumer, SIGNAL(Finished()),
                               consumer_thread, SLOT(quit()));
    assert(success);

    success = QObject::connect(color_frame_consumer, SIGNAL(Finished()),
                               color_frame_consumer, SLOT(deleteLater()));
    assert(success);

    success = QObject::connect(color_frame_consumer, SIGNAL(Finished()), this,
                               SLOT(FinishedRecordingStartPlaying()));
    assert(success);

    success = QObject::connect(producer_thread, SIGNAL(finished()),
                               producer_thread, SLOT(deleteLater()));
    assert(success);


// connect producer finish to stop consumer. But remember to flush the buffer first.
    success = QObject::connect(producer_rgb, SIGNAL(FinishedProducing()),
                               color_frame_consumer, SLOT(StopCapture()),Qt::DirectConnection);
    assert(success);

    producer_thread->start();

    consumer_thread->start();
  }
}

void RecorderMainWindow::StopRecording() {
 emit RecordingStopRequested(); }

QString RecorderMainWindow::GetSaveDirPathFromDialog() {
  bool ok;
  const QString path_to_save_dir =
      QInputDialog::getText(this, tr("Input save directory"), tr("Save dir:"),
                            QLineEdit::Normal, "Path to save dir", &ok);

  if (ok && QDir(path_to_save_dir).exists()) {
    return path_to_save_dir;
  } else {
    return GetSaveDirPathFromDialog();
  }
}

void RecorderMainWindow::FinishedRecordingStartPlaying() {
  finished_recording_ = true;
  // send pop up saying capture was success.
  QMessageBox msgBox;
  msgBox.setText("Capture successful.");
  msgBox.exec();
  Play();
}

}  // namespace apps
}  // namespace nfs
