#include <QApplication>
#include "recorder_main_window.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  nfs::apps::RecorderMainWindow window;
  window.show();

  return app.exec();
}
