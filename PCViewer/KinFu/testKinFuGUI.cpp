#include <QApplication>
#include <KinFu/KinFuWindow.h>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);

  KinFuWindow w;
  w.show ();

  return a.exec ();
}
