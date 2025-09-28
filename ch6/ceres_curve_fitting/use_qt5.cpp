#include <QApplication>
#include <QLabel>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QLabel l("Hello Qt5");
    l.resize(200, 100);
    l.show();
    return a.exec();
}