#include "widget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // 设置中文编码
//    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));

    Widget w;
    w.show();
    return a.exec();
}
