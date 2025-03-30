#include "svrmain.h"
#include <QApplication>
#include <QTranslator>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QTranslator translator;

    bool ok = translator.load(QLocale(), "rtklib", "_", ":/i18n/");
    if (!ok)
        qDebug("Failed to open translation file.");

    a.installTranslator(&translator);

    MainForm w;

    w.show();

    return a.exec();
}
