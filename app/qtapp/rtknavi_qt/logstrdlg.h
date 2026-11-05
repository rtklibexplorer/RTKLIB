//---------------------------------------------------------------------------
#ifndef logstrdlgH
#define logstrdlgH
//---------------------------------------------------------------------------
#include <QDialog>

#include "rtklib.h"

namespace Ui {
class LogStrDialog;
}

class KeyDialog;
class SerialOptDialog;
class TcpOptDialog;

//---------------------------------------------------------------------------
class LogStrDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LogStrDialog(QWidget* parent);

    void setStreamEnabled(int stream, int enabled);
    int getStreamEnabled(int stream);

    void setStreamType(int stream, int type);
    int getStreamType(int stream);

    void setPath(int stream, int type, const QString &);
    QString getPath(int stream, int type);

    void setLogTimeTagEnabled(bool);
    bool getLogTimeTagEnabled();


    void setSwapInterval(const QString &);
    QString getSwapInterval();

    void setHistory(int i, const QString &history);
    const QString getHistory(int i);

protected:
    KeyDialog *keyDialog;
    SerialOptDialog *serialOptDialog;
    TcpOptDialog *tcpOptDialog;

    QString paths[RTKSVRNIN][4];
    QString history[10];

public slots:
    void showStreamOptions1();
    void showStreamOptions2();
    void showStreamOptions3();
    void showStreamOptions4();
    void selectFile1();
    void selectFile2();
    void selectFile3();
    void selectFile4();
    void showKeyDialog();
    void updateEnable();

private:
    QString getFilePath(const QString &path);
    QString setFilePath(const QString &path);

    void showSerialOptions(int index, int opt);
    void showTcpOptions(int index, int opt);

    Ui::LogStrDialog *ui;
};
//---------------------------------------------------------------------------
#endif
