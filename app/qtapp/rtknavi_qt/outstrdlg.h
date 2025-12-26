//---------------------------------------------------------------------------
#ifndef outstrdlgH
#define outstrdlgH
//---------------------------------------------------------------------------
#include <QDialog>

namespace Ui {
class OutputStrDialog;
}

class KeyDialog;
class SerialOptDialog;
class TcpOptDialog;

//---------------------------------------------------------------------------
class OutputStrDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OutputStrDialog(QWidget* parent);

    void setStreamEnabled(int stream, int enabled);
    int getStreamEnabled(int stream);

    void setStreamType(int stream, int type);
    int getStreamType(int stream);

    void setStreamFormat(int stream, int format);
    int getStreamFormat(int stream);

    void setPath(int stream, int type, const QString &);
    QString getPath(int stream, int type);

    void setTimeTagEnabled(bool);
    bool getTimeTagEnabled();

    void setSwapInterval(const QString &);
    QString getSwapInterval();

    void setHistory(int i, const QString &history);
    const QString getHistory(int i);

protected:
    KeyDialog *keyDialog;
    SerialOptDialog *serialOptDialog;
    TcpOptDialog *tcpOptDialog;

public slots:
    void showStream1Options();
    void showStream2Options();
    void showStream3Options();
    void selectFile1();
    void selectFile2();
    void selectFile3();
    void showKeyDialog();
    void updateEnable();

private:
    QString getFilePath(const QString &path);
    QString setFilePath(const QString &path);
    void showSerialOptions(int index, int opt);
    void showTcpOptions(int index, int opt);

    Ui::OutputStrDialog *ui;

    QString paths[3][4];
    QString history[10];

};
//---------------------------------------------------------------------------
#endif
