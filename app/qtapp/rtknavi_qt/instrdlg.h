//---------------------------------------------------------------------------
#ifndef instrdlgH
#define instrdlgH
//---------------------------------------------------------------------------

#include <QDialog>

#include "rtklib.h"

namespace Ui {
class InputStrDialog;
}

class CmdOptDialog;
class RcvOptDialog;
class RefDialog;
class SerialOptDialog;
class TcpOptDialog;
class FtpOptDialog;

//---------------------------------------------------------------------------
class InputStrDialog : public QDialog
{
    Q_OBJECT

public:
    explicit InputStrDialog(QWidget* parent);

    void setStreamEnabled(int stream, int enabled);
    int getStreamEnabled(int stream);

    void setStreamType(int stream, int type);
    int getStreamType(int stream);

    void setStreamFormat(int stream, int format);
    int getStreamFormat(int stream);

    void setReceiverOptions(int stream, const QString &options);
    QString getReceiverOptions(int stream);

    void setFilePath(int stream, const QString &path);
    const QString &getFilePath(int stream);

    void setResetCommand(const QString &command);
    QString getResetCommand();

    void setNMeaPosition(int i, double value);
    double getNMeaPosition(int i);

    void setNmeaRequestType(int);
    int getNmeaRequestType();

    void setMaxBaseLine(double baseline);
    double getMaxBaseLine();

    void setHistory(int i, const QString &history);
    const QString &getHistory(int i);

    void setTimeTagEnabled(bool);
    bool getTimeTagEnabled();

    void setTimeTag64bit(bool);
    bool getTimeTag64bit();

    void setTimeStart(double);
    double getTimeStart();

    void setTimeSpeed(const QString &speed);
    QString getTimeSpeed();

    void setStationPositionFile(const QString &file);
    QString getStationPositionFile();

    void setCommands(int stream, int type, const QString &);
    QString getCommands(int stream, int type);

    void setCommandsEnabled(int stream, int type, bool ena);
    bool getCommandsEnabled(int stream, int type);

    void setCommandsTcp(int stream, int type, const QString &);
    QString getCommandsTcp(int stream, int type);

    void setCommandsTcpEnabled(int stream, int type, bool ena);
    bool getCommandsTcpEnabled(int stream, int type);

    void setPath(int stream, int type, const QString &);
    QString getPath(int stream, int type);

public slots:
    void  showStreamOptions1();
    void  showStreamOptions2();
    void  showStreamOptions3();
    void  showStreamOptions4();
    void  showCommandDialog1();
    void  showCommandDialog2();
    void  showCommandDialog3();
    void  showCommandDialog4();
    void  selectFile1();
    void  selectFile2();
    void  selectFile3();
    void  selectFile4();
    void  showReceiverOptions1();
    void  showReceiverOptions2();
    void  showReceiverOptions3();
    void  showReceiverOptions4();
    void  selectPosition();
    void  updateEnable();

private:
    QString extractFilePath(const QString &path);
    QString makePath(const QString &path);
    void showSerialOptionsDialog(int index, int opt);
    void showTcpOptionsDialog(int index, int opt);
    void showFtpOptionsDialog(int index, int opt);
    void showCommandDialog(int streamNo);
    void showReceiverOptionDialog(int streamNo);

    CmdOptDialog *cmdOptDialog;
    RcvOptDialog *rcvOptDialog;
    RefDialog *refDialog;
    SerialOptDialog *serialOptDialog;
    TcpOptDialog *tcpOptDialog;
    FtpOptDialog *ftpOptDialog;

    int commandEnable[RTKSVRNIN][3], commandEnableTcp[RTKSVRNIN][3];
    QString commands[RTKSVRNIN][3], commandsTcp[RTKSVRNIN][3];
    QString paths[RTKSVRNIN][4];


    QString receiverOptions[RTKSVRNIN];
    QString history[10];
    int noFormats;
    QString stationPositionFile;

    Ui::InputStrDialog *ui;
};
#endif
