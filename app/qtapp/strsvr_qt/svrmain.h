//---------------------------------------------------------------------------
#ifndef svrmainH
#define svrmainH
//---------------------------------------------------------------------------
#include <QDialog>
#include <QTimer>
#include <QSystemTrayIcon>

#include "rtklib.h"
#include "tcpoptdlg.h"

namespace Ui {
class MainForm;
}

class QCloseEvent;
class SvrOptDialog;
class SerialOptDialog;
class TcpOptDialog;
class FileOptDialog;
class StrMonDialog;

#define MAXSTR        7    // number of streams

//---------------------------------------------------------------------------
class MainForm : public QDialog
{
    Q_OBJECT

public slots:
    void showInputOptions();
    void showOptionsDialog();
    void updateServerStat();
    void showStreamOptionsDialog();
    void showStreamCommandDialog();
    void showAboutDialog();
    void showMonitorDialog();
    void updateStreamMonitor();
    void minimizeToTaskTray();
    void restoreFromTaskTray(QSystemTrayIcon::ActivationReason);
    void expandFromTaskTray();
    void showConvertDialog();
    void showLogStreamDialog();
    void startServer();
    void stopServer();

protected:
    void closeEvent(QCloseEvent*);
    void showEvent(QShowEvent*);

private:
    QString iniFile;
    QString paths[MAXSTR][7], commands[MAXSTR][3], commandsTcp[MAXSTR][3];
    QString tcpHistory[MAXHIST], tcpMountpointHistory[MAXHIST];
    QString conversionMessage[MAXSTR - 1], conversionOptions[MAXSTR - 1];
    QString pathLog[MAXSTR];
    int conversionEnabled[MAXSTR - 1], conversionInputFormat[MAXSTR - 1], conversionOutputFormat[MAXSTR - 1];
    int commandsEnabled[MAXSTR][3], commandsEnabledTcp[MAXSTR][3], pathEnabled[MAXSTR];
    gtime_t startTime, endTime;
    QSystemTrayIcon *trayIcon;
    SvrOptDialog *svrOptDialog;
    TcpOptDialog *tcpOptDialog;
    SerialOptDialog *serialOptDialog;
    FileOptDialog *fileOptDialog;
    StrMonDialog * strMonDialog;
    QTimer serverStatusTimer, streamMonitorTimer;

    void serialOptions(int index, int path);
    void tcpClientOptions(int index, int path);
    void tcpServerOptions(int index, int path);
    void ntripServerOptions(int index, int path);
    void ntripClientOptions(int index, int path);
    void ntripCasterOptions(int index, int path);
    void udpClientOptions(int index, int path);
    void udpServerOptions(int index, int path);
    void fileOptions(int index, int path);
    void showMessage(const QString &msg);
    void updateEnable();
    void setTrayIcon(int index);
    void loadOptions();
    void saveOptions();

    Ui::MainForm *ui;
public:
    explicit MainForm(QWidget *parent=0);
};
//---------------------------------------------------------------------------
#endif
