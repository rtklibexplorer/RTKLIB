//---------------------------------------------------------------------------
#ifndef launchmainH
#define launchmainH
//---------------------------------------------------------------------------
#include <QDialog>
#include <QSystemTrayIcon>
#include <QMenu>

namespace Ui {
class MainForm;
}

class QCloseEvent;
class LaunchOptDialog;

//---------------------------------------------------------------------------
class MainForm : public QDialog
{
    Q_OBJECT

public:
    explicit MainForm(QWidget *parent = 0);

protected:
    void showEvent(QShowEvent *);
    void closeEvent(QCloseEvent *);

    int option, minimize;

protected slots:
    void launchRTKPlot();
    void launchRTKConv();
    void launchStrSvr();
    void launchRTKPost();
    void launchSrcTblBrows();
    void launchRTKNavi();
    void launchRTKGet();
    void moveToTray();
    void showOptions();
    void restoreFromTaskTray(QSystemTrayIcon::ActivationReason);
    void expandWindow();

private:
    int execCommand(const QString &cmd, const QStringList &opt);
    void updatePanel();

    QString iniFile;
    QSystemTrayIcon trayIcon;
    QMenu *trayMenu;
    LaunchOptDialog *launchOptDlg;
    bool tray;
    Ui::MainForm *ui;
};
//---------------------------------------------------------------------------
#endif
