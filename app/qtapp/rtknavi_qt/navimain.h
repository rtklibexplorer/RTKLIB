//---------------------------------------------------------------------------
#ifndef navimainH
#define navimainH
//---------------------------------------------------------------------------
#include <QDialog>
#include <QColor>
#include <QSystemTrayIcon>
#include <QTimer>
#include <QMenu>

#include "rtklib.h"

#define MAXSCALE	18
#define MAXMAPPNT	10
#define MAXSTR      1024                /* max length of a string */

namespace Ui {
class MainForm;
}

class AboutDialog;
class OptDialog;
class InputStrDialog;
class OutputStrDialog;
class LogStrDialog;
class MonitorDialog;
class MarkDialog;
class QLabel;
class LabelStretcher;

//---------------------------------------------------------------------------
class MainWindow : public QDialog
{
        Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void updateServer();

    void startServer();
    void stopServer();
    void showRtkPlot();
    void showOptionsDialog();
    void btnMarkClicked();
    void btnSaveClicked();
    void exit();

    void changeTimeSystem();
    void changeSolutionType();
    void showInputStreamDialog();
    void showOutputStreamDialog();
    void showLogStreamDialog();
    void showMonitorDialog();
    void showAboutDialog();
    void minimizeToTray();

    void expandFromTray();
    void maximizeFromTray(QSystemTrayIcon::ActivationReason);

    void changeSolutionIndex();
    void changeFrequencyType1();
    void changeFrequencyType2();
    void changeFrequencyType3();
    void changeFrequencyType4();
    void changePlotType1();
    void changePlotType2();
    void changePlotType3();
    void changePlotType4();
    void changePanelMode();

    void expandPlot1();
    void shrinkPlot1();
    void expandPlot2();
    void shrinkPlot2();
    void expandPlot3();
    void shrinkPlot3();
    void expandPlot4();
    void shrinkPlot4();

protected:
    void showEvent(QShowEvent*);
    void closeEvent(QCloseEvent *);

    rtksvr_t *rtksvr;                       // rtk server struct
    stream_t monistr;                       // monitor stream

private:
    InputStrDialog *inputStrDialog;
    OutputStrDialog *outputStrDialog;
    LogStrDialog *logStrDialog;
    MonitorDialog *monitor;
    QSystemTrayIcon *systemTray;
    QMenu *trayMenu;
    QAction *menuStartAction, *menuStopAction, *menuExitAction;
    LabelStretcher *stretch;

    Ui::MainForm *ui;

    void serverStart();
    void serverStop();

    void updateLog (int stat, gtime_t time, double *rr, float *qr,
                     double *rb, int ns, double age, double ratio);
    void updatePanels();
    void updateTimeSystem();
    void updateSolutionType();
    void updateFont();
    void updateTime();
    void updatePosition();
    void updateStream();
    void updatePlot();
    void updateEnable();
    int confirmOverwrite(const QString &path);

    void drawSolutionPlot(QLabel *plot, int type, int freq);
    void drawSnr(QPainter &c, int w, int h, int x0, int y0, int index, int freq);
    void drawSatellites(QPainter &c, int w, int h, int x0, int y0, int index, int freq);
    void drawBaseline(QPainter &c, int id, int w, int h);
    void drawTrack(QPainter &c, int id, QPaintDevice * plot);
    void drawSky(QPainter &c, int w, int h, int x0, int y0);
    void drawText(QPainter &c, int x, int y, const QString &s, const QColor &color, int ha, int va);
    void drawArrow(QPainter &c, int x, int y, int size, int ang, const QColor &color);
    void openMonitorPort(int port);
    void initSolutionBuffer();
    void saveLogs();
    void loadNavigation(nav_t *nav);
    void saveNavigation(nav_t *nav);
    void loadOptions();
    void saveOptions();
    void setTrayIcon(int index);
    int execCommand(const QString &cmd, const QStringList &opt);
    void changeFrequencyType(int i);
    void changePlotType(int i);
    void showFrequenciesDialog();
    void showMaskDialog();
    void showKeyDialog();
    QColor snrColor(int snr);

public:
    OptDialog *optDialog;
    MarkDialog *markDialog;
    QTimer updateTimer;
    QString iniFile;

    int timerCycle,timerInactive;
    int panelMode;

    int inputTimeTag, inputTimeTag64bit;
    int outputTimeTag, outputAppend, logTimeTag;
    int streamType[MAXSTRRTK], streamEnabled[MAXSTRRTK], inputFormat[MAXSTRRTK];
    int commandEnabled[3][3], commandEnableTcp[3][3];
    int timeSystem, solutionType;
    int plotType[4], frequencyType[4];
    int trackType[4];
    int trackScale[4];
    int baselineMode[4];
    int monitorPortOpen;

    int solutionsCurrent, solutionsStart, solutionsEnd, numSatellites[2], solutionCurrentStatus;
    int satellites[2][MAXSAT], satellitesSNR[2][MAXSAT][NFREQ], validSatellites[2][MAXSAT];
    double satellitesAzimuth[2][MAXSAT], satellitesElevation[2][MAXSAT];
    gtime_t *timeStamps;
    int *solutionStatus, *numValidSatellites;
    double *solutionRover, *solutionReference, *solutionQr, *velocityRover, *ages, *ratioAR;
    double trackOrigin[3], maxBaseLine;

    QString paths[MAXSTRRTK][4], commands[3][3], commandsTcp[3][3];
    QString inputTimeSpeed;
    double inputTimeStart;
    QString receiverOptions[3];
    QString outputSwapInterval, logSwapInterval, resetCommand;

    int nmeaRequestType;
    double nmeaPosition[3];

    QString history[10];

    QString pointName[MAXMAPPNT];
    double pointPosition[MAXMAPPNT][3];
    int nMapPoint;

    QString markerName, markerComment;
};
//---------------------------------------------------------------------------
#endif
