//---------------------------------------------------------------------------
// rtknavi : real-time positioning AP
//
//          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann (2016-2024)
//
// options : rtknavi [-t title][-i file][-auto][-tray]
//
//           -t title   window title
//           -i file    ini file path
//           -auto      auto start
//           -tray      start as task tray icon
//
// version : $Revision:$ $Date:$
// history : 2008/07/14  1.0 new
//           2010/07/18  1.1 rtklib 2.4.0
//           2010/08/16  1.2 fix bug on setting of satellite antenna model
//           2010/09/04  1.3 fix bug on setting of receiver antenna delta
//           2011/06/10  1.4 rtklib 2.4.1
//           2012/04/03  1.5 rtklib 2.4.2
//           2014/09/06  1.6 rtklib 2.4.3
//           2016/01/31  2.0 ported to Qt
//           2017/09/01  1.7 add option -auto and -tray
//           2020/11/30  1.8 add "Output Velocity" option
//                           support saving multiple sets of ephemeris
//                           fix bug on unable deselecting antenna PCV
//                           fix bug on unable saving TGD2 of ephemeris
//---------------------------------------------------------------------------
#include <stdio.h>
#include <math.h>
#include <clocale>

#include <QMessageBox>
#include <QMainWindow>
#include <QTimer>
#include <QFileInfo>
#include <QFont>
#include <QSettings>
#include <QProcess>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QShowEvent>
#include <QCloseEvent>
#include <QPainter>
#include <QDebug>
#include <QCommandLineParser>

#include "rtklib.h"
#include "instrdlg.h"
#include "outstrdlg.h"
#include "logstrdlg.h"
#include "mondlg.h"
#include "aboutdlg.h"
#include "markdlg.h"
#include "navi_post_opt.h"
#include "navimain.h"
#include "graph.h"
#include "helper.h"
#include "labelstretcher.h"

#include "ui_navimain.h"

MainWindow *mainForm;

//---------------------------------------------------------------------------

#define PRGNAME     "RtkNavi Qt"           // program name
#define TRACEFILE   "rtknavi_%Y%m%d%h%M.trace" // debug trace file
#define STATFILE    "rtknavi_%Y%m%d%h%M.stat"  // solution status file
#define MINSNR      10                  // minimum snr
#define MAXSNR      60                  // maximum snr
#define MIN_BASELINE_LEN    0.01                // minimum baseline length to show

#define KACYCLE     5000                // keep alive cycle (ms)
#define TIMEOUT     10000               // inactive timeout time (ms)
#define MAX_PORT_OFFSET  9              // max port number offset
#define MAXTRKSCALE 23                  // track scale
#define MAXPANELMODE 7                  // max panel mode

#define SQRT(x)     ((x)<0.0||(x)!=(x)?0.0:sqrt(x))

// receiver options table ---------------------------------------------------
static int strtype[MAXSTRRTK] = {                  /* stream types */
    STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE
};
static char strpath[MAXSTRRTK][MAXSTR] = { "" };      /* stream paths */
static int strfmt[MAXSTRRTK] = {                      /* stream formats */
    STRFMT_RTCM3, STRFMT_RTCM3, STRFMT_SP3, 0, 0, 0, SOLF_LLH, SOLF_NMEA, SOLF_NMEA
};

#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripcli,7:ftp,8:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,5:ntripsvr,9:ntripcas"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,4:ubx,5:swift,6:hemis,7:skytraq,8:javad,9:nvs,10:binex,11:rt17,12:sbf,14:unicore,15:rinex,16:sp3,17:clk"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea,4:stat"

// show message in message area ---------------------------------------------
extern "C" {
    extern int showmsg(const char *, ...)
    {
        return 0;
    }
extern void settime(gtime_t) {}
extern void settspan(gtime_t, gtime_t) {}
}

// constructor --------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QDialog(parent), ui(new Ui::MainForm)
{
    mainForm = this;
    ui->setupUi(this);

    setlocale(LC_NUMERIC, "C");

    for (int i = 0; i < MAXSTRRTK; i++) {
        streamEnabled[i] = streamType[i] = inputFormat[i] = 0;
    }
    for (int i = 0; i < 3; i++)
        commandEnabled[i][0] = commandEnabled[i][1] = commandEnabled[i][2] = 0;

    timeSystem = solutionType = 0;
    for (int i = 0; i < 4; i++) {
        plotType[i] = frequencyType[i] = baselineMode[i] = trackType[i] = 0;
        trackScale[i] = 5;
    };
    solutionsCurrent = solutionsStart = solutionsEnd = numSatellites[0] = numSatellites[1] = 0;
    nMapPoint = 0;
    monitorPortOpen = 0;
    timeStamps = NULL;
    solutionStatus = numValidSatellites = NULL;
    solutionCurrentStatus = 0;
    solutionRover = solutionReference = solutionQr = velocityRover = ages = ratioAR = NULL;

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < MAXSAT; j++) {
            satellites[i][j] = validSatellites[i][j] = 0;
            satellitesAzimuth[i][j] = satellitesElevation[i][j] = 0.0;
            for (int k = 0; k < NFREQ; k++)
                satellitesSNR[i][j][k] = 0;
        }

    static opt_t rcvopts[] = {
        { "inpstr1-type",     3, (void *)&strtype[0],  ISTOPT  },
        { "inpstr1-path",     2, (void *)strpath [0],  ""      },
        { "inpstr1-format",   3, (void *)&strfmt [0],  FMTOPT  },
        { "inpstr2-type",     3, (void *)&strtype[1],  ISTOPT  },
        { "inpstr2-path",     2, (void *)strpath [1],  ""      },
        { "inpstr2-format",   3, (void *)&strfmt [1],  FMTOPT  },
        { "inpstr2-nmeareq",  3, (void *)&nmeaRequestType, NMEOPT},
        { "inpstr2-nmealat",  1, (void *)&nmeaPosition[0], "deg"},
        { "inpstr2-nmealon",  1, (void *)&nmeaPosition[1], "deg"},
        { "inpstr3-type",     3, (void *)&strtype[2],  ISTOPT  },
        { "inpstr3-path",     2, (void *)strpath [2],  ""      },
        { "inpstr3-format",   3, (void *)&strfmt [2],  FMTOPT  },
        { "logstr1-type",     3, (void *)&strtype[3],  OSTOPT  },
        { "logstr1-path",     2, (void *)strpath [3],  ""      },
        { "logstr2-path",     2, (void *)strpath [4],  ""      },
        { "logstr2-type",     3, (void *)&strtype[4],  OSTOPT  },
        { "logstr3-type",     3, (void *)&strtype[5],  OSTOPT  },
        { "logstr3-path",     2, (void *)strpath [5],  ""      },
        { "outstr1-type",     3, (void *)&strtype[6],  OSTOPT  },
        { "outstr1-path",     2, (void *)strpath [6],  ""      },
        { "outstr1-format",   3, (void *)&strfmt [6],  SOLOPT  },
        { "outstr2-type",     3, (void *)&strtype[7],  OSTOPT  },
        { "outstr2-path",     2, (void *)strpath [7],  ""      },
        { "outstr2-format",   3, (void *)&strfmt [7],  SOLOPT  },
        { "outstr3-type",     3, (void *)&strtype[8],  OSTOPT  },
        { "outstr3-path",     2, (void *)strpath [8],  ""      },
        { "outstr3-format",   3, (void *)&strfmt [8],  SOLOPT  },
        { "",		      0, NULL,		       ""      }
    };

    rtksvr = (rtksvr_t *)malloc(sizeof(rtksvr_t));
    if (rtksvr == NULL) std::exit(1);
    rtksvrinit(rtksvr);
    strinit(&monistr);

    setWindowTitle(tr("%1 ver. %2 %3").arg(PRGNAME).arg(VER_RTKLIB, PATCH_LEVEL));
    setWindowIcon(QIcon(":/icons/rtknavi_Icon.ico"));

    panelMode = 0;

    for (int i = 0; i < 3; i++)
        trackOrigin[i] = 0.0;
    optDialog = new OptDialog(this, OptDialog::NaviOptions);
    optDialog->appOptions = rcvopts;
    inputStrDialog = new InputStrDialog(this);
    outputStrDialog = new OutputStrDialog(this);
    logStrDialog = new LogStrDialog(this);
    monitor = new MonitorDialog(this, rtksvr, &monistr);
    markDialog = new MarkDialog(this);
    systemTray = new QSystemTrayIcon(this);

    setTrayIcon(1);

    trayMenu = new QMenu(this);
    timerCycle = timerInactive = 0;

    ui->btnStop->setVisible(false);

    // setup stretcher to adjust label font size to avaiable space
    stretch = new LabelStretcher(ui->panelSolutionData);
    stretch->setManaged(ui->lblPosition1);
    stretch->setManaged(ui->lblPosition2);
    stretch->setManaged(ui->lblPosition3);
    stretch->setManaged(ui->lblPositionText1);
    stretch->setManaged(ui->lblPositionText2);
    stretch->setManaged(ui->lblPositionText3);
    stretch->setManaged(ui->lblSolution);
    stretch->setManaged(ui->lblSolutionText);
    stretch->setManaged(ui->lblStd);
    stretch->setManaged(ui->lblNSatellites);

    // set up tray menu
    trayMenu->addAction(tr("Main Window..."), this, &MainWindow::expandFromTray);
    trayMenu->addAction(tr("Monitor..."), this, &MainWindow::showMonitorDialog);
    trayMenu->addAction(tr("Plot..."), this, &MainWindow::showRtkPlot);
    trayMenu->addSeparator();
    menuStartAction = trayMenu->addAction(tr("Start"), this, &MainWindow::startServer);
    menuStopAction = trayMenu->addAction(tr("Stop"), this, &MainWindow::stopServer);
    trayMenu->addSeparator();
    menuExitAction = trayMenu->addAction(tr("Exit"), this, &MainWindow::exit);

    systemTray->setContextMenu(trayMenu);

    connect(ui->btnExit, &QPushButton::clicked, this, &MainWindow::exit);
    connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::startServer);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::stopServer);
    connect(ui->btnAbout, &QPushButton::clicked, this, &MainWindow::showAboutDialog);
    connect(ui->btnFrequencyType1, &QPushButton::clicked, this, &MainWindow::changeFrequencyType1);
    connect(ui->btnFrequencyType2, &QPushButton::clicked, this, &MainWindow::changeFrequencyType2);
    connect(ui->btnFrequencyType3, &QPushButton::clicked, this, &MainWindow::changeFrequencyType3);
    connect(ui->btnFrequencyType4, &QPushButton::clicked, this, &MainWindow::changeFrequencyType4);
    connect(ui->btnExpand1, &QPushButton::clicked, this, &MainWindow::expandPlot1);
    connect(ui->btnShrink1, &QPushButton::clicked, this, &MainWindow::shrinkPlot1);
    connect(ui->btnExpand2, &QPushButton::clicked, this, &MainWindow::expandPlot2);
    connect(ui->btnShrink2, &QPushButton::clicked, this, &MainWindow::shrinkPlot2);
    connect(ui->btnExpand3, &QPushButton::clicked, this, &MainWindow::expandPlot3);
    connect(ui->btnShrink3, &QPushButton::clicked, this, &MainWindow::shrinkPlot3);
    connect(ui->btnExpand4, &QPushButton::clicked, this, &MainWindow::expandPlot4);
    connect(ui->btnShrink4, &QPushButton::clicked, this, &MainWindow::shrinkPlot4);
    connect(ui->btnInputStream, &QPushButton::clicked, this, &MainWindow::showInputStreamDialog);
    connect(ui->btnLogStream, &QPushButton::clicked, this, &MainWindow::showLogStreamDialog);
    connect(ui->btnMonitor, &QPushButton::clicked, this, &MainWindow::showMonitorDialog);
    connect(ui->btnOptions, &QPushButton::clicked, this, &MainWindow::showOptionsDialog);
    connect(ui->btnOutputStream, &QPushButton::clicked, this, &MainWindow::showOutputStreamDialog);
    connect(ui->btnPanel, &QPushButton::clicked, this, &MainWindow::changePanelMode);
    connect(ui->btnPlot, &QPushButton::clicked, this, &MainWindow::showRtkPlot);
    connect(ui->btnPlotType1, &QPushButton::clicked, this, &MainWindow::changePlotType1);
    connect(ui->btnPlotType2, &QPushButton::clicked, this, &MainWindow::changePlotType2);
    connect(ui->btnPlotType3, &QPushButton::clicked, this, &MainWindow::changePlotType3);
    connect(ui->btnPlotType4, &QPushButton::clicked, this, &MainWindow::changePlotType4);
    connect(ui->btnSave, &QPushButton::clicked, this, &MainWindow::btnSaveClicked);
    connect(ui->btnSolutionType, &QPushButton::clicked, this, &MainWindow::changeSolutionType);
    connect(ui->btnSolutionType2, &QPushButton::clicked, this, &MainWindow::changeSolutionType);
    connect(ui->btnTaskTray, &QPushButton::clicked, this, &MainWindow::minimizeToTray);
    connect(ui->btnTimeSys, &QPushButton::clicked, this, &MainWindow::changeTimeSystem);
    connect(ui->btnMark, &QPushButton::clicked, this, &MainWindow::btnMarkClicked);
    connect(ui->sBSolution, &QScrollBar::valueChanged, this, &MainWindow::changeSolutionIndex);
    connect(systemTray, &QSystemTrayIcon::activated, this, &MainWindow::maximizeFromTray);
    connect(&updateTimer, &QTimer::timeout, this, &MainWindow::updateServer);

    updateTimer.setInterval(100);
    updateTimer.setSingleShot(false);
}

MainWindow::~MainWindow()
{
    delete [] timeStamps;
    delete [] solutionStatus;
    delete [] numValidSatellites;
    delete [] solutionRover;
    delete [] solutionReference;
    delete [] solutionQr;
    delete [] velocityRover;
    delete [] ages;
    delete [] ratioAR;

    rtksvrfree(rtksvr);
    free(rtksvr);
}

// callback on form create --------------------------------------------------
void MainWindow::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    trace(3, "showEvent\n");

#if 1
    QString file = QApplication::applicationFilePath();
    QFileInfo fi(file);
    iniFile = fi.absoluteDir().filePath(fi.baseName()) + ".ini";
#else // use unix config path
    QSettings tempSettings(QSettings::IniFormat, QSettings::UserScope, "rtknavi-qt", "rtklib");
    IniFile = tempSettings.fileName();
#endif
    initSolutionBuffer();
    strinitcom();

    QCommandLineParser parser;
    parser.setApplicationDescription(tr("RTK Navigation Qt"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption iniFileOption(QStringList() << "i" << "ini-file",
                     QCoreApplication::translate("main", "ini file to use"),
                     QCoreApplication::translate("main", "ini file"));
    parser.addOption(iniFileOption);

    QCommandLineOption trayOption(QStringList() << "tray",
                      QCoreApplication::translate("main", "start as task tray icon."));
    parser.addOption(trayOption);

    QCommandLineOption autoOption(QStringList() << "auto",
                      QCoreApplication::translate("main", "auto start."));
    parser.addOption(autoOption);

    parser.process(*QApplication::instance());

    if (parser.isSet(iniFileOption))
        iniFile = parser.value(iniFileOption);

    if (parser.isSet(trayOption)) {
        setVisible(false);
        systemTray->setVisible(true);
    }

    loadOptions();
    loadNavigation(&rtksvr->nav);
    openMonitorPort(optDialog->monitorPort);

    updatePanels();
    updateTimeSystem();
    updateSolutionType();
    updateFont();
    updatePosition();
    updateEnable();

    updatePlot();

    stretch->resized(ui->panelSolutionData);

    if (parser.isSet(autoOption)) {
        serverStart();
    }
    updateTimer.start();
}
// callback on form close ---------------------------------------------------
void MainWindow::closeEvent(QCloseEvent *)
{
    trace(3, "closeEvent\n");

    updateTimer.stop();

    if (monitorPortOpen > 0) {
        // send disconnect message
        strwrite(&monistr, (uint8_t *)MSG_DISCONN, strlen(MSG_DISCONN));

        strclose(&monistr);
    }
    saveOptions();
    saveNavigation(&rtksvr->nav);
}
// update panel -------------------------------------------------------------
void MainWindow::updatePanels()
{
    // panel modes
    // 0: solution big; 1 panel
    // 1: solution big; 2 panels
    // 2: solution big; 3 panels
    // 3: solution big; 4 panels
    // 4: solution small; 1 panel
    // 5: solution small; 2 panels
    // 6: solution small; 3 panels
    // 7: solution small; 4 panels

    if (panelMode <= 3) {
        ui->panelSolution->setVisible(true);
        ui->panelSolutionStatus ->setVisible(false);
    } else {
        ui->panelSolution->setVisible(false);
        ui->panelSolutionStatus ->setVisible(true);
    }

    if (panelMode == 0 || panelMode == 4) {
        ui->panelDisplay1->setVisible(true);
        ui->panelDisplay2->setVisible(false);
        ui->panelDisplay3->setVisible(false);
        ui->panelDisplay4->setVisible(false);
    }
    else if (panelMode == 1 || panelMode == 5) {
        ui->panelDisplay1->setVisible(true);
        ui->panelDisplay2->setVisible(true);
        ui->panelDisplay3->setVisible(false);
        ui->panelDisplay4->setVisible(false);
    }
    else if (panelMode == 2 || panelMode == 6) {
        ui->panelDisplay1->setVisible(true);
        ui->panelDisplay2->setVisible(true);
        ui->panelDisplay3->setVisible(true);
        ui->panelDisplay4->setVisible(false);
    }
    else {
        ui->panelDisplay1->setVisible(true);
        ui->panelDisplay2->setVisible(true);
        ui->panelDisplay3->setVisible(true);
        ui->panelDisplay4->setVisible(true);
    }

    if (optDialog->panelStacking == 0) { // horizontal
        ui->splitter->setOrientation(Qt::Horizontal);
        ui->splitter->setSizes({ui->splitter->size().width()/5, ui->splitter->size().width()/5,
                                ui->splitter->size().width()/5, ui->splitter->size().width()/5, ui->splitter->size().width()/5});
    }
    else { // vertical
        ui->splitter->setOrientation(Qt::Vertical);
        ui->splitter->setSizes({ui->splitter->size().height()/5, ui->splitter->size().height()/5,
                                ui->splitter->size().height()/5, ui->splitter->size().height()/5, ui->splitter->size().height()/5});
    }

}
// update enabled -----------------------------------------------------------
void MainWindow::updateEnable()
{
    QPushButton *btnFrequencyType[] = {ui->btnFrequencyType1, ui->btnFrequencyType2, ui->btnFrequencyType3, ui->btnFrequencyType4};
    QPushButton *btnExpand[] = {ui->btnExpand1, ui->btnExpand2, ui->btnExpand3, ui->btnExpand4};
    QPushButton *btnShrink[] = {ui->btnShrink1, ui->btnShrink3, ui->btnShrink3, ui->btnShrink4};

    for (int i = 0; i < 4; i++)
    {
        btnExpand[i]->setVisible(plotType[i] == 6);
        btnShrink[i]->setVisible(plotType[i] == 6);

        if (plotType[i] == 5)  // baseline
            btnFrequencyType[i]->setToolTip(tr("Change baseline mode"));
        if (plotType[i] == 6)
            btnFrequencyType[i]->setToolTip(tr("Change track type"));
        else
            btnFrequencyType[i]->setToolTip(tr("Change selected frqeuencies"));
    }
}
// callback on button-exit --------------------------------------------------
void MainWindow::exit()
{
    trace(3, "exit\n");

    close();
}
// callback on button-start -------------------------------------------------
void MainWindow::startServer()
{
    trace(3, "startServer\n");

    serverStart();
}
// callback on button-stop --------------------------------------------------
void MainWindow::stopServer()
{
    trace(3, "stopServer\n");

    serverStop();
}
// callback on button-plot --------------------------------------------------
void MainWindow::showRtkPlot()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"rtkplot_qt", "../../../bin/rtkplot_qt", "../rtkplot_qt/rtkplot_qt"};
    QStringList opts;

    trace(3, "showRtkPlot\n");

    if (monitorPortOpen <= 0) {
        QMessageBox::critical(this, tr("Error"), tr("Monitor port not open"));
        return;
    }

    opts << "-p" << QString("tcpcli://localhost:%1").arg(monitorPortOpen) << "-t" << QString("%1:  %2")
                .arg(windowTitle(), PRGNAME);

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }

    QMessageBox::critical(this, tr("Error"), tr("Error: rtkplot_qt could not be executed"));
}
// callback on button-options -----------------------------------------------
void MainWindow::showOptionsDialog()
{
    int itype[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPCLI, STR_FILE, STR_FTP, STR_HTTP};
    int otype[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPSVR, STR_NTRIPCAS, STR_FILE};
    char buff[1024], *p, *q;

    trace(3, "showOptionsDialog\n");

    for (int i = 0; i < MAXSTRRTK; i++) {
        int stype = streamType[i];
        if (i < 3) {
            if (stype >= 0 && stype < (int)(sizeof(itype) / sizeof(int)))
                strtype[i] = itype[stype];
            else
                strtype[i] = STR_NONE;
        } else {
            if (stype >= 0 && stype < (int)(sizeof(otype) / sizeof(int)))
                strtype[i] = otype[stype];
            else
                strtype[i] = STR_NONE;
        }

        strfmt[i] = inputFormat[i];

        if (!streamEnabled[i]) {
            strtype[i] = STR_NONE;
            strncpy(strpath[i], "", MAXSTR-1);
        } else if (strtype[i] == STR_SERIAL) {
            strncpy(strpath[i], qPrintable(paths[i][0]), MAXSTR-1);
        } else if (strtype[i] == STR_FILE) {
            strncpy(strpath[i], qPrintable(paths[i][2]), MAXSTR-1);
        } else if (strtype[i] == STR_TCPSVR) {
            strncpy(buff, qPrintable(paths[i][1]), MAXSTR-1);
            if ((p = strchr(buff, '/'))) *p = '\0'; // TODO
            if ((p = strrchr(buff, ':'))) {
                strncpy(strpath[i], p, MAXSTR-1);
            }
            else {
                strncpy(strpath[i], "", MAXSTR-1);
            }
        }
        else if (strtype[i] == STR_TCPCLI) {
            strncpy(buff, qPrintable(paths[i][1]), 1023);
            if ((p = strchr(buff, '/'))) *p = '\0';
            if ((p = strrchr(buff, '@'))) {
                strncpy(strpath[i], p+1, MAXSTR-1);
            }
            else {
                strncpy(strpath[i], buff, MAXSTR-1);
            }
        }
        else if (strtype[i] == STR_NTRIPSVR) {
            strncpy(buff, qPrintable(paths[i][1]), 1023);
            if ((p = strchr(buff, ':')) && strchr(p + 1, '@')) {
                strncpy(strpath[i], p, MAXSTR-1);
            }
            else {
                strncpy(strpath[i], buff, MAXSTR-1);
            }
        }
        else if (strtype[i] == STR_NTRIPCLI) {
            strncpy(buff, qPrintable(paths[i][1]), 1023);
            if ((p = strchr(buff, '/')) && (q = strchr(p + 1, ':'))) *q = '\0';
            strncpy(strpath[i], buff, MAXSTR);
        }
        else if (strtype[i] == STR_NTRIPCAS) {
            strncpy(buff, qPrintable(paths[i][1]), 1023);
            if ((p = strchr(buff, '/')) && (q = strchr(p + 1, ':'))) *q = '\0';
            if ((p = strchr(buff, '@'))) {
                *(p + 1) = '\0';
                strncpy(strpath[i], buff, MAXSTR-1);
            }
            if ((p = strchr(p ? p + 2 : buff, ':'))) {
                strncat(strpath[i], p, MAXSTR-1);
            }
        } else if (strtype[i] == STR_FTP || strtype[i] == STR_HTTP)
        {
            strncpy(strpath[i], qPrintable(paths[i][3]), MAXSTR-1);
        }
    }

    int solutionBufferSize_old = optDialog->solutionBufferSize;
    int monitorPort_old = optDialog->monitorPort;

    updateFont();
    updatePanels();

    optDialog->exec();

    if (optDialog->result() != QDialog::Accepted) return;

    for (int i = 0; i < MAXSTRRTK; i++) {
        streamType[i] = 0; // Default to serial
        bool found = false;
        if (i < 3) {
            // Input
            for (int j = 0; j < (int)(sizeof(itype) / sizeof(int)); j++) {
                if (strtype[i] != itype[j]) continue;
                streamType[i] = j;
                found = true;
                break;
            }
        } else {
            // Output or log
            for (int j = 0; j < (int)(sizeof(otype) / sizeof(int)); j++) {
                if (strtype[i] != otype[j]) continue;
                streamType[i] = j;
                found = true;
                break;
            }
        }
        // Disable if the stream type is not found.
        if (found == false) streamEnabled[i] = false;
        if (i < 3 || i >= 6) inputFormat[i] = strfmt[i];

        if (strtype[i] == STR_SERIAL)
            paths[i][0] = strpath[i];
        else if (strtype[i] == STR_FILE)
            paths[i][2] = strpath[i];
        else if (strtype[i] <= STR_NTRIPCLI)
            paths[i][1] = strpath[i];
        else if (strtype[i] <= STR_HTTP)
            paths[i][3] = strpath[i];
    }

    if (solutionBufferSize_old != optDialog->solutionBufferSize) {
        initSolutionBuffer();
        updateTime();
        updatePosition();
        updatePlot();
    }

    updateFont();
    updatePanels();

    if (monitorPort_old != optDialog->monitorPort)  {
        // send disconnect message
        if (monitorPortOpen > 0) {
            strwrite(&monistr, (uint8_t *)MSG_DISCONN, strlen(MSG_DISCONN));

            strclose(&monistr);
        }
        // reopen monitor stream
        openMonitorPort(optDialog->monitorPort);
    }
}
// callback on button-input-streams -----------------------------------------
void MainWindow::showInputStreamDialog()
{
    int i, j;

    trace(3, "showInputStreamDialog\n");

    for (i = 0; i < 3; i++) {
        inputStrDialog->setStreamEnabled(i, streamEnabled[i]);
        inputStrDialog->setStreamType(i, streamType[i]);
        inputStrDialog->setStreamFormat(i, inputFormat[i]);
        inputStrDialog->setReceiverOptions(i, receiverOptions[i]);

        /* Paths -> [0]:serial, [1]:tcp, [2]:file, [3]:ftp */
        for (j = 0; j < 4; j++) inputStrDialog->setPath(i, j, paths[i][j]);
    }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            inputStrDialog->setCommandsEnabled(i, j, commandEnabled[i][j]);
            inputStrDialog->setCommands(i, j, commands[i][j]);
            inputStrDialog->setCommandsTcpEnabled(i, j, commandEnableTcp[i][j]);
            inputStrDialog->setCommandsTcp(i, j, commandsTcp[i][j]);
        }
    for (i = 0; i < 10; i++) {
        inputStrDialog->setHistory(i, history[i]);
    }
    inputStrDialog->setNmeaRequestType(nmeaRequestType);
    inputStrDialog->setTimeTagEnabled(inputTimeTag);
    inputStrDialog->setTimeSpeed(inputTimeSpeed);
    inputStrDialog->setTimeStart(inputTimeStart);
    inputStrDialog->setTimeTag64bit(inputTimeTag64bit);
    inputStrDialog->setNMeaPosition(0, nmeaPosition[0]);
    inputStrDialog->setNMeaPosition(1, nmeaPosition[1]);
    inputStrDialog->setNMeaPosition(2, nmeaPosition[2]);
    inputStrDialog->setResetCommand(resetCommand);
    inputStrDialog->setMaxBaseLine(maxBaseLine);
    inputStrDialog->setStationPositionFile(optDialog->fileOptions.stapos);

    inputStrDialog->exec();

    if (inputStrDialog->result() != QDialog::Accepted) return;

    for (i = 0; i < 3; i++) {
        streamEnabled[i] = inputStrDialog->getStreamEnabled(i);
        streamType[i] = inputStrDialog->getStreamType(i);
        inputFormat [i] = inputStrDialog->getStreamFormat(i);
        receiverOptions [i] = inputStrDialog->getReceiverOptions(i);
        for (j = 0; j < 4; j++)
            paths[i][j] = inputStrDialog->getPath(i, j);
    }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            commandEnabled[i][j] = inputStrDialog->getCommandsEnabled(i, j);
            commands[i][j] = inputStrDialog->getCommands(i, j);
            commandEnableTcp[i][j] = inputStrDialog->getCommandsTcpEnabled(i, j);
            commandsTcp[i][j] = inputStrDialog->getCommandsTcp(i, j);
        }
    for (i = 0; i < 10; i++) {
        history[i] = inputStrDialog->getHistory(i);
    }
    nmeaRequestType = inputStrDialog->getNmeaRequestType();
    inputTimeTag = inputStrDialog->getTimeTagEnabled();
    inputTimeSpeed = inputStrDialog->getTimeSpeed();
    inputTimeStart = inputStrDialog->getTimeStart();
    inputTimeTag64bit = inputStrDialog->getTimeTag64bit();
    nmeaPosition[0] = inputStrDialog->getNMeaPosition(0);
    nmeaPosition[1] = inputStrDialog->getNMeaPosition(1);
    nmeaPosition[2] = inputStrDialog->getNMeaPosition(2);
    resetCommand = inputStrDialog->getResetCommand();
    maxBaseLine = inputStrDialog->getMaxBaseLine();
}
// confirm overwrite --------------------------------------------------------
int MainWindow::confirmOverwrite(const QString &path)
{
    int itype[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPCLI, STR_FILE, STR_FTP, STR_HTTP};
    int i;
    QString filename, streamFilename;

    trace(3, "confirmOverwrite\n");

    filename = path.mid(path.indexOf("::"));

    if (!QFile::exists(filename)) return 1; // file not exists

    // check overwrite input files
    for (i = 0; i < 3; i++) {
        if (!streamEnabled[i] || itype[streamType[i]] != STR_FILE) continue;

        streamFilename = paths[i][2];
        streamFilename = streamFilename.mid(streamFilename.indexOf("::"));

        if (filename == streamFilename) {
            ui->lblMessage->setText(tr("Invalid output %1").arg(filename));
            return 0;
        }
    }

    return QMessageBox::question(this, tr("File exists"), filename) == QMessageBox::Yes;
}
// callback on button-output-streams ----------------------------------------
void MainWindow::showOutputStreamDialog()
{
    int otype[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPSVR, STR_NTRIPCAS, STR_FILE};
    int i, j, str, update[RTKSVRNSOL] = { 0 };
    QString path;

    trace(3, "showOutputStreamDialog\n");

    for (i = 6; i < MAXSTRRTK; i++) {
        outputStrDialog->setStreamEnabled(i - 6, streamEnabled[i]);
        outputStrDialog->setStreamType(i - 6, streamType[i]);
        outputStrDialog->setStreamFormat(i - 6, inputFormat[i]);
        for (j = 0; j < 4; j++)
            outputStrDialog->setPath(i - 6, j, paths[i][j]);
    }
    for (i = 0; i < 10; i++) {
        outputStrDialog->setHistory(i, history[i]);
    }
    outputStrDialog->setTimeTagEnabled(outputTimeTag);
    outputStrDialog->setSwapInterval(outputSwapInterval);
    outputStrDialog->exec();

    if (outputStrDialog->result() != QDialog::Accepted) return;

    for (i = 6; i < MAXSTRRTK; i++) {
        if (streamEnabled[i] != outputStrDialog->getStreamEnabled(i - 6) ||
            streamType[i] != outputStrDialog->getStreamType(i - 6) ||
            inputFormat[i] != outputStrDialog->getStreamFormat(i - 6) ||
            paths[i][0] != outputStrDialog->getPath(i - 6, 0) ||
            paths[i][1] != outputStrDialog->getPath(i - 6, 1) ||
            paths[i][2] != outputStrDialog->getPath(i - 6, 2) ||
            paths[i][3] != outputStrDialog->getPath(i - 6, 3))
                update[i - 6] = 1;

        streamEnabled[i] = outputStrDialog->getStreamEnabled(i - 6);
        streamType[i] = outputStrDialog->getStreamType(i - 6);
        inputFormat[i] = outputStrDialog->getStreamFormat(i - 6);
        for (j = 0; j < 4; j++)
            paths[i][j] = outputStrDialog->getPath(i - 6, j);
    }
    for (i = 0; i < 10; i++) {
        history[i] = outputStrDialog->getHistory(i);
    }
    outputTimeTag = outputStrDialog->getTimeTagEnabled();
    outputSwapInterval = outputStrDialog->getSwapInterval();

    if (ui->btnStart->isEnabled()) return;

    for (i = 6; i < MAXSTRRTK; i++) {
        if (!update[i - 6]) continue;

        rtksvrclosestr(rtksvr, i);

        if (!streamEnabled[i]) continue;

        str = otype[streamType[i]];
        if (str == STR_SERIAL) path = paths[i][0];
        else if (str == STR_FILE) path = paths[i][2];
        else if (str == STR_FTP || str == STR_HTTP) path = paths[i][3];
        else path = paths[i][1];
        if (str == STR_FILE && !confirmOverwrite(path)) {
            streamEnabled[i] = 0;
            continue;
        }
        optDialog->solutionOptions.posf = inputFormat[i];
        rtksvropenstr(rtksvr, i, str, qPrintable(path), &optDialog->solutionOptions, &optDialog->processingOptions);
    }
}
// callback on button-log-streams -------------------------------------------
void MainWindow::showLogStreamDialog()
{
    int otype[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPSVR, STR_NTRIPCAS, STR_FILE};
    int i, j, str, update[3] = {0};
    QString path;

    trace(3, "showLogStreamDialog\n");

    for (i = 3; i < 6; i++) {
        logStrDialog->setStreamEnabled(i - 3, streamEnabled[i]);
        logStrDialog->setStreamType(i - 3, streamType [i]);
        for (j = 0; j < 4; j++)
            logStrDialog->setPath(i - 3, j, paths[i][j]);
    }
    for (i = 0; i < 10; i++) {
        logStrDialog->setHistory(i, history[i]);
    }
    logStrDialog->setLogTimeTagEnabled(logTimeTag);
    logStrDialog->setSwapInterval(logSwapInterval);

    logStrDialog->exec();

    if (logStrDialog->result() != QDialog::Accepted) return;

    for (i = 3; i < 6; i++) {
        if (streamEnabled[i] != logStrDialog->getStreamEnabled(i - 3) ||
            streamType[i] != logStrDialog->getStreamType(i - 3) ||
            paths[i][0] != logStrDialog->getPath(i - 3, 0) ||
            paths[i][1] != logStrDialog->getPath(i - 3, 1) ||
            paths[i][2] != logStrDialog->getPath(i - 3, 2) ||
            paths[i][3] != logStrDialog->getPath(i - 3, 3))
                update[i - 3] = 1;

        streamEnabled[i] = logStrDialog->getStreamEnabled(i - 3);
        streamType[i] = logStrDialog->getStreamType(i - 3);

        for (j = 0; j < 4; j++)
            paths[i][j] = logStrDialog->getPath(i - 3, j);
    }
    for (i = 0; i < 10; i++) {
        history[i] = logStrDialog->getHistory(i);
    }

    logTimeTag = logStrDialog->getLogTimeTagEnabled();
    logSwapInterval = logStrDialog->getSwapInterval();

    if (ui->btnStart->isEnabled()) return;

    for (i = 3; i < 6; i++) {
        if (!update[i - 3]) continue;

        rtksvrclosestr(rtksvr, i);

        if (!streamEnabled[i]) continue;

        str = otype[streamType[i]];
        if (str == STR_SERIAL) path = paths[i][0];
        else if (str == STR_FILE) path = paths[i][2];
        else if (str == STR_FTP || str == STR_HTTP) path = paths[i][3];
        else path = paths[i][1];
        if (str == STR_FILE && !confirmOverwrite(path)) {
            streamEnabled[i] = 0;
            continue;
        }
        rtksvropenstr(rtksvr, i, str, qPrintable(path), &optDialog->solutionOptions, &optDialog->processingOptions);
    }
}
// callback on button-solution-show -----------------------------------------
void MainWindow::changePanelMode()
{
    trace(3, "changePanelMode\n");

    if (++panelMode > MAXPANELMODE) panelMode = 0;

    updatePanels();
}
// callback on button-plot-type-1 -------------------------------------------
void MainWindow::changeTimeSystem()
{
    trace(3, "changeTimeSystem\n");

    if (++timeSystem > 3) timeSystem = 0;

    updateTimeSystem();
}
// callback on button-solution-type -----------------------------------------
void MainWindow::changeSolutionType()
{
    trace(3, "changeSolutionType\n");

    if (++solutionType > 4) solutionType = 0;

    updateSolutionType();
}
// callback on button-plottype-1 --------------------------------------------
void MainWindow::changePlotType1()
{
    trace(3, "changePlotType1\n");

    changePlotType(0);
}
// callback on button-plottype-2 --------------------------------------------
void MainWindow::changePlotType2()
{
    trace(3, "changePlotType2\n");

    changePlotType(1);
}
//---------------------------------------------------------------------------
void MainWindow::changePlotType3()
{
    trace(3, "changePlotType3\n");

    changePlotType(2);
}
//---------------------------------------------------------------------------
void MainWindow::changePlotType4()
{
    trace(3, "changePlotType4\n");

    changePlotType(3);
}
//---------------------------------------------------------------------------
void MainWindow::changePlotType(int i)
{
    if (++plotType[i] > 6) plotType[i] = 0;

    updatePlot();
    updatePosition();
    updateEnable();
}//---------------------------------------------------------------------------
void MainWindow::changeFrequencyType(int i)
{
    trace(3, "changeFrequencyType\n");

    if (plotType[i] == 6) {
        if (++trackType[i] > 1) trackType[i] = 0;

        updatePlot();
    } else if (plotType[i] == 5) {
        if (++baselineMode[i] > 1) baselineMode[i] = 0;

        updatePlot();
    } else {
        if (++frequencyType[i] > NFREQ + 1) frequencyType[i] = 0;

        updateSolutionType();
    }
}
// callback on button frequency-type-1 --------------------------------------
void MainWindow::changeFrequencyType1()
{
    trace(3, "changeFrequencyType1\n");

    changeFrequencyType(0);
}
// callback on button frequency-type-2 --------------------------------------
void MainWindow::changeFrequencyType2()
{
    trace(3, "changeFrequencyType2\n");

    changeFrequencyType(1);
}
//---------------------------------------------------------------------------
void MainWindow::changeFrequencyType3()
{
    trace(3,"changeFrequencyType3\n");

    changeFrequencyType(2);
}
//---------------------------------------------------------------------------
void MainWindow::changeFrequencyType4()
{
    trace(3,"changeFrequencyType\n");

    changeFrequencyType(3);
}
// callback on button expand-1 ----------------------------------------------
void MainWindow::expandPlot1()
{
    if (trackScale[0] <= 0) return;
    trackScale[0]--;

    updatePlot();
}
// callback on button shrink-1 ----------------------------------------------
void MainWindow::shrinkPlot1()
{
    if (trackScale[0] >= MAXTRKSCALE) return;
    trackScale[0]++;

    updatePlot();
}
// callback on button expand-2 ----------------------------------------------
void MainWindow::expandPlot2()
{
    if (trackScale[1] <= 0) return;
    trackScale[1]--;

    updatePlot();
}
// callback on button shrink-2 ----------------------------------------------
void MainWindow::shrinkPlot2()
{
    if (trackScale[1] >= MAXTRKSCALE) return;
    trackScale[1]++;

    updatePlot();
}
// callback on button expand-3 ----------------------------------------------
void MainWindow::expandPlot3()
{
    if (trackScale[2] <= 0) return;
    trackScale[2]--;

    updatePlot();
}
// callback on button shrink-3 ----------------------------------------------
void MainWindow::shrinkPlot3()
{
    if (trackScale[2] >= MAXTRKSCALE) return;
    trackScale[2]++;

    updatePlot();
}
// callback on button expand-4 ----------------------------------------------
void MainWindow::expandPlot4
    ()
{
    if (trackScale[3] <= 0) return;
    trackScale[3]--;

    updatePlot();
}
// callback on button shrink-4 ----------------------------------------------
void MainWindow::shrinkPlot4()
{
    if (trackScale[3] >= MAXTRKSCALE) return;
    trackScale[3]++;

    updatePlot();
}
// callback on button-rtk-monitor -------------------------------------------
void MainWindow::showMonitorDialog()
{
    trace(3, "showMonitorDialog\n");

    monitor->setWindowTitle(windowTitle() + tr(": RTK Monitor"));
    monitor->show();
}
// callback on scroll-solution change ---------------------------------------
void MainWindow::changeSolutionIndex()
{
    trace(3, "changeSolutionIndex\n");

    solutionsCurrent = solutionsStart + ui->sBSolution->value();
    if (solutionsCurrent >= optDialog->solutionBufferSize) solutionsCurrent -= optDialog->solutionBufferSize;  // wrap around

    updateTime();
    updatePosition();
    updatePlot();
}
// callback on button-save --------------------------------------------------
void MainWindow::btnSaveClicked()
{
    trace(3, "btnSaveClicked\n");

    saveLogs();
}
// callback on button-about -------------------------------------------------
void MainWindow::showAboutDialog()
{
    trace(3, "showAboutDialog\n");
    AboutDialog aboutDialog(this, QPixmap(":/icons/rtknavi"), PRGNAME);

    aboutDialog.exec();
}
// callback on button-tasktray ----------------------------------------------
void MainWindow::minimizeToTray()
{
    trace(3, "minimizeToTray\n");

    setVisible(false);
    systemTray->setToolTip(windowTitle());
    systemTray->setVisible(true);
}
// callback on button-tasktray ----------------------------------------------
void MainWindow::maximizeFromTray(QSystemTrayIcon::ActivationReason reason)
{
    trace(3, "maximizeFromTray\n");
    if (reason != QSystemTrayIcon::DoubleClick) return;

    setVisible(true);
    systemTray->setVisible(false);
}
// callback on menu-expand --------------------------------------------------
void MainWindow::expandFromTray()
{
    trace(3, "menuExpandClicked\n");

    setVisible(true);
    systemTray->setVisible(false);
}
// start rtk server ---------------------------------------------------------
void MainWindow::serverStart()
{
    solopt_t solopt[RTKSVRNSOL];
    double pos[3], nmeapos[3];
    int itype[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPCLI, STR_FILE, STR_FTP, STR_HTTP};
    int otype[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPSVR, STR_NTRIPCAS, STR_FILE};
    int i, j, streamTypes[MAXSTRRTK] = {0}, stropt[8] = {0};
    char *serverPaths[MAXSTRRTK], *cmds[3] = {0}, *cmds_periodic[3] = {0}, *rcvopts[3] = {0};
    char errmsg[20148];
    gtime_t time = timeget();
    pcvs_t pcvs;
    trace(3, "serverStart\n");

    memset(&pcvs, 0, sizeof(pcvs_t));

    ui->lblMessage->setText("");

    if (optDialog->solutionOptions.trace > 0) {
        traceopen(TRACEFILE);
        tracelevel(optDialog->solutionOptions.trace);
    }

    if (optDialog->fileOptions.rcvantp[0] != '\0' &&
        !readpcv(optDialog->fileOptions.rcvantp, &rtksvr->pcvsr)) {
        if (optDialog->solutionOptions.trace > 0) traceclose();
        ui->lblMessage->setText(tr("Receiver antenna file read error: %1").arg(optDialog->fileOptions.rcvantp));
        return;
    }

    if (optDialog->processingOptions.sateph == EPHOPT_PREC ||
        optDialog->processingOptions.sateph == EPHOPT_SSRCOM ||
        optDialog->processingOptions.mode >= PMODE_PPP_KINEMA) {
        if (optDialog->fileOptions.satantp[0] != '\0' &&
            !readpcv(optDialog->fileOptions.satantp, &pcvs)) {
            if (optDialog->solutionOptions.trace > 0) traceclose();
            free_pcvs(&rtksvr->pcvsr);
            ui->lblMessage->setText(tr("Satellite antenna file read error: %1").arg(optDialog->fileOptions.satantp));
            return;
        }
        for (i = 0; i < MAXSAT; i++) {
            pcv_t *pcv = searchpcv(i + 1, "", time, &pcvs);
            if (!pcv) continue;
            rtksvr->nav.pcvs[i] = *pcv;
        }
        free_pcvs(&pcvs);
    }

    for (i = 0; i < 3; i++) streamTypes[i] = streamEnabled[i] ? itype[streamType[i]] : STR_NONE;  // input stream
    for (i = 3; i < 6; i++) streamTypes[i] = streamEnabled[i] ? otype[streamType[i]] : STR_NONE;  // log streams
    for (i = 6; i < MAXSTRRTK; i++) streamTypes[i] = streamEnabled[i] ? otype[streamType[i]] : STR_NONE;  // output stream

    for (i = 0; i < MAXSTRRTK; i++) {
        serverPaths[i] = new char[1024];
        serverPaths[i][0] = '\0';
        if (streamTypes[i] == STR_NONE) strncpy(serverPaths[i], "", 1023);
        else if (streamTypes[i] == STR_SERIAL) strncpy(serverPaths[i], qPrintable(paths[i][0]), 1023);
        else if (streamTypes[i] == STR_FILE) strncpy(serverPaths[i], qPrintable(paths[i][2]), 1023);
        else if (streamTypes[i] == STR_FTP || streamTypes[i] == STR_HTTP) strncpy(serverPaths[i], qPrintable(paths[i][3]), 1023);
        else strncpy(serverPaths[i], qPrintable(paths[i][1]), 1023);
    }

    for (i = 0; i < 3; i++) {
        rcvopts[i] = new char[1024];
        cmds[i] = cmds_periodic[i] = NULL;

        if (streamTypes[i] == STR_SERIAL) {
            cmds[i] = new char[1024];
            cmds[i][0] = '\0';
            cmds_periodic[i] = new char[1024];
            cmds_periodic[i][0] = '\0';
            if (commandEnabled[i][0]) strncpy(cmds[i], qPrintable(commands[i][0]), 1023);
            if (commandEnabled[i][2]) strncpy(cmds_periodic[i], qPrintable(commands[i][2]), 1023);
        } else if (streamTypes[i] == STR_TCPCLI || streamTypes[i] == STR_TCPSVR || streamTypes[i] == STR_NTRIPCLI) {
            cmds[i] = new char[1024];
            cmds[i][0] = '\0';
            cmds_periodic[i] = new char[1024];
            cmds_periodic[i][0] = '\0';
            if (commandEnableTcp[i][0]) strncpy(cmds[i], qPrintable(commandsTcp[i][0]), 1023);
            if (commandEnabled[i][2]) strncpy(cmds_periodic[i], qPrintable(commandsTcp[i][2]), 1023);
        };
        strncpy(rcvopts[i], qPrintable(receiverOptions[i]), 1023);
    }
    pos[0] = nmeaPosition[0] * D2R;
    pos[1] = nmeaPosition[1] * D2R;
    pos[2] = nmeaPosition[2];
    pos2ecef(pos, nmeapos);

    strsetdir(optDialog->fileOptions.tempdir);
    strsetproxy(qPrintable(optDialog->proxyAddress));

    for (i = 3; i < MAXSTRRTK; i++)
        if (streamTypes[i] == STR_FILE && !confirmOverwrite(serverPaths[i])) {
            if (optDialog->solutionOptions.trace > 0) traceclose();
            free_pcvs(&rtksvr->pcvsr);
            for (j = 0; j < MAXSTRRTK; j++) delete[] serverPaths[j];
            for (j = 0; j < 3; j++) delete[] rcvopts[j];
            for (j = 0; j < 3; j++)
                if (cmds[j]) delete[] cmds[j];
            for (j = 0; j < 3; j++)
                if (cmds_periodic[j]) delete[] cmds_periodic[j];
            return;
        }

    if (optDialog->solutionOptions.sstat > 0)
        rtkopenstat(STATFILE, optDialog->solutionOptions.sstat);
    if (optDialog->solutionOptions.geoid > 0 && strlen(optDialog->fileOptions.geoid) > 0)
        opengeoid(optDialog->solutionOptions.geoid, qPrintable(optDialog->fileOptions.geoid));
    if (strlen(optDialog->fileOptions.dcb) > 0)
        readdcb(optDialog->fileOptions.dcb, &rtksvr->nav, NULL);

    for (i = 0; i < RTKSVRNSOL; i++) {
        solopt[i] = optDialog->solutionOptions;
        solopt[i].posf = inputFormat[6 + i];
    }
    stropt[0] = optDialog->timeoutTime;
    stropt[1] = optDialog->reconnectTime;
    stropt[2] = 1000;  // averaging time of data rate (ms)
    stropt[3] = optDialog->serverBufferSize;
    stropt[4] = optDialog->fileSwapMargin;
    strsetopt(stropt);
    strncpy(rtksvr->cmd_reset, qPrintable(resetCommand), 4095);
    rtksvr->bl_reset = maxBaseLine;

    // start rtk server
    if (!rtksvrstart(rtksvr, optDialog->serverCycle, optDialog->serverBufferSize, streamTypes, (const char **)serverPaths, inputFormat, optDialog->navSelect,
                      (const char **)cmds, (const char **)cmds_periodic, (const char **)rcvopts, optDialog->nmeaCycle, nmeaRequestType, nmeapos,
                     &optDialog->processingOptions, solopt, &monistr, errmsg)) {

        trace(2, "rtksvrstart error %s\n", errmsg);
        if (optDialog->solutionOptions.trace > 0) traceclose();
        free_pcvs(&rtksvr->pcvsr);
        for (i = 0; i < MAXSTRRTK; i++) delete[] serverPaths[i];
        for (i = 0; i < 3; i++) delete[] rcvopts[i];
        for (i = 0; i < 3; i++)
            if (cmds[i]) delete[] cmds[i];
        for (i = 0; i < 3; i++)
            if (cmds_periodic[i]) delete[] cmds_periodic[i];
        return;
    }

    for (i = 0; i < MAXSTRRTK; i++) delete[] serverPaths[i];
    for (i = 0; i < 3; i++) delete[] rcvopts[i];
    for (i = 0; i < 3; i++)
        if (cmds[i]) delete[] cmds[i];
    for (i = 0; i < 3; i++)
        if (cmds_periodic[i]) delete[] cmds_periodic[i];

    solutionsCurrent = solutionsStart = solutionsEnd = 0;
    solutionStatus[0] = numValidSatellites[0] = 0;

    for (i = 0; i < 3; i++) solutionRover[i] = solutionReference[i] = velocityRover[i] = 0.0;
    for (i = 0; i < 9; i++) solutionQr[i] = 0.0;

    ages[0] = ratioAR[0] = 0.0;
    numSatellites[0] = numSatellites[1] = 0;

    updatePosition();
    updatePlot();

    ui->btnStart->setVisible(false);
    ui->btnStart->setEnabled(false);
    ui->btnOptions->setEnabled(false);
    ui->btnExit->setEnabled(false);
    ui->btnInputStream->setEnabled(false);
    ui->btnSave->setEnabled(false);
    menuStartAction->setEnabled(false);
    menuExitAction->setEnabled(false);
    ui->sBSolution->setEnabled(false);
    ui->btnStop->setVisible(true);
    menuStopAction->setEnabled(true);
    setWidgetBackgroundColor(ui->lblServer, Color::Orange);

    setTrayIcon(0);
}
// stop rtk server ---------------------------------------------------------
void MainWindow::serverStop()
{
    char *cmds[3] = { 0 };
    int i, n, m, streamTypes;

    trace(3, "serverStop\n");

    for (i = 0; i < 3; i++) {
        cmds[i] = NULL;
        streamTypes = rtksvr->stream[i].type;

        if (streamTypes == STR_SERIAL) {
            cmds[i] = new char[1024];
            cmds[i][0] = '\0';
            if (commandEnabled[i][1]) strncpy(cmds[i], qPrintable(commands[i][1]), 1023);
        } else if (streamTypes == STR_TCPCLI || streamTypes == STR_TCPSVR || streamTypes == STR_NTRIPCLI) {
            cmds[i] = new char[1024];
            cmds[i][0] = '\0';
            if (commandEnableTcp[i][1]) strncpy(cmds[i], qPrintable(commandsTcp[i][1]), 1023);
        }
    }
    rtksvrstop(rtksvr, (const char **)cmds);

    free_pcvs(&rtksvr->pcvsr);

    for (i = 0; i < 3; i++) delete[] cmds[i];

    ui->btnStart->setVisible(true);
    ui->btnStart->setEnabled(true);
    ui->btnOptions->setEnabled(true);
    ui->btnExit->setEnabled(true);
    ui->btnInputStream->setEnabled(true);
    ui->btnSave->setEnabled(true);
    menuStartAction->setEnabled(true);
    menuExitAction->setEnabled(true);
    ui->sBSolution->setEnabled(true);
    ui->btnStop->setVisible(false);
    menuStopAction->setEnabled(false);
    setWidgetBackgroundColor(ui->lblServer, QColor(Qt::darkGray));

    setTrayIcon(1);

    setWidgetTextColor(ui->lblTime, QColor(Qt::darkGray));
    setWidgetBackgroundColor(ui->lblIndicatorSolution, QColor(Qt::white));

    n = solutionsEnd - solutionsStart; if (n < 0) n += optDialog->solutionBufferSize;
    m = solutionsCurrent - solutionsStart;  if (m < 0) m += optDialog->solutionBufferSize;
    if (n > 0) {
        ui->sBSolution->setMaximum(n - 1);
        ui->sBSolution->setValue(m);
    }
    ui->lblMessage->setText("");

    if (optDialog->solutionOptions.trace > 0) traceclose();
    if (optDialog->solutionOptions.sstat > 0) rtkclosestat();
    if (optDialog->solutionOptions.geoid > 0 && strlen(optDialog->fileOptions.geoid) == 0) closegeoid();
}
// callback on interval timer -----------------------------------------------
void MainWindow::updateServer()
{
    sol_t *sol;
    int i, update = 0;

    trace(4, "updateServer\n");

    timerCycle++;

    rtksvrlock(rtksvr);

    for (i = 0; i < rtksvr->nsol; i++) {
        sol = rtksvr->solbuf + i;
        updateLog(sol->stat, sol->time, sol->rr, sol->qr, rtksvr->rtk.rb, sol->ns, sol->age, sol->ratio);
        update = 1;
    }
    rtksvr->nsol = 0;
    solutionCurrentStatus = rtksvr->state ? rtksvr->rtk.sol.stat : 0;

    rtksvrunlock(rtksvr);

    if (update) {
        updateTime();
        updatePosition();
        timerInactive = 0;
    } else {
        if (++timerInactive * updateTimer.interval() > TIMEOUT) solutionCurrentStatus = 0;
    }

    if (solutionCurrentStatus) {
        setWidgetBackgroundColor(ui->lblServer, Color::Lime);
        setWidgetTextColor(ui->lblTime, QColor(Qt::black));
    } else {
        setWidgetBackgroundColor(ui->lblIndicatorSolution, QColor(Qt::white));
        setWidgetTextColor(ui->lblSolution, QColor(Qt::darkGray));
        setWidgetBackgroundColor(ui->lblServer, rtksvr->state ? QColor(Qt::darkGreen) : QColor(Qt::darkGray));
    }

    if (!(++timerCycle % 5)) updatePlot();

    updateStream();

    // keep alive for monitor port
    if (!(++timerCycle % (KACYCLE / updateTimer.interval())) && monitorPortOpen) {
        unsigned char buf[1];
        buf[0] = '\r';
        strwrite(&monistr, buf, 1);
    }
}

// update time-system -------------------------------------------------------
void MainWindow::updateTimeSystem()
{
    static const QString label[] = {tr("GPST"), tr("UTC"), tr("Local"), tr("GPST")};

    trace(3, "updateTimeSystem\n");

    ui->btnTimeSys->setText(label[timeSystem]);

    updateTime();
}
// update solution type -----------------------------------------------------
void MainWindow::updateSolutionType()
{
    static const QString label[] = {
        tr("Lat/Lon/Height"), tr("Lat/Lon/Height"), tr("X/Y/Z-ECEF"), tr("E/N/U-Baseline"),
        tr("Pitch/Yaw/Length-Baseline"), ""
    };

    trace(3, "updateSolutionType\n");

    ui->lblLatLonHeight->setText(label[solutionType]);

    updatePosition();
}
// update log ---------------------------------------------------------------
void MainWindow::updateLog(int stat, gtime_t time, double *rr,
               float *qr, double *rb, int ns, double age, double ratio)
{
    if (!stat) return;
    trace(4, "updateLog\n");

    solutionStatus[solutionsEnd] = stat;
    timeStamps[solutionsEnd] = time;
    numValidSatellites[solutionsEnd] = ns;
    ages[solutionsEnd] = age;
    ratioAR[solutionsEnd] = ratio;

    for (int i = 0; i < 3; i++) {
        solutionRover[i + solutionsEnd * 3] = rr[i];
        solutionReference[i + solutionsEnd * 3] = rb[i];
        velocityRover[i + solutionsEnd * 3] = rr[i + 3];
    }

    solutionQr[solutionsEnd * 9] = qr[0];
    solutionQr[solutionsEnd * 9 + 4] = qr[1];
    solutionQr[solutionsEnd * 9 + 8] = qr[2];
    solutionQr[solutionsEnd * 9 + 1] = solutionQr[solutionsEnd * 9 + 3] = qr[3];
    solutionQr[solutionsEnd * 9 + 5] = solutionQr[solutionsEnd * 9 + 7] = qr[4];
    solutionQr[solutionsEnd * 9 + 2] = solutionQr[solutionsEnd * 9 + 6] = qr[5];

    solutionsCurrent = solutionsEnd;

    // check for wrap around of ring buffer
    if (++solutionsEnd >= optDialog->solutionBufferSize) solutionsEnd = 0;
    if (solutionsEnd == solutionsStart && ++solutionsStart >= optDialog->solutionBufferSize) solutionsStart = 0;
}
// update font --------------------------------------------------------------
void MainWindow::updateFont()
{
    QLabel *label[] = {
        ui->lblSolutionText, ui->lblPositionText1, ui->lblPositionText2, ui->lblPositionText3,
        ui->lblPosition1, ui->lblPosition2, ui->lblPosition3, ui->lblSolution, ui->lblStd, ui->lblNSatellites
    };
    const QColor &color = label[7]->palette().color(label[7]->foregroundRole());

    trace(4, "updateFont\n");

    for (int i = 0; i < 10; i++) {
        label[i]->setFont(optDialog->positionFont);
        setWidgetTextColor(label[8], optDialog->positionFontColor);
    }
    QFont tmpFont = optDialog->positionFont;
    label[0]->setFont(tmpFont);
    setWidgetTextColor(label[7], color);
    label[8]->setFont(tmpFont); setWidgetTextColor(label[8], QColor(Qt::darkGray));
    label[9]->setFont(tmpFont); setWidgetTextColor(label[9], QColor(Qt::darkGray));
}
// update time --------------------------------------------------------------
void MainWindow::updateTime()
{
    gtime_t time = timeStamps[solutionsCurrent];
    struct tm *t;
    double tow;
    int week;
    char tstr[40];
    QString str;

    trace(4, "updateTime\n");

    if (timeSystem == 0) {  // GPST
        time2str(time, tstr, 1);
        str = tstr;
    } else if (timeSystem == 1) {  // UTC
        time2str(gpst2utc(time), tstr, 1);
        str = tstr;
    } else if (timeSystem == 2) {  // local time
        time = gpst2utc(time);

        if (!(t = localtime(&time.time))) str = "2000/01/01 00:00:00.0";
        else str = QString("%1/%2/%3 %4:%5:%6.%7").arg(t->tm_year + 1900, 4, 10, QChar('0'))
               .arg(t->tm_mon + 1, 2, 10, QChar('0')).arg(t->tm_mday, 2, 10, QChar('0')).arg(t->tm_hour, 2, 10, QChar('0')).arg(t->tm_min, 2, 10, QChar('0'))
               .arg(t->tm_sec, 2, 10, QChar('0')).arg(static_cast<int>(time.sec * 10));
    } else if (timeSystem == 3) {  // GPS time (week & TOW)
        tow = time2gpst(time, &week);
        str = tr("week: %L1, %L2 s").arg(week, 4, 10, QChar('0')).arg(tow, 8, 'f', 1);
    }
    ui->lblTime->setText(str);
}
// update solution display --------------------------------------------------
void MainWindow::updatePosition()
{
    QLabel *label[] = {ui->lblPositionText1, ui->lblPositionText2, ui->lblPositionText3,
                       ui->lblPosition1, ui->lblPosition2, ui->lblPosition3, ui->lblStd, ui->lblNSatellites};
    static const QString sol[] = {tr("----"), tr("FIX"), tr("FLOAT"), tr("SBAS"), tr("DGPS"), tr("SINGLE"), tr("PPP")};
    QString s[9], ext;
    static const QColor color[] = {QColor(Qt::lightGray), QColor(Qt::darkGreen), Color::Orange, Color::Fuchsia, QColor(Qt::blue), QColor(Qt::red), Color::Teal};
    double *rrover = solutionRover + solutionsCurrent * 3;
    double *rbase = solutionReference + solutionsCurrent * 3;
    double *qrover = solutionQr + solutionsCurrent * 9;
    double pos[3] = {0}, Qe[9] = {0};
    double dms1[3] = {0}, dms2[3] = {0}, baseline[3] = {0}, enu[3] = {0}, pitch = 0.0, yaw = 0.0, len;
    int i, stat = solutionStatus[solutionsCurrent];

    trace(4, "updatePosition\n");

    if (rtksvr->rtk.opt.mode == PMODE_STATIC || rtksvr->rtk.opt.mode == PMODE_PPP_STATIC)
        ext = " (S)";
    else if (rtksvr->rtk.opt.mode == PMODE_FIXED || rtksvr->rtk.opt.mode == PMODE_PPP_FIXED)
        ext = " (F)";
    ui->lblSolutionText->setText(tr("Solution%1:").arg(ext));

    ui->lblSolution->setText(sol[stat]);
    setWidgetTextColor(ui->lblSolution, rtksvr->state ? color[stat] : QColor(Qt::darkGray));
    setWidgetBackgroundColor(ui->lblIndicatorSolution, rtksvr->state ? color[stat] : QColor(Qt::white));
    ui->lblIndicatorSolution->setToolTip(sol[stat]);

    if (norm(rrover, 3) > 0.0 && norm(rbase, 3) > 0.0)
        for (i = 0; i < 3; i++)
            baseline[i] = rrover[i] - rbase[i];

    len = norm(baseline, 3);
    if (solutionType == 0) {  // DMS
        if (norm(rrover, 3) > 0.0) {
            ecef2pos(rrover, pos);
            covenu(pos, qrover, Qe);
            degtodms(pos[0] * R2D, dms1);
            degtodms(pos[1] * R2D, dms2);
            if (optDialog->solutionOptions.height == 1) pos[2] -= geoidh(pos); /* geodetic */
        }
        s[0] = pos[0] < 0 ? tr("S:") : tr("N:");
        s[1] = pos[1] < 0 ? tr("W:") : tr("E:");
        s[2] = optDialog->solutionOptions.height == 1 ? tr("H:") : tr("He:");
        s[3] = tr("%L1° %L2' %L3\"").arg(fabs(dms1[0]), 0, 'f', 0).arg(dms1[1], 2, 'f', 0, '0').arg(dms1[2], 7, 'f', 4, '0');
        s[4] = tr("%L1° %L2' %L3\"").arg(fabs(dms2[0]), 0, 'f', 0).arg(dms2[1], 2, 'f', 0, '0').arg(dms2[2], 7, 'f', 4, '0');
        s[5] = tr("%L1 m").arg(pos[2], 0, 'f', 3);
        s[6] = tr("N: %L1 m, E: %L2 m, U: %L3 m").arg(SQRT(Qe[4]), 6, 'f', 3).arg(SQRT(Qe[0]), 6, 'f', 3).arg(SQRT(Qe[8]), 6, 'f', 3);
    } else if (solutionType == 1) {  // deg
        if (norm(rrover, 3) > 0.0) {
            ecef2pos(rrover, pos); covenu(pos, qrover, Qe);
            if (optDialog->solutionOptions.height == 1) pos[2] -= geoidh(pos); /* geodetic */
        }
        s[0] = pos[0] < 0 ? tr("S:") : tr("N:");
        s[1] = pos[1] < 0 ? tr("W:") : tr("E:");
        s[2] = optDialog->solutionOptions.height == 1 ? tr("H:") : tr("He:");
        s[3] = QStringLiteral("%L1°").arg(fabs(pos[0]) * R2D, 0, 'f', 8);
        s[4] = QStringLiteral("%L1°").arg(fabs(pos[1]) * R2D, 0, 'f', 8);
        s[5] = QStringLiteral("%L1").arg(pos[2], 0, 'f', 3);
        s[6] = tr("N: %L1 m, E: %L2 m, U: %L3 m").arg(SQRT(Qe[4]), 6, 'f', 3).arg(SQRT(Qe[0]), 6, 'f', 3).arg(SQRT(Qe[8]), 6, 'f', 3);
    } else if (solutionType == 2) {  // XYZ
        s[0] = "X:";
        s[1] = "Y:";
        s[2] = "Z:";
        s[3] = tr("%L1 m").arg(rrover[0], 0, 'f', 3);
        s[4] = tr("%L1 m").arg(rrover[1], 0, 'f', 3);
        s[5] = tr("%L1 m").arg(rrover[2], 0, 'f', 3);
        s[6] = tr("X: %L1 m, Y: %L2 m, Z: %L3 m").arg(SQRT(qrover[0]), 6, 'f', 3).arg(SQRT(qrover[4]), 6, 'f', 3).arg(SQRT(qrover[8]), 6, 'f', 3);
    } else if (solutionType == 3) {  // ENU
        if (len > 0.0) {
            ecef2pos(rbase, pos);
            ecef2enu(pos, baseline, enu);
            covenu(pos, qrover, Qe);
        }
        s[0] = tr("E:"); s[1] = tr("N:"); s[2] = tr("U:");
        s[3] = tr("%L1 m").arg(enu[0], 0, 'f', 3);
        s[4] = tr("%L1 m").arg(enu[1], 0, 'f', 3);
        s[5] = tr("%L1 m").arg(enu[2], 0, 'f', 3);
        s[6] = tr("N: %L1 m, E: %L2 m, U: %L3 m").arg(SQRT(Qe[4]), 6, 'f', 3).arg(SQRT(Qe[0]), 6, 'f', 3).arg(SQRT(Qe[8]), 6, 'f', 3);
    } else {  // pitch/yaw/len
        if (len > 0.0) {
            ecef2pos(rbase, pos);
            ecef2enu(pos, baseline, enu);
            covenu(pos, qrover, Qe);
            pitch = asin(enu[2] / len);
            yaw = atan2(enu[0], enu[1]); if (yaw < 0.0) yaw += 2.0 * PI;
        }
        s[0] = tr("P:");
        s[1] = tr("Y:");
        s[2] = tr("L:");
        s[3] = tr("%L1°").arg(pitch * R2D, 0, 'f', 3);
        s[4] = tr("%L1°").arg(yaw * R2D, 0, 'f', 3);
        s[5] = tr("%L1 m").arg(len, 0, 'f', 3);
        s[6] = tr("N: %L1 m, E: %L2 m, U: %L3 m").arg(SQRT(Qe[4]), 6, 'f', 3).arg(SQRT(Qe[0]), 6, 'f', 3).arg(SQRT(Qe[8]), 6, 'f', 3);
    }
    s[7] = tr("Age: %L1 s, Ratio: %L2, #Sat: %L3").arg(ages[solutionsCurrent], 4, 'f', 1).arg(ratioAR[solutionsCurrent], 4, 'f', 1).arg(numValidSatellites[solutionsCurrent], 2);

    if (ratioAR[solutionsCurrent] > 0.0)
        s[8] = tr(" R: %L1").arg(ratioAR[solutionsCurrent], 4, 'f', 1);

    for (i = 0; i < 8; i++)
        label[i]->setText(s[i]);
    for (i = 3; i < 6; i++)
        setWidgetTextColor(label[i], optDialog->processingOptions.mode == PMODE_MOVEB && solutionType <= 2 ? QColor(Qt::darkGray) : QColor(Qt::black));

    setWidgetBackgroundColor(ui->lblIndicatorQ, ui->lblIndicatorSolution->palette().color(ui->lblIndicatorSolution->backgroundRole()));
    ui->lblIndicatorQ->setToolTip(ui->lblIndicatorSolution->toolTip());
    ui->lblSolutionS->setText(ui->lblSolution->text());
    setWidgetTextColor(ui->lblSolutionS, ui->lblSolution->palette().color(ui->lblSolution->foregroundRole()));
    ui->lblSolutionQ->setText(QStringLiteral("%1 %2 %3, %4 %5, %6 %7 %8").arg(
                                                                ext,
                                                                label[0]->text(),
                                                                label[3]->text(),
                                                                label[1]->text(),
                                                                label[4]->text(),
                                                                label[2]->text(),
                                                                label[5]->text(),
                                                                s[8]));
}
// update stream status indicators ------------------------------------------
void MainWindow::updateStream()
{
    static const QColor color[] = {QColor(Qt::red), QColor(Qt::white), Color::Orange, Qt::darkGreen, Color::Lime};
    QLabel *ind[MAXSTRRTK] = {ui->lblStream1, ui->lblStream2, ui->lblStream3, ui->lblStream4, ui->lblStream5, ui->lblStream6, ui->lblStream7, ui->lblStream8, ui->lblStream9};
    int i, sstat[MAXSTRRTK] = {0};
    char msg[MAXSTRMSG] = "";

    trace(4, "updateStream\n");

    rtksvrsstat(rtksvr, sstat, msg);
    for (i = 0; i < MAXSTRRTK; i++) {
        setWidgetBackgroundColor(ind[i], color[sstat[i]+1]);
        if (sstat[i]) {
            ui->lblMessage->setText(msg);
            ui->lblMessage->setToolTip(msg);
        }
    }
}
// draw solution plot -------------------------------------------------------
void MainWindow::drawSolutionPlot(QLabel *plot, int type, int freq)
{
    QString s1, s2, fstr[NFREQ+2];
    gtime_t time;
    int id;

    if (!plot) return;

    QPixmap buffer(plot->size());

    QPainter c(&buffer);
    c.setRenderHint(QPainter::Antialiasing);
    QFont font;
    font.setPixelSize(8);
    c.setFont(font);

    if (plot == ui->lblDisplay1) id = 0;
    else if (plot == ui->lblDisplay2) id = 1;
    else if (plot == ui->lblDisplay3) id = 2;
    else if (plot == ui->lblDisplay4) id = 3;
    else return;

    int w = buffer.size().width() - 2;
    int h = buffer.height() - 2;
    int i, j, x, sat[2][MAXSAT], ns[2], snr[2][MAXSAT][NFREQ], vsat[2][MAXSAT];
    int *snr0[MAXSAT], *snr1[MAXSAT], topMargin = QFontMetrics(optDialog->panelFont).height()*3/2;
    double az[2][MAXSAT], el[2][MAXSAT], rr[3], pos[3];

    trace(4, "drawSolutionPlot\n");

    for (i = 0; i < NFREQ; i++)
        fstr[i + 1] = QStringLiteral("L%1").arg(i + 1);
    fstr[i + 1] = " SYS";

    for (i = 0; i < MAXSAT; i++) {
        snr0[i] = snr[0][i];
        snr1[i] = snr[1][i];
    }
    ns[0] = rtksvrostat(rtksvr, 0, &time, sat[0], az[0], el[0], snr0, vsat[0]);
    ns[1] = rtksvrostat(rtksvr, 1, &time, sat[1], az[1], el[1], snr1, vsat[1]);

    rtksvrlock(rtksvr);
    matcpy(rr, rtksvr->rtk.sol.rr, 3, 1);
    ecef2pos(rr, pos);
    rtksvrunlock(rtksvr);

    for (i = 0; i < 2; i++) {
        if (ns[i] > 0) {
            numSatellites[i] = ns[i];
            for (int j = 0; j < ns[i]; j++) {
                satellites[i][j] = sat[i][j];
                satellitesAzimuth[i][j] = az[i][j];
                satellitesElevation[i][j] = el[i][j];
                for (int k = 0; k < NFREQ; k++)
                    satellitesSNR[i][j][k] = snr[i][j][k];
                validSatellites[i][j] = vsat[i][j];
            }
        } else {
            for (j = 0; j < numSatellites[i]; j++) {
                validSatellites[i][j] = 0;
                for (int k = 0; k < NFREQ; k++)
                    satellitesSNR[i][j][k] = 0;
            }
        }
    }
    c.fillRect(buffer.rect(), QBrush(Qt::white));
    x = 4;
    if (type == 0) { // snr plot rover+base
        if (w <= 3 * h) { // vertical
            drawSnr(c, w, (h - topMargin) / 2, 0, topMargin, 0, freq);
            drawSnr(c, w, (h - topMargin) / 2, 0, topMargin + (h - topMargin) / 2, 1, freq);
            s1 = tr("Rover: Base %L1 SNR (dBHz)").arg(fstr[freq]);
            drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
        } else { // horizontal
            drawSnr(c, w / 2, h - topMargin, 0, topMargin, 0, freq);
            drawSnr(c, w / 2, h - topMargin, w / 2, topMargin, 1, freq);
            s1 = tr("Rover %L1 SNR (dBHz)").arg(fstr[freq]);
            s2 = tr("Base %L1 SNR (dBHz)").arg(fstr[freq]);
            drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
            drawText(c, w / 2 + x, 1, s2, Qt::darkGray, 1, 2);
        }
    } else if (type == 1) { // snr plot rover
        drawSnr(c, w, h - topMargin, 0, topMargin, 0, type);
        s1 = tr("Rover %L1 SNR (dBHz)").arg(fstr[freq]);
        drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
    } else if (type == 2) { // skyplot rover
        drawSatellites(c, w, h, 0, 0, 0, type);
        s1 = tr("Rover %L1").arg(fstr[type]);
        drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
    } else if (type == 3) { // skyplot+snr plot rover
        s1 = tr("Rover %L1").arg(fstr[freq]);
        s2 = tr("SNR (dBHz)");
        if (w >= h * 2) { // horizontal
            drawSatellites(c, h, h, 0, 0, 0, freq);
            drawSnr(c, w - h, h - topMargin, h, topMargin, 0, freq);
            drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
            drawText(c, x + h, 1, s2, Qt::darkGray, 1, 2);
        } else { // vertical
            drawSatellites(c, w, h / 2, 0, 0, 0, freq);
            drawSnr(c, w, (h - topMargin) / 2, 0, topMargin + (h - topMargin) / 2, 0, freq);
            drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
        }
    } else if (type == 4) { // skyplot rover+base
        s1 = tr("Rover %L1").arg(fstr[freq]);
        s2 = tr("Base %L1").arg(fstr[freq]);
        if (w >= h) { // horizontal
            drawSatellites(c, w / 2, h, 0, 0, 0, freq);
            drawSatellites(c, w / 2, h, w / 2, 0, 1, freq);
            drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
            drawText(c, x + w / 2, 1, s2, Qt::darkGray, 1, 2);
        } else { // vertical
            drawSatellites(c, w, h / 2, 0, 0, 0, freq);
            drawSatellites(c, w, h / 2, 0, h / 2, 1, freq);
            drawText(c, x, 1, s1, Qt::darkGray, 1, 2);
            drawText(c, x, h / 2 + 1, s2, Qt::darkGray, 1, 2);
        }
    } else if (type == 5) { // baseline plot
        drawBaseline(c, id, w, h);
        drawText(c, x, 1, tr("Baseline"), Qt::darkGray, 1, 2);
    } else if (type == 6) { // track plot
        drawTrack(c, id, &buffer);
        drawText(c, x, 3, tr("Gnd Trk"), Qt::darkGray, 1, 2);
    }
    plot->setPixmap(buffer);
}
// update solution plot ------------------------------------------------------
void MainWindow::updatePlot()
{
    if (ui->panelDisplay1->isVisible()) {
        drawSolutionPlot(ui->lblDisplay1, plotType[0], frequencyType[0]);
    }
    if (ui->panelDisplay2->isVisible()) {
        drawSolutionPlot(ui->lblDisplay2, plotType[1], frequencyType[1]);
    }
    if (ui->panelDisplay3->isVisible()) {
        drawSolutionPlot(ui->lblDisplay3, plotType[2], frequencyType[2]);
    }
    if (ui->panelDisplay4->isVisible()) {
        drawSolutionPlot(ui->lblDisplay4, plotType[3], frequencyType[3]);
    }
}
// snr color ----------------------------------------------------------------
QColor MainWindow::snrColor(int snr)
{
    QColor color[] = {Qt::darkGreen, Color::Orange, Color::Fuchsia, Qt::blue, Qt::red, Qt::darkGray};
    uint32_t r1, g1, b1;
    QColor c1, c2;
    double a;
    int i;

    if (snr < 25) return color[5];
    if (snr < 27) return color[4];
    if (snr > 47) return color[0];
    a = (snr - 27.5) / 5.0;
    i = static_cast<int>(a);
    a -= i;
    c1 = color[3 - i];
    c2 = color[4 - i];
    r1 = static_cast<uint32_t>(a * c1.red() + (1.0 - a) * c2.red()) & 0xFF;
    g1 = static_cast<uint32_t>(a * c1.green() + (1.0 - a) * c2.green()) & 0xFF;
    b1 = static_cast<uint32_t>(a * c1.blue() + (1.0 - a) * c2.blue()) & 0xFF;

    return QColor(r1, g1, b1);
}
// draw snr plot ------------------------------------------------------------
void MainWindow::drawSnr(QPainter &c, int w, int h, int x0, int y0,
             int index, int freq)
{
    static const QColor color[] = {
        QColor(0  , 128,   0), QColor(128, 128,  0), QColor(0xA0,   0, 0xA0),
        QColor(0  ,   0, 128), QColor(128,   0,  0), QColor(   0, 128,  128),
        QColor(128, 128, 128)
    };
    static const QColor color_sys[] = {Qt::darkGreen, Color::Orange, Color::Fuchsia, Qt::blue, Qt::red, Color::Teal, Qt::darkGray};
    int i, j, snrIdx, sysIdx, numSystems, x1, y1, height, offset, topMargin, bottomMargin, hh, barDistance, barWidth, snr[NFREQ + 1], sysMask[7] = {0};
    char id[8], sys[] = "GREJCS", *q;

    trace(4, "drawSnr: w=%d h=%d x0=%d y0=%d index=%d freq=%d\n", w, h, x0, y0, index, freq);

    QFontMetrics fm(optDialog->panelFont);
    topMargin = fm.height();
    bottomMargin = fm.height();
    y0 += topMargin;
    hh = h - topMargin - bottomMargin;

    // draw horizontal lines
    c.setPen(Color::Silver);
    for (snr[0] = MINSNR + 10; snr[0] < MAXSNR; snr[0] += 10) {
        y1 = y0 + hh - (snr[0] - MINSNR) * hh / (MAXSNR - MINSNR);
        c.drawLine(x0 + 2, y1, x0 + w - 20, y1);
        drawText(c, x0 + w - 4, y1, QLocale().toString(snr[0]), Qt::darkGray, 2, 0);
    }

    // draw outer box
    y1 = y0 + hh;
    QRect b(x0 + 2, y0, w - 2, hh);
    c.setBrush(Qt::NoBrush);
    c.setPen(Qt::darkGray);
    c.drawRect(b);

    // draw SNR bars
    for (i = 0; i < numSatellites[index] && i < MAXSAT; i++) {
        barDistance = (w - 16) / numSatellites[index];
        barWidth = barDistance - 2 < 8 ? barDistance - 2 : 8;
        x1 = x0 + i * barDistance + barDistance / 2;
        satno2id(satellites[index][i], id);
        sysIdx = (q = strchr(sys, id[0])) ? (int)(q - sys) : 6;

        for (j = snr[0] = 0; j < NFREQ; j++) {
            snr[j + 1] = satellitesSNR[index][i][j];
            if ((freq && freq == j + 1) || ((!freq || freq > NFREQ) && snr[j + 1] > snr[0]))
                snr[0] = snr[j + 1];  // store max snr
        }
        for (j = 0; j < NFREQ + 2; j++) {
            snrIdx = j < NFREQ + 1 ? j : 0;
            offset = j < NFREQ + 1 ? 0 : 2;
            if (snr[snrIdx] > 0) height = (snr[snrIdx] - MINSNR) * hh / (MAXSNR - MINSNR);
            else height = offset;
            height = height > y1 - 2 ? y1 - 2 : (height < 0 ? 0 : height);  // limit bar from going negative or too high

            QRect r1(x1, y1, barWidth, -height);
            if (j == 0) {  // filled bar
                c.setBrush(QBrush(freq < NFREQ + 1 ? snrColor(snr[snrIdx]) : color_sys[sysIdx], Qt::SolidPattern));
                if (!validSatellites[index][i])
                    c.setBrush(QBrush(QColor(QColorConstants::LightGray), Qt::SolidPattern));
            } else {  // outline only
                c.setPen(j < NFREQ + 1 ? QColor(QColorConstants::LightGray) : Qt::darkGray);
                c.setBrush(Qt::NoBrush);
            }
            c.drawRect(r1);
        }
        // draw satellite id
        drawText(c, x1 + barWidth / 2, y1, id + 1, color[sysIdx], 0, 2);
        sysMask[sysIdx] = 1;
    }
    for (i = numSystems = 0; i < 7; i++)
        if (sysMask[i]) numSystems++;

    // draw indicators for used navigation systems
    for (i = j = 0; i < 7; i++) {
        if (!sysMask[i]) continue;
        sprintf(id, "%c", sys[i]);
        drawText(c, x0 + w - topMargin*3/2 + fm.averageCharWidth()*9/8 * (j++ - numSystems), y0, id, color[i], 0, 2);
    }
}
// draw satellites in skyplot -----------------------------------------------
void MainWindow::drawSatellites(QPainter &c, int w, int h, int x0, int y0,
                                int index, int freq)
{
    static const QColor color_sys[] = {
        Qt::darkGreen, Color::Orange, Color::Fuchsia, Qt::blue, Qt::red, Qt::darkCyan, Qt::darkGray
    };
    QColor color_text;
    QPoint p(w / 2, h / 2);
    double r = qMin(w * 0.95, h * 0.95) / 2, azel[MAXSAT * 2], dop[4];
    int i, j, k, sysIdx, radius, x[MAXSAT], y[MAXSAT], snr[NFREQ + 1], nsats = 0;
    char id[8], sys[] = "GREJCIS", *q;

    trace(4, "drawSatellites: w=%d h=%d index=%d freq=%d\n", w, h, index, freq);

    drawSky(c, w, h, x0, y0);  // draw background

    // draw satellites
    for (i = 0, k = numSatellites[index] - 1; i < numSatellites[index] && i < MAXSAT; i++, k--) {
        if (satellitesElevation[index][k] <= 0.0) continue;
        for (j = snr[0] = 0; j < NFREQ; j++) {
            snr[j + 1] = satellitesSNR[index][k][j];
            if ((freq && freq == j + 1) || ((!freq || freq > NFREQ) && snr[j + 1] > snr[0])) {
                snr[0] = snr[j + 1]; // max snr
            }
        }
        if (validSatellites[index][k] && (freq > NFREQ || snr[freq] > 0)) {
            azel[nsats * 2] = satellitesAzimuth[index][k];
            azel[nsats * 2 + 1] = satellitesElevation[index][k];
            nsats++;
        }
        satno2id(satellites[index][k], id);
        sysIdx = (q = strchr(sys, id[0])) ? (int)(q - sys) : 6;
        x[i] = static_cast<int>(p.x() + r * (90 - satellitesElevation[index][k] * R2D) / 90 * sin(satellitesAzimuth[index][k])) + x0;
        y[i] = static_cast<int>(p.y() - r * (90 - satellitesElevation[index][k] * R2D) / 90 * cos(satellitesAzimuth[index][k])) + y0;
        radius = QFontMetrics(optDialog->panelFont).height();

        c.setBrush(!validSatellites[index][k] ? Color::Silver :
                        (freq < NFREQ ? snrColor(snr[freq]) : color_sys[sysIdx]));
        c.setPen(Qt::darkGray);
        color_text = Qt::white;
        if (freq < NFREQ + 1 && snr[freq] <= 0) {
            c.setPen(Color::Silver);
            color_text = Color::Silver;
        }
        // draw satllites
        c.drawEllipse(x[i] - radius, y[i] - radius, 2 * radius + 1, 2 * radius + 1);
        drawText(c, x[i], y[i], id, color_text, 0, 0);
    }

    // draw annotations
    dops(nsats, azel, 0.0, dop);
    drawText(c, x0 + 3, y0 + h, tr("# Sat: %L1/%L2").arg(nsats, 2).arg(numSatellites[index], 2), Qt::darkGray, 1, 1);
    drawText(c, x0 + w - 3, y0 + h, tr("GDOP: %L1").arg(dop[0], 0, 'f', 1), Qt::darkGray, 2, 1);
}
// draw baseline plot -------------------------------------------------------
void MainWindow::drawBaseline(QPainter &c, int id, int w, int h)
{
    static const QColor colors[] = {Color::Silver, Qt::darkGreen, Color::Orange, Color::Fuchsia, Qt::blue, Qt::red, Color::Teal};
    static const QString directions = tr("NESW");
    QPoint center(w / 2, h / 2), p1, p2, pp;
    double radius = qMin(w * 0.95, h * 0.95) / 2;
    double *rrover = solutionRover + solutionsCurrent * 3;
    double *rbase = solutionReference + solutionsCurrent * 3;
    double baseline[3] = {0}, pos[3], enu[3], len = 0.0, pitch = 0.0, yaw = 0.0;
    double cp, q, az = 0.0;
    QColor col = Qt::white;
    int i, d1 = 10, d2 = 16, d3 = 10, cy = 0, sy = 0, cya = 0, sya = 0, a, x1, x2, y1, y2, radius2, digit;

    trace(4, "drawBaseline: w=%d h=%d\n", w, h);

    if (PMODE_DGPS <= optDialog->processingOptions.mode && optDialog->processingOptions.mode <= PMODE_FIXED) {
        col = (rtksvr->state && solutionStatus[solutionsCurrent] && solutionCurrentStatus) ? colors[solutionStatus[solutionsCurrent]] : Qt::white;

        if (norm(rrover, 3) > 0.0 && norm(rbase, 3) > 0.0)
            for (i = 0; i < 3; i++)
                baseline[i] = rrover[i] - rbase[i];

        if ((len = norm(baseline, 3)) > 0.0) {
            ecef2pos(rbase, pos);
            ecef2enu(pos, baseline, enu);
            pitch = asin(enu[2] / len);
            yaw = atan2(enu[0], enu[1]);
            if (yaw < 0.0) yaw += 2.0 * PI;
            if (baselineMode[id]) az = yaw;
        }
    }
    if (len >= MIN_BASELINE_LEN) {
        cp = cos(pitch);
        cy = static_cast<int>((radius - d1 - d2 / 2) * cp * cos(yaw - az));
        sy = static_cast<int>((radius - d1 - d2 / 2) * cp * sin(yaw - az));
        cya = static_cast<int>(((radius - d1 - d2 / 2) * cp - d2 / 2 - 4) * cos(yaw - az));
        sya = static_cast<int>(((radius - d1 - d2 / 2) * cp - d2 / 2 - 4) * sin(yaw - az));
    }
    p1 = QPoint(center.x() - sy, center.y() + cy);       // base
    p2 = QPoint(center.x() + sy, center.y() - cy);       // rover

    // draw two outer circles
    c.setPen(Qt::darkGray);
    c.drawEllipse(center.x() - radius, center.y() - radius, 2 * radius + 1, 2 * radius + 1);
    radius2 = static_cast<int>(radius - d1 / 2);
    c.drawEllipse(center.x() - radius2, center.y() - radius2, 2 * radius2 + 1, 2 * radius2 + 1);
    for (a = 0; a < 360; a += 5) {  // draw ticks
        q = a % 90 == 0 ? 0 : (a % 30 == 0 ? radius - d1 * 3 : (a % 10 == 0 ? radius - d1 * 2 : radius - d1));
        x1 = static_cast<int>(radius * sin(a * D2R - az));
        y1 = static_cast<int>(radius * cos(a * D2R - az));
        x2 = static_cast<int>(q * sin(a * D2R - az));
        y2 = static_cast<int>(q * cos(a * D2R - az));
        c.setPen(Color::Silver);
        c.drawLine(center.x() + x1, center.y() - y1, center.x() + x2, center.y() - y2);
        c.setBrush(Qt::white);
        if (a % 90 == 0)  // draw label
            drawText(c, center.x() + x1, center.y() - y1, directions[a / 90], Qt::darkGray, 0, 0);
        if (a == 0) {  // raw arrow to north
            x1 = static_cast<int>((radius - d1 * 3 / 2) * sin(a * D2R - az));
            y1 = static_cast<int>((radius - d1 * 3 / 2) * cos(a * D2R - az));
            drawArrow(c, center.x() + x1, center.y() - y1, d3, -static_cast<int>(az * R2D), Color::Silver);
        }
    }

    pp = pitch < 0.0 ? p2 : p1;
    c.setPen(Color::Silver);
    c.drawLine(center, pp);
    if (pitch < 0.0) {
        c.setBrush(Qt::white);
        c.drawEllipse(pp.x() - d2 / 2, pp.y() - d2 / 2, d2 + 1, d2 + 1);
        drawArrow(c, center.x() + sya, center.y() - cya, d3, static_cast<int>((yaw - az) * R2D), Color::Silver);
    }
    c.setBrush(col);
    c.drawEllipse(pp.x() - d2 / 2 + 2, pp.y() - d2 / 2 + 2, d2 - 1, d2 - 1);

    pp = pitch >= 0.0 ? p2 : p1;
    c.setPen(Qt::darkGray);
    c.drawLine(center, pp);
    if (pitch >= 0.0) {
        c.setBrush(Qt::white);
        c.drawEllipse(pp.x() - d2 / 2, pp.y() - d2 / 2, d2, d2);
        drawArrow(c, center.x() + sya, center.y() - cya, d3, static_cast<int>((yaw - az) * R2D), Qt::darkGray);
    }
    c.setBrush(col);
    c.drawEllipse(pp.x() - d2 / 2 + 2, pp.y() - d2 / 2 + 2, d2 - 4, d2 - 4);

    // draw annotations
    c.setBrush(Qt::white);
    digit = len < 1000.0 ? 3 : (len < 10000.0 ? 2 : (len < 100000.0 ? 1 : 0));
    drawText(c, center.x(), center.y(), tr("%L1 m").arg(len, 0, 'f', digit), Qt::darkGray, 0, 0);
    drawText(c, 3, h, tr("Y: %L1°").arg(yaw * R2D, 0, 'f', 1), Qt::darkGray, 1, 1);
    drawText(c, w - 3, h, tr("P: %L1°").arg(pitch * R2D, 0, 'f', 1), Qt::darkGray, 2, 1);
}
// draw track plot ----------------------------------------------------------
void MainWindow::drawTrack(QPainter &c, int id, QPaintDevice *plot)
{
    static const QColor mcolor[] = {Color::Silver, Qt::darkGreen, Color::Orange, Color::Fuchsia, Qt::blue, Qt::red, Color::Teal};
    QColor *color;
    Graph *graph = new Graph(plot);
    QPoint p1, p2;
    QString label;
    static const double scale[] = {
        0.00021, 0.00047, 0.001, 0.0021, 0.0047, 0.01, 0.021, 0.047, 0.1,   0.21,   0.47,
        1.0,	 2.1,	  4.7,	 10.0,	 21.0,	 47.0, 100.0, 210.0, 470.0, 1000.0, 2100.0,4700.0,
        10000.0
    };
    double *x, *y, xt, yt, sx, sy, ref[3], pos[3], dr[3], enu[3];
    int i, j, currentPointNo, numPoints = 0, type, scl;

    trace(3, "drawTrack\n");

    type = id == 0 ? trackType[0] : trackType[1];
    scl = id == 0 ? trackScale[0] : trackScale[1];

    x = new double[optDialog->solutionBufferSize];
    y = new double[optDialog->solutionBufferSize];
    color = new QColor[optDialog->solutionBufferSize];

    if (norm(trackOrigin, 3) < 1E-6) {
        if (norm(solutionReference + solutionsCurrent * 3, 3) > 1E-6)
            matcpy(trackOrigin, solutionReference + solutionsCurrent * 3, 3, 1);
        else
            matcpy(trackOrigin, solutionRover + solutionsCurrent * 3, 3, 1);
    }
    if (norm(solutionReference + solutionsCurrent * 3, 3) > 1E-6)
        matcpy(ref, solutionReference + solutionsCurrent * 3, 3, 1);
    else
        matcpy(ref, trackOrigin, 3, 1);
    ecef2pos(ref, pos);

    for (i = currentPointNo = solutionsStart; i != solutionsEnd; ) {
        for (j = 0; j < 3; j++) dr[j] = solutionRover[j + i * 3] - ref[j];
        if (i == solutionsCurrent) currentPointNo = numPoints;
        ecef2enu(pos, dr, enu);
        x[numPoints] = enu[0];
        y[numPoints] = enu[1];
        color[numPoints] = mcolor[solutionStatus[i]];
        numPoints++;
        if (++i >= optDialog->solutionBufferSize) i = 0;
    }
    graph->setSize(plot->width(), plot->height());
    graph->setScale(scale[scl], scale[scl]);
    graph->color[1] = Color::Silver;

    // center to current position
    if (numPoints > 0)
        graph->setCenter(x[currentPointNo], y[currentPointNo]);

    if (type == 1) {
        graph->xLabelPosition = Graph::LabelPosition::Axis;
        graph->yLabelPosition = Graph::LabelPosition::Axis;
        graph->drawCircles(c, 0);
    } else {
        graph->xLabelPosition = Graph::LabelPosition::Inner;
        graph->yLabelPosition = Graph::LabelPosition::InnerRot;
        graph->drawAxis(c, 0, 0);
    }
    graph->drawPoly(c, x, y, numPoints, Color::Silver, 0); // draw lines through all points
    graph->drawMarks(c, x, y, color, numPoints, 0, 3, 0); // draw markers

    // highlight current position
    if (numPoints > 0) {
        graph->toPoint(x[currentPointNo], y[currentPointNo], p1);
        graph->drawMark(c, p1, Graph::MarkerTypes::Dot, Qt::white, 18, 0);
        graph->drawMark(c, p1, Graph::MarkerTypes::Circle, rtksvr->state ? Qt::black : Qt::darkGray, 16, 0);
        graph->drawMark(c, p1, Graph::MarkerTypes::Plus, rtksvr->state ? Qt::black : Qt::darkGray, 20, 0);
        graph->drawMark(c, p1, Graph::MarkerTypes::Dot, rtksvr->state ? Qt::black : Qt::darkGray, 12, 0);
        graph->drawMark(c, p1, Graph::MarkerTypes::Dot, rtksvr->state ? color[currentPointNo] : Qt::white, 10, 0);
    }

    // draw scale
    graph->getExtent(p1, p2);
    graph->getTick(xt, yt);
    graph->getScale(sx, sy);
    p2.rx() -= 35;
    p2.ry() -= 12;
    graph->drawMark(c, p2, Graph::MarkerTypes::HScale, Qt::darkGray, static_cast<int>(xt / sx + 0.5), 0);

    // scale text
    p2.ry() -= 2;
    if (xt < 0.01)
        label = tr("%L1 mm").arg(xt * 1000.0, 0, 'f', 0);
    else if (xt < 1.0)
        label = tr("%L1 cm").arg(xt * 100.0, 0, 'f', 0);
    else if (xt < 1000.0)
        label = tr("%L1 m").arg(xt, 0, 'f', 0);
    else
        label = tr("%L1 km").arg(xt / 1000.0, 0, 'f', 0);
    graph->drawText(c, p2, label, Qt::darkGray, Qt::white, 0, 1, 0);

    // reference position
    if (norm(ref, 3) > 1E-6) {
        p1.rx() += 2;
        p1.ry() = p2.y() + 11;
        label = tr("%L1° %L2°").arg(pos[0] * R2D, 0, 'f', 9).arg(pos[1] * R2D, 0, 'f', 9);
        graph->drawText(c, p1, label, Qt::darkGray, Qt::white, 1, 1, 0);
    }

    delete graph;
    delete[] x;
    delete[] y;
    delete[] color;
}
// draw skyplot -------------------------------------------------------------
void MainWindow::drawSky(QPainter &c, int w, int h, int x0, int y0)
{
    static const QString label = tr("NESW");
    QPoint p(x0 + w / 2, y0 + h / 2);
    double radius = qMin(w * 0.95, h * 0.95) / 2;
    int a, e, d, x, y;

    c.setBrush(Qt::white);
    // draw elevation circles every 30 deg
    for (e = 0; e < 90; e += 30) {
        d = static_cast<int>(radius * (90 - e) / 90);
        c.setPen(e == 0 ? Qt::darkGray : Color::Silver);  // outline in gray, other lines in silver
        c.drawEllipse(p.x() - d, p.y() - d, 2 * d + 1, 2 * d + 1);
    }
    // draw azimuth lines from center outwards every 45 deg
    for (a = 0; a < 360; a += 45) {
        x = static_cast<int>(radius * sin(a * D2R));
        y = static_cast<int>(radius * cos(a * D2R));
        c.setPen(Color::Silver);
        c.drawLine(p.x(), p.y(), p.x() + x, p.y() - y);
        if (a % 90 == 0)  // draw labels every 90 deg
            drawText(c, p.x() + x, p.y() - y, label[a / 90], Qt::darkGray, 0, 0);
    }
}
// draw text ----------------------------------------------------------------
void MainWindow::drawText(QPainter &c, int x, int y, const QString &str,
              const QColor &color, int ha, int va)
{
    // ha  = horizontal alignment (0: center, 1: left,   2: right)
    // va  = vertical alignment   (0: center, 1: bottom, 2: top  )
    int flags = 0;

    switch (ha) {
        case 0: flags |= Qt::AlignHCenter; break;
        case 1: flags |= Qt::AlignLeft; break;
        case 2: flags |= Qt::AlignRight; break;
    }
    switch (va) {
        case 0: flags |= Qt::AlignVCenter; break;
        case 1: flags |= Qt::AlignBottom; break;
        case 2: flags |= Qt::AlignTop; break;
    }
    QFont old_font = c.font();
    c.setFont(optDialog->panelFont);

    QPen pen = c.pen();
    c.setBrush(Qt::NoBrush);
    pen.setColor(color);
    c.setPen(pen);

    QRect off = c.boundingRect(QRect(), flags, str);

    c.translate(x, y);
    c.drawText(off, str);
    c.translate(-x, -y);
    c.setFont(old_font);
}
// draw arrow ---------------------------------------------------------------
void MainWindow::drawArrow(QPainter &c, int x, int y, int siz,
               int ang, const QColor &color)
{
    QPoint p1[4], p2[4];
    int i;

    // create arrow head
    p1[0] = QPoint(0, siz / 2);
    p1[1] = QPoint(siz / 2, -siz / 2);
    p1[2] = QPoint(-siz / 2, -siz / 2);
    p1[3] = QPoint(0, siz / 2);

    // rotate points
    for (i = 0; i < 4; i++) {
        p2[i] = QPoint(x + static_cast<int>(p1[i].x() * cos(-ang * D2R) - p1[i].y() * sin(-ang * D2R) + 0.5),
                       y - static_cast<int>(p1[i].x() * sin(-ang * D2R) + p1[i].y() * cos(-ang * D2R) + 0.5));
    }
    c.setBrush(QBrush(color, Qt::SolidPattern));
    c.setPen(color);
    c.drawPolygon(p2, 4);
}
// open monitor port --------------------------------------------------------
void MainWindow::openMonitorPort(int port)
{
    int i;

    if (port <= 0) return;

    trace(3, "openMonitorPort: port=%d\n", port);

    for (i = 0; i <= MAX_PORT_OFFSET; i++) {
        if (stropen(&monistr, STR_TCPSVR, STR_MODE_RW, qPrintable(QString(":%1").arg(port + i)))) {
            strsettimeout(&monistr, optDialog->timeoutTime, optDialog->reconnectTime);
            if (i > 0)
                setWindowTitle(tr("%1 ver. %2 (%3)").arg(PRGNAME, VER_RTKLIB).arg(i + 1));
            monitorPortOpen = port + i;
            return;
        }
    }
    QMessageBox::critical(this, tr("Error"), tr("Could not open any monitor port betwenn %L1-%L2").arg(port).arg(port + MAX_PORT_OFFSET));
    monitorPortOpen = 0;
}
// initialize solution buffer -----------------------------------------------
void MainWindow::initSolutionBuffer()
{
    double ep[] = {2000, 1, 1, 0, 0, 0};
    int i, j;

    trace(3, "initSolutionBuffer\n");

    delete [] timeStamps;
    delete [] solutionStatus;
    delete [] numValidSatellites;
    delete [] solutionRover;
    delete [] solutionReference;
    delete [] solutionQr;
    delete [] velocityRover;
    delete [] ages;
    delete [] ratioAR;

    if (optDialog->solutionBufferSize <= 0) optDialog->solutionBufferSize = 1;

    timeStamps = new gtime_t[optDialog->solutionBufferSize];
    solutionStatus = new int[optDialog->solutionBufferSize];
    numValidSatellites = new int[optDialog->solutionBufferSize];
    solutionRover = new double[optDialog->solutionBufferSize * 3];
    solutionReference = new double[optDialog->solutionBufferSize * 3];
    velocityRover = new double[optDialog->solutionBufferSize * 3];
    solutionQr = new double[optDialog->solutionBufferSize * 9];
    ages = new double[optDialog->solutionBufferSize];
    ratioAR = new double[optDialog->solutionBufferSize];

    solutionsCurrent = solutionsStart = solutionsEnd = 0;

    for (i = 0; i < optDialog->solutionBufferSize; i++) {
        timeStamps[i] = epoch2time(ep);
        solutionStatus[i] = numValidSatellites[i] = 0;
        for (j = 0; j < 3; j++) solutionRover[j + i * 3] = solutionReference[j + i * 3] = velocityRover[j + i * 3] = 0.0;
        for (j = 0; j < 9; j++) solutionQr[j + i * 9] = 0.0;
        ages[i] = ratioAR[i] = 0.0;
    }

    ui->sBSolution->setMaximum(0);
    ui->sBSolution->setValue(0);
}
// save log file ------------------------------------------------------------
void MainWindow::saveLogs()
{
    QString fileName;
    int posf[] = {SOLF_LLH, SOLF_LLH, SOLF_XYZ, SOLF_ENU, SOLF_ENU, SOLF_LLH};
    solopt_t opt;
    sol_t sol = {};
    double ep[6], pos[3];
    QString fileTemplate;
    int i;

    trace(3, "saveLogs\n");

    time2epoch(timeget(), ep);
    fileTemplate = QString("rtk_%1%2%3%4%5%6.txt")
               .arg(ep[0], 4, 'f', 0, QChar('0')).arg(ep[1], 2, 'f', 0, QChar('0')).arg(ep[2], 2, 'f', 0, QChar('0'))
               .arg(ep[3], 2, 'f', 0, QChar('0')).arg(ep[4], 2, 'f', 0, QChar('0')).arg(ep[5], 2, 'f', 0, QChar('0'));

    fileName = QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, QString(), fileTemplate));
    if (fileName.isEmpty()) return;
    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, tr("Error"), tr("Could not open log file: %1").arg(fileName));
        return;
    }
    QTextStream str(&file);

    opt = optDialog->solutionOptions;
    opt.posf = posf[solutionType];
    if (optDialog->solutionOptions.outhead) {
        QString data;

        data = tr("%% program : %1 ver. %2 %3\n").arg(PRGNAME, VER_RTKLIB, PATCH_LEVEL);
        str << data;
        if (optDialog->processingOptions.mode >= PMODE_DGPS && optDialog->processingOptions.mode <= PMODE_FIXED) {
            ecef2pos(optDialog->processingOptions.rb, pos);
            data = QStringLiteral("%% ref pos   :%1 %2 %3\n").arg(pos[0] * R2D, 13, 'f', 9)
                   .arg(pos[1] * R2D, 14, 'f', 9).arg(pos[2], 10, 'f', 4);
            str << data;
        }
        file.write("%%\n");
    }
    FILE *f = fdopen(file.handle(), "w");
    outsolhead(f, &opt);

    for (i = solutionsStart; i != solutionsEnd; ) {
        sol.time = timeStamps[i];
        matcpy(sol.rr, solutionRover + i * 3, 3, 1);
        sol.stat = solutionStatus[i];
        sol.ns = numValidSatellites[i];
        sol.ratio = ratioAR[i];
        sol.age = ages[i];
        outsol(f, &sol, solutionReference + i * 3, &opt);
        if (++i >= optDialog->solutionBufferSize) i = 0;
    }
}
// load navigation data -----------------------------------------------------
void MainWindow::loadNavigation(nav_t *nav)
{
    QSettings settings(iniFile, QSettings::IniFormat);
    QString str;
    eph_t eph0;
    char buff[2049], *p;
    long toe_time,toc_time,ttr_time;
    int i;

    trace(3, "loadNavigation\n");

    memset(&eph0, 0, sizeof(eph_t));

    for (i = 0; i < 2 * MAXSAT; i++) {
        if ((str = settings.value(QString("navi/eph_%1").arg(i, 2)).toString()).isEmpty()) continue;
        nav->eph[i] = eph0;
        strncpy(buff, qPrintable(str), 2047);
        if (!(p = strchr(buff, ','))) continue;
        *p = '\0';
        if (!(nav->eph[i].sat = satid2no(buff))) continue;
        sscanf(p + 1, "%d,%d,%d,%d,%ld,%ld,%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf",
               &nav->eph[i].iode,
               &nav->eph[i].iodc,
               &nav->eph[i].sva,
               &nav->eph[i].svh,
               &toe_time,
               &toc_time,
               &ttr_time,
               &nav->eph[i].A,
               &nav->eph[i].e,
               &nav->eph[i].i0,
               &nav->eph[i].OMG0,
               &nav->eph[i].omg,
               &nav->eph[i].M0,
               &nav->eph[i].deln,
               &nav->eph[i].OMGd,
               &nav->eph[i].idot,
               &nav->eph[i].crc,
               &nav->eph[i].crs,
               &nav->eph[i].cuc,
               &nav->eph[i].cus,
               &nav->eph[i].cic,
               &nav->eph[i].cis,
               &nav->eph[i].toes,
               &nav->eph[i].fit,
               &nav->eph[i].f0,
               &nav->eph[i].f1,
               &nav->eph[i].f2,
               &nav->eph[i].tgd[0],
               &nav->eph[i].code,
               &nav->eph[i].flag,
               &nav->eph[i].tgd[1]);
        nav->eph[i].toe.time = toe_time;
        nav->eph[i].toc.time = toc_time;
        nav->eph[i].ttr.time = ttr_time;
    }
    // read ionospheric parameters
    str = settings.value("navi/ion", "").toString();
    QStringList tokens = str.split(",");
    for (i = 0; i < 8; i++) nav->ion_gps[i] = 0.0;
    for (i = 0; (i < 8) && (i < tokens.size()); i++) nav->ion_gps[i] = tokens.at(i).toDouble();

    // read time offsets
    str = settings.value("navi/utc", "").toString();
    tokens = str.split(",");
    for (i = 0; i < 8; i++) nav->utc_gps[i] = 0.0;
    for (i = 0; (i < 8) && (i < tokens.size()); i++) nav->utc_gps[i] = tokens.at(i).toDouble();
}
// save navigation data -----------------------------------------------------
void MainWindow::saveNavigation(nav_t *nav)
{
    QSettings settings(iniFile, QSettings::IniFormat);
    QString str;
    char id[8];
    int i;

    trace(3, "saveNavigation\n");

    if (nav == NULL) return;

    for (i = 0; i < nav->n; i++) {
        if (nav->eph[i].ttr.time == 0) continue;
        str = "";
        satno2id(nav->eph[i].sat, id);
        str = str + id + ",";
        str = str + QString("%1,").arg(nav->eph[i].iode);
        str = str + QString("%1,").arg(nav->eph[i].iodc);
        str = str + QString("%1,").arg(nav->eph[i].sva);
        str = str + QString("%1,").arg(nav->eph[i].svh);
        str = str + QString("%1,").arg(static_cast<int>(nav->eph[i].toe.time));
        str = str + QString("%1,").arg(static_cast<int>(nav->eph[i].toc.time));
        str = str + QString("%1,").arg(static_cast<int>(nav->eph[i].ttr.time));
        str = str + QString("%1,").arg(nav->eph[i].A, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].e, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].i0, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].OMG0, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].omg, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].M0, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].deln, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].OMGd, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].idot, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].crc, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].crs, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].cuc, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].cus, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].cic, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].cis, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].toes, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].fit, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].f0, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].f1, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].f2, 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].tgd[0], 0, 'E', 14);
        str = str + QString("%1,").arg(nav->eph[i].code);
        str = str + QString("%1,").arg(nav->eph[i].flag);
        str = str + QString("%1,").arg(nav->eph[i].tgd[1],0,'E',14);
        settings.setValue(QString("navi/eph_%1").arg(i, 2), str);
    }
    str = "";
    for (i = 0; i < 8; i++) str = str + QString("%1,").arg(nav->ion_gps[i], 0, 'E', 14);
    settings.setValue("navi/ion", str);

    str = "";
    for (i = 0; i < 8; i++) str = str + QString("%1,").arg(nav->utc_gps[i], 0, 'E', 14);
    settings.setValue("navi/utc", str);
}
// set tray icon ------------------------------------------------------------
void MainWindow::setTrayIcon(int index)
{
    QPixmap pix(":/buttons/navi");

    systemTray->setIcon(QIcon(pix.copy(index * 16, 0, 16, 16)));
}
// load option from ini file ------------------------------------------------
void MainWindow::loadOptions()
{
    QSettings settings(iniFile, QSettings::IniFormat);
    int i, j, no, strno[] = { 0, 1, 6, 4, 5, 7, 2, 3, 8 };

    trace(3, "loadOptions\n");

    for (i = 0; i < MAXSTRRTK; i++) {
        no = strno[i];
        streamEnabled[i] = settings.value(QString("stream/streamc%1").arg(no), 0).toInt();
        streamType[i] = settings.value(QString("stream/stream%1").arg(no), 0).toInt();
        inputFormat[i] = settings.value(QString("stream/format%1").arg(no), 0).toInt();
        for (j = 0; j < 4; j++)
            paths[i][j] = settings.value(QString("stream/path_%1_%2").arg(no).arg(j), "").toString();
    }

    for (i = 0; i < 3; i++)
        receiverOptions [i] = settings.value(QString("stream/rcvopt%1").arg(i + 1), "").toString();

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            commands[i][j] = settings.value(QString("serial/cmd_%1_%2").arg(i).arg(j), "").toString();
            commandEnabled[i][j] = settings.value(QString("serial/cmdena_%1_%2").arg(i).arg(j), 0).toInt();
            commands[i][j].replace("@@", "\r\n");
        }

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            commandsTcp[i][j] = settings.value(QString("tcpip/cmd_%1_%2").arg(i).arg(j), "").toString();
            commandEnableTcp[i][j] = settings.value(QString("tcpip/cmdena_%1_%2").arg(i).arg(j), 0).toInt();
            commandsTcp[i][j].replace("@@", "\r\n");
        }

    optDialog->loadOptions(settings);

    inputTimeTag = settings.value("setting/intimetag", 0).toInt();
    inputTimeSpeed = settings.value("setting/intimespeed", "x1").toString();
    inputTimeStart = settings.value("setting/intimestart", 0).toDouble();
    inputTimeTag64bit = settings.value("setting/intime64bit", "0").toInt();
    outputTimeTag = settings.value("setting/outtimetag", 0).toInt();
    outputSwapInterval = settings.value("setting/outswapinterval", "").toString();
    logTimeTag = settings.value("setting/logtimetag", 0).toInt();
    logSwapInterval = settings.value("setting/logswapinterval", "").toString();
    resetCommand = settings.value("setting/resetcmd", "").toString();
    maxBaseLine = settings.value("setting/maxbl", 10.0).toDouble();

    timeSystem = settings.value("setting/timesys", 0).toInt();
    solutionType = settings.value("setting/soltype", 0).toInt();
    plotType[0] = settings.value("setting/plottype", 0).toInt();
    plotType[1] = settings.value("setting/plottype2", 0).toInt();
    plotType[2] = settings.value("setting/plottype3", 0).toInt();
    plotType[3] = settings.value("setting/plottype4", 0).toInt();
    panelMode = settings.value("setting/panelmode", 0).toInt();
    trackType[0] = settings.value("setting/trktype1", 0).toInt();
    trackType[1] = settings.value("setting/trktype2", 0).toInt();
    trackType[2] = settings.value("setting/trktype3", 0).toInt();
    trackType[3] = settings.value("setting/trktype4", 0).toInt();
    trackScale[0] = settings.value("setting/trkscale1", 5).toInt();
    trackScale[1] = settings.value("setting/trkscale2", 5).toInt();
    trackScale[2] = settings.value("setting/trkscale3", 5).toInt();
    trackScale[3] = settings.value("setting/trkscale4", 5).toInt();
    frequencyType[0] = settings.value("setting/freqtype1", 0).toInt();
    frequencyType[1] = settings.value("setting/freqtype2", 0).toInt();
    frequencyType[2] = settings.value("setting/freqtype3", 0).toInt();
    frequencyType[3] = settings.value("setting/freqtype4", 0).toInt();
    baselineMode[0] = settings.value("setting/blmode1", 0).toInt();
    baselineMode[1] = settings.value("setting/blmode2", 0).toInt();
    baselineMode[2] = settings.value("setting/blmode3", 0).toInt();
    baselineMode[3] = settings.value("setting/blmode4", 0).toInt();
    markerName = settings.value("setting/markername", "").toString();
    markerComment = settings.value("setting/markercomment", "").toString();

    nmeaRequestType = settings.value("setting/nmeareq", 0).toInt();
    nmeaPosition[0] = settings.value("setting/nmeapos1", 0.0).toDouble();
    nmeaPosition[1] = settings.value("setting/nmeapos2", 0.0).toDouble();
    nmeaPosition[2] = settings.value("setting/nmeapos3", 0.0).toDouble();

    for (i = 0; i < 10; i++)
        history[i] = settings.value(QString("tcpopt/history%1").arg(i), "").toString();

    nMapPoint = settings.value("mapopt/nmappnt", 0).toInt();

    for (i = 0; i < nMapPoint; i++) {
        pointName[i] = settings.value(QString("mapopt/pntname%1").arg(i + 1), "").toString();
        QString pos = settings.value(QString("mapopt/pntpos%1").arg(i + 1), "0,0,0").toString();
        pointPosition[i][0] = pointPosition[i][1] = pointPosition[i][2] = 0.0;
        sscanf(qPrintable(pos), "%lf,%lf,%lf", pointPosition[i], pointPosition[i] + 1, pointPosition[i] + 2);
    }

    updatePanels();

    ui->splitter->restoreState(settings.value("window/splitpos").toByteArray());

    resize(settings.value("window/width", 388).toInt(),
           settings.value("window/height", 284).toInt());
}
// save option to ini file --------------------------------------------------
void MainWindow::saveOptions()
{
    QSettings settings(iniFile, QSettings::IniFormat);
    int i, j, no, strno[] = { 0, 1, 6, 4, 5, 7, 2, 3, 8 };

    trace(3, "saveOptions\n");

    optDialog->saveOptions(settings);

    for (i = 0; i < MAXSTRRTK; i++) {
        no = strno[i];
        settings.setValue(QString("stream/streamc%1").arg(no), streamEnabled[i]);
        settings.setValue(QString("stream/stream%1").arg(no), streamType[i]);
        settings.setValue(QString("stream/format%1").arg(no), inputFormat[i]);
        for (j = 0; j < 4; j++)
            settings.setValue(QString("stream/path_%1_%2").arg(no).arg(j), paths[i][j]);
    }
    for (i = 0; i < 3; i++)
        settings.setValue(QString("stream/rcvopt%1").arg(i + 1), receiverOptions[i]);

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            commands[i][j].replace("\r\n", "@@");
            settings.setValue(QString("serial/cmd_%1_%2").arg(i).arg(j), commands[i][j]);
            settings.setValue(QString("serial/cmdena_%1_%2").arg(i).arg(j), commandEnabled[i][j]);
        }
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            commandsTcp[i][j].replace("\r\n", "@@");
            settings.setValue(QString("tcpip/cmd_%1_%2").arg(i).arg(j), commandsTcp[i][j]);
            settings.setValue(QString("tcpip/cmdena_%1_%2").arg(i).arg(j), commandEnableTcp[i][j]);
        }
    settings.setValue("setting/intimetag", inputTimeTag);
    settings.setValue("setting/intimespeed", inputTimeSpeed);
    settings.setValue("setting/intimestart", inputTimeStart);
    settings.setValue("setting/intime64bit", inputTimeTag64bit);
    settings.setValue("setting/outtimetag", outputTimeTag);
    settings.setValue("setting/outswapinterval", outputSwapInterval);
    settings.setValue("setting/logtimetag", logTimeTag);
    settings.setValue("setting/logswapinterval", logSwapInterval);
    settings.setValue("setting/resetcmd", resetCommand);
    settings.setValue("setting/maxbl", maxBaseLine);

    settings.setValue("setting/timesys", timeSystem);
    settings.setValue("setting/soltype", solutionType);
    settings.setValue("setting/plottype", plotType[0]);
    settings.setValue("setting/plottype2", plotType[1]);
    settings.setValue("setting/plottype3", plotType[2]);
    settings.setValue("setting/plottype4", plotType[3]);
    settings.setValue("setting/panelmode", panelMode);
    settings.setValue("setting/trktype1", trackType[0]);
    settings.setValue("setting/trktype2", trackType[1]);
    settings.setValue("setting/trktype3", trackType[2]);
    settings.setValue("setting/trktype4", trackType[3]);
    settings.setValue("setting/trkscale1", trackScale[0]);
    settings.setValue("setting/trkscale2", trackScale[1]);
    settings.setValue("setting/trkscale3", trackScale[2]);
    settings.setValue("setting/trkscale4", trackScale[3]);
    settings.setValue("setting/freqtype1", frequencyType[0]);
    settings.setValue("setting/freqtype2", frequencyType[1]);
    settings.setValue("setting/freqtype3", frequencyType[2]);
    settings.setValue("setting/freqtype4", frequencyType[3]);
    settings.setValue("setting/blmode1", baselineMode[0]);
    settings.setValue("setting/blmode2", baselineMode[1]);
    settings.setValue("setting/blmode3", baselineMode[2]);
    settings.setValue("setting/blmode4", baselineMode[3]);
    settings.setValue("setting/markername", markerName);
    settings.setValue("setting/markercomment", markerComment);

    settings.setValue("setting/nmeareq", nmeaRequestType);
    settings.setValue("setting/nmeapos1", nmeaPosition[0]);
    settings.setValue("setting/nmeapos2", nmeaPosition[1]);
    settings.setValue("setting/nmeapos3", nmeaPosition[2]);

    for (i = 0; i < 10; i++)
        settings.setValue(QString("tcpopt/history%1").arg(i), history[i]);
    settings.setValue("mapopt/nmappnt", nMapPoint);
    for (i = 0; i < nMapPoint; i++) {
        settings.setValue(QString("mapopt/pntname%1").arg(i + 1), pointName[i]);
        settings.setValue(QString("mapopt/pntpos%1").arg(i + 1),
                  QString("%1,%2,%3").arg(pointPosition[i][0], 0, 'f', 4).arg(pointPosition[i][1], 0, 'f', 4).arg(pointPosition[i][2], 0, 'f', 4));
    }

    settings.setValue("window/width", size().width());
    settings.setValue("window/height", size().height());

    settings.setValue("window/splitpos", ui->splitter->saveState());
}
//---------------------------------------------------------------------------
void MainWindow::btnMarkClicked()
{
    markDialog->setPositionMode(rtksvr->rtk.opt.mode);
    markDialog->setName(markerName);
    markDialog->setComment(markerComment);
    markDialog->setStationPositionFile(optDialog->fileOptions.stapos);

    if (markDialog->exec() != QDialog::Accepted) return;

    rtksvr->rtk.opt.mode = markDialog->getPositionMode();
    markerName = markDialog->getName();
    markerComment = markDialog->getComment();

    if (!markerName.isEmpty())
        rtksvrmark(rtksvr, qPrintable(markerName), qPrintable(markerComment));

    updatePosition();
}
// execute command ----------------------------------------------------------
int MainWindow::execCommand(const QString &cmd, const QStringList &opt)
{
    return QProcess::startDetached(cmd, opt);
}
//---------------------------------------------------------------------------
