//---------------------------------------------------------------------------
// rtkplot_qt : visualization of solution and obs data ap
//
//          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann
//
// options : rtkplot [-t title][-i file][-r][-p path][-x level][file ...]
//
//           -t title  window title
//           -i file   ini file path
//           -r        open file as obs and nav file
//           -p path   connect to path
//                       serial://port[:brate[:bsize[:parity[:stopb[:fctr]]]]]
//                       tcpsvr://:port
//                       tcpcli://addr[:port]
//                       ntrip://[user[:passwd]@]addr[:port][/mntpnt]
//                       file://path
//           -p1 path  connect port 1 to path
//           -p2 path  connect port 2 to path
//           -x level  debug trace level (0:off)
//           file      solution files or rinex obs/nav file
//
// version : $Revision: 1.1 $ $Date: 2008/07/17 22:15:27 $
// history : 2008/07/14  1.0 new
//           2009/11/27  1.1 rtklib 2.3.0
//           2010/07/18  1.2 rtklib 2.4.0
//           2010/06/10  1.3 rtklib 2.4.1
//           2010/06/19  1.4 rtklib 2.4.1 p1
//           2012/11/21  1.5 rtklib 2.4.2
//           2016/06/11  1.6 rtklib 2.4.3
//           2020/11/30  1.7 support NavIC/IRNSS
//                           delete functions for Google Earth View
//                           support Map API by Leaflet for Map View
//                           move Google Map API Key option to Map View
//                           modify order of ObsType selections
//                           improve slip detection performance by LG jump
//                           fix bug on MP computation for L3,L4,... freq.

//---------------------------------------------------------------------------
#include <clocale>

#include <QShowEvent>
#include <QCloseEvent>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QFocusEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QPaintEvent>
#include <QDebug>
#include <QToolBar>
#include <QScreen>
#include <QtGlobal>
#include <QFileInfo>
#include <QWidgetAction>
#include <QToolTip>
#include <QListWidget>
#include <QMessageBox>

#include <QFileInfo>
#include <QCommandLineParser>
#include <QFileDialog>
#include <QClipboard>
#include <QSettings>
#include <QFont>
#include <QMimeData>
#include <QTableView>

#include "rtklib.h"
#include "plotmain.h"
#include "plotopt.h"
#include "tspandlg.h"
#include "aboutdlg.h"
#include "conndlg.h"
#include "console.h"
#include "pntdlg.h"
#include "mapoptdlg.h"
#include "skydlg.h"
#include "freqdlg.h"
#include "mapview.h"
#include "viewer.h"
#include "vmapdlg.h"
#include "fileseldlg.h"
#include "helper.h"

#include "ui_plotmain.h"

#define YLIM_AGE    10.0        // ylimit of age of differential
#define YLIM_RATIO  20.0        // ylimit of ratio factor
#define qMaxSHAPEFILE 16             // max number of shape files

extern "C" {
    extern void settime(gtime_t) {}
    extern void settspan(gtime_t, gtime_t) {}
}


// constructor --------------------------------------------------------------
Plot::Plot(QWidget *parent) : QMainWindow(parent), ui(new Ui::Plot)
{
    ui->setupUi(this);

    setWindowIcon(QIcon(":/icons/rtk2"));

    setlocale(LC_NUMERIC, "C"); // use point as decimal separator in formatted output

    gtime_t t0 = {0, 0};
    nav_t nav0 = {};
    obs_t obs0 = {};
    sta_t sta0 = {};
    gis_t gis0 = {};
    solstatbuf_t solstat0 = {0, 0, 0};
    double ep[] = {2000, 1, 1, 0, 0, 0}, xl[2], yl[2];
    double xs[] = {-DEFTSPAN / 2, DEFTSPAN / 2};

    QString file = QApplication::applicationFilePath();
    QFileInfo fi(file);
    iniFile = fi.absoluteDir().filePath(fi.baseName()) + ".ini";

    ui->toolBar->addWidget(ui->toolPanel);

    formWidth = formHeight = 0;
    dragState = 0;
    dragCurrentX = dragCurrentY = -1;
    nObservation = 0;
    indexObservation = NULL;
    week = flush = plotType = 0;

    for (int i = 0; i < 2; i++) {
        initsolbuf(solutionData + i, 0, 0);
        solutionStat[i] = solstat0;
        solutionIndex[i] = 0;
    }

    obs0.data = NULL; obs0.n = obs0.nmax = 0;
    nav0.eph = NULL; nav0.n = nav0.nmax = 0;
    nav0.geph = NULL; nav0.ng = nav0.ngmax = 0;
    nav0.seph = NULL; nav0.ns = nav0.nsmax = 0;

    observationIndex = 0;
    observation = obs0;
    navigation = nav0;
    station = sta0;
    gis = gis0;
    simulatedObservation = 0;

    dragStartX = dragStartY = dragCenterX = dragCenterY = dragScaleX = dragScaleY = centX = 0.0;
    mouseDownTick = 0;
    GEState = GEDataState[0] = GEDataState[1] = 0;
    GEHeading = 0.0;

    originEpoch = t0;
    for (int i = 0; i < 3; i++)
        originPosition[i] = originVelocity[i] = 0.0;

    azimuth = elevation = NULL;
    for (int i = 0; i < NFREQ + NEXOBS; i++)
        multipath[i] = NULL;

    graphTrack = new Graph(ui->lblDisplay);
    graphTrack->fit = 0;

    graphSky = new Graph(ui->lblDisplay);

    graphSingle = new Graph(ui->lblDisplay);
    graphSingle->getLimits(xl, yl);
    graphSingle->setLimits(xs, yl);

    for (int i = 0; i < 2; i++)
        graphDual[i] = new Graph(ui->lblDisplay);

    for (int i = 0; i < 3; i++) {
        graphTriple[i] = new Graph(ui->lblDisplay);
        graphTriple[i]->xLabelPosition = Graph::LabelPosition::Off;
        graphTriple[i]->getLimits(xl, yl);
        graphTriple[i]->setLimits(xs, yl);
    }

    pointCoordinateType = 0;

    wayPoints.clear();
    selectedWayPoint = -1;

    for (int i = 0; i < 3; i++) timeEnabled[i] = 0;
    timeInterval = 0.0;

    timeStart = timeEnd = epoch2time(ep);
    console1 = new Console(this);
    console2 = new Console(this);

    for (int i = 0; i < 361; i++) elevationMaskData[i] = 0.0;

    traceLevel = 0;
    connectState = openRaw = 0;
    rtConnectionType = 0;

    strinitcom();
    strinit(stream);
    strinit(stream + 1);

    ui->cBFrequencyType->clear();
    ui->cBFrequencyType->addItem(QStringLiteral("L1/LC"));
    if (NFREQ >= 2) ui->cBFrequencyType->addItem(QStringLiteral("L2/E5b"));
    if (NFREQ >= 3) ui->cBFrequencyType->addItem(QStringLiteral("L5/E5a"));
    if (NFREQ >= 4) ui->cBFrequencyType->addItem(QStringLiteral("L6"));
    if (NFREQ >= 5) ui->cBFrequencyType->addItem(QStringLiteral("L7"));
    if (NFREQ >= 6) ui->cBFrequencyType->addItem(QStringLiteral("L8"));
    ui->cBFrequencyType->setCurrentIndex(0);

    tleData.n = tleData.nmax = 0;
    tleData.data = NULL;

    vecMapDialog = new VecMapDialog(this, this);
    freqDialog = new FreqDialog(this);
    mapOptDialog = new MapOptDialog(this, this);
    mapView = new MapView(this);
    spanDialog = new SpanDialog(this);
    connectDialog = new ConnectDialog(this);
    skyImgDialog = new SkyImgDialog(this);
    plotOptDialog = new PlotOptDialog(this);
    aboutDialog = new AboutDialog(this, QPixmap(":/icons/rtkplot"), PRGNAME);
    waypointsDialog = new PntDialog(this);
    fileSelDialog = new FileSelDialog(this, this);
    viewer = new TextViewer(this);

    ui->btnReload->setDefaultAction(ui->menuReload);
    ui->btnClear->setDefaultAction(ui->menuClear);
    ui->btnOptions->setDefaultAction(ui->menuOptions);
    ui->btnMapView->setDefaultAction(ui->menuMapView);
    ui->btnCenterOrigin->setDefaultAction(ui->menuCenterOrigin);
    ui->btnFitHorizontal->setDefaultAction(ui->menuFitHoriz);
    ui->btnFitVertical->setDefaultAction(ui->menuFitVert);
    ui->btnShowTrack->setDefaultAction(ui->menuShowTrack);
    ui->btnFixCenter->setDefaultAction(ui->menuFixCenter);
    ui->btnFixHorizontal->setDefaultAction(ui->menuFixHoriz);
    ui->btnFixVertical->setDefaultAction(ui->menuFixVert);
    ui->btnShowMap->setDefaultAction(ui->menuShowMap);
    ui->btnShowGrid->setDefaultAction(ui->menuShowGrid);
    ui->btnShowImage->setDefaultAction(ui->menuShowImage);
    ui->btnShowSkyplot->setDefaultAction(ui->menuShowSkyplot);
    ui->menuShowSkyplot->setChecked(true);
    ui->menuShowGrid->setChecked(true);

    rangeListPopupAction = new QWidgetAction(ui->btnRangeList);
    lWRangeList = new QListWidget();
    float rangeValue[] = {0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10};
    for (int i = 0; i < (int)(sizeof(rangeValue)/sizeof(float)); i++) {
        float value = rangeValue[i];
        QString unit;

        if (fmod(value, 1000) == 0) {
            unit = "km";
            value /= 1000;
        } else if ((fmod(value*100, 1) == 0) && (value < 1)) {
            unit = "cm";
            value *= 100;
        } else unit = "m";

        QListWidgetItem *item = new QListWidgetItem();
        item->setText(QString("%L1 %L2").arg(value).arg(unit));
        item->setData(Qt::UserRole+1, rangeValue[i]);
        lWRangeList->addItem(item);
    }
    rangeListPopupAction->setDefaultWidget(lWRangeList);
    ui->btnRangeList->setPopupMode(QToolButton::InstantPopup);
    ui->btnRangeList->addAction(rangeListPopupAction);

    dirModel = new QFileSystemModel(this);
    dirModel->setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    tVdirectorySelector = new QTreeView(this);
    ui->panelBrowse->layout()->addWidget(tVdirectorySelector);
    tVdirectorySelector->setModel(dirModel);
    tVdirectorySelector->hideColumn(1);
    tVdirectorySelector->hideColumn(2);
    tVdirectorySelector->hideColumn(3); //only show diretory names

    fileModel = new QFileSystemModel(this);
    fileModel->setFilter((fileModel->filter() & ~QDir::Dirs & ~QDir::AllDirs));
    fileModel->setNameFilterDisables(false);
    ui->lVFileList->setModel(fileModel);

    connect(ui->btnOn1, &QPushButton::clicked, this, &Plot::updatePlotSizeAndRefresh);
    connect(ui->btnOn2, &QPushButton::clicked, this, &Plot::updatePlotSizeAndRefresh);
    connect(ui->btnOn3, &QPushButton::clicked, this, &Plot::updatePlotSizeAndRefresh);
    connect(ui->btnSolution1, &QPushButton::clicked, this, &Plot::activateSolution1);
    connect(ui->btnSolution2, &QPushButton::clicked, this, &Plot::activateSolution2);
    connect(ui->btnSolution12, &QPushButton::clicked, this, &Plot::activateSolution12);
    connect(ui->btnConnect, &QPushButton::clicked, this, &Plot::toggleConnectState);
    connect(ui->btnAnimate, &QPushButton::clicked, this, &Plot::updateEnable);
    connect(ui->btnFrequency, &QPushButton::clicked, this, &Plot::showFrequencyDialog);
    connect(ui->menuAbout, &QAction::triggered, this, &Plot::showAboutDialog);
    connect(ui->menuAnimationStart, &QAction::triggered, this, &Plot::menuAnimationStartClicked);
    connect(ui->menuAnimationStop, &QAction::triggered, this, &Plot::menuAnimationStopClicked);
    connect(ui->menuBrowse, &QAction::triggered, this, &Plot::updateBrowseBarVisibility);
    connect(ui->menuCenterOrigin, &QAction::triggered, this, &Plot::centerOrigin);
    connect(ui->menuClear, &QAction::triggered, this, &Plot::clear);
    connect(ui->menuConnect, &QAction::triggered, this, &Plot::connectStream);
    connect(ui->menuDisconnect, &QAction::triggered, this, &Plot::disconnectStream);
    connect(ui->menuFitHoriz, &QAction::triggered, this, &Plot::fitHorizontally);
    connect(ui->menuFitVert, &QAction::triggered, this, &Plot::fitVertically);
    connect(ui->menuFixCenter, &QAction::triggered, this, &Plot::updatePlotAndEnable);
    connect(ui->menuFixHoriz, &QAction::triggered, this, &Plot::fixHorizontally);
    connect(ui->menuFixVert, &QAction::triggered, this, &Plot::updatePlotAndEnable);
    connect(ui->menuMapView, &QAction::triggered, this, &Plot::showMapView);
    connect(ui->menuMapImage, &QAction::triggered, this, &Plot::showMapOptionDialog);
    connect(ui->menuMax, &QAction::triggered, this, &Plot::showWindowMaximized);
    connect(ui->menuMonitor1, &QAction::triggered, this, &Plot::showMonitorConsole1);
    connect(ui->menuMonitor2, &QAction::triggered, this, &Plot::showMonitorConsole2);
    connect(ui->menuOpenElevationMask, &QAction::triggered, this, &Plot::openElevationMaskFile);
    connect(ui->menuOpenMapImage, &QAction::triggered, this, &Plot::openMapImage);
    connect(ui->menuOpenShape, &QAction::triggered, this, &Plot::openShapeFile);
    connect(ui->menuOpenNav, &QAction::triggered, this, &Plot::openNavigationFile);
    connect(ui->menuOpenObs, &QAction::triggered, this, &Plot::openObservationFile);
    connect(ui->menuOpenSkyImage, &QAction::triggered, this, &Plot::openSkyImage);
    connect(ui->menuOpenSolution1, &QAction::triggered, this, &Plot::openSolution1);
    connect(ui->menuOpenSolution2, &QAction::triggered, this, &Plot::openSolution2);
    connect(ui->menuOptions, &QAction::triggered, this, &Plot::showPlotOptionsDialog);
    connect(ui->menuPlotMapViewHorizontal, &QAction::triggered, this, &Plot::arrangePlotMapViewHorizontally);
    connect(ui->menuPort, &QAction::triggered, this, &Plot::showConnectionSettingsDialog);
    connect(ui->menuQuit, &QAction::triggered, this, &Plot::close);
    connect(ui->menuReload, &QAction::triggered, this, &Plot::reload);
    connect(ui->menuSaveDop, &QAction::triggered, this, &Plot::saveDopFile);
    connect(ui->menuSaveElevationMask, &QAction::triggered, this, &Plot::saveElevationMaskFile);
    connect(ui->menuSaveImage, &QAction::triggered, this, &Plot::savePlotImage);
    connect(ui->menuSaveSnrMp, &QAction::triggered, this, &Plot::saveSnrMpFile);
    connect(ui->menuShowMap, &QAction::triggered, this, &Plot::updatePlotAndEnable);
    connect(ui->menuShowImage, &QAction::triggered, this, &Plot::updatePlotAndEnable);
    connect(ui->menuShowGrid, &QAction::triggered, this, &Plot::updateShowGrid);
    connect(ui->menuShowSkyplot, &QAction::triggered, this, &Plot::updatePlotAndEnable);
    connect(ui->menuShowTrack, &QAction::triggered, this, &Plot::updateShowTrack);
    connect(ui->menuSkyImage, &QAction::triggered, this, &Plot::showSkyImageDialog);
    connect(ui->menuSourceObservation, &QAction::triggered, this, &Plot::showObservationText);
    connect(ui->menuSourceSolution, &QAction::triggered, this, &Plot::showSolutionText);
    connect(ui->menuStatusBar, &QAction::triggered, this, &Plot::updateStatusBarVisibility);
    connect(ui->menuTime, &QAction::triggered, this, &Plot::showStartEndTimeDialog);
    connect(ui->menuToolBar, &QAction::triggered, this, &Plot::updateToolBarVisibility);
    connect(ui->menuVisibilityAnalysis, &QAction::triggered, this, &Plot::visibilityAnalysis);
    connect(ui->menuWaypoints, &QAction::triggered, this, &Plot::showWaypointsDialog);
    connect(ui->menuMapLayer, &QAction::triggered, this, &Plot::showVectorMapDialog);
    connect(ui->menuOpenWaypoints, &QAction::triggered, this, &Plot::openWaypointsFile);
    connect(ui->menuSaveWaypoints, &QAction::triggered, this, &Plot::saveWaypointsFile);
    connect(ui->btnPointCoordinateType, &QPushButton::clicked, this, &Plot::changePointCoordinateType);
    connect(lWRangeList, &QListWidget::clicked, this, &Plot::rangeListItemSelected);
    connect(&timer, &QTimer::timeout, this, &Plot::timerTimer);
    connect(ui->cBPlotTypeSelection, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::updateSelectedPlotType);
    connect(ui->cBQFlag, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::updatePlotAndEnable);
    connect(ui->sBTime, &QScrollBar::sliderMoved, this, &Plot::updateCurrentObsSol);
    connect(ui->cBSatelliteList, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::satelliteListChanged);
    connect(ui->cBDopType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::updatePlotAndEnable);
    connect(ui->cBObservationType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::updatePlotAndEnable);
    connect(ui->cBObservationTypeSNR, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::updatePlotAndEnable);
    connect(ui->cBFrequencyType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::updatePlotAndEnable);
    connect(ui->cBDriveSelect, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::driveSelectChanged);
    connect(tVdirectorySelector, &QTableView::clicked, this, &Plot::directorySelectionChanged);
    connect(tVdirectorySelector, &QTableView::doubleClicked, this, &Plot::directorySelectionSelected);
    connect(ui->btnDirectorySelect, &QPushButton::clicked, this, &Plot::btnDirectorySelectorClicked);
    connect(ui->lVFileList, &QListView::clicked, this, &Plot::fileListClicked);
    connect(ui->cBFilter, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Plot::filterClicked);

    ui->statusbar->addPermanentWidget(ui->lblMessage2);
    ui->statusbar->addPermanentWidget(ui->btnPointCoordinateType);

    bool mapAvailable = false;
#ifdef QWEBENGINE
    mapAvailable = true;
#endif
    ui->menuMapView->setEnabled(mapAvailable);
    ui->menuPlotMapViewHorizontal->setEnabled(mapAvailable);

    ui->lblDisplay->setAttribute(Qt::WA_OpaquePaintEvent);
    setMouseTracking(true);

    loadOptions();

    updateTimer.start();
    qApp->installEventFilter(this);
}
// destructor ---------------------------------------------------------------
Plot::~Plot()
{
    delete [] indexObservation;
    delete [] azimuth;
    delete [] elevation;

    for (int i = 0; i < NFREQ + NEXOBS; i++) delete multipath[i];

    delete graphTrack;
    delete graphSingle;
    delete graphSky;

    for (int i = 0; i < 2; i++)
        delete graphDual[i];
    for (int i = 0; i < 3; i++)
        delete graphTriple[i];

    delete lWRangeList;

    delete tVdirectorySelector;
}
// callback on all events ----------------------------------------------------
bool Plot::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::MouseMove)
    {
        if (obj == ui->lblDisplay) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            mouseMove(mouseEvent);
            return true;
        } else
            ui->lblMessage2->setText("");
    }
    if (event->type() == QEvent::Resize)
    {
        if (obj == ui->lblDisplay) {
            updatePlotSizes();
            //refresh();
            return true;
        }
    }

    // buttons in Qt do not support capturing double-clicks
    if ((obj == ui->btnSolution1) && (event->type() == QEvent::MouseButtonDblClick))
    {
        openSolution1();
        event->accept();
        return true;
    }
    if ((obj == ui->btnSolution2) && (event->type() == QEvent::MouseButtonDblClick))
    {
        openSolution2();
        event->accept();
        return true;
    }

    return false;
}
// callback on form-show ----------------------------------------------------
void Plot::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) {
        refresh();
        return;
    }
    QString path1, path2;

    trace(3, "showEvent\n");

    QCommandLineParser parser;
    parser.setApplicationDescription(tr("RTK plot"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption rawOption(QStringList() << "r",
                     QCoreApplication::translate("main", "open raw"));
    parser.addOption(rawOption);

    QCommandLineOption titleOption(QStringList() << "t" << "title",
                       QCoreApplication::translate("main", "use window tile <title>."),
                       QCoreApplication::translate("main", "title"));
    parser.addOption(titleOption);

    QCommandLineOption pathOption(QStringList() << "p" << "path",
                      QCoreApplication::translate("main", "open <path>."),
                      QCoreApplication::translate("main", "path"));
    parser.addOption(pathOption);

    QCommandLineOption path1Option(QStringList() << "1",
                       QCoreApplication::translate("main", "open <path> as solution 1."),
                       QCoreApplication::translate("main", "path"));
    parser.addOption(path1Option);

    QCommandLineOption path2Option(QStringList() << "2",
                       QCoreApplication::translate("main", "open <path> as solution 2."),
                       QCoreApplication::translate("main", "path"));
    parser.addOption(path2Option);

    QCommandLineOption traceOption(QStringList() << "x" << "tracelevel",
                       QCoreApplication::translate("main", "set trace lavel to <tracelavel>."),
                       QCoreApplication::translate("main", "tracelevel"));
    parser.addOption(traceOption);

    parser.addPositionalArgument("file", QCoreApplication::translate("main", "Read files."));

    parser.process(*QApplication::instance());

    if (parser.isSet(rawOption))
        openRaw = 1;
    if (parser.isSet(titleOption))
        title = parser.value(titleOption);
    if (parser.isSet(pathOption))
        path1 = parser.value(pathOption);
    if (parser.isSet(path1Option))
        path1 = parser.value(path1Option);
    if (parser.isSet(path2Option))
        path2 = parser.value(path2Option);
    if (parser.isSet(traceOption))
        traceLevel = parser.value(traceOption).toInt();

    const QStringList args = parser.positionalArguments();
    foreach(const QString &file, args)
        openFiles.append(file);

    if (traceLevel > 0) {
        traceopen(TRACEFILE);
        tracelevel(traceLevel);
    }
    loadOptions();

    updatePlotType(plotType >= PLOT_OBS ? PLOT_TRK : plotType);

    updateColor();
    updateSatelliteMask();
    updateOrigin();
    updatePlotSizes();

    if (!path1.isEmpty() || !path2.isEmpty()) {
        connectPath(path1, 0);
        connectPath(path2, 1);
        connectStream();
    } else if (openFiles.count() <= 0) {
        setWindowTitle(!title.isEmpty() ? title : tr("%1 ver. %2 %3").arg(PRGNAME, VER_RTKLIB, PATCH_LEVEL));
    }

    if (!plotOptDialog->getShapeFile().isEmpty()) {
        QStringList files;
        char *paths[qMaxSHAPEFILE];

        for (int i = 0; i < qMaxSHAPEFILE; i++) {
            paths[i] = new char [1024];
        }
        int n = expath(qPrintable(plotOptDialog->getShapeFile()), paths, qMaxSHAPEFILE);

        for (int i = 0; i < n; i++) {
            files.append(paths[i]);
        }
        readShapeFile(files);

        for (int i = 0; i < qMaxSHAPEFILE; i++) {
            delete[] paths[i];
        }
    }

    if (!plotOptDialog->getTleFile().isEmpty())
        tle_read(qPrintable(plotOptDialog->getTleFile()), &tleData);
    if (!plotOptDialog->getTleSatelliteFile().isEmpty())
        tle_name_read(qPrintable(plotOptDialog->getTleSatelliteFile()), &tleData);

    QFileInfoList drives = QDir::drives();
    if (drives.size() > 1 && drives.at(0).filePath() != "/") {
        ui->panelDrive->setVisible(true);
        ui->cBDriveSelect->clear();

        foreach(const QFileInfo &drive, drives) {
            ui->cBDriveSelect->addItem(drive.filePath());
        }
    } else {
        ui->panelDrive->setVisible(false); // do not show drive selection on unix
    }
    if (directory.isEmpty()) directory = drives.at(0).filePath();

    ui->cBDriveSelect->setCurrentText(directory.mid(0, directory.indexOf(":") + 2));  // preselect correct drive
    dirModel->setRootPath(directory);
    tVdirectorySelector->setVisible(false);
    ui->lblDirectorySelected->setText(directory);
    fileModel->setRootPath(directory);
    ui->lVFileList->setRootIndex(fileModel->index(directory));
    filterClicked();

    if (ui->menuBrowse->isChecked()) {
        ui->panelBrowse->setVisible(true);
    } else {
        ui->panelBrowse->setVisible(false);
    }

    strinit(&streamTimeSync);
    nStreamBuffer = 0;

    if (plotOptDialog->getTimeSyncOut()) {
        stropen(&streamTimeSync, STR_TCPSVR, STR_MODE_RW, qPrintable(QStringLiteral(":%1").arg(plotOptDialog->getTimeSyncPort())));
    }

    timer.start(plotOptDialog->getRefreshCycle());
    updatePlot();
    updateEnable();

    if (openFiles.count() > 0) {
        if (isObservation(openFiles.at(0)) || openRaw)
            readObservation(openFiles);
        else
            readSolution(openFiles, 0);
    }
}
// callback on form-close ---------------------------------------------------
void Plot::closeEvent(QCloseEvent *)
{
    trace(3, "closeEvent\n");

    lWRangeList->hide();

    saveOption();

    if (traceLevel > 0) traceclose();
}

// callback for painting   --------------------------------------------------
void Plot::paintEvent(QPaintEvent *)
{
    updateDisplay();
}
// callback on form-resize --------------------------------------------------
void Plot::resizeEvent(QResizeEvent *)
{
    trace(3, "resizeEvent\n");

    // suppress repeated resize callback
    if (formWidth == width() && formHeight == height()) return;

    updatePlotSizes();
    //refresh();

    formWidth = width();
    formHeight = height();
}
// callback on drag-and-drop files ------------------------------------------
void Plot::dragEnterEvent(QDragEnterEvent *event)
{
    trace(3, "dragEnterEvent\n");

    if (event->mimeData()->hasFormat("text/uri-list"))
        event->acceptProposedAction();
}

// callback on drag-and-drop files ------------------------------------------
void Plot::dropEvent(QDropEvent *event)
{
    QStringList files;

    trace(3, "dropEvent\n");

    if (connectState || !event->mimeData()->hasUrls())
        return;

    foreach(QUrl url, event->mimeData()->urls()) {
        files.append(QDir::toNativeSeparators(url.toString()));
    }

    const QString &file = files.at(0);

    if (files.size() == 1) {
        QFileInfo fi(file);
        QString ext = fi.completeSuffix();
        if ((ext == "jpg") || (ext == "jpeg")) {
            if (plotType == PLOT_TRK)
                readMapData(file);
            else if (plotType == PLOT_SKY || plotType == PLOT_MPS)
                readSkyData(file);
        };
    } else if (isObservation(files.at(0))) {
        readObservation(files);
    } else if (!ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked()) {
        readSolution(files, 1);
    } else {
        readSolution(files, 0);
    }
}
// callback on menu-open-solution-1 -----------------------------------------
void Plot::openSolution1()
{
    trace(3, "openSolution1\n");

    readSolution(QStringList(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open Solution 1"), solutionFiles[0].value(0), tr("Solution File (*.pos *.stat *.nmea *.txt *.ubx);;All (*.*)")))), 0);
}
// callback on menu-open-solution-2 -----------------------------------------
void Plot::openSolution2()
{
    trace(3, "openSolution2\n");

    readSolution(QStringList(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open Solution 2"), solutionFiles[1].value(0), tr("Solution File (*.pos *.stat *.nmea *.txt *.ubx);;All (*.*)")))), 1);
}
// callback on menu-open-map-image ------------------------------------------
void Plot::openMapImage()
{
    trace(3, "openMapImage\n");

    readMapData(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open Map Image"), mapImageFile, tr("JPEG File (*.jpg *.jpeg);;All (*.*)"))));
}
// callback on menu-open-track-points ---------------------------------------
void Plot::openShapeFile()
{
    trace(3, "openShapeFile\n");

    QStringList files = QFileDialog::getOpenFileNames(this, tr("Open Shape File"), QString(), tr("Shape File (*.shp);;All (*.*)"));
    for (int i = 0; i < files.size(); i++)
        files[i] = QDir::toNativeSeparators(files.at(i));

    readShapeFile(files);
}
// callback on menu-open-sky-image ------------------------------------------
void Plot::openSkyImage()
{
    trace(3, "openSkyImage\n");

    readSkyData(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open Sky Image"), skyImageFile, tr("JPEG File (*.jpg *.jpeg);;All (*.*)"))));
}
// callback on menu-oepn-waypoint -------------------------------------------
void Plot::openWaypointsFile()
{
    trace(3, "openWaypointFile\n");

    readWaypoints(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open Waypoints"), QString(), tr("Waypoints File (*.gpx *.pos);;All (*.*)"))));
}
// callback on menu-open-obs-data -------------------------------------------
void Plot::openObservationFile()
{
    trace(3, "openObservationFile\n");
    readObservation(QFileDialog::getOpenFileNames(this, tr("Open Obs/Nav Data"), observationFiles.value(0), tr("RINEX OBS (*.obs *.*o *.*O *.*d *.O.rnx *O.rnx *O.rnx.gz *O.crx *O.crx.gz *.*o.gz *.*o.Z *.d.gz *.d.Z);;All (*.*)")));
}
// callback on menu-open-nav-data -------------------------------------------
void Plot::openNavigationFile()
{
    trace(3, "openNavigationFile\n");

    readNavigation(QFileDialog::getOpenFileNames(this, tr("Open Raw Obs/Nav Messages"), navigationFiles.value(0), tr("RINEX NAV (*.nav *.gnav *.hnav *.qnav *.*n *.*g *.*h *.*q *.*p *N.rnx *N.rnx.gz);;All (*.*)")));
}
// callback on menu-open-elev-mask ------------------------------------------
void Plot::openElevationMaskFile()
{
    trace(3, "openElevationMaskFile\n");

    readElevationMaskData(QDir::toNativeSeparators(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open Elevation Mask"), QString(), tr("Text File (*.txt);;All (*.*)")))));
}
// callback on menu-vis-analysis --------------------------------------------
void Plot::visibilityAnalysis()
{
    if (plotOptDialog->getReceiverPosition() != 1) { // lat/lon/height
        QMessageBox::information(this, tr("Visibility analysis failed."),
                                 tr("Please set \"Receiver Position\" to \"Lat/Lon/Height\" in the option dialog and provide its coordinates."));
        return;
    }
    if (plotOptDialog->getTleFile().isEmpty() || plotOptDialog->getTleSatelliteFile().isEmpty()) {
        QMessageBox::information(this, tr("Visibility analysis failed."),
                                 tr("Please specify TLE data in the option dialog."));
        return;
    }

    if (spanDialog->getStartTime().time == 0) {
        int week;
        gtime_t now;
        double tow = time2gpst(utc2gpst(timeget()), &week);

        now = gpst2time(week, floor(tow / 3600.0) * 3600.0);
        spanDialog->setStartTime(now);
        spanDialog->setEndTime(timeadd(now, 86400.0));
        spanDialog->setTimeInterval(30.0);
    }
    for (int i = 0; i < 3; i++) {
        spanDialog->setTimeEnable(i, true);
        spanDialog->setTimeValid(i, 2);
    }

    spanDialog->exec();

    if (spanDialog->result() == QDialog::Accepted) {
        timeStart = spanDialog->getStartTime();
        timeEnd = spanDialog->getEndTime();
        timeInterval = spanDialog->getTimeInterval();
        generateVisibilityData();
    }
    for (int i = 0; i < 3; i++)
        spanDialog->setTimeValid(i, 1);
}
// callback on menu-save image ----------------------------------------------
void Plot::savePlotImage()
{
    trace(3, "savePlotImage\n");
    buffer.save(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Save Image"), QString(), tr("JPEG  (*.jpg);;Windows Bitmap (*.bmp)"))));
}
// callback on menu-save-waypoint -------------------------------------------
void Plot::saveWaypointsFile()
{
    trace(3, "saveWaypointsFile\n");

    saveWaypoints(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Save Waypoints"), QString(), tr("GPX File (*.gpx *.pos);;All (*.*)"))));
}
// callback on menu-save-# of sats/dop --------------------------------------
void Plot::saveDopFile()
{
    trace(3, "saveDopFile\n");

    saveDop(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Save Data"), QString(), tr("All (*.*);;Text File (*.txt)"))));
}
// callback on menu-save-snr,azel -------------------------------------------
void Plot::saveSnrMpFile()
{
    trace(3, "saveSnrMpFile\n");

    saveSnrMp(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Save Data"), QString(), tr("All (*.*);;Text File (*.txt)"))));
}
// callback on menu-save-elmask ---------------------------------------------
void Plot::saveElevationMaskFile()
{
    trace(3, "saveElevationMaskFile\n");

    saveElevationMask(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Save Data"), QString(), tr("All (*.*);;Text File (*.txt)"))));
}
// callback on menu-connection-settings -------------------------------------
void Plot::showConnectionSettingsDialog()
{
    trace(3, "showConnectionSettingsDialog\n");

    for (int i = 0; i < 2; i++) {
        connectDialog->setStreamType(i, rtStream[i]);
        connectDialog->setStreamFormat(i, rtFormat[i]);
    }
    connectDialog->setTimeFormat(rtTimeFormat);
    connectDialog->setDegFormat(rtDegFormat);
    connectDialog->setFieldSeparator(rtFieldSeperator);
    connectDialog->setTimeoutTime(rtTimeoutTime);
    connectDialog->setReconnectTime(rtReconnectTime);
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            connectDialog->setPath(i, j, streamPaths[i][j]);

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            connectDialog->setCommands(i, j, streamCommands[i][j]);
            connectDialog->setCommandsEnabled(i,j, streamCommandEnabled[i][j]);
    }
    for (int i = 0; i < 10; i++) connectDialog->setHistory(i, streamHistory[i]);

    connectDialog->exec();
    if (connectDialog->result() != QDialog::Accepted) return;

    for (int i = 0; i < 2; i++) {
        rtStream[i] = connectDialog->getStreamType(i);
        rtFormat[i] = connectDialog->getStreamFormat(i);
    }
    rtTimeFormat = connectDialog->getTimeFormat();
    rtDegFormat = connectDialog->getDegFormat();
    rtFieldSeperator = connectDialog->getFieldSeparator();
    rtTimeoutTime = connectDialog->getTimeoutTime();
    rtReconnectTime = connectDialog->getReconnectTime();
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++) {
            streamPaths[i][j] = connectDialog->getPath(i, j);
    }
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            streamCommands[i][j] = connectDialog->getCommands(i, j);
            streamCommandEnabled[i][j] = connectDialog->getCommandsEnabled(i, j);
    }
    for (int i = 0; i < 10; i++) streamHistory[i] = connectDialog->getHistory(i);
}
// callback on menu-time-span/interval --------------------------------------
void Plot::showStartEndTimeDialog()
{
    sol_t *sols, *sole;
    int i;

    trace(3, "showTimeSpanDialog\n");

    // if no start time is set by the user...
    if (!timeEnabled[0]) {
        if (observation.n > 0) {
            timeStart = observation.data[0].time;
        } else if (ui->btnSolution2->isChecked() && solutionData[1].n > 0) {
            sols = getsol(solutionData + 1, 0);
            timeStart = sols->time;
        } else if (solutionData[0].n > 0) {
            sols = getsol(solutionData, 0);
            timeStart = sols->time;
        }
    }
    // if no end time is set by the user...
    if (!timeEnabled[1]) {
        if (observation.n > 0) {
            timeEnd = observation.data[observation.n - 1].time;
        } else if (ui->btnSolution2->isChecked() && solutionData[1].n > 0) {
            sole = getsol(solutionData + 1, solutionData[1].n - 1);
            timeEnd = sole->time;
        } else if (solutionData[0].n > 0) {
            sole = getsol(solutionData, solutionData[0].n - 1);
            timeEnd = sole->time;
        }
    }

    spanDialog->setStartTime(timeStart);
    spanDialog->setEndTime(timeEnd);
    spanDialog->setTimeInterval(timeInterval);
    spanDialog->setTimeValid(0, !connectState);
    spanDialog->setTimeValid(1, !connectState);
    for (i = 0; i < 3; i++)
        spanDialog->setTimeEnable(i, timeEnabled[i]);

    spanDialog->exec();
    if (spanDialog->result() != QDialog::Accepted) return;

    // check for changed settings
    if (timeEnabled[0] != spanDialog->getTimeEnable(0) ||
        timeEnabled[1] != spanDialog->getTimeEnable(1) ||
        timeEnabled[2] != spanDialog->getTimeEnable(2) ||
        timediff(timeStart, spanDialog->getStartTime()) != 0.0 ||
        timediff(timeEnd, spanDialog->getEndTime()) != 0.0 ||
        !qFuzzyCompare(timeInterval, spanDialog->getTimeInterval()))
    {
        for (i = 0; i < 3; i++) timeEnabled[i] = spanDialog->getTimeEnable(i);

        timeStart = spanDialog->getStartTime();
        timeEnd = spanDialog->getEndTime();
        timeInterval = spanDialog->getTimeInterval();

        reload();
    }
}
// callback on menu-map-image -----------------------------------------------
void Plot::showMapOptionDialog()
{
    trace(3, "showMapOptionDialog\n");

    mapOptDialog->show();
}
// callback on menu-sky image -----------------------------------------------
void Plot::showSkyImageDialog()
{
    trace(3, "showSkyImageDialog\n");

    skyImgDialog->show();
}
// callback on menu-vec map -------------------------------------------------
void Plot::showVectorMapDialog()
{
    trace(3, "showVectorMapDialog\n");

    vecMapDialog->exec();
}
// callback on menu-solution-source -----------------------------------------
void Plot::showSolutionText()
{
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked();

    trace(3, "showSolutionText\n");

    if (solutionFiles[sel].count() <= 0) return;

    viewer->setWindowTitle(solutionFiles[sel].at(0));
    viewer->setOption(0);
    viewer->show();
    viewer->read(solutionFiles[sel].at(0));
}
// callback on menu-obs-data-source -----------------------------------------
void Plot::showObservationText()
{
    char file[1024], tmpfile[1024];
    int cstat;

    trace(3, "showObservationText\n");

    if (observationFiles.count() <= 0) return;

    strncpy(file, qPrintable(observationFiles.at(0)), 1023);
    cstat = rtk_uncompress(file, tmpfile);

    viewer->setWindowTitle(observationFiles.at(0));
    viewer->setOption(0);
    viewer->show();
    viewer->read(!cstat ? file : tmpfile);

    if (cstat) remove(tmpfile);
}
// callback on menu-copy-to-clipboard ---------------------------------------
void Plot::copyPlotToClipboard()
{
    trace(3, "menuCopyClicked\n");

    QClipboard *clipboard = QApplication::clipboard();

    clipboard->setPixmap(buffer);
}
// callback on menu-options -------------------------------------------------
void Plot::showPlotOptionsDialog()
{
    double ooPosition_old[3];
    QString tlefile_old = plotOptDialog->getTleFile(), tlesatfile_old = plotOptDialog->getTleSatelliteFile();
    int timesyncout_old = plotOptDialog->getTimeSyncOut(), rcvpos_old = plotOptDialog->getReceiverPosition();

    trace(3, "showPlotOptionsDialog\n");

    matcpy(ooPosition_old, plotOptDialog->getOoPosition(), 3, 1);

    plotOptDialog->move(pos().x() + width() / 2 - plotOptDialog->width() / 2,
                        pos().y() + height() / 2 - plotOptDialog->height() / 2);

    plotOptDialog->exec();
    if (plotOptDialog->result() != QDialog::Accepted) return;

    saveOption();

    for (int i = 0; i < 3; i++) ooPosition_old[i] -= plotOptDialog->getOoPosition()[i];

    if (plotOptDialog->getTleFile() != tlefile_old) {
        free(tleData.data);
        tleData.data = NULL;
        tleData.n = tleData.nmax = 0;
        tle_read(qPrintable(plotOptDialog->getTleFile()), &tleData);
    }

    if (plotOptDialog->getTleFile() != tlefile_old || plotOptDialog->getTleSatelliteFile() != tlesatfile_old)
        tle_name_read(qPrintable(plotOptDialog->getTleSatelliteFile()), &tleData);

    if (rcvpos_old != plotOptDialog->getReceiverPosition() || norm(ooPosition_old, 3) > 1E-3 || plotOptDialog->getTleFile() != tlefile_old) {
        if (simulatedObservation)
            generateVisibilityData();
        else
            updateObservation(nObservation);
    }

    updateColor();
    updatePlotSizes();
    updateOrigin();
    updateStatusBarInformation();
    updateSatelliteMask();
    updateSatelliteList();

    refresh();
    timer.start(plotOptDialog->getRefreshCycle());

    // select correct y range
    for (int i = 0; i < lWRangeList->count(); i++) {
        bool ok;
        double range = lWRangeList->item(i)->data(Qt::UserRole+1).toDouble(&ok);
        if (ok) {
            if (fabs(range - plotOptDialog->getYRange()) < 0.0001) {
                lWRangeList->item(i)->setSelected(true);
                break;
            }
        }
    }
    if (!timesyncout_old && plotOptDialog->getTimeSyncOut()) {
        stropen(&streamTimeSync, STR_TCPSVR, STR_MODE_RW, qPrintable(QStringLiteral(":%d").arg(plotOptDialog->getTimeSyncPort())));
    }
    else if (timesyncout_old && !plotOptDialog->getTimeSyncOut()) {
        strclose(&streamTimeSync);
    }
}
// callback on menu-show-tool-bar -------------------------------------------
void Plot::updateToolBarVisibility()
{
    trace(3, "updateToolBarVisibility\n");

    ui->toolBar->setVisible(ui->menuToolBar->isChecked());

    updatePlotSizes();
    refresh();
}
// callback on menu-show-status-bar -----------------------------------------
void Plot::updateStatusBarVisibility()
{
    trace(3, "updateStatusBarVisibility\n");

    ui->statusbar->setVisible(ui->menuStatusBar->isChecked());
    updatePlotSizes();
    refresh();
}
// callback on menu-show-browse-panel ---------------------------------------
void Plot::updateBrowseBarVisibility()
{
    trace(3,"updateBrowseBarVisibility\n");

    ui->panelBrowse->setVisible(ui->menuBrowse->isChecked());

    ui->lblDisplay->updateGeometry();

    updatePlotSizes();
    refresh();
}
// callback on menu-waypoints -----------------------------------------------
void Plot::showWaypointsDialog()
{
    trace(3, "showWaypointsDialog\n");

    waypointsDialog->show();
}
// callback on menu-input-monitor-1 -----------------------------------------
void Plot::showMonitorConsole1()
{
    trace(3, "showMonitorConsole1\n");

    console1->setWindowTitle(tr("Monitor RT Input 1"));
    console1->show();
}
// callback on menu-input-monitor-2 -----------------------------------------
void Plot::showMonitorConsole2()
{
    trace(3, "showMonitorConsole2\n");

    console2->setWindowTitle(tr("Monitor RT Input 2"));
    console2->show();
}
// callback on menu-map-view ---------------------------------------
void Plot::showMapView()
{
    trace(3, "showMapView\n");

    mapView->setWindowTitle(
        QString(tr("%1 ver.%2 %3: Map View")).arg(PRGNAME,VER_RTKLIB, PATCH_LEVEL));
    mapView->show();
}
// callback on menu-center-origin -------------------------------------------
void Plot::centerOrigin()
{
    trace(3, "centerOrigin\n");

    setRange(0, getYRange());
    refresh();
}
// callback on menu-fit-horizontal ------------------------------------------
void Plot::fitHorizontally()
{
    trace(3, "fitHorizontally\n");

    if (plotType == PLOT_TRK)
        fitRange(0);
    else
        fitTime();

    refresh();
}
// callback on menu-fit-vertical --------------------------------------------
void Plot::fitVertically()
{
    trace(3, "fitVertically\n");

    fitRange(0);

    refresh();
}
// callback on menu-show-grid -----------------------------------------------
void Plot::updateShowGrid()
{
    trace(3,"menuShowGridClicked\n");

    updatePlot();
    updateEnable();
    refresh();
}
// callback on menu-show-track-points ---------------------------------------
void Plot::updateShowTrack()
{
    trace(3, "menuShowTrackClicked\n");

    if (!ui->menuShowTrack->isChecked()) {
        ui->menuFixHoriz->setChecked(false);
        ui->menuFixVert->setChecked(false);
    }
    updatePlot();
    updateEnable();
}
// callback on menu-fix-horizontal ------------------------------------------
void Plot::fixHorizontally()
{
    trace(3, "menuFixHorizontalClicked\n");

    centX = 0.0;
    updatePlot();
    updateEnable();
}
// callback on menu-windows-maximize ----------------------------------------
void Plot::showWindowMaximized()
{
    trace(3, "showWindowMaximized\n");

    this->showMaximized();

    mapView->hide();
}
// callback on menu-windows-mapview -----------------------------------------
void Plot::arrangePlotMapViewHorizontally()
{
    // arrange main window and map window side-by-side
    QScreen *scr = QApplication::screens().at(0);
    QRect rect = scr->availableGeometry();
    QSize thisDecoration = this->frameSize() - this->size();

    this->setGeometry(rect.x(), rect.y(), rect.width() / 2 - thisDecoration.width(), rect.height() - thisDecoration.height());

    QSize MapDecoration = mapView->frameSize() - mapView->size();
    mapView->setGeometry(rect.x() + rect.width() / 2, rect.y(),
                         rect.width()/2 - MapDecoration.width(), rect.height() - MapDecoration.height());

    mapView->setVisible(true);
}
//---------------------------------------------------------------------------
#if 0
void Plot::DispGesture()
{
    AnsiString s;
    int b, e;

    b = EventInfo.Flags.Contains(gfBegin);
    e = EventInfo.Flags.Contains(gfEnd);

    if (EventInfo.GestureID == Controls::igiZoom) {
        s.sprintf("zoom: Location=%d,%d,Flag=%d,%d,Angle=%.1f,Disnance=%d",
              EventInfo.Location.X, EventInfo.Location.Y, b, e,
              EventInfo.Angle, EventInfo.Distance);
        Message1->Caption = s;
    } else if (EventInfo.GestureID == Controls::igiPan) {
        s.sprintf("pan: Location=%d,%d,Flag=%d,%d,Angle=%.1f,Disnance=%d",
              EventInfo.Location.X, EventInfo.Location.Y, b, e,
              EventInfo.Angle, EventInfo.Distance);
        Message1->Caption = s;
    } else if (EventInfo.GestureID == Controls::igiRotate) {
        s.sprintf("rotate: Location=%d,%d,Flag=%d,%d,Angle=%.1f,Disnance=%d",
              EventInfo.Location.X, EventInfo.Location.Y, b, e,
              EventInfo.Angle, EventInfo.Distance);
        Message1->Caption = s;
    }
}
#endif
// callback on menu-animation-start -----------------------------------------
void Plot::menuAnimationStartClicked()
{
    trace(3, "menuAnimationStartClicked\n");
}
// callback on menu-animation-stop ------------------------------------------
void Plot::menuAnimationStopClicked()
{
    trace(3, "menuAnimationStopClicked\n");
}
// callback on menu-about ---------------------------------------------------
void Plot::showAboutDialog()
{
    trace(3, "menuAboutClick\n");

    aboutDialog->exec();
}
// callback on button-connect/disconnect ------------------------------------
void Plot::toggleConnectState()
{
    trace(3, "toggleConnectState\n");

    if (!connectState)
        connectStream();
    else
        disconnectStream();
}
// callback on button-solution-1 --------------------------------------------
void Plot::activateSolution1()
{
    trace(3, "activateSolution1\n");

    if (solutionData[0].n <= 0) {
        ui->btnSolution1->setChecked(false);
        return;
    }

    ui->btnSolution12->setChecked(false);

    updateTime();
    updatePlot();
    updateEnable();
}
// callback on button-solution-2 --------------------------------------------
void Plot::activateSolution2()
{
    trace(3, "activateSolution2\n");

    if (solutionData[1].n <= 0) {
        ui->btnSolution2->setChecked(false);
        return;
    }

    ui->btnSolution12->setChecked(false);

    updateTime();
    updatePlot();
    updateEnable();
}
// callback on button-solution-1-2 ------------------------------------------
void Plot::activateSolution12()
{
    trace(3, "activateSolution12\n");

    ui->btnSolution1->setChecked(false);
    ui->btnSolution2->setChecked(false);

    updateTime();
    updatePlot();
    updateEnable();
}
// --------------------------------------------------------------------------
void Plot::updatePlotSizeAndRefresh()
{
    trace(3, "updatePlotSizeAndRefresh\n");

    updatePlotSizes();
    refresh();
}
// --------------------------------------------------------------------------
double Plot::getYRange()
{
    bool okay;
    QListWidgetItem *i;
    double defaultYRange = 5;

    if ((i = lWRangeList->currentItem()) == NULL) return defaultYRange;

    float yRange = i->data(Qt::UserRole+1).toDouble(&okay);

    if (!okay) return defaultYRange;

    return yRange;
}

// callback on button-range-list --------------------------------------------
void Plot::rangeListItemSelected()
{
    trace(3, "rangeListItemSelected\n");

    lWRangeList->close();
    rangeListPopupAction->toggle();

    setRange(0, getYRange());
    updatePlot();
    updateEnable();
}
// callback on button-message 2 ---------------------------------------------
void Plot::changePointCoordinateType()
{
    if (++pointCoordinateType > 2) pointCoordinateType = 0;
}
// callback on plot-type selection change -----------------------------------
void Plot::updateSelectedPlotType()
{
    int i;

    trace(3, "updateSelectedPlotType\n");

    i = ui->cBPlotTypeSelection->currentData().toInt();
    updatePlotType(i);

    updateTime();
    updatePlot();
    updateEnable();
}
// -------------------------------------------------------------------------
void Plot::updatePlotAndEnable()
{
    trace(3, "updatePlotAndEnable\n");

    updatePlot();
    updateEnable();
}
// callback on satellite-list selection change ------------------------------
void Plot::satelliteListChanged()
{
    trace(3, "satelliteListChanged\n");

    updateSatelliteSelection();
    updatePlot();
    updateEnable();
}
// callback on time scroll-bar change ---------------------------------------
void Plot::updateCurrentObsSol()
{
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "updateCurrentObsSol\n");

    if (plotType <= PLOT_NSAT || plotType == PLOT_RES)
        solutionIndex[sel] = ui->sBTime->value();
    else
        observationIndex = ui->sBTime->value();
    updatePlot();
}
// callback on mouse-down event ---------------------------------------------
void Plot::mousePressEvent(QMouseEvent *event)
{
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
    dragStartX = mapFromGlobal(event->globalPosition()).x();
    dragStartY = mapFromGlobal(event->globalPosition()).y();
#else
    dragStartX = mapFromGlobal(event->globalPos()).x();
    dragStartY = mapFromGlobal(event->globalPos()).y();
#endif
    dragCentX = centX;

    trace(3, "mousePressEvent: X=%d Y=%d\n", dragStartX, dragStartY);

    dragState = event->buttons().testFlag(Qt::LeftButton) ? 1 : (event->buttons().testFlag(Qt::RightButton) ? 11 : 0);

    if (plotType == PLOT_TRK)
        mouseDownTrack(dragStartX, dragStartY);
    else if (plotType <= PLOT_NSAT || plotType == PLOT_RES || plotType == PLOT_SNR)
        mouseDownSolution(dragStartX, dragStartY);
    else if (plotType == PLOT_OBS || plotType == PLOT_DOP)
        mouseDownObservation(dragStartX, dragStartY);
    else
        dragState = 0;
}
// callback on mouse-move event ---------------------------------------------
void Plot::mouseMove(QMouseEvent *event)
{
    double dx, dy, dxs, dys;

#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
    if ((abs(mapFromGlobal(event->globalPosition()).x() - dragCurrentX) < 1) &&
        (abs(mapFromGlobal(event->globalPosition()).y() - dragCurrentY) < 1)) return;

    dragCurrentX = mapFromGlobal(event->globalPosition()).x();
    dragCurrentY = mapFromGlobal(event->globalPosition()).y();
#else
    if ((abs(mapFromGlobal(event->globalPos()).x() - dragCurrentX) < 1) &&
        (abs(mapFromGlobal(event->globalPos()).y() - dragCurrentY) < 1)) return;

    dragCurrentX = mapFromGlobal(event->globalPos()).x();
    dragCurrentY = mapFromGlobal(event->globalPos()).y();
#endif
    trace(4, "mouseMove: X=%d Y=%d\n", dragCurrentX, dragCurrentY);

    if (dragState == 0) {
        updatePoint(dragCurrentX, dragCurrentY);
        return;
    }

    dx = (dragStartX - dragCurrentX) * dragScaleX;
    dy = (dragCurrentY - dragStartY) * dragScaleY;
    dxs = pow(2.0, (dragStartX - dragCurrentX) / 100.0);
    dys = pow(2.0, (dragCurrentY - dragStartY) / 100.0);

    if (plotType == PLOT_TRK)
        mouseMoveTrack(dragCurrentX, dragCurrentY, dx, dy, dxs, dys);
    else if (plotType <= PLOT_NSAT || plotType == PLOT_RES || plotType == PLOT_SNR)
        mouseMoveSolution(dragCurrentX, dragCurrentY, dx, dy, dxs, dys);
    else if (plotType == PLOT_OBS || plotType == PLOT_DOP)
        mouseMoveObservation(dragCurrentX, dragCurrentY, dx, dy, dxs, dys);
}
// callback on mouse-up event -----------------------------------------------
void Plot::mouseReleaseEvent(QMouseEvent *event)
{
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
    trace(3, "mouseReleaseEvent: X=%d Y=%d\n", mapFromGlobal(event->globalPosition()).x(), mapFromGlobal(event->globalPosition()).y());
#else
    trace(3, "mouseReleaseEvent: X=%d Y=%d\n", mapFromGlobal(event->globalPos()).x(), mapFromGlobal(event->globalPos()).y());
#endif
    if (dragState == 0) return;
    dragState = 0;

    setCursor(Qt::ArrowCursor);
    refresh();
    refresh_MapView();
}
// callback on mouse-double-click -------------------------------------------
void Plot::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) return;

    QPoint p(static_cast<int>(dragStartX), static_cast<int>(dragStartY));
    double x, y;

    trace(3, "mouseDoubleClickEvent X=%d Y=%d\n", p.x(), p.y());

    if (ui->btnFixHorizontal->isChecked()) return;

    if (plotType == PLOT_TRK) {
        graphTrack->toPos(p, x, y);
        graphTrack->setCenter(x, y);

        refresh();
        refresh_MapView();
    } else if (plotType <= PLOT_NSAT || plotType == PLOT_RES || plotType == PLOT_SNR) {
        graphTriple[0]->toPos(p, x, y);

        setCenterX(x);
        refresh();
    } else if (plotType == PLOT_OBS || plotType == PLOT_DOP) {
        graphSingle->toPos(p, x, y);

        setCenterX(x);
        refresh();
    }
}
// callback on mouse-leave event --------------------------------------------
void Plot::leaveEvent(QEvent *)
{
    trace(3, "leaveEvent\n");

    dragCurrentX = dragCurrentY = -1;
    ui->lblMessage2->setVisible(false);
    ui->lblMessage2->setText("");
}
// callback on mouse-down event on track-plot -------------------------------
void Plot::mouseDownTrack(int x, int y)
{
    int i, sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "mouseDownTrack: X=%d Y=%d\n", x, y);

    if (dragState == 1 && (i = searchPosition(x, y)) >= 0) {
        solutionIndex[sel] = i;

        dragState = 0;

        updateTime();
        updateStatusBarInformation();        
        refresh();
    } else {
        graphTrack->getCenter(dragCenterX, dragCenterY);
        graphTrack->getScale(dragScaleX, dragScaleY);

        setCursor(dragState == 1 ? Qt::SizeAllCursor : Qt::SplitVCursor);
    }
}
// callback on mouse-down event on solution-plot ----------------------------
void Plot::mouseDownSolution(int x, int y)
{
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3};
    QPoint pnt, p(x, y);
    gtime_t time = {0, 0};
    sol_t *data;
    double x_pos, xl[2], yl[2];
    int i, area = -1, sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "mouseDownSolution: X=%d Y=%d\n", x, y);

    if (plotType == PLOT_SNR) {
        if (0 <= observationIndex && observationIndex < nObservation)
            time = observation.data[indexObservation[observationIndex]].time;
    } else {
        if ((data = getsol(solutionData + sel, solutionIndex[sel])))
            time = data->time;
    }
    if (time.time && !ui->menuFixHoriz->isChecked()) {
        x_pos = timePosition(time);

        graphTriple[0]->getLimits(xl, yl);
        graphTriple[0]->toPoint(x_pos, yl[1], pnt);

        if ((x - pnt.x()) * (x - pnt.x()) + (y - pnt.y()) * (y - pnt.y()) < 5*5) {
            setCursor(Qt::SizeHorCursor);

            dragState = 20;
            refresh();

            return;
        }
    }
    for (i = 0; i < 3; i++) {
        if (!btn[i]->isChecked() || (i != 1 && plotType == PLOT_SNR)) continue;

        graphTriple[i]->getCenter(dragCenterX, dragCenterY);
        graphTriple[i]->getScale(dragScaleX, dragScaleY);
        area = graphTriple[i]->onAxis(mapFromGlobal(p));

        if (dragState == 1 && area == 0) { // within plot
            setCursor(Qt::SizeAllCursor);
            dragState += i;
            return;
        } else if (area == 1) { // left of plot (on y axis)
            setCursor(dragState == 1 ? Qt::SizeVerCursor : Qt::SplitVCursor);
            dragState += i + 4;
            return;
        } else if (area == 0) {
            break;
        }
    }
    if (area == 0 || area == 8) {  // within or below plot
        setCursor(dragState == 1 ? Qt::SizeHorCursor : Qt::SplitHCursor);
        dragState += 3;
    } else {
        dragState = 0;
    }
}
// callback on mouse-down event on observation-data-plot --------------------
void Plot::mouseDownObservation(int x, int y)
{
    QPoint pnt, p(x, y);
    double x_pos, xl[2], yl[2];
    int area;

    trace(3, "mouseDownObservation: X=%d Y=%d\n", x, y);

    if (0 <= observationIndex && observationIndex < nObservation && !ui->menuFixHoriz->isChecked()) {
        x_pos = timePosition(observation.data[indexObservation[observationIndex]].time);

        graphSingle->getLimits(xl, yl);
        graphSingle->toPoint(x_pos, yl[1], pnt);

        if ((x - pnt.x()) * (x - pnt.x()) + (y - pnt.y()) * (y - pnt.y()) < 5*5) {
            setCursor(Qt::SizeHorCursor);
            dragState = 20;
            refresh();
            return;
        }
    }
    graphSingle->getCenter(dragCenterX, dragCenterY);
    graphSingle->getScale(dragScaleX, dragScaleY);
    area = graphSingle->onAxis(mapFromGlobal(p));

    if (area == 0 || area == 8) {  // within or below plot
        setCursor(dragState == 1 ? Qt::SizeHorCursor : Qt::SplitHCursor);
        dragState += 3;
    } else {
        dragState = 0;
    }
}
// callback on mouse-move event on track-plot -------------------------------
void Plot::mouseMoveTrack(int x, int y, double dx, double dy,
                          double dxs, double dys)
{
    trace(4, "mouseMoveTrack: X=%d Y=%d\n", x, y);

    Q_UNUSED(dxs);

    if (dragState == 1 && !ui->menuFixHoriz->isChecked())
        graphTrack->setCenter(dragCenterX + dx, dragCenterY + dy);
    else if (dragState > 1)
        graphTrack->setScale(dragScaleX * dys, dragScaleY * dys);

    ui->menuCenterOrigin->setChecked(false);

    if (updateTimer.elapsed() < plotOptDialog->getRefreshCycle()) return;

    updateTimer.restart();

    refresh();
}
// callback on mouse-move event on solution-plot ----------------------------
void Plot::mouseMoveSolution(int x, int y, double dx, double dy,
                            double dxs, double dys)
{
    QPoint p1, p2, p = mapFromGlobal(QPoint(x, y));
    double cx, cy, xs, ys;
    int i, sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(4, "mouseMoveSolution: X=%d Y=%d\n", x, y);

    if (dragState <= 4) {
        for (int panel = 0; panel < 3; panel++) {
            graphTriple[panel]->getCenter(cx, cy);
            if (!ui->menuFixHoriz->isChecked())
                cx = dragCenterX + dx;
            if (!ui->menuFixVert->isChecked() || !ui->menuFixVert->isEnabled())
                cy = (panel == dragState - 1) ? dragCenterY + dy : cy;
            graphTriple[panel]->setCenter(cx, cy);
            setCenterX(cx);
        }
        if (ui->menuFixHoriz->isChecked()) {
            graphTriple[0]->getExtent(p1, p2);
            centX = dragCentX + 2.0 * (x - dragStartX) / (p2.x() - p1.x());
            if (centX > 1.0) centX = 1.0;
            if (centX < -1.0) centX = -1.0;
        }
    } else if (dragState <= 7) {
        graphTriple[dragState - 5]->getCenter(cx, cy);
        if (!ui->menuFixVert->isChecked() || !ui->menuFixVert->isEnabled())
            cy = dragCenterY + dy;
        graphTriple[dragState - 5]->setCenter(cx, cy);
    } else if (dragState <= 14) {
        for (int panel = 0; panel < 3; panel++) {
            graphTriple[panel]->getScale(xs, ys);
            graphTriple[panel]->setScale(dragScaleX * dxs, ys);
        }
        setScaleX(dragScaleX * dxs);
    } else if (dragState <= 17) {
        graphTriple[dragState - 15]->getScale(xs, ys);
        graphTriple[dragState - 15]->setScale(xs, dragScaleY * dys);
    } else if (dragState == 20) {
        graphTriple[0]->toPos(p, cx, cy);
        if (plotType == PLOT_SNR) {
            for (i = 0; i < nObservation; i++)
                if (timePosition(observation.data[indexObservation[i]].time) >= cx) break;
            observationIndex = i < nObservation ? i : nObservation - 1;
        } else {
            for (i = 0; i < solutionData[sel].n; i++)
                if (timePosition(solutionData[sel].data[i].time) >= cx) break;
            solutionIndex[sel] = i < solutionData[sel].n ? i : solutionData[sel].n - 1;
        }
        updateTime();
    }
    ui->menuCenterOrigin->setChecked(false);

    if (updateTimer.elapsed() < plotOptDialog->getRefreshCycle()) return;
    updateTimer.restart();

    refresh();
}
// callback on mouse-move events on observataion-data-plot ------------------
void Plot::mouseMoveObservation(int x, int y, double dx, double dy,
            double dxs, double dys)
{
    QPoint p1, p2, p = mapFromGlobal(QPoint(x, y));
    double cx, cy, xs, ys;
    int i;

    Q_UNUSED(dys);

    trace(4, "mouseMoveObservation: X=%d Y=%d\n", x, y);

    if (dragState <= 4) {
        graphSingle->getCenter(cx, cy);
        if (!ui->menuFixHoriz->isChecked()) cx = dragCenterX + dx;
        if (!ui->menuFixVert->isChecked()) cy = dragCenterY + dy;
        graphSingle->setCenter(cx, cy);
        setCenterX(cx);

        if (ui->menuFixHoriz->isChecked()) {
            graphSingle->getExtent(p1, p2);
            centX = dragCentX + 2.0 * (x - dragStartX) / (p2.x() - p1.x());
            if (centX > 1.0) centX = 1.0;
            if (centX < -1.0) centX = -1.0;
        }
    } else if (dragState <= 14) {
        graphSingle->getScale(xs, ys);
        graphSingle->setScale(dragScaleX * dxs, ys);
        setScaleX(dragScaleX * dxs);
    } else if (dragState == 20) {
        graphSingle->toPos(p, cx, cy);
        for (i = 0; i < nObservation; i++)
            if (timePosition(observation.data[indexObservation[i]].time) >= cx) break;
        observationIndex = i < nObservation ? i : nObservation - 1;
        updateTime();
    }
    ui->menuCenterOrigin->setChecked(false);

    if (updateTimer.elapsed() < plotOptDialog->getRefreshCycle()) return;
    updateTimer.restart();

    refresh();
}
// callback on mouse-wheel events -------------------------------------------
void Plot::wheelEvent(QWheelEvent *event)
{
    QPoint p;
    double xs, ys, ds = pow(2.0, -event->angleDelta().y() / 1200.0);
    int panel, area = -1;

    event->accept();

#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
    p = ui->lblDisplay->mapFromGlobal(event->globalPosition()).toPoint();
#else
    p = ui->lblDisplay->mapFromGlobal(event->globalPosition().toPoint());
#endif

    trace(4, "wheelEvent: WheelDelta=%d\n", event->angleDelta().y());

    if (p.x() < 0 || p.y() < 0) return;

    if (plotType == PLOT_TRK) { // track-plot
        graphTrack->getScale(xs, ys);
        graphTrack->setScale(xs * ds, ys * ds);
    } else if (plotType <= PLOT_NSAT || plotType == PLOT_RES || plotType == PLOT_SNR) {
        for (panel = 0; panel < 3; panel++) {
            if (plotType == PLOT_SNR && panel != 1) continue;

            area = graphTriple[panel]->onAxis(p);
            if (area == 0 || area == 1 || area == 2) {
                graphTriple[panel]->getScale(xs, ys);
                graphTriple[panel]->setScale(xs, ys * ds);
            }
        }
        if (area == 8) {
            for (panel = 0; panel < 3; panel++) {
                graphTriple[panel]->getScale(xs, ys);
                graphTriple[panel]->setScale(xs * ds, ys);
                setScaleX(xs * ds);
            }
        }
    } else if (plotType == PLOT_OBS || plotType == PLOT_DOP) {
        area = graphSingle->onAxis(p);
        if (area == 0 || area == 8) {
            graphSingle->getScale(xs, ys);
            graphSingle->setScale(xs * ds, ys);
            setScaleX(xs * ds);
        }
    } else {
        return;
    }

    refresh();
}
// callback on key-down events ----------------------------------------------
void Plot::keyPressEvent(QKeyEvent *event)
{
    double scaleFactor = 1.05, factor = event->modifiers().testFlag(Qt::ShiftModifier) ? 1.0 : 10.0;
    double xc, yc, yc1, yc2, yc3, xs, ys, ys1, ys2, ys3;

    trace(3, "keyPressEvent:\n");

    if (event->modifiers().testFlag(Qt::AltModifier)) return;

    if (plotType == PLOT_TRK) {
        graphTrack->getCenter(xc, yc);
        graphTrack->getScale(xs, ys);
        if (event->key() == Qt::Key_Up) {
            if (!ui->menuFixHoriz->isChecked()) yc += factor * ys;
        }
        if (event->key() == Qt::Key_Down) {
            if (!ui->menuFixHoriz->isChecked()) yc -= factor * ys;
        }
        if (event->key() == Qt::Key_Left) {
            if (!ui->menuFixHoriz->isChecked()) xc -= factor * xs;
        }
        if (event->key() == Qt::Key_Right) {
            if (!ui->menuFixHoriz->isChecked()) xc += factor * xs;
        }
        if ((event->key() == Qt::Key_Up) && (event->modifiers().testFlag(Qt::ControlModifier))) {
            xs /= scaleFactor; ys /= scaleFactor;
        }
        if ((event->key() == Qt::Key_Down) && (event->modifiers().testFlag(Qt::ControlModifier))) {
            xs *= scaleFactor; ys *= scaleFactor;
        }
        graphTrack->setCenter(xc, yc);
        graphTrack->setScale(xs, ys);
    } else if (plotType <= PLOT_NSAT || plotType == PLOT_RES) {
        graphTriple[0]->getCenter(xc, yc1);
        graphTriple[1]->getCenter(xc, yc2);
        graphTriple[2]->getCenter(xc, yc3);
        graphTriple[0]->getScale(xs, ys1);
        graphTriple[1]->getScale(xs, ys2);
        graphTriple[2]->getScale(xs, ys3);
        if (event->key() == Qt::Key_Up) {
            if (!ui->menuFixVert->isChecked()) yc1 += factor * ys1;
            yc2 += factor * ys2;
            yc3 += factor * ys3;
        }
        if (event->key() == Qt::Key_Down) {
            if (!ui->menuFixVert->isChecked()) yc1 -= factor * ys1;
            yc2 -= factor * ys2;
            yc3 -= factor * ys3;
        }
        if (event->key() == Qt::Key_Left) {
            if (!ui->menuFixHoriz->isChecked()) xc -= factor * xs;
        }
        if (event->key() == Qt::Key_Right) {
            if (!ui->menuFixHoriz->isChecked()) xc += factor * xs;
        }
        if ((event->key() == Qt::Key_Up) && (event->modifiers().testFlag(Qt::ControlModifier))) {
            ys1 /= scaleFactor;
            ys2 /= scaleFactor;
            ys3 /= scaleFactor;
        }
        if ((event->key() == Qt::Key_Down) && (event->modifiers().testFlag(Qt::ControlModifier))) {
            ys1 *= scaleFactor;
            ys2 *= scaleFactor;
            ys3 *= scaleFactor;
        }
        if ((event->key() == Qt::Key_Left) && (event->modifiers().testFlag(Qt::ControlModifier)))
            xs *= scaleFactor;
        if ((event->key() == Qt::Key_Right) && (event->modifiers().testFlag(Qt::ControlModifier)))
            xs /= scaleFactor;
        graphTriple[0]->setCenter(xc, yc1);
        graphTriple[1]->setCenter(xc, yc2);
        graphTriple[2]->setCenter(xc, yc3);
        graphTriple[0]->setScale(xs, ys1);
        graphTriple[1]->setScale(xs, ys2);
        graphTriple[2]->setScale(xs, ys3);
    } else if (plotType == PLOT_OBS || plotType == PLOT_DOP || plotType == PLOT_SNR) {
        graphSingle->getCenter(xc, yc);
        graphSingle->getScale(xs, ys);
        if (event->key() == Qt::Key_Up) {
            if (!ui->menuFixVert->isChecked()) yc += factor * ys;
        }
        if (event->key() == Qt::Key_Down) {
            if (!ui->menuFixVert->isChecked()) yc -= factor * ys;
        }
        if (event->key() == Qt::Key_Left) {
            if (!ui->menuFixHoriz->isChecked()) xc -= factor * xs;
        }
        if (event->key() == Qt::Key_Right) {
            if (!ui->menuFixHoriz->isChecked()) xc += factor * xs;
        }
        if ((event->key() == Qt::Key_Up) && (event->modifiers().testFlag(Qt::ControlModifier)))
            ys /= scaleFactor;
        if ((event->key() == Qt::Key_Down) && (event->modifiers().testFlag(Qt::ControlModifier)))
            ys *= scaleFactor;
        if ((event->key() == Qt::Key_Left) && (event->modifiers().testFlag(Qt::ControlModifier)))
            xs *= scaleFactor;
        if ((event->key() == Qt::Key_Right) && (event->modifiers().testFlag(Qt::ControlModifier)))
            xs /= scaleFactor;

        graphSingle->setCenter(xc, yc);
        graphSingle->setScale(xs, ys);
    }
    refresh();
}
// callback on interval-timer -----------------------------------------------
void Plot::timerTimer()
{
    const QColor color[] = {Qt::red, Qt::gray, Color::Orange, Qt::green, Color::Lime};
    QLabel *lblStreamStatus[] = {ui->lblStreamStatus1, ui->lblStreamStatus2};
    Console *console[] = {console1, console2};
    QString connectmsg ;
    static uint8_t buff[16384];
    solopt_t solopt = solopt_default;
    sol_t *sol;
    const gtime_t ts = {0, 0};
    gtime_t time = {0, 0};
    double tint = timeEnabled[2] ? timeInterval : 0.0, pos[3], ep[6];
    int i, j, n, inb, inr, cycle, nmsg[2] = {0}, stat, istat;
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;
    char msg[MAXSTRMSG] = "";

    trace(4, "timerTimer\n");

    if (connectState) { // real-time input mode
        for (int streamNo = 0; streamNo < 2; streamNo++) {
            solopt.posf = rtFormat[streamNo];
            solopt.times = (rtTimeFormat == 0) ? TIMES_GPST : (rtTimeFormat - 1);
            solopt.timef = rtTimeFormat >= 1;
            solopt.degf = rtDegFormat;
            strncpy(solopt.sep, qPrintable(rtFieldSeperator), 63);
            strsum(stream + streamNo, &inb, &inr, NULL, NULL);
            stat = strstat(stream + streamNo, msg);
            setWidgetTextColor(lblStreamStatus[streamNo], color[stat < 3 ? stat + 1 : 3]);
            if (*msg && strcmp(msg, "localhost"))
                connectmsg += QStringLiteral("(%1) %2 ").arg(streamNo + 1).arg(msg);
            while ((n = strread(stream + streamNo, buff, sizeof(buff))) > 0) {
                for (j = 0; j < n; j++) {
                    istat = inputsol(buff[j], ts, ts, tint, SOLQ_NONE, &solopt, solutionData + streamNo);
                    if (istat == 0) continue;
                    if (istat < 0) { // disconnect received
                        disconnectStream();
                        return;
                    }
                    if (week == 0 && solutionData[streamNo].n == 1) { // first data
                        if (plotType > PLOT_NSAT)
                            updatePlotType(PLOT_TRK);
                        time2gpst(solutionData[streamNo].time, &week);
                        updateOrigin();
                        ecef2pos(solutionData[streamNo].data[0].rr, pos);
                        mapView->setCenter(pos[0] * R2D, pos[1] * R2D);
                    }
                    nmsg[streamNo]++;
                }
                console[streamNo]->addMessage(buff, n);
            }
            if (nmsg[streamNo] > 0) {
                setWidgetTextColor(lblStreamStatus[streamNo], color[4]);
                solutionIndex[streamNo] = solutionData[streamNo].n - 1;
            }
        }
        ui->lblConnectMessage->setText(connectmsg);
        if (nmsg[0] <= 0 && nmsg[1] <= 0) return;
    } else if (ui->btnAnimate->isEnabled() && ui->btnAnimate->isChecked()) { // animation mode
        cycle = plotOptDialog->getAnimationCycle() <= 0 ? 1 : plotOptDialog->getAnimationCycle();

        if (plotType <= PLOT_NSAT || plotType == PLOT_RES) {
            solutionIndex[sel] += cycle;
            if (solutionIndex[sel] >= solutionData[sel].n - 1) {
                solutionIndex[sel] = solutionData[sel].n - 1;
                ui->btnAnimate->setChecked(false);
            }
        } else {
            observationIndex += cycle;
            if (observationIndex >= nObservation - 1) {
                observationIndex = nObservation - 1;
                ui->btnAnimate->setChecked(false);
            }
        }
    } else if (plotOptDialog->getTimeSyncOut()) { // time sync
        time.time = 0;
        while (strread(&streamTimeSync, (uint8_t *)streamBuffer + nStreamBuffer, 1)) {
            if (++nStreamBuffer >= 1023) {
                nStreamBuffer = 0;
                continue;
            }
            if (streamBuffer[nStreamBuffer - 1] == '\n') {
                streamBuffer[nStreamBuffer - 1] = '\0';
                if (sscanf(streamBuffer, "%lf/%lf/%lf %lf:%lf:%lf", ep, ep + 1, ep + 2,
                           ep + 3, ep + 4, ep + 5) >= 6) {
                    time = epoch2time(ep);
                }
                nStreamBuffer = 0;
            }
        }
        if (time.time && (plotType <= PLOT_NSAT || plotType <= PLOT_RES)) {
           i = solutionIndex[sel];
           if (!(sol = getsol(solutionData + sel, i))) return;

           double tt = timediff(sol->time, time);
           if (tt < -DTTOL) {
               for (;i < solutionData[sel].n; i++) {
                   if (!(sol = getsol(solutionData + sel,i))) continue;
                   if (timediff(sol->time, time) >= -DTTOL) {
                       i--;
                       break;
                   }
               }
           } else if (tt > DTTOL) {
               for (;i >= 0; i--) {
                   if (!(sol = getsol(solutionData + sel,i))) continue;
                   if (timediff(sol->time, time) <= DTTOL) break;
               }
           }
           solutionIndex[sel] = qMax(0, qMin(solutionData[sel].n - 1, i));
        }
        else return;
    }
    else {
        return;
    }

    updateTime();

    if (updateTimer.elapsed() < plotOptDialog->getRefreshCycle()) return;
    updateTimer.restart();

    updatePlot();
}
// set center of x-axis -----------------------------------------------------
void Plot::setCenterX(double c)
{
    double x, y;
    int panel;

    trace(3, "SetCenterX: c=%.3f:\n", c);

    graphSingle->getCenter(x, y);
    graphSingle->setCenter(c, y);
    for (panel = 0; panel < 3; panel++) {
        graphTriple[panel]->getCenter(x, y);
        graphTriple[panel]->setCenter(c, y);
    }
}
// set scale of x-axis ------------------------------------------------------
void Plot::setScaleX(double s)
{
    double xs, ys;
    int panel;

    trace(3, "SetScaleX: s=%.3f:\n", s);

    graphSingle->getScale(xs, ys);
    graphSingle->setScale(s, ys);
    for (panel = 0; panel < 3; panel++) {
        graphTriple[panel]->getScale(xs, ys);
        graphTriple[panel]->setScale(s, ys);
    }
}
// update plot-type with fit-range ------------------------------------------
void Plot::updatePlotType(int type)
{
    trace(3, "updatePlotType: type=%d\n", type);

    plotType = type;

    if (plotOptDialog->getAutoScale() && plotType <= PLOT_SOLA && (solutionData[0].n > 0 || solutionData[1].n > 0))
        fitRange(0);
    else
        setRange(0, getYRange());

    updatePlotTypeMenu();
}
// update size of plot ------------------------------------------------------
void Plot::updatePlotSizes()
{
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3};
    QPoint p1(0, 0), p2(ui->lblDisplay->width(), ui->lblDisplay->height());
    double xs, ys, font_px = plotOptDialog->getFont().pointSize()*1.33;
    int i, numPanels, h, tmargin, bmargin, rmargin, lmargin;

    trace(3, "updatePlotSizes\n");

    tmargin = (int)(font_px * 0.9); // top margin (px)
    bmargin = (int)(font_px * 1.8); // bottom
    rmargin = (int)(font_px * 1.2); // right
    lmargin = (int)(font_px * 4.6); // left

    graphTrack->setPosition(p1, p2);

    graphSky->setPosition(p1, p2);
    graphSky->getScale(xs, ys);
    xs = qMax(xs, ys);
    graphSky->setScale(xs, xs);

    p1.rx() += lmargin;
    p1.ry() += tmargin;
    p2.rx() -= rmargin;
    p2.ry() -= bmargin;
    graphSingle->setPosition(p1, p2);

    p1.setY(tmargin);
    p2.setY(p1.y());

    // tripple panel
    for (i = numPanels = 0; i < 3; i++) if (btn[i]->isChecked()) numPanels++;
    for (i = 0; i < 3; i++) {
        if (!btn[i]->isChecked() || (numPanels <= 0)) continue;
        h = (ui->lblDisplay->height() - tmargin - bmargin) / numPanels;
        p2.ry() += h;
        graphTriple[i]->setPosition(p1, p2);
        p1.ry() += h;
    }

    // dual panel
    p1.setY(tmargin);
    p2.setY(p1.y());
    for (i = numPanels = 0; i < 2; i++) if (btn[i]->isChecked()) numPanels++;
    for (i = 0; i < 2; i++) {
        if (!btn[i]->isChecked() || (numPanels <= 0)) continue;
        h = (ui->lblDisplay->height() - tmargin - bmargin) / numPanels;
        p2.ry() += h;
        graphDual[i]->setPosition(p1, p2);
        p1.ry() += h;
    }
}
// update colors on plot ----------------------------------------------------
void Plot::updateColor()
{
    int i;

    trace(3, "updateColor\n");

    for (i = 0; i < 3; i++) {
        graphTrack->color[i] = plotOptDialog->getCColor(i);
        graphSingle->color[i] = plotOptDialog->getCColor(i);
        graphSky->color[i] = plotOptDialog->getCColor(i);
        graphTriple[0]->color[i] = plotOptDialog->getCColor(i);
        graphTriple[1]->color[i] = plotOptDialog->getCColor(i);
        graphTriple[2]->color[i] = plotOptDialog->getCColor(i);
    }
    ui->lblDisplay->setFont(font);
}
// update time-cursor -------------------------------------------------------
void Plot::updateTime()
{
    gtime_t time;
    sol_t *sol;
    double tt;
    int i, sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "updateTime\n");

    if (observationIndex < 0) return;

    // time-cursor change on solution-plot
    if (plotType <= PLOT_NSAT || plotType <= PLOT_RES) {
        ui->sBTime->setMaximum(qMax(1, solutionData[sel].n - 1));
        ui->sBTime->setValue(solutionIndex[sel]);
        if (!(sol = getsol(solutionData + sel, solutionIndex[sel]))) return;
        time = sol->time;
    } else if (nObservation > 0) { // time-cursor change on observation-data-plot
        ui->sBTime->setMaximum(qMax(1, nObservation - 1));
        ui->sBTime->setValue(observationIndex);
        time = observation.data[indexObservation[observationIndex]].time;
    } else {
        return;
    }

    // time-synchronization between solutions and observation-data
    for (sel = 0; sel < 2; sel++) {
        i = solutionIndex[sel];
        if (!(sol = getsol(solutionData + sel, i))) continue;
        tt = timediff(sol->time, time);
        if (tt < -DTTOL) {
            for (; i < solutionData[sel].n; i++) {
                if (!(sol = getsol(solutionData + sel, i))) continue;
                if (timediff(sol->time, time) >= -DTTOL) break;
            }
        } else if (tt > DTTOL) {
            for (; i >= 0; i--) {
                if (!(sol = getsol(solutionData + sel, i))) continue;
                if (timediff(sol->time, time) <= DTTOL) break;
            }
        }
        solutionIndex[sel] = qMax(0, qMin(solutionData[sel].n - 1, i));
    }
    i = observationIndex;
    if (i <= nObservation - 1) {
        tt = timediff(observation.data[indexObservation[i]].time, time);
        if (tt < -DTTOL) {
            for (; i < nObservation; i++)
                if (timediff(observation.data[indexObservation[i]].time, time) >= -DTTOL) break;
        } else if (tt > DTTOL) {
            for (; i >= 0; i--)
                if (timediff(observation.data[indexObservation[i]].time, time) <= DTTOL) break;
        }
        observationIndex = qMax(0, qMin(nObservation - 1, i));
    }
}
// update origin of plot ----------------------------------------------------
void Plot::updateOrigin()
{
    gtime_t time = {0, 0};
    sol_t *sol;
    double opos[3] = {0}, pos[3], ovel[3] = {0};
    int i, j, n = 0, sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;
    QString sta;

    trace(3, "updateOrigin\n");

    if (plotOptDialog->getOrigin() == ORG_STARTPOS) {
        if (!(sol = getsol(solutionData, 0)) || sol->type != 0) return;
        for (i = 0; i < 3; i++)
            opos[i] = sol->rr[i];
    } else if (plotOptDialog->getOrigin() == ORG_ENDPOS) {
        if (!(sol = getsol(solutionData, solutionData[0].n - 1)) || sol->type != 0) return;
        for (i = 0; i < 3; i++)
            opos[i] = sol->rr[i];
    } else if (plotOptDialog->getOrigin() == ORG_AVEPOS) {
        for (i = 0; (sol = getsol(solutionData, i)) != NULL; i++) {
            if (sol->type != 0) continue;
            for (j = 0; j < 3; j++)
                opos[j] += sol->rr[j];
            n++;
        }
        if (n > 0)
            for (i = 0; i < 3; i++) opos[i] /= n;
    } else if (plotOptDialog->getOrigin() == ORG_FITPOS) {
        if (!fitPositions(&time, opos, ovel)) return;
    } else if (plotOptDialog->getOrigin() == ORG_REFPOS) {
        if (norm(solutionData[0].rb, 3) > 0.0) {
            for (i = 0; i < 3; i++)
                opos[i] = solutionData[0].rb[i];
        } else {
            if (!(sol = getsol(solutionData, 0)) || sol->type != 0) return;
            for (i = 0; i < 3; i++)
                opos[i] = sol->rr[i];
        }
    } else if (plotOptDialog->getOrigin() == ORG_LLHPOS) {
        pos2ecef(plotOptDialog->getOoPosition(), opos);
    } else if (plotOptDialog->getOrigin() == ORG_AUTOPOS) {
        if (solutionFiles[sel].count() > 0) {
            QFileInfo fi(solutionFiles[sel].at(0));

            readStationPosition(fi.baseName().left(4).toUpper(), sta, opos);
        }
    } else if (plotOptDialog->getOrigin() == ORG_IMGPOS) {
        pos[0] = mapOptDialog->getMapLatitude() * D2R;
        pos[1] = mapOptDialog->getMapLongitude() * D2R;
        pos[2] = 0.0;
        pos2ecef(pos, opos);
    } else if (plotOptDialog->getOrigin() == ORG_MAPPOS) {
        pos[0] = (gis.bound[0] + gis.bound[1]) / 2.0;
        pos[1] = (gis.bound[2] + gis.bound[3]) / 2.0;
        pos[2] = 0.0;
        pos2ecef(pos, opos);
    } else if (plotOptDialog->getOrigin() - ORG_PNTPOS < wayPoints.size()) {
        for (i = 0; i < 3; i++)
            opos[i] = wayPoints[plotOptDialog->getOrigin() - ORG_PNTPOS].position[i];
    }
    if (norm(opos, 3) <= 0.0) {
        // default start position
        if (!(sol = getsol(solutionData, 0)) || sol->type != 0) return;
        for (i = 0; i < 3; i++)
            opos[i] = sol->rr[i];
    }
    originEpoch = time;
    for (i = 0; i < 3; i++) {
        originPosition[i] = opos[i];
        originVelocity[i] = ovel[i];
    }
    refresh_MapView();
}
// update satellite mask ----------------------------------------------------
void Plot::updateSatelliteMask()
{
    int sat, prn;

    trace(3, "updateSatelliteMask\n");

    // clear mask
    for (sat = 1; sat <= MAXSAT; sat++) satelliteMask[sat - 1] = 0;

    for (sat = 1; sat <= MAXSAT; sat++)
        if (!(satsys(sat, &prn) & plotOptDialog->getNavSys())) satelliteMask[sat - 1] = 1;

    if (!plotOptDialog->getExcludedSatellites().isEmpty()) {
        foreach (QString sat, plotOptDialog->getExcludedSatellites().split(' ', Qt::SkipEmptyParts)) {
            unsigned char ex;
            int satNo;
            if (sat[0] == '+')
            {
                ex = 0;
                sat = sat.mid(1);
            } else ex = 1;
            if (!(satNo = satid2no(qPrintable(sat)))) continue;
            satelliteMask[satNo - 1] = ex;
        }
    }
}
// update satellite select ---------------------------------------------------
void Plot::updateSatelliteSelection()
{
    QString satelliteList = ui->cBSatelliteList->currentText();
    char id[16];
    int sys = 0;

    if (satelliteList == "G") sys = SYS_GPS;
    else if (satelliteList == "R") sys = SYS_GLO;
    else if (satelliteList == "E") sys = SYS_GAL;
    else if (satelliteList == "J") sys = SYS_QZS;
    else if (satelliteList == "C") sys = SYS_CMP;
    else if (satelliteList == "I") sys = SYS_IRN;
    else if (satelliteList == "S") sys = SYS_SBS;

    for (int i = 0; i < MAXSAT; i++) {
        satno2id(i + 1, id);
        satelliteSelection[i] = (satelliteList == "ALL") || (satelliteList == id) || satsys(i + 1, NULL) == sys;
    }
}
// update enable/disable of widgets -----------------------------------------
void Plot::updateEnable()
{
    bool data = ui->btnSolution1->isChecked() || ui->btnSolution2->isChecked() || ui->btnSolution12->isChecked();
    bool plot = (PLOT_SOLP <= plotType) && (plotType <= PLOT_NSAT);
    bool sel = (!ui->btnSolution1->isChecked()) && (ui->btnSolution2->isChecked()) ? 1 : 0;

    trace(3, "updateEnable\n");

    ui->toolPanel->setVisible(ui->menuToolBar->isChecked());
    ui->statusbar->setVisible(ui->menuStatusBar->isChecked());

    ui->menuConnect->setChecked(connectState);
    ui->btnSolution1->setEnabled(true);
    ui->btnSolution2->setEnabled(plotType <= PLOT_NSAT || plotType == PLOT_RES || plotType == PLOT_RESE);
    ui->btnSolution12->setEnabled(!connectState && plotType <= PLOT_SOLA && solutionData[0].n > 0 && solutionData[1].n > 0);

    // combo boxes
    ui->cBQFlag->setVisible(plotType == PLOT_TRK || plotType == PLOT_SOLP ||
                            plotType == PLOT_SOLV || plotType == PLOT_SOLA ||
                            plotType == PLOT_NSAT);
    ui->cBObservationType->setVisible(plotType == PLOT_OBS || plotType == PLOT_SKY);
    ui->cBObservationTypeSNR->setVisible(plotType == PLOT_SNR || plotType == PLOT_SNRE || plotType == PLOT_MPS);

    ui->cBFrequencyType->setVisible(plotType == PLOT_RES || plotType == PLOT_RESE);
    ui->cBDopType->setVisible(plotType == PLOT_DOP);
    ui->cBSatelliteList->setVisible(plotType == PLOT_RES || plotType == PLOT_RESE || plotType >= PLOT_OBS ||
                                    plotType == PLOT_SKY || plotType == PLOT_DOP ||
                                    plotType == PLOT_SNR || plotType == PLOT_SNRE ||
                                    plotType == PLOT_MPS);
    ui->cBQFlag->setEnabled(data);
    ui->cBObservationType->setEnabled(data && !simulatedObservation);
    ui->cBObservationTypeSNR->setEnabled(data && !simulatedObservation);

    // tool bar part
    ui->toolPanelOn->setVisible(plotType == PLOT_SOLP || plotType == PLOT_SOLV ||
                                        plotType == PLOT_SOLA || plotType == PLOT_NSAT ||
                                        plotType == PLOT_RES || plotType == PLOT_RESE ||
                                        plotType == PLOT_SNR || plotType == PLOT_SNRE);
    ui->btnOn1->setEnabled(plot || plotType == PLOT_SNR || plotType == PLOT_RES ||
                           plotType == PLOT_RESE || plotType == PLOT_SNRE);
    ui->btnOn2->setEnabled(plot || plotType == PLOT_SNR || plotType == PLOT_RES ||
                           plotType == PLOT_RESE || plotType == PLOT_SNRE);
    ui->btnOn3->setEnabled(plot || plotType == PLOT_SNR || plotType == PLOT_RES);

    ui->btnRangeList->setVisible(plotType == PLOT_TRK || plotType == PLOT_SOLP ||
                                 plotType == PLOT_SOLV || plotType == PLOT_SOLA ||
                                 plotType == PLOT_NSAT);
    ui->btnRangeList->setEnabled(plotType != PLOT_NSAT);


    ui->btnCenterOrigin->setVisible(plotType == PLOT_TRK || plotType == PLOT_SOLP ||
                                    plotType == PLOT_SOLV || plotType == PLOT_SOLA ||
                                    plotType == PLOT_NSAT);
    ui->btnCenterOrigin->setEnabled(plotType != PLOT_NSAT);
    ui->menuCenterOrigin->setEnabled(ui->btnCenterOrigin->isEnabled());

    // fit actions
    ui->btnFitHorizontal->setVisible(plotType == PLOT_SOLP || plotType == PLOT_SOLV ||
                                     plotType == PLOT_SOLA || plotType == PLOT_NSAT ||
                                     plotType == PLOT_RES || plotType == PLOT_OBS ||
                                     plotType == PLOT_DOP || plotType == PLOT_SNR ||
                                     plotType == PLOT_SNRE);
    ui->btnFitVertical->setVisible(plotType == PLOT_TRK || plotType == PLOT_SOLP ||
                                   plotType == PLOT_SOLV || plotType == PLOT_SOLA);
    ui->btnFitHorizontal->setEnabled(data && ui->btnFitHorizontal->isVisible());
    ui->menuFitHoriz->setEnabled(ui->btnFitHorizontal->isEnabled());
    ui->btnFitVertical->setEnabled(data && ui->btnFitVertical->isVisible());
    ui->menuFitVert->setEnabled(ui->btnFitVertical->isEnabled());

    // fix actions
    ui->btnFixCenter->setVisible(plotType == PLOT_TRK);
    ui->btnFixHorizontal->setVisible(plotType == PLOT_SOLP || plotType == PLOT_SOLV ||
                                     plotType == PLOT_SOLA || plotType == PLOT_NSAT ||
                                     plotType == PLOT_RES || plotType == PLOT_OBS ||
                                     plotType == PLOT_DOP || plotType == PLOT_SNR);
    ui->btnFixVertical->setVisible(plotType == PLOT_SOLP || plotType == PLOT_SOLV ||
                                   plotType == PLOT_SOLA);
    ui->btnFixCenter->setEnabled(data);
    ui->menuFixCenter->setEnabled(ui->btnFixCenter->isEnabled());
    ui->btnFixHorizontal->setEnabled(data);
    ui->menuFixHoriz->setEnabled(ui->btnFixHorizontal->isEnabled());
    ui->btnFixVertical->setEnabled(data);
    ui->menuFixVert->setEnabled(ui->btnFixVertical->isEnabled());

    if (!ui->menuShowTrack->isChecked()) {
        ui->menuFixHoriz->setEnabled(false);
        ui->menuFixVert->setEnabled(false);
        ui->menuFixCenter->setEnabled(false);
        ui->btnAnimate->setChecked(false);
    }

    // animations
    ui->toolPanelAnimate->setVisible(!connectState);
    ui->btnAnimate->setVisible(data && ui->menuShowTrack->isChecked());
    ui->sBTime->setVisible(data && ui->menuShowTrack->isChecked());
    ui->sBTime->setEnabled(data && ui->menuShowTrack->isChecked());
    ui->menuAnimationStart->setEnabled(!connectState && ui->btnAnimate->isVisible() && !ui->btnAnimate->isChecked());
    ui->menuAnimationStop->setEnabled(!connectState && ui->btnAnimate->isVisible() && ui->btnAnimate->isChecked());

    // show dialog actions
    ui->menuShowTrack->setEnabled(data);
    ui->btnShowTrack->setVisible(ui->menuShowTrack->isEnabled());
    ui->btnShowSkyplot->setVisible(plotType == PLOT_SKY || plotType == PLOT_MPS);
    ui->menuShowSkyplot->setEnabled(ui->btnShowSkyplot->isVisible());
    ui->btnShowMap->setVisible(plotType == PLOT_TRK);
    ui->menuShowMap->setEnabled(!ui->btnSolution12->isChecked());
    ui->menuMapView->setEnabled(plotType == PLOT_TRK || plotType == PLOT_SOLP);
    ui->btnMapView->setVisible(ui->menuMapView->isEnabled());
    ui->menuMapImage->setEnabled(mapImageOriginal.height() > 0);
    ui->menuSkyImage->setEnabled(skyImageOriginal.height() > 0);
    ui->menuSourceSolution->setEnabled(solutionFiles[sel].count() > 0);
    ui->menuSourceObservation->setEnabled(observationFiles.count() > 0);
    ui->menuMapLayer->setEnabled(true);

    // show actions
    ui->btnShowImage->setVisible((plotType == PLOT_TRK && !mapImageOriginal.isNull()) ||
                                 ((plotType == PLOT_SKY || plotType == PLOT_MPS) && !skyImageOriginal.isNull()));
    ui->menuShowImage->setEnabled(ui->btnShowImage->isVisible());
    ui->btnShowGrid->setVisible(plotType == PLOT_TRK);
    ui->menuShowGrid->setEnabled(ui->btnShowGrid->isVisible());

    ui->menuOpenSolution1->setEnabled(!connectState);
    ui->menuOpenSolution2->setEnabled(!connectState);
    ui->menuConnect->setEnabled(!connectState);
    ui->menuDisconnect->setEnabled(connectState);
    ui->menuPort->setEnabled(!connectState);
    ui->menuOpenObs->setEnabled(!connectState);
    ui->menuOpenNav->setEnabled(!connectState);
    ui->menuOpenElevationMask->setEnabled(!connectState);
    ui->menuReload->setEnabled(!connectState);

    ui->wgStreamStatus->setEnabled(connectState);
    ui->btnFrequency->setVisible(ui->cBFrequencyType->isVisible() || ui->cBObservationType->isVisible() || ui->cBObservationTypeSNR->isVisible());

    ui->btnPointCoordinateType->setVisible(plotType == PLOT_TRK);
}
// linear-fitting of positions ----------------------------------------------
int Plot::fitPositions(gtime_t *time, double *opos, double *ovel)
{
    sol_t *sol;
    int i, j;
    double t, x[2], Ay[3][2] = {{0}}, AA[3][4] = {{0}};

    trace(3, "fitPosition\n");

    if (solutionData[0].n <= 0) return 0;

    for (i = 0; (sol = getsol(solutionData, i)) != NULL; i++) {
        if (sol->type != 0) continue;
        if (time->time == 0) *time = sol->time;
        t = timediff(sol->time, *time);

        for (j = 0; j < 3; j++) {
            Ay[j][0] += sol->rr[j];
            Ay[j][1] += sol->rr[j] * t;
            AA[j][0] += 1.0;
            AA[j][1] += t;
            AA[j][2] += t;
            AA[j][3] += t * t;
        }
    }
    for (i = 0; i < 3; i++) {
        if (solve("N", AA[i], Ay[i], 2, 1, x)) return 0;
        opos[i] = x[0];
        ovel[i] = x[1];
    }
    return 1;
}
// fit time-range of plot ---------------------------------------------------
void Plot::fitTime()
{
    sol_t *sol_start, *sol_end;
    double tl[2] = {86400.0 * 7, 0.0}, tp[2], xl[2], yl[2], zl[2];
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "fitTime\n");

    // time from solutions
    sol_start = getsol(solutionData + sel, 0);
    sol_end = getsol(solutionData + sel, solutionData[sel].n - 1);
    if (sol_start && sol_end) {
        tl[0] = qMin(tl[0], timePosition(sol_start->time));
        tl[1] = qMax(tl[1], timePosition(sol_end->time));
    }
    // time from observations
    if (observation.n > 0) {
        tl[0] = qMin(tl[0], timePosition(observation.data[0].time));
        tl[1] = qMax(tl[1], timePosition(observation.data[observation.n - 1].time));
    }
    // time from user input
    if (timeEnabled[0]) tl[0] = timePosition(timeStart);
    if (timeEnabled[1]) tl[1] = timePosition(timeEnd);

    if (qFuzzyCompare(tl[0], tl[1])) {
        tl[0] = tl[0] - DEFTSPAN / 2.0;
        tl[1] = tl[0] + DEFTSPAN / 2.0;
    } else if (tl[0] > tl[1]) {
        tl[0] = -DEFTSPAN / 2.0;
        tl[1] = DEFTSPAN / 2.0;
    }

    graphTriple[0]->getLimits(tp, xl);
    graphTriple[1]->getLimits(tp, yl);
    graphTriple[2]->getLimits(tp, zl);
    graphTriple[0]->setLimits(tl, xl);
    graphTriple[1]->setLimits(tl, yl);
    graphTriple[2]->setLimits(tl, zl);

    graphSingle->getLimits(tp, xl);
    graphSingle->setLimits(tl, xl);
}
// set x/y-range of plot ----------------------------------------------------
void Plot::setRange(int all, double range)
{
    double xl[] = {-range, range};
    double yl[] = {-range, range};
    double zl[] = {-range, range};
    double xs, ys, tl[2], xp[2], pos[3];

    trace(3, "setRange: all=%d range=%.3f\n", all, range);

    if (all || plotType == PLOT_TRK) {
        graphTrack->setLimits(xl, yl);
        graphTrack->getScale(xs, ys);
        graphTrack->setScale(qMax(xs, ys), qMax(xs, ys));
        if (norm(originPosition, 3) > 0.0) {
            ecef2pos(originPosition, pos);
            mapView->setCenter(pos[0] * R2D, pos[1] * R2D);
        }
    }
    if (PLOT_SOLP <= plotType && plotType <= PLOT_SOLA) {
        graphTriple[0]->getLimits(tl, xp);
        graphTriple[0]->setLimits(tl, xl);
        graphTriple[1]->setLimits(tl, yl);
        graphTriple[2]->setLimits(tl, zl);
    } else if (plotType == PLOT_NSAT) {
        graphTriple[0]->getLimits(tl, xp);
        xl[0] = yl[0] = zl[0] = 0.0;
        xl[1] = plotOptDialog->getMaxDop();
        yl[1] = YLIM_AGE;
        zl[1] = YLIM_RATIO;
        graphTriple[0]->setLimits(tl, xl);
        graphTriple[1]->setLimits(tl, yl);
        graphTriple[2]->setLimits(tl, zl);
    } else if (plotType < PLOT_SNR) {
        graphTriple[0]->getLimits(tl, xp);
        xl[0] = -plotOptDialog->getMaxMP();
        xl[1] = plotOptDialog->getMaxMP();
        yl[0] = -plotOptDialog->getMaxMP()/100.0;
        yl[1] = plotOptDialog->getMaxMP()/100.0;
        zl[0] = 0.0;
        zl[1] = 90.0;
        graphTriple[0]->setLimits(tl, xl);
        graphTriple[1]->setLimits(tl, yl);
        graphTriple[2]->setLimits(tl, zl);
    } else {
        graphTriple[0]->getLimits(tl, xp);
        xl[0] = 10.0;
        xl[1] = 60.0;
        yl[0] = -plotOptDialog->getMaxMP();
        yl[1] = plotOptDialog->getMaxMP();
        zl[0] = 0.0;
        zl[1] = 90.0;
        graphTriple[0]->setLimits(tl, xl);
        graphTriple[1]->setLimits(tl, yl);
        graphTriple[2]->setLimits(tl, zl);
    }
}
// fit x/y-range of plot ----------------------------------------------------
void Plot::fitRange(int all)
{
    TIMEPOS *pos, *pos1, *pos2;
    sol_t *data;
    double xs, ys, xp[2], tl[2], xl[] = {1E8, -1E8}, yl[2] = {1E8, -1E8}, zl[2] = {1E8, -1E8};
    double lat, lon, lats[2] = {90, -90}, lons[2] = {180, -180}, llh[3];
    int i, type = plotType - PLOT_SOLP;

    trace(3, "fitRange: all=%d\n", all);

    ui->menuFixHoriz->setChecked(false);

    if (ui->btnSolution1->isChecked()) {
        pos = solutionToPosition(solutionData, -1, ui->cBQFlag->currentIndex(), type);

        for (i = 0; i < pos->n; i++) {
            xl[0] = qMin(xl[0], pos->x[i]);
            yl[0] = qMin(yl[0], pos->y[i]);
            zl[0] = qMin(zl[0], pos->z[i]);
            xl[1] = qMax(xl[1], pos->x[i]);
            yl[1] = qMax(yl[1], pos->y[i]);
            zl[1] = qMax(zl[1], pos->z[i]);
        }
        delete pos;
    }
    if (ui->btnSolution2->isChecked()) {
        pos = solutionToPosition(solutionData + 1, -1, ui->cBQFlag->currentIndex(), type);

        for (i = 0; i < pos->n; i++) {
            xl[0] = qMin(xl[0], pos->x[i]);
            yl[0] = qMin(yl[0], pos->y[i]);
            zl[0] = qMin(zl[0], pos->z[i]);
            xl[1] = qMax(xl[1], pos->x[i]);
            yl[1] = qMax(yl[1], pos->y[i]);
            zl[1] = qMax(zl[1], pos->z[i]);
        }
        delete pos;
    }
    if (ui->btnSolution12->isChecked()) {
        pos1 = solutionToPosition(solutionData, -1, 0, type);
        pos2 = solutionToPosition(solutionData + 1, -1, 0, type);
        pos = pos1->diff(pos2, ui->cBQFlag->currentIndex());

        for (i = 0; i < pos->n; i++) {
            xl[0] = qMin(xl[0], pos->x[i]);
            yl[0] = qMin(yl[0], pos->y[i]);
            zl[0] = qMin(zl[0], pos->z[i]);
            xl[1] = qMax(xl[1], pos->x[i]);
            yl[1] = qMax(yl[1], pos->y[i]);
            zl[1] = qMax(zl[1], pos->z[i]);
        }
        delete pos1;
        delete pos2;
        delete pos;
    }
    // add margins
    xl[0] -= 0.05;
    xl[1] += 0.05;
    yl[0] -= 0.05;
    yl[1] += 0.05;
    zl[0] -= 0.05;
    zl[1] += 0.05;

    if (all || plotType == PLOT_TRK) {
        graphTrack->setLimits(xl, yl);
        graphTrack->getScale(xs, ys);
        graphTrack->setScale(qMax(xs, ys), qMax(xs, ys));
    }
    if (all || plotType <= PLOT_SOLA || plotType == PLOT_RES) {
        graphTriple[0]->getLimits(tl, xp);

        graphTriple[0]->setLimits(tl, xl);
        graphTriple[1]->setLimits(tl, yl);
        graphTriple[2]->setLimits(tl, zl);
    }
    if (all) {
        if (ui->btnSolution1->isChecked()) {
            for (i = 0; (data = getsol(solutionData, i)) != NULL; i++) {
                ecef2pos(data->rr, llh);
                lats[0] = qMin(lats[0], llh[0] * R2D);
                lons[0] = qMin(lons[0], llh[1] * R2D);
                lats[1] = qMax(lats[1], llh[0] * R2D);
                lons[1] = qMax(lons[1], llh[1] * R2D);
            }
        }
        if (ui->btnSolution2->isChecked()) {
            for (i = 0; (data = getsol(solutionData + 1, i)) != NULL; i++) {
                ecef2pos(data->rr, llh);
                lats[0] = qMin(lats[0], llh[0] * R2D);
                lons[0] = qMin(lons[0], llh[1] * R2D);
                lats[1] = qMax(lats[1], llh[0] * R2D);
                lons[1] = qMax(lons[1], llh[1] * R2D);
            }
        }
        if (lats[0] <= lats[1] && lons[0] <= lons[1]) {
            lat = (lats[0] + lats[1]) / 2.0;
            lon = (lons[0] + lons[1]) / 2.0;
        }
        // FIXME: for what reason was lat/lon calculated here?
    }
}
// set center of track plot -------------------------------------------------
void Plot::setTrackCenter(double lat, double lon)
{
    gtime_t time = {0, 0};
    double pos[3] = {0}, rr[3], xyz[3];

    if (plotType != PLOT_TRK) return;

    pos[0] = lat * D2R;
    pos[1] = lon * D2R;
    pos2ecef(pos, rr);
    positionToXyz(time, rr, 0, xyz);
    graphTrack->setCenter(xyz[0], xyz[1]);

    updatePlot();
}
// load options from ini-file -----------------------------------------------
void Plot::loadOptions()
{
    QSettings settings(iniFile, QSettings::IniFormat);

    trace(3, "loadOptions\n");

//    plotType = settings.value("plot/plottype", 0).toInt();
    rtStream[0] = settings.value("plot/rtstream1", 0).toInt();
    rtStream[1] = settings.value("plot/rtstream2", 0).toInt();
    rtFormat[0] = settings.value("plot/rtformat1", 0).toInt();
    rtFormat[1] = settings.value("plot/rtformat2", 0).toInt();
    rtTimeFormat = settings.value("plot/rttimeform", 0).toInt();
    rtDegFormat = settings.value("plot/rtdegform", 0).toInt();
    rtFieldSeperator = settings.value("plot/rtfieldsep", "").toString();
    rtTimeoutTime = settings.value("plot/rttimeouttime", 0).toInt();
    rtReconnectTime = settings.value("plot/rtreconntime", 10000).toInt();

    ui->menuBrowse->setChecked(settings.value("solbrows/show", 0).toBool());
    ui->browseSplitter->restoreState(settings.value("solbrows/split1", 100).toByteArray());
    directory = settings.value("solbrows/dir",  "C:\\").toString();

    for (int i = 0; i < 2; i++) {
        streamCommands[0][i] = settings.value(QString("str/strcmd1_%1").arg(i), "").toString();
        streamCommands[1][i] = settings.value(QString("str/strcmd2_%1").arg(i), "").toString();
        streamCommandEnabled[0][i] = settings.value(QString("str/strcmdena1_%1").arg(i), 0).toInt();
        streamCommandEnabled[1][i] = settings.value(QString("str/strcmdena2_%1").arg(i), 0).toInt();
    }
    for (int i = 0; i < 3; i++) {
        streamPaths[0][i] = settings.value(QString("str/strpath1_%1").arg(i), "").toString();
        streamPaths[1][i] = settings.value(QString("str/strpath2_%1").arg(i), "").toString();
    }
    for (int i = 0; i < 10; i++) {
        streamHistory [i] = settings.value(QString("str/strhistry_%1").arg(i), "").toString();
    }

    plotOptDialog->loadOptions(settings);
    vecMapDialog->loadOptions(settings);
    viewer->loadOptions(settings);
    mapView->loadOptions(settings);

    for (int i = 0; i < lWRangeList->count(); i++) {
        bool ok;
        double range = lWRangeList->item(i)->data(Qt::UserRole+1).toDouble(&ok);

        if (ok) {
            if (fabs(range - plotOptDialog->getYRange()) < 0.0001) {
                lWRangeList->item(i)->setSelected(true);
                break;
            }
        }
    }
}
// save options to ini-file -------------------------------------------------
void Plot::saveOption()
{
    QSettings settings(iniFile, QSettings::IniFormat);

    trace(3, "saveOption\n");

    //settings.setValue("plot/plottype", PlotType);
    settings.setValue("plot/rtstream1", rtStream[0]);
    settings.setValue("plot/rtstream2", rtStream[1]);
    settings.setValue("plot/rtformat1", rtFormat[0]);
    settings.setValue("plot/rtformat2", rtFormat[1]);
    settings.setValue("plot/rttimeform", rtTimeFormat);
    settings.setValue("plot/rtdegform", rtDegFormat);
    settings.setValue("plot/rtfieldsep", rtFieldSeperator);
    settings.setValue("plot/rttimeouttime", rtTimeoutTime);
    settings.setValue("plot/rtreconntime", rtReconnectTime);

    settings.setValue("solbrows/dir", fileSelDialog->getDirectory());
    settings.setValue("solbrows/split1", ui->browseSplitter->saveState());
    settings.setValue("solbrows/show", ui->menuBrowse->isChecked());

    for (int i = 0; i < 2; i++) {
        settings.setValue(QString("str/strcmd1_%1").arg(i), streamCommands[0][i]);
        settings.setValue(QString("str/strcmd2_%1").arg(i), streamCommands[1][i]);
        settings.setValue(QString("str/strcmdena1_%1").arg(i), streamCommandEnabled[0][i]);
        settings.setValue(QString("str/strcmdena2_%1").arg(i), streamCommandEnabled[1][i]);
    }
    for (int i = 0; i < 3; i++) {
        settings.setValue(QString("str/strpath1_%1").arg(i), streamPaths[0][i]);
        settings.setValue(QString("str/strpath2_%1").arg(i), streamPaths[1][i]);
    }
    for (int i = 0; i < 10; i++) {
        settings.setValue(QString("str/strhistry_%1").arg(i), streamHistory [i]);
    }

    plotOptDialog->saveOptions(settings);
    vecMapDialog->saveOptions(settings);
    viewer->saveOptions(settings);
    mapView->saveOptions(settings);
}

//---------------------------------------------------------------------------
void Plot::driveSelectChanged()
{
    tVdirectorySelector->setVisible(false);

    tVdirectorySelector->setRootIndex(dirModel->index(ui->cBDriveSelect->currentText()));
    ui->lblDirectorySelected->setText(ui->cBDriveSelect->currentText());
}
//---------------------------------------------------------------------------
void Plot::btnDirectorySelectorClicked()
{
#ifdef FLOATING_DIRSELECTOR
    QPoint pos = Panel5->mapToGlobal(BtnDirSel->pos());
    pos.rx() += BtnDirSel->width() - DirSelector->width();
    pos.ry() += BtnDirSel->height();

    DirSelector->move(pos);
#endif
    tVdirectorySelector->setVisible(!tVdirectorySelector->isVisible());
}
//---------------------------------------------------------------------------
void Plot::directorySelectionChanged(QModelIndex index)
{
    tVdirectorySelector->expand(index);

    directory = dirModel->filePath(index);
    ui->lblDirectorySelected->setText(directory);
    fileModel->setRootPath(directory);
    ui->lVFileList->setRootIndex(fileModel->index(directory));
}
//---------------------------------------------------------------------------
void Plot::directorySelectionSelected(QModelIndex)
{
    tVdirectorySelector->setVisible(false);
}
//---------------------------------------------------------------------------
void Plot::fileListClicked(QModelIndex index)
{
    QStringList file;

    file.append(fileModel->filePath(index));
    readSolution(file, 0);

    tVdirectorySelector->setVisible(false);
}
//---------------------------------------------------------------------------
void Plot::filterClicked()
{
    QString filter = ui->cBFilter->currentText();

    // only keep data between brackets
    filter = filter.mid(filter.indexOf("(") + 1);
    filter.remove(")");

    fileModel->setNameFilters(filter.split(" ", Qt::SkipEmptyParts));
    tVdirectorySelector->setVisible(false);
}
//---------------------------------------------------------------------------
void Plot::showFrequencyDialog()
{
    freqDialog->exec();
}
//---------------------------------------------------------------------------
QString Plot::getMapImageFileName()
{
    return mapImageFile;
}
//---------------------------------------------------------------------------
QString Plot::getSkyImageFileName()
{
    return skyImageFile;
}
//---------------------------------------------------------------------------
void Plot::setWayPoints(const QList<WayPoint>& pnts)
{
    wayPoints = pnts;
}
//---------------------------------------------------------------------------
const QList<WayPoint>& Plot::getWayPoints()
{
    return wayPoints;
}
//---------------------------------------------------------------------------
void Plot::setSelectedWayPoint(int pnt)
{
    selectedWayPoint = pnt;
}
//---------------------------------------------------------------------------
int Plot::getSelectedWayPoint()
{
    return selectedWayPoint;
}
//---------------------------------------------------------------------------
