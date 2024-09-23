//---------------------------------------------------------------------------
#ifndef plotmainH
#define plotmainH
//---------------------------------------------------------------------------
#include <QTimer>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QFileSystemModel>
#include <QTreeView>

#include "graph.h"
#include "console.h"
#include "rtklib.h"

#define MAXNFILE    256                 // max number of solution files
#define MAXSTRBUFF  1024                // max length of stream buffer
#define MAXMAPPATH  4096                // max number of map paths
#define MAXMAPLAYER 12                  // max number of map layers

#define PRGNAME     "RTKPlot-Qt"           // program name

#define DEFTSPAN    600.0               // default time span (s)
#define INTARROW    60.0                // direction arrow interval (s)
#define MAXTDIFF    60.0                // max differential time (s)
#define DOPLIM      30.0                // dop view limit
#define TTOL        DTTOL               // time-differnce tolerance (s)
#define TBRK        300.0               // time to recognize break (s)
#define THRESLIP    0.1                 // slip threshold of LG-jump (m)
#define SIZE_COMPASS   45                  // compass size (pixels)
#define SIZE_VELC   45                  // velocity circle size (pixels)
#define MIN_RANGE_GE 10.0               // min range for GE view

#define PLOT_TRK    0                   // plot-type: track-plot
#define PLOT_SOLP   1                   // plot-type: position-plot
#define PLOT_SOLV   2                   // plot-type: velocity-plot
#define PLOT_SOLA   3                   // plot-type: accel-plot
#define PLOT_NSAT   4                   // plot-type: number-of-satellite-plot
#define PLOT_RES    5                   // plot-type: residual-plot
#define PLOT_RESE   6                   // plot-type: residual-elevation plot
#define PLOT_OBS    7                   // plot-type: observation-data-plot
#define PLOT_SKY    8                   // plot-type: sky-plot
#define PLOT_DOP    9                   // plot-type: dop-plot
#define PLOT_SNR    10                  // plot-type: snr/mp-plot
#define PLOT_SNRE   11                  // plot-type: snr/mp-el-plot
#define PLOT_MPS    12                  // plot-type: mp-skyplot

#define ORG_STARTPOS 0                  // plot-origin: start position
#define ORG_ENDPOS  1                   // plot-origin: end position
#define ORG_AVEPOS  2                   // plot-origin: average position
#define ORG_FITPOS  3                   // plot-origin: linear-fit position
#define ORG_REFPOS  4                   // plot-origin: reference position
#define ORG_LLHPOS  5                   // plot-origin: lat/lon/hgt position
#define ORG_AUTOPOS 6                   // plot-origin: auto-input position
#define ORG_IMGPOS  7                   // plot-origin: image-center position
#define ORG_MAPPOS  8                   // plot-origin: map center position
#define ORG_PNTPOS  9                   // plot-origin: way-point position

#define TRACEFILE   "rtkplot.trace"     // trace file
#define QCTMPFILE   "rtkplot_qc.temp"   // temporary file for qc
#define QCERRFILE   "rtkplot_qc.err"    // error file for qc

#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<0.0||(x)!=(x)?0.0:sqrt(x))

extern const QString PTypes[];

namespace Ui {
class Plot;
}

class FreqDialog;
class MapOptDialog;
class MapView;
class SpanDialog;
class ConnectDialog;
class SkyImgDialog;
class PlotOptDialog;
class AboutDialog;
class PntDialog;
class FileSelDialog;
class TextViewer;
class VecMapDialog;

class QEvent;
class QMouseEvent;
class QFocusEvent;
class QKeyEvent;
class QResizeEvent;
class QShowEvent;
class QWheelEvent;
class QPaintEVent;
class QListWidget;
class QWidgetAction;

// time-position class ------------------------------------------------------
class TIMEPOS
{
private:
    int nmax_;
    TIMEPOS(TIMEPOS &){}

public:
    int n;
    gtime_t *t;
    double *x, *y, *z, *xs, *ys, *zs, *xys;
    int *q;
    TIMEPOS(int nmax, int sflg);
    ~TIMEPOS();
    TIMEPOS *tdiff();
    TIMEPOS *diff(const TIMEPOS *pos2, int qflag);
};

struct WayPoint {
    QString name;
    double position[3];
};

// rtkplot class ------------------------------------------------------------
class Plot : public QMainWindow
{
    Q_OBJECT

public:
    explicit Plot(QWidget* parent=NULL);
    ~Plot();

protected:
    void mousePressEvent(QMouseEvent*);
    void mouseReleaseEvent(QMouseEvent*);
    void mouseDoubleClickEvent(QMouseEvent*);
    void wheelEvent(QWheelEvent*);
    void closeEvent(QCloseEvent*);
    void keyPressEvent(QKeyEvent*);
    void showEvent(QShowEvent*);
    void resizeEvent(QResizeEvent*);
    void leaveEvent(QEvent*);
    void dragEnterEvent(QDragEnterEvent*);
    void dropEvent(QDropEvent*);
    void paintEvent(QPaintEvent*);

    void mouseMove(QMouseEvent*);

protected slots:
    void connectStream();
    void disconnectStream();

    void openSolution1();
    void openSolution2();
    void openMapImage();
    void openShapeFile();
    void openObservationFile();
    void openNavigationFile();
    void openElevationMaskFile();
    void showConnectionSettingsDialog();

    void showStartEndTimeDialog();
    void showMapOptionDialog();
    void showWaypointsDialog();
    void showSolutionText();
    void showObservationText();
    void copyPlotToClipboard();
    void showPlotOptionsDialog();

    void updateToolBarVisibility();
    void updateStatusBarVisibility();
    void showMonitorConsole1();
    void showMonitorConsole2();
    void showMapView();
    void centerOrigin();
    void fitHorizontally();
    void fitVertically();
    void updateShowTrack();
    void fixHorizontally();
    void updateShowGrid();
    void menuAnimationStartClicked();
    void menuAnimationStopClicked();
    void showAboutDialog();

    void toggleConnectState();
    void activateSolution1();
    void activateSolution2();
    void activateSolution12();
    void updatePlotSizeAndRefresh();
    void showFrequencyDialog();
    
    void updateSelectedPlotType();
    void updatePlotAndEnable();
    void satelliteListChanged();
    void updateCurrentObsSol();
    void rangeListItemSelected();

    void timerTimer();

    void saveDopFile();
    void savePlotImage();
    void visibilityAnalysis();
    void saveSnrMpFile();
    void openSkyImage();
    void showSkyImageDialog();
    void arrangePlotMapViewHorizontally();
    void showWindowMaximized();
    void saveElevationMaskFile();
    void showVectorMapDialog();
    void changePointCoordinateType();
    void openWaypointsFile();
    void saveWaypointsFile();
    void updateBrowseBarVisibility();

    void fileListClicked(QModelIndex);
    void directorySelectionChanged(QModelIndex);
    void directorySelectionSelected(QModelIndex);
    void driveSelectChanged();
    void filterClicked();
    void btnDirectorySelectorClicked();

    void clear();
    void refresh();
    void reload();

private:    
    QTreeView *tVdirectorySelector;
    QFileSystemModel *fileModel, *dirModel;
    QString directory;

    QTimer timer;
    QElapsedTimer updateTimer;

    QPixmap buffer;
    QImage mapImageScaled, mapImageOriginal;
    QImage skyImageOriginal;

    Graph *graphTrack;
    Graph *graphTriple[3];
    Graph *graphSingle;
    Graph *graphSky;
    Graph *graphDual[2];
    Console *console1, *console2;

    QStringList openFiles;
    QStringList solutionFiles[2];
    QStringList observationFiles;
    QStringList navigationFiles;

    stream_t stream[2];
    stream_t streamTimeSync;
    solbuf_t solutionData[2];
    solstatbuf_t solutionStat[2];
    int solutionIndex[2];
    obs_t observation;
    nav_t navigation;
    sta_t station;
    double *azimuth, *elevation, *multipath[NFREQ+NEXOBS];
    char streamBuffer[1024];
    int nStreamBuffer;
    
    gtime_t originEpoch;
    int formWidth, formHeight;
    int connectState, openRaw;
    int nObservation, *indexObservation, observationIndex;
    int week;
    int flush, plotType;
    int satelliteMask[MAXSAT];
    bool satelliteSelection[MAXSAT];
    int simulatedObservation;
    
    int dragState, dragCurrentX, dragCurrentY;
    double dragStartX, dragStartY, dragCenterX, dragCenterY, dragScaleX, dragScaleY, centX, dragCentX;
    uint32_t mouseDownTick;
    
    int GEState, GEDataState[2];
    double GEHeading;

    bool eventFilter(QObject *obj, QEvent *event);

    void readSolutionStat(const QStringList &files, int sel);
    int readObservationRinex(const QStringList &files, obs_t *obs, nav_t *nav, sta_t *sta);
    void readShapeFile(const QStringList &file);
    void generateVisibilityData();
    void saveDop(const QString &file);
    void saveSnrMp(const QString &file);
    void saveElevationMask(const QString &file);
    void connectPath(const QString &path, int ch);
    int isObservation(const QString &file);
    void updateObservation(int nobs);
    void updateMp();
    void clearObservation();
    void clearSolution();
    void readWaitStart();
    void readWaitEnd();
    
    void updateDisplay();
    void updatePlotType(int type);
    void updatePlotTypeMenu();
    void updateSatelliteList();
    void updateObservationType();
    void updatePlotSizes();
    void updateColor();
    void updateTime();
    void updateOrigin();
    void updateSatelliteMask();
    void updateSatelliteSelection();
    void updateStatusBarInformation();
    void updateTimeSolution();
    void updateTimeObservation();
    void updateInfoSolution();
    void updateInfoObservation();
    void updatePoint(int x, int y);
    void updateEnable();
    void fitTime();
    void setRange(int all, double range);
    void fitRange(int all);
    
    void setCenterX(double c);
    void setScaleX(double s);
    void mouseDownTrack(int X, int Y);
    void mouseDownSolution(int X, int Y);
    void mouseDownObservation(int X, int Y);
    void mouseMoveTrack(int X, int Y, double dx, double dy, double dxs, double dys);
    void mouseMoveSolution(int X, int Y, double dx, double dy, double dxs, double dys);
    void mouseMoveObservation(int X, int Y, double dx, double dy, double dxs, double dys);

    void drawTrack(QPainter &g,int level);
    void drawTrackImage(QPainter &g,int level);
    void drawTrackGis(QPainter &g,int level);
    void drawTrackPoint(QPainter &g,const TIMEPOS *pos, int level, int style);
    void drawTrackPosition(QPainter &g,const double *rr, int type, int siz, QColor color, const QString &label);
    void drawTrackStatistics(QPainter &g,const TIMEPOS *pos, const QString &header, int p);
    void drawTrackError(QPainter &g,const TIMEPOS *pos, int style);
    void drawTrackArrow(QPainter &g,const TIMEPOS *pos);
    void drawTrackVelocityIndicator(QPainter &g,const TIMEPOS *vel);
    void drawLabel(Graph *,QPainter &g, const QPoint &p, const QString &label, int ha, int va);
    void drawMark(Graph *,QPainter &g, const QPoint &p, int mark, const QColor &color, int size, int rot);
    void drawSolution(QPainter &g,int level, int type);
    void drawSolutionPoint(QPainter &g,const TIMEPOS *pos, int level, int style);
    void drawSolutionStat(QPainter &g,const TIMEPOS *pos, const QString &unit, int p);
    void drawNsat(QPainter &g,int level);
    void drawResidual(QPainter &g,int level);
    void drawResidualE(QPainter &g,int level);
    void drawPolyS(Graph *, QPainter &c, double *x, double *y, int n, const QColor &color, int style);
    
    void drawObservation(QPainter &g,int level);
    void drawObservationSlips(QPainter &g,double *yp);
    void drawObservationEphemeris(QPainter &g,double *yp);
    void drawSkyImage(QPainter &g,int level);
    void drawSky(QPainter &g,int level);
    void drawDop(QPainter &g,int level);
    void drawDopStat(QPainter &g,double *dop, int *ns, int n);
    void drawSnr(QPainter &g,int level);
    void drawSnrE(QPainter &g,int level);
    void drawMpSky(QPainter &g,int level);
    
    TIMEPOS *solutionToPosition(solbuf_t *sol, int index, int qflag, int type);
    TIMEPOS *solutionToNsat(solbuf_t *sol, int index, int qflag);
    
    void positionToXyz(gtime_t time, const double *rr, int type, double *xyz);
    void covarianceToXyz(const double *rr, const float *qr, int type, double *xyzs);
    void calcStats(const double *x, int n, double ref, double &ave, double &std, double &rms);
    int fitPositions(gtime_t *time, double *opos, double *ovel);
    
    QString latLonString(const double *pos, int ndec);
    QColor observationColor(const obsd_t *obs, double az, double el, QVariant obstype);
    QColor sysColor(int sat);
    QColor snrColor(double snr);
    QColor mpColor(double mp);
    void readStationPosition(const QString &file, const QString &sta, double *rr);
    int searchPosition(int x, int y);
    void timeSpan(gtime_t *ts, gtime_t *te, double *tint);
    double timePosition(gtime_t time);
    QString timeString(gtime_t time, int n, int tsys);
    int execCmd(const QString &cmd, const QStringList &opt);
    void showMessage(const QString &msg);
    void showLegend (const QStringList &msgs);
    void loadOptions();
    void saveOption();
    
    FreqDialog * freqDialog;
    MapOptDialog *mapOptDialog;
    MapView *mapView;
    SpanDialog *spanDialog;
    ConnectDialog *connectDialog;
    SkyImgDialog *skyImgDialog;
    PlotOptDialog *plotOptDialog;
    AboutDialog *aboutDialog;
    PntDialog *waypointsDialog;
    FileSelDialog *fileSelDialog;
    TextViewer *viewer;
    VecMapDialog *vecMapDialog;
    QListWidget *lWRangeList;
    QWidgetAction* rangeListPopupAction;

    Ui::Plot *ui;

protected:
    QImage skyImageResampled;
    QString iniFile;
    QString mapImageFile;
    QString skyImageFile;
    tle_t tleData;
    QFont font;
    int traceLevel;

    // connection settings
    int rtStream[2];
    QString rtPath1, rtPath2;
    QString streamPaths[2][3];
    QString streamCommands[2][2];
    int streamCommandEnabled[2][2];
    int rtFormat[2];
    int rtConnectionType;
    int rtTimeFormat;
    int rtDegFormat;
    QString rtFieldSeperator;
    int rtTimeoutTime;
    int rtReconnectTime;
    double elevationMaskData[361];
    
    // time options
    int timeEnabled[3];
    gtime_t timeStart;
    gtime_t timeEnd;
    double timeInterval;

    // map options
    int pointCoordinateType;
    
    QString title;
    QList<WayPoint> wayPoints;
    int selectedWayPoint;
    int oPositionType;
    double originPosition[3], originVelocity[3];
    
    QString streamHistory [10];
    
    void readObservation(const QStringList &files);
    void readNavigation(const QStringList &files);
    void readMapData(const QString &file);
    void readSkyData(const QString &file);
    void readGpxFile(const QString &file);
    void readPositionFile(const QString &file);
    void readWaypoints(const QString &file);
    void saveGpxFile(const QString &file);
    void savePositionFile(const QString &file);
    void saveWaypoints(const QString &file);
    void generateObservationSlips(int *LLI);
    void readElevationMaskData(const QString &file);
    void refresh_MapView();
    double getYRange();
    QString plotDetails(const QPoint &globalPos);

public:
    void updateSky();
    void updatePlot ();
    void readSolution(const QStringList &files, int sel);
    bool getCurrentPosition(double *rr);
    bool getCenterPosition(double *rr);
    QString getMapImageFileName();
    QString getSkyImageFileName();
    void setTrackCenter(double lat, double lon);
    void setWayPoints(const QList<WayPoint>& pnts);
    const QList<WayPoint>& getWayPoints();
    void setSelectedWayPoint(int);
    int getSelectedWayPoint();
    void generateElevationMaskFromSkyImage();

    gis_t gis;
};

//---------------------------------------------------------------------------
#endif
