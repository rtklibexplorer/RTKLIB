//---------------------------------------------------------------------------
#ifndef gmviewH
#define gmviewH
//---------------------------------------------------------------------------
#include <QDialog>
#include <QTimer>

#define NUM_MARK 256

namespace Ui {
class MapView;
}

class QWebEngineView;
class QWebChannel;
class QResizeEvent;
class QShowEvent;
class MapViewOptDialog;
class QSettings;

//---------------------------------------------------------------------------
class MapViewPageState : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString text MEMBER text NOTIFY textChanged)

public:
    explicit MapViewPageState(QObject *parent = NULL): QObject(parent){}
    QString getText() {return text;}

signals:
    void textChanged(const QString &text);

private:
    QString text;
};

//---------------------------------------------------------------------------
class MapView : public QDialog
{
    Q_OBJECT

protected slots:
    void zoomOut();
    void zoomIn();
    void center();
    void pageLoaded(bool);

    void showMapLL();
    void showMapGM();
    void setView(int map, double lat, double lon, int zoom);
    void showOptionsDialog();
    void selectMapLL();
    void selectMapGM();
    void timerLLTimer();
    void timerGMTimer();

private:
    QWebEngineView *webBrowser;
    MapViewPageState *pageState;
    QWebChannel *webChannel;
    bool loaded;

    QString mapStrings[6][3];
    int selectedMap;
    QString apiKey;

    double center_latitude, center_longitude;
    int markState[NUM_MARK];
    double markPosition[NUM_MARK][2];
    QString markTitle[NUM_MARK];
    QString markMessage[NUM_MARK];
    int highlightedMark;

    QTimer timerLL, timerGM;

    MapViewOptDialog *mapViewOptDialog;

    void updateMap();
    void selectMap(int map);
    int  setState(int map);
    void execFunction(int map, const QString &func);

    Ui::MapView *ui;
public:
    explicit MapView(QWidget *parent = NULL);

    void showMap(int map);
    void addMark(int map, int index, double lat, double lon, const QString& title, const QString& msg);
    void setCenter(double lat, double lon);
    void setViewBounds(double min_lat, double min_lon, double max_lat, double max_lon);
    void setMark(int index, const QString &title, double lat, double lon);
    void showMark(int index);
    void hideMark(int index);
    void highlightMark(int index);
    void clearMark();
    bool isLoaded();
    void setApiKey(const QString & key); // TODO: store API key in config
    const QString &getApiKey();
    void loadOptions(QSettings &);
    void saveOptions(QSettings &);
};
//---------------------------------------------------------------------------
#endif
