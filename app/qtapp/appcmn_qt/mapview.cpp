//---------------------------------------------------------------------------
// gmview.c: map view
//---------------------------------------------------------------------------
#ifdef QWEBENGINE
#include <QWebEngineView>
#include <QWebEnginePage>
#include <QWebChannel>
#endif

#include <QShowEvent>
#include <QSettings>
#include <QFile>
#include <QTextStream>
#include <QLabel>

#include <cmath>

#include "mapviewopt.h"
#include "mapview.h"

#include "ui_mapview.h"

#define RTKLIB_GM_TEMP ":/html/rtklib_gm.htm"
#define RTKLIB_GM_FILE "rtklib_gm_a.htm"
#define RTKLIB_LL_TEMP ":/html/rtklib_ll.htm"
#define RTKLIB_LL_FILE "rtklib_ll_a.htm"
#define URL_GM_API     "http://maps.google.com/maps/api/js"
#define MAP_OPACITY    0.8
#define INIT_ZOOM      8  // initial zoom level

//---------------------------------------------------------------------------
MapView::MapView(QWidget *parent)
    : QDialog(parent), ui(new Ui::MapView)
{
    loaded = false;
    ui->setupUi(this);

    selectedMap = 0;
    center_latitude = center_longitude = 0.0;
    highlightedMark = -1;
    for (int i = 0; i < 256; i++) {
        markState[0] = markState[1] = 0;
        markPosition[i][0] = markPosition[i][1] = NAN;
    }
    mapViewOptDialog = new MapViewOptDialog(this);

    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &MapView::reject);
    connect(ui->btnOptions, &QPushButton::clicked, this, &MapView::showOptionsDialog);
    connect(ui->btnShrink, &QPushButton::clicked, this, &MapView::zoomOut);
    connect(ui->btnExpand, &QPushButton::clicked, this, &MapView::zoomIn);
    connect(ui->btnCenter, &QPushButton::clicked, this, &MapView::center);
    connect(ui->rBMapSelect1, &QRadioButton::clicked, this, &MapView::selectMapLL);
    connect(ui->rBMapSelect2, &QRadioButton::clicked, this, &MapView::selectMapGM);
    connect(&timerLL, &QTimer::timeout, this, &MapView::timerLLTimer);
    connect(&timerGM, &QTimer::timeout, this, &MapView::timerGMTimer);

    mapStrings[0][0] = "OpenStreetMap";                                     //todo: save map strings in settins
    mapStrings[0][1] = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
    mapStrings[0][2] = "https://osm.org/copyright";

    QHBoxLayout *layout = new QHBoxLayout();
    ui->wgMap->setContentsMargins(0, 0, 0, 0);
    ui->wgMap->setLayout(layout);
#ifdef QWEBENGINE
    webBrowser = new QWebEngineView(ui->wgMap);
    layout->addWidget(webBrowser);
    pageState = new MapViewPageState(this);
    webChannel = new QWebChannel(this);
    webChannel->registerObject(QStringLiteral("state"), pageState);

    connect(webBrowser, &QWebEngineView::loadFinished, this, &MapView::pageLoaded);
#else
    QLabel *label = new QLabel();
    label->setText(tr("QWebEngine is not available to show a map."));
    layout->addWidget(label);
#endif

    ui->rBMapSelect1->setChecked(!selectedMap);
    ui->rBMapSelect2->setChecked(selectedMap);
    selectMap(selectedMap);
    showMap(selectedMap);
}
//---------------------------------------------------------------------------
void MapView::setApiKey(const QString & key)
{
    apiKey = key;
}
//---------------------------------------------------------------------------
const QString &MapView::getApiKey()
{
    return apiKey;
}
//---------------------------------------------------------------------------
void MapView::selectMapLL()
{
    selectMap(0);
}
//---------------------------------------------------------------------------
void MapView::selectMapGM()
{
    selectMap(1);
}
//---------------------------------------------------------------------------
void MapView::showOptionsDialog()
{
    mapViewOptDialog->move(x() + width() / 2 - mapViewOptDialog->width() / 2,
                           y() + height() / 2 - mapViewOptDialog->height() / 2);

    mapViewOptDialog->setApiKey(apiKey);
    for (int i = 0; i < 6; i++)
        mapViewOptDialog->setMapStrings(i, mapStrings[i][0], mapStrings[i][1], mapStrings[i][2]);
    if (mapViewOptDialog->exec() != QDialog::Accepted) return;

    apiKey = mapViewOptDialog->getApiKey();
    for (int i = 0; i < 6; i++)
        mapViewOptDialog->getMapStrings(i, mapStrings[i][0], mapStrings[i][1], mapStrings[i][2]);

    showMap(selectedMap);
}
//---------------------------------------------------------------------------
void MapView::pageLoaded(bool ok)
{
    if (!ok) return;

#ifdef QWEBENGINE
    QFile webchannel(":/html/qwebchannel.js");
    webchannel.open(QIODevice::ReadOnly);
    webBrowser->page()->runJavaScript(webchannel.readAll());
    webBrowser->page()->runJavaScript("new QWebChannel(qt.webChannelTransport,function(channel) {channel.objects.state.text=document.getElementById('state').value;});");
#endif

    loaded = true;
}
//---------------------------------------------------------------------------
void MapView::zoomOut()
{
    execFunction(selectedMap, "ZoomOut()");
}
//---------------------------------------------------------------------------
void MapView::zoomIn()
{
    execFunction(selectedMap, "ZoomIn()");
}
//---------------------------------------------------------------------------
void MapView::center()
{
    setCenter(center_latitude,center_longitude);
}
//---------------------------------------------------------------------------
void MapView::showMap(int map)
{
    if (map == 0) {
        showMapLL();
    }
    else if (map == 1) {
        showMapGM();
    }
}
//---------------------------------------------------------------------------
void MapView::showMapLL()
{
    QString pageSource;
    int i, j;

    QFile ifp(RTKLIB_LL_TEMP);

    if (!ifp.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return;
    }

    QTextStream out(&pageSource);
    while (!ifp.atEnd()) {
        QString buff = ifp.readLine();
        out << buff;
        if (buff.contains("<script src=\"qrc:/leaflet/leaflet.js\"></script>")) out << "<script type=\"text/javascript\" src=\"https://getfirebug.com/firebug-lite.js\"></script>\n";
        if (!buff.contains("// start map tiles")) continue;
        for (i = 0, j = 1; i < 6; i++) {
            if (mapStrings[i][0] == "") continue;
            QString title = mapStrings[i][0];
            QString url = mapStrings[i][1];
            QString attr = mapStrings[i][2];

            out << QStringLiteral("var tile%1 = L.tileLayer('%2', {\n").arg(j).arg(url);
            out << QStringLiteral("  attribution: \"<a href='%1' target='_blank'>%2</a>\",\n").arg(attr, title);
            out << QStringLiteral("  opacity: %1});\n").arg(MAP_OPACITY,0,'f',1);
            j++;
        }
        out << "var basemaps = {";
        for (i = 0, j = 1; i < 6; i++) {
            if (mapStrings[i][0] == "") continue;
            QString title = mapStrings[i][0];
            out << QStringLiteral("%1\"%2\":tile%3").arg((j == 1) ? "" : ",", title).arg(j);
            j++;
        }
        out << "};\n";
    }
    ifp.close();

#ifdef QWEBENGINE
    webBrowser->setHtml(pageSource);
    webBrowser->page()->setWebChannel(webChannel);
    webBrowser->show();
    loaded = false;
#endif

    timerLL.start();
}
//---------------------------------------------------------------------------
void MapView::timerLLTimer()
{
    if (!setState(0)) return;

    setView(0, center_latitude, center_longitude, INIT_ZOOM);
    updateMap();

    timerLL.stop();
}
//---------------------------------------------------------------------------
void MapView::showMapGM()
{
    QString pageSource;

    QFile ifp(RTKLIB_GM_TEMP);

    if (!ifp.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return;
    }

    QTextStream out(&pageSource);
    while (!ifp.atEnd()) {
        QString buff = ifp.readLine();

        if (!apiKey.isEmpty()) {
            buff = buff.replace(QStringLiteral("_insertKeyHere_"), apiKey);
        }

        out << buff;
    }
    ifp.close();

#ifdef QWEBENGINE
    webBrowser->setHtml(pageSource);
    webBrowser->page()->setWebChannel(webChannel);
    webBrowser->show();
    loaded = false;
#endif
    timerGM.start();

}
//---------------------------------------------------------------------------
void MapView::timerGMTimer()
{
    if (!setState(1)) return;

    setView(1, center_latitude, center_longitude, INIT_ZOOM);
    updateMap();

    timerGM.stop();
}

//---------------------------------------------------------------------------
void MapView::setView(int map, double lat, double lon, int zoom)
{
    execFunction(map, QStringLiteral("SetView(%1,%2,%3)").arg(lat, 0, 'f', 9).arg(lon, 0, 'f', 9).arg(zoom));
}
//---------------------------------------------------------------------------
void MapView::updateMap()
{
    setCenter(center_latitude, center_longitude);
    for (int i = 0; i < 255; i++) {
        if (std::isnan(markPosition[i][0]) || std::isnan(markPosition[i][0])) continue;
        addMark(selectedMap, i, markPosition[i][0], markPosition[i][1], markTitle[i], markMessage[i]);
        if (markState[i]) showMark(i);
        else hideMark(i);
    }
}
//---------------------------------------------------------------------------
void MapView::selectMap(int map)
{
    selectedMap = map;
    setView(map, center_latitude, center_longitude, INIT_ZOOM);
    showMap(map);
}

//---------------------------------------------------------------------------
void MapView::setCenter(double lat, double lon)
{
    QString func = QStringLiteral("SetCent(%1,%2)").arg(lat, 0, 'f', 9).arg(lon, 0, 'f', 9);
    center_latitude = lat;
    center_longitude = lon;

    execFunction(selectedMap, func);
}
//---------------------------------------------------------------------------
void MapView::setViewBounds(double min_lat, double min_lon, double max_lat, double max_lon)
{ //TODO: to be implemented for Google Maps
    execFunction(selectedMap, QStringLiteral("SetVuewBounds(%1,%2,%3,%4)").arg(min_lat, 0, 'f', 9).arg(min_lon, 0, 'f', 9).
                      arg(max_lat, 0, 'f', 9).arg(max_lon, 0, 'f', 9));
};
//---------------------------------------------------------------------------
void MapView::addMark(int map, int index, double lat, double lon, const QString &title, const QString &msg)
{
    QString func = QStringLiteral("AddMark(%1,%2,'%3','%4')").arg(lat, 0, 'f', 9).arg(lon, 0, 'f', 9).arg(title, msg);

    if (index >= NUM_MARK) return;

    markTitle[index] = title;
    markMessage[index] = msg;
    markPosition[index][0] = lat;
    markPosition[index][1] = lon;
    markState[index] = 1;

    execFunction(map, func);
    func = QStringLiteral("ShowMark('%1')").arg(title);
    execFunction(map, func);
}
//---------------------------------------------------------------------------
void MapView::setMark(int index, const QString &title, double lat, double lon)
{
    if (index >= NUM_MARK) return;

    markTitle[index] = title;
    QString func = QStringLiteral("PosMark(%1,%2,'%3')").arg(lat, 0, 'f', 9).arg(lon, 0, 'f', 9).arg(title);

    markPosition[index][0] = lat;
    markPosition[index][1] = lon;

    execFunction(selectedMap, func);
}
//---------------------------------------------------------------------------
void MapView::showMark(int index)
{
    if (index >= NUM_MARK) return;

    QString title = markTitle[index];
    QString func = QStringLiteral("ShowMark('%1')").arg(title);

    markState[index] = 1;
    execFunction(selectedMap,func);
}
//---------------------------------------------------------------------------
void MapView::hideMark(int index)
{
    if (index >= NUM_MARK) return;

    QString title = markTitle[index];
    QString func = QStringLiteral("HideMark('%1')").arg(title);

    markState[index] = 0;
    execFunction(selectedMap,func);
}
//---------------------------------------------------------------------------
void MapView::highlightMark(int index)
{
    if (index >= NUM_MARK) return;

    QString title = markTitle[index];
    QString func = QStringLiteral("HighlightMark('%1')").arg(title);

    highlightedMark = index;
    execFunction(selectedMap, func);
}
//---------------------------------------------------------------------------
void MapView::clearMark()
{
    QString func = QStringLiteral("ClearMark()");

    highlightedMark = -1;
    for (int i = 0; i < NUM_MARK; i++) {
        markState[0] = markState[1] = 0;
        markPosition[i][0] = markPosition[i][1] = NAN;
    }
    execFunction(selectedMap,func);
}
//---------------------------------------------------------------------------
int MapView::setState(int map)
{
    Q_UNUSED(map)
    if (!loaded) return 0;
    return pageState->getText().toInt();
}
//---------------------------------------------------------------------------
void MapView::execFunction(int map, const QString &func)
{
    Q_UNUSED(map)
    if (!loaded) return;

#ifdef QWEBENGINE
    QWebEnginePage *page = webBrowser->page();
    if (page == NULL) return;

    page->runJavaScript(func);
#else
    Q_UNUSED(func)
#endif
}
//---------------------------------------------------------------------------
bool MapView::isLoaded()
{
    return setState(1);
}
//---------------------------------------------------------------------------
void MapView::loadOptions(QSettings &settings)
{
    QStringList strs;
    for (int i = 0; i < 6; i++)
    {
        if (i==0)
            strs = settings.value("mapview/mapstrs_0", "OpenStreetMap,"
                                                       "https://tile.openstreetmap.org/{z}/{x}/{y}.png,"
                                                       "https://osm.org/copyright").toString().split(",");
        else
            strs = settings.value(QStringLiteral("set/mapstring%1").arg(i), "").toString().split(",");
        if (strs.length() == 3)
        {
            for (int j = 0; j < 3; j++)
                mapStrings[i][j] = strs[j];
        }
    }
    apiKey = settings.value("mapview/apikey" ,"").toString();
}
//---------------------------------------------------------------------------
void MapView::saveOptions(QSettings &settings)
{
    for (int i = 0; i < 6; i++)
    {
        QString str = mapStrings[i][0] + "," + mapStrings[i][1] + "," + mapStrings[i][2];
        settings.setValue(QStringLiteral("set/mapstring%1").arg(i), str);
    }

    settings.setValue("mapview/apikey", apiKey);
}
//---------------------------------------------------------------------------
