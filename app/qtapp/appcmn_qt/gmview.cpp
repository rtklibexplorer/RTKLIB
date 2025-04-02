//---------------------------------------------------------------------------
// gmview.c: google map view
//---------------------------------------------------------------------------
#include <QWebEngineView>
#include <QWebEnginePage>
#include <QWebChannel>

#include <QFile>
#include <QShowEvent>

#include "gmview.h"
#include "rtklib.h"

#include "gm_template.h"

//---------------------------------------------------------------------------
GoogleMapView::GoogleMapView(QWidget *parent)
    : QDialog(parent)
{
    loaded = false;
    setupUi(this);

    connect(btnClose, &QPushButton::clicked, this, &GoogleMapView::btnCloseClicked);
    connect(btnShrink, &QPushButton::clicked, this, &GoogleMapView::btnShrinkClicked);
    connect(btnExpand, &QPushButton::clicked, this, &GoogleMapView::btnExpandClicked);
    connect(btnFixCenter, &QPushButton::clicked, this, &GoogleMapView::btnFixCenterClicked);
    connect(&loadTimer, &QTimer::timeout, this, &GoogleMapView::loadTimerExpired);

    webBrowser = new QWebEngineView(mapPanel);
    QHBoxLayout *layout = new QHBoxLayout();
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(webBrowser);
    mapPanel->setLayout(layout);
    pageState = new GMPageState(this);

    connect(webBrowser, &QWebEngineView::loadFinished, this, &GoogleMapView::pageLoaded);

    state = 0;
    latitude = longitude = 0.0;
    zoom = 2;
    fixCenter = true;

    load_page();
    htmlPage = getHtmlPage();
}
//---------------------------------------------------------------------------
void GoogleMapView::load_page()
{
    webBrowser->setHtml(htmlPage);
    QWebChannel *channel = new QWebChannel(this);
    channel->registerObject(QStringLiteral("state"), pageState);

    webBrowser->page()->setWebChannel(channel);

    webBrowser->show();

    loadTimer.start(300);
}
//---------------------------------------------------------------------------
int GoogleMapView::setApiKey(QString apiKey)
{
    htmlPage.replace("_APIKEY_", apiKey);

    load_page();

    return 0;
}
//---------------------------------------------------------------------------
void GoogleMapView::btnCloseClicked()
{
    trace(2,"gmview close\n");
    close();
}
//---------------------------------------------------------------------------
void GoogleMapView::pageLoaded(bool ok)
{
    if (!ok) return;

    QFile webchannel(":/html/qwebchannel.js");
    webchannel.open(QIODevice::ReadOnly);
    webBrowser->page()->runJavaScript(webchannel.readAll());
    webBrowser->page()->runJavaScript("new QWebChannel(qt.webChannelTransport,function(channel) {channel.objects.state.text=document.getElementById('state').value;});");

    loaded = true;
}
//---------------------------------------------------------------------------
void GoogleMapView::loadTimerExpired()
{
    if (!getState()) return;

    state = 1;

    setView(latitude, longitude, zoom);

    addMark(0.0, 0.0, "SOL1", tr("SOLUTION 1"));
    addMark(0.0, 0.0, "SOL2", tr("SOLUTION 2"));

    hideMark(1);
    hideMark(2);

    for (int i = 0; i < 2; i++) markPosition[i][0] = markPosition[i][1] = 0.0;

    loadTimer.stop();
}
//---------------------------------------------------------------------------
void GoogleMapView::btnShrinkClicked()
{
    setZoom(zoom - 1);
}
//---------------------------------------------------------------------------
void GoogleMapView::btnExpandClicked()
{
    setZoom(zoom + 1);
}
//---------------------------------------------------------------------------
void GoogleMapView::btnFixCenterClicked()
{
    fixCenter = btnFixCenter->isChecked();
    if (fixCenter) setCent(latitude, longitude);
}
//---------------------------------------------------------------------------
void GoogleMapView::resizeEvent(QResizeEvent *)
{
    if (fixCenter) setCent(latitude, longitude);
}
//---------------------------------------------------------------------------
void GoogleMapView::setView(double lat, double lon, int z)
{
    latitude = lat;
    longitude = lon;
    zoom = z;

    execFunction(QString("SetView(%1,%2,%3)").arg(lat, 0, 'f', 9).arg(lon, 0, 'f', 9).arg(z));
}
//---------------------------------------------------------------------------
void GoogleMapView::setCent(double lat, double lon)
{
    latitude = lat;
    longitude = lon;

    if (fixCenter) execFunction(QString("SetCent(%1,%2)").arg(lat, 0, 'f', 9).arg(lon, 0, 'f', 9));
}
//---------------------------------------------------------------------------
void GoogleMapView::setZoom(int z)
{
    if (z < 2 || z > 21) return;
    zoom = z;
    execFunction(QString("SetZoom(%1)").arg(zoom));
}
//---------------------------------------------------------------------------
void GoogleMapView::clearMark(void)
{
    execFunction("ClearMark()");
}
//---------------------------------------------------------------------------
void GoogleMapView::addMark(double lat, double lon,
                const QString &title, const QString &msg)
{
    execFunction(QString("AddMark(%1,%2,\"%3\",\"%4\")").arg(lat, 0, 'f', 9).arg(lon, 0, 'f', 9).arg(title).arg(msg));
}
//---------------------------------------------------------------------------
void GoogleMapView::setMark(int index, const double *pos)
{
    QString title;

    title = QString("SOL%1").arg(index);
    execFunction(QString("PosMark(%1,%2,\"%3\")").arg(pos[0] * R2D, 0, 'f', 9).arg(pos[1] * R2D, 0, 'f', 9).arg(title));

    markPosition[index - 1][0] = pos[0] * R2D;
    markPosition[index - 1][1] = pos[1] * R2D;
}
//---------------------------------------------------------------------------
void GoogleMapView::showMark(int index)
{
    QString title;

    title = QString("SOL%1").arg(index);
    execFunction(QString("ShowMark(\"%1\")").arg(title));
}
//---------------------------------------------------------------------------
void GoogleMapView::hideMark(int index)
{
    QString title;

    title = QString("SOL%1").arg(index);
    execFunction(QString("HideMark(\"%1\")").arg(title));
}
//---------------------------------------------------------------------------
int GoogleMapView::getState(void)
{
    if (!loaded) return 0;
    return pageState->getText().toInt();
}
//---------------------------------------------------------------------------
void GoogleMapView::execFunction(const QString &func)
{
    if (!loaded) return;

    QWebEnginePage *page = webBrowser->page();
    if (page == NULL) return;

    page->runJavaScript(func);
}
//---------------------------------------------------------------------------
void GoogleMapView::highlightMark(const QString &title)
{
    execFunction(QString("HighlightMark(\"%1\")").arg(title));
}
//---------------------------------------------------------------------------
