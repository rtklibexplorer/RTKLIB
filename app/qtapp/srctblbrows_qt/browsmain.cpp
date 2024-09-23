//---------------------------------------------------------------------------
// Ported to Qt by Jens Reimann
#include <clocale>

#include <QCloseEvent>
#include <QShowEvent>
#include <QSettings>
#include <QFileDialog>
#include <QTimer>
#include <QCommandLineParser>
#include <QtConcurrent>
#include <QFutureWatcher>
#include <QMetaObject>
#include <QLineEdit>

#include "rtklib.h"

#include "aboutdlg.h"
#include "mapview.h"
#include "browsmain.h"
#include "staoptdlg.h"
#include "mntpoptdlg.h"


//---------------------------------------------------------------------------

#define PRGNAME                 "NTRIP Browser Qt"
#define PRGVERSION              "1.0"
#define NTRIP_HOME              "rtcm-ntrip.org:2101"   // caster list home
#define NTRIP_TIMEOUT           10000                   // response timeout (ms)
#define MAXSRCTBL               512000                  // max source table size (bytes)
#define ENDSRCTBL               "ENDSOURCETABLE"        // end marker of table
#define ADDRESS_WIDTH           184                     // width of Address (px)

static char buff[MAXSRCTBL];                            // source table buffer

MainForm *mainForm;

extern "C" {
    extern int showmsg(const char *, ...)  {return 0;}
    extern void settime(gtime_t) {}
    extern void settspan(gtime_t, gtime_t) {}
}

/* get source table -------------------------------------------------------*/
static char *getsrctbl(const QString addr)
{
    static int lock = 0;
	stream_t str;
    char *p = buff, msg[MAXSTRMSG];
    unsigned int tick = tickget();

    if (lock) return NULL;
    else lock = 1;

    strinit(&str);

    if (!stropen(&str, STR_NTRIPCLI, STR_MODE_R, qPrintable(addr))) {
        lock = 0;
        QMetaObject::invokeMethod(mainForm, "showMsg", Qt::QueuedConnection, Q_ARG(QString, QT_TR_NOOP("stream open error")));
		return NULL;
	}
    QMetaObject::invokeMethod(mainForm, "showMsg", Qt::QueuedConnection, Q_ARG(QString, QT_TR_NOOP("connecting...")));

    while (p < buff + MAXSRCTBL - 1) {
        int ns = strread(&str, (uint8_t *)p, (buff + MAXSRCTBL - p - 1));
        p += ns; *p='\0';
        qApp->processEvents();
        int stat = strstat(&str, msg);

        QMetaObject::invokeMethod(mainForm, "showMsg", Qt::QueuedConnection, Q_ARG(QString, msg));

        if (stat <= 0) break;
        if (strstr(buff, ENDSRCTBL)) break;
        if ((int)(tickget() - tick) > NTRIP_TIMEOUT) {
            QMetaObject::invokeMethod(mainForm, "showMsg", Qt::QueuedConnection, Q_ARG(QString, QT_TR_NOOP("response timeout")));
			break;
		}
	}
	strclose(&str);
    lock = 0;
	return buff;
}
//---------------------------------------------------------------------------
MainForm::MainForm(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainForm)
{
    mainForm = this;
    ui->setupUi(this);
    ui->cBFilterFormat->setVisible(false);  // TODO: not yet implemented

    QCoreApplication::setApplicationName(PRGNAME);
    QCoreApplication::setApplicationVersion(PRGVERSION);
    setWindowTitle(QStringLiteral("%1 ver.%2 %3").arg(PRGNAME).arg(VER_RTKLIB, PATCH_LEVEL));

    setWindowIcon(QIcon(":/icons/srctblbrows"));

    // retrieve config file name
    QString prg_filename = QApplication::applicationFilePath();
    QFileInfo prg_fileinfo(prg_filename);
    iniFile = prg_fileinfo.absoluteDir().filePath(prg_fileinfo.baseName() + ".ini");

    mapView = new MapView(this);
    staListDialog = new StaListDialog(this);
    loadTimer = new QTimer(this);
    mountPointDialog = new MntpOptDialog(this);

    ui->btnTypeCas->setDefaultAction(ui->actMenuViewCas);
    ui->btnTypeNet->setDefaultAction(ui->actMenuViewNet);
    ui->btnTypeSrc->setDefaultAction(ui->actMenuViewSrc);
    ui->btnTypeStr->setDefaultAction(ui->actMenuViewStr);

    connect(ui->cBAddress, &QComboBox::textActivated, this, &MainForm::getTable);
    connect(ui->btnUpdate, &QPushButton::clicked, this, &MainForm::getTable);
    connect(ui->btnMap, &QPushButton::clicked, this, &MainForm::showMap);
    connect(ui->btnList, &QPushButton::clicked, this, &MainForm::getCaster);
    connect(ui->btnSta, &QPushButton::clicked, this, &MainForm::showStationDialog);
    connect(ui->cBStatationMask, &QCheckBox::clicked, this, &MainForm::updateEnable);
    connect(ui->actMenuOpen, &QAction::triggered, this, &MainForm::openSourceTable);
    connect(ui->actMenuSave, &QAction::triggered, this, &MainForm::saveSourceTable);
    connect(ui->actMenuQuit, &QAction::triggered, this, &MainForm::close);
    connect(ui->actMenuUpdateCaster, &QAction::triggered, this, &MainForm::getTable);
    connect(ui->actMenuUpdateTable, &QAction::triggered, this, &MainForm::getTable);
    connect(ui->actMenuViewCas, &QAction::triggered, this, &MainForm::viewCasterTable);
    connect(ui->actMenuViewNet, &QAction::triggered, this, &MainForm::viewNetTable);
    connect(ui->actMenuViewSrc, &QAction::triggered, this, &MainForm::viewSourceTable);
    connect(ui->actMenuViewStr, &QAction::triggered, this, &MainForm::viewStreamTable);
    connect(ui->actMenuAbout, &QAction::triggered, this, &MainForm::showAboutDialog);
    connect(loadTimer, &QTimer::timeout, this, &MainForm::loadTimerExpired);
    connect(ui->tWTableStr, &QTableWidget::cellClicked, this, &MainForm::streamTableSelectItem);
    connect(ui->tWTableStr, &QTableWidget::cellDoubleClicked, this, &MainForm::streamTableShowMountpoint);
    connect(ui->tWTableCas, &QTableWidget::cellDoubleClicked, this, &MainForm::casterTableSelectItem);

    ui->btnMap->setEnabled(false);
#ifdef QWEBENGINE
    ui->btnMap->setEnabled(true);
#endif

    ui->tWTableStr->setSortingEnabled(true);
    ui->tWTableCas->setSortingEnabled(true);
    ui->tWTableNet->setSortingEnabled(true);

    ui->cBAddress->lineEdit()->setPlaceholderText(NTRIP_HOME);

    ui->statusbar->addWidget(ui->lblMessage);

    strinitcom();

    loadTimer->setInterval(100);
}
//---------------------------------------------------------------------------
void MainForm::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    const QString default_column_width0 = "30,74,116,56,244,18,52,62,28,50,50,18,18,120,28,18,18,40,600,";
    const QString default_column_width1 = "30,112,40,96,126,18,28,50,50,160,40,600,0,0,0,0,0,0,0,";
    const QString default_column_width2 = "30,80,126,18,18,300,300,300,600,0,0,0,0,0,0,0,0,0,0,";
    QSettings settings(iniFile, QSettings::IniFormat);
    QString list, url;
    QStringList colw, stas;
    int i;

    QCommandLineParser parser;
    parser.setApplicationDescription(PRGNAME);
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("URL", QCoreApplication::translate("main", "file with list of Ntrip sources"));

    parser.process(*QApplication::instance());

    const QStringList &args = parser.positionalArguments();
    if (args.count() >= 1) url = args.at(0);

    list = settings.value("srctbl/addrlist", "").toString();
    QStringList items = list.split("@");
    foreach(QString item, items){
        ui->cBAddress->addItem(item);
    }

    if (!url.isEmpty()) {
        ui->cBAddress->setCurrentText(url);
        getTable();
    } else {
        QString url = settings.value("srctbl/address", "").toString();
        if (!url.isEmpty())
            ui->cBAddress->setCurrentText(url);
        else
            ui->cBAddress->setCurrentIndex(-1);
    }

    fontScale = QFontMetrics(ui->tWTableStr->font()).height() * 4;

    colw = settings.value("srctbl/colwidth0", default_column_width0).toString().split(',');
    i = 0;
    foreach(QString width, colw){
        ui->tWTableStr->setColumnWidth(i++, width.toDouble() * fontScale / 96);
    }

    colw = settings.value("srctbl/colwidth1", default_column_width1).toString().split(',');
    i = 0;
    foreach(QString width, colw){
        ui->tWTableCas->setColumnWidth(i++, width.toDouble() * fontScale / 96);
    }

    colw = settings.value("srctbl/colwidth2", default_column_width2).toString().split(',');
    i = 0;
    foreach(QString width, colw){
        ui->tWTableNet->setColumnWidth(i++, width.toDouble() * fontScale / 96);
	}

    stationList.clear();
    stas = settings.value("sta/stations", "").toString().split(',');
    foreach(QString sta, stas){
        stationList.append(sta);
    }

    mapView->loadOptions(settings);

    showTable();
    updateEnable();
    getTable();

    QTimer::singleShot(0, this, &MainForm::updateTable);
}
//---------------------------------------------------------------------------
void MainForm::closeEvent(QCloseEvent *)
{
    QSettings settings(iniFile, QSettings::IniFormat);
    QString list, colw;

    // save table layout
    settings.setValue("srctbl/address", ui->cBAddress->currentText());
    for (int i = 0; i < ui->cBAddress->count(); i++)
        list = list + ui->cBAddress->itemText(i) + "@";
    settings.setValue("srctbl/addrlist", list);

    colw = "";
    for (int i = 0; i < ui->tWTableStr->columnCount(); i++)
        colw = colw + QString::number(ui->tWTableStr->columnWidth(i) * 96 / fontScale);
    settings.setValue("srctbl/colwidth0", colw);

    colw = "";
    for (int i = 0; i < ui->tWTableCas->columnCount(); i++)
        colw = colw + QString::number(ui->tWTableCas->columnWidth(i) * 96 / fontScale);
    settings.setValue("srctbl/colwidth1", colw);

    colw = "";
    for (int i = 0; i < ui->tWTableNet->columnCount(); i++)
        colw = colw + QString::number(ui->tWTableNet->columnWidth(i) * 96 / fontScale);
    settings.setValue("srctbl/colwidth2", colw);

    settings.setValue("sta/stations", stationList.join(','));

    mapView->saveOptions(settings);
}
//---------------------------------------------------------------------------
void MainForm::openSourceTable()
{
    QString fileName = QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open"), QString(), tr("All File (*.*)")));
    QFile file(fileName);

    sourceTable = "";

    if (!file.open(QIODevice::ReadOnly)) return;

    sourceTable = file.readAll();

    addressCaster = ui->cBAddress->currentText();

    showTable();
    showMsg(tr("Source table loaded"));
}
//---------------------------------------------------------------------------
void MainForm::saveSourceTable()
{
    QString fileName = QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Save File"), QString(), tr("All File (*.*)")));
    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly)) return;

    file.write(sourceTable.toLatin1());

    showMsg(tr("Source table saved"));
}
//---------------------------------------------------------------------------
void MainForm::viewStreamTable()
{
    ui->actMenuViewCas->setChecked(false);
    ui->actMenuViewNet->setChecked(false);
    ui->actMenuViewSrc->setChecked(false);

    showTable();
    updateEnable();
}
//---------------------------------------------------------------------------
void MainForm::viewCasterTable()
{
    ui->actMenuViewStr->setChecked(false);
    ui->actMenuViewNet->setChecked(false);
    ui->actMenuViewSrc->setChecked(false);

    showTable();
    updateEnable();
}
//---------------------------------------------------------------------------
void MainForm::viewNetTable()
{
    ui->actMenuViewStr->setChecked(false);
    ui->actMenuViewCas->setChecked(false);
    ui->actMenuViewSrc->setChecked(false);

    showTable();
    updateEnable();
}
//---------------------------------------------------------------------------
void MainForm::viewSourceTable()
{
    ui->actMenuViewStr->setChecked(false);
    ui->actMenuViewCas->setChecked(false);
    ui->actMenuViewNet->setChecked(false);

    showTable();
    updateEnable();
}
//---------------------------------------------------------------------------
void MainForm::showAboutDialog()
{
    AboutDialog *aboutDialog = new AboutDialog(this, QPixmap(":/icons/srctblbrows"), PRGNAME);

    aboutDialog->exec();

    delete aboutDialog;
}
//---------------------------------------------------------------------------
void MainForm::showMap()
{
    int num = 0;
    float mean_lat = 0, mean_lon = 0, min_lat = 180, max_lat = -180, min_lon = 90, max_lon = -90;
    bool okay;

    updateMap();

    if (ui->tWTableStr->columnCount() < 11) return;

    for (int i = 0; i < ui->tWTableStr->rowCount(); i++) {
        QString latitudeText = ui->tWTableStr->item(i, 9)->text();
        QString longitudeText = ui->tWTableStr->item(i, 10)->text();
        float lat = latitudeText.toDouble(&okay); if (!okay) continue;
        float lon = longitudeText.toDouble(&okay); if (!okay) continue;

        if ((lat == 0) && (lon == 0))  // skip unknown positions
            continue;

        mean_lat += lat;
        mean_lon += lon;
        if (lat < min_lat) min_lat = lat;
        if (lat > max_lat) max_lat = lat;
        if (lon < min_lon) min_lon = lon;
        if (lon > max_lon) max_lon = lon;
        num++;
    }
    if (num > 0)
    {
        mean_lat /= num;
        mean_lon /= num;

        mapView->setCenter(mean_lat, mean_lon); // fallback until setViewBounds() is implemented for Google Maps
        mapView->setViewBounds(min_lat, min_lon, max_lat, max_lon);
    }
    mapView->show();
}
//---------------------------------------------------------------------------
void MainForm::streamTableSelectItem(int row, int col)
{
    Q_UNUSED(col);
    QString title;

    if (ui->tWTableStr->columnCount() < 2) return;

    if (0 <= row && row < ui->tWTableStr->rowCount()) {
        title = ui->tWTableStr->item(row, 1)->text();
        mapView->highlightMark(row);
        mapView->setWindowTitle(tr("NTRIP Data Stream Map: %1/%2").arg(ui->cBAddress->currentText(), title));
	}
}
//---------------------------------------------------------------------------
void MainForm::streamTableShowMountpoint(int row, int col)
{
    Q_UNUSED(col);
    if (ui->tWTableStr->columnCount() < 2) return;

    if (0 <= row && row < ui->tWTableStr->rowCount()) {
        mountPointDialog->setOption(0);
        mountPointDialog->setMountPoint(ui->tWTableStr->item(row, 1)->text());
        QString mntpStr;
        for (int i = 0; i < ui->tWTableStr->columnCount(); i++)
            mntpStr += ui->tWTableStr->item(row, i)->text() + ";" ;

        mountPointDialog->setMountPointString(mntpStr);
        mountPointDialog->show();
    }
}
//---------------------------------------------------------------------------
void MainForm::casterTableSelectItem(int row, int col)
{
    Q_UNUSED(col);

    if (ui->tWTableStr->columnCount() < 3) return;

    if (0 <= row && row < ui->tWTableCas->rowCount()) {

        QString address = ui->tWTableCas->item(row, 1)->text();
        QString port = ui->tWTableCas->item(row, 2)->text();
        ui->cBAddress->setCurrentText(QStringLiteral("%1:%2").arg(address, port));

        getTable();
    }
}
//---------------------------------------------------------------------------
void MainForm::loadTimerExpired()
{
    if (!mapView->isLoaded()) return;

    updateMap();

    loadTimer->stop();
}
//---------------------------------------------------------------------------
void MainForm::getCaster()
{
    if (casterWatcher.isRunning()) return;

    QString addressText = ui->cBAddress->currentText();
    QString addr = NTRIP_HOME;

    if (!addressText.isEmpty()) addr = addressText;

    ui->btnList->setEnabled(false);
    ui->actMenuUpdateCaster->setEnabled(false);

    // update source table in background and call updateCaster() when finished
    QFuture<char *> tblFuture = QtConcurrent::run(getsrctbl, addr);
    connect(&casterWatcher, &QFutureWatcher<char*>::finished, this, &MainForm::updateCaster);
    casterWatcher.setFuture(tblFuture);
}
//---------------------------------------------------------------------------
void MainForm::updateCaster()
{
    QString currentAddress;
    QString srctbl;

    ui->btnList->setEnabled(true);
    ui->actMenuUpdateCaster->setEnabled(true);

    if ((srctbl = casterWatcher.result()).isEmpty()) return;

    currentAddress = ui->cBAddress->currentText();
    ui->cBAddress->clear();
    ui->cBAddress->setCurrentText(currentAddress);
    ui->cBAddress->addItem("");

    QStringList tokens;
    QStringList lines = srctbl.split('\n');
    foreach(const QString &line, lines) {
        if (!line.contains("CAS")) continue;

        tokens = line.split(";");
        if (tokens.size() < 3) continue;
        ui->cBAddress->addItem(tokens.at(1) + ":" + tokens.at(2), QString(line));
	}

    if (ui->cBAddress->count() > 1) ui->cBAddress->setCurrentIndex(0);
}
//---------------------------------------------------------------------------
void MainForm::getTable()
{
    if (tableWatcher.isRunning()) return;

    QString addressText = ui->cBAddress->currentText();
    QString addr = NTRIP_HOME;

    if (!addressText.isEmpty()) addr = addressText;

    ui->btnUpdate->setEnabled(false);
    ui->cBAddress->setEnabled(false);
    ui->actMenuUpdateTable->setEnabled(false);

    // retrieve data in background and call updateTable() when finished
    QFuture<char *> tblFuture = QtConcurrent::run(getsrctbl, addr);
    connect(&tableWatcher, &QFutureWatcher<char*>::finished, this, &MainForm::updateTable);
    tableWatcher.setFuture(tblFuture);
}
//---------------------------------------------------------------------------
void MainForm::updateTable()
{
    QString srctbl;

    ui->btnUpdate->setEnabled(true);
    ui->cBAddress->setEnabled(true);
    ui->actMenuUpdateTable->setEnabled(true);

    srctbl = tableWatcher.result();

    if (!srctbl.isEmpty()) {
        sourceTable = srctbl;
        addressCaster = ui->cBAddress->currentText();
	}

    showTable();
}
//---------------------------------------------------------------------------
void MainForm::showTable()
{
    const QString ti[3][19] =
        {{tr("No"), tr("Mountpoint"), tr("ID"), tr("Format"), tr("Format-Details"), tr("Carrier"), tr("Nav-System"),
          tr("Network"), tr("Country"), tr("Latitude"), tr("Longitude"), tr("NMEA"), tr("Solution"),
          tr("Generator"), tr("Compr-Encrp"), tr("Authentication"), tr("Fee"), tr("Bitrate"), ""},
         {tr("No"), tr("Host"),	tr("Port"),	tr("ID"), tr("Operator"), tr("NMEA"), tr("Country"), tr("Latitude"), tr("Longitude"),
          tr("Fallback Host"), tr("Fallback Port"), "", "", "", "", "", "", ""},
         {tr("No"), tr("ID"), tr("Operator"), tr("Authentication"), tr("Fee"), tr("Web-Net"), tr("Web-Str"), tr("Web-Reg"), "", "", "", "",
          "", "", "", "", "", ""}};

    QTableWidget *table[] = {ui->tWTableStr, ui->tWTableCas, ui->tWTableNet};
    QAction *action[] = {ui->actMenuViewStr, ui->actMenuViewCas, ui->actMenuViewNet, ui->actMenuViewSrc};
    int i, j, numStations, type;

    ui->tETableSrc->setVisible(false);
    for (i = 0; i < 3; i++)
        table[i]->setVisible(false);

    type = ui->actMenuViewStr->isChecked() ? 0 : (ui->actMenuViewCas->isChecked() ? 1 : (ui->actMenuViewNet->isChecked() ? 2 : 3));
    for (i = 0; i < 4; i++)
        action[i]->setChecked(i == type);

    if (type == 3) {
        ui->tETableSrc->setVisible(true);
        ui->tETableSrc->setPlainText("");
    } else {
        table[type]->setVisible(true);
        table[type]->setColumnCount(18);
        for (i = 0; i < 18; i++) {
            table[type]->setHorizontalHeaderItem(i, new QTableWidgetItem(ti[type][i]));
		}
	}
    if (addressCaster != ui->cBAddress->currentText()) return;

    if (type == 3) {
        ui->tETableSrc->setPlainText(sourceTable);
		return;
	}

    QStringList lines = sourceTable.split("\n");
    numStations = 0;
    foreach(QString line, lines) {
		switch (type) {
            case 0: if (line.startsWith("STR")) numStations++; break;
            case 1: if (line.startsWith("CAS")) numStations++; break;
            case 2: if (line.startsWith("NET")) numStations++; break;
		}
	}
    if (numStations <= 0) return;

    table[type]->setRowCount(numStations);
    j = 0;
    foreach(QString line, lines) {
		switch (type) {
            case 0: if (line.startsWith("STR")) break; else continue;
            case 1: if (line.startsWith("CAS")) break; else continue;
            case 2: if (line.startsWith("NET")) break; else continue;
		}
        table[type]->setItem(j, 0, new QTableWidgetItem(QString::number(j)));
        QStringList tokens = line.split(";");
        for (int i = 0; i < table[type]->columnCount() && i < tokens.size(); i++)
            table[type]->setItem(j, i, new QTableWidgetItem(tokens.at(i)));
		j++;
	}
    updateMap();
}
//---------------------------------------------------------------------------
void MainForm::showMsg(const QString &msg)
{
    ui->lblMessage->setText(msg);
}
//---------------------------------------------------------------------------
void MainForm::updateMap()
{
    QString title, msg, latitudeText, longitudeText;
    double latitude, longitude;
    bool okay;

    if (ui->cBAddress->currentText().isEmpty())
        mapView->setWindowTitle(tr("NTRIP Data Stream Map"));
    else
        mapView->setWindowTitle(tr("NTRIP Data Stream Map: %1").arg(ui->cBAddress->currentText()));
    mapView->clearMark();

    if (ui->tWTableStr->columnCount() < 14) return;

    for (int i = 0; i < ui->tWTableStr->rowCount(); i++) {
        latitudeText = ui->tWTableStr->item(i, 9)->text();
        longitudeText = ui->tWTableStr->item(i, 10)->text();
        latitude = latitudeText.toDouble(&okay); if (!okay) continue;
        longitude = longitudeText.toDouble(&okay); if (!okay) continue;

        title = ui->tWTableStr->item(i, 1)->text();
        msg = "<b>" + ui->tWTableStr->item(i, 1)->text() + "</b>: " + ui->tWTableStr->item(i, 2)->text() + " (" + ui->tWTableStr->item(i,8)->text() + ")<br>" +
              "Format: " + ui->tWTableStr->item(i, 3)->text() + ", " + ui->tWTableStr->item(i, 4)->text() + ", <br> " +
              "Nav-Sys: " + ui->tWTableStr->item(i, 6)->text() + "<br>" +
              "Network: " + ui->tWTableStr->item(i, 7)->text() + "<br>" +
              "Latitude/Longitude: " + ui->tWTableStr->item(i, 9)->text() + ", " + ui->tWTableStr->item(i, 10)->text() + "<br>" +
              "Generator: " + ui->tWTableStr->item(i, 13)->text();

        mapView->addMark(0, i, latitude, longitude, title, msg);
	}
}
//---------------------------------------------------------------------------
void MainForm::showStationDialog()
{
    staListDialog->setStationList(stationList);
    staListDialog->exec();
    stationList = staListDialog->getStationList();
}
//---------------------------------------------------------------------------
void MainForm::updateEnable()
{
    ui->btnSta->setEnabled(ui->cBStatationMask->isChecked());
    ui->btnMap->setEnabled(ui->actMenuViewStr->isChecked());
}
//---------------------------------------------------------------------------
