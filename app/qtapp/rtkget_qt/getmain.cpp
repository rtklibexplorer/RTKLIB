//---------------------------------------------------------------------------
// rtkget : gnss data downloader
//
//          Copyright (C) 2012 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann
//
// options : rtkget [-t title][-i file]
//
//           -t title   window title
//           -i file    ini file path
//
// version : $Revision:$ $Date:$
// history : 2012/12/28  1.0 new
//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QDesktopServices>
#include <QCloseEvent>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QFileDialog>
#include <QProcess>
#include <QDropEvent>
#include <QMimeData>
#include <QDebug>
#include <QtConcurrent/QtConcurrentRun>
#include <QFuture>
#include <QThread>
#include <QCompleter>
#include <QFileSystemModel>
#include <QStringList>

//---------------------------------------------------------------------------
#include "keydlg.h"
#include "aboutdlg.h"
#include "getoptdlg.h"
#include "staoptdlg.h"
#include "timedlg.h"
#include "viewer.h"
#include "helper.h"

#include "ui_getmain.h"

#include "getmain.h"
#include "rtklib.h"

#include <clocale>
#include <cstdio>

#define PRGNAME     "RtkGet Qt"  // program name

#define URL_FILE    "../../../data/URL_LIST.txt"
#define TEST_FILE   "rtkget_test.txt"
#define TRACE_FILE  "rtkget_qt.trace"

#define MAX_URL     2048
#define MAX_URL_SEL 64
#define MAX_STA     2048
#define MAX_HIST    16

static int abortf = 0;          // abort flag

MainForm *mainForm;

// show message in message area ---------------------------------------------
extern "C" {
    extern int showmsg(const char *format, ...)
    {
        va_list arg;
        QString str;
        static QString buff2;
        char buff[1024], *p;

        va_start(arg, format);
        vsprintf(buff, format, arg);
        va_end(arg);

        if ((p = strstr(buff, "STAT="))) {
            // keep only the last 66 characters of buff
            buff2.append(QStringView{str}.right(66));
            if (buff2.endsWith('_')) buff2.remove(buff2.length() - 1, 1);
            // append everthing which follows after "STAT="
            buff2.append(p + 5);
            QMetaObject::invokeMethod(mainForm, "showMessage", Qt::QueuedConnection, Q_ARG(int, 3), Q_ARG(QString, QString(buff2)));
        } else if ((p = strstr(buff, "->"))) {
            *p = '\0';  // split message at "->"
            QMetaObject::invokeMethod(mainForm, "showMessage", Qt::QueuedConnection, Q_ARG(int, 1), Q_ARG(QString, QString(buff)));
            QMetaObject::invokeMethod(mainForm, "showMessage", Qt::QueuedConnection, Q_ARG(int, 2), Q_ARG(QString, QString(p + 2)));
        }
        return abortf;
    }
    void settspan(gtime_t, gtime_t)
    { //empty
    }
    void settime(gtime_t)
    { // empty
    }
}

class DownloadThread : public QThread
{
    // download data in a dedicated thread to keep GUI responsive
    public:
        QString usr, pwd, proxy;
        FILE *fp;
        url_t urls[MAX_URL_SEL];
        gtime_t ts, te;
        double ti;
        char *stas[MAX_STA], msg[1024];
        int nsta, nurl, seqnos, seqnoe, opts;
        QString logFile, dir;
        bool test, append;
        int columnCnt;
        int dateFormat;

        explicit DownloadThread(QObject *parent, const QString lf, bool a, bool t) : QThread(parent)
        {
            seqnos = 0;
            seqnoe = 0;
            opts = 0;
            ts.time = 0;
            te.time = 0;
            fp = NULL;
            test = t;
            logFile = lf;
            append = a;
            columnCnt = 0;
            dateFormat = 0;

            for (int i = 0; i < MAX_STA; i++) {
                stas[i] = new char [16];
                stas[i][0] = '\0';
            }
            ;
            for (int i = 0; i < MAX_URL_SEL; i++) memset(&urls[i], 0, sizeof(url_t));
            msg[0] = '\0';
            nurl = nsta = 0; ti = 0;
        }
        ~DownloadThread()
        {
            for (int i = 0; i < MAX_STA; i++) delete [] stas[i];
        }

    protected:
        void run()
        {
            if (!logFile.isEmpty()) {
                char path[1024];

                reppath(qPrintable(logFile), path, utc2gpst(timeget()), "", "");
                fp = fopen(path, append ? "a" : "w");
            }
            if (test)
                dl_test(ts, te, ti, urls, nurl, (const char **)stas, nsta, qPrintable(dir), columnCnt, dateFormat, fp);
            else
                dl_exec(ts, te, ti, seqnos, seqnoe, urls, nurl, (const char **)stas, nsta, qPrintable(dir), qPrintable(usr),
                    qPrintable(pwd), qPrintable(proxy), opts, msg, fp);
            if (fp)
                fclose(fp);
        }
};

//---------------------------------------------------------------------------
MainForm::MainForm(QWidget *parent)
    : QWidget(parent), ui(new Ui::MainForm)
{
    mainForm = this;
    ui->setupUi(this);

    setlocale(LC_NUMERIC, "C");

    setWindowTitle(QStringLiteral("%1 v.%2 %3").arg(PRGNAME, VER_RTKLIB, PATCH_LEVEL));

    viewer = new TextViewer(this);
    timeDialog = new TimeDialog(this);
    downOptDialog = new DownOptDialog(this);


    QCompleter *dirCompleter = new QCompleter(this);
    QFileSystemModel *dirModel = new QFileSystemModel(dirCompleter);
    dirModel->setRootPath("");
    dirModel->setFilter(QDir::AllDirs | QDir::Drives | QDir::NoDotAndDotDot);
    dirCompleter->setModel(dirModel);
    ui->cBDirectory->setCompleter(dirCompleter);

    connect(ui->btnAll, &QPushButton::clicked, this, &MainForm::selectDeselectAllStations);
    connect(ui->btnDir, &QPushButton::clicked, this, &MainForm::selectOutputDirectory);
    connect(ui->btnDownload, &QPushButton::clicked, this, &MainForm::download);
    connect(ui->btnExit, &QPushButton::clicked, this, &MainForm::close);
    connect(ui->btnFile, &QPushButton::clicked, this, &MainForm::openOutputDirectory);
    connect(ui->btnAbout, &QPushButton::clicked, this, &MainForm::showAboutDialog);
    connect(ui->btnKeywords, &QPushButton::clicked, this, &MainForm::showKeyDialog);
    connect(ui->btnLog, &QPushButton::clicked, this, &MainForm::viewLogFile);
    connect(ui->btnOptions, &QPushButton::clicked, this, &MainForm::showOptionsDialog);
    connect(ui->btnStations, &QPushButton::clicked, this, &MainForm::showStationDialog);
    connect(ui->btnTest, &QPushButton::clicked, this, &MainForm::testDownload);
    connect(ui->btnTray, &QPushButton::clicked, this, &MainForm::minimizeToTray);
    connect(ui->cBDataType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateDataListWidget);
    connect(ui->cBSubType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateDataListWidget);
    connect(ui->cBDirectory, &QComboBox::currentTextChanged, this, &MainForm::updateMessage);
    connect(ui->cBLocalDirectory, &QCheckBox::clicked, this, &MainForm::localDirectoryCheckBoxClicked);
    connect(ui->cBHidePasswd, &QCheckBox::clicked, this, &MainForm::updateEnable);
    connect(ui->dataListWidget, &QListWidget::clicked, this, &MainForm::dataListSelectionChanged);
    connect(ui->stationListWidget, &QListWidget::clicked, this, &MainForm::updateStationListLabel);
    connect(ui->btnTimeStart, &QPushButton::clicked, this, &MainForm::showStartTimeDetails);
    connect(ui->btnTimeStop, &QPushButton::clicked, this, &MainForm::showStopTimeDetails);
    connect(ui->cBTimeInterval, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(&trayIcon, &QSystemTrayIcon::activated, this, &MainForm::restoreFromTaskTray);
    connect(&busyTimer, &QTimer::timeout, this, &MainForm::busyTimerTriggered);

    for (int i = 0; i < 8; i++) {
        images[i].load(QString(":/buttons/wait%1").arg(i + 1));
        images[i] = images[i].scaledToHeight(16);
    };

    timerCnt = 0;

    trayIcon.setIcon(QPixmap(":/icons/rtkget"));
    setWindowIcon(QIcon(":/icons/rtkget"));

    setAcceptDrops(true);
}
//---------------------------------------------------------------------------
void MainForm::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    QString appFilename = QApplication::applicationFilePath();
    QFileInfo fi(appFilename);

    iniFilename = fi.absoluteDir().filePath(fi.baseName() + ".ini");

    QCommandLineParser parser;
    parser.setApplicationDescription(tr("RTK Get Qt"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption iniFileOption(QStringList() << "i" << "ini-file",
                     QCoreApplication::translate("main", "ini file to use"),
                     QCoreApplication::translate("main", "ini file"));
    parser.addOption(iniFileOption);

    QCommandLineOption titleOption(QStringList() << "t" << "title",
                       QCoreApplication::translate("main", "window title"),
                       QCoreApplication::translate("main", "title"));
    parser.addOption(titleOption);

    parser.process(*QApplication::instance());

    if (parser.isSet(iniFileOption))
        iniFilename = parser.value(iniFileOption);

    if (parser.isSet(titleOption))
        setWindowTitle(parser.value(titleOption));

    loadOptions();
    loadUrlList(downOptDialog->urlFile);
    updateDataListWidget();
    updateEnable();

    if (downOptDialog->traceLevel > 0) {
        traceopen(TRACE_FILE);
        tracelevel(downOptDialog->traceLevel);
    }
}
//---------------------------------------------------------------------------
void MainForm::closeEvent(QCloseEvent *)
{
    traceclose();

    saveOptions();
}
//---------------------------------------------------------------------------
void MainForm::openOutputDirectory()
{
    QString str;
    gtime_t ts, te;
    double ti;
    char path[1024] = ".";

    str = ui->cBLocalDirectory->isChecked() ? ui->cBDirectory->currentText() : ui->lbMessage2->text();
    getTime(&ts, &te, &ti);
    if (!str.isEmpty()) reppath(qPrintable(str), path, ts, "", "");

    QDesktopServices::openUrl(QUrl::fromLocalFile(path));
}
//---------------------------------------------------------------------------
void MainForm::viewLogFile()
{
    if (downOptDialog->logFile.isEmpty()) return;

    viewer->setWindowTitle(downOptDialog->logFile);
    viewer->read(downOptDialog->logFile);
    viewer->exec();
}
//---------------------------------------------------------------------------
void MainForm::testDownload()
{
    if (ui->btnTest->text().remove('&') == tr("Abort")) {
        ui->btnTest->setEnabled(false);
        abortf = 1;
        return;
    }
    
    processingThread = new DownloadThread(this, TEST_FILE, false, true);
    getTime(&processingThread->ts, &processingThread->te, &processingThread->ti);
    processingThread->nurl = selectUrl(processingThread->urls);
    if (timediff(processingThread->ts, processingThread->te) > 0.0 || processingThread->nurl <= 0) {
        ui->lbMessage3->setText(tr("No local data"));
        delete processingThread;
        return;
    }
    
    processingThread->nsta = selectStation(processingThread->stas);
    processingThread->columnCnt = downOptDialog->columnCnt;
    processingThread->dateFormat = downOptDialog->dateFormat;

    if (ui->cBLocalDirectory->isChecked())
        processingThread->dir = ui->cBDirectory->currentText();

    panelEnable(true);

    ui->btnTest->setEnabled(true);
    ui->btnTest->setText(tr("&Abort"));
    setWidgetTextColor(ui->lbMessage1, Qt::gray);
    ui->lbMessage3->setText("");
    abortf = 0;
    
    connect(processingThread, &QThread::finished, this, &MainForm::downloadFinished);
    
    processingThread->start();
}
//---------------------------------------------------------------------------
void MainForm::showOptionsDialog()
{
    QString urlfile_old = downOptDialog->urlFile;

    downOptDialog->exec();

    if (downOptDialog->result() != QDialog::Accepted) return;

    if (downOptDialog->urlFile == urlfile_old) return;

    loadUrlList(downOptDialog->urlFile);
    updateDataListWidget();
}
//---------------------------------------------------------------------------
void MainForm::showMessage(int i, const QString& msg)
{
    QLabel *lbl[] = {ui->lbMessage1, ui->lbMessage2, ui->lbMessage3};
    lbl[i]->setText(msg);
}
//---------------------------------------------------------------------------
void MainForm::download()
{
    QString str;

    if (ui->btnDownload->text().remove('&') == tr("Abort")) {
        ui->btnDownload->setEnabled(false);
        abortf = 1;
        return;
    }
    
    processingThread = new DownloadThread(this, downOptDialog->logFile, downOptDialog->logAppend, false);
    getTime(&processingThread->ts, &processingThread->te, &processingThread->ti);

    str = ui->lENumber->text();
    QStringList tokens = str.split('-');
    if (tokens.size() == 2) {
        processingThread->seqnos = tokens.at(0).toInt();
        processingThread->seqnoe = tokens.at(1).toInt();
    } else if (tokens.size() == 1) {
        processingThread->seqnos = processingThread->seqnoe = tokens.at(0).toInt();
    } else {
        delete processingThread;
        return;
    }
    
    processingThread->nurl = selectUrl(processingThread->urls);
    if (timediff(processingThread->ts, processingThread->te) > 0.0 || processingThread->nurl <= 0) {
        ui->lbMessage3->setText(tr("No download data"));
        delete processingThread;
        return;
    }
    
    processingThread->nsta = selectStation(processingThread->stas);
    processingThread->usr = ui->lEFtpLogin->text();
    processingThread->pwd = ui->lEFtpPasswd->text();
    processingThread->proxy = downOptDialog->proxyAddr;
    
    if (!ui->cBSkipExist->isChecked()) processingThread->opts |= DLOPT_FORCE;
    if (!ui->cBUnzip->isChecked()) processingThread->opts |= DLOPT_KEEPCMP;
    if (downOptDialog->holdErr) processingThread->opts |= DLOPT_HOLDERR;
    if (downOptDialog->holdList) processingThread->opts |= DLOPT_HOLDLST;

    if (ui->cBLocalDirectory->isChecked())
        processingThread->dir = ui->cBDirectory->currentText();

    abortf = 0;

    panelEnable(false);

    ui->btnDownload->setEnabled(true);
    ui->btnDownload->setText(tr("&Abort"));
    ui->lbMessage3->setText("");
    
    connect(processingThread, &QThread::finished, this, &MainForm::downloadFinished);

    busyTimer.start(200);
    
    processingThread->start();
}
//---------------------------------------------------------------------------
void MainForm::downloadFinished()
{
    panelEnable(true);
    updateEnable();

    // stop busy animation
    busyTimer.stop();
    ui->lbImage->setPixmap(QPixmap());

    // add processed directory to history
    if (ui->cBDirectory->isEnabled())
        addHistory(ui->cBDirectory);

    // if it was a tes download...
    if (processingThread->test) {
        ui->btnTest->setText(tr("&Test..."));
        setWidgetTextColor(ui->lbMessage1, Qt::black);
        ui->lbMessage3->setText("");

        viewer->setOption(2);  // allow to save file
        viewer->read(TEST_FILE);
        viewer->setWindowTitle(tr("Local File Test"));
        viewer->exec();

        remove(TEST_FILE);
    } else {
        ui->btnDownload->setText(tr("&Download"));
        ui->lbMessage3->setText(processingThread->msg);
    }

    updateMessage();
    updateEnable();
    
    delete processingThread;
}

//---------------------------------------------------------------------------
void MainForm::showStationDialog()
{
    StaListDialog staListDialog(this);
    QStringList stations;

    for (int i = 0; i < ui->stationListWidget->count(); i++)
        stations.append(ui->stationListWidget->item(i)->text());
    staListDialog.setStationList(stations);

    staListDialog.exec();

    if (staListDialog.result() != QDialog::Accepted) return;

    ui->stationListWidget->clear();
    foreach (const QString &str, staListDialog.getStationList()) {
        ui->stationListWidget->addItem(str);
    };

    updateStationListLabel();
    ui->btnAll->setText("&All");
}
//---------------------------------------------------------------------------
void MainForm::selectOutputDirectory()
{
    QString dir = ui->cBDirectory->currentText();

    dir = QDir::toNativeSeparators(QFileDialog::getExistingDirectory(this, tr("Output Directory"), dir));

    if (dir.isEmpty())
        return;

    ui->cBDirectory->insertItem(0, dir);
    ui->cBDirectory->setCurrentIndex(0);
}
//---------------------------------------------------------------------------
void MainForm::showStartTimeDetails()
{
    QDateTime time(ui->dateTimeStart->dateTime());
    gtime_t t1;

    t1.time = static_cast<time_t>(time.toSecsSinceEpoch());
    t1.sec = time.time().msec() / 1000;
    timeDialog->setTime(t1);
    timeDialog->exec();
}
//---------------------------------------------------------------------------
void MainForm::showStopTimeDetails()
{
    QDateTime time(ui->dateTimeStop->dateTime());
    gtime_t t2;

    t2.time = static_cast<time_t>(time.toSecsSinceEpoch());
    t2.sec = time.time().msec() / 1000;
    timeDialog->setTime(t2);
    timeDialog->exec();
}
//---------------------------------------------------------------------------
void MainForm::selectDeselectAllStations()
{
    int i, station_cnt = 0;

    for (i = ui->stationListWidget->count() - 1; i >= 0; i--) {
        ui->stationListWidget->item(i)->setSelected(ui->btnAll->text() == tr("&All"));
        if (ui->stationListWidget->item(i)->isSelected())
            station_cnt++;
    }

    ui->btnAll->setText(ui->btnAll->text() == tr("&All") ? tr("&Clear") : tr("&All"));
    ui->lbStation->setText(tr("Stations (%1)").arg(station_cnt));
}
//---------------------------------------------------------------------------
void MainForm::showKeyDialog()
{
    KeyDialog keyDialog(this);

    keyDialog.setFlag(3);
    keyDialog.exec();
}
//---------------------------------------------------------------------------
void MainForm::showAboutDialog()
{
    AboutDialog aboutDialog(this, QPixmap(":/icons/rtkget"), PRGNAME);

    aboutDialog.exec();
}
//---------------------------------------------------------------------------
void MainForm::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasFormat("text/uri-list"))
        event->acceptProposedAction();
}
//---------------------------------------------------------------------------
void MainForm::dropEvent(QDropEvent *event)
{
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
    QPoint pos = event->position().toPoint();
#else
    QPoint pos = event->pos();
#endif
    if (ui->stationListWidget == childAt(pos))
        loadStationFile(event->mimeData()->text());

    event->acceptProposedAction();
}
//---------------------------------------------------------------------------
void MainForm::minimizeToTray()
{
    setVisible(false);
    trayIcon.setVisible(true);
}
//---------------------------------------------------------------------------
void MainForm::restoreFromTaskTray(QSystemTrayIcon::ActivationReason reason)
{
    if ((reason != QSystemTrayIcon::DoubleClick) && (reason != QSystemTrayIcon::Trigger)) return;

    setVisible(true);
    trayIcon.setVisible(false);
}
//---------------------------------------------------------------------------
void MainForm::localDirectoryCheckBoxClicked()
{
    updateMessage();
    updateEnable();
}
//---------------------------------------------------------------------------
void MainForm::dataListSelectionChanged()
{
    updateMessage();
    ui->lbMessage3->setText("");
}
//---------------------------------------------------------------------------
void MainForm::busyTimerTriggered()
{
    ui->lbImage->setPixmap(images[timerCnt % 8]);
    qApp->processEvents();

    timerCnt++;
}
//---------------------------------------------------------------------------
void MainForm::loadOptions()
{
    QSettings settings(iniFilename, QSettings::IniFormat);
    QStringList stas;

    ui->dateTimeStart->setDate(settings.value("opt/startd", "2020/01/01").toDate());
    ui->dateTimeStart->setTime(settings.value("opt/starth", "00:00").toTime());
    ui->dateTimeStop->setDate(settings.value("opt/endd", "2020/01/01").toDate());
    ui->dateTimeStop->setTime(settings.value("opt/endh", "00:00").toTime());
    ui->cBTimeInterval->setCurrentText(settings.value("opt/timeint", "24 H").toString());
    ui->lENumber->setText(settings.value("opt/number", "0").toString());
    ui->lEFtpLogin->setText(settings.value("opt/login", "anonymous").toString());
    ui->lEFtpPasswd->setText(settings.value("opt/passwd", "user@").toString());
    ui->cBUnzip->setChecked(settings.value("opt/unzip", true).toBool());
    ui->cBSkipExist->setChecked(settings.value("opt/skipexist", true).toBool());
    ui->cBHidePasswd->setChecked(settings.value("opt/hidepasswd", false).toBool());
    ui->cBLocalDirectory->setChecked(settings.value("opt/localdirena", false).toBool());

    ui->stationListWidget->clear();


    stas = settings.value("sta/stations", "").toString().split(",");
    foreach(QString sta, stas){
        if (!sta.isEmpty())
            ui->stationListWidget->addItem(sta);
    }

    readHistory(settings, "dir", ui->cBDirectory);

    ui->cBDirectory->insertItem(0, settings.value("opt/localdir", "").toString()); ui->cBDirectory->setCurrentIndex(0);
    ui->cBDataType->insertItem(0, settings.value("opt/datatype", "").toString()); ui->cBDataType->setCurrentIndex(0);

    downOptDialog->loadOptions(settings);
    viewer->loadOptions(settings);
}
//---------------------------------------------------------------------------
void MainForm::saveOptions()
{
    QSettings settings(iniFilename, QSettings::IniFormat);

    settings.setValue("opt/startd", ui->dateTimeStart->date());
    settings.setValue("opt/starth", ui->dateTimeStart->time());
    settings.setValue("opt/endd", ui->dateTimeStop->date());
    settings.setValue("opt/endh", ui->dateTimeStop->time());
    settings.setValue("opt/timeint", ui->cBTimeInterval->currentText());
    settings.setValue("opt/number", ui->lENumber->text());
    settings.setValue("opt/login", ui->lEFtpLogin->text());
    settings.setValue("opt/passwd", ui->lEFtpPasswd->text());
    settings.setValue("opt/unzip", ui->cBUnzip->isChecked());
    settings.setValue("opt/skipexist", ui->cBSkipExist->isChecked());
    settings.setValue("opt/hidepasswd", ui->cBHidePasswd->isChecked());
    settings.setValue("opt/localdirena", ui->cBLocalDirectory->isChecked());
    settings.setValue("opt/localdir", ui->cBDirectory->currentText());
    settings.setValue("opt/datatype", ui->cBDataType->currentText());


    QStringList stationList;
    for (int i = 0; i < ui->stationListWidget->count(); i++)
        stationList.append(ui->stationListWidget->item(i)->text());
    settings.setValue(QString("sta/stations"), stationList.join(','));

    writeHistory(settings, "dir", ui->cBDirectory);

    downOptDialog->saveOptions(settings);
    viewer->saveOptions(settings);
}
//---------------------------------------------------------------------------
void MainForm::loadUrlList(const QString &file)
{
    url_t *urls_list;
    QString subtype, basetype;
    const char *sel[1];
    int i, n;

    sel[0] = (char*)"*";

    urls_list = new url_t [MAX_URL];

    types.clear();
    urls.clear();
    locals.clear();
    ui->cBDataType->clear();
    ui->cBSubType->clear();
    ui->dataListWidget->clear();
    ui->cBDataType->addItem(tr("ALL"));
    ui->cBSubType->addItem("");

    n = dl_readurls(file.isEmpty() ? URL_FILE : qPrintable(file), (const char **)sel, 1, urls_list, MAX_URL);

    for (i = 0; i < n; i++) {
        int p;
        types.append(urls_list[i].type);
        urls.append(urls_list[i].path);
        locals.append(urls_list[i].dir);

        p = types.last().indexOf('_');
        if (p == -1) continue;
        basetype = types.last().mid(0, p);
        if (ui->cBDataType->findText(basetype) == -1)
            ui->cBDataType->addItem(basetype);

        subtype = types.last().mid(p + 1);
        if ((p = subtype.indexOf('_')) != -1) subtype = subtype.mid(0, p);

        if (ui->cBSubType->findText(subtype) == -1)
            ui->cBSubType->addItem(subtype);
    }
    ui->cBDataType->setCurrentIndex(0);
    ui->cBSubType->setCurrentIndex(0);

    delete [] urls_list;
}
//---------------------------------------------------------------------------
void MainForm::loadStationFile(const QString &file)
{
    QFile f(file);
    QByteArray buff;

    if (!f.open(QIODevice::ReadOnly)) return;

    ui->stationListWidget->clear();

    while (!f.atEnd()) {
        buff = f.readLine();
        buff = buff.mid(buff.indexOf('#'));
        ui->stationListWidget->addItem(buff);
    }

    updateStationListLabel();
    ui->btnAll->setText("&All");
}
//---------------------------------------------------------------------------
void MainForm::getTime(gtime_t *ts, gtime_t *te, double *ti)
{
    QString interval_str;
    double eps[6] = {2010, 1, 1}, epe[6] = {2010, 1, 1}, val;

    // start time
    eps[0] = ui->dateTimeStart->date().year(); eps[1] = ui->dateTimeStart->date().month(); eps[2] = ui->dateTimeStart->date().day();
    eps[3] = ui->dateTimeStart->time().hour(); eps[4] = ui->dateTimeStart->time().minute(); eps[5] = 0;

    // end time
    epe[0] = ui->dateTimeStop->date().year(); epe[1] = ui->dateTimeStop->date().month(); epe[2] = ui->dateTimeStop->date().day();
    epe[3] = ui->dateTimeStop->time().hour(); epe[4] = ui->dateTimeStop->time().minute(); epe[5] = 0;

    *ts = epoch2time(eps);
    *te = epoch2time(epe);
    *ti = 86400.0;

    interval_str = ui->cBTimeInterval->currentText();

    if (interval_str == "-") {
        *te = *ts;
    }

    QStringList tokens = interval_str.split(" ", Qt::SkipEmptyParts);
    if (tokens.size() == 0) return;

    val = tokens.at(0).toDouble();
    if (tokens.size() == 1) {
        if (tokens.at(1) == tr("day"))
            *ti = val * 86400.0;
        else if (tokens.at(1) == tr("min"))
            *ti = val * 60.0;
        else
            *ti = val * 3600.0;
    } else {
        // assume unit of hours
        *ti = val * 3600.0;
    }
}
//---------------------------------------------------------------------------
// prepare the selected urls for download
//
int MainForm::selectUrl(url_t *urls)
{
    QString str, file = downOptDialog->urlFile;
    char *types[MAX_URL_SEL];
    int i, nurl_in = 0, nurl_out;

    for (i = 0; i < ui->dataListWidget->count() && nurl_in < MAX_URL_SEL; i++) {
        if (!ui->dataListWidget->item(i)->isSelected()) continue;

        str = ui->dataListWidget->item(i)->text();
        types[nurl_in] = new char[str.length()];
        strncpy(types[nurl_in++], qPrintable(str), str.length());
    }
    if (downOptDialog->urlFile.isEmpty()) file = URL_FILE;

    nurl_out = dl_readurls(qPrintable(file), (const char **)types, nurl_in, urls, MAX_URL_SEL);

    for (i = 0; i < nurl_in; i++) delete []types[i];

    return nurl_out;
}
//---------------------------------------------------------------------------
// prepare selected stations for download
//
int MainForm::selectStation(char **stas)
{
    QString str;
    int i, nsta = 0, len;

    for (i = 0; i < ui->stationListWidget->count() && nsta < MAX_STA; i++) {
        if (!ui->stationListWidget->item(i)->isSelected()) continue;

        str = ui->stationListWidget->item(i)->text();
        len = str.length();
        if (str.indexOf(' ') != -1) len = str.indexOf(' ');
        if (len > 15) len = 15;
        strncpy(stas[nsta], qPrintable(str), static_cast<unsigned int>(len));
        stas[nsta++][len] = '\0';
    }
    return nsta;
}
//---------------------------------------------------------------------------
void MainForm::updateDataListWidget()
{
    QString str;
    QString type, subtype;
    int i;

    ui->dataListWidget->clear();

    for (i = 0; i < types.size(); i++) {
        str = types.at(i);
        QStringList tokens = str.split('_');

        type = subtype = "";
        if (tokens.size() > 1) {
            type = tokens.at(0);
            subtype = tokens.at(1);
        } else {
            type = tokens.at(0);
        }

        if (ui->cBDataType->currentText() != tr("ALL") && ui->cBDataType->currentText() != type) continue;
        if (!ui->cBSubType->currentText().isEmpty() && ui->cBSubType->currentText() != subtype) continue;
        ui->dataListWidget->addItem(types.at(i));
    }
    ui->lbMessage1->setText("");
    ui->lbMessage2->setText("");
}
//---------------------------------------------------------------------------
void MainForm::updateMessage()
{
    int i, j, n = 0;

    for (i = 0; i < ui->dataListWidget->count(); i++) {
        if (!ui->dataListWidget->item(i)->isSelected()) continue;

        for (j = 0; j < types.count(); j++) {
            if (ui->dataListWidget->item(i)->text() != types.at(j)) continue;

            ui->lbMessage1->setText(urls.at(j));
            ui->lbMessage2->setText(ui->cBLocalDirectory->isChecked() ? ui->cBDirectory->currentText() : locals.at(j));
            ui->lbMessage1->setToolTip(ui->lbMessage1->text());
            ui->lbMessage2->setToolTip(ui->lbMessage2->text());
            n++;
            break;
        }
    }
    if (n >= 2) {
        ui->lbMessage1->setText(ui->lbMessage1->text() + " ...");
        if (!ui->cBLocalDirectory->isChecked())
            ui->lbMessage2->setText(ui->lbMessage2->text() + " ...");
    }
}
//---------------------------------------------------------------------------
void MainForm::updateStationListLabel()
{
    int i, n = 0;

    for (i = 0; i < ui->stationListWidget->count(); i++)
        if (ui->stationListWidget->item(i)->isSelected())
            n++;

    ui->lbStation->setText(tr("Stations (%1)").arg(n));
}
//---------------------------------------------------------------------------
void MainForm::updateEnable()
{
    ui->cBDirectory->setEnabled(ui->cBLocalDirectory->isChecked());
    ui->btnDir->setEnabled(ui->cBLocalDirectory->isChecked());

    if (ui->cBHidePasswd->isChecked())
        ui->lEFtpPasswd->setEchoMode(QLineEdit::Password);
    else
        ui->lEFtpPasswd->setEchoMode(QLineEdit::Normal);

    ui->lbEnd->setEnabled(ui->cBTimeInterval->currentText() != "-");
    ui->dateTimeStop->setEnabled(ui->cBTimeInterval->currentText() != "-");
    ui->btnTimeStop->setEnabled(ui->cBTimeInterval->currentText() != "-");
}
//---------------------------------------------------------------------------
void MainForm::panelEnable(int ena)
{
    ui->timePanel->setEnabled(ena);
    ui->stationPanel->setEnabled(ena);
    ui->btnFile->setEnabled(ena);
    ui->btnLog->setEnabled(ena);
    ui->btnOptions->setEnabled(ena);
    ui->btnTest->setEnabled(ena);
    ui->btnDownload->setEnabled(ena);
    ui->btnExit->setEnabled(ena);
}
// --------------------------------------------------------------------------
void MainForm::readHistory(QSettings &setting, const QString &key, QComboBox *combo)
{
    QString item;
    int i;

    combo->clear();

    for (i = 0; i < MAX_HIST; i++) {
        item = setting.value(QStringLiteral("history/%1_%2").arg(key).arg(i, 3), "").toString();
        if (!item.isEmpty()) combo->addItem(item);
    }
}
// --------------------------------------------------------------------------
void MainForm::writeHistory(QSettings &setting, const QString &key, QComboBox *combo)
{
    int i;

    for (i = 0; i < combo->count(); i++)
        setting.setValue(QStringLiteral("history/%1_%2").arg(key).arg(i), combo->itemText(i));
}
// --------------------------------------------------------------------------
void MainForm::addHistory(QComboBox *combo)
{
    QString hist = combo->currentText();

    if (hist == "") return;
    int i = combo->currentIndex();
    combo->removeItem(i);
    combo->insertItem(0, hist);
    for (int i = combo->count() - 1; i >= MAX_HIST; i--) combo->removeItem(i);
    combo->setCurrentIndex(0);
}
//---------------------------------------------------------------------------
int MainForm::execCommand(const QString &cmd, const QStringList &opt)
{
    return QProcess::startDetached(cmd, opt);
}
//---------------------------------------------------------------------------
