//---------------------------------------------------------------------------
// rtkpost_qt : post-processing analysis
//
//          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann
//
// options : rtkpost [-t title][-i file][-r file][-b file][-n file ...]
//                   [-d dir][-o file]
//                   [-ts y/m/d h:m:s][-te y/m/d h:m:s][-ti tint][-tu tunit]
//
//           -t title   window title
//           -i file    ini file path
//           -r file    rinex obs rover file
//           -b file    rinex obs base station file
//           -n file    rinex nav/clk, sp3, Bias-SINEX or ionex file
//           -d dir     output directory
//           -o file    output file
//           -ts y/m/d h:m:s time start
//           -te y/m/d h:m:s time end
//           -ti tint   time interval (s)
//           -tu tunit  time unit (hr)
//
// version : $Revision: 1.1 $ $Date: 2008/07/17 22:14:45 $
// history : 2008/07/14  1.0 new
//           2008/11/17  1.1 rtklib 2.1.1
//           2008/04/03  1.2 rtklib 2.3.1
//           2010/07/18  1.3 rtklib 2.4.0
//           2010/09/07  1.3 rtklib 2.4.1
//           2011/04/03  1.4 rtklib 2.4.2
//           2020/11/30  1.5 rtklib 2.4.3
//---------------------------------------------------------------------------
#include <clocale>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <QShowEvent>
#include <QCloseEvent>
#include <QCommandLineParser>
#include <QSettings>
#include <QFileDialog>
#include <QTimer>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QProcess>
#include <QtGlobal>
#include <QThread>
#include <QMimeData>
#include <QFileSystemModel>
#include <QCompleter>
#include <QRegularExpression>
#include <QTime>
#include <QMessageBox>

#include "rtklib.h"
#include "postmain.h"
#include "navi_post_opt.h"
#include "kmzconv.h"
#include "timedlg.h"
#include "keydlg.h"
#include "aboutdlg.h"
#include "viewer.h"
#include "helper.h"

#include "ui_postmain.h"

#define PRGNAME     "RtkPost Qt"
#define MAXHIST     20


// global variables ---------------------------------------------------------
static gtime_t tstart_ = {0, 0};         // time start for progress-bar
static gtime_t tend_ = {0, 0};         // time end for progress-bar

MainForm *mainForm;

extern "C" {

// show message in message area ---------------------------------------------
extern int showmsg(const char *format, ...)
{
    va_list arg;
    char buff[1024];

    if (*format) {
        va_start(arg, format);
        vsprintf(buff, format, arg);
        va_end(arg);
        QMetaObject::invokeMethod(mainForm, "showMessage", Qt::QueuedConnection,
                                  Q_ARG(QString, QString(buff)));
    }
    return mainForm->abortFlag;
}
// set time span of progress bar --------------------------------------------
extern void settspan(gtime_t ts, gtime_t te)
{
    tstart_ = ts;
    tend_ = te;
}
// set current time to show progress ----------------------------------------
extern void settime(gtime_t time)
{
    double tt;
    if (tend_.time != 0 && tstart_.time != 0 && (tt = timediff(tend_, tstart_)) > 0.0) {
        QMetaObject::invokeMethod(mainForm, "setProgress",
                  Qt::QueuedConnection,
                  Q_ARG(int, (int)(timediff(time, tstart_) / tt * 100.0 + 0.5)));
    }
}

} // extern "C"

ProcessingThread::ProcessingThread(QObject *parent) : QThread(parent)
{
    n = stat = 0;
    prcopt = prcopt_default;
    solopt = solopt_default;
    ts.time = ts.sec = 0;
    te.time = te.sec = 0;
    ti = tu = 0;
    for (int i = 0; i < 6; i++)
    {
        infile[i] = new char[1024];
        infile[i][0] = '\0';
    };
    outfile[0] = '\0';

    memset(&prcopt, 0, sizeof(prcopt_t));
    memset(&solopt, 0, sizeof(solopt_t));
    memset(&filopt, 0, sizeof(filopt_t));

    connect(this, &QThread::finished, this, &QObject::deleteLater);
}
ProcessingThread::~ProcessingThread()
{
    for (int i = 0; i < 6; i++) delete[] infile[i];
}
bool ProcessingThread::addInput(const QString & file) {
    if (file.isEmpty()) return true;  // skip empty files

    if (check_compression(file)) {
        strncpy(infile[n++], qPrintable(file), 1023);
        return true;
    }
    return false;
}
void ProcessingThread::run()
{
    if ((stat = postpos(ts, te, ti, tu, &prcopt, &solopt, &filopt, (const char **)infile, n, outfile,
                        qPrintable(rov), qPrintable(base))) == 1) {
        showmsg("Aborted");
    };
    emit done(stat);
}

QString ProcessingThread::toList(const QString & list) {
    QString stations;
    QStringList lines = list.split("\n");

    foreach(QString line, lines)
    {
        int index = line.indexOf("#");
        if (index == -1) index = line.length();
        stations += line.mid(0, index) + " ";
    }
    return stations;
}

// constructor --------------------------------------------------------------
MainForm::MainForm(QWidget *parent)
    : QDialog(parent), ui(new Ui::MainForm)
{
    ui->setupUi(this);
    mainForm = this;

    setWindowIcon(QIcon(":/icons/rktpost_Icon.ico"));
    setWindowTitle(QString("%1 ver.%2 %3").arg(PRGNAME, VER_RTKLIB, PATCH_LEVEL));
    setlocale(LC_NUMERIC, "C");

    QString file = QApplication::applicationFilePath();
    QFileInfo fi(file);
    iniFile = fi.absoluteDir().filePath(fi.baseName()) + ".ini";

    ui->pBProgress->setVisible(false);
    setAcceptDrops(true);

    optDialog = new OptDialog(this, OptDialog::PostOptions);
    convDialog = new ConvDialog(this);
    textViewer = new TextViewer(this);

    // setup completers
    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->cBInputFile1->setCompleter(fileCompleter);
    ui->cBInputFile2->setCompleter(fileCompleter);
    ui->cBInputFile3->setCompleter(fileCompleter);
    ui->cBInputFile4->setCompleter(fileCompleter);
    ui->cBInputFile5->setCompleter(fileCompleter);
    ui->cBInputFile6->setCompleter(fileCompleter);
    ui->cBOutputFile->setCompleter(fileCompleter);

    QCompleter *dirCompleter = new QCompleter(this);
    QFileSystemModel *dirModel = new QFileSystemModel(dirCompleter);
    dirModel->setRootPath("");
    dirModel->setFilter(QDir::AllDirs|QDir::Drives|QDir::NoDotAndDotDot);
    dirCompleter->setModel(dirModel);
    ui->lEOutputDirectory->setCompleter(dirCompleter);

    // connect signals
    connect(ui->btnPlot, &QPushButton::clicked, this, &MainForm::callRtkPlot);
    connect(ui->btnView, &QPushButton::clicked, this, &MainForm::viewOutputFile);
    connect(ui->btnToKML, &QPushButton::clicked, this, &MainForm::convertToKML);
    connect(ui->btnOption, &QPushButton::clicked, this, &MainForm::showOptionsDialog);
    connect(ui->btnExec, &QPushButton::clicked, this, &MainForm::startPostProcessing);
    connect(ui->btnAbort, &QPushButton::clicked, this, &MainForm::abortProcessing);
    connect(ui->btnExit, &QPushButton::clicked, this, &MainForm::close);
    connect(ui->btnAbout, &QPushButton::clicked, this, &MainForm::showAboutDialog);
    connect(ui->btnTimeStart, &QPushButton::clicked, this, &MainForm::showStartTimeDialog);
    connect(ui->btnTimeStop, &QPushButton::clicked, this, &MainForm::showStopTimeDialog);
    connect(ui->btnInputFile1, &QPushButton::clicked, this, &MainForm::selectInputFile1);
    connect(ui->btnInputFile2, &QPushButton::clicked, this, &MainForm::selectInputFile2);
    connect(ui->btnInputFile3, &QPushButton::clicked, this, &MainForm::selectInputFile3);
    connect(ui->btnInputFile4, &QPushButton::clicked, this, &MainForm::selectInputFile4);
    connect(ui->btnInputFile5, &QPushButton::clicked, this, &MainForm::selectInputFile5);
    connect(ui->btnInputFile6, &QPushButton::clicked, this, &MainForm::selectInputFile6);
    connect(ui->btnOutputFile, &QPushButton::clicked, this, &MainForm::selectOutputFile);
    connect(ui->btnInputView1, &QPushButton::clicked, this, &MainForm::viewInputFile1);
    connect(ui->btnInputView2, &QPushButton::clicked, this, &MainForm::viewInputFile2);
    connect(ui->btnInputView3, &QPushButton::clicked, this, &MainForm::viewInputFile3);
    connect(ui->btnInputView4, &QPushButton::clicked, this, &MainForm::viewInputFile4);
    connect(ui->btnInputView5, &QPushButton::clicked, this, &MainForm::viewInputFile5);
    connect(ui->btnInputView6, &QPushButton::clicked, this, &MainForm::viewInputFile6);
    connect(ui->btnOutputViewStat, &QPushButton::clicked, this, &MainForm::viewOutputFileStat);
    connect(ui->btnOutputViewTrace, &QPushButton::clicked, this, &MainForm::viewOutputFileTrace);
    connect(ui->btnInputPlot1, &QPushButton::clicked, this, &MainForm::plotInputFile1);
    connect(ui->btnInputPlot2, &QPushButton::clicked, this, &MainForm::plotInputFile2);
    connect(ui->btnKeyword, &QPushButton::clicked, this, &MainForm::showKeyDialog);
    connect(ui->cBTimeStart, &QCheckBox::clicked, this, &MainForm::updateEnable);
    connect(ui->cBTimeEnd, &QCheckBox::clicked, this, &MainForm::updateEnable);
    connect(ui->cBTimeIntervalF, &QCheckBox::clicked, this, &MainForm::updateEnable);
    connect(ui->cBTimeUnitF, &QCheckBox::clicked, this, &MainForm::updateEnable);
    connect(ui->cBInputFile1, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::setOutputFile);
    connect(ui->cBOutputDirectoryEnable, &QCheckBox::clicked, this, &MainForm::outputDirectoryEnableClicked);
    connect(ui->lEOutputDirectory, &QLineEdit::textChanged, this, &MainForm::setOutputFile);
    connect(ui->btnOutputDirectory, &QPushButton::clicked, this, &MainForm::selectOutputDirectory);

    ui->btnAbort->setVisible(false);
}
// callback on form show ----------------------------------------------------
void MainForm::showEvent(QShowEvent* event)
{
    if (event->spontaneous()) return;

    QComboBox *ifile[] = {ui->cBInputFile3, ui->cBInputFile4, ui->cBInputFile5, ui->cBInputFile6};
    int inputflag = 0;

    QCommandLineParser parser;
    parser.setApplicationDescription("RTK Post-Processing Tool Qt");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);

    QCommandLineOption iniFileOption(QStringList() << "i",
            QCoreApplication::translate("main", "use init file <file>"),
            QCoreApplication::translate("main", "ini file"));
    parser.addOption(iniFileOption);

    QCommandLineOption titleOption(QStringList() << "t",
            QCoreApplication::translate("main", "use window tile <title>"),
            QCoreApplication::translate("main", "title"));
    parser.addOption(titleOption);

    QCommandLineOption roverOption(QStringList() << "r",
            QCoreApplication::translate("main", "rinex obs rover <file>"),
            QCoreApplication::translate("main", "file"));
    parser.addOption(roverOption);

    QCommandLineOption baseStationOption(QStringList() << "b",
            QCoreApplication::translate("main", "rinex obs base station <path>"),
            QCoreApplication::translate("main", "file"));
    parser.addOption(baseStationOption);

    QCommandLineOption navFileOption(QStringList() << "n" << "file",
            QCoreApplication::translate("main", "rinex nav/clk, sp3, ionex or sp3 <file>"),
            QCoreApplication::translate("main", "file"));
    parser.addOption(navFileOption);

    QCommandLineOption outputOption(QStringList() << "o",
            QCoreApplication::translate("main", "output file <file>"),
            QCoreApplication::translate("main", "file"));
    parser.addOption(outputOption);

    QCommandLineOption outputDirOption(QStringList() << "d",
            QCoreApplication::translate("main", "output directory <dir>"),
            QCoreApplication::translate("main", "dir"));
    parser.addOption(outputDirOption);

    QCommandLineOption timeStartOption(QStringList() << "ts",
            QCoreApplication::translate("main", "time start"),
            QCoreApplication::translate("main", "yyyy/mm/dd hh:mm:ss"));
    parser.addOption(timeStartOption);

    QCommandLineOption timeEndOption(QStringList() << "te",
            QCoreApplication::translate("main", "time end"),
            QCoreApplication::translate("main", "yyyy/mm/dd hh:mm:ss"));
    parser.addOption(timeEndOption);

    QCommandLineOption timeIntervalOption(QStringList() << "ti",
            QCoreApplication::translate("main", "time interval (s)"),
            QCoreApplication::translate("main", "time"));
    parser.addOption(timeIntervalOption);

    QCommandLineOption timeUnitOption(QStringList() << "tu",
            QCoreApplication::translate("main", "time unit (hr)"),
            QCoreApplication::translate("main", "unit"));
    parser.addOption(timeUnitOption);

    parser.process(*QApplication::instance());

    if (parser.isSet(iniFileOption)) {
        iniFile = parser.value(iniFileOption);
    }

    loadOptions();

    if (parser.isSet(titleOption)) {
        setWindowTitle(parser.value(titleOption));
    }
    if (parser.isSet(roverOption)) {
        ui->cBInputFile1->setCurrentText(parser.value(roverOption));
        inputflag = 1;
    };
    if (parser.isSet(baseStationOption)) {
        ui->cBInputFile2->setCurrentText(parser.value(baseStationOption));
    }
    if (parser.isSet(navFileOption)) {
        QStringList files = parser.values(navFileOption);
        for (int n = 0; n < files.size() && n < 4; n++)
            ifile[n]->setCurrentText(files.at(n));
    }
    if (parser.isSet(outputOption)) {
        ui->cBOutputFile->setCurrentText(parser.value(outputOption));
    }
    if (parser.isSet(outputDirOption)) {
        ui->cBOutputDirectoryEnable->setChecked(true);
        ui->lEOutputDirectory->setText(parser.value(outputDirOption));
    }
    if (parser.isSet(timeStartOption)) {
        ui->cBTimeStart->setChecked(true);
        ui->dtDateTimeStart->setDateTime(QDateTime::fromString(parser.value(timeStartOption), "yyyy/MM/dd hh:mm:ss"));
    }
    if (parser.isSet(timeEndOption)) {
        ui->cBTimeEnd->setChecked(true);
        ui->dtDateTimeStop->setDateTime(QDateTime::fromString(parser.value(timeEndOption), "yyyy/MM/dd hh:mm:ss"));
    }
    if (parser.isSet(timeIntervalOption)) {
        ui->cBTimeIntervalF->setChecked(true);
        ui->cBTimeInterval->setCurrentText(QString("%1 s").arg(parser.value(timeIntervalOption)));
    }
    if (parser.isSet(timeUnitOption)) {
        ui->cBTimeUnitF->setChecked(true);
        ui->sBTimeUnit->setValue(parser.value(timeUnitOption).toDouble());
    }

    if (inputflag) setOutputFile();

    updateEnable();
}
// callback on form close ---------------------------------------------------
void MainForm::closeEvent(QCloseEvent *)
{
    saveOptions();
}
// callback on drop files ---------------------------------------------------
void  MainForm::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasFormat("text/uri-list"))
        event->acceptProposedAction();
}
// callback on drop files ---------------------------------------------------
void  MainForm::dropEvent(QDropEvent *event)
{
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
    QPointF point = event->position();
#else
    QPointF point = event->pos();
#endif
    int top;

    if (!event->mimeData()->hasFormat("text/uri-list")) return;

    QString file = QDir::toNativeSeparators(event->mimeData()->text());

    top = ui->panelMain->pos().y() + ui->panelFiles->pos().y();
    if (point.y() <= top + ui->cBInputFile1->pos().y() + ui->cBInputFile1->height()) {
        ui->cBInputFile1->setCurrentText(file);
        setOutputFile();
    }
    else if (point.y() <= top + ui->cBInputFile2->pos().y() + ui->cBInputFile2->height()) {
        ui->cBInputFile2->setCurrentText(file);
    }
    else if (point.y() <= top + ui->cBInputFile3->pos().y() + ui->cBInputFile3->height()) {
        ui->cBInputFile3->setCurrentText(file);
    }
    else if (point.y() <= top + ui->cBInputFile4->pos().y() + ui->cBInputFile4->height()) {
        ui->cBInputFile4->setCurrentText(file);
    }
    else if (point.y() <= top + ui->cBInputFile5->pos().y() + ui->cBInputFile5->height()) {
        ui->cBInputFile5->setCurrentText(file);
    }
    else if (point.y() <= top + ui->cBInputFile6->pos().y() + ui->cBInputFile6->height()) {
        ui->cBInputFile6->setCurrentText(file);
    }
}
// callback on button-plot --------------------------------------------------
void MainForm::callRtkPlot()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QString file = filePath(ui->cBOutputFile->currentText());
    QStringList cmds = {"rtkplot_qt", "../rtkplot_qt/rtkplot_qt", "../../../bin/rtkplot_qt"};
    QStringList opts;

    opts += file;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }

    showMessage("Error: rtkplot_qt execution failed");
}
// --------------------------------------------------------------------------
void MainForm::setProgress(int value)
{
    ui->pBProgress->setValue(value);
}
// callback on button-view --------------------------------------------------
void MainForm::viewOutputFile()
{
    viewFile(filePath(ui->cBOutputFile->currentText()));
}
// callback on button-to-kml ------------------------------------------------
void MainForm::convertToKML()
{
    convDialog->setInput(filePath(ui->cBOutputFile->currentText()));
    convDialog->exec();
}
// callback on button-options -----------------------------------------------
void MainForm::showOptionsDialog()
{
    int format_old = optDialog->solutionOptions.posf;

    optDialog->exec();
    if (optDialog->result() != QDialog::Accepted) return;

    if ((format_old == SOLF_NMEA) != (optDialog->solutionOptions.posf == SOLF_NMEA)) {
        setOutputFile();
    }
    updateEnable();
}
// callback on button-execute -----------------------------------------------
void MainForm::startPostProcessing()
{
    QString outputFile = ui->cBOutputFile->currentText();
    static QRegularExpression reg = QRegularExpression(R"(\.\d\d[ondg])", QRegularExpression::CaseInsensitiveOption);
    abortFlag = false;

    if (ui->cBInputFile1->currentText().isEmpty()) {
        showMessage(tr("Error: No rinex observation file (rover)"));
        return;
    }
    if (ui->cBInputFile2->currentText().isEmpty() && PMODE_DGPS <=  optDialog->processingOptions.mode &&  optDialog->processingOptions.mode <= PMODE_FIXED) {
        showMessage(tr("Error: No rinex observation file (base station)"));
        return;
    }
    if (ui->cBOutputFile->currentText().isEmpty()) {
        showMessage(tr("Error: No output file"));
        return;
    }
    QFileInfo f = QFileInfo(outputFile);
    if (f.suffix().contains(".obs", Qt::CaseInsensitive) ||
        f.suffix().contains(".rnx", Qt::CaseInsensitive) ||
        f.suffix().contains(".nav", Qt::CaseInsensitive) ||
        f.suffix().contains(".gnav", Qt::CaseInsensitive) ||
        f.suffix().contains(".gz", Qt::CaseInsensitive) ||
        f.suffix().contains(".Z", Qt::CaseInsensitive) ||
        f.suffix().contains(reg)) {
        showMessage(tr("Error: Invalid extension of output file (%1)").arg(outputFile));
            return;
    }
    showMessage("");

    ui->btnAbort ->setVisible(true);
    ui->btnExec->setVisible(false);
    ui->btnExit->setEnabled(false);
    ui->btnView->setEnabled(false);
    ui->btnToKML->setEnabled(false);
    ui->btnPlot->setEnabled(false);
    ui->btnOption->setEnabled(false);
    ui->panelMain->setEnabled(false);

    execProcessing();
}
// callback on processing finished-------------------------------------------
void MainForm::processingFinished(int stat)
{
    setCursor(Qt::ArrowCursor);
    ui->pBProgress->setVisible(false);

    if (stat >= 0) {
        addHistory(ui->cBInputFile1);
        addHistory(ui->cBInputFile2);
        addHistory(ui->cBInputFile3);
        addHistory(ui->cBInputFile4);
        addHistory(ui->cBInputFile5);
        addHistory(ui->cBInputFile6);
        addHistory(ui->cBOutputFile);
    }

    if (ui->lblMessage->text().contains("processing")) {
        showMessage(tr("Done"));
    }
    ui->btnAbort ->setVisible(false);
    ui->btnExec->setVisible(true);
    ui->btnExec->setEnabled(true);
    ui->btnExit->setEnabled(true);
    ui->btnView->setEnabled(true);
    ui->btnToKML->setEnabled(true);
    ui->btnPlot->setEnabled(true);
    ui->btnOption->setEnabled(true);
    ui->panelMain->setEnabled(true);
}
// callback on button-abort -------------------------------------------------
void MainForm::abortProcessing()
{
    abortFlag = true;
    showMessage(tr("Aborted"));
}
// callback on button-about -------------------------------------------------
void MainForm::showAboutDialog()
{
    AboutDialog aboutDialog(this, QPixmap(":/icons/rtkpost"), PRGNAME);

    aboutDialog.exec();
}
// callback on button-time-1 ------------------------------------------------
void MainForm::showStartTimeDialog()
{
    TimeDialog timeDialog(this);
    timeDialog.setTime(getTimeStart());
    timeDialog.exec();
}
// callback on button-time-2 ------------------------------------------------
void MainForm::showStopTimeDialog()
{
    TimeDialog timeDialog(this);

    timeDialog.setTime(getTimeStop());
    timeDialog.exec();
}
// callback on button-inputfile-1 -------------------------------------------
void MainForm::selectInputFile1()
{
    ui->cBInputFile1->setCurrentText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("RINEX OBS (Rover) File"), ui->cBInputFile1->currentText(),
                                                                                           tr("All (*.*);;RINEX OBS (*.rnx *.obs *.*O *.*D)"))));
    setOutputFile();
}
// callback on button-inputfile-2 ------------------------------------------
void MainForm::selectInputFile2()
{
    ui->cBInputFile2->setCurrentText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("RINEX OBS (Base Station) File"), ui->cBInputFile2->currentText(),
                                                                                           tr("All (*.*);;RINEX OBS (*.rnx *.obs *.*O *.*D)"))));
}
// callback on button-inputfile-3 -------------------------------------------
void MainForm::selectInputFile3()
{
    ui->cBInputFile3->setCurrentText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("RINEX NAV/CLK,SP3,Bias-SINEX,IONEX or SBAS/EMS File"), ui->cBInputFile3->currentText(),
                                                                                           tr("All (*.*);;RINEX NAV (*.rnx *.*nav *.*N *.*P *.*G *.*H *.*Q)"))));
}
// callback on button-inputfile-4 -------------------------------------------
void MainForm::selectInputFile4()
{
    ui->cBInputFile4->setCurrentText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("RINEX NAV/CLK,SP3,Bias-SINEX,IONEX or SBAS/EMS File"), ui->cBInputFile4->currentText(),
                                                                                           tr("All (*.*);;Precise Ephemeris/Clock/Biases (*.SP3 *.sp3 *.eph* *.CLK *.clk* *.BIA)"))));
}
// callback on button-inputfile-5 -------------------------------------------
void MainForm::selectInputFile5()
{
    ui->cBInputFile5->setCurrentText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("RINEX NAV/CLK,SP3,Bias-SINEX,IONEX or SBAS/EMS File"), ui->cBInputFile5->currentText(),
                                                                                           tr("All (*.*);;Precise Ephemeris/Clock/Biases (*.SP3 *.sp3 *.eph* *.CLK *.clk* *.BIA)"))));
}
// callback on button-inputfile-6 -------------------------------------------
void MainForm::selectInputFile6()
{
    ui->cBInputFile6->setCurrentText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("RINEX NAV/CLK,SP3,Bias-SINEX,IONEX or SBAS/EMS File"), ui->cBInputFile6->currentText(),
                                                                                           tr("All (*.*);;Bias-SINEX (*.BIA *.BSX),IONEX (*.*i *.ionex, *.inx),SBAS (*.sbs *.ems)"))));
}
// callback on button-outputfile --------------------------------------------
void MainForm::selectOutputFile()
{
    ui->cBOutputFile->setCurrentText(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Output File"), ui->cBOutputFile->currentText(),
                                                                                           tr("All (*.*);;Position Files (*.pos)"))));
}
// callback on button-inputview-1 -------------------------------------------
void MainForm::viewInputFile1()
{
    viewFile(filePath(ui->cBInputFile1->currentText()));
}
// callback on button-inputview-2 -------------------------------------------
void MainForm::viewInputFile2()
{
    viewFile(filePath(ui->cBInputFile2->currentText()));
}
// callback on button-inputview-3 -------------------------------------------
void MainForm::viewInputFile3()
{
    QString filename = filePath(ui->cBInputFile3->currentText());
    QString nav_filename;

    if (filename.isEmpty()) {
        const QString &obs_filename = filePath(ui->cBInputFile1->currentText());
        if (!obsToNav(obs_filename, nav_filename)) return;
        filename = nav_filename;
    }
    viewFile(filename);
}
// callback on button-inputview-4 -------------------------------------------
void MainForm::viewInputFile4()
{
    viewFile(filePath(ui->cBInputFile4->currentText()));
}
// callback on button-inputview-5 -------------------------------------------
void MainForm::viewInputFile5()
{
    viewFile(filePath(ui->cBInputFile5->currentText()));
}
// callback on button-inputview-6 -------------------------------------------
void MainForm::viewInputFile6()
{
    viewFile(filePath(ui->cBInputFile6->currentText()));
}
// callback on button-outputview-1 ------------------------------------------
void MainForm::viewOutputFileStat()
{
    QString file = filePath(ui->cBOutputFile->currentText()) + ".stat";
    if (!QFile::exists(file)) return;

    viewFile(file);
}
// callback on button-outputview-2 ------------------------------------------
void MainForm::viewOutputFileTrace()
{
    QString file=filePath(ui->cBOutputFile->currentText())+".trace";
    if (!QFile::exists(file)) return;

    viewFile(file);
}
// callback on button-inputplot-1 -------------------------------------------
void MainForm::plotInputFile1()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QString files[5];
    QStringList cmds = {"rtkplot_qt", "../rtkplot_qt/rtkplot_qt", "../../../bin/rtkplot_qt"};
    QStringList opts;
    QString navfile;

    files[0] = filePath(ui->cBInputFile1->currentText()); /* obs rover */
    files[1] = filePath(ui->cBInputFile3->currentText());
    files[2] = filePath(ui->cBInputFile4->currentText());
    files[3] = filePath(ui->cBInputFile5->currentText());
    files[4] = filePath(ui->cBInputFile6->currentText());

    if ((files[1].isEmpty()) && (obsToNav(files[0], navfile)))
        files[1] = navfile;

    opts << "-r" << files[0] << files[1] << files[2] << files[3] << files[4];

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }

    showMessage(tr("Error: rtkplot_qt execution failed"));
}
// callback on button-inputplot-2 -------------------------------------------
void MainForm::plotInputFile2()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QString files[5];
    QStringList cmds = {"rtkplot_qt", "../rtkplot_qt/rtkplot_qt", "../../../bin/rtkplot_qt"};
    QStringList opts;
    QString navfile;

    files[0] = filePath(ui->cBInputFile2->currentText()); /* obs base */
    files[1] = filePath(ui->cBInputFile3->currentText());
    files[2] = filePath(ui->cBInputFile4->currentText());
    files[3] = filePath(ui->cBInputFile5->currentText());
    files[4] = filePath(ui->cBInputFile6->currentText());

    if ((files[1].isEmpty()) && (obsToNav(files[0], navfile)))
        files[1] = navfile;

    opts << "-r" << files[0] << files[1] << files[2] << files[3] << files[4];

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }

    showMessage(tr("Error: rtkplot_qt execution failed"));
}
// callback on button-output-directory --------------------------------------
void MainForm::selectOutputDirectory()
{
    ui->lEOutputDirectory->setText(QDir::toNativeSeparators(QFileDialog::getExistingDirectory(this, tr("Output Directory"), ui->lEOutputDirectory->text())));
}
// callback on button keyword -----------------------------------------------
void MainForm::showKeyDialog()
{
    KeyDialog keyDialog(this);

    keyDialog.setFlag(2);
    keyDialog.exec();
}
// callback on output-directory checked -------------------------------------
void MainForm::outputDirectoryEnableClicked()
{
    updateEnable();
    setOutputFile();
}
// set output file path -----------------------------------------------------
void MainForm::setOutputFile()
{
    QString ofile, ifile;

    if (ui->cBInputFile1->currentText().isEmpty()) return;

    ifile = ui->cBInputFile1->currentText();
    if (ui->cBOutputDirectoryEnable->isChecked()) {
        QFileInfo f(ifile);
        ofile = QDir(ui->lEOutputDirectory->text()).filePath(f.baseName());
    } else {
        QFileInfo f(ifile);
        ofile = f.absoluteDir().filePath(f.baseName());
    }
    ofile += optDialog->solutionOptions.posf == SOLF_NMEA ? ".nmea" : ".pos";
    ofile.replace('*', '0');
    ui->cBOutputFile->setCurrentText(QDir::toNativeSeparators(ofile));
}
// execute post-processing --------------------------------------------------
void MainForm::execProcessing()
{
    QString inputFile1 = ui->cBInputFile1->currentText();
    QString inputFile2 = ui->cBInputFile2->currentText();
    QString inputFile3 = ui->cBInputFile3->currentText();
    QString inputFile4 = ui->cBInputFile4->currentText();
    QString inputFile5 = ui->cBInputFile5->currentText();
    QString inputFile6 = ui->cBInputFile6->currentText();
    QString outputFile = ui->cBOutputFile->currentText();
    QString temp;

    processingThread = new ProcessingThread(this);

    // get processing options
    if (ui->cBTimeStart->isChecked())
        processingThread->ts = getTimeStart();
    if (ui->cBTimeEnd->isChecked())
        processingThread->te = getTimeStop();
    if (ui->cBTimeIntervalF ->isChecked())
        processingThread->ti = ui->cBTimeInterval->currentText().split(" ").first().toDouble();
    if (ui->cBTimeUnitF->isChecked())
        processingThread->tu = ui->sBTimeUnit->value() * 3600.0;

    processingThread->prcopt = prcopt_default;
    if (!getOption(processingThread->prcopt, processingThread->solopt, processingThread->filopt)) {
        processingFinished(0);

        delete processingThread;
        return;
    }

    // set input/output files
    if (!processingThread->addInput(inputFile1)) {
        processingFinished(0);
        delete processingThread;
        return;
    };

    if (PMODE_DGPS <= processingThread->prcopt.mode && processingThread->prcopt.mode <= PMODE_FIXED) {
        if (!processingThread->addInput(inputFile2)) {
            processingFinished(0);
            delete processingThread;
            return;
        };
    }

    if (!inputFile3.isEmpty()) {
        if (!processingThread->addInput(inputFile3)) {
            processingFinished(0);
            delete processingThread;
            return;
        };
    } else if (!obsToNav(inputFile1, temp)) {
        showMessage(tr("Error: No navigation data"));
        processingFinished(0);
        delete processingThread;
        return;
    } else {
        if (!processingThread->addInput(temp)) {
            processingFinished(0);
            delete processingThread;
            return;
        }
    }

    if (!inputFile4.isEmpty())
        if (!processingThread->addInput(inputFile4)) {
            processingFinished(0);
            delete processingThread;
            return;
        };

    if (!inputFile5.isEmpty())
        if (!processingThread->addInput(inputFile5)) {
            processingFinished(0);
            delete processingThread;
            return;
        };

    if (!inputFile6.isEmpty())
        if (!processingThread->addInput(inputFile6)) {
            processingFinished(0);
            delete processingThread;
            return;
        };

    strncpy(processingThread->outfile, qPrintable(outputFile), 1023);

    // confirm overwrite
    if (!ui->cBTimeStart->isChecked() || !ui->cBTimeEnd->isChecked()) {
        if (QFileInfo::exists(processingThread->outfile)) {
            if (QMessageBox::question(this, tr("Overwrite"), tr("Overwrite existing file %1.").arg(processingThread->outfile)) != QMessageBox::Yes) {
                processingFinished(0);
                delete processingThread;
                return;
            }
        }
    }

    // set rover and base station list
    processingThread->rov = processingThread->toList(optDialog->roverList);
    processingThread->base = processingThread->toList(optDialog->baseList);

    ui->pBProgress->setValue(0);
    ui->pBProgress->setVisible(true);
    showMessage(tr("Reading..."));

    setCursor(Qt::WaitCursor);

    connect(processingThread, &ProcessingThread::done, this, &MainForm::processingFinished);

    // post processing positioning
    processingThread->start();
}
// get processing and solution options --------------------------------------
bool MainForm::getOption(prcopt_t &prcopt, solopt_t &solopt, filopt_t &filopt)
{
    prcopt = optDialog->processingOptions;
    solopt = optDialog->solutionOptions;
    filopt = optDialog->fileOptions;

    strncpy(solopt.prog, qPrintable(QString("%1 Version %2 %3").arg(PRGNAME, VER_RTKLIB, PATCH_LEVEL)), 63);

    // file options

    return true;
}
// observation file to nav file ---------------------------------------------
bool MainForm::obsToNav(const QString &obsfile, QString &navfile)
{
    int p;
    QFileInfo f(obsfile);
    navfile = f.canonicalPath() + QDir::separator() + f.completeBaseName();
    QString suffix = f.suffix();

    if (suffix.isEmpty()) return false;

    if ((suffix.length() == 3) && (suffix.at(2).toLower() == 'o'))
        suffix[2] = '*';
    else if ((suffix.length() == 3) && (suffix.at(2).toLower() == 'd'))
        suffix[2] = '*';
    else if (suffix == "obs")
        suffix = "*nav";
    else if (suffix == "NAV")
        suffix = "*NAV";
    else if (((p = suffix.indexOf("gz")) != -1) || (( p = suffix.indexOf('Z')) != -1)) {
        if (p < 1) return false;

        if (suffix.at(p - 1).toLower() == 'o')
            suffix[p - 1] = '*';
        else if (suffix.at(p - 1).toLower() == 'd')
            suffix[p - 1] = '*';
        else return false;
    } else
        return false;
    navfile += "." + suffix;
    return true;
}
// replace file path with keywords ------------------------------------------
QString MainForm::filePath(const QString &file)
{
    gtime_t ts = {0, 0};
    int p;
    QString rover, base;
    char path[1024];

    if (ui->cBTimeStart->isChecked()) ts = getTimeStart();

    // rover
    p = 0;
    while ((p = optDialog->roverList.indexOf("\n", p)) != -1){
        if ((p < optDialog->roverList.length()) && (optDialog->roverList.at(p) != '#')) break;
    }
    if (p != -1)
        rover = optDialog->roverList.mid(p);
    else
        rover = optDialog->roverList;

    // base station
    p = 0;
    while ((p = optDialog->baseList.indexOf("\n", p)) != -1){
        if ((p < optDialog->baseList.length()) && (optDialog->baseList.at(p) != '#')) break;
    }
    if (p != -1)
        base = optDialog->baseList.mid(p);
    else
        base = optDialog->baseList;

    reppath(qPrintable(file), path, ts, qPrintable(rover), qPrintable(base));

    return QString(path);
}
// read history -------------------------------------------------------------
void MainForm::readList(QComboBox* combo, QSettings *ini, const QString &key)
{
    QString item;
    int i;

    for (i = 0; i < 100; i++) {
        item = ini->value(QString("%1_%2").arg(key).arg(i, 3), "").toString();
        if ((!item.isEmpty()) && (combo->findText(item) == -1)) combo->addItem(item);
    }
}
// write history ------------------------------------------------------------
void MainForm::writeList(QSettings *ini, const QString &key, const QComboBox *combo)
{
    for (int i = 0;i < combo->count(); i++) {
        ini->setValue(QString("%1_%2").arg(key).arg(i, 3), combo->itemText(i));
    }
}
// add history --------------------------------------------------------------
void MainForm::addHistory(QComboBox *combo)
{
    QString hist = combo->currentText();
    if (hist.isEmpty()) return;

    // move item to first position
    int idx = combo->currentIndex();
    if (idx >= 0) combo->removeItem(idx);
    combo->insertItem(0, hist);

    // limit history size
    for (int i = combo->count() - 1; i >= MAXHIST; i--) combo->removeItem(i);
    combo->setCurrentIndex(0);
}
// execute command ----------------------------------------------------------
bool MainForm::execCommand(const QString &cmd, const QStringList &opt)
{
    return QProcess::startDetached(cmd, opt);  /* FIXME: show option not yet supported */
}
// view file ----------------------------------------------------------------
void MainForm::viewFile(const QString &file)
{
    QString f;
    char tmpfile[1024];
    int cstat;

    if (file.isEmpty()) return;
    cstat = rtk_uncompress(qPrintable(file), tmpfile);
    f = !cstat ? file : tmpfile;

    textViewer->setWindowTitle(file);
    textViewer->show();
    if (!textViewer->read(f)) textViewer->close();
    if (cstat == 1) remove(tmpfile);
}
// show message in message area ---------------------------------------------
void MainForm::showMessage(const QString &msg)
{
    ui->lblMessage->setText(msg);
}
// get time from time-1 -----------------------------------------------------
gtime_t MainForm::getTimeStart()
{
    QDateTime time(ui->dtDateTimeStart->dateTime());

    gtime_t t;
    t.time = time.toSecsSinceEpoch();
    t.sec = time.time().msec() / 1000;

    return t;
}
// get time from time-2 -----------------------------------------------------
gtime_t MainForm::getTimeStop()
{
    QDateTime time(ui->dtDateTimeStop->dateTime());

    gtime_t t;
    t.time = time.toSecsSinceEpoch();
    t.sec = time.time().msec() / 1000;

    return t;
}
// set time to time-1 -------------------------------------------------------
void MainForm::setTimeStart(gtime_t time)
{
    QDateTime t = QDateTime::fromSecsSinceEpoch(time.time);
    t = t.addMSecs(time.sec * 1000);
    ui->dtDateTimeStart->setDateTime(t);
}
// set time to time-2 -------------------------------------------------------
void MainForm::setTimeStop(gtime_t time)
{
    QDateTime t = QDateTime::fromSecsSinceEpoch(time.time);
    t = t.addMSecs(time.sec * 1000);
    ui->dtDateTimeStop->setDateTime(t);
}
// update enable/disable of widgets -----------------------------------------
void MainForm::updateEnable()
{
    bool moder = PMODE_DGPS <= optDialog->processingOptions.mode && optDialog->processingOptions.mode <= PMODE_FIXED;

    ui->lblInputFile1->setText(moder ? tr("RINEX OBS: Rover") : tr("RINEX OBS"));
    ui->cBInputFile2->setEnabled(moder);
    ui->btnInputFile2->setEnabled(moder);
    ui->btnInputPlot2->setEnabled(moder);
    ui->btnInputView2->setEnabled(moder);
    ui->btnOutputViewStat->setEnabled(optDialog->solutionOptions.sstat > 0);
    ui->btnOutputViewTrace->setEnabled(optDialog->solutionOptions.trace > 0);
    ui->lblInputFile2->setEnabled(moder);
    ui->dtDateTimeStart->setEnabled(ui->cBTimeStart->isChecked());
    ui->btnTimeStart->setEnabled(ui->cBTimeStart->isChecked());
    ui->dtDateTimeStop->setEnabled(ui->cBTimeEnd->isChecked());
    ui->btnTimeStop->setEnabled(ui->cBTimeEnd->isChecked());
    ui->cBTimeInterval->setEnabled(ui->cBTimeIntervalF->isChecked());
    ui->cBTimeUnitF->setEnabled(ui->cBTimeStart->isChecked() && ui->cBTimeEnd->isChecked());
    ui->sBTimeUnit->setEnabled(ui->cBTimeUnitF->isEnabled() && ui->cBTimeUnitF->isChecked());
    ui->lEOutputDirectory->setEnabled(ui->cBOutputDirectoryEnable->isChecked());
    ui->btnOutputDirectory->setEnabled(ui->cBOutputDirectoryEnable->isChecked());
}
// load options from ini file -----------------------------------------------
void MainForm::loadOptions()
{
    QSettings ini(iniFile, QSettings::IniFormat);

    ui->cBTimeStart->setChecked(ini.value("set/timestart", 0).toBool());
    ui->cBTimeEnd->setChecked(ini.value("set/timeend", 0).toBool());
    ui->dtDateTimeStart->setDate(ini.value ("set/timey1", QDate(2000, 01, 01)).toDate());
    ui->dtDateTimeStart->setTime(ini.value ("set/timeh1", QTime(0, 0, 0)).toTime());
    ui->dtDateTimeStop->setDate(ini.value ("set/timey2", QDate(2000, 01, 01)).toDate());
    ui->dtDateTimeStop->setTime(ini.value ("set/timeh2",  QTime(0, 0, 0)).toTime());
    ui->cBTimeIntervalF ->setChecked(ini.value("set/timeintf", 0).toBool());
    ui->cBTimeInterval->setCurrentText(QString("%1 s").arg(ini.value ("set/timeint", "0").toString()));
    ui->cBTimeUnitF->setChecked(ini.value("set/timeunitf", 0).toBool());
    ui->sBTimeUnit->setValue(ini.value("set/timeunit", "24").toDouble());
    ui->cBInputFile1->setCurrentText(ini.value ("set/inputfile1", "").toString());
    ui->cBInputFile2->setCurrentText(ini.value ("set/inputfile2", "").toString());
    ui->cBInputFile3->setCurrentText(ini.value ("set/inputfile3", "").toString());
    ui->cBInputFile4->setCurrentText(ini.value ("set/inputfile4", "").toString());
    ui->cBInputFile5->setCurrentText(ini.value ("set/inputfile5", "").toString());
    ui->cBInputFile6->setCurrentText(ini.value ("set/inputfile6", "").toString());
    ui->cBOutputDirectoryEnable->setChecked(ini.value("set/outputdirena", 0).toBool());
    ui->lEOutputDirectory->setText(ini.value ("set/outputdir", "").toString());
    ui->cBOutputFile->setCurrentText(ini.value ("set/outputfile", "").toString());

    readList(ui->cBInputFile1, &ini,"hist/inputfile1");
    readList(ui->cBInputFile2, &ini,"hist/inputfile2");
    readList(ui->cBInputFile3, &ini,"hist/inputfile3");
    readList(ui->cBInputFile4, &ini,"hist/inputfile4");
    readList(ui->cBInputFile5, &ini,"hist/inputfile5");
    readList(ui->cBInputFile6, &ini,"hist/inputfile6");
    readList(ui->cBOutputFile, &ini,"hist/outputfile");

    optDialog->loadOptions(ini);
    textViewer->loadOptions(ini);
    convDialog->loadOptions(ini);

}
// save options to ini file -------------------------------------------------
void MainForm::saveOptions()
{
    QSettings ini(iniFile, QSettings::IniFormat);

    ini.setValue("set/timestart", ui->cBTimeStart->isChecked() ? 1 : 0);
    ini.setValue("set/timeend", ui->cBTimeEnd->isChecked()? 1 : 0);
    ini.setValue("set/timey1", ui->dtDateTimeStart->date());
    ini.setValue("set/timeh1", ui->dtDateTimeStart->time());
    ini.setValue("set/timey2", ui->dtDateTimeStop->date());
    ini.setValue("set/timeh2", ui->dtDateTimeStop->time());
    ini.setValue("set/timeintf", ui->cBTimeIntervalF->isChecked() ? 1 : 0);
    ini.setValue("set/timeint", ui->cBTimeInterval->currentText().split(" ").first());
    ini.setValue("set/timeunitf", ui->cBTimeUnitF->isChecked() ? 1 : 0);
    ini.setValue("set/timeunit", ui->sBTimeUnit->value());
    ini.setValue("set/inputfile1", ui->cBInputFile1->currentText());
    ini.setValue("set/inputfile2", ui->cBInputFile2->currentText());
    ini.setValue("set/inputfile3", ui->cBInputFile3->currentText());
    ini.setValue("set/inputfile4", ui->cBInputFile4->currentText());
    ini.setValue("set/inputfile5", ui->cBInputFile5->currentText());
    ini.setValue("set/inputfile6", ui->cBInputFile6->currentText());
    ini.setValue("set/outputdirena", ui->cBOutputDirectoryEnable->isChecked());
    ini.setValue("set/outputdir", ui->lEOutputDirectory->text());
    ini.setValue("set/outputfile", ui->cBOutputFile->currentText());

    writeList(&ini,"hist/inputfile1", ui->cBInputFile1);
    writeList(&ini,"hist/inputfile2", ui->cBInputFile2);
    writeList(&ini,"hist/inputfile3", ui->cBInputFile3);
    writeList(&ini,"hist/inputfile4", ui->cBInputFile4);
    writeList(&ini,"hist/inputfile5", ui->cBInputFile5);
    writeList(&ini,"hist/inputfile6", ui->cBInputFile6);
    writeList(&ini,"hist/outputfile", ui->cBOutputFile);
    
    optDialog->saveOptions(ini);
    textViewer->saveOptions(ini);
    convDialog->saveOptions(ini);
}
//---------------------------------------------------------------------------

