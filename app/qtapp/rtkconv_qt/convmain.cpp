//---------------------------------------------------------------------------
// rtkconv : rinex converter
//
//          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann
//
// options : rtkconv [-t title][-i file]
//
//           -t title   window title
//           -i file    ini file path
//
// version : $Revision:$ $Date:$
// history : 2008/07/14  1.0 new
//			 2010/07/18  1.1 rtklib 2.4.0
//			 2011/06/10  1.2 rtklib 2.4.1
//			 2013/04/01  1.3 rtklib 2.4.2
//			 2020/11/30  1.4 support RINEX 3.04
//			                 support NavIC/IRNSS
//                           support SBF for auto format recognition
//                           support "Phase Shift" option
//                           no support "Scan Obs" option
//---------------------------------------------------------------------------
#include <clocale>

#include <QShowEvent>
#include <QTimer>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QDoubleValidator>
#include <QMessageBox>
#include <QProcess>
#include <QSettings>
#include <QMimeData>
#include <QDebug>
#include <QCompleter>
#include <QFileSystemModel>
#include <QAction>
#include <QToolTip>

#include "convmain.h"
#include "timedlg.h"
#include "aboutdlg.h"
#include "startdlg.h"
#include "keydlg.h"
#include "convopt.h"
#include "viewer.h"
#include "rtklib.h"

#include "ui_convmain.h"

//---------------------------------------------------------------------------

MainWindow *mainWindow;

#define PRGNAME     "RtkConv Qt"        // program name
#define MAXHIST     20                  // max number of histories
#define TSTARTMARGIN 60.0               // time margin for file name replacement
#define TRACEFILE   "rtkconv_qt.trace"     // trace file

static int abortf = 0;

// show message in message area ---------------------------------------------
extern "C" {
    extern int showmsg(const char *format, ...)
    {
        va_list arg;
        char buff[1024];

        va_start(arg, format); vsprintf(buff, format, arg); va_end(arg);

        QMetaObject::invokeMethod(mainWindow, "showMessage", Qt::AutoConnection, Q_ARG(QString, QString(buff)));

        return abortf;
    }
    extern void settime(gtime_t) {}
    extern void settspan(gtime_t, gtime_t) {}
}

// constructor --------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    mainWindow = this;

    setlocale(LC_NUMERIC, "C");
    setWindowIcon(QIcon(":/icons/rtkconv_Icon.ico"));
    setAcceptDrops(true);

    QString app_filename = QApplication::applicationFilePath();
    QFileInfo fi(app_filename);
    iniFile = QDir(fi.absolutePath()).absoluteFilePath(fi.baseName() + ".ini");

    convOptDialog = new ConvOptDialog(this);
    timeDialog = new TimeDialog(this);
    keyDialog = new KeyDialog(this);
    aboutDialog = new AboutDialog(this, QPixmap(":/icons/rtkconv"), PRGNAME);
    startDialog = new StartDialog(this);
    viewer = new TextViewer(this);

    // populate format combo box
    ui->cBFormat->clear();
    ui->cBFormat->addItem(tr("Auto"));
    for (int i = 0; i <= MAXRCVFMT; i++)
        ui->cBFormat->addItem(formatstrs[i], i);
    ui->cBFormat->addItem(formatstrs[STRFMT_RINEX], STRFMT_RINEX);

    // set up completers to propose the user valid paths
    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lEOutputFile1->setCompleter(fileCompleter);
    ui->lEOutputFile2->setCompleter(fileCompleter);
    ui->lEOutputFile3->setCompleter(fileCompleter);
    ui->lEOutputFile4->setCompleter(fileCompleter);
    ui->lEOutputFile5->setCompleter(fileCompleter);
    ui->lEOutputFile6->setCompleter(fileCompleter);
    ui->lEOutputFile7->setCompleter(fileCompleter);
    ui->lEOutputFile8->setCompleter(fileCompleter);
    ui->lEOutputFile9->setCompleter(fileCompleter);
    ui->cBInputFile->setCompleter(fileCompleter);

    ui->comboTimeInterval->setValidator(new QDoubleValidator());

    QCompleter *dirCompleter = new QCompleter(this);
    QFileSystemModel *dirModel = new QFileSystemModel(dirCompleter);
    dirModel->setRootPath("");
    dirModel->setFilter(QDir::AllDirs | QDir::Drives | QDir::NoDotAndDotDot);
    dirCompleter->setModel(dirModel);
    ui->lEOutputDirectory->setCompleter(dirCompleter);

    // set up actions for viewing and selecting the output files
    QAction *acOutputFileSelect[9], *acOutputFileView[9];
    QLineEdit *lEOutputs[9] = {ui->lEOutputFile1, ui->lEOutputFile2, ui->lEOutputFile3, ui->lEOutputFile4,
                               ui->lEOutputFile5, ui->lEOutputFile6, ui->lEOutputFile7, ui->lEOutputFile8, ui->lEOutputFile9};
    for (int i = 0; i < 9; i++) {
        acOutputFileSelect[i] = lEOutputs[i]->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
        acOutputFileSelect[i]->setToolTip(tr("Select File"));
        acOutputFileView[i] = lEOutputs[i]->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
        acOutputFileView[i]->setToolTip(tr("View File"));
    }

    QAction *acOutputDirSelect = ui->lEOutputDirectory->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acOutputDirSelect->setToolTip(tr("Select Output Directory"));

    connect(ui->btnPlot, &QPushButton::clicked, this, &MainWindow::callRtkPlot);
    connect(ui->btnConvert, &QPushButton::clicked, this, &MainWindow::convertFile);
    connect(ui->btnOptions, &QPushButton::clicked, this, &MainWindow::showOptions);
    connect(ui->btnExit, &QPushButton::clicked, this, &MainWindow::close);
    connect(ui->btnAbout, &QPushButton::clicked, this, &MainWindow::showAboutDialog);
    connect(ui->btnTimeStart, &QPushButton::clicked, this, &MainWindow::showStartTimeDialog);
    connect(ui->btnTimeStop, &QPushButton::clicked, this, &MainWindow::showStopTimeDialog);
    connect(ui->btnKey, &QPushButton::clicked, this, &MainWindow::showKeyDialog);
    connect(ui->btnPost, &QPushButton::clicked, this, &MainWindow::callRtkPost);
    connect(ui->btnAbort, &QCheckBox::clicked, this, &MainWindow::abort);
    connect(ui->btnInputFile, &QPushButton::clicked, this, &MainWindow::selectInputFile);
    connect(ui->btnInputFileView, &QPushButton::clicked, this, &MainWindow::viewInputFile);
    connect(ui->cBOutputFileEnable1, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable2, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable3, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable4, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable5, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable6, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable7, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable8, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputFileEnable9, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(acOutputFileSelect[0], &QAction::triggered, this, &MainWindow::selectOutputFile1);
    connect(acOutputFileSelect[1], &QAction::triggered, this, &MainWindow::selectOutputFile2);
    connect(acOutputFileSelect[2], &QAction::triggered, this, &MainWindow::selectOutputFile3);
    connect(acOutputFileSelect[3], &QAction::triggered, this, &MainWindow::selectOutputFile4);
    connect(acOutputFileSelect[4], &QAction::triggered, this, &MainWindow::selectOutputFile5);
    connect(acOutputFileSelect[5], &QAction::triggered, this, &MainWindow::selectOutputFile6);
    connect(acOutputFileSelect[6], &QAction::triggered, this, &MainWindow::selectOutputFile7);
    connect(acOutputFileSelect[7], &QAction::triggered, this, &MainWindow::selectOutputFile8);
    connect(acOutputFileSelect[8], &QAction::triggered, this, &MainWindow::selectOutputFile9);
    connect(acOutputFileView[0], &QAction::triggered, this, &MainWindow::viewOutputFile1);
    connect(acOutputFileView[1], &QAction::triggered, this, &MainWindow::viewOutputFile2);
    connect(acOutputFileView[2], &QAction::triggered, this, &MainWindow::viewOutputFile3);
    connect(acOutputFileView[3], &QAction::triggered, this, &MainWindow::viewOutputFile4);
    connect(acOutputFileView[4], &QAction::triggered, this, &MainWindow::viewOutputFile5);
    connect(acOutputFileView[5], &QAction::triggered, this, &MainWindow::viewOutputFile6);
    connect(acOutputFileView[6], &QAction::triggered, this, &MainWindow::viewOutputFile7);
    connect(acOutputFileView[7], &QAction::triggered, this, &MainWindow::viewOutputFile8);
    connect(acOutputFileView[8], &QAction::triggered, this, &MainWindow::viewOutputFile9);
    connect(acOutputDirSelect, &QAction::triggered, this, &MainWindow::selectOutputDirectory);
    connect(ui->lEOutputDirectory, &QLineEdit::textChanged, this, &MainWindow::outputDirectoryChanged);
    connect(ui->cBTimeStart, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBTimeEnd, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBTimeInterval, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBTimeUnit, &QCheckBox::clicked, this, &MainWindow::updateEnable);
    connect(ui->cBOutputDirectoryEnable, &QCheckBox::clicked, this, &MainWindow::outputDirectoryEnableClicked);
    connect(ui->cBInputFile, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainWindow::inputFileChanged);
    connect(ui->cBInputFile, &QComboBox::editTextChanged, this, &MainWindow::inputFileChanged);
    connect(ui->cBFormat, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainWindow::updateEnable);

    setWindowTitle(QString(tr("%1 Version %2 %3")).arg(PRGNAME).arg(VER_RTKLIB, PATCH_LEVEL));
    ui->btnAbort->setVisible(false);
}
// callback on form show ----------------------------------------------------
void MainWindow::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    QCommandLineParser parser;
    parser.setApplicationDescription("RTK conv");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption iniFileOption(QStringList() << "i",
                     QCoreApplication::translate("main", "ini file to use"),
                     QCoreApplication::translate("main", "ini file"));
    parser.addOption(iniFileOption);

    QCommandLineOption titleOption(QStringList() << "t",
                       QCoreApplication::translate("main", "window title"),
                       QCoreApplication::translate("main", "title"));
    parser.addOption(titleOption);

    parser.process(*QApplication::instance());

    if (parser.isSet(iniFileOption))
        iniFile = parser.value(iniFileOption);

    loadOptions();

    if (parser.isSet(titleOption))
        setWindowTitle(parser.value(titleOption));
}
// callback on form close ---------------------------------------------------
void MainWindow::closeEvent(QCloseEvent *)
{
    saveOptions();
}
// set output file paths ----------------------------------------------------
void MainWindow::setOutputFiles(const QString &infile)
{
    QLineEdit *edit[] = {
        ui->lEOutputFile1, ui->lEOutputFile2, ui->lEOutputFile3, ui->lEOutputFile4, ui->lEOutputFile5,
        ui->lEOutputFile6, ui->lEOutputFile7, ui->lEOutputFile8, ui->lEOutputFile9
    };
    QDir outputDirectory(ui->lEOutputDirectory->text());
    QString ofile[10];

    if (infile.isEmpty()) return;

    if (ui->cBOutputDirectoryEnable->isChecked()) {
        QFileInfo info(infile);

        ofile[0] = outputDirectory.filePath(info.fileName());
    } else {
        ofile[0] = infile;
    }
    ofile[0].replace('*', '0');
    ofile[0].replace('?', '0');

    if (!convOptDialog->rinexFile) {
        QFileInfo info(ofile[0]);
        ofile[0] = info.absoluteDir().filePath(info.baseName());
        ofile[1] = ofile[0] + ".obs";
        ofile[2] = ofile[0] + ".nav";
        ofile[3] = ofile[0] + ".gnav";
        ofile[4] = ofile[0] + ".hnav";
        ofile[5] = ofile[0] + ".qnav";
        ofile[6] = ofile[0] + ".lnav";
        ofile[7] = ofile[0] + ".cnav";
        ofile[8] = ofile[0] + ".inav";
        ofile[9] = ofile[0] + ".sbs";
    } else {
        QFileInfo info(ofile[0]);
        ofile[0] = info.filePath() + "/";
        ofile[1] += ofile[0] + QString("%%r%%n0.%%yO");
        if (convOptDialog->rinexVersion >= 3 && convOptDialog->navSys && (convOptDialog->navSys != SYS_GPS)) /* ver.3 and mixed system */
            ofile[2] += ofile[0] + "%%r%%n0.%%yP";
        else
            ofile[2] += ofile[0] + "%%r%%n0.%%yN";
        ofile[3] += ofile[0] + "%%r%%n0.%%yG";
        ofile[4] += ofile[0] + "%%r%%n0.%%yH";
        ofile[5] += ofile[0] + "%%r%%n0.%%yQ";
        ofile[6] += ofile[0] + "%%r%%n0.%%yL";
        ofile[7] += ofile[0] + "%%r%%n0.%%yC";
        ofile[8] += ofile[0] + "%%r%%n0.%%yI";
        ofile[9] += ofile[0] + "%%r%%n0_%%y.sbs";
    }
    for (int i = 0; i < 9; i++) {
        if (ofile[i + 1] == infile) ofile[i + 1] += "_";
        ofile[i + 1] = QDir::toNativeSeparators(ofile[i + 1]);
        edit[i]->setText(ofile[i + 1]);
    }
}
// callback on file drag and drop -------------------------------------------
void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    trace(3, "dragEnterEvent\n");

    if (event->mimeData()->hasFormat("text/uri-list"))
        event->acceptProposedAction();
}

// callback on file drag and drop -------------------------------------------
void MainWindow::dropEvent(QDropEvent *event)
{
    trace(3, "dropEvent\n");

    if (!event->mimeData()->hasFormat("text/uri-list")) return;

    QString file = QDir::toNativeSeparators(QUrl(event->mimeData()->text()).toLocalFile());

    ui->cBInputFile->setCurrentText(file);
    setOutputFiles(ui->cBInputFile->currentText());
}
// show message --------------------------------------------------------------
void MainWindow::showMessage(const QString &msg)
{
    ui->lblMessage->setText(msg);
}
// add history --------------------------------------------------------------
void MainWindow::addHistory(QComboBox *combo)
{
    QString history = combo->currentText();

    if (history.isEmpty()) return;

    // move current item to to
    int idx = combo->currentIndex();
    if (idx >= 0) combo->removeItem(idx);
    combo->insertItem(0, history);

    // limit length of history
    for (int i = combo->count() - 1; i >= MAXHIST; i--) combo->removeItem(i);

    combo->setCurrentIndex(0);
}
// read history -------------------------------------------------------------
void MainWindow::readHistory(QComboBox *combo, QSettings *ini, const QString &key)
{
    QString item;
    for (int i = 0; i < 100; i++) {
        item = ini->value(QString("%1_%2").arg(key).arg(i, 3), "").toString();
        if ((!item.isEmpty()) && (combo->findText(item)==-1)) combo->addItem(item);
    }
}
// write history ------------------------------------------------------------
void MainWindow::writeHistory(QSettings *ini, const QString &key, const QComboBox *combo)
{
    for (int i = 0; i < combo->count(); i++)
        ini->setValue(QString("%1_%2").arg(key).arg(i, 3), combo->itemText(i));
}
// callback on button-plot --------------------------------------------------
void MainWindow::callRtkPlot()
{
    int i, ena[8];
    QString file[8] = {
        ui->lEOutputFile1->text(), ui->lEOutputFile2->text(), ui->lEOutputFile3->text(), ui->lEOutputFile4->text(),
        ui->lEOutputFile5->text(), ui->lEOutputFile6->text(), ui->lEOutputFile7->text(), ui->lEOutputFile8->text()
    };
    QCheckBox *cb[8] = {
        ui->cBOutputFileEnable1, ui->cBOutputFileEnable2, ui->cBOutputFileEnable3, ui->cBOutputFileEnable4,
        ui->cBOutputFileEnable5, ui->cBOutputFileEnable6, ui->cBOutputFileEnable7, ui->cBOutputFileEnable8
    };
    QStringList cmds = {"rtkplot_qt", "../../../bin/rtkplot_qt", "../rtkplot_qt/rtkplot_qt"};
    QStringList opts;
    QDir appDir = QDir(QCoreApplication::applicationDirPath());

    opts << "-r";

    for (i = 0; i < 8; i++) ena[i] = cb[i]->isEnabled() && cb[i]->isChecked();

    for (i = 0; i < 8; i++)
        if (ena[i]) opts << repPath(file[i]);

    if (opts.size() == 1) return;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }

    showMessage(tr("Error: Could not execute rtkplot_qt"));
}
// callback on button-post-proc ---------------------------------------------
void MainWindow::callRtkPost()
{
    QStringList cmds = {commandPostExe, QDir("../../../bin/").filePath(commandPostExe), QDir("../rtkpost_qt/").filePath(commandPostExe)};
    QStringList opts;
    QDir appDir = QDir(QCoreApplication::applicationDirPath());

    if (!ui->cBOutputFileEnable1->isChecked()) return;

    opts << "-r" << ui->lEOutputFile1->text();
    opts << "-n" << "" << "-n" << "";

    if (ui->cBOutputFileEnable9->isChecked())
        opts << "-n" << ui->lEOutputFile9->text();

    if (ui->cBTimeStart->isChecked()) opts << + "-ts" << ui->dateTimeStart->dateTime().toString("yyyy/MM/dd hh:mm:ss");
    if (ui->cBTimeEnd->isChecked()) opts << "-te" << ui->dateTimeStop->dateTime().toString("yyyy/MM/dd hh:mm:ss");
    if (ui->cBTimeInterval->isChecked()) opts << "-ti" << ui->comboTimeInterval->currentText();
    if (ui->cBTimeUnit->isChecked()) opts << "-tu" << ui->cBTimeUnit->text();

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }

    showMessage(tr("Error: Could not execute rtkpost_qt"));
}
// callback on button-options -----------------------------------------------
void MainWindow::showOptions()
{
    int rnxfile_old = convOptDialog->rinexFile;

    convOptDialog->exec();
    if (convOptDialog->result() != QDialog::Accepted) return;

    if (convOptDialog->rinexFile != rnxfile_old)
        setOutputFiles(ui->cBInputFile->currentText());

    updateEnable();
}
//---------------------------------------------------------------------------
void MainWindow::abort()
{
    abortf = 1;  // set global abort flag
}
// callbck on button-time-1 -------------------------------------------------
void MainWindow::showStartTimeDialog()
{
    gtime_t ts = {0, 0}, te = {0, 0};
    double tint = 0.0, tunit = 0.0;

    getTime(&ts, &te, &tint, &tunit);
    timeDialog->setTime(ts);
    timeDialog->exec();
}
// callbck on button-time-2 -------------------------------------------------
void MainWindow::showStopTimeDialog()
{
    gtime_t ts = {0, 0}, te = {0, 0};
    double tint = 0.0, tunit = 0.0;

    getTime(&ts, &te, &tint, &tunit);
    timeDialog->setTime(te);
    timeDialog->exec();
}
// callback on button-input-file --------------------------------------------
void MainWindow::selectInputFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Input RTCM, RCV RAW or RINEX File"), ui->cBInputFile->currentText(),
                                                    tr("All (*.*);;RTCM 2 (*.rtcm2);;RTCM 3 (*.rtcm3);;NovtAtel (*.gps);;ublox (*.ubx);;SuperStart II (*.log);;"
                                                       "Hemisphere (*.bin);;Javad (*.jps);;RINEX OBS (*.obs *.*O);Septentrio (*.sbf)"));

    if (!filename.isEmpty()) {
        ui->cBInputFile->setCurrentText(QDir::toNativeSeparators(filename));
        setOutputFiles(ui->cBInputFile->currentText());
    }
}
// callback on output-directory change --------------------------------------
void MainWindow::outputDirectoryChanged()
{
    setOutputFiles(ui->cBInputFile->currentText());
}
// callback on button-output-directory --------------------------------------
void MainWindow::selectOutputDirectory()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Output Directory"), ui->lEOutputDirectory->text());
    if (dir.isEmpty()) return;
    ui->lEOutputDirectory->setText(QDir::toNativeSeparators(dir));
    setOutputFiles(ui->cBInputFile->currentText());
}
// callback on button-keyword -----------------------------------------------
void MainWindow::showKeyDialog()
{
    keyDialog->setFlag(1);
    keyDialog->show();
}
// callback on button-output-file-1 -----------------------------------------
void MainWindow::selectOutputFile1()
{
    QString selectedFilter = tr("RINEX OBS (*.obs *.*O)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output RINEX OBS File"), ui->lEOutputFile1->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);

    if (!filename.isEmpty())
        ui->lEOutputFile1->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-2 -----------------------------------------
void MainWindow::selectOutputFile2()
{
    QString selectedFilter = tr("RINEX NAV (*.nav *.*N *.*P)");
    QString filename = QFileDialog::getOpenFileName(this, tr("Output RINEX NAV File"), ui->lEOutputFile2->text(),
                                        tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                           "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                           "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);

    if (!filename.isEmpty())
        ui->lEOutputFile2->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-3 -----------------------------------------
void MainWindow::selectOutputFile3()
{
    QString selectedFilter = tr("RINEX GNAV (*.gnav *.*G)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output RINEX GNAV File"), ui->lEOutputFile3->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);
    if (!filename.isEmpty())
        ui->lEOutputFile3->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-4 -----------------------------------------
void MainWindow::selectOutputFile4()
{
    QString selectedFilter = tr("RINEX HNAV (*.hnav *.*H)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output RINEX HNAV File"), ui->lEOutputFile4->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);
    if (!filename.isEmpty())
        ui->lEOutputFile4->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-5 -----------------------------------------
void MainWindow::selectOutputFile5()
{
    QString selectedFilter = tr("RINEX QNAV (*.qnav *.*Q)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output RINEX QNAV File"), ui->lEOutputFile5->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);

    if (!filename.isEmpty())
        ui->lEOutputFile5->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-6 -----------------------------------------
void MainWindow::selectOutputFile6()
{
    QString selectedFilter = tr("RINEX LNAV (*.lnav *.*L)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output RINEX LNAV File"), ui->lEOutputFile6->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);

    if (!filename.isEmpty())
        ui->lEOutputFile6->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-7 -----------------------------------------
void MainWindow::selectOutputFile7()
{
    QString selectedFilter = tr("RINEX CNAV (*.cnav *.*C)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output SRINEX CNAVFile"), ui->lEOutputFile7->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);

    if (!filename.isEmpty())
        ui->lEOutputFile7->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-8 -----------------------------------------
void MainWindow::selectOutputFile8()
{
    QString selectedFilter = tr("RINEX INAV (*.inav *.*I)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output SBAS/LEX Log File"), ui->lEOutputFile8->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);

    if (!filename.isEmpty())
        ui->lEOutputFile8->setText(QDir::toNativeSeparators(filename));
}
// callback on button-output-file-9 -----------------------------------------
void MainWindow::selectOutputFile9()
{
    QString selectedFilter = tr("SBAS Log (*.sbs)");

    QString filename = QFileDialog::getOpenFileName(this, tr("Output SBAS/LEX Log File"), ui->lEOutputFile9->text(),
                                                    tr("All (*.*);;RINEX OBS (*.obs *.*O);;RINEX NAV (*.nav *.*N *.*P);;RINEX GNAV (*.gnav *.*G);;RINEX HNAV (*.hnav *.*H);;"
                                                       "RINEX QNAV (*.qnav *.*Q);;RINEX LNAV (*.lnav *.*L);;RINEX CNAV (*.cnav *.*C);;RINEX INAV (*.inav *.*I);;"
                                                       "SBAS Log (*.sbs);;LEX Log (*.lex)"), &selectedFilter);

    if (!filename.isEmpty())
        ui->lEOutputFile9->setText(QDir::toNativeSeparators(filename));
}
// callback on button-view-input-file ----------------------------------------
void MainWindow::viewInputFile()
{
    QString inputFilename = ui->cBInputFile->currentText();
    QString ext = QFileInfo(inputFilename).suffix();

    if (ext.length() < 3) return;

    if ((ext.toLower() == "obs") || (ext.toLower() == "nav") ||
        (ext.mid(1).toLower() == "nav") ||
        (ext.at(2) == 'o') || (ext.at(2) == 'O') || (ext.at(2) == 'n') ||
        (ext.at(2) == 'N') || (ext.at(2) == 'p') || (ext.at(2) == 'P') ||
        (ext.at(2) == 'g') || (ext.at(2) == 'G') || (ext.at(2) == 'h') ||
        (ext.at(2) == 'H') || (ext.at(2) == 'q') || (ext.at(2) == 'Q') ||
        (ext.at(2) == 'l') || (ext.at(2) == 'L') || (ext.at(2) == 'c') ||
        (ext.at(2) == 'C') || (ext.at(2) == 'I') || (ext.at(2) == 'i') ||
            (ext == "rnx") || (ext == "RNX")) {
        viewer->show();
        viewer->read(repPath(inputFilename));
    } else
        QToolTip::showText(ui->btnInputFileView->mapToGlobal( QPoint( 0, 0 ) ), tr("<font color='#ff0000'>Unrecognized file type</font>") );
}
// callback on button-view-file-1 -------------------------------------------
void MainWindow::viewOutputFile1()
{
    viewer->read(repPath(ui->lEOutputFile1->text()));
    viewer->show();
}
// callback on button-view-file-2 -------------------------------------------
void MainWindow::viewOutputFile2()
{
    viewer->read(repPath(ui->lEOutputFile2->text()));
    viewer->show();
}
// callback on button-view-file-3 -------------------------------------------
void MainWindow::viewOutputFile3()
{
    viewer->read(repPath(ui->lEOutputFile3->text()));
    viewer->show();
}
// callback on button-view-file-4 -------------------------------------------
void MainWindow::viewOutputFile4()
{
    viewer->read(repPath(ui->lEOutputFile4->text()));
    viewer->show();
}
// callback on button-view-file-5 -------------------------------------------
void MainWindow::viewOutputFile5()
{
    viewer->read(repPath(ui->lEOutputFile5->text()));
    viewer->show();
}
// callback on button-view-file-6 -------------------------------------------
void MainWindow::viewOutputFile6()
{
    viewer->read(repPath(ui->lEOutputFile6->text()));
    viewer->show();
}
// callback on button-view-file-7 -------------------------------------------
void MainWindow::viewOutputFile7()
{
    viewer->read(repPath(ui->lEOutputFile7->text()));
    viewer->show();
}
// callback on button-view-file-8 -------------------------------------------
void MainWindow::viewOutputFile8()
{
    viewer->read(repPath(ui->lEOutputFile8->text()));
    viewer->show();
}
// callback on button-view-file-9 -------------------------------------------
void MainWindow::viewOutputFile9()
{
    viewer->read(repPath(ui->lEOutputFile9->text()));
    viewer->show();
}
// callback on button-about -------------------------------------------------
void MainWindow::showAboutDialog()
{
    aboutDialog->exec();
}
// callback on output-file check/uncheck ------------------------------------
void MainWindow::outputDirectoryEnableClicked()
{
    setOutputFiles(ui->cBInputFile->currentText());

    updateEnable();
}
// callback on input-file-change --------------------------------------------
void MainWindow::inputFileChanged()
{
    setOutputFiles(ui->cBInputFile->currentText());
}
// get time -----------------------------------------------------------------
void MainWindow::getTime(gtime_t *ts, gtime_t *te, double *tint, double *tunit)
{
    if (ui->cBTimeStart->isChecked()) {
        QDateTime start(ui->dateTimeStart->dateTime());
        ts->time = start.toSecsSinceEpoch();
        ts->sec = start.time().msec() / 1000;
    } else {
        ts->time = 0;
        ts->sec = 0;
    }

    if (ui->cBTimeEnd->isChecked()) {
        QDateTime end(ui->dateTimeStop->dateTime());
        te->time = end.toSecsSinceEpoch();
        te->sec = end.time().msec() / 1000;
    } else {
        te->time = 0;
        te->sec = 0;
    }

    if (ui->cBTimeInterval->isChecked())
        *tint = ui->comboTimeInterval->currentText().toDouble();
    else
        *tint = 0;

    if (ui->cBTimeUnit->isChecked())
        *tunit = ui->sBTimeUnit->value() * 3600;
    else
        *tunit = 0;
}
// replace keywords in file path --------------------------------------------
QString MainWindow::repPath(const QString &File)
{
    char path[1024];

    reppath(qPrintable(File), path, timeadd(convOptDialog->rinexTime, TSTARTMARGIN), qPrintable(convOptDialog->rinexStationCode), "");
    return QString(path);
}
// execute command ----------------------------------------------------------
int MainWindow::execCommand(const QString &cmd, QStringList &opts)
{
    return QProcess::startDetached(cmd, opts);
}
// update enable/disable of widgets -----------------------------------------
void MainWindow::updateEnable()
{
    int rnx = (ui->cBFormat->currentText() == tr("RINEX"));
    int sep_nav = (convOptDialog->rinexVersion < 3 || convOptDialog->separateNavigation);

    ui->dateTimeStart->setEnabled(ui->cBTimeStart->isChecked());
    ui->btnTimeStart->setEnabled(ui->cBTimeStart->isChecked());
    ui->dateTimeStop->setEnabled(ui->cBTimeEnd->isChecked());
    ui->btnTimeStop->setEnabled(ui->cBTimeEnd->isChecked());
    ui->comboTimeInterval->setEnabled(ui->cBTimeInterval->isChecked());
    ui->lbTimeInterval->setEnabled(ui->cBTimeInterval->isEnabled());
    ui->cBTimeUnit->setEnabled(ui->cBTimeStart->isChecked() && ui->cBTimeEnd->isChecked());
    ui->sBTimeUnit->setEnabled(ui->cBTimeStart->isChecked() && ui->cBTimeEnd->isChecked() && ui->cBTimeUnit->isChecked());
    ui->cBOutputFileEnable3->setEnabled(sep_nav && (convOptDialog->navSys & SYS_GLO));
    ui->cBOutputFileEnable4->setEnabled(sep_nav && (convOptDialog->navSys & SYS_SBS));
    ui->cBOutputFileEnable5->setEnabled(sep_nav && (convOptDialog->navSys & SYS_QZS) && convOptDialog->rinexVersion >= 5);
    ui->cBOutputFileEnable6->setEnabled(sep_nav && (convOptDialog->navSys & SYS_GAL) && convOptDialog->rinexVersion >= 2);
    ui->cBOutputFileEnable7->setEnabled(sep_nav && (convOptDialog->navSys & SYS_CMP) && convOptDialog->rinexVersion >= 4);
    ui->cBOutputFileEnable8->setEnabled(sep_nav && (convOptDialog->navSys & SYS_IRN) && convOptDialog->rinexVersion >= 6);
    ui->cBOutputFileEnable9->setEnabled(!rnx);
    ui->lEOutputDirectory->setEnabled(ui->cBOutputDirectoryEnable->isChecked());
    ui->lEOutputFile1->setEnabled(ui->cBOutputFileEnable1->isChecked());
    ui->lEOutputFile2->setEnabled(ui->cBOutputFileEnable2->isChecked());
    ui->lEOutputFile3->setEnabled(ui->cBOutputFileEnable3->isChecked() && ui->cBOutputFileEnable3->isEnabled());
    ui->lEOutputFile4->setEnabled(ui->cBOutputFileEnable4->isChecked() && ui->cBOutputFileEnable4->isEnabled());
    ui->lEOutputFile5->setEnabled(ui->cBOutputFileEnable5->isChecked() && ui->cBOutputFileEnable5->isEnabled());
    ui->lEOutputFile6->setEnabled(ui->cBOutputFileEnable6->isChecked() && ui->cBOutputFileEnable6->isEnabled());
    ui->lEOutputFile7->setEnabled(ui->cBOutputFileEnable7->isChecked() && ui->cBOutputFileEnable7->isEnabled());
    ui->lEOutputFile8->setEnabled(ui->cBOutputFileEnable8->isChecked() && ui->cBOutputFileEnable8->isEnabled());
    ui->lEOutputFile9->setEnabled(ui->cBOutputFileEnable9->isChecked() && ui->cBOutputFileEnable9->isEnabled() && !rnx);

    convOptDialog->setTimeIntervalEnabled(ui->cBTimeInterval->isChecked());
}
// convert file -------------------------------------------------------------
void MainWindow::convertFile()
{
    QString inputFile_Text = ui->cBInputFile->currentText();
    int i;
    double RNXVER[] = { 210, 211, 212, 300, 301, 302, 303, 304 };

    conversionThread = new ConversionThread(this);

    // recognize input file format
    strncpy(conversionThread->ifile, qPrintable(inputFile_Text), 1023);
    QFileInfo fi(inputFile_Text);
    if (ui->cBFormat->currentIndex() == 0) { // auto
        if (fi.completeSuffix() == "rtcm2") {
            conversionThread->format = STRFMT_RTCM2;
        } else if (fi.completeSuffix() == "rtcm3") {
            conversionThread->format = STRFMT_RTCM3;
        } else if (fi.completeSuffix() == "gps") {
            conversionThread->format = STRFMT_OEM4;
        } else if (fi.completeSuffix() == "ubx") {
            conversionThread->format = STRFMT_UBX;
        } else if (fi.completeSuffix() == "sbp") {
            conversionThread->format = STRFMT_SBP;
        } else if (fi.completeSuffix() == "bin") {
            conversionThread->format = STRFMT_CRES;
        } else if (fi.completeSuffix() == "jps") {
            conversionThread->format = STRFMT_JAVAD;
        } else if (fi.completeSuffix() == "bnx") {
            conversionThread->format = STRFMT_BINEX;
        } else if (fi.completeSuffix() == "binex") {
            conversionThread->format = STRFMT_BINEX;
        } else if (fi.completeSuffix() == "rt17") {
            conversionThread->format = STRFMT_RT17;
        } else if (fi.completeSuffix() == "sbf") {
            conversionThread->format = STRFMT_SEPT;
        } else if (fi.completeSuffix() == "unc") {
            conversionThread->format = STRFMT_UNICORE;
        } else if (fi.completeSuffix().toLower() == "obs") {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().toLower().contains("nav")) {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'o') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'O') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'n') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'N') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'p') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'P') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'g') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'G') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'h') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'H') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'q') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'Q') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'l') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'L') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'c') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'C') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'i') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix().at(2) == 'I') {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix() == "rnx") {
            conversionThread->format = STRFMT_RINEX;
        } else if (fi.completeSuffix() == "RNX") {
            conversionThread->format = STRFMT_RINEX;
        } else {
            showMessage(tr("Error: File format can not be recognized"));
            return;
        }
    } else {
        conversionThread->format = ui->cBFormat->currentData().toInt();
        if (conversionThread->format < 0)
            return;
    }
    conversionThread->rnxopt.rnxver = RNXVER[convOptDialog->rinexVersion];

    if (conversionThread->format == STRFMT_RTCM2 || conversionThread->format == STRFMT_RTCM3 || conversionThread->format == STRFMT_RT17) {
        // input start date/time for rtcm 2, rtcm 3, RT17 or CMR
        startDialog->exec();
        if (startDialog->result() != QDialog::Accepted) {
            delete conversionThread;
            return;
        }

        conversionThread->rnxopt.trtcm = startDialog->getTime();
    }
    if (ui->lEOutputFile1->isEnabled() && ui->cBOutputFileEnable1->isChecked()) strncpy(conversionThread->ofile[0], qPrintable(ui->lEOutputFile1->text()), 1023);
    if (ui->lEOutputFile2->isEnabled() && ui->cBOutputFileEnable2->isChecked()) strncpy(conversionThread->ofile[1], qPrintable(ui->lEOutputFile2->text()), 1023);
    if (ui->lEOutputFile3->isEnabled() && ui->cBOutputFileEnable3->isChecked()) strncpy(conversionThread->ofile[2], qPrintable(ui->lEOutputFile3->text()), 1023);
    if (ui->lEOutputFile4->isEnabled() && ui->cBOutputFileEnable4->isChecked()) strncpy(conversionThread->ofile[3], qPrintable(ui->lEOutputFile4->text()), 1023);
    if (ui->lEOutputFile5->isEnabled() && ui->cBOutputFileEnable5->isChecked()) strncpy(conversionThread->ofile[4], qPrintable(ui->lEOutputFile5->text()), 1023);
    if (ui->lEOutputFile6->isEnabled() && ui->cBOutputFileEnable6->isChecked()) strncpy(conversionThread->ofile[5], qPrintable(ui->lEOutputFile6->text()), 1023);
    if (ui->lEOutputFile7->isEnabled() && ui->cBOutputFileEnable7->isChecked()) strncpy(conversionThread->ofile[6], qPrintable(ui->lEOutputFile7->text()), 1023);
    if (ui->lEOutputFile8->isEnabled() && ui->cBOutputFileEnable8->isChecked()) strncpy(conversionThread->ofile[7], qPrintable(ui->lEOutputFile8->text()), 1023);
    if (ui->lEOutputFile9->isEnabled() && ui->cBOutputFileEnable9->isChecked()) strncpy(conversionThread->ofile[8], qPrintable(ui->lEOutputFile9->text()), 1023);

    // check overwrite output file
    for (i = 0; i < 9; i++) {
        if (!QFile(conversionThread->ofile[i]).exists()) continue;
        if (QMessageBox::question(this, tr("Overwrite"), QString(tr("%1 exists. Do you want to overwrite?")).arg(conversionThread->ofile[i])) != QMessageBox::Yes) {
            delete conversionThread;
            return;
        }
    }
    getTime(&conversionThread->rnxopt.ts, &conversionThread->rnxopt.te, &conversionThread->rnxopt.tint, &conversionThread->rnxopt.tunit);
    strncpy(conversionThread->rnxopt.staid, qPrintable(convOptDialog->rinexStationCode), 31);
    sprintf(conversionThread->rnxopt.prog, "%s %s %s", PRGNAME, VER_RTKLIB, PATCH_LEVEL);
    strncpy(conversionThread->rnxopt.runby, qPrintable(convOptDialog->runBy), 31);
    strncpy(conversionThread->rnxopt.marker, qPrintable(convOptDialog->marker), 63);
    strncpy(conversionThread->rnxopt.markerno, qPrintable(convOptDialog->markerNo), 31);
    strncpy(conversionThread->rnxopt.markertype, qPrintable(convOptDialog->markerType), 31);
    for (i = 0; i < 2; i++) strncpy(conversionThread->rnxopt.name[i], qPrintable(convOptDialog->name[i]), 31);
    for (i = 0; i < 3; i++) strncpy(conversionThread->rnxopt.rec[i], qPrintable(convOptDialog->receiver [i]), 31);
    for (i = 0; i < 3; i++) strncpy(conversionThread->rnxopt.ant[i], qPrintable(convOptDialog->antenna [i]), 31);
    if (convOptDialog->autoPosition)
        for (i = 0; i < 3; i++) conversionThread->rnxopt.apppos[i] = convOptDialog->approxPosition[i];
    for (i = 0; i < 3; i++) conversionThread->rnxopt.antdel[i] = convOptDialog->antennaDelta[i];
    strncpy(conversionThread->rnxopt.rcvopt, qPrintable(convOptDialog->receiverOptions), 255);
    conversionThread->rnxopt.navsys = convOptDialog->navSys;
    conversionThread->rnxopt.obstype = convOptDialog->observationType;
    conversionThread->rnxopt.freqtype = convOptDialog->frequencyType;
    for (i = 0; i < 2; i++)
        rnxcomment(&conversionThread->rnxopt, "%s", qPrintable(convOptDialog->comment[i]));
    for (i = 0; i < 7; i++) strncpy(conversionThread->rnxopt.mask[i], qPrintable(convOptDialog->codeMask[i]), 63);
    conversionThread->rnxopt.autopos = convOptDialog->autoPosition;
    conversionThread->rnxopt.phshift = convOptDialog->phaseShift;
    conversionThread->rnxopt.halfcyc = convOptDialog->halfCycle;
    conversionThread->rnxopt.outiono = convOptDialog->outputIonoCorr;
    conversionThread->rnxopt.outtime = convOptDialog->outputTimeCorr;
    conversionThread->rnxopt.outleaps = convOptDialog->outputLeapSeconds;
    conversionThread->rnxopt.sep_nav = convOptDialog->separateNavigation;
    conversionThread->rnxopt.ttol = convOptDialog->timeTolerance;
    if (convOptDialog->enableGlonassFrequency) {
        for (i = 0; i < MAXPRNGLO; i++) conversionThread->rnxopt.glofcn[i] = convOptDialog->glonassFrequency[i];
    }

    QStringList exsatsLst = convOptDialog->excludedSatellites.split(" ", Qt::SkipEmptyParts);
    foreach(const QString &sat, exsatsLst){
        int satid;

        if (!(satid = satid2no(qPrintable(sat)))) continue;
        conversionThread->rnxopt.exsats[satid - 1] = 1;
    }

    abortf = 0;
    ui->btnConvert->setVisible(false);
    ui->btnAbort->setVisible(true);
    ui->panelTime->setEnabled(false);
    ui->panelMain->setEnabled(false);
    ui->btnPlot->setEnabled(false);
    ui->btnPost->setEnabled(false);
    ui->btnOptions->setEnabled(false);
    ui->btnExit->setEnabled(false);
    ui->cBFormat->setEnabled(false);
    ui->btnKey->setEnabled(false);
    ui->lbInputFile->setEnabled(false);
    ui->cBOutputDirectoryEnable->setEnabled(false);
    ui->lbOutputFile->setEnabled(false);
    ui->lbFormat->setEnabled(false);
    ui->lblMessage->setText("");

    if (convOptDialog->traceLevel > 0) {
        traceopen(TRACEFILE);
        tracelevel(convOptDialog->traceLevel);
    }
    setCursor(Qt::WaitCursor);

    // post processing positioning
    connect(conversionThread, &QThread::finished, this, &MainWindow::conversionFinished);

    conversionThread->start();
}
// conversion done -------------------------------------------------------------
void MainWindow::conversionFinished()
{
    setCursor(Qt::ArrowCursor);

    if (convOptDialog->traceLevel > 0)
        traceclose();

    ui->btnConvert->setVisible(true);
    ui->btnAbort->setVisible(false);
    ui->panelTime->setEnabled(true);
    ui->panelMain->setEnabled(true);
    ui->btnPlot->setEnabled(true);
    ui->btnPost->setEnabled(true);
    ui->btnOptions->setEnabled(true);
    ui->btnExit->setEnabled(true);
    ui->cBFormat->setEnabled(true);
    ui->btnKey->setEnabled(true);
    ui->lbInputFile->setEnabled(true);
    ui->cBOutputDirectoryEnable->setEnabled(true);
    ui->lbOutputFile->setEnabled(true);
    ui->lbFormat->setEnabled(true);

    convOptDialog->rinexTime = conversionThread->rnxopt.tstart;

    conversionThread->deleteLater();

    addHistory(ui->cBInputFile);
}
// load options -------------------------------------------------------------
void MainWindow::loadOptions()
{
    QSettings ini(iniFile, QSettings::IniFormat);

    convOptDialog->loadOptions(ini);

    ui->cBTimeStart->setChecked(ini.value("set/timestartf", false).toBool());
    ui->cBTimeEnd->setChecked(ini.value("set/timeendf", false).toBool());
    ui->cBTimeInterval->setChecked(ini.value("set/timeintf", false).toBool());
    ui->dateTimeStart->setDate(ini.value("set/timey1", "2020/01/01").value<QDate>());
    ui->dateTimeStart->setTime(ini.value("set/timeh1", "00:00:00").value<QTime>());
    ui->dateTimeStop->setDate(ini.value("set/timey2", "2020/01/01").value<QDate>());
    ui->dateTimeStop->setTime(ini.value("set/timeh2", "00:00:00").value<QTime>());
    ui->comboTimeInterval->setCurrentText(ini.value("set/timeint", "1").toString());
    ui->cBTimeUnit->setChecked(ini.value("set/timeunitf", false).toBool());
    ui->sBTimeUnit->setValue(ini.value("set/timeunit", 24).toDouble());
    ui->cBInputFile->setCurrentText(ini.value("set/infile", "").toString());
    ui->lEOutputDirectory->setText(ini.value("set/outdir", "").toString());
    ui->lEOutputFile1->setText(ini.value("set/outfile1", "").toString());
    ui->lEOutputFile2->setText(ini.value("set/outfile2", "").toString());
    ui->lEOutputFile3->setText(ini.value("set/outfile3", "").toString());
    ui->lEOutputFile4->setText(ini.value("set/outfile4", "").toString());
    ui->lEOutputFile5->setText(ini.value("set/outfile5", "").toString());
    ui->lEOutputFile6->setText(ini.value("set/outfile6", "").toString());
    ui->lEOutputFile7->setText(ini.value("set/outfile7", "").toString());
    ui->lEOutputFile8->setText(ini.value("set/outfile8", "").toString());
    ui->lEOutputFile9->setText(ini.value("set/outfile9", "").toString());
    ui->cBOutputDirectoryEnable->setChecked(ini.value("set/outdirena", false).toBool());
    ui->cBOutputFileEnable1->setChecked(ini.value("set/outfileena1", true).toBool());
    ui->cBOutputFileEnable2->setChecked(ini.value("set/outfileena2", true).toBool());
    ui->cBOutputFileEnable3->setChecked(ini.value("set/outfileena3", true).toBool());
    ui->cBOutputFileEnable4->setChecked(ini.value("set/outfileena4", true).toBool());
    ui->cBOutputFileEnable5->setChecked(ini.value("set/outfileena5", true).toBool());
    ui->cBOutputFileEnable6->setChecked(ini.value("set/outfileena6", true).toBool());
    ui->cBOutputFileEnable7->setChecked(ini.value("set/outfileena7", true).toBool());
    ui->cBOutputFileEnable8->setChecked(ini.value("set/outfileena8", true).toBool());
    ui->cBOutputFileEnable9->setChecked(ini.value("set/outfileena9", true).toBool());
    ui->cBFormat->setCurrentIndex(ini.value("set/format", 0).toInt());
    commandPostExe = ini.value("set/cmdpostexe", "rtkpost_qt").toString();

    readHistory(ui->cBInputFile, &ini, "hist/inputfile");

    viewer->loadOptions(ini);

    updateEnable();
}
// save options -------------------------------------------------------------
void MainWindow::saveOptions()
{
    QSettings ini(iniFile, QSettings::IniFormat);

    convOptDialog->saveOptions(ini);

    ini.setValue("set/timestartf", ui->cBTimeStart->isChecked());
    ini.setValue("set/timeendf", ui->cBTimeEnd->isChecked());
    ini.setValue("set/timeintf", ui->cBTimeInterval->isChecked());
    ini.setValue("set/timey1", ui->dateTimeStart->date());
    ini.setValue("set/timeh1", ui->dateTimeStart->time());
    ini.setValue("set/timey2", ui->dateTimeStop->date());
    ini.setValue("set/timeh2", ui->dateTimeStop->time());
    ini.setValue("set/timeint", ui->comboTimeInterval->currentText());
    ini.setValue("set/timeunitf", ui->cBTimeUnit->isChecked());
    ini.setValue("set/timeunit", ui->sBTimeUnit->value());
    ini.setValue("set/infile", ui->cBInputFile->currentText());
    ini.setValue("set/outdir", ui->lEOutputDirectory->text());
    ini.setValue("set/outfile1", ui->lEOutputFile1->text());
    ini.setValue("set/outfile2", ui->lEOutputFile2->text());
    ini.setValue("set/outfile3", ui->lEOutputFile3->text());
    ini.setValue("set/outfile4", ui->lEOutputFile4->text());
    ini.setValue("set/outfile5", ui->lEOutputFile5->text());
    ini.setValue("set/outfile6", ui->lEOutputFile6->text());
    ini.setValue("set/outfile7", ui->lEOutputFile7->text());
    ini.setValue("set/outfile8", ui->lEOutputFile8->text());
    ini.setValue("set/outfile9", ui->lEOutputFile9->text());
    ini.setValue("set/outdirena", ui->cBOutputDirectoryEnable->isChecked());
    ini.setValue("set/outfileena1", ui->cBOutputFileEnable1->isChecked());
    ini.setValue("set/outfileena2", ui->cBOutputFileEnable2->isChecked());
    ini.setValue("set/outfileena3", ui->cBOutputFileEnable3->isChecked());
    ini.setValue("set/outfileena4", ui->cBOutputFileEnable4->isChecked());
    ini.setValue("set/outfileena5", ui->cBOutputFileEnable5->isChecked());
    ini.setValue("set/outfileena6", ui->cBOutputFileEnable6->isChecked());
    ini.setValue("set/outfileena7", ui->cBOutputFileEnable7->isChecked());
    ini.setValue("set/outfileena8", ui->cBOutputFileEnable8->isChecked());
    ini.setValue("set/outfileena9", ui->cBOutputFileEnable9->isChecked());
    ini.setValue("set/format", ui->cBFormat->currentIndex());
    ini.setValue("set/cmdpostexe", commandPostExe);

    writeHistory(&ini, "hist/inputfile", ui->cBInputFile);

    viewer->saveOptions(ini);
};
//---------------------------------------------------------------------------
