//---------------------------------------------------------------------------
// strsvr_qt : stream server
//
//          Copyright (C) 2007-2012 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann
//
// options : strsvr_qt [-t title][-i file][-auto][-tray]
//
//           -t title   window title
//           -i file    ini file path
//           -auto      auto start
//           -tray      start as task tray icon
//
// version : $Revision:$ $Date:$
// history : 2008/04/03  1.1 rtklib 2.3.1
//           2010/07/18  1.2 rtklib 2.4.0
//           2011/06/10  1.3 rtklib 2.4.1
//           2012/12/15  1.4 rtklib 2.4.2
//                       add stream conversion function
//                       add option -auto and -tray
//---------------------------------------------------------------------------
#include <clocale>

#include <QTimer>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QSystemTrayIcon>
#include <QMessageBox>
#include <QSettings>
#include <QMenu>
#include <QStringList>
#include <QDir>
#include <QLocale>
#include <QTime>

#include "rtklib.h"
#include "svroptdlg.h"
#include "serioptdlg.h"
#include "fileoptdlg.h"
#include "tcpoptdlg.h"
#include "cmdoptdlg.h"
#include "convdlg.h"
#include "aboutdlg.h"
#include "mondlg.h"
#include "svrmain.h"
#include "helper.h"

#include "ui_svrmain.h"

//---------------------------------------------------------------------------

#define PRGNAME     "StrSvr-Qt"        // program name
#define TRACEFILE   "strsvr.trace"     // debug trace file

strsvr_t strsvr;

extern "C" {
extern int showmsg(const char *, ...)  {return 0;}
extern void settime(gtime_t) {}
extern void settspan(gtime_t, gtime_t) {}
}


// constructor --------------------------------------------------------------
MainForm::MainForm(QWidget *parent)
    : QDialog(parent), ui(new Ui::MainForm)
{
    ui->setupUi(this);

    QCoreApplication::setApplicationName(PRGNAME);
    QCoreApplication::setApplicationVersion(QString(VER_RTKLIB) + " " + PATCH_LEVEL);

    setWindowIcon(QIcon(":/icons/strsvr"));
    setWindowTitle(QStringLiteral("%1 Ver. %2 %3").arg(PRGNAME).arg(VER_RTKLIB, PATCH_LEVEL));

    setlocale(LC_NUMERIC, "C"); // use point as decimal separator in formatted output

    QString file = QApplication::applicationFilePath();
    QFileInfo fi(file);
    iniFile = fi.absoluteDir().filePath(fi.baseName() + ".ini");

    trayIcon = new QSystemTrayIcon(QIcon(":/icons/strsvr_Icon"));

    // setup tray menu
    QMenu *trayMenu = new QMenu(this);
    trayMenu->addAction(ui->acMenuExpand);
    trayMenu->addSeparator();
    trayMenu->addAction(ui->acMenuStart);
    trayMenu->addAction(ui->acMenuStop);
    trayMenu->addSeparator();
    trayMenu->addAction(ui->acMenuExit);
    trayIcon->setContextMenu(trayMenu);

    // setup dialos
    svrOptDialog = new SvrOptDialog(this);
    tcpOptDialog = new TcpOptDialog(this);
    serialOptDialog = new SerialOptDialog(this);
    fileOptDialog = new FileOptDialog(this);
    strMonDialog = new StrMonDialog(this);

    startTime.sec = startTime.time = endTime.sec = endTime.time = 0;

    connect(ui->btnExit, &QPushButton::clicked, this, &MainForm::close);
    connect(ui->btnInputOptions, &QPushButton::clicked, this, &MainForm::showInputOptions);
    connect(ui->btnStart, &QPushButton::clicked, this, &MainForm::startServer);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainForm::stopServer);
    connect(ui->btnOptions, &QPushButton::clicked, this, &MainForm::showOptionsDialog);
    connect(ui->btnAbout, &QPushButton::clicked, this, &MainForm::showAboutDialog);
    connect(ui->btnStreamMonitor, &QPushButton::clicked, this, &MainForm::showMonitorDialog);
    connect(ui->btnCmd, &QPushButton::clicked, this, &MainForm::showStreamCommandDialog);
    connect(ui->btnCmd1, &QPushButton::clicked, this, &MainForm::showStreamCommandDialog);
    connect(ui->btnCmd2, &QPushButton::clicked, this, &MainForm::showStreamCommandDialog);
    connect(ui->btnCmd3, &QPushButton::clicked, this, &MainForm::showStreamCommandDialog);
    connect(ui->btnCmd4, &QPushButton::clicked, this, &MainForm::showStreamCommandDialog);
    connect(ui->btnCmd5, &QPushButton::clicked, this, &MainForm::showStreamCommandDialog);
    connect(ui->btnCmd6, &QPushButton::clicked, this, &MainForm::showStreamCommandDialog);
    connect(ui->btnOutputOptions1, &QPushButton::clicked, this, &MainForm::showStreamOptionsDialog);
    connect(ui->btnOutputOptions2, &QPushButton::clicked, this, &MainForm::showStreamOptionsDialog);
    connect(ui->btnOutputOptions3, &QPushButton::clicked, this, &MainForm::showStreamOptionsDialog);
    connect(ui->btnOutputOptions4, &QPushButton::clicked, this, &MainForm::showStreamOptionsDialog);
    connect(ui->btnOutputOptions5, &QPushButton::clicked, this, &MainForm::showStreamOptionsDialog);
    connect(ui->btnOutputOptions6, &QPushButton::clicked, this, &MainForm::showStreamOptionsDialog);
    connect(ui->cBInput, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(ui->cBOutput1, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(ui->cBOutput2, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(ui->cBOutput3, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(ui->cBOutput4, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(ui->cBOutput5, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(ui->cBOutput6, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MainForm::updateEnable);
    connect(ui->btnConv1, &QPushButton::clicked, this, &MainForm::showConvertDialog);
    connect(ui->btnConv2, &QPushButton::clicked, this, &MainForm::showConvertDialog);
    connect(ui->btnConv3, &QPushButton::clicked, this, &MainForm::showConvertDialog);
    connect(ui->btnConv4, &QPushButton::clicked, this, &MainForm::showConvertDialog);
    connect(ui->btnConv5, &QPushButton::clicked, this, &MainForm::showConvertDialog);
    connect(ui->btnConv6, &QPushButton::clicked, this, &MainForm::showConvertDialog);
    connect(ui->btnLog, &QPushButton::clicked, this, &MainForm::showLogStreamDialog);
    connect(ui->btnLog1, &QPushButton::clicked, this, &MainForm::showLogStreamDialog);
    connect(ui->btnLog2, &QPushButton::clicked, this, &MainForm::showLogStreamDialog);
    connect(ui->btnLog3, &QPushButton::clicked, this, &MainForm::showLogStreamDialog);
    connect(ui->btnLog4, &QPushButton::clicked, this, &MainForm::showLogStreamDialog);
    connect(ui->btnLog5, &QPushButton::clicked, this, &MainForm::showLogStreamDialog);
    connect(ui->btnLog6, &QPushButton::clicked, this, &MainForm::showLogStreamDialog);
    connect(ui->acMenuStart, &QAction::triggered, this, &MainForm::startServer);
    connect(ui->acMenuStop, &QAction::triggered, this, &MainForm::stopServer);
    connect(ui->acMenuExit, &QAction::triggered, this, &MainForm::close);
    connect(ui->acMenuExpand, &QAction::triggered, this, &MainForm::expandFromTaskTray);
    connect(ui->btnTaskIcon, &QPushButton::clicked, this, &MainForm::minimizeToTaskTray);
    connect(trayIcon, &QSystemTrayIcon::activated, this, &MainForm::restoreFromTaskTray);
    connect(&serverStatusTimer, &QTimer::timeout, this, &MainForm::updateServerStat);
    connect(&streamMonitorTimer, &QTimer::timeout, this, &MainForm::updateStreamMonitor);

    ui->btnStop->setVisible(false);

    serverStatusTimer.setInterval(50);
    streamMonitorTimer.setInterval(100);
}
// callback on form create --------------------------------------------------
void MainForm::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    strsvrinit(&strsvr, MAXSTR-1);

    QCommandLineParser parser;
    parser.setApplicationDescription(tr("Stream Server Qt"));
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("source", QCoreApplication::translate("main", "Source file to copy."));
    parser.addPositionalArgument("destination", QCoreApplication::translate("main", "Destination directory."));

    // A boolean option with a single name (-p)
    QCommandLineOption showProgressOption("p", QCoreApplication::translate("main", "Show progress during copy."));
    parser.addOption(showProgressOption);

    QCommandLineOption trayOption(QStringList() << "tray",
                                  QCoreApplication::translate("main", "Start as task tray icon."));
    parser.addOption(trayOption);

    QCommandLineOption autoOption(QStringList() << "auto",
                                  QCoreApplication::translate("main", "Auto start."));
    parser.addOption(autoOption);

    QCommandLineOption windowTitleOption(QStringList() << "t",
                                         QCoreApplication::translate("main", "Window title."),
                                         QCoreApplication::translate("main", "title"));
    parser.addOption(windowTitleOption);

    QCommandLineOption iniFileOption(QStringList() << "i",
                                     QCoreApplication::translate("main", "ini file path."),
                                     QCoreApplication::translate("main", "file"));
    parser.addOption(iniFileOption);

    parser.process(*QApplication::instance());

    if (parser.isSet(iniFileOption))
        iniFile = parser.value(iniFileOption);

    loadOptions();
    setTrayIcon(0);

    if (parser.isSet(windowTitleOption))
        setWindowTitle(parser.value(windowTitleOption));

    if (parser.isSet(trayOption)) {
        setVisible(false);
        trayIcon->setVisible(true);
    }

    if (parser.isSet(autoOption))
        startServer();

    serverStatusTimer.start();
    streamMonitorTimer.start();
}
// callback on form close ---------------------------------------------------
void MainForm::closeEvent(QCloseEvent *)
{
    saveOptions();
}
// callback on button-options -----------------------------------------------
void MainForm::showOptionsDialog()
{
    svrOptDialog->exec();
    if (svrOptDialog->result() != QDialog::Accepted) return;
}
// callback on button-input-opt ---------------------------------------------
void MainForm::showInputOptions()
{
    switch (ui->cBInput->currentIndex()) {
        case 0: serialOptions(0, 0); break;
        case 1: tcpClientOptions(0, 1); break; // TCP Client
        case 2: tcpServerOptions(0, 2); break; // TCP Server
        case 3: ntripClientOptions(0, 3); break; // Ntrip Client
        case 4: udpServerOptions(0, 4); break;  // UDP Server
        case 5: fileOptions(0, 5); break;
    }
}
// callback on button-input-cmd ---------------------------------------------
void MainForm::showStreamCommandDialog()
{
    QPushButton *btn[MAXSTR] = { ui->btnCmd, ui->btnCmd1, ui->btnCmd2, ui->btnCmd3, ui->btnCmd4, ui->btnCmd5, ui->btnCmd6};
    QComboBox *type[MAXSTR] = { ui->cBInput, ui->cBOutput1, ui->cBOutput2, ui->cBOutput3, ui->cBOutput4, ui->cBOutput5, ui->cBOutput6};
    int stream = -1;

    for (int i = 0; i < MAXSTR; i++)
        if (btn[i] == qobject_cast<QPushButton *>(sender()))
        {
            stream = i;
            break;
        }
    if (stream == -1) return;

    CmdOptDialog *cmdOptDialog = new CmdOptDialog(this);

    for (int j = 0; j < 3; j++) {
        if (type[stream]->currentText() == tr("Serial")) {
            cmdOptDialog->setCommands(j, commands[stream][j]);
            cmdOptDialog->setCommandsEnabled(j, commandsEnabled[stream][j]);
        }
        else {
            cmdOptDialog->setCommands(j, commandsTcp[stream][j]);
            cmdOptDialog->setCommandsEnabled(j, commandsEnabledTcp[stream][j]);
        }
    }
    if (stream == 0)
        cmdOptDialog->setWindowTitle(tr("Input Serial/TCP Commands"));
    else
        cmdOptDialog->setWindowTitle(tr("Output%1 Serial/TCP Commands").arg(stream));

    cmdOptDialog->exec();
    if (cmdOptDialog->result() != QDialog::Accepted) {
        delete cmdOptDialog;
        return;
    }

    for (int j = 0; j < 3; j++) {
        if (type[stream]->currentText() == tr("Serial")) {
            commands[stream][j] = cmdOptDialog->getCommands(j);
            commandsEnabled[stream][j] = cmdOptDialog->getCommandsEnabled(j);
        }
        else {
            commandsTcp[stream][j] = cmdOptDialog->getCommands(j);
            commandsEnabledTcp[stream][j] = cmdOptDialog->getCommandsEnabled(j);
        }
    }

    delete cmdOptDialog;
}
// callback on button-output1-opt -------------------------------------------
void MainForm::showStreamOptionsDialog()
{
    QPushButton *btn[MAXSTR - 1] = {ui->btnOutputOptions1, ui->btnOutputOptions2, ui->btnOutputOptions3, ui->btnOutputOptions4, ui->btnOutputOptions5, ui->btnOutputOptions6};
    QComboBox *type[MAXSTR - 1] = {ui->cBOutput1, ui->cBOutput2, ui->cBOutput3, ui->cBOutput4, ui->cBOutput5, ui->cBOutput6};
    int stream = -1;

    for (int i = 0; i < MAXSTR - 1; i++) {
        if ((QPushButton *)sender() == btn[i])
        {
            stream = i;
            break;
        }
    }
    if (stream == -1) return;

    switch (type[stream]->currentIndex()) {
        case 1: serialOptions(stream + 1, 0); break;
        case 2: tcpClientOptions(stream + 1, 1); break;
        case 3: tcpServerOptions(stream + 1, 2); break;
        case 4: ntripServerOptions(stream + 1, 3); break;
        case 5: ntripCasterOptions(stream + 1, 4); break;
        case 6: udpClientOptions(stream + 1, 5); break;
        case 7: fileOptions(stream + 1, 6); break;
    }
}
// callback on button-output-conv ------------------------------------------
void MainForm::showConvertDialog()
{
    QPushButton *btn[MAXSTR - 1] = {ui->btnConv1, ui->btnConv2, ui->btnConv3, ui->btnConv4, ui->btnConv5, ui->btnConv6};
    int stream = -1;

    for (int i = 0; i < MAXSTR - 1; i++) {
        if ((QPushButton *)sender() == btn[i])
        {
            stream = i;
            break;
        }
    }
    if (stream == -1) return;

    ConvDialog *convDialog = new ConvDialog(this);

    convDialog->setConversionEnabled(conversionEnabled[stream]);
    convDialog->setInputFormat(conversionInputFormat[stream]);
    convDialog->setOutputFormat(conversionOutputFormat[stream]);
    convDialog->setConversionMessage(conversionMessage[stream]);
    convDialog->setConversionOptions(conversionOptions[stream]);
    convDialog->setWindowTitle(tr("Output %1: Conversion Options").arg(stream+1));

    convDialog->exec();
    if (convDialog->result() != QDialog::Accepted) {
        delete convDialog;
        return;
    }

    conversionEnabled[stream] = convDialog->getConversionEnabled();
    conversionInputFormat[stream] = convDialog->getInputFormat();
    conversionOutputFormat[stream] = convDialog->getOutputFormat();
    conversionMessage[stream] = convDialog->getConversionMessage();
    conversionOptions[stream] = convDialog->getConversionOptions();

    delete convDialog;
}
// callback on buttn-about --------------------------------------------------
void MainForm::showAboutDialog()
{
    AboutDialog *aboutDialog = new AboutDialog(this, QPixmap(":/icons/strsvr"), PRGNAME);

    aboutDialog->exec();

    delete aboutDialog;
}
// callback on task-icon ----------------------------------------------------
void MainForm::minimizeToTaskTray()
{
    setVisible(false);
    trayIcon->setVisible(true);
}
// callback on task-icon double-click ---------------------------------------
void MainForm::restoreFromTaskTray(QSystemTrayIcon::ActivationReason reason)
{
    if (reason != QSystemTrayIcon::DoubleClick) return;

    setVisible(true);
    trayIcon->setVisible(false);
}
// callback on menu-expand --------------------------------------------------
void MainForm::expandFromTaskTray()
{
    setVisible(true);
    trayIcon->setVisible(false);
}
// callback on stream-monitor -----------------------------------------------
void MainForm::showMonitorDialog()
{
    strMonDialog->show();
}
// callback on interval timer -----------------------------------------------
void MainForm::updateServerStat()
{
    QColor color[] = {Qt::red, Qt::white, Color::Orange, Qt::darkGreen, Qt::green, Color::Aqua};
    QLabel *lblStatus[MAXSTR] = {ui->indInput, ui->indOutput1, ui->indOutput2, ui->indOutput3, ui->indOutput4, ui->indOutput5, ui->indOutput6};
    QLabel *lblBytes[MAXSTR] = {ui->lblInputByte, ui->lblOutput1Byte, ui->lblOutput2Byte, ui->lblOutput3Byte, ui->lblOutput4Byte, ui->lblOutput5Byte, ui->lblOutput6Byte};
    QLabel *lblBps[MAXSTR] = {ui->lblInputBps, ui->lblOutput1Bps, ui->lblOutput2Bps, ui->lblOutput3Bps, ui->lblOutput4Bps, ui->lblOutput5Bps, ui->lblOutput6Bps};
    QLabel *lblLog[MAXSTR] = {ui->indLog, ui->indLog1, ui->indLog2, ui->indLog3, ui->indLog4, ui->indLog5, ui->indLog6};
    static const QString statusStr[] = {tr("Error"), tr("Closed"), tr("Waiting..."), tr("Connected"), tr("Active")};
    gtime_t time = utc2gpst(timeget());
    int stat[MAXSTR] = {0}, byte[MAXSTR] = {0}, bps[MAXSTR] = {0}, log_stat[MAXSTR] = {0};
    char msg[MAXSTRMSG * MAXSTR] = "", str[256];
    double ctime, t[4], pos;

    strsvrstat(&strsvr, stat, log_stat, byte, bps, msg);
    // update status indicators
    for (int i = 0; i < MAXSTR; i++) {
        lblStatus[i]->setStyleSheet(QStringLiteral("QLabel {background-color: %1;}").arg(color2String(color[stat[i] + 1])));
        lblStatus[i]->setToolTip(statusStr[stat[i]+1]);
        lblBytes[i]->setText(QLocale::system().toString(byte[i]));
        lblBps[i]->setText(QLocale::system().toString(bps[i]));
        lblLog[i]->setStyleSheet(QStringLiteral("QLabel {background-color :%1;}").arg(color2String(color[log_stat[i]+1])));
        lblLog[i]->setToolTip(statusStr[log_stat[i]+1]);
    }
    // update progress bar
    pos = fmod(byte[0] / 1e3 / qMax(svrOptDialog->progressBarRange, 1), 1.0) * 110.0;
    ui->pBProgress->setValue(!stat[0] ? 0 : qMin((int)pos, 100));

    // update time
    time2str(time, str, 0);
    ui->lblTime->setText(tr("%1 GPST").arg(str));

    if (ui->panelStreams->isEnabled())
        ctime = timediff(endTime, startTime);
    else  // server is running
        ctime = timediff(time, startTime);
    ctime = floor(ctime);
    t[0] = floor(ctime / 86400.0); ctime -= t[0] * 86400.0;
    t[1] = floor(ctime / 3600.0); ctime -= t[1] * 3600.0;
    t[2] = floor(ctime / 60.0); ctime -= t[2] * 60.0;
    t[3] = ctime;
    ui->lblCurrentConnectionTime->setText(QStringLiteral("%1d %2").arg(t[0], 0, 'f', 0).arg(QLocale().toString(QTime(t[1],t[2], t[3]), "hh:mm:ss")));

    // update task tray icon
    trayIcon->setToolTip(tr("%1, %2 bps").arg(QLocale().formattedDataSize(byte[0]), QLocale().toString(bps[0])));
    setTrayIcon(stat[0] <= 0 ? 0 : (stat[0] == 3 ? 2 : 1));

    ui->lblMessage->setText(QString(msg).trimmed());
    ui->lblMessage->setToolTip(QString(msg).trimmed());
}
// start stream server ------------------------------------------------------
void MainForm::startServer()
{
    QComboBox *type[MAXSTR] = {ui->cBInput, ui->cBOutput1, ui->cBOutput2, ui->cBOutput3, ui->cBOutput4, ui->cBOutput5, ui->cBOutput6};
    strconv_t *conv[MAXSTR - 1] = {0};
    static char str1[MAXSTR][1024], str2[MAXSTR][1024];
    int inputTypes[] = {
        STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPCLI, STR_UDPSVR,
        STR_FILE, STR_FTP, STR_HTTP
    };
    int outputTypes[] = {
        STR_NONE, STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPSVR, STR_NTRIPCAS,
        STR_UDPCLI, STR_FILE
    };
    int streamTypes[MAXSTR] = {0}, opt[8] = {0};
    char *pths[MAXSTR], *logs[MAXSTR], *cmds[MAXSTR] = {0}, *cmds_periodic[MAXSTR] = {0};
    char filepath[1024];
    char *p;
    QStringList tokens;
    bool error = false;

    if (svrOptDialog->traceLevel > 0) {
        traceopen(svrOptDialog->logFile.isEmpty() ? TRACEFILE : qPrintable(svrOptDialog->logFile)); // use default if no log file was specified
        tracelevel(svrOptDialog->traceLevel);
    }
    for (int i = 0; i < MAXSTR; i++) {
        pths[i] = str1[i];
        logs[i] = str2[i];
    }

    // input stream
    streamTypes[0] = inputTypes[type[0]->currentIndex()];
    strncpy(pths[0], qPrintable(paths[0][type[0]->currentIndex()]), 1023);
    strncpy(logs[0], type[0]->currentIndex() > 5 || !pathEnabled[0] ? "" : qPrintable(pathLog[0]), 1023);

    for (int i = 1; i < MAXSTR; i++) {  // all output streams
        streamTypes[i] = outputTypes[type[i]->currentIndex()];
        strncpy(pths[i], type[i]->currentIndex() == 0 ? "" : qPrintable(paths[i][type[i]->currentIndex() - 1]), 1023);
        strncpy(logs[i], !pathEnabled[i] ? "" : qPrintable(pathLog[i]), 1023);
    }

    // get start comannds and period commands
    for (int i = 0; i < MAXSTR; i++) {
        cmds[i] = cmds_periodic[i] = NULL;
        if (streamTypes[i] == STR_SERIAL) {
            cmds[i] = new char[1024];
            cmds[i][0] = '\0';
            cmds_periodic[i] = new char[1024];
            cmds_periodic[i][0] = '\0';
            if (commandsEnabled[i][0]) strncpy(cmds[i], qPrintable(commands[i][0]), 1023);
            if (commandsEnabled[i][2]) strncpy(cmds_periodic[i], qPrintable(commands[i][2]), 1023);
        } else if (streamTypes[i] == STR_TCPCLI || streamTypes[i] == STR_NTRIPCLI || streamTypes[i] == STR_TCPSVR) {
            cmds[i] = new char[1024];
            cmds[i][0] = '\0';
            cmds_periodic[i] = new char[1024];
            cmds_periodic[i][0] = '\0';
            if (commandsEnabledTcp[i][0]) strncpy(cmds[i], qPrintable(commandsTcp[i][0]), 1023);
            if (commandsEnabledTcp[i][2]) strncpy(cmds_periodic[i], qPrintable(commandsTcp[i][2]), 1023);
        }
    }

    // fill server options
    for (int i = 0; i < 5; i++) {
        opt[i] = svrOptDialog->serverOptions[i];
    }
    opt[5] = svrOptDialog->nmeaRequest ? svrOptDialog->serverOptions[5] : 0; // set NMEA cycle time
    opt[6] = svrOptDialog->fileSwapMargin;
    opt[7] = svrOptDialog->relayBack;

    // check for file overwrite
    for (int i = 1; i < MAXSTR; i++) { // for each out stream
        if (streamTypes[i] != STR_FILE) continue;
        strncpy(filepath, pths[i], 1023);
        if (strstr(filepath, "::A")) continue;
        if ((p = strstr(filepath, "::"))) *p = '\0';
        if (!QFile::exists(filepath)) continue;
        if (QMessageBox::question(this, tr("Overwrite"), tr("File %1 exists.\nDo you want to overwrite?").arg(filepath)) != QMessageBox::Yes)
            error = true;
    }
    for (int i = 0; i < MAXSTR; i++) { // for each log stream
        if (!*logs[i]) continue;
        strncpy(filepath, logs[i], 1023);
        if (strstr(filepath, "::A")) continue;
        if ((p  =strstr(filepath, "::"))) *p = '\0';
        if (!QFile::exists(filepath)) continue;
        if (QMessageBox::question(this, tr("Overwrite"), tr("File %1 exists.\nDo you want to overwrite?").arg(filepath)) != QMessageBox::Yes)
            error = true;
    }

    strsetdir(qPrintable(svrOptDialog->localDirectory));
    strsetproxy(qPrintable(svrOptDialog->proxyAddress));

    // set up conversion if neccessary
    for (int i = 0; i < MAXSTR - 1; i++) { // for each output stream
        if (ui->cBInput->currentIndex() == 2 || ui->cBInput->currentIndex() == 4) continue;  // TCP/UDP server
        if (!conversionEnabled[i]) continue;
        if (!(conv[i] = strconvnew(conversionInputFormat[i], conversionOutputFormat[i], qPrintable(conversionMessage[i]),
                                   svrOptDialog->stationId, svrOptDialog->stationSelect, qPrintable(conversionOptions[i])))) continue;

        tokens = svrOptDialog->antennaType.split(',');
        if (tokens.size() >= 3)
        {
            strncpy(conv[i]->out.sta.antdes, qPrintable(tokens.at(0)), 63);
            strncpy(conv[i]->out.sta.antsno, qPrintable(tokens.at(1)), 63);
            conv[i]->out.sta.antsetup = tokens.at(2).toInt();
        }
        tokens = svrOptDialog->receiverType.split(',');
        if (tokens.size() >= 3)
        {
            strncpy(conv[i]->out.sta.rectype, qPrintable(tokens.at(0)), 63);
            strncpy(conv[i]->out.sta.recver, qPrintable(tokens.at(1)), 63);
            strncpy(conv[i]->out.sta.recsno, qPrintable(tokens.at(2)), 63);
        }
        matcpy(conv[i]->out.sta.pos, svrOptDialog->antennaPosition, 3, 1);
        matcpy(conv[i]->out.sta.del, svrOptDialog->antennaOffsets, 3, 1);
    }

    // stream server start (if no error in preparation occured)
    if (!error && strsvrstart(&strsvr, opt, streamTypes, (const char **)pths, (const char **)logs, conv, (const char **)cmds, (const char **)cmds_periodic, svrOptDialog->antennaPosition)) {
        startTime = utc2gpst(timeget());
        ui->panelStreams->setEnabled(false);
        ui->btnStart->setVisible(false);
        ui->btnStop->setVisible(true);
        ui->btnOptions->setEnabled(false);
        ui->btnExit->setEnabled(false);
        ui->acMenuStart->setEnabled(false);
        ui->acMenuStop->setEnabled(true);
        ui->acMenuExit->setEnabled(false);

        setTrayIcon(1);
    }

    // clean up
    for (int i = 0; i < 4; i++) {
        if (cmds[i]) delete[] cmds[i];
        if (cmds_periodic[i]) delete[] cmds_periodic[i];
    };

}
// stop stream server -------------------------------------------------------
void MainForm::stopServer()
{
    char *cmds[MAXSTR];
    QComboBox *type[] = {ui->cBInput, ui->cBOutput1, ui->cBOutput2, ui->cBOutput3, ui->cBOutput4, ui->cBOutput5, ui->cBOutput6};
    const int inputTypes[] = {STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPCLI, STR_UDPSVR, STR_FILE, STR_FTP, STR_HTTP};
    const int outputTypes[] = {
        STR_NONE, STR_SERIAL, STR_TCPCLI, STR_TCPSVR, STR_NTRIPSVR, STR_NTRIPCAS, STR_UDPCLI, STR_FILE
    };
    int streamTypes[MAXSTR];

    streamTypes[0] = inputTypes[ui->cBInput->currentIndex()];
    for (int i = 1; i < MAXSTR; i++) {
        streamTypes[1] = outputTypes[type[i]->currentIndex()];
    }

    // get stop commands
    for (int i = 0; i < MAXSTR; i++) {
        cmds[i] = NULL;
        if (streamTypes[i] == STR_SERIAL) {
            cmds[i] = new char[1024];
            if (commandsEnabled[i][1]) strncpy(cmds[i], qPrintable(commands[i][1]), 1023);
        } else if (streamTypes[i] == STR_TCPCLI || streamTypes[i] == STR_NTRIPCLI || streamTypes[i] == STR_TCPSVR) {
            cmds[i] = new char[1024];
            if (commandsEnabledTcp[i][1]) strncpy(cmds[i], qPrintable(commandsTcp[i][1]), 1023);
        }
    }
    strsvrstop(&strsvr, (const char **)cmds);

    endTime = utc2gpst(timeget());
    ui->panelStreams->setEnabled(true);
    ui->btnStart->setVisible(true);
    ui->btnStop->setVisible(false);
    ui->btnOptions->setEnabled(true);
    ui->btnExit->setEnabled(true);
    ui->acMenuStart->setEnabled(true);
    ui->acMenuStop->setEnabled(false);
    ui->acMenuExit->setEnabled(true);

    setTrayIcon(0);

    for (int i = 0; i < MAXSTR; i++)
        if (cmds[i]) delete[] cmds[i];
    for (int i = 0; i < MAXSTR - 1; i++)
        if (conversionEnabled[i]) strconvfree(strsvr.conv[i]);
    if (svrOptDialog->traceLevel > 0) traceclose();
}
// callback on interval timer for stream monitor ----------------------------
void MainForm::updateStreamMonitor()
{
    static const QString types[] = {
        tr("None"), tr("Serial"), tr("File"), tr("TCP Server"), tr("TCP Client"), tr("Ntrip Sever"),
        tr("Ntrip Client"), tr("FTP"), tr("HTTP"), tr("Ntrip Caster"), tr("UDP Server"),
        tr("UDP Client"), tr("Mem Buffer")
    };
    unsigned char *msg = 0;
    char *p;
    int i, len, inb, inr, outb, outr;

    if (strMonDialog->getStreamFormat()) {
        rtklib_lock(&strsvr.lock);
        len = strsvr.npb;
        if (len > 0 && (msg = (unsigned char *)malloc(len))) {
            memcpy(msg, strsvr.pbuf, len);
            strsvr.npb = 0;
        }
        rtklib_unlock(&strsvr.lock);
        if (len <= 0 || !msg) return;
        strMonDialog->addMessage(msg, len);
        free(msg);
    } else {
        if (!(msg = (unsigned char *)malloc(16000))) return;

        for (i = 0, p = (char*)msg; i < MAXSTR; i++) {
            p += sprintf(p, "[STREAM %d]\n", i);
            strsum(strsvr.stream + i, &inb, &inr, &outb, &outr);
            strstatx(strsvr.stream + i, p);
            p += strlen(p);
            if (inb > 0) {
                p += sprintf(p,"  inb     = %d\n", inb);
                p += sprintf(p,"  inr     = %d\n", inr);
            }
            if (outb > 0) {
                p += sprintf(p,"  outb    = %d\n", outb);
                p += sprintf(p,"  outr    = %d\n", outr);
            }
        }
        strMonDialog->addMessage(msg, strlen((char*)msg));

        free(msg);
    }
}
// set serial options -------------------------------------------------------
void MainForm::serialOptions(int index, int path)
{
    serialOptDialog->setOptions((index == 0) ? 0 : 1);
    serialOptDialog->setPath(paths[index][path]);

    serialOptDialog->exec();
    if (serialOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = serialOptDialog->getPath();
}
// set tcp server options -------------------------------------------------------
void MainForm::tcpServerOptions(int index, int path)
{
    tcpOptDialog->setOptions(TcpOptDialog::OPT_TCP_SERVER);  // 0: TCP Server
    tcpOptDialog->setPath(paths[index][path]);

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = tcpOptDialog->getPath();
}
// set tcp client options ---------------------------------------------------
void MainForm::tcpClientOptions(int index, int path)
{
    tcpOptDialog->setOptions(TcpOptDialog::OPT_TCP_CLIENT);  // 1: TCP Client
    tcpOptDialog->setHistory(tcpHistory, MAXHIST);
    tcpOptDialog->setPath(paths[index][path]);

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = tcpOptDialog->getPath();
    for (int i = 0; i < MAXHIST; i++)
        tcpHistory[i] = tcpOptDialog->getHistory()[i];
}
// set ntrip server options ---------------------------------------------------------
void MainForm::ntripServerOptions(int index, int path)
{
    tcpOptDialog->setOptions(TcpOptDialog::OPT_NTRIP_SERVER);  // 2: Ntrip Server
    tcpOptDialog->setHistory(tcpHistory, MAXHIST);
    tcpOptDialog->setPath(paths[index][path]);

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = tcpOptDialog->getPath();
    for (int i = 0; i < MAXHIST; i++)
        tcpHistory[i] = tcpOptDialog->getHistory()[i];
}
// set ntrip client options ---------------------------------------------------------
void MainForm::ntripClientOptions(int index, int path)
{
    tcpOptDialog->setOptions(TcpOptDialog::OPT_NTRIP_CLIENT);  // Ntrip Client
    tcpOptDialog->setHistory(tcpHistory, MAXHIST);
    tcpOptDialog->setPath(paths[index][path]);

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = tcpOptDialog->getPath();
    for (int i = 0; i < MAXHIST; i++)
        tcpHistory[i] = tcpOptDialog->getHistory()[i];
}
// set ntrip caster options ---------------------------------------------------------
void MainForm::ntripCasterOptions(int index, int path)
{
    tcpOptDialog->setOptions(TcpOptDialog::OPT_NTRIP_CASTER_CLIENT);  // Ntrip caster
    tcpOptDialog->setPath(paths[index][path]);

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = tcpOptDialog->getPath();
}
// set udp server options ---------------------------------------------------------
void MainForm::udpServerOptions(int index, int path)
{
    tcpOptDialog->setOptions(TcpOptDialog::OPT_UDP_SERVER);  // UDP Server
    tcpOptDialog->setPath(paths[index][path]);

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = tcpOptDialog->getPath();
}
// set udp client options ---------------------------------------------------------
void MainForm::udpClientOptions(int index, int path)
{
    tcpOptDialog->setPath(paths[index][path]);
    tcpOptDialog->setOptions(7);  // UDP Client

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = tcpOptDialog->getPath();
}
// set file options ---------------------------------------------------------
void MainForm::fileOptions(int index, int path)
{
    fileOptDialog->setOptions((index == 0) ? 0 : 1);
    fileOptDialog->setPath(paths[index][path]);
    fileOptDialog->setWindowTitle(tr("File Options"));

    fileOptDialog->exec();
    if (fileOptDialog->result() != QDialog::Accepted) return;

    paths[index][path] = fileOptDialog->getPath();
}
// undate enable of widgets -------------------------------------------------
void MainForm::updateEnable()
{
    QComboBox *type[MAXSTR - 1] = {ui->cBOutput1, ui->cBOutput2, ui->cBOutput3, ui->cBOutput4, ui->cBOutput5, ui->cBOutput6};
    QLabel *lblOutput[MAXSTR - 1] = {ui->lblOutput1, ui->lblOutput2, ui->lblOutput3, ui->lblOutput4, ui->lblOutput5, ui->lblOutput6};
    QLabel *lblOutputByte[MAXSTR - 1] = {ui->lblOutput1Byte, ui->lblOutput2Byte, ui->lblOutput3Byte, ui->lblOutput4Byte, ui->lblOutput5Byte, ui->lblOutput6Byte};
    QLabel *lblOutputBps[MAXSTR - 1] = {ui->lblOutput1Bps, ui->lblOutput2Bps, ui->lblOutput3Bps, ui->lblOutput4Bps, ui->lblOutput5Bps, ui->lblOutput6Bps};
    QPushButton *btnOutput[MAXSTR - 1] = {ui->btnOutputOptions1, ui->btnOutputOptions2, ui->btnOutputOptions3, ui->btnOutputOptions4, ui->btnOutputOptions5, ui->btnOutputOptions6};
    QPushButton *btnCmd[MAXSTR - 1] = {ui->btnCmd1, ui->btnCmd2, ui->btnCmd3, ui->btnCmd4, ui->btnCmd5, ui->btnCmd6};
    QPushButton *btnConv[MAXSTR - 1] = {ui->btnConv1, ui->btnConv2, ui->btnConv3, ui->btnConv4, ui->btnConv5, ui->btnConv6};
    QPushButton *btnLog[MAXSTR - 1] = {ui->btnLog1, ui->btnLog2, ui->btnLog3, ui->btnLog4, ui->btnLog5, ui->btnLog6};

    ui->btnCmd->setEnabled(ui->cBInput->currentIndex() <= 3);

    for (int i = 0; i < MAXSTR - 1; i++) {
        lblOutput[i]->setEnabled(type[i]->currentIndex() > 0);
        lblOutputByte[i]->setEnabled(type[i]->currentIndex() > 0);
        lblOutputBps[i]->setEnabled(type[i]->currentIndex() > 0);
        btnOutput[i]->setEnabled(type[i]->currentIndex() > 0);
        btnCmd[i]->setEnabled(btnOutput[i]->isEnabled() && (type[i]->currentIndex() == 1 || type[i]->currentIndex() == 2));
        btnConv[i]->setEnabled(btnOutput[i]->isEnabled() && ui->cBInput->currentIndex() != 2 && ui->cBInput->currentIndex() != 4);
        btnLog[i]->setEnabled(btnOutput[i]->isEnabled() && (type[i]->currentIndex() == 1 || type[i]->currentIndex() == 2));
    }
}
// set task-tray icon -------------------------------------------------------
void MainForm::setTrayIcon(int index)
{
    static const QString icon[] = {":/icons/tray0", ":/icons/tray1", ":/icons/tray2"};

    trayIcon->setIcon(QPixmap(icon[index]));
}
// load options -------------------------------------------------------------
void MainForm::loadOptions()
{
    QSettings settings(iniFile, QSettings::IniFormat);
    QComboBox *type[MAXSTR - 1] = {ui->cBOutput1, ui->cBOutput2, ui->cBOutput3, ui->cBOutput4, ui->cBOutput5, ui->cBOutput6};
    int optdef[] = {10000, 10000, 1000, 32768, 10, 0};

    // stream types
    ui->cBInput->setCurrentIndex(settings.value("set/input", 0).toInt());
    for (int i = 0; i < MAXSTR - 1; i++)
        type[i]->setCurrentIndex(settings.value(QString("set/output%1").arg(i), 0).toInt());

    // options
    svrOptDialog->traceLevel = settings.value("set/tracelevel", 0).toInt();
    svrOptDialog->nmeaRequest = settings.value("set/nmeareq", 0).toInt();
    svrOptDialog->fileSwapMargin = settings.value("set/fswapmargin", 30).toInt();
    svrOptDialog->relayBack = settings.value("set/relayback", 30).toInt();
    svrOptDialog->progressBarRange = settings.value("set/progbarrange", 30).toInt();
    svrOptDialog->stationId = settings.value("set/staid", 0).toInt();
    svrOptDialog->stationSelect = settings.value("set/stasel", 0).toInt();
    svrOptDialog->antennaType = settings.value("set/anttype", "").toString();
    svrOptDialog->receiverType = settings.value("set/rcvtype", "").toString();
    svrOptDialog->stationPositionFile = settings.value("stapos/staposfile", "").toString();
    svrOptDialog->exeDirectory = settings.value("dirs/exedirectory", "").toString();
    svrOptDialog->localDirectory = settings.value("dirs/localdirectory", "").toString();
    svrOptDialog->proxyAddress = settings.value("dirs/proxyaddress", "").toString();
    svrOptDialog->logFile = settings.value("file/logfile", "").toString();

    for (int i = 0; i < 6; i++)
        svrOptDialog->serverOptions[i] = settings.value(QString("set/svropt_%1").arg(i), optdef[i]).toInt();

    for (int i = 0; i < 3; i++) {
        svrOptDialog->antennaPosition[i] = settings.value(QString("set/antpos_%1").arg(i), 0.0).toDouble();
        svrOptDialog->antennaOffsets[i] = settings.value(QString("set/antoff_%1").arg(i), 0.0).toDouble();
    }

    // format conversion
    for (int i = 0; i < MAXSTR - 1; i++) {
        conversionEnabled[i] = settings.value(QString("conv/ena_%1").arg(i), 0).toInt();
        conversionInputFormat[i] = settings.value(QString("conv/inp_%1").arg(i), 0).toInt();
        conversionOutputFormat[i] = settings.value(QString("conv/out_%1").arg(i), 0).toInt();
        conversionMessage[i] = settings.value(QString("conv/msg_%1").arg(i), "").toString();
        conversionOptions[i] = settings.value(QString("conv/opt_%1").arg(i), "").toString();
    }

    // paths
    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 7; j++)
            paths[i][j] = settings.value(QString("path/path_%1_%2").arg(i).arg(j), "").toString();

    for (int i=0;i<MAXSTR;i++) {
        pathLog[i] = settings.value(QString("path/path_log_%1").arg(i), "").toString();
        pathEnabled[i] = settings.value(QString("path/path_ena_%1").arg(i), 0).toInt();
    }
    // start/stop commands
    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 2; j++) {
            commandsEnabled[i][j] = settings.value(QString("serial/cmdena_%1_%2").arg(i).arg(j), 1).toInt();
            commandsEnabledTcp[i][j] = settings.value(QString("tcpip/cmdena_%1_%2").arg(i).arg(j), 1).toInt();
        }
    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 2; j++) {
            commands[i][j] = settings.value(QString("serial/cmd_%1_%2").arg(i).arg(j), "").toString();
            commands[i][j] = commands[i][j].replace("@@", "\n");
        }
    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 2; j++) {
            commandsTcp[i][j] = settings.value(QString("tcpip/cmd_%1_%2").arg(i).arg(j), "").toString();
            commandsTcp[i][j] = commandsTcp[i][j].replace("@@", "\n");
        }

    // histroy
    for (int i = 0; i < MAXHIST; i++)
        tcpHistory[i] = settings.value(QString("tcpopt/history%1").arg(i), "").toString();
    for (int i = 0; i < MAXHIST; i++)
        tcpMountpointHistory[i] = settings.value(QString("tcpopt/mntphist%1").arg(i), "").toString();

    updateEnable();
}
// save options--------------------------------------------------------------
void MainForm::saveOptions()
{
    QSettings settings(iniFile, QSettings::IniFormat);
    QComboBox *type[MAXSTR - 1] = {ui->cBOutput1, ui->cBOutput2, ui->cBOutput3, ui->cBOutput4, ui->cBOutput5, ui->cBOutput6};

    // stream types
    settings.setValue("set/input", ui->cBInput->currentIndex());
    for (int i = 0; i < MAXSTR - 1; i++) {
        settings.setValue(QString("set/output%1").arg(i), type[i]->currentIndex());
    }

    // options
    settings.setValue("set/tracelevel", svrOptDialog->traceLevel);
    settings.setValue("set/nmeareq", svrOptDialog->nmeaRequest);
    settings.setValue("set/fswapmargin", svrOptDialog->fileSwapMargin);
    settings.setValue("set/relayback", svrOptDialog->relayBack);
    settings.setValue("set/progbarrange", svrOptDialog->progressBarRange);
    settings.setValue("set/staid", svrOptDialog->stationId);
    settings.setValue("set/stasel", svrOptDialog->stationSelect);
    settings.setValue("set/anttype", svrOptDialog->antennaType);
    settings.setValue("set/rcvtype", svrOptDialog->receiverType);
    settings.setValue("stapos/staposfile", svrOptDialog->stationPositionFile);
    settings.setValue("dirs/exedirectory", svrOptDialog->exeDirectory);
    settings.setValue("dirs/localdirectory", svrOptDialog->localDirectory);
    settings.setValue("dirs/proxyaddress", svrOptDialog->proxyAddress);
    settings.setValue("file/logfile",svrOptDialog->logFile);

    for (int i = 0; i < 6; i++)
        settings.setValue(QString("set/svropt_%1").arg(i), svrOptDialog->serverOptions[i]);

    for (int i = 0; i < 3; i++) {
        settings.setValue(QString("set/antpos_%1").arg(i), svrOptDialog->antennaPosition[i]);
        settings.setValue(QString("set/antoff_%1").arg(i), svrOptDialog->antennaOffsets[i]);
    }

    // format conversion
    for (int i = 0; i < MAXSTR - 1; i++) {
        settings.setValue(QString("conv/ena_%1").arg(i), conversionEnabled[i]);
        settings.setValue(QString("conv/inp_%1").arg(i), conversionInputFormat[i]);
        settings.setValue(QString("conv/out_%1").arg(i), conversionOutputFormat[i]);
        settings.setValue(QString("conv/msg_%1").arg(i), conversionMessage[i]);
        settings.setValue(QString("conv/opt_%1").arg(i), conversionOptions[i]);
    }

    // path
    for (int i=0;i<MAXSTR;i++) {
        settings.setValue(QString("path/path_log_%1").arg(i), pathLog[i]);
        settings.setValue(QString("path/path_ena_%1").arg(i), pathEnabled[i]);
    }

    // start/stop commands
    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 2; j++) {
            settings.setValue(QString("serial/cmdena_%1_%2").arg(i).arg(j), commandsEnabled[i][j]);
            settings.setValue(QString("tcpip/cmdena_%1_%2").arg(i).arg(j), commandsEnabledTcp[i][j]);
        }
    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 7; j++)
            settings.setValue(QString("path/path_%1_%2").arg(i).arg(j), paths[i][j]);

    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 2; j++) {
            commands[i][j] = commands[i][j].replace("\n", "@@");
            settings.setValue(QString("serial/cmd_%1_%2").arg(i).arg(j), commands[i][j]);
        }
    for (int i = 0; i < MAXSTR; i++)
        for (int j = 0; j < 2; j++) {
            commandsTcp[i][j] = commandsTcp[i][j].replace("\n", "@@");
            settings.setValue(QString("tcpip/cmd_%1_%2").arg(i).arg(j), commandsTcp[i][j]);
        }

    // history
    for (int i = 0; i < MAXHIST; i++)
        settings.setValue(QString("tcpopt/history%1").arg(i), tcpOptDialog->getHistory()[i]);
    for (int i = 0; i < MAXHIST; i++)
        settings.setValue(QString("tcpopt/mntphist%1").arg(i), tcpMountpointHistory[i]);
}
//---------------------------------------------------------------------------
void MainForm::showLogStreamDialog()
{
    QPushButton *btn[MAXSTR] = {ui->btnLog, ui->btnLog1, ui->btnLog2, ui->btnLog3, ui->btnLog4, ui->btnLog5, ui->btnLog6};
    int stream = -1;

    for (int i = 0; i < MAXSTR; i++) {
        if ((QPushButton *)sender() == btn[i])
        {
            stream = i;
            break;
        }
    }
    if (stream == -1) return;

    fileOptDialog->setOptions(2);
    fileOptDialog->setPath(pathLog[stream]);
    fileOptDialog->setPathEnabled(pathEnabled[stream]);
    fileOptDialog->setWindowTitle((stream == 0) ? tr("Input Log Options") : tr("Return Log Options"));

    fileOptDialog->exec();
    if (fileOptDialog->result() != QDialog::Accepted) return;

    pathLog[stream] = fileOptDialog->getPath();
    pathEnabled[stream] = fileOptDialog->getPathEnabled();
}
//---------------------------------------------------------------------------
