//---------------------------------------------------------------------------
// rtklaunch_qt : rtklib app launcher
//
//          Copyright (C) 2013-2016 by T.TAKASU, All rights reserved.
//          ported to Qt by Jens Reimann
//
// options : rtklib launcher [-t title][-tray]
//
//           -t title   window title
//           -tray      start as task tray icon
//
// version : $Revision:$ $Date:$
// history : 2013/01/10  1.1 new
//---------------------------------------------------------------------------

#include <QSettings>
#include <QFileInfo>
#include <QShowEvent>
#include <QCommandLineParser>
#include <QProcess>
#include <QMenu>
#include <QDir>

#include "rtklib.h"
#include "launchmain.h"
#include "launchoptdlg.h"

#include "ui_launchmain.h"


//---------------------------------------------------------------------------

#define BTN_SIZE        42
#define BTN_COUNT       8
#define MAX(x, y)       ((x) > (y) ? (x) : (y))

//---------------------------------------------------------------------------
MainForm::MainForm(QWidget *parent)
    : QDialog(parent), ui(new Ui::MainForm)
{
    ui->setupUi(this);

    QString prgFilename = QApplication::applicationFilePath();
    QFileInfo prgFileInfo(prgFilename);
    iniFile = prgFileInfo.absoluteDir().filePath(prgFileInfo.baseName()) + ".ini";;

    launchOptDlg = new LaunchOptDialog(this);

    setWindowTitle(tr("RTKLIB Version %1 %2").arg(VER_RTKLIB, PATCH_LEVEL));
    setWindowIcon(QIcon(":/icons/rtk9"));

    QCoreApplication::setApplicationName("rtklaunch_qt");
    QCoreApplication::setApplicationVersion("1.0");

    QSettings settings(iniFile, QSettings::IniFormat);
    option = settings.value("pos/option", 0).toInt();
    minimize = settings.value("pos/minimize", 1).toInt();
    ui->btnRtklib->setStatusTip(tr("RTKLIB Version %1 %2").arg(VER_RTKLIB, PATCH_LEVEL));

    QCommandLineParser parser;
    parser.setApplicationDescription(tr("RTKLIB application launcher Qt"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption titleOption("t",
                       QCoreApplication::translate("main", "Set Window Title"),
                       QCoreApplication::translate("main", "title"));
    parser.addOption(titleOption);

    QCommandLineOption trayOption(QStringList() << "tray",
                      QCoreApplication::translate("main", "Only show tray icon"));
    parser.addOption(trayOption);

    QCommandLineOption miniOption(QStringList() << "min",
                      QCoreApplication::translate("main", "Start minimized"));
    parser.addOption(miniOption);

    parser.process(*QApplication::instance());

    if (parser.isSet(titleOption))
        setWindowTitle(parser.value(titleOption));

    if (parser.isSet(trayOption)) {
        setVisible(false);
        trayIcon.setVisible(true);
    }

    if (parser.isSet(miniOption)) minimize = 1;

    ui->btnOption->setVisible(false);  // used options dialog

    trayMenu = new QMenu(this);
    trayMenu->addAction(tr("Expand"), this, &MainForm::expandWindow);
    trayMenu->addSeparator();
    trayMenu->addAction(tr("RTK&PLOT"), this, &MainForm::launchRTKPlot);
    trayMenu->addAction(tr("&RTKPOST"), this, &MainForm::launchRTKPost);
    trayMenu->addAction(tr("RTK&NAVI"), this, &MainForm::launchRTKNavi);
    trayMenu->addAction(tr("RTK&CONV"), this, &MainForm::launchRTKConv);
    trayMenu->addAction(tr("RTK&GET"), this, &MainForm::launchRTKGet);
    trayMenu->addAction(tr("RTK&STR"), this, &MainForm::launchStrSvr);
    trayMenu->addAction(tr("&NTRIP BROWSER"), this, &MainForm::launchSrcTblBrows);
    trayMenu->addSeparator();
    trayMenu->addAction(tr("&Exit"), this, &MainForm::close);

    trayIcon.setContextMenu(trayMenu);
    trayIcon.setIcon(QIcon(":/icons/rtk9"));
    trayIcon.setToolTip(windowTitle());

    QMenu *popup_menu = new QMenu();
    popup_menu->addAction(tr("&Expand"), this, &MainForm::expandWindow);
    popup_menu->addSeparator();
    popup_menu->addAction(ui->actionRtkConv);
    popup_menu->addAction(ui->actionRtkGet);
    popup_menu->addAction(ui->actionRtkNavi);
    popup_menu->addAction(ui->actionRtkNtrip);
    popup_menu->addAction(ui->actionRtkPlot);
    popup_menu->addAction(ui->actionRtkPost);
    popup_menu->addAction(ui->actionRtkStr);
    popup_menu->addAction(ui->actionRtkVideo);
    popup_menu->addSeparator();
    popup_menu->addAction(tr("E&xit"),this, &MainForm::accept);
    ui->btnRtklib->setMenu(popup_menu);

    connect(ui->btnPlot, &QPushButton::clicked, this, &MainForm::launchRTKPlot);
    connect(ui->btnConv, &QPushButton::clicked, this, &MainForm::launchRTKConv);
    connect(ui->btnStrSvr, &QPushButton::clicked, this, &MainForm::launchStrSvr);
    connect(ui->btnPost, &QPushButton::clicked, this, &MainForm::launchRTKPost);
    connect(ui->btnSrcTblBrows, &QPushButton::clicked, this, &MainForm::launchSrcTblBrows);
    connect(ui->btnNavi, &QPushButton::clicked, this, &MainForm::launchRTKNavi);
    connect(ui->btnTray, &QPushButton::clicked, this, &MainForm::moveToTray);
    connect(ui->btnGet, &QPushButton::clicked, this, &MainForm::launchRTKGet);
    connect(ui->btnOption, &QPushButton::clicked, this, &MainForm::showOptions);
    connect(ui->btnExit, &QPushButton::clicked, this, &MainForm::accept);
    connect(&trayIcon, &QSystemTrayIcon::activated, this, &MainForm::restoreFromTaskTray);

    connect(ui->actionRtkConv, &QAction::triggered, this, &MainForm::launchRTKConv);
    connect(ui->actionRtkGet, &QAction::triggered, this, &MainForm::launchRTKGet);
    connect(ui->actionRtkNavi, &QAction::triggered, this, &MainForm::launchRTKNavi);
    connect(ui->actionRtkNtrip, &QAction::triggered, this, &MainForm::launchSrcTblBrows);
    connect(ui->actionRtkPlot, &QAction::triggered, this, &MainForm::launchRTKPlot);
    connect(ui->actionRtkPost, &QAction::triggered, this, &MainForm::launchRTKPost);
    connect(ui->actionRtkStr, &QAction::triggered, this, &MainForm::launchStrSvr);

    updatePanel();
}
//---------------------------------------------------------------------------
void MainForm::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    QSettings settings(iniFile, QSettings::IniFormat);

    move(settings.value("pos/left", 0).toInt(),
         settings.value("pos/top", 0).toInt());

    resize(settings.value("pos/width", 310).toInt(),
           settings.value("pos/height", 79).toInt());
}
//---------------------------------------------------------------------------
void MainForm::closeEvent(QCloseEvent *)
{
    QSettings settings(iniFile, QSettings::IniFormat);

    settings.setValue("pos/left", pos().x());
    settings.setValue("pos/top", pos().y());
    settings.setValue("pos/width", width());
    settings.setValue("pos/height", height());
    settings.setValue("pos/option", option);
    settings.setValue("pos/minimize", minimize);
}
//---------------------------------------------------------------------------
void MainForm::launchRTKPlot()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"rtkplot_qt", "../../../bin/rtkplot_qt", "../rtkplot_qt/rtkplot_qt"};
    QStringList opts;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }
}
//---------------------------------------------------------------------------
void MainForm::launchRTKConv()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"rtkconv_qt", "../../../bin/rtkconv_qt", "../rtkconv_qt/rtkconv_qt"};
    QStringList opts;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }
}
//---------------------------------------------------------------------------
void MainForm::launchStrSvr()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"strsvr_qt", "../../../bin/strsvr_qt", "../strsvr_qt/strsvr_qt"};
    QStringList opts;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }
}
//---------------------------------------------------------------------------
void MainForm::launchRTKPost()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"rtkpost_qt", "../../../bin/rtkpost_qt", "../rtkpost_qt/rtkpost_qt"};
    QStringList opts;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }
}
//---------------------------------------------------------------------------
void MainForm::launchSrcTblBrows()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"srctblbrows_qt", "../../../bin/srctblbrows_qt", "../srctblbrows_qt/srctblbrows_qt"};
    QStringList opts;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }
}
//---------------------------------------------------------------------------
void MainForm::launchRTKNavi()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"rtknavi_qt", "../../../bin/rtknavi_qt", "../rtknavi_qt/rtknavi_qt"};
    QStringList opts;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }
}
//---------------------------------------------------------------------------
void MainForm::launchRTKGet()
{
    QDir appDir = QDir(QCoreApplication::applicationDirPath());
    QStringList cmds = {"rtkget_qt", "../../../bin/rtkget_qt", "../rtkget_qt/rtkget_qt"};
    QStringList opts;

    for (const auto& path: cmds)
        if (execCommand(appDir.filePath(path), opts)) {
            return;
        }
}
//---------------------------------------------------------------------------
int MainForm::execCommand(const QString &cmd, const QStringList &opt)
{
    return QProcess::startDetached(cmd, opt);
}
//---------------------------------------------------------------------------
void MainForm::moveToTray()
{
    setVisible(false);
    trayIcon.setVisible(true);
}
//---------------------------------------------------------------------------
void MainForm::restoreFromTaskTray(QSystemTrayIcon::ActivationReason reason)
{
    if (reason != QSystemTrayIcon::DoubleClick && reason != QSystemTrayIcon::Trigger) return;

    expandWindow();
}
//---------------------------------------------------------------------------
void MainForm::expandWindow()
{
    setVisible(true);
    trayIcon.setVisible(false);
    minimize = 0;

    updatePanel();
}
//---------------------------------------------------------------------------
void MainForm::updatePanel()
{
    if (minimize) {
        ui->panelTop->setVisible(false);
        ui->panelBottom->setVisible(true);
    }
    else {
        ui->panelTop->setVisible(true);
        ui->panelBottom->setVisible(false);
    }
}
//---------------------------------------------------------------------------
void MainForm::showOptions()
{
    launchOptDlg->setOption(option);
    launchOptDlg->setMinimize(minimize);

    launchOptDlg->exec();
    if (launchOptDlg->result() != QDialog::Accepted) return;

    option = launchOptDlg->getOption();
    minimize = launchOptDlg->getMinimize();

    updatePanel();
}
