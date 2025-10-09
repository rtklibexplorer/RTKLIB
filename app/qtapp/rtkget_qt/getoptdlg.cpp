//---------------------------------------------------------------------------

#include "getoptdlg.h"
#include "getmain.h"

#include <QFileDialog>
#include <QShowEvent>
#include <QIntValidator>
#include <QCompleter>
#include <QFileSystemModel>
#include <QAction>

#include "ui_getoptdlg.h"

//---------------------------------------------------------------------------
DownOptDialog::DownOptDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::DownOptDialog)
{
    ui->setupUi(this);

    QFileSystemModel *fileModel = new QFileSystemModel(this);
    fileModel->setRootPath("");
    ui->lEUrlFilename->setCompleter(new QCompleter(fileModel, this));
    ui->lELogFilename->setCompleter(new QCompleter(fileModel, this));

    QAction *acLogFilename = ui->lELogFilename->addAction(QIcon(":/icons/folder"), QLineEdit::TrailingPosition);
    acLogFilename->setToolTip(tr("Select Log File Path"));

    QAction *acUrlFilename = ui->lEUrlFilename->addAction(QIcon(":/icons/folder"), QLineEdit::TrailingPosition);
    acUrlFilename->setToolTip(tr("Select URL File Path"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &DownOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &DownOptDialog::reject);
    connect(acLogFilename, &QAction::triggered, this, &DownOptDialog::selectLogFilename);
    connect(acUrlFilename, &QAction::triggered, this, &DownOptDialog::selectUrlFile);
}
//---------------------------------------------------------------------------
void DownOptDialog::selectUrlFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("GNSS Data URL File"));

    if (!filename.isEmpty())
        ui->lEUrlFilename->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void DownOptDialog::selectLogFilename()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Download Log File"));

    if (!filename.isEmpty())
        ui->lELogFilename->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void DownOptDialog::updateUi()
{
    ui->cBKeepErr->setChecked(holdErr);
    ui->cBKeepList->setChecked(holdList);
    ui->sBTestColumnCount->setValue(columnCnt);
    ui->lEProxy->setText(proxyAddr);
    ui->lEUrlFilename->setText(urlFile);
    ui->lELogFilename->setText(logFile);
    ui->cBLogAppend->setChecked(logAppend);
    ui->cBDateFormat->setCurrentIndex(dateFormat);
    ui->cBTraceLevel->setCurrentIndex(traceLevel);
}
//---------------------------------------------------------------------------
void DownOptDialog::accept()
{
    holdErr = ui->cBKeepErr->isChecked();
    holdList = ui->cBKeepList->isChecked();
    columnCnt = ui->sBTestColumnCount->value();
    proxyAddr = ui->lEProxy->text();
    urlFile = ui->lEUrlFilename->text();
    logFile = ui->lELogFilename->text();
    logAppend = ui->cBLogAppend->isChecked();
    dateFormat = ui->cBDateFormat->currentIndex();
    traceLevel = ui->cBTraceLevel->currentIndex();

    QDialog::accept();
}
//---------------------------------------------------------------------------
void DownOptDialog::loadOptions(QSettings &setting)
{
    urlFile = setting.value("opt/urlfile", "").toString();
    logFile = setting.value("opt/logfile", "").toString();
    stations = setting.value("opt/stations", "").toString();
    proxyAddr = setting.value("opt/proxyaddr", "").toString();
    holdErr = setting.value("opt/holderr", 0).toInt();
    holdList = setting.value("opt/holdlist", 0).toInt();
    columnCnt = setting.value("opt/ncol", 35).toInt();
    logAppend = setting.value("opt/logappend", 0).toInt();
    dateFormat = setting.value("opt/dateformat", 0).toInt();
    traceLevel = setting.value("opt/tracelevel", 0).toInt();

    updateUi();
}
//---------------------------------------------------------------------------
void DownOptDialog::saveOptions(QSettings &setting)
{
    setting.setValue("opt/urlfile", urlFile);
    setting.setValue("opt/logfile", logFile);
    setting.setValue("opt/stations", stations);
    setting.setValue("opt/proxyaddr", proxyAddr);
    setting.setValue("opt/holderr", holdErr);
    setting.setValue("opt/holdlist", holdList);
    setting.setValue("opt/ncol", columnCnt);
    setting.setValue("opt/logappend", logAppend);
    setting.setValue("opt/dateformat", dateFormat);
    setting.setValue("opt/tracelevel", traceLevel);
}
//---------------------------------------------------------------------------
