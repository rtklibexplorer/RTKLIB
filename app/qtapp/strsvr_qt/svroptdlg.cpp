//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QCompleter>
#include <QFileDialog>
#include <QAction>

#include "rtklib.h"
#include "refdlg.h"
#include "svroptdlg.h"
//---------------------------------------------------------------------------

#include "ui_svroptdlg.h"

//---------------------------------------------------------------------------
SvrOptDialog::SvrOptDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::SvrOptDialog)
{
    ui->setupUi(this);

    refDialog = new RefDialog(this, 1);

    QCompleter *dirCompleter = new QCompleter(this);
    QFileSystemModel *dirModel = new QFileSystemModel(dirCompleter);
    dirModel->setRootPath("");
    dirModel->setFilter(QDir::AllDirs | QDir::Drives | QDir::NoDotAndDotDot);
    dirCompleter->setModel(dirModel);
    ui->lELocalDir->setCompleter(dirCompleter);

    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lELogFile->setCompleter(fileCompleter);

    QAction *acLogFile = ui->lELogFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acLogFile->setToolTip(tr("Select log file"));

    QAction *acLocalDir = ui->lELocalDir->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acLocalDir->setToolTip(tr("Select local directory"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &SvrOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &SvrOptDialog::reject);
    connect(ui->btnPosition, &QPushButton::clicked, this, &SvrOptDialog::positionSelect);
    connect(acLogFile, &QAction::triggered, this, &SvrOptDialog::logFileSelect);
    connect(ui->cBNmeaReq, &QPushButton::clicked, this, &SvrOptDialog::updateEnable);
    connect(acLocalDir, &QAction::triggered, this, &SvrOptDialog::localDirectorySelect);
    connect(ui->cBStationId, &QPushButton::clicked, this, &SvrOptDialog::updateEnable);

}
//---------------------------------------------------------------------------
void SvrOptDialog::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    ui->sBDataTimeout->setValue(serverOptions[0]);
    ui->sBReconnectInterval->setValue(serverOptions[1]);
    ui->sBAveragePeriodRate->setValue(serverOptions[2]);
    ui->sBServerBufferSize->setValue(serverOptions[3]);
    ui->sBServerCycle->setValue(serverOptions[4]);
    ui->cBRelayMessage->setCurrentIndex(relayBack);
    ui->sBProgressBar->setValue(progressBarRange);
    ui->sBNmeaCycle->setValue(serverOptions[5]);
    ui->sBFileSwapMargin->setValue(fileSwapMargin);

    if (norm(antennaPosition, 3) > 0.0) {
        double pos[3];

        ecef2pos(antennaPosition, pos);
        ui->sBAntennaPos1->setValue(pos[0] * R2D);
        ui->sBAntennaPos2->setValue(pos[1] * R2D);
        ui->sBAntennaPos3->setValue(pos[2]);
    } else {
        ui->sBAntennaPos1->setValue(0);
        ui->sBAntennaPos2->setValue(0);
        ui->sBAntennaPos3->setValue(0);
	}
    ui->cBTraceLevel->setCurrentIndex(traceLevel);
    ui->cBNmeaReq->setChecked(nmeaRequest);
    ui->lELocalDir->setText(localDirectory);
    ui->lEProxyAddress->setText(proxyAddress);
    ui->sBStationId->setValue(stationId);
    ui->cBStationId->setChecked(stationSelect);
    ui->lEAntennaInfo->setText(antennaType);
    ui->lEReceiverInfo->setText(receiverType);
    ui->sBAntennaOffset1->setValue(antennaOffsets[0]);
    ui->sBAntennaOffset2->setValue(antennaOffsets[1]);
    ui->sBAntennaOffset3->setValue(antennaOffsets[2]);
    ui->lELogFile->setText(logFile);

	updateEnable();
}
//---------------------------------------------------------------------------
void SvrOptDialog::accept()
{
	double pos[3];

    serverOptions[0] = ui->sBDataTimeout->value();
    serverOptions[1] = ui->sBReconnectInterval->value();
    serverOptions[2] = ui->sBAveragePeriodRate->value();
    serverOptions[3] = ui->sBServerBufferSize->value();
    serverOptions[4] = ui->sBServerCycle->value();
    serverOptions[5] = ui->sBNmeaCycle->value();
    fileSwapMargin = ui->sBFileSwapMargin->value();
    relayBack = ui->cBRelayMessage->currentIndex();
    progressBarRange = ui->sBProgressBar->value();

    pos[0] = ui->sBAntennaPos1->value() * D2R;
    pos[1] = ui->sBAntennaPos2->value() * D2R;
    pos[2] = ui->sBAntennaPos3->value();

    if (norm(pos, 3) > 0.0)
        pos2ecef(pos, antennaPosition);
    else
        for (int i = 0; i < 3; i++) antennaPosition[i] = 0.0;

    logFile = ui->lELogFile->text();
    traceLevel = ui->cBTraceLevel->currentIndex();
    nmeaRequest = ui->cBNmeaReq->isChecked();
    localDirectory = ui->lELocalDir->text();
    proxyAddress = ui->lEProxyAddress->text();
    stationId = ui->sBStationId->value();
    stationSelect = ui->cBStationId->isChecked();
    antennaType = ui->lEAntennaInfo->text();
    receiverType = ui->lEReceiverInfo->text();
    antennaOffsets[0] = ui->sBAntennaOffset1->value();
    antennaOffsets[1] = ui->sBAntennaOffset2->value();
    antennaOffsets[2] = ui->sBAntennaOffset3->value();

    QDialog::accept();
}
//---------------------------------------------------------------------------
void SvrOptDialog::positionSelect()
{    
    refDialog->setRoverPosition(ui->sBAntennaPos1->value(), ui->sBAntennaPos2->value(), ui->sBAntennaPos3->value());
    refDialog->setLoadEnabled(true);
    refDialog->stationPositionFile = stationPositionFile;

    refDialog->exec();
    if (refDialog->result() != QDialog::Accepted) return;

    ui->sBAntennaPos1->setValue(refDialog->getPosition()[0]);
    ui->sBAntennaPos2->setValue(refDialog->getPosition()[1]);
    ui->sBAntennaPos3->setValue(refDialog->getPosition()[2]);
    stationPositionFile = refDialog->stationPositionFile;
}
//---------------------------------------------------------------------------
void SvrOptDialog::localDirectorySelect()
{
    QString dir = ui->lELocalDir->text();

    dir = QFileDialog::getExistingDirectory(this, tr("Local Directory"), dir);
    ui->lELocalDir->setText(dir);
}
//---------------------------------------------------------------------------
void SvrOptDialog::updateEnable()
{
    ui->sBNmeaCycle->setEnabled(ui->cBNmeaReq->isChecked());
    ui->sBStationId->setEnabled(ui->cBStationId->isChecked());

    ui->sBAntennaPos1->setEnabled(ui->cBStationId->isChecked() || ui->cBNmeaReq->isChecked());
    ui->sBAntennaPos2->setEnabled(ui->cBStationId->isChecked() || ui->cBNmeaReq->isChecked());
    ui->sBAntennaPos3->setEnabled(ui->cBStationId->isChecked() || ui->cBNmeaReq->isChecked());
    ui->lblLatLonHeight->setEnabled(ui->cBStationId->isChecked() || ui->cBNmeaReq->isChecked());
    ui->btnPosition->setEnabled(ui->cBStationId->isChecked() || ui->cBNmeaReq->isChecked());

    ui->sBAntennaOffset1->setEnabled(ui->cBStationId->isChecked());
    ui->sBAntennaOffset2->setEnabled(ui->cBStationId->isChecked());
    ui->sBAntennaOffset3->setEnabled(ui->cBStationId->isChecked());
    ui->lblAntennaOffsets->setEnabled(ui->cBStationId->isChecked());

    ui->lEAntennaInfo->setEnabled(ui->cBStationId->isChecked());
    ui->lblAntennaInfo->setEnabled(ui->lEAntennaInfo->isEnabled());
    ui->lEReceiverInfo->setEnabled(ui->cBStationId->isChecked());
    ui->lblReceiverInfo->setEnabled(ui->lEReceiverInfo->isEnabled());
}
//---------------------------------------------------------------------------
void SvrOptDialog::logFileSelect()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Log File"), ui->lELogFile->text(), tr("All (*.*)"));
    if (!filename.isEmpty()) ui->lELogFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
