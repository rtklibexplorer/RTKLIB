//---------------------------------------------------------------------------

#include <QShowEvent>

#include "rtklib.h"
#include "keydlg.h"
#include "markdlg.h"
#include "refdlg.h"

#include "ui_markdlg.h"

//---------------------------------------------------------------------------
MarkDialog::MarkDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::MarkDialog)
{
    ui->setupUi(this);

    nMark = 1;
    ui->lblMarker->setText(QStringLiteral("%r=%1").arg(nMark, 3, 10, QLatin1Char('0')));

    connect(ui->btnKeyDlg, &QPushButton::clicked, this, &MarkDialog::showKeyDialog);
    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &MarkDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &MarkDialog::reject);
    connect(ui->btnPosition, &QPushButton::clicked, this, &MarkDialog::btnPositionClicked);
    connect(ui->cBMarkerNameC, &QCheckBox::clicked, this, &MarkDialog::updateEnable);
    connect(ui->rBGo, &QRadioButton::toggled, this, &MarkDialog::updateEnable);
    connect(ui->rBStop, &QRadioButton::toggled, this, &MarkDialog::updateEnable);
    connect(ui->rBFix, &QRadioButton::toggled, this, &MarkDialog::updateEnable);

    keyDialog = new KeyDialog(this);
}
//---------------------------------------------------------------------------
void MarkDialog::accept()
{
    if (ui->cBMarkerNameC->isChecked()) {
        nMark++;
        ui->lblMarker->setText(QStringLiteral("%r=%1").arg(nMark, 3, 10, QLatin1Char('0')));
    }

    QDialog::accept();
}
//---------------------------------------------------------------------------
void MarkDialog::updateEnable()
{
    int positionMode = getPositionMode();
    bool ena = positionMode == PMODE_STATIC || positionMode == PMODE_PPP_STATIC ||
               positionMode == PMODE_KINEMA || positionMode == PMODE_PPP_KINEMA ||
               positionMode == PMODE_FIXED || positionMode == PMODE_PPP_FIXED;

    ui->rBStop->setEnabled(ena);
    ui->rBGo->setEnabled(ena);
    ui->rBFix->setEnabled(ena);
    ui->gBPositionMode->setEnabled(ena);
    ui->sBLatitude->setEnabled(ui->rBFix->isChecked());
    ui->sBLongitude->setEnabled(ui->rBFix->isChecked());
    ui->sBHeight->setEnabled(ui->rBFix->isChecked());
    ui->btnPosition->setEnabled(ui->rBFix->isChecked());
    ui->lblPosition->setEnabled(ui->rBFix->isChecked());
    ui->cBMarkerName->setEnabled(ui->cBMarkerNameC->isChecked());
}
//---------------------------------------------------------------------------
void MarkDialog::showKeyDialog()
{
    keyDialog->setWindowTitle(tr("Key Replacement in Marker Name"));
    keyDialog->show();
}
//---------------------------------------------------------------------------
void MarkDialog::btnPositionClicked()
{
    RefDialog *refDialog =  new RefDialog(this);

    refDialog->setRoverPosition(ui->sBLatitude->value(), ui->sBLongitude->value(), ui->sBHeight->value());
    refDialog->stationPositionFile = stationPositionFileF;

    refDialog->exec();
    if (refDialog->result() != QDialog::Accepted) return;

    ui->sBLatitude->setValue(refDialog->getPosition()[0]);
    ui->sBLongitude->setValue(refDialog->getPosition()[1]);
    ui->sBHeight->setValue(refDialog->getPosition()[2]);
}
//---------------------------------------------------------------------------
void MarkDialog::setPositionMode(int mode)
{
    positionMode = mode;

    if (positionMode == PMODE_STATIC || positionMode == PMODE_PPP_STATIC) {
        ui->rBStop->setChecked(true);
    } else if (positionMode == PMODE_KINEMA || positionMode == PMODE_PPP_KINEMA) {
        ui->rBGo->setChecked(true);
    } else {
        ui->rBStop->setChecked(false);
        ui->rBGo->setChecked(false);
    }

    updateEnable();
}
//---------------------------------------------------------------------------
int MarkDialog::getPositionMode()
{
    if (ui->rBGo->isChecked()) {
        if (positionMode == PMODE_STATIC || positionMode == PMODE_FIXED)
            positionMode = PMODE_KINEMA;
        else if (positionMode == PMODE_PPP_STATIC || positionMode == PMODE_PPP_FIXED)
            positionMode = PMODE_PPP_KINEMA;
    } else if (ui->rBStop->isChecked()) {
        if (positionMode == PMODE_KINEMA || positionMode == PMODE_FIXED)
            positionMode = PMODE_STATIC;
        else if (positionMode == PMODE_PPP_KINEMA || positionMode == PMODE_PPP_FIXED)
            positionMode = PMODE_PPP_STATIC;
    } else if (ui->rBFix->isChecked()) {
        if (positionMode == PMODE_KINEMA || positionMode == PMODE_STATIC) {
            positionMode = PMODE_FIXED;
        }
        else if (positionMode == PMODE_PPP_KINEMA || positionMode == PMODE_PPP_STATIC) {
            positionMode = PMODE_PPP_FIXED;
        }
    }
    return positionMode;
}
//---------------------------------------------------------------------------
void MarkDialog::setName(const QString &name)
{
    ui->cBMarkerName->setCurrentText(name);
}
//---------------------------------------------------------------------------
QString MarkDialog::getName()
{
    QString mrkr = ui->cBMarkerName->currentText();
    char str2[1024];

    if (!ui->cBMarkerNameC->isChecked())
        return "";

    reppath(qPrintable(mrkr), str2, utc2gpst(timeget()), qPrintable(QString("%1").arg(nMark, 3, 10, QChar('0'))), "");
    return mrkr;
}
//---------------------------------------------------------------------------
void MarkDialog::setComment(const QString &comment)
{
    ui->lEMarkerComment->setText(comment);
}
//---------------------------------------------------------------------------
QString MarkDialog::getComment()
{
    return ui->lEMarkerComment->text();
}
//---------------------------------------------------------------------------
void MarkDialog::setStationPositionFile(const QString &file)
{
    stationPositionFileF = file;
}
//---------------------------------------------------------------------------
QString MarkDialog::getStationPositionFile()
{
    return stationPositionFileF;
}
//---------------------------------------------------------------------------
