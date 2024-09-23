//---------------------------------------------------------------------------

#include <QShowEvent>
#include <QDateTime>
#include <QTimeZone>

#include "plotmain.h"
#include "tspandlg.h"
#include "timedlg.h"

#include "ui_tspandlg.h"

//---------------------------------------------------------------------------
SpanDialog::SpanDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::SpanDialog)
{
    ui->setupUi(this);

    for (int i = 0; i < 3; i++) {
        timeValid[i] = true;
	}

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &SpanDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &SpanDialog::reject);
    connect(ui->cBTimeEndEnabled, &QCheckBox::clicked, this, &SpanDialog::updateEnable);
    connect(ui->cBTimeIntervalEnabled, &QCheckBox::clicked, this, &SpanDialog::updateEnable);
    connect(ui->cBTimeStartEnabled, &QCheckBox::clicked, this, &SpanDialog::updateEnable);
    connect(ui->btnTimeStart, &QPushButton::clicked, this, &SpanDialog::showStartTimeDialog);
    connect(ui->btnTimeEnd, &QPushButton::clicked, this, &SpanDialog::showEndTimeDialog);
}
//---------------------------------------------------------------------------
void SpanDialog::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    updateEnable();
}
//---------------------------------------------------------------------------
void SpanDialog::updateEnable()
{
    ui->dTTimeStart->setEnabled(ui->cBTimeStartEnabled->isChecked() && timeValid[0]);
    ui->dTTimeEnd->setEnabled(ui->cBTimeEndEnabled->isChecked() && timeValid[1]);
    ui->sBTimeInterval->setEnabled(ui->cBTimeIntervalEnabled->isChecked() && timeValid[2]);
    ui->cBTimeStartEnabled->setEnabled(timeValid[0] == 1);
    ui->cBTimeEndEnabled->setEnabled(timeValid[1] == 1);
    ui->cBTimeIntervalEnabled->setEnabled(timeValid[2] == 1);
}
//---------------------------------------------------------------------------
void SpanDialog::showStartTimeDialog()
{
    TimeDialog * timeDialog = new TimeDialog(this);

    gtime_t time;
    time.time = ui->dTTimeStart->dateTime().toUTC().toSecsSinceEpoch();
    time.sec = ui->dTTimeStart->dateTime().toUTC().time().msec() / 1000;
    timeDialog->setTime(time);

    timeDialog->exec();
}
//---------------------------------------------------------------------------
void SpanDialog::showEndTimeDialog()
{
    TimeDialog * timeDialog = new TimeDialog(this);

    gtime_t time;
    time.time = ui->dTTimeEnd->dateTime().toSecsSinceEpoch();
    time.sec = ui->dTTimeEnd->dateTime().time().msec() / 1000;
    timeDialog->setTime(time);

    timeDialog->exec();
}
//---------------------------------------------------------------------------
gtime_t SpanDialog::getStartTime()
{
    gtime_t timeStart = {0, 0};

    if (ui->cBTimeStartEnabled->isChecked()) {
        QDateTime start(ui->dTTimeStart->dateTime());
        timeStart.time = start.toSecsSinceEpoch();
        timeStart.sec = start.time().msec() / 1000;
    }

    return timeStart;
}
//---------------------------------------------------------------------------
void SpanDialog::setStartTime(gtime_t timeStart)
{
    QDateTime start = QDateTime::fromSecsSinceEpoch(timeStart.time, QTimeZone::utc());

    start = start.addMSecs(timeStart.sec * 1000);

    ui->dTTimeStart->setDateTime(start);

    ui->cBTimeStartEnabled->setCheckState(Qt::Checked);
}
//---------------------------------------------------------------------------
gtime_t SpanDialog::getEndTime()
{
    gtime_t timeEnd = {0, 0};

    if (ui->cBTimeEndEnabled->isChecked()) {
        QDateTime end(ui->dTTimeEnd->dateTime());
        timeEnd.time = end.toSecsSinceEpoch();
        timeEnd.sec = end.time().msec() / 1000;
    }

    return timeEnd;
}
//---------------------------------------------------------------------------
void SpanDialog::setEndTime(gtime_t timeEnd)
{
    QDateTime end = QDateTime::fromSecsSinceEpoch(timeEnd.time, QTimeZone::utc());
    end = end.addMSecs(timeEnd.sec * 1000);

    ui->dTTimeEnd->setDateTime(end);
    ui->cBTimeEndEnabled->setCheckState(Qt::Checked);
}
//---------------------------------------------------------------------------
double SpanDialog::getTimeInterval()
{
    if (ui->cBTimeIntervalEnabled->isChecked())
        return ui->sBTimeInterval->value();
    else
        return 0;
}
//---------------------------------------------------------------------------
void SpanDialog::setTimeInterval(double timeInterval)
{
    ui->sBTimeInterval->setValue(timeInterval);
    ui->cBTimeIntervalEnabled->setCheckState(Qt::Checked);

}
//---------------------------------------------------------------------------
int SpanDialog::getTimeEnable(int i)
{
    switch(i)
    {
        case 0: return ui->cBTimeStartEnabled->isChecked();
        case 1: return ui->cBTimeEndEnabled->isChecked();
        case 2: return ui->cBTimeIntervalEnabled->isChecked();
        default: return -1;
    }
}
//---------------------------------------------------------------------------
void SpanDialog::setTimeEnable(int i, bool enable)
{
    switch(i)
    {
        case 0: ui->cBTimeStartEnabled->setChecked(enable);break;
        case 1: ui->cBTimeEndEnabled->setChecked(enable);break;
        case 2: ui->cBTimeIntervalEnabled->setChecked(enable);break;
    }
}
//---------------------------------------------------------------------------
int SpanDialog::getTimeValid(int i)
{
    return timeValid[i];
}
//---------------------------------------------------------------------------
void SpanDialog::setTimeValid(int i, int valid)
{
    timeValid[i] = valid;

    updateEnable();
}
//---------------------------------------------------------------------------

