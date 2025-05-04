//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QDateTime>
#include <QTimeZone>
#include <QLocale>

#include "timedlg.h"

#include "ui_timedlg.h"

//---------------------------------------------------------------------------
TimeDialog::TimeDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::TimeDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &TimeDialog::accept);
}
//---------------------------------------------------------------------------
void TimeDialog::setTime(const gtime_t &time)
{
    QString msg;
    QDateTime qtime_gpst, qtime_utc;
    gtime_t utc;
    double tow, doy;
    int week;
    char t_utc[40],t_gpst[40];
    utc = gpst2utc(time);

    qtime_gpst = QDateTime::fromSecsSinceEpoch(time.time, Qt::UTC);
    qtime_gpst = qtime_gpst.addMSecs(time.sec * 1000);

    qtime_utc = QDateTime::fromSecsSinceEpoch(utc.time, Qt::UTC);
    qtime_utc = qtime_utc.addMSecs(utc.sec * 1000);

    tow = time2gpst(time, &week);
    doy = time2doy(time);

    msg += QString(tr("GPST: %1\n")).arg(qtime_gpst.toString("yyyy/MM/dd hh:mm:ss"));
    msg += QString(tr("UTC : %1\n\n")).arg(qtime_utc.toString("yyyy/MM/dd hh:mm:ss"));
    msg += QString(tr("GPS Week: %L1\n")).arg(week);
    msg += QString(tr("GPS Time: %L1 s\n")).arg(tow, 0, 'f', 0);
    msg += QString(tr("Day of Year: %L1\n")).arg((int)floor(doy), 3, 10, QChar('0'));
    msg += QString(tr("Day of Week: %L1\n")).arg((int)floor(tow / 86400.0));
    msg += QString(tr("Time of Day: %L1 s\n")).arg(fmod(tow, 86400.0), 0, 'f', 0);
    msg += QString(tr("Leap Seconds: %L1 s\n")).arg(timediff(time, utc), 0, 'f', 0);

    ui->message->setText(msg);
}
//---------------------------------------------------------------------------
