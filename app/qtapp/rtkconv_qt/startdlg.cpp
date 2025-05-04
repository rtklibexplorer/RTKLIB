//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QDateTime>
#include <QFileInfo>
#include <QDateTime>

#include "rtklib.h"
#include "startdlg.h"

#include "ui_startdlg.h"

//---------------------------------------------------------------------------
StartDialog::StartDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::StartDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &StartDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &StartDialog::reject);
    connect(ui->btnFileTime, &QPushButton::clicked, this, &StartDialog::setTimeFromFile);

    setTime(utc2gpst(timeget()));
}
//---------------------------------------------------------------------------
void StartDialog::setFileName(const QString &fn)
{
    FILE *fp;
    uint32_t timetag = 0;
    uint8_t buff[80] = {0};
    char path_tag[1024], path[1020], *paths[1];
    gtime_t time = {0, 0};

    // read time tag file if exists
    paths[0] = path;
    if (expath(qPrintable(fn), paths, 1)) {
        sprintf(path_tag, "%s.tag", path);
        if ((fp = fopen(path_tag, "rb"))) {
            fread(buff, 64, 1, fp);
            if (!strncmp((char *)buff, "TIMETAG", 7) && fread(&timetag, 4, 1, fp)) {
                time.time = timetag;
                time.sec = 0;
                filename = fn;
            }
            fclose(fp);
            setTime(time);
        }
    }
}
//---------------------------------------------------------------------------
void StartDialog::setTime(const gtime_t &time)
{
    QDateTime date = QDateTime::fromSecsSinceEpoch(time.time, Qt::UTC);
    date = date.addMSecs(time.sec*1000);

    ui->tETime->setDateTime(date);
}
//---------------------------------------------------------------------------
gtime_t StartDialog::getTime()
{
    QDateTime date(ui->tETime->dateTime());
    gtime_t time;

    time.time = date.toSecsSinceEpoch();
    time.sec = date.time().msec() / 1000;

    return time;
}
//---------------------------------------------------------------------------
void StartDialog::setTimeFromFile()
{
    char path[1024], *paths[1];

    // extend wild-card and get first file
    paths[0] = path;
    if (expath(qPrintable(filename), paths, 1)) {
        filename = path;
    }

    QFileInfo fi(filename);
    QDateTime d = fi.birthTime();

    ui->tETime->setDateTime(d);
}
//---------------------------------------------------------------------------
