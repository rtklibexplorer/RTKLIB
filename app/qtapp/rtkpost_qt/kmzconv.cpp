//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QProcess>
#include <QtGlobal>
#include <QFileSystemModel>
#include <QCompleter>
#include <QAction>

#include <string.h>

#include "postmain.h"
#include "viewer.h"
#include "helper.h"
#include "kmzconv.h"

#include "ui_kmzconv.h"

#include "rtklib.h"

#ifdef Q_OS_WIN
#define GOOGLE_EARTH "C:\\Program Files\\Google\\Google Earth Pro\\client\\googleearth.exe"
#endif
#ifdef Q_OS_LINUX
#define GOOGLE_EARTH "google-earth"
#endif
#ifndef GOOGLE_EARTH
#define GOOGLE_EARTH ""
#endif

//---------------------------------------------------------------------------
ConvDialog::ConvDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ConvDialog)
{
    ui->setupUi(this);

    viewer = new TextViewer(this);

    QFileSystemModel *fileModel = new QFileSystemModel(this);
    fileModel->setRootPath("");
    ui->lEInputFile->setCompleter(new QCompleter(fileModel, this));
    ui->lEOutputFile->setCompleter(new QCompleter(fileModel, this));
    ui->lEGoogleEarthFile->setCompleter(new QCompleter(fileModel, this));

    // Google Earth file line edit actions
    QAction *acGoogleEarthFileSelect = ui->lEGoogleEarthFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acGoogleEarthFileSelect->setToolTip(tr("Select Google Earth Executable"));

    // input file line edit actions
    QAction *acInputFileSelect = ui->lEInputFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acInputFileSelect->setToolTip(tr("Select File"));

    connect(ui->btnClose, &QPushButton::clicked, this, &ConvDialog::accept);
    connect(ui->btnConvert, &QPushButton::clicked, this, &ConvDialog::convert);
    connect(acGoogleEarthFileSelect, &QAction::triggered, this, &ConvDialog::selectGoogleEarthFile);
    connect(ui->btnView, &QPushButton::clicked, this, &ConvDialog::viewOutputFile);
    connect(ui->btnGoogleEarth, &QPushButton::clicked, this, &ConvDialog::callGoogleEarth);
    connect(ui->cBAddOffset, &QCheckBox::clicked, this, &ConvDialog::updateEnable);
    connect(ui->cBTimeSpan, &QCheckBox::clicked, this, &ConvDialog::updateEnable);
    connect(ui->sBTimeInterval, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &ConvDialog::updateEnable);
    connect(ui->lEInputFile, &QLineEdit::editingFinished, this, &ConvDialog::updateEnable);
    connect(acInputFileSelect, &QAction::triggered, this, &ConvDialog::selectInputFile);
    connect(ui->cBCompress, &QCheckBox::clicked, this, &ConvDialog::updateOutputFile);
    connect(ui->rBFormatKML, &QRadioButton::clicked, this, &ConvDialog::formatChanged);
    connect(ui->rBFormatGPX, &QRadioButton::clicked, this, &ConvDialog::formatChanged);

    ui->rBFormatGPX->setChecked(!ui->rBFormatKML->isChecked());

    updateEnable();
}
//---------------------------------------------------------------------------
void ConvDialog::setInput(const QString &filename)
{
    ui->lEInputFile->setText(filename);
	updateOutputFile();
}

//---------------------------------------------------------------------------
void ConvDialog::selectInputFile()
{
    ui->lEInputFile->setText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEInputFile->text(), tr("All (*.*)"))));
}
//---------------------------------------------------------------------------
void ConvDialog::convert()
{
    QString OutputFilename = ui->lEOutputFile->text();
	int stat;
    QString cmd;
    QStringList opt;
    char file[1024], kmlfile[1024], *p;
    double offset[3] = { 0 }, tint = 0.0;
    gtime_t ts = { 0, 0 }, te = { 0, 0 };

	showMessage("");

    if (ui->lEInputFile->text().isEmpty() || ui->lEOutputFile->text().isEmpty()) return;

    showMessage(tr("Converting ..."));

    if (ui->cBTimeSpan->isChecked()) {
        QDateTime start(ui->dateTimeStart->dateTime());
        QDateTime end(ui->dateTimeStop->dateTime());
        ts.time = start.toSecsSinceEpoch(); ts.sec = start.time().msec() / 1000;
        te.time = end.toSecsSinceEpoch(); te.sec = end.time().msec() / 1000;
  }

    if (ui->cBAddOffset->isChecked()) {
        offset[0] = ui->sBOffset1->value();
        offset[1] = ui->sBOffset2->value();
        offset[2] = ui->sBOffset3->value();
	}

    if (ui->cBTimeInterval->isChecked())
        tint = ui->sBTimeInterval->value();

    strncpy(file, qPrintable(ui->lEInputFile->text()), 1023);

    if (ui->rBFormatKML->isChecked()) {
        if (ui->cBCompress->isChecked()) { // generate .kml file and compress it afterwards
            strncpy(kmlfile, file, 1020);
            if (!(p = strrchr(kmlfile, '.'))) p = kmlfile + strlen(kmlfile);
            strncpy(p, ".kml", 5);
        }
        stat = convkml(file, ui->cBCompress->isChecked() ? kmlfile : qPrintable(OutputFilename),
                   ts, te, tint, ui->cBQFlags->currentIndex(), offset,
                   ui->cBTrackColor->currentIndex(), ui->cBPointColor->currentIndex(),
                   ui->cBOutputAltitude->currentIndex(), ui->cBOutputTime->currentIndex());
    } else {
        stat = convgpx(file, ui->cBCompress->isChecked() ? kmlfile : qPrintable(OutputFilename),
                   ts, te, tint, ui->cBQFlags->currentIndex(), offset,
                   ui->cBTrackColor->currentIndex(), ui->cBPointColor->currentIndex(),
                   ui->cBOutputAltitude->currentIndex(), ui->cBOutputTime->currentIndex());
    }

    if (stat < 0) {
        if (stat == -1) showMessage(tr("Error : Reading input file"));
        else if (stat == -2) showMessage(tr("Error : Input file format"));
        else if (stat == -3) showMessage(tr("Error : No data in input file"));
        else showMessage(tr("Error : Writing kml file"));
		return;
	}
    if (ui->rBFormatKML->isChecked() && ui->cBCompress->isChecked()) {
#ifdef Q_OS_WIN
        cmd = "zip.exe";
        opt << "-j";
        opt << QString("-m %1").arg(ui->lEOutputFile->text()); //TODO: zip for other platforms
#endif
#ifdef Q_OS_LINUX
        cmd = "gzip";
        opt << QString("-3 %1 %2").arg(ui->lEOutputFile->text());
#endif
        opt << kmlfile;
        if (!execCommand(cmd, opt)) {
            showMessage(tr("Error: zip execution"));
			return;
		}
	}
	showMessage("done");
}

//---------------------------------------------------------------------------
void ConvDialog::updateEnable()
{
    ui->sBOffset1->setEnabled(ui->cBAddOffset->isChecked());
    ui->sBOffset2->setEnabled(ui->cBAddOffset->isChecked());
    ui->sBOffset3->setEnabled(ui->cBAddOffset->isChecked());
    ui->dateTimeStart->setEnabled(ui->cBTimeSpan->isChecked());
    ui->dateTimeStop->setEnabled(ui->cBTimeSpan->isChecked());
    ui->sBTimeInterval->setEnabled(ui->cBTimeInterval->isChecked());

    ui->lEGoogleEarthFile->setEnabled(ui->rBFormatKML->isChecked());
    ui->btnGoogleEarth->setEnabled(ui->rBFormatKML->isChecked());

    if (ui->rBFormatKML->isChecked()) {
        ui->cBCompress->setEnabled(false);
#ifdef Q_OS_WIN
        ui->cBCompress->setEnabled(true);
#endif
#ifdef Q_OS_LINUX
        ui->cBCompress->setEnabled(true);
#endif
    } else
        ui->cBCompress->setEnabled(false);
}
//---------------------------------------------------------------------------
int ConvDialog::execCommand(const QString &cmd, const QStringList &opt)
{
    return QProcess::startDetached(cmd, opt);
}
//---------------------------------------------------------------------------
void ConvDialog::showMessage(const QString &msg)
{
    ui->lblMessage->setText(msg);
    if (msg.contains("error", Qt::CaseInsensitive))
        setWidgetTextColor(ui->lblMessage, Qt::red);
    else
        setWidgetTextColor(ui->lblMessage, Qt::blue);
}
//---------------------------------------------------------------------------
void ConvDialog::updateOutputFile()
{
    QString inputFilename = ui->lEInputFile->text();

    if (inputFilename.isEmpty()) return;

    QFileInfo fi(inputFilename);
    if (fi.suffix().isEmpty()) return;

    QString filename = fi.baseName() + (ui->rBFormatGPX->isChecked() ? ".gpx" : (ui->cBCompress->isChecked() ? ".kmz" : ".kml"));

    ui->lEOutputFile->setText(fi.absoluteDir().absoluteFilePath(filename));
}
//---------------------------------------------------------------------------
void ConvDialog::selectGoogleEarthFile()
{
    ui->lEGoogleEarthFile->setText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Google Earth Exe File"), ui->lEGoogleEarthFile->text())));
}
//---------------------------------------------------------------------------
void ConvDialog::callGoogleEarth()
{
    QString cmd;
    QStringList opt;

    cmd = ui->lEGoogleEarthFile->text();
    opt << ui->lEOutputFile->text();
    
    if (!execCommand(cmd, opt))
        showMessage(tr("Error: Google Earth execution"));

    return;
}
//---------------------------------------------------------------------------
void ConvDialog::formatChanged()
{
    updateOutputFile();
    updateEnable();
}
//---------------------------------------------------------------------------
void ConvDialog::viewOutputFile()
{
    QString file = ui->lEOutputFile->text();

    if (file.isEmpty()) return;

    viewer->setWindowTitle(file);
    viewer->show();
    viewer->read(file);
}
//---------------------------------------------------------------------------
void ConvDialog::loadOptions(QSettings &ini)
{
    ui->cBTimeSpan->setChecked(ini.value("conv/timespan", 0).toInt());
    ui->cBTimeInterval->setChecked(ini.value("conv/timeintf", 0).toInt());
    ui->dateTimeStart->setDate(ini.value("conv/timey1", QDate(2000, 1, 1)).toDate());
    ui->dateTimeStart->setTime(ini.value("conv/timeh1", QTime(0, 0, 0)).toTime());
    ui->dateTimeStop->setDate(ini.value("conv/timey2", QDate(2000, 1, 1)).toDate());
    ui->dateTimeStop->setTime(ini.value("conv/timeh2", QTime(0, 0, 0)).toTime());
    ui->sBTimeInterval->setValue(ini.value ("conv/timeint", 0.0).toDouble());
    ui->cBTrackColor->setCurrentIndex(ini.value("conv/trackcolor", 5).toInt());
    ui->cBPointColor->setCurrentIndex(ini.value("conv/pointcolor", 5).toInt());
    ui->cBOutputAltitude->setCurrentIndex(ini.value("conv/outputalt", 0).toInt());
    ui->cBOutputTime->setCurrentIndex(ini.value("conv/outputtime",0).toInt());
    ui->cBAddOffset->setChecked(ini.value("conv/addoffset", 0).toInt());
    ui->sBOffset1->setValue(ini.value("conv/offset1", 0).toDouble());
    ui->sBOffset2->setValue(ini.value("conv/offset2", 0).toDouble());
    ui->sBOffset3->setValue(ini.value("conv/offset3", 0).toDouble());
    ui->cBCompress->setChecked(ini.value("conv/compress", 0).toInt());
    ui->rBFormatKML->setChecked(ini.value("conv/format", 0).toInt());

    ui->lEGoogleEarthFile->setText(ini.value("opt/googleearthfile", GOOGLE_EARTH).toString());

    viewer->loadOptions(ini);
}
//---------------------------------------------------------------------------
void ConvDialog::saveOptions(QSettings &ini)
{
    ini.setValue ("conv/timespan", ui->cBTimeSpan->isChecked());
    ini.setValue ("conv/timey1", ui->dateTimeStart->date());
    ini.setValue ("conv/timeh1", ui->dateTimeStart->time());
    ini.setValue ("conv/timey2", ui->dateTimeStop->date());
    ini.setValue ("conv/timeh2", ui->dateTimeStop->time());
    ini.setValue ("conv/timeintf", ui->cBTimeInterval->isChecked());
    ini.setValue ("conv/timeint", ui->sBTimeInterval->text());
    ini.setValue ("conv/trackcolor", ui->cBTrackColor->currentIndex());
    ini.setValue ("conv/pointcolor", ui->cBPointColor->currentIndex());
    ini.setValue ("conv/outputalt", ui->cBOutputAltitude->currentIndex());
    ini.setValue ("conv/outputtime", ui->cBOutputTime->currentIndex());
    ini.setValue ("conv/addoffset", ui->cBAddOffset ->isChecked());
    ini.setValue ("conv/offset1", ui->sBOffset1->text());
    ini.setValue ("conv/offset2", ui->sBOffset2->text());
    ini.setValue ("conv/offset3", ui->sBOffset3->text());
    ini.setValue ("conv/compress", ui->cBCompress->isChecked());
    ini.setValue ("conv/format", ui->rBFormatKML->isChecked());

    ini.setValue("opt/googleearthfile", ui->lEGoogleEarthFile->text());

    viewer->saveOptions(ini);
}
//---------------------------------------------------------------------------
