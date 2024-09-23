//---------------------------------------------------------------------------

#include "plotmain.h"
#include "plotopt.h"
#include "refdlg.h"
#include "viewer.h"
#include "rtklib.h"
#include "helper.h"

#include <QShowEvent>
#include <QColorDialog>
#include <QFontDialog>
#include <QFileDialog>
#include <QPoint>
#include <QDebug>
#include <QFileSystemModel>
#include <QCompleter>
#include <QAction>

//---------------------------------------------------------------------------
#include "ui_plotopt.h"

//---------------------------------------------------------------------------
PlotOptDialog::PlotOptDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::PlotOptDialog)
{
    ui->setupUi(this);

    ui->cBReceiverPosition->setItemData(0, tr("Use single‐point results as receiver positions by using observation and navigation data. For moving receivers, you shall set it."), Qt::ToolTipRole);
    ui->cBReceiverPosition->setItemData(1, tr("Use uses latitude, longitude and height for static receivers as specified in the Lat/Lon/Hgt fields to the right."), Qt::ToolTipRole);
    ui->cBReceiverPosition->setItemData(2, tr("Use \"APPROX POSITION XYZ\" in RINEX observation data header as the receiver position."), Qt::ToolTipRole);

    ui->cBOrigin->setItemData(0, tr("Use first solution position"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(1, tr("Use last solution position"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(2, tr("Use the average of all solution positions"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(3, tr("Set origin based on the linearly fitted positions"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(4, tr("Use the base station position"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(5, tr("Use latitude, longitude and height specified"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(6, tr("The receiver ID is assumed as the 4-charactors of the solution file name head and the position is read from the position file"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(7, tr("Use image center"), Qt::ToolTipRole);
    ui->cBOrigin->setItemData(8, tr("Use map center"), Qt::ToolTipRole);
    for (int i = 0; i < 10; i++)
        ui->cBOrigin->setItemData(9 + i, tr("Use Waypoint %1").arg(i+1), Qt::ToolTipRole);


    refDialog = new RefDialog(this, 1);

    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lETLEFile->setCompleter(fileCompleter);
    ui->lETLESatelliteFile->setCompleter(fileCompleter);
    ui->lEShapeFile->setCompleter(fileCompleter);

    QAction * acTLEFileOpen = ui->lETLEFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLEFileOpen->setToolTip(tr("Select TLE file"));
    QAction * acTLEFileView = ui->lETLEFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acTLEFileView->setToolTip(tr("View TLE file"));

    QAction * acTLESatelliteFileOpen = ui->lETLESatelliteFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acTLESatelliteFileOpen->setToolTip(tr("Select TLE satellite file"));
    QAction * acTLESatelliteFileView = ui->lETLESatelliteFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acTLESatelliteFileView->setToolTip(tr("View TLE satellite file"));

    QAction * acShapeFileOpen = ui->lEShapeFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acShapeFileOpen->setToolTip(tr("Select shape file"));


    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &PlotOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &PlotOptDialog::reject);
    connect(ui->btnColor1, &QPushButton::clicked, this, &PlotOptDialog::color1Select);
    connect(ui->btnColor2, &QPushButton::clicked, this, &PlotOptDialog::color2Select);
    connect(ui->btnColor3, &QPushButton::clicked, this, &PlotOptDialog::color3Select);
    connect(ui->btnColor4, &QPushButton::clicked, this, &PlotOptDialog::color4Select);
    connect(ui->btnFont, &QPushButton::clicked, this, &PlotOptDialog::fontSelect);
    connect(ui->btnReferencePosition, &QPushButton::clicked, this, &PlotOptDialog::referencePositionSelect);
    connect(acTLEFileOpen, &QAction::triggered, this, &PlotOptDialog::tleFileOpen);
    connect(acTLEFileView, &QAction::triggered, this, &PlotOptDialog::tleFileView);
    connect(acTLESatelliteFileOpen, &QAction::triggered, this, &PlotOptDialog::tleSatelliteFileOpen);
    connect(acTLESatelliteFileView, &QAction::triggered, this, &PlotOptDialog::tleSatelliteFileView);
    connect(acShapeFileOpen, &QAction::triggered, this, &PlotOptDialog::shapeFileOpen);
    connect(ui->cBOrigin, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &PlotOptDialog::updateEnable);
    connect(ui->cBAutoScale, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &PlotOptDialog::updateEnable);
    connect(ui->cBReceiverPosition, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &PlotOptDialog::updateEnable);
    connect(ui->tBMColor1, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor2, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor3, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor4, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor5, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor6, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor7, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor8, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor9, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor10, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor11, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->tBMColor12, &QToolButton::clicked, this, &PlotOptDialog::markerColorSelect);
    connect(ui->cBTimeSync, &QCheckBox::clicked, this, &PlotOptDialog::updateEnable);
}
//---------------------------------------------------------------------------
void PlotOptDialog::updateConfig()
{
    elevation_mask = ui->cBElevationMask->currentText().toDouble();
    timeFormat = ui->cBTimeFormat->currentIndex();
    elevationMaskEnabled = ui->cBElevationMaskPattern->currentIndex();
}

//---------------------------------------------------------------------------
void PlotOptDialog::accept()
{
    updateConfig();
    QDialog::accept();
}
//---------------------------------------------------------------------------
void PlotOptDialog::markerColorSelect()
{
    QToolButton *button = dynamic_cast<QToolButton *>(QObject::sender());

    if (!button) return;

    QColorDialog dialog(this);
    QColor *current;

    if (button == ui->tBMColor1) current = &markerColor[0][1];
    else if (button == ui->tBMColor2) current = &markerColor[0][2];
    else if (button == ui->tBMColor3) current = &markerColor[0][3];
    else if (button == ui->tBMColor4) current = &markerColor[0][4];
    else if (button == ui->tBMColor5) current = &markerColor[0][5];
    else if (button == ui->tBMColor6) current = &markerColor[0][6];
    else if (button == ui->tBMColor7) current = &markerColor[1][1];
    else if (button == ui->tBMColor8) current = &markerColor[1][2];
    else if (button == ui->tBMColor9) current = &markerColor[1][3];
    else if (button == ui->tBMColor10) current = &markerColor[1][4];
    else if (button == ui->tBMColor11) current = &markerColor[1][5];
    else if (button == ui->tBMColor12) current = &markerColor[1][6];
    else return;
    dialog.setCurrentColor(*current);

    dialog.exec();
    if (dialog.result() != QDialog::Accepted) return;

    button->setStyleSheet(QString("background-color: %1").arg(color2String(dialog.currentColor())));
    *current = dialog.currentColor();
}
//---------------------------------------------------------------------------
void PlotOptDialog::color1Select()
{
    QColorDialog dialog(this);

    dialog.setCurrentColor(cColor[0]);
    dialog.exec();
    if (dialog.result() != QDialog::Accepted) return;
    ui->lblColor1->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(dialog.currentColor())));
    cColor[0] = dialog.currentColor();
}
//---------------------------------------------------------------------------
void PlotOptDialog::color2Select()
{
    QColorDialog dialog(this);

    dialog.setCurrentColor(cColor[1]);
    dialog.exec();
    if (dialog.result() != QDialog::Accepted) return;
    ui->lblColor2->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(dialog.currentColor())));
    cColor[1] = dialog.currentColor();
}
//---------------------------------------------------------------------------
void PlotOptDialog::color3Select()
{
    QColorDialog dialog(this);

    dialog.setCurrentColor(cColor[2]);
    dialog.exec();
    if (dialog.result() != QDialog::Accepted) return;
    ui->lblColor3->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(dialog.currentColor())));
    cColor[2] = dialog.currentColor();
}
//---------------------------------------------------------------------------
void PlotOptDialog::color4Select()
{
    QColorDialog dialog(this);

    dialog.setCurrentColor(cColor[3]);
    dialog.exec();
    if (dialog.result() != QDialog::Accepted) return;
    ui->lblColor4->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(dialog.currentColor())));
    cColor[3] = dialog.currentColor();
}
//---------------------------------------------------------------------------
void PlotOptDialog::fontSelect()
{
    QFontDialog dialog(this);

    dialog.setCurrentFont(fontOption);
    dialog.exec();

    if (dialog.result() != QDialog::Accepted) return;
    fontOption = dialog.currentFont();

    updateFont();
}
//---------------------------------------------------------------------------
void PlotOptDialog::shapeFileOpen()
{
    ui->lEShapeFile->setText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open"), ui->lEShapeFile->text())));
}
//---------------------------------------------------------------------------
void PlotOptDialog::tleFileOpen()
{
    ui->lETLEFile->setText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open"), ui->lETLEFile->text(), tr("Text Files (*.txt);;Position Files (*.pos *.snx);;All (*.*)"))));
}
//---------------------------------------------------------------------------
void PlotOptDialog::tleSatelliteFileOpen()
{
    ui->lETLESatelliteFile->setText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open"), ui->lETLESatelliteFile->text(), tr("Text Files (*.txt);;Position Files (*.pos *.snx);;All (*.*)"))));
}
//---------------------------------------------------------------------------
void PlotOptDialog::referencePositionSelect()
{
    refDialog->setRoverPosition(ui->sBReferencePosition1->value(), ui->sBReferencePosition2->value(), ui->sBReferencePosition3->value());

    refDialog->move(pos().x() + size().width() / 2 - refDialog->size().width() / 2,
            pos().y() + size().height() / 2 - refDialog->size().height() / 2);

    refDialog->exec();
    if (refDialog->result() != QDialog::Accepted) return;

    ui->sBReferencePosition1->setValue(refDialog->getPosition()[0]);
    ui->sBReferencePosition2->setValue(refDialog->getPosition()[1]);
    ui->sBReferencePosition3->setValue(refDialog->getPosition()[2]);
}
//---------------------------------------------------------------------------
void PlotOptDialog::updateFont()
{
    ui->lblFontName->setFont(fontOption);
    ui->lblFontName->setText(fontOption.family() + " " + QString::number(fontOption.pointSize()) + " pt");
}
//---------------------------------------------------------------------------
void PlotOptDialog::updateEnable()
{
    ui->sBReferencePosition1->setEnabled(ui->cBOrigin->currentIndex() == 5 || ui->cBReceiverPosition->currentIndex() == 1);
    ui->sBReferencePosition2->setEnabled(ui->cBOrigin->currentIndex() == 5 || ui->cBReceiverPosition->currentIndex() == 1);
    ui->sBReferencePosition3->setEnabled(ui->cBOrigin->currentIndex() == 5 || ui->cBReceiverPosition->currentIndex() == 1);
    ui->lblReferencePosition->setEnabled(ui->cBOrigin->currentIndex() == 5 || ui->cBOrigin->currentIndex() == 6 || ui->cBReceiverPosition->currentIndex() == 1);
    ui->btnReferencePosition->setEnabled(ui->cBOrigin->currentIndex() == 5 || ui->cBOrigin->currentIndex() == 6 || ui->cBReceiverPosition->currentIndex() == 1);
    ui->sBTimeSyncPort->setEnabled(ui->cBTimeSync->isChecked());
}
//---------------------------------------------------------------------------
void PlotOptDialog::tleFileView()
{
    TextViewer *viewer;
    QString file = ui->lETLEFile->text();

    if (file == "") return;
    viewer = new TextViewer(this);
    viewer->setWindowTitle(file);
    viewer->show();
    viewer->read(file);
}
//---------------------------------------------------------------------------
void PlotOptDialog::tleSatelliteFileView()
{
    TextViewer *viewer;
    QString file = ui->lETLESatelliteFile->text();

    if (file == "") return;
    viewer = new TextViewer(this);
    viewer->setWindowTitle(file);
    viewer->show();
    viewer->read(file);
}
//---------------------------------------------------------------------------
int PlotOptDialog::getTimeFormat()
{
    return timeFormat;
}
//---------------------------------------------------------------------------
int PlotOptDialog::getLatLonFormat()
{
    return ui->cBLatLonFormat->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowStats()
{
    return ui->cBShowStats->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowSlip()
{
    return ui->cBShowSlip->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowHalfC(){
    return ui->cBShowHalfC->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowEphemeris()
{
    return ui->cBShowEphemeris->currentIndex();
}
//---------------------------------------------------------------------------
double PlotOptDialog::getElevationMask()
{
    return elevation_mask;
}
//---------------------------------------------------------------------------
int PlotOptDialog::getElevationMaskEnabled()
{
    return elevationMaskEnabled;
}
//---------------------------------------------------------------------------
int PlotOptDialog::getHideLowSatellites()
{
    return ui->cBHideLowSatellites->currentIndex();
}
//---------------------------------------------------------------------------
double PlotOptDialog::getMaxDop()
{
    return ui->cBMaxDop->currentText().toDouble();
}
//---------------------------------------------------------------------------
double PlotOptDialog::getMaxMP()
{
    return ui->cBMaxMP->currentText().toDouble();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getNavSys()
{
    return (ui->cBNavSys1->isChecked() ? SYS_GPS : 0) |
           (ui->cBNavSys2->isChecked() ? SYS_GLO : 0) |
           (ui->cBNavSys3->isChecked() ? SYS_GAL : 0) |
           (ui->cBNavSys4->isChecked() ? SYS_QZS : 0) |
           (ui->cBNavSys5->isChecked() ? SYS_SBS : 0) |
           (ui->cBNavSys6->isChecked() ? SYS_CMP : 0) |
           (ui->cBNavSys7->isChecked() ? SYS_IRN : 0);
}
//---------------------------------------------------------------------------
QString PlotOptDialog::getExcludedSatellites()
{
    return ui->lEExcludedSatellites->text();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowError()
{
    return ui->cBShowError->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowArrow()
{
    return ui->cBShowArrow->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowGridLabel()
{
    return ui->cBShowGridLabel->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowLabel()
{
    return ui->cBShowLabel->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowCompass()
{
    return ui->cBShowCompass->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getShowScale()
{
    return ui->cBShowScale->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getAutoScale()
{
    return ui->cBAutoScale->currentIndex();
}
//---------------------------------------------------------------------------
double PlotOptDialog::getYRange()
{
    QStringList tokens = ui->cBYRange->currentText().split(' ', Qt::SkipEmptyParts);
    if (tokens.length() == 2) {
        bool ok;
        int range = tokens.at(0).toInt(&ok);
        QString unit = tokens.at(1);

        if (ok) {
            if (unit == "cm") range *= 0.01;
            else if (unit == "km") range *= 1000.0;
            return range;
        }
    }
    return NAN;
}
//---------------------------------------------------------------------------
int PlotOptDialog::getRtBufferSize()
{
    return ui->sBBufferSize->value();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getTimeSyncOut()
{
    return ui->cBTimeSync->isChecked();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getTimeSyncPort()
{
    return ui->sBTimeSyncPort->value();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getOrigin()
{
    return ui->cBOrigin->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getReceiverPosition()
{
    return ui->cBReceiverPosition->currentIndex();
}
//---------------------------------------------------------------------------
double* PlotOptDialog::getOoPosition()
{
    static double ooPos[3];

    ooPos[0] = ui->sBReferencePosition1->value() * D2R;
    ooPos[1] = ui->sBReferencePosition2->value() * D2R;
    ooPos[2] = ui->sBReferencePosition3->value();

    return ooPos;
}
//---------------------------------------------------------------------------
int PlotOptDialog::getPlotStyle()
{
    return ui->cBPlotStyle->currentIndex();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getMarkSize()
{
    int marks[] = { 1, 2, 3, 4, 5, 10, 15, 20 };

    return marks[ui->cBMarkSize->currentIndex()];
}
//---------------------------------------------------------------------------
int PlotOptDialog::getAnimationCycle()
{
    return ui->cBAnimationCycle->currentText().toInt();
}
//---------------------------------------------------------------------------
int PlotOptDialog::getRefreshCycle()
{
    return ui->sBRefreshCycle->value();
}
//---------------------------------------------------------------------------
QString PlotOptDialog::getShapeFile()
{
    return ui->lEShapeFile->text();
}
//---------------------------------------------------------------------------
QString PlotOptDialog::getTleFile()
{
    return ui->lETLEFile->text();
}
//---------------------------------------------------------------------------
QString PlotOptDialog::getRinexOptions()
{
    return ui->lERinexOptions->text();
}
//---------------------------------------------------------------------------
QString PlotOptDialog::getTleSatelliteFile()
{
    return ui->lETLESatelliteFile->text();
}
//---------------------------------------------------------------------------
QFont PlotOptDialog::getFont()
{
    return fontOption;
}
//---------------------------------------------------------------------------
QColor PlotOptDialog::getCColor(int i)
{
    if ((i >= 0) && (i < 4))
        return cColor[i];
    return QColor();
}
//---------------------------------------------------------------------------
QColor PlotOptDialog::getMarkerColor(int i, int j)
{
    if ((i >= 0) && (i < 2) && (j >= 0) && (j < 8))
        return markerColor[i][j];
    return QColor();
}
//---------------------------------------------------------------------------
void PlotOptDialog::loadOptions(QSettings & settings)
{
    int marks[] = { 1, 2, 3, 4, 5, 10, 15, 20 };
    for (int i = 0; i < 8; i++)
        if (marks[i] == settings.value("plot/marksize", 2).toInt()) ui->cBMarkSize->setCurrentIndex(i);

    ui->cBTimeFormat->setCurrentIndex(settings.value("plot/timelabel", 1).toInt());
    ui->cBLatLonFormat->setCurrentIndex(settings.value("plot/latlonfmt", 0).toInt());
    ui->cBAutoScale->setCurrentIndex(settings.value("plot/autoscale", 1).toInt());
    ui->cBShowStats->setCurrentIndex(settings.value("plot/showstats", 0).toInt());
    ui->cBShowArrow->setCurrentIndex(settings.value("plot/showarrow", 0).toInt());
    ui->cBShowSlip->setCurrentIndex(settings.value("plot/showslip", 0).toInt());
    ui->cBShowHalfC->setCurrentIndex(settings.value("plot/showhalfc", 0).toInt());
    ui->cBShowError->setCurrentIndex(settings.value("plot/showerr", 0).toInt());
    ui->cBShowEphemeris->setCurrentIndex(settings.value("plot/showeph", 0).toInt());
    ui->cBShowLabel->setCurrentIndex(settings.value("plot/showlabel", 1).toInt());
    ui->cBShowGridLabel->setCurrentIndex(settings.value("plot/showglabel", 1).toInt());
    ui->cBShowScale->setCurrentIndex(settings.value("plot/showscale", 1).toInt());
    ui->cBShowCompass->setCurrentIndex(settings.value("plot/showcompass", 0).toInt());
    ui->cBPlotStyle->setCurrentIndex(settings.value("plot/plotstyle", 0).toInt());


    ui->cBElevationMask->setCurrentText(QString::number(settings.value("plot/elmask", 0.0).toDouble()));
    ui->cBMaxDop->setCurrentText(QString::number(settings.value("plot/maxdop", 30.0).toDouble()));
    ui->cBMaxMP->setCurrentText(QString::number(settings.value("plot/maxmp", 10.0).toDouble()));
    ui->cBOrigin->setCurrentIndex(settings.value("plot/orgin", 2).toInt());
    ui->cBReceiverPosition->setCurrentIndex(settings.value("plot/rcvpos", 0).toInt());
    ui->sBReferencePosition1->setValue(settings.value("plot/oopos1", 0).toDouble() * R2D);
    ui->sBReferencePosition2->setValue(settings.value("plot/oopos2", 0).toDouble() * R2D);
    ui->sBReferencePosition3->setValue(settings.value("plot/oopos3", 0).toDouble());
    int navSys = settings.value("plot/navsys", SYS_ALL).toInt();
    ui->cBNavSys1->setChecked(navSys & SYS_GPS);
    ui->cBNavSys2->setChecked(navSys & SYS_GLO);
    ui->cBNavSys3->setChecked(navSys & SYS_GAL);
    ui->cBNavSys4->setChecked(navSys & SYS_QZS);
    ui->cBNavSys5->setChecked(navSys & SYS_SBS);
    ui->cBNavSys6->setChecked(navSys & SYS_CMP);
    ui->cBNavSys7->setChecked(navSys & SYS_IRN);
    ui->cBAnimationCycle->setCurrentText(QString::number(settings.value("plot/animcycle", 10).toInt()));
    ui->sBRefreshCycle->setValue(settings.value("plot/refcycle", 100).toInt());
    ui->cBHideLowSatellites->setCurrentIndex(settings.value("plot/hidelowsat", 0).toInt());
    ui->cBElevationMaskPattern->setCurrentText(QString::number(settings.value("plot/elmaskp", 0).toInt()));
    ui->lEExcludedSatellites->setText(settings.value("plot/exsats", "").toString());
    ui->sBBufferSize->setValue(settings.value("plot/rtbuffsize", 10800).toInt());
    ui->cBTimeSync->setChecked(settings.value("plot/timesyncout", 0).toInt());
    ui->sBTimeSyncPort->setValue(settings.value("plot/timesyncport", 10071).toInt());
    ui->lERinexOptions->setText(settings.value("plot/rnxopts", "").toString());
    ui->lEShapeFile->setText(settings.value("plot/shapefile", "").toString());
    ui->lETLEFile->setText(settings.value("plot/tlefile", "").toString());
    ui->lETLESatelliteFile->setText(settings.value("plot/tlesatfile", "").toString());

    for (int i = 0; i < ui->cBYRange->count(); i++) {
        double range;
        bool ok;
        QString unit;

        QString s = ui->cBYRange->itemText(i);
        QStringList tokens = s.split(' ', Qt::SkipEmptyParts);

        if (tokens.length() != 2) continue;

        range = tokens.at(0).toInt(&ok);
        if (!ok) continue;

        unit = tokens.at(1);
        if (unit == "cm") range *= 0.01;
        else if (unit == "km") range *= 1000.0;

        if (range == settings.value("plot/yrange", 5.0).toDouble()) {
            ui->cBYRange->setCurrentIndex(i);
            break;
        }
    }

    fontOption.setFamily(settings.value("plot/fontname", "Tahoma").toString());
    fontOption.setPointSize(settings.value("plot/fontsize", 8).toInt());
    updateFont();

    markerColor[0][0] = settings.value("plot/mcolor0", QColor(0xc0, 0xc0, 0xc0)).value<QColor>();
    markerColor[0][1] = settings.value("plot/mcolor1", QColor(Qt::darkGreen)).value<QColor>();
    markerColor[0][2] = settings.value("plot/mcolor2", QColor(0xff, 0xAA, 0x00)).value<QColor>();
    markerColor[0][3] = settings.value("plot/mcolor3", QColor(0xff, 0x00, 0xff)).value<QColor>();
    markerColor[0][4] = settings.value("plot/mcolor4", QColor(Qt::darkBlue)).value<QColor>();
    markerColor[0][5] = settings.value("plot/mcolor5", QColor(Qt::red)).value<QColor>();
    markerColor[0][6] = settings.value("plot/mcolor6", QColor(0x80, 0x80, 0x00)).value<QColor>();
    markerColor[0][7] = settings.value("plot/mcolor7", QColor(Qt::gray)).value<QColor>();
    markerColor[1][0] = settings.value("plot/mcolor8", QColor(0xc0, 0xc0, 0xc0)).value<QColor>();
    markerColor[1][1] = settings.value("plot/mcolor9", QColor(0x00, 0x40, 0x80)).value<QColor>();
    markerColor[1][2] = settings.value("plot/mcolor10", QColor(0x80, 0x80, 0x00)).value<QColor>();
    markerColor[1][3] = settings.value("plot/mcolor11", QColor(0x80, 0x00, 0xff)).value<QColor>();
    markerColor[1][4] = settings.value("plot/mcolor12", QColor(0x00, 0x80, 0xff)).value<QColor>();
    markerColor[1][5] = settings.value("plot/mcolor13", QColor(0xff, 0x80, 0x80)).value<QColor>();
    markerColor[1][6] = settings.value("plot/mcolor14", QColor(0x80, 0x80, 0xff)).value<QColor>();
    markerColor[1][7] = settings.value("plot/mcolor15", QColor(Qt::gray)).value<QColor>();

    ui->tBMColor1->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[0][1])));
    ui->tBMColor2->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[0][2])));
    ui->tBMColor3->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[0][3])));
    ui->tBMColor4->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[0][4])));
    ui->tBMColor5->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[0][5])));
    ui->tBMColor6->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[0][6])));
    ui->tBMColor7->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[1][1])));
    ui->tBMColor8->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[1][2])));
    ui->tBMColor9->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[1][3])));
    ui->tBMColor10->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[1][4])));
    ui->tBMColor11->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[1][5])));
    ui->tBMColor12->setStyleSheet(QString("background-color: %1;").arg(color2String(markerColor[1][6])));

    cColor[0] = settings.value("plot/color1", QColor(Qt::white)).value<QColor>();
    cColor[1] = settings.value("plot/color2", QColor(0xc0, 0xc0, 0xc0)).value<QColor>();
    cColor[2] = settings.value("plot/color3", QColor(Qt::black)).value<QColor>();
    cColor[3] = settings.value("plot/color4", QColor(0xc0, 0xc0, 0xc0)).value<QColor>();
    ui->lblColor1->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(cColor[0])));
    ui->lblColor2->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(cColor[1])));
    ui->lblColor3->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(cColor[2])));
    ui->lblColor4->setStyleSheet(QString("QLabel {background-color: %1;}").arg(color2String(cColor[3])));


    refDialog->stationPositionFile = settings.value("plot/staposfile", "").toString();

    updateEnable();
    updateConfig();
}
//---------------------------------------------------------------------------
void PlotOptDialog::saveOptions(QSettings & settings)
{

    settings.setValue("plot/timelabel", ui->cBTimeFormat->currentIndex());
    settings.setValue("plot/latlonfmt", ui->cBLatLonFormat->currentIndex());
    settings.setValue("plot/autoscale", ui->cBAutoScale->currentIndex());
    settings.setValue("plot/showstats", ui->cBShowStats->currentIndex());
    settings.setValue("plot/showlabel", ui->cBShowLabel->currentIndex());
    settings.setValue("plot/showglabel", ui->cBShowGridLabel->currentIndex());
    settings.setValue("plot/showcompass", ui->cBShowCompass->currentIndex());
    settings.setValue("plot/showscale", ui->cBShowScale->currentIndex());
    settings.setValue("plot/showarrow", ui->cBShowArrow->currentIndex());
    settings.setValue("plot/showslip", ui->cBShowSlip->currentIndex());
    settings.setValue("plot/showhalfc", ui->cBShowHalfC->currentIndex());
    settings.setValue("plot/showerr", ui->cBShowError->currentIndex());
    settings.setValue("plot/showeph", ui->cBShowEphemeris->currentIndex());
    settings.setValue("plot/plotstyle", ui->cBPlotStyle->currentIndex());
    settings.setValue("plot/marksize", getMarkSize());
    settings.setValue("plot/navsys", (ui->cBNavSys1->isChecked() ? SYS_GPS : 0) |
                                     (ui->cBNavSys2->isChecked() ? SYS_GLO : 0) |
                                     (ui->cBNavSys3->isChecked() ? SYS_GAL : 0) |
                                     (ui->cBNavSys4->isChecked() ? SYS_QZS : 0) |
                                     (ui->cBNavSys5->isChecked() ? SYS_SBS : 0) |
                                     (ui->cBNavSys6->isChecked() ? SYS_CMP : 0) |
                                     (ui->cBNavSys7->isChecked() ? SYS_IRN : 0));
    settings.setValue("plot/animcycle", ui->cBAnimationCycle->currentText().toInt());
    settings.setValue("plot/refcycle", ui->sBRefreshCycle->value());
    settings.setValue("plot/hidelowsat", ui->cBHideLowSatellites->currentIndex());
    settings.setValue("plot/exsats", ui->lEExcludedSatellites->text());
    settings.setValue("plot/timesyncout", ui->cBTimeSync->isChecked());
    settings.setValue("plot/timesyncport", ui->sBTimeSyncPort->value());
    settings.setValue("plot/rnxopts", ui->lERinexOptions->text());

    settings.setValue("plot/mcolor0", markerColor[0][0]);
    settings.setValue("plot/mcolor1", markerColor[0][1]);
    settings.setValue("plot/mcolor2", markerColor[0][2]);
    settings.setValue("plot/mcolor3", markerColor[0][3]);
    settings.setValue("plot/mcolor4", markerColor[0][4]);
    settings.setValue("plot/mcolor5", markerColor[0][5]);
    settings.setValue("plot/mcolor6", markerColor[0][6]);
    settings.setValue("plot/mcolor7", markerColor[0][7]);
    settings.setValue("plot/mcolor8", markerColor[0][0]);
    settings.setValue("plot/mcolor9", markerColor[1][1]);
    settings.setValue("plot/mcolor10", markerColor[1][2]);
    settings.setValue("plot/mcolor11", markerColor[1][3]);
    settings.setValue("plot/mcolor12", markerColor[1][4]);
    settings.setValue("plot/mcolor13", markerColor[1][5]);
    settings.setValue("plot/mcolor14", markerColor[1][6]);
    settings.setValue("plot/mcolor15", markerColor[1][7]);

    settings.setValue("plot/color1", cColor[0]);
    settings.setValue("plot/color2", cColor[1]);
    settings.setValue("plot/color3", cColor[2]);
    settings.setValue("plot/color4", cColor[3]);

    settings.setValue("plot/elmask", ui->cBElevationMask->currentText().toDouble());
    settings.setValue("plot/elmaskp", ui->cBElevationMaskPattern->currentIndex());
    settings.setValue("plot/rtbuffsize", ui->sBBufferSize->value());
    settings.setValue("plot/maxdop", ui->cBMaxDop->currentText().toDouble());
    settings.setValue("plot/maxmp", ui->cBMaxMP->currentText().toDouble());
    settings.setValue("plot/orgin", ui->cBOrigin->currentIndex());
    settings.setValue("plot/rcvpos", ui->cBReceiverPosition->currentIndex());
    settings.setValue("plot/oopos1", ui->sBReferencePosition1->value() * D2R);
    settings.setValue("plot/oopos2", ui->sBReferencePosition2->value() * D2R);
    settings.setValue("plot/oopos3", ui->sBReferencePosition3->value());
    settings.setValue("plot/shapefile", ui->lEShapeFile->text());
    settings.setValue("plot/tlefile", ui->lETLEFile->text());
    settings.setValue("plot/tlesatfile", ui->lETLESatelliteFile->text());
    settings.setValue("plot/fontname", fontOption.family());
    settings.setValue("plot/fontsize", fontOption.pointSize());
    settings.setValue("plot/staposfile", refDialog->stationPositionFile);

    settings.setValue("plot/yrange", getYRange());
}
//---------------------------------------------------------------------------
