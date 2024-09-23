//--------------------------------------------------------------------------

#include <QShowEvent>
#include <QKeyEvent>
#include <QTextStream>
#include <QFile>
#include <QMessageBox>

#include "plotmain.h"
#include "mapoptdlg.h"

#include "ui_mapoptdlg.h"

#include "rtklib.h"

#define INC_LATLON  0.000001
#define INC_SCALE   0.0001

//---------------------------------------------------------------------------
MapOptDialog::MapOptDialog( Plot *plt, QWidget* parent)
    : QDialog(parent), ui(new Ui::MapAreaDialog), plot(plt)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &MapOptDialog::reject);
    connect(ui->btnSaveTag, &QPushButton::clicked, this, &MapOptDialog::saveTag);
    connect(ui->btnUpdate, &QPushButton::clicked, this, &MapOptDialog::updateMap);
    connect(ui->cBScaleEqual, &QCheckBox::clicked, this, &MapOptDialog::scaleEqualChanged);
}
//---------------------------------------------------------------------------
void MapOptDialog::showEvent(QShowEvent* event)
{
    if (event->spontaneous()) return;

	updateEnable();
}
//---------------------------------------------------------------------------
void MapOptDialog::saveTag()
{
    if (plot->getMapImageFileName().isEmpty()) return;
    QString file = plot->getMapImageFileName() + ".tag";

    QFile fp(file);
    if (!fp.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (QMessageBox::question(this, file, tr("File exists. Overwrite it?")) != QMessageBox::Yes) return;
	}
    QTextStream out(&fp);
    out << QString("%% map image tag file: rtkplot %1 %2\n\n").arg(VER_RTKLIB, PATCH_LEVEL);
    out << QString("scalex  = %1\n").arg(getMapScaleX(), 0, 'g', 6);
    out << QString("scaley  = %1\n").arg(getMapScaleEqual() ? getMapScaleX() : getMapScaleY(), 0, 'g', 6);
    out << QString("scaleeq = %1\n").arg(getMapScaleEqual());
    out << QString("lat     = %1\n").arg(getMapLatitude(), 0, 'g', 9);
    out << QString("lon     = %1\n").arg(getMapLongitude(), 0, 'g', 9);

}
// read map tag data --------------------------------------------------------
void MapOptDialog::readMapTag(const QString &file)
{
    QFile fp(file + ".tag");
    QByteArray buff;

    trace(3, "readMapTag\n");

    if (!(fp.open(QIODevice::ReadOnly))) return;

    // set default values
    ui->sBScaleX->setValue(1);
    ui->sBScaleY->setValue(1);
    ui->cBScaleEqual->setChecked(false);
    ui->sBLatitude->setValue(0);
    ui->sBLongitude->setValue(0);

    // read tags
    while (!fp.atEnd()) {
        buff = fp.readLine();
        if (buff.at(0) == '\0' || buff.at(0) == '%' || buff.at(0) == '#') continue;
        QList<QByteArray> tokens = buff.split('=');
        if (tokens.size() < 2) continue;

        if (tokens.at(0).simplified() == "scalex") ui->sBScaleX->setValue(tokens.at(1).toDouble());
        else if (tokens.at(0).simplified() == "scaley") ui->sBScaleY->setValue(tokens.at(1).toDouble());
        else if (tokens.at(0).simplified() == "scaleeq") ui->cBScaleEqual->setChecked(tokens.at(1).toInt());
        else if (tokens.at(0).simplified() == "lat") ui->sBLatitude->setValue(tokens.at(1).toDouble());
        else if (tokens.at(0).simplified() == "lon") ui->sBLongitude->setValue(tokens.at(1).toDouble());
    }

    setWindowTitle(file);
}
//---------------------------------------------------------------------------
void MapOptDialog::btnCenterClicked()
{
    double rr[3], pos[3];
    if (!plot->getCenterPosition(rr)) return;

	ecef2pos(rr,pos);

    ui->sBLatitude->setValue(pos[0]*R2D);
    ui->sBLongitude->setValue(pos[1]*R2D);

	updateMap();
}
//---------------------------------------------------------------------------
void MapOptDialog::scaleEqualChanged()
{
	updateMap();
	updateEnable();
}
//---------------------------------------------------------------------------
void MapOptDialog::updateMap()
{
    plot->updatePlot();
}
//---------------------------------------------------------------------------
void MapOptDialog::updateEnable()
{
    ui->sBScaleY->setEnabled(!ui->cBScaleEqual->isChecked());
}
//---------------------------------------------------------------------------
void MapOptDialog::setMapSize(const QImage &mapImage)
{
    ui->sBMapSize1->setValue(mapImage.width());
    ui->sBMapSize2->setValue(mapImage.height());
}
//---------------------------------------------------------------------------
int MapOptDialog::getMapSize(int dim)
{
    if (dim == 0)
        return ui->sBMapSize1->value();
    else if (dim == 1)
        return ui->sBMapSize2->value();
    return 0;
}
//---------------------------------------------------------------------------
bool MapOptDialog::getMapScaleEqual()
{
    return ui->cBScaleEqual->isChecked();
}
//---------------------------------------------------------------------------
double MapOptDialog::getMapScaleX()
{
    return ui->sBScaleX->value();
}
//---------------------------------------------------------------------------
double MapOptDialog::getMapScaleY()
{
    return ui->sBScaleY->value();
}
//---------------------------------------------------------------------------
double MapOptDialog::getMapLatitude()
{
    return ui->sBLatitude->value();
}
//---------------------------------------------------------------------------
double MapOptDialog::getMapLongitude()
{
    return ui->sBLongitude->value();
}
//---------------------------------------------------------------------------
