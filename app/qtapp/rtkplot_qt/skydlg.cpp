//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QMessageBox>
#include <QFileDialog>

#include "rtklib.h"

#include "plotmain.h"
#include "skydlg.h"

#include "ui_skydlg.h"

//---------------------------------------------------------------------------
SkyImgDialog::SkyImgDialog(Plot *_plot, QWidget *parent)
    : QDialog(parent), plot(_plot), ui(new Ui::SkyImgDialog)
{
    ui->setupUi(this);

    skyScaleR = 240;

    connect(ui->btnClose, &QPushButton::clicked, this, &SkyImgDialog::accept);
    connect(ui->btnGenMask, &QPushButton::clicked, this, &SkyImgDialog::generateMask);
    connect(ui->btnLoad, &QPushButton::clicked, this, &SkyImgDialog::loadSkyImageTag);
    connect(ui->btnSave, &QPushButton::clicked, this, &SkyImgDialog::saveSkyImageTag);
    connect(ui->cBSkyResample, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &SkyImgDialog::updateSky);
    connect(ui->cBSkyElevationMask, &QCheckBox::clicked, this, &SkyImgDialog::updateSky);
    connect(ui->gBSkyDestCorrection, &QGroupBox::clicked, this, &SkyImgDialog::updateSkyEnabled);
    connect(ui->cBSkyFlip, &QCheckBox::clicked, this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyCenter1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyCenter2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyScale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkySize1, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkySize2, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyFOV1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyFOV2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyFOV3, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyBinThres1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyBinThres2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->gBSkyBinarize, &QGroupBox::clicked, this, &SkyImgDialog::updateSkyEnabled);
    connect(ui->sBSkyDest1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest3, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest4, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest5, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest6, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest7, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest8, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);
    connect(ui->sBSkyDest9, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SkyImgDialog::updateSky);

    updateEnable();
}
//---------------------------------------------------------------------------
void SkyImgDialog::saveSkyImageTag()
{
    QFile fp;
    QString file = plot->getSkyImageFileName();

    if (file.isEmpty()) return;

	updateSky();

    file = file + ".tag";
    fp.setFileName(file);
    if (QFile::exists(file))
        if (QMessageBox::question(this, file, tr("File exists. Overwrite it?")) != QMessageBox::Yes) return;
    if (!fp.open(QIODevice::WriteOnly)) return;

    QString data;
    data = QString("%% sky image tag file: rtkplot %1 %2\n\n").arg(VER_RTKLIB, PATCH_LEVEL);
    data += QString("centx   = %1\n").arg(getSkyCenter()[0], 0, 'g', 6);
    data += QString("centy   = %1\n").arg(getSkyCenter()[1], 0, 'g', 6);
    data += QString("scale   = %1\n").arg(getSkyScale(), 0, 'g', 6);
    data += QString("roll    = %1\n").arg(getSkyFOV()[0], 0, 'g', 6);
    data += QString("pitch   = %1\n").arg(getSkyFOV()[1], 0, 'g', 6);
    data += QString("yaw     = %1\n").arg(getSkyFOV()[2], 0, 'g', 6);
    data += QString("destcorr= %1\n").arg(getSkyDistortionCorrection());
    data += QString("resample= %1\n").arg(getSkyResample());
    data += QString("flip    = %1\n").arg(getSkyFlip());
    data += QString("dest    = %1 %2 %3 %4 %5 %6 %7 %8 %9\n")
                .arg(getSkyDistortion(1), 0, 'g', 6).arg(getSkyDistortion(2), 0, 'g', 6).arg(getSkyDistortion(3), 0, 'g', 6).arg(getSkyDistortion(4), 0, 'g', 6)
                .arg(getSkyDistortion(5), 0, 'g', 6).arg(getSkyDistortion(6), 0, 'g', 6).arg(getSkyDistortion(7), 0, 'g', 6).arg(getSkyDistortion(8), 0, 'g', 6)
                .arg(getSkyDistortion(9), 0, 'g', 6);
    data += QString("elmask  = %1\n").arg(getSkyElevationMask());
    data += QString("binarize= %1\n").arg(getSkyBinarize());
    data += QString("binthr1 = %1\n").arg(getSkyBinThres1(), 0, 'f', 2);
    data += QString("binthr2 = %1\n").arg(getSkyBinThres2(), 0, 'f', 2);
    fp.write(data.toLatin1());
}
//---------------------------------------------------------------------------
void SkyImgDialog::updateSky()
{
    plot->updateSky();
}
//---------------------------------------------------------------------------
void SkyImgDialog::updateEnable()
{
    ui->sBSkyDest1->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest2->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest3->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest4->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest5->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest6->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest7->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest8->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyDest9->setEnabled(ui->gBSkyDestCorrection->isChecked());
    ui->sBSkyBinThres1->setEnabled(ui->gBSkyBinarize->isChecked());
    ui->sBSkyBinThres2->setEnabled(ui->gBSkyBinarize->isChecked());
    ui->btnGenMask->setEnabled(ui->gBSkyBinarize->isChecked());
}
//---------------------------------------------------------------------------
void SkyImgDialog::updateSkyEnabled()
{
	updateSky();
	updateEnable();
}
//---------------------------------------------------------------------------
void SkyImgDialog::loadSkyImageTag()
{
    readSkyTag(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open Tag"), plot->getSkyImageFileName(), tr("Tag File (*.tag);;All (*.*)"))));

    updateSky();
}
//---------------------------------------------------------------------------
void SkyImgDialog::generateMask()
{
    plot->generateElevationMaskFromSkyImage();
}
// update value for image ---------------------------------------------------
void SkyImgDialog::setImage(QImage &image, double w, double h)
{
    QDoubleSpinBox *skyDistortion[] = {ui->sBSkyDest1, ui->sBSkyDest2, ui->sBSkyDest3, ui->sBSkyDest4, ui->sBSkyDest5, ui->sBSkyDest6, ui->sBSkyDest7, ui->sBSkyDest8, ui->sBSkyDest9};

    skySize[0] = image.width();
    skySize[1] = image.height();
    ui->sBSkySize1->setValue(w);
    ui->sBSkySize2->setValue(h);
    ui->sBSkyCenter1->setMinimum(0);
    ui->sBSkyCenter1->setMaximum(w);
    ui->sBSkyCenter1->setValue(w / 2.0);
    ui->sBSkyCenter2->setMinimum(0);
    ui->sBSkyCenter2->setMaximum(h);
    ui->sBSkyCenter2->setValue(h / 2.0);
    ui->sBSkyFOV1->setValue(0);
    ui->sBSkyFOV2->setValue(0);
    ui->sBSkyFOV3->setValue(0);
    ui->sBSkyScale->setValue(h / 2.0);
    skyScaleR = getSkyScale() * image.width() / w;
    ui->gBSkyDestCorrection->setChecked(false);
    ui->cBSkyResample->setCurrentIndex(0);
    ui->cBSkyFlip->setChecked(false);
    ui->cBSkyElevationMask->setChecked(true);;
    for (int i = 0; i < 9; i++) skyDistortion[i]->setValue(0.0);
}
// read sky tag data --------------------------------------------------------
void SkyImgDialog::readSkyTag(const QString &file)
{
    QDoubleSpinBox *skyDistortion[] = {ui->sBSkyDest1, ui->sBSkyDest2, ui->sBSkyDest3, ui->sBSkyDest4, ui->sBSkyDest5, ui->sBSkyDest6, ui->sBSkyDest7, ui->sBSkyDest8, ui->sBSkyDest9};
    QFile fp(file);
    QByteArray buff;

    trace(3, "readSkyTag\n");

    if (!fp.open(QIODevice::ReadOnly)) return;

    while (!fp.atEnd()) {
        buff = fp.readLine();
        if (buff.at(0) == '\0' || buff.at(0) == '%' || buff.at(0) == '#') continue;
        QList<QByteArray> tokens = buff.split('=');
        if (tokens.size() < 2) continue;

        if (tokens.at(0).simplified() == "centx") {
            ui->sBSkyCenter1->setValue(tokens.at(1).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "centy") {
            ui->sBSkyCenter2->setValue(tokens.at(1).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "scale") {
            ui->sBSkyScale->setValue(tokens.at(1).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "roll") {
            ui->sBSkyFOV1->setValue(tokens.at(1).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "pitch") {
            ui->sBSkyFOV2->setValue(tokens.at(1).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "yaw") {
            ui->sBSkyFOV3->setValue(tokens.at(1).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "destcorr") {
            ui->gBSkyDestCorrection->setChecked(tokens.at(1).simplified().toInt());
        } else if (tokens.at(0).simplified() == "elmask") {
            ui->cBSkyElevationMask->setChecked(tokens.at(1).simplified().toInt());
        } else if (tokens.at(0).simplified() == "resample") {
            ui->cBSkyResample->setCurrentIndex(tokens.at(1).simplified().toInt());
        } else if (tokens.at(0).simplified() == "flip") {
            ui->cBSkyFlip->setChecked(tokens.at(1).simplified().toInt());
        } else if (tokens.at(0).simplified() == "dest") {
            QList<QByteArray> t = tokens.at(1).simplified().split(' ');
            for (int i = 0; i < 9 && i < t.size(); i++)
                skyDistortion[i]->setValue(t.at(i).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "binarize") {
            ui->gBSkyBinarize->setChecked(tokens.at(1).simplified().toInt());
        } else if (tokens.at(0).simplified() == "binthr1") {
            ui->sBSkyBinThres1->setValue(tokens.at(1).simplified().toDouble());
        } else if (tokens.at(0).simplified() == "binthr2") {
            ui->sBSkyBinThres2->setValue(tokens.at(1).simplified().toDouble());
        }
    }
    updateSkyEnabled();
}
//---------------------------------------------------------------------------
int* SkyImgDialog::getSkySize()
{
    return skySize;
}
//---------------------------------------------------------------------------
bool SkyImgDialog::getSkyDistortionCorrection()
{
    return ui->gBSkyDestCorrection->isChecked();
}
//---------------------------------------------------------------------------
bool SkyImgDialog::getSkyElevationMask()
{
    return ui->cBSkyElevationMask->isChecked();
}
//---------------------------------------------------------------------------
int SkyImgDialog::getSkyResample()
{
    return ui->cBSkyResample->currentIndex();
}
//---------------------------------------------------------------------------
bool SkyImgDialog::getSkyFlip()
{
    return ui->cBSkyFlip->isChecked();
}
//---------------------------------------------------------------------------
bool SkyImgDialog::getSkyBinarize()
{
    return ui->gBSkyBinarize->isChecked();
}
//---------------------------------------------------------------------------
double *SkyImgDialog::getSkyCenter()
{
    static double center[2];
    center[0] = ui->sBSkyCenter1->value();
    center[1] = ui->sBSkyCenter2->value();

    return center;
}
//---------------------------------------------------------------------------
double SkyImgDialog::getSkyScale()
{
    return ui->sBSkyScale->value();
}
//---------------------------------------------------------------------------
double SkyImgDialog::getSkyScaleR()
{
    return skyScaleR;
}
//---------------------------------------------------------------------------
double *SkyImgDialog::getSkyFOV()
{
    static double FOV[3];

    FOV[0] = ui->sBSkyFOV1->value();
    FOV[1] = ui->sBSkyFOV2->value();
    FOV[2] = ui->sBSkyFOV3->value();

    return FOV;
}
//---------------------------------------------------------------------------
double SkyImgDialog::getSkyDistortion(int i)
{
    switch (i) {
    case 0: return 0;
    case 1: return ui->sBSkyDest1->value();
    case 2: return ui->sBSkyDest2->value();
    case 3: return ui->sBSkyDest3->value();
    case 4: return ui->sBSkyDest4->value();
    case 5: return ui->sBSkyDest5->value();
    case 6: return ui->sBSkyDest6->value();
    case 7: return ui->sBSkyDest7->value();
    case 8: return ui->sBSkyDest8->value();
    case 9: return ui->sBSkyDest9->value();
    default: return NAN;
    }
}
//---------------------------------------------------------------------------
double SkyImgDialog::getSkyBinThres1()
{
    return ui->sBSkyBinThres1->value();
}
//---------------------------------------------------------------------------
double SkyImgDialog::getSkyBinThres2()
{
    return ui->sBSkyBinThres2->value();
}
//---------------------------------------------------------------------------
