//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QSettings>

#include "convmain.h"
#include "convopt.h"
#include "codeopt.h"
#include "freqdlg.h"
#include "glofcndlg.h"

#include "ui_convopt.h"

//---------------------------------------------------------------------------
ConvOptDialog::ConvOptDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ConvOptDialog)
{
    ui->setupUi(this);

    rinexTime.sec = rinexTime.time = 0;

    codeOptDialog = new CodeOptDialog(this);
    gloFcnDialog = new GloFcnDialog(this);
    freqDialog = new FreqDialog(this);

    int glo = MAXPRNGLO, gal = MAXPRNGAL, qzs = MAXPRNQZS, cmp = MAXPRNCMP, irn = MAXPRNIRN;;
    if (glo <= 0) ui->cBNavSys2->setEnabled(false);
    if (gal <= 0) ui->cBNavSys3->setEnabled(false);
    if (qzs <= 0) ui->cBNavSys4->setEnabled(false);
    if (cmp <= 0) ui->cBNavSys6->setEnabled(false);
    if (irn <= 0) ui->cBNavSys7->setEnabled(false);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ConvOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &ConvOptDialog::reject);
    connect(ui->btnMask, &QPushButton::clicked, this, &ConvOptDialog::showMaskDialog);
    connect(ui->cBAutoPosition, &QCheckBox::clicked, this, &ConvOptDialog::updateEnable);
    connect(ui->cBRinexFilename, &QCheckBox::clicked, this, &ConvOptDialog::updateEnable);
    connect(ui->cBRinexVersion, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &ConvOptDialog::updateEnable);
    connect(ui->btnFcn, &QPushButton::clicked, this, &ConvOptDialog::showFcnDialog);
    connect(ui->btnFrequencies, &QPushButton::clicked, this, &ConvOptDialog::showFrequencyDialog);
}
//---------------------------------------------------------------------------
void ConvOptDialog::updateUi()
{
    ui->cBRinexVersion->setCurrentIndex(rinexVersion);
    ui->cBRinexFilename->setChecked(rinexFile);
    ui->lERinexStationCode->setText(rinexStationCode);
    ui->lERunBy->setText(runBy);
    ui->lEMarker->setText(marker);
    ui->lEMarkerNo->setText(markerNo);
    ui->lEMarkerType->setText(markerType);
    ui->lEObserver->setText(name[0]);
    ui->lEAgency->setText(name[1]);
    ui->lEReceiver->setText(receiver[0]);
    ui->lEReceiverType->setText(receiver[1]);
    ui->lEReceiverVersion->setText(receiver[2]);
    ui->lEAntenna->setText(antenna[0]);
    ui->lEAntennaType->setText(antenna[1]);  // antenna[2] is unused
    ui->sBApproxPosition0->setValue(approxPosition[0]);
    ui->sBApproxPosition1->setValue(approxPosition[1]);
    ui->sBApproxPosition2->setValue(approxPosition[2]);
    ui->sBAntennaDelta0->setValue(antennaDelta[0]);
    ui->sBAntennaDelta1->setValue(antennaDelta[1]);
    ui->sBAntennaDelta2->setValue(antennaDelta[2]);
    ui->lEComment0->setText(comment[0]);
    ui->lEComment1->setText(comment[1]);
    ui->lEReceiverOptions->setText(receiverOptions);
    ui->cBAutoPosition->setChecked(autoPosition);
    ui->cBPhaseShift->setChecked(phaseShift);
    ui->cBHalfCycle->setChecked(halfCycle);
    ui->cBOutputIonoCorr->setChecked(outputIonoCorr);
    ui->cBOutputTimeCorr->setChecked(outputTimeCorr);
    ui->cBOutputLeapSecs->setChecked(outputLeapSeconds);

    for (int i = 0; i < 7; i++) codeOptDialog->setCodeMask(i, codeMask[i]);

    gloFcnDialog->setGloFcnEnable(enableGlonassFrequency);
    for (int i = 0; i < 27; i++) {
        gloFcnDialog->setGloFcn(i, glonassFrequency[i]);
    }

    ui->cBNavSys1->setChecked(navSys & SYS_GPS);
    ui->cBNavSys2->setChecked(navSys & SYS_GLO);
    ui->cBNavSys3->setChecked(navSys & SYS_GAL);
    ui->cBNavSys4->setChecked(navSys & SYS_QZS);
    ui->cBNavSys5->setChecked(navSys & SYS_SBS);
    ui->cBNavSys6->setChecked(navSys & SYS_CMP);
    ui->cBNavSys7->setChecked(navSys & SYS_IRN);
    ui->cBObservationTypeC->setChecked(observationType & OBSTYPE_PR);
    ui->cBObservationTypeL->setChecked(observationType & OBSTYPE_CP);
    ui->cBObservationTypeD->setChecked(observationType & OBSTYPE_DOP);
    ui->cBObservationTypeS->setChecked(observationType & OBSTYPE_SNR);
    ui->cBFreq1->setChecked(frequencyType & FREQTYPE_L1);
    ui->cBFreq2->setChecked(frequencyType & FREQTYPE_L2);
    ui->cBFreq3->setChecked(frequencyType & FREQTYPE_L3);
    ui->cBFreq4->setChecked(frequencyType & FREQTYPE_L4);
    ui->cBFreq5->setChecked(frequencyType & FREQTYPE_L5);

    ui->lEExcludedSatellites->setText(excludedSatellites);
    ui->cBTraceLevel->setCurrentIndex(traceLevel);
    ui->cBSeperateNavigation->setChecked(separateNavigation);
    ui->sBTimeTolerance->setValue(timeTolerance);

    updateEnable();
}
//---------------------------------------------------------------------------
void ConvOptDialog::accept()
{
    rinexVersion = ui->cBRinexVersion->currentIndex();
    rinexFile = ui->cBRinexFilename->isChecked();
    rinexStationCode = ui->lERinexStationCode->text();
    runBy = ui->lERunBy->text();
    marker = ui->lEMarker->text();
    markerNo = ui->lEMarkerNo->text();
    markerType = ui->lEMarkerType->text();
    name[0] = ui->lEObserver->text();
    name[1] = ui->lEAgency->text();
    receiver[0] = ui->lEReceiver->text();
    receiver[1] = ui->lEReceiverType->text();
    receiver[2] = ui->lEReceiverVersion->text();
    antenna[0] = ui->lEAntenna->text();
    antenna[1] = ui->lEAntennaType->text();
    approxPosition[0] = ui->sBApproxPosition0->value();
    approxPosition[1] = ui->sBApproxPosition1->value();
    approxPosition[2] = ui->sBApproxPosition2->value();
    antennaDelta[0] = ui->sBAntennaDelta0->value();
    antennaDelta[1] = ui->sBAntennaDelta1->value();
    antennaDelta[2] = ui->sBAntennaDelta2->value();
    comment[0] = ui->lEComment0->text();
    comment[1] = ui->lEComment1->text();
    receiverOptions = ui->lEReceiverOptions->text();
    autoPosition = ui->cBAutoPosition->isChecked();
    phaseShift = ui->cBPhaseShift->isChecked();
    halfCycle = ui->cBHalfCycle->isChecked();
    outputIonoCorr = ui->cBOutputIonoCorr->isChecked();
    outputTimeCorr = ui->cBOutputTimeCorr->isChecked();
    outputLeapSeconds = ui->cBOutputLeapSecs->isChecked();

    for (int i = 0; i < 7; i++) codeMask[i] = codeOptDialog->getCodeMask(i);

    enableGlonassFrequency = gloFcnDialog->getGloFcnEnable();
    for (int i = 0; i < 27; i++) {
        glonassFrequency[i] = gloFcnDialog->getGloFcn(i);
    }

    navSys = 0;
    observationType = 0;
    frequencyType = 0;

    if (ui->cBNavSys1->isChecked()) navSys |= SYS_GPS;
    if (ui->cBNavSys2->isChecked()) navSys |= SYS_GLO;
    if (ui->cBNavSys3->isChecked()) navSys |= SYS_GAL;
    if (ui->cBNavSys4->isChecked()) navSys |= SYS_QZS;
    if (ui->cBNavSys5->isChecked()) navSys |= SYS_SBS;
    if (ui->cBNavSys6->isChecked()) navSys |= SYS_CMP;
    if (ui->cBNavSys7->isChecked()) navSys |= SYS_IRN;

    if (ui->cBObservationTypeC->isChecked()) observationType |= OBSTYPE_PR;
    if (ui->cBObservationTypeL->isChecked()) observationType |= OBSTYPE_CP;
    if (ui->cBObservationTypeD->isChecked()) observationType |= OBSTYPE_DOP;
    if (ui->cBObservationTypeS->isChecked()) observationType |= OBSTYPE_SNR;

    if (ui->cBFreq1->isChecked()) frequencyType |= FREQTYPE_L1;
    if (ui->cBFreq2->isChecked()) frequencyType |= FREQTYPE_L2;
    if (ui->cBFreq3->isChecked()) frequencyType |= FREQTYPE_L3;
    if (ui->cBFreq4->isChecked()) frequencyType |= FREQTYPE_L4;
    if (ui->cBFreq5->isChecked()) frequencyType |= FREQTYPE_L5;

    excludedSatellites = ui->lEExcludedSatellites->text();
    traceLevel = ui->cBTraceLevel->currentIndex();
    separateNavigation = ui->cBSeperateNavigation->isChecked();
    timeTolerance = ui->sBTimeTolerance->value();

    QDialog::accept();
}
//---------------------------------------------------------------------------
void ConvOptDialog::showFrequencyDialog()
{
    freqDialog->show();
}
//---------------------------------------------------------------------------
void ConvOptDialog::showMaskDialog()
{
    int navSystem = 0;
    int frequencyType = 0;
    
    if (ui->cBNavSys1->isChecked()) navSystem |= SYS_GPS;
    if (ui->cBNavSys2->isChecked()) navSystem |= SYS_GLO;
    if (ui->cBNavSys3->isChecked()) navSystem |= SYS_GAL;
    if (ui->cBNavSys4->isChecked()) navSystem |= SYS_QZS;
    if (ui->cBNavSys5->isChecked()) navSystem |= SYS_SBS;
    if (ui->cBNavSys6->isChecked()) navSystem |= SYS_CMP;
    if (ui->cBNavSys7->isChecked()) navSystem |= SYS_IRN;
    
    codeOptDialog->setNavSystem(navSystem);

    if (ui->cBFreq1->isChecked()) frequencyType |= FREQTYPE_L1;
    if (ui->cBFreq2->isChecked()) frequencyType |= FREQTYPE_L2;
    if (ui->cBFreq3->isChecked()) frequencyType |= FREQTYPE_L3;
    if (ui->cBFreq4->isChecked()) frequencyType |= FREQTYPE_L4;
    if (ui->cBFreq5->isChecked()) frequencyType |= FREQTYPE_L5;

    codeOptDialog->setFrequencyType(frequencyType);

    codeOptDialog->exec();

}
//---------------------------------------------------------------------------
void ConvOptDialog::setTimeIntervalEnabled(bool ena)
{
    timeIntervalEnabled = ena;
}
//---------------------------------------------------------------------------
bool ConvOptDialog::getTimeIntervalEnabled()
{
    return timeIntervalEnabled;
}
//---------------------------------------------------------------------------
void ConvOptDialog::updateEnable()
{
    ui->sBApproxPosition0->setEnabled(ui->cBAutoPosition->isChecked());
    ui->sBApproxPosition1->setEnabled(ui->cBAutoPosition->isChecked());
    ui->sBApproxPosition2->setEnabled(ui->cBAutoPosition->isChecked());
    ui->cBSeperateNavigation->setEnabled(ui->cBRinexVersion->currentIndex() >= 3);
    ui->lbTimeTolerance->setEnabled(timeIntervalEnabled);
    ui->sBTimeTolerance->setEnabled(timeIntervalEnabled);
    ui->cBNavSys3->setEnabled(ui->cBRinexVersion->currentIndex() >= 1);
    ui->cBNavSys4->setEnabled(ui->cBRinexVersion->currentIndex() >= 5);
    ui->cBNavSys6->setEnabled(ui->cBRinexVersion->currentIndex() == 2 || ui->cBRinexVersion->currentIndex() >= 4);
    ui->cBNavSys7->setEnabled(ui->cBRinexVersion->currentIndex() >= 6);
    ui->cBFreq3->setEnabled(ui->cBRinexVersion->currentIndex() >=1 );
    ui->cBFreq4->setEnabled(ui->cBRinexVersion->currentIndex() >=1 );
    ui->cBFreq5->setEnabled(ui->cBRinexVersion->currentIndex() >=1 );
    ui->cBPhaseShift->setEnabled(ui->cBRinexVersion->currentIndex() >=4 );
}
//---------------------------------------------------------------------------
void ConvOptDialog::showFcnDialog()
{
    gloFcnDialog->exec();
}
//---------------------------------------------------------------------------
void ConvOptDialog::loadOptions(QSettings &ini)
{
    QString mask = "11111111111111111111111111111111111111111111111111111111111111111111";

    rinexVersion = ini.value("opt/rnxver", 6).toInt();
    rinexFile = ini.value("opt/rnxfile", 0).toInt();
    rinexStationCode = ini.value("opt/rnxcode", "0000").toString();
    runBy = ini.value("opt/runby", "").toString();
    marker = ini.value("opt/marker", "").toString();
    markerNo = ini.value("opt/markerno", "").toString();
    markerType = ini.value("opt/markertype", "").toString();
    name[0] = ini.value("opt/name0", "").toString();
    name[1] = ini.value("opt/name1", "").toString();
    receiver[0] = ini.value("opt/rec0", "").toString();
    receiver[1] = ini.value("opt/rec1", "").toString();
    receiver[2] = ini.value("opt/rec2", "").toString();
    antenna[0] = ini.value("opt/ant0", "").toString();
    antenna[1] = ini.value("opt/ant1", "").toString();
    antenna[2] = ini.value("opt/ant2", "").toString();
    approxPosition[0] = ini.value("opt/apppos0", 0.0).toDouble();
    approxPosition[1] = ini.value("opt/apppos1", 0.0).toDouble();
    approxPosition[2] = ini.value("opt/apppos2", 0.0).toDouble();
    antennaDelta[0] = ini.value("opt/antdel0", 0.0).toDouble();
    antennaDelta[1] = ini.value("opt/antdel1", 0.0).toDouble();
    antennaDelta[2] = ini.value("opt/antdel2", 0.0).toDouble();
    comment[0] = ini.value("opt/comment0", "").toString();
    comment[1] = ini.value("opt/comment1", "").toString();
    receiverOptions = ini.value("opt/rcvoption", "").toString();
    navSys = ini.value("opt/navsys", 0xff).toInt();
    observationType = ini.value("opt/obstype", 0xF).toInt();
    frequencyType = ini.value("opt/freqtype", 0x7).toInt();
    excludedSatellites = ini.value("opt/exsats", "").toString();
    traceLevel = ini.value("opt/tracelevel", 0).toInt();
    rinexTime.time = ini.value("opt/rnxtime", 0).toInt();
    codeMask[0] = ini.value("opt/codemask_1", mask).toString();
    codeMask[1] = ini.value("opt/codemask_2", mask).toString();
    codeMask[2] = ini.value("opt/codemask_3", mask).toString();
    codeMask[3] = ini.value("opt/codemask_4", mask).toString();
    codeMask[4] = ini.value("opt/codemask_5", mask).toString();
    codeMask[5] = ini.value("opt/codemask_6", mask).toString();
    codeMask[6] = ini.value("opt/codemask_7", mask).toString();
    autoPosition = ini.value("opt/autopos", 0).toInt();
    phaseShift = ini.value("opt/phaseShift", 0).toInt();
    halfCycle = ini.value("opt/halfcyc", 0).toInt();
    outputIonoCorr = ini.value("opt/outiono", 0).toInt();
    outputTimeCorr = ini.value("opt/outtime", 0).toInt();
    outputLeapSeconds = ini.value("opt/outleaps", 0).toInt();
    separateNavigation = ini.value("opt/sepnav", 0).toInt();
    timeTolerance = ini.value("opt/timetol", 0.005).toDouble();
    enableGlonassFrequency = ini.value("opt/glofcnena", 0).toInt();
    for (int i = 0; i < 27; i++)
        glonassFrequency[i] = ini.value(QString("opt/glofcn%1").arg(i + 1, 2, 10, QLatin1Char('0')), 0).toInt();

    updateUi();
}
//---------------------------------------------------------------------------
void ConvOptDialog::saveOptions(QSettings &ini)
{
    ini.setValue("opt/rnxver", rinexVersion);
    ini.setValue("opt/rnxfile", rinexFile);
    ini.setValue("opt/rnxcode", rinexStationCode);
    ini.setValue("opt/runby", runBy);
    ini.setValue("opt/marker", marker);
    ini.setValue("opt/markerno", markerNo);
    ini.setValue("opt/markertype", markerType);
    ini.setValue("opt/name0", name[0]);
    ini.setValue("opt/name1", name[1]);
    ini.setValue("opt/rec0", receiver[0]);
    ini.setValue("opt/rec1", receiver[1]);
    ini.setValue("opt/rec2", receiver[2]);
    ini.setValue("opt/ant0", antenna[0]);
    ini.setValue("opt/ant1", antenna[1]);
    ini.setValue("opt/ant2", antenna[2]);
    ini.setValue("opt/apppos0", approxPosition[0]);
    ini.setValue("opt/apppos1", approxPosition[1]);
    ini.setValue("opt/apppos2", approxPosition[2]);
    ini.setValue("opt/antdel0", antennaDelta[0]);
    ini.setValue("opt/antdel1", antennaDelta[1]);
    ini.setValue("opt/antdel2", antennaDelta[2]);
    ini.setValue("opt/comment0", comment[0]);
    ini.setValue("opt/comment1", comment[1]);
    ini.setValue("opt/rcvoption", receiverOptions);
    ini.setValue("opt/navsys", navSys);
    ini.setValue("opt/obstype", observationType);
    ini.setValue("opt/freqtype", frequencyType);
    ini.setValue("opt/exsats", excludedSatellites);
    ini.setValue("opt/tracelevel", traceLevel);
    ini.setValue("opt/rnxtime", (int)(rinexTime.time));
    ini.setValue("opt/codemask_1", codeMask[0]);
    ini.setValue("opt/codemask_2", codeMask[1]);
    ini.setValue("opt/codemask_3", codeMask[2]);
    ini.setValue("opt/codemask_4", codeMask[3]);
    ini.setValue("opt/codemask_5", codeMask[4]);
    ini.setValue("opt/codemask_6", codeMask[5]);
    ini.setValue("opt/codemask_7", codeMask[6]);
    ini.setValue("opt/autopos", autoPosition);
    ini.setValue("opt/phasewhift", phaseShift);
    ini.setValue("opt/halfcyc", halfCycle);
    ini.setValue("opt/outiono", outputIonoCorr);
    ini.setValue("opt/outtime", outputTimeCorr);
    ini.setValue("opt/outleaps", outputLeapSeconds);
    ini.setValue("opt/sepnav", separateNavigation);
    ini.setValue("opt/timetol", timeTolerance);
    ini.setValue("opt/glofcnena",  enableGlonassFrequency);

    for (int i = 0; i < 27; i++) {
        ini.setValue(QString("opt/glofcn%1").arg(i + 1, 2, 10, QLatin1Char('0')), glonassFrequency[i]);
    }

}
//---------------------------------------------------------------------------
