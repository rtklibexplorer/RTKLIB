//---------------------------------------------------------------------------
#ifndef optdlgH
#define optdlgH
//---------------------------------------------------------------------------
#include <QDialog>
#include "rtklib.h"

namespace Ui {
class ConvOptDialog;
}

class QSettings;
class CodeOptDialog;
class GloFcnDialog;
class FreqDialog;

//---------------------------------------------------------------------------
class ConvOptDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConvOptDialog(QWidget *parent);
    void loadOptions(QSettings &);
    void saveOptions(QSettings &);
    void setTimeIntervalEnabled(bool ena);
    bool getTimeIntervalEnabled();

    gtime_t rinexTime;
    QString runBy, marker, markerNo, markerType, name[2], receiver[3], antenna[3];
    QString rinexStationCode, comment[2], receiverOptions, excludedSatellites;
    QString codeMask[7];
    double approxPosition[3], antennaDelta[3], timeTolerance;
    int rinexVersion, rinexFile, navSys, observationType, frequencyType, traceLevel;
    int autoPosition, phaseShift, halfCycle, sortSats, outputIonoCorr, outputTimeCorr, outputLeapSeconds, separateNavigation;
    int enableGlonassFrequency, glonassFrequency[27];

protected slots:
    void accept();
    void showMaskDialog();
    void showFrequencyDialog();
    void showFcnDialog();

protected:
    bool timeIntervalEnabled;

private:
    void updateEnable();
    void updateUi();

    CodeOptDialog *codeOptDialog;
    GloFcnDialog *gloFcnDialog;
    FreqDialog *freqDialog;

    Ui::ConvOptDialog *ui;
};
//---------------------------------------------------------------------------
#endif
