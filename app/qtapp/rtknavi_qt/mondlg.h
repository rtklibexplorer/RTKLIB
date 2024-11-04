//---------------------------------------------------------------------------
#ifndef mondlgH
#define mondlgH
//---------------------------------------------------------------------------
#include <QStringList>
#include <QDialog>
#include <QTimer>

#include "rtklib.h"

namespace Ui {
class MonitorDialog;
}

//---------------------------------------------------------------------------
class MonitorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MonitorDialog(QWidget* parent, rtksvr_t *server, stream_t* stream);
    ~MonitorDialog();

    int getDisplayType();

protected:
    void showEvent(QShowEvent*);
    void closeEvent(QCloseEvent*);

    rtksvr_t *rtksvr;		// rtk server struct
    stream_t *monistr;	// monitor stream

public slots:
    void clearOutput();
    void scrollDown();
    void displayTypeChanged();
    void updateDisplays();
    void showBuffers();
    void observationModeChanged();
    void consoleFormatChanged();
    void inputStreamChanged();
    void solutionStreamChanged();
    void ephemerisSetChanged();

private:
    int consoleFormat, inputStream, solutionStream, fontScale;
    QStringList consoleBuffer;
    QStringList header;
	rtcm_t rtcm;
	raw_t raw;
    QTimer updateTimer;
	
    void clearTable();
    void setRtk();
    void setSat();
    void setEstimates();
    void setCovariance();
    void setObservations();
    void setNavigationGPS();
    void setGlonassNavigations();
    void setStream();
    void setSbsMessages();
    void setSbsLong();
    void setSbsIono();
    void setSbsFast();
    void setSbsNavigations();
    void setIonUtc();
    void setRtcm();
    void setRtcmDgps();
    void setRtcmSsr();
    void setReferenceStation();
    void showRtk();
    void showSat();
    void showEstimates();
    void showCovariance();
    void showObservations();
    void showNavigationsGPS();
    void showGlonassNavigations();
    void showSbsMessages();
    void showIonUtc();
    void showStream();
    void showSbsLong();
    void showSbsIono();
    void showSbsFast();
    void showSbsNavigations();
    void showRtcm();
    void showRtcmDgps();
    void showRtcmSsr();
    void showReferenceStation();

    void addConsole(const unsigned char *msg, int n, int mode, bool newline);

    Ui::MonitorDialog *ui;
};
//---------------------------------------------------------------------------
#endif
