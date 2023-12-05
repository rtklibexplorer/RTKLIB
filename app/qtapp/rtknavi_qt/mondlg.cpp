//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QCloseEvent>
#include <QScrollBar>
#include <QDebug>

#include "rtklib.h"
#include "mondlg.h"

//---------------------------------------------------------------------------

#define SQRT(x)     ((x)<0.0||(x)!=(x)?0.0:sqrt(x))
#define MAXLINE		128
#define MAXLEN		256

#define NMONITEM	17


static const int sys_tbl[]={
        SYS_ALL, SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS
};

//---------------------------------------------------------------------------

extern rtksvr_t rtksvr;		// rtk server struct
extern stream_t monistr;	// monitor stream

//---------------------------------------------------------------------------
MonitorDialog::MonitorDialog(QWidget *parent)
    : QDialog(parent)
{
    int i;

    setupUi(this);

    fontScale = physicalDpiX() * 2;

    observationMode = 0;
    consoleFormat = -1;
    inputStream=solutionStream=0;

    for (i = 0; i <= MAXRCVFMT; i++) cBSelectFormat->addItem(formatstrs[i]);

    init_rtcm(&rtcm);
    init_raw(&raw, -1);

    connect(btnClear, &QPushButton::clicked, this, &MonitorDialog::btnClearClicked);
    connect(btnClose, &QPushButton::clicked, this, &MonitorDialog::accept);
    connect(btnDown, &QPushButton::clicked, this, &MonitorDialog::btnDownClicked);
    connect(cBType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MonitorDialog::displayTypeChanged);
    connect(cBSelectFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MonitorDialog::consoleFormatChanged);
    connect(cBSelectObservation, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MonitorDialog::observationModeChanged);
    connect(cBSelectInputStream, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MonitorDialog::inputStreamChanged);
    connect(cBSelectSolutionStream, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MonitorDialog::solutionStreamChanged);
    connect(&updateTimer, &QTimer::timeout, this, &MonitorDialog::updateTimerTriggered);

    typeF = cBType->currentIndex();

    updateTimer.start(1000);
}
//---------------------------------------------------------------------------
MonitorDialog::~MonitorDialog()
{
    free_raw(&raw);
    free_rtcm(&rtcm);
};
//---------------------------------------------------------------------------
void MonitorDialog::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    lblInformation->setText("");

    clearTable();
}
//---------------------------------------------------------------------------
void MonitorDialog::closeEvent(QCloseEvent *event)
{
    updateTimer.stop();

    free_rtcm(&rtcm);
  free_raw(&raw);

    event->accept();
}
//---------------------------------------------------------------------------
void MonitorDialog::displayTypeChanged()
{
  int index;

  typeF = cBType->currentIndex();
  index = typeF - NMONITEM;

  if (0 <= index) {
    rtksvrlock(&rtksvr);
    if (index < 2) rtksvr.npb[index] = 0;
    else if (index < 4) rtksvr.nsb[index - 2] = 0;
    else rtksvr.rtk.neb = 0;
    rtksvrunlock(&rtksvr);
  }
  clearTable();
  lblInformation->setText("");
  consoleBuffer.clear();
  tWConsole->clear();
}
//---------------------------------------------------------------------------
void MonitorDialog::consoleFormatChanged()
{
    addConsole((uint8_t *)"\n", 1, 1);

    if (consoleFormat >= 3 && consoleFormat < 17)
        free_raw(&raw);
    consoleFormat = cBSelectFormat->currentIndex();

    if (consoleFormat >= 3 && consoleFormat < 17)
        init_raw(&raw, consoleFormat - 2);
}
//---------------------------------------------------------------------------
void MonitorDialog::inputStreamChanged()
{
    inputStream = cBSelectInputStream->currentIndex();
    consoleBuffer.clear();
    tWConsole->clear();
}
//---------------------------------------------------------------------------
void MonitorDialog::solutionStreamChanged()
{
    solutionStream = cBSelectSolutionStream->currentIndex();
    consoleBuffer.clear();
    tWConsole->clear();
}
//---------------------------------------------------------------------------
void MonitorDialog::updateTimerTriggered()
{
    if (!isVisible()) return;

  switch (typeF) {
        case  0: showRtk(); break;
        case  1: showObservations(); break;
        case  2: showNavigations(); break;
        case  3: showIonUtc(); break;
        case  4: showStream(); break;
        case  5: showSat(); break;
        case  6: showEstimates(); break;
        case  7: showCovariance(); break;
        case  8: showSbsMessages(); break;
        case  9: showSbsLong(); break;
        case 10: showSbsIono(); break;
        case 11: showSbsFast(); break;
        case 12: showRtcm(); break;
        case 13: showRtcmDgps(); break;
        case 14: showRtcmSsr(); break;
        case 15: showReferenceStation(); break;
        default: showBuffers(); break;
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::clearTable()
{
    int console = 0;

  switch (typeF) {
        case  0: setRtk(); break;
        case  1: setObservations(); break;
        case  2: break;
        case  3: setIonUtc(); break;
        case  4: setStream(); break;
        case  5: setSat(); break;
        case  6: setEstimates(); break;
        case  7: setCovariance(); break;
        case  8: setSbsMessages(); break;
        case  9: setSbsLong(); break;
        case 10: setSbsIono(); break;
        case 11: setSbsFast(); break;
        case 12: setRtcm(); break;
        case 13: setRtcmDgps(); break;
        case 14: setRtcmSsr(); break;
        case 15: setReferenceStation(); break;
        default: console = 1;
            tWConsole->setColumnWidth(0, tWConsole->width());
            break;
  }
    tWConsole->setVisible(true);
    Panel2->setVisible(console);
    btnPause->setVisible(console != 0);
    btnDown->setVisible(console != 0);
    btnClear->setVisible(console != 0);

    cBSelectObservation->setVisible(typeF == 1);
    cBSelectNavigationSystems->setVisible(typeF == 1 || typeF == 5);
    cBSelectSingleNavigationSystem->setVisible(typeF == 2 || typeF == 14);
    cBSelectSatellites->setVisible(typeF == 2 || typeF == 5);
    cBSelectInputStream->setVisible(typeF == 12 || typeF == 14 || typeF == 15 || typeF == 16);
    cBSelectSolutionStream->setVisible(typeF == 17);
    cBSelectFormat->setVisible(typeF == 16);
    cBSelectEphemeris->setVisible(typeF == 2);
}
//---------------------------------------------------------------------------
void MonitorDialog::showBuffers()
{
    unsigned char *msg = 0;
    int i, len;

    if (typeF < 16) return;

  rtksvrlock(&rtksvr);

    if (typeF == 16) { // input buffer
        len = rtksvr.npb[inputStream];
        if (len > 0 && (msg = (uint8_t *)malloc(size_t(len)))) {
            memcpy(msg, rtksvr.pbuf[inputStream], size_t(len));
            rtksvr.npb[inputStream] = 0;
    }
    } else if (typeF == 17) { // solution buffer
        len = rtksvr.nsb[solutionStream];
        if (len > 0 && (msg = (uint8_t*)malloc(size_t(len)))) {
            memcpy(msg, rtksvr.sbuf[solutionStream], size_t(len));
            rtksvr.nsb[solutionStream] = 0;
    }
    } else { // error message buffer
        len = rtksvr.rtk.neb;
        if (len > 0 && (msg = (uint8_t *)malloc(size_t(len)))) {
            memcpy(msg, rtksvr.rtk.errbuf, size_t(len));
            rtksvr.rtk.neb = 0;
    }
  }
  rtksvrunlock(&rtksvr);

    if (len <= 0 || !msg) return;

    rtcm.outtype = raw.outtype = 1;

    if (typeF >=17) {
        addConsole(msg, len, 1);
    }
    else if (consoleFormat < 2) {
        addConsole(msg, len, consoleFormat);
    } else if (consoleFormat == 2) {
        for (i = 0; i < len; i++) {
            input_rtcm2(&rtcm, msg[i]);
      if (rtcm.msgtype[0]) {
                QString buff = QString::fromLatin1(rtcm.msgtype, -1) + "\n";
                addConsole((uint8_t*)qPrintable(buff), buff.size(), 1);
                rtcm.msgtype[0] = '\0';
      }
        }
    } else if (consoleFormat == 3) {
        for (i = 0; i < len; i++) {
            input_rtcm3(&rtcm, msg[i]);
      if (rtcm.msgtype[0]) {
                QString buff = QString::fromLatin1(rtcm.msgtype, -1) + "\n";
                addConsole((uint8_t*)qPrintable(buff), buff.size(), 1);
                rtcm.msgtype[0] = '\0';
      }
        }
    } else if (consoleFormat < 17) {
        for (i = 0; i < len; i++) {
            input_raw(&raw, consoleFormat - 2, msg[i]);
      if (raw.msgtype[0]) {
                QString buff=QString::fromLatin1(raw.msgtype, -1) + "\n";
                addConsole((uint8_t*)qPrintable(buff), buff.size(), 1);
                raw.msgtype[0] = '\0';
      }
        }
  }
  free(msg);
}
//---------------------------------------------------------------------------
void MonitorDialog::addConsole(const uint8_t *msg, int n, int mode)
{
    char buff[MAXLEN + 16], *p = buff;

    if (btnPause->isChecked()) return;

    if (n <= 0) return;

    if (consoleBuffer.count() == 0) consoleBuffer.append(""); // make sure that there is always at least one line in consoleBuffer

    // fill buffer with last incomplete line
    p += sprintf(p, "%s", qPrintable(consoleBuffer.at(consoleBuffer.count() - 1)));

    for (int i = 0; i < n; i++) {
        if (mode) {
            if (msg[i] == '\r') continue;
            p += sprintf(p, "%c", msg[i] == '\n' || isprint(msg[i]) ? msg[i] : '.');
        } else { // add a space after 16 and a line break after 67 characters
            p += sprintf(p, "%s%02X", (p - buff) % 17 == 16 ? " " : "", msg[i]);
            if (p - buff >= 67) p += sprintf(p, "\n");
        }
        if (p - buff >= MAXLEN) p += sprintf(p, "\n");

        if (*(p - 1) == '\n') {
            consoleBuffer[consoleBuffer.count() - 1] = buff;
            consoleBuffer.append("");
            *(p = buff) = 0;
            if (consoleBuffer.count() >= MAXLINE) consoleBuffer.removeFirst();
        }
    }
    consoleBuffer[consoleBuffer.count() - 1] = buff;

    tWConsole->setColumnCount(1);
    tWConsole->setRowCount(consoleBuffer.size());
    for (int i = 0; i < consoleBuffer.size(); i++)
        tWConsole->setItem(i, 0, new QTableWidgetItem(consoleBuffer.at(i)));

    if (btnDown->isChecked()) tWConsole->verticalScrollBar()->setValue(tWConsole->verticalScrollBar()->maximum());
}
//---------------------------------------------------------------------------
void MonitorDialog::btnClearClicked()
{
    consoleBuffer.clear();
    tWConsole->clear();
    tWConsole->setRowCount(0);
}
//---------------------------------------------------------------------------
void MonitorDialog::btnDownClicked()
{
    tWConsole->verticalScrollBar()->setValue(tWConsole->verticalScrollBar()->maximum());
}
//---------------------------------------------------------------------------
void MonitorDialog::observationModeChanged()
{
    observationMode = cBSelectObservation->currentIndex();
    setObservations();
    showObservations();
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtk()
{
    header << tr("Parameter") << tr("Value");
    int width[] = {220, 380};

    tWConsole->setColumnCount(2);
    tWConsole->setRowCount(54+NFREQ*2);
    tWConsole->setHorizontalHeaderLabels(header);

    for (int i = 0; i < tWConsole->columnCount() & i < 2; i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
}
//---------------------------------------------------------------------------
void MonitorDialog::showRtk()
{
  rtk_t rtk;
    QString exsats, navsys = "";
    const QString svrstate[] = {tr("Stop"), tr("Run")};
    const QString sol[] = {tr("-"), tr("Fix"), tr("Float"), tr("SBAS"), tr("DGPS"), tr("Single"), tr("PPP"), ""};
    const QString mode[] = {tr("Single"), tr("DGPS"), tr("Kinematic"), tr("Static"), tr("Static-Start"), tr("Moving-Base"),
               tr("Fixed"),	 tr("PPP-Kinematic"), tr("PPP-Static"), ""};
    const QString freq[] = {tr("-"), tr("L1"), tr("L1+L2"), tr("L1+L2+L5"), tr("L1+L2+L5+L6"), tr("L1+L2+L5+L6+L7"), ""};
    double *del, *off, rt[3] = {0}, dop[4] = {0};
    double azel[MAXSAT * 2], pos[3], vel[3], rr[3] = {0}, enu[3] = {0};
    int i, j, k, cycle, state, rtkstat, nsat0, nsat1, prcout, nave;
    unsigned long thread;
    int cputime, nb[3] = {0}, ne;
    unsigned int nmsg[3][10] = {{0}};
    char tstr[64], id[32], s1[64] = "-", s2[64] = "-", s3[64] = "-";
    char file[1024] = "";
    const QString ionoopt[] = {tr("OFF"), tr("Broadcast"), tr("SBAS"), tr("Dual-Frequency"), tr("Estimate STEC"), tr("IONEX TEC"), tr("QZSS LEX"), ""};
    const QString tropopt[] = {tr("OFF"), tr("Saastamoinen"), tr("SBAS"), tr("Estimate ZTD"), tr("Estimate ZTD+Grad"), ""};
    const QString ephopt [] = {tr("Broadcast"), tr("Precise"), tr("Broadcast+SBAS"), tr("Broadcat+SSR APC"), tr("Broadcast+SSR CoM"), tr("QZSS LEX"), ""};

  rtksvrlock(&rtksvr); // lock

    rtk = rtksvr.rtk;
    thread = (unsigned long)rtksvr.thread;
    cycle = rtksvr.cycle;
    state = rtksvr.state;
    rtkstat = rtksvr.rtk.sol.stat;
    nsat0 = rtksvr.obs[0][0].n;
    nsat1 = rtksvr.obs[1][0].n;
    cputime = rtksvr.cputime;
    prcout = rtksvr.prcout;
    nave = rtksvr.nave;

    for (i = 0; i < 3; i++) nb[i] = rtksvr.nb[i];

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 10; j++)
            nmsg[i][j] = rtksvr.nmsg[i][j];
    }

  if (rtksvr.state) {
        double runtime;
        runtime = static_cast<double>(tickget() - rtksvr.tick) / 1000.0;
        rt[0] = floor(runtime / 3600.0); runtime -= rt[0] * 3600.0;
        rt[1] = floor(runtime / 60.0); rt[2] = runtime - rt[1] * 60.0;
  }
    if ((ne = rtksvr.nav.ne) > 0) {
        time2str(rtksvr.nav.peph[0].time, s1, 0);
        time2str(rtksvr.nav.peph[ne - 1].time, s2, 0);
        time2str(rtksvr.ftime[2], s3, 0);
  }
    strncpy(file, rtksvr.files[2], 1024);

  rtksvrunlock(&rtksvr); // unlock

    for (j = k = 0; j < MAXSAT; j++) {
        if (rtk.opt.mode == PMODE_SINGLE && !rtk.ssat[j].vs) continue;
        if (rtk.opt.mode != PMODE_SINGLE && !rtk.ssat[j].vsat[0]) continue;
        azel[k * 2] = rtk.ssat[j].azel[0];
        azel[k * 2 + 1] = rtk.ssat[j].azel[1];
    k++;
  }
    dops(k, azel, 0.0, dop);

    if (rtk.opt.navsys & SYS_GPS) navsys = navsys + tr("GPS ");
    if (rtk.opt.navsys & SYS_GLO) navsys = navsys + tr("GLONASS ");
    if (rtk.opt.navsys & SYS_GAL) navsys = navsys + tr("Galileo ");
    if (rtk.opt.navsys & SYS_QZS) navsys = navsys + tr("QZSS ");
    if (rtk.opt.navsys & SYS_CMP) navsys = navsys + tr("BDS ");
    if (rtk.opt.navsys & SYS_IRN) navsys = navsys + tr("NavIC ");
    if (rtk.opt.navsys & SYS_SBS) navsys = navsys + tr("SBAS ");

    lblInformation->setText("");
    if (tWConsole->rowCount() < 56) return;
    tWConsole->setHorizontalHeaderLabels(header);

    i = 0;

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("RTKLIB Version")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1 %2").arg(VER_RTKLIB).arg(PATCH_LEVEL)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("RTK Server Thread")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(thread)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("RTK Server State")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(svrstate[state]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Processing Cycle (ms)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(cycle)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Positioning Mode")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(mode[rtk.opt.mode]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Frequencies")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(freq[rtk.opt.nf]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Elevation Mask (deg)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtk.opt.elmin * R2D, 'f', 0)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("SNR Mask L1 (dBHz)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(!rtk.opt.snrmask.ena[0] ? "" :
                              QString("%1, %2, %3, %4, %5, %6, %7, %8, %9")
                              .arg(rtk.opt.snrmask.mask[0][0], 0).arg(rtk.opt.snrmask.mask[0][1], 0).arg(rtk.opt.snrmask.mask[0][2], 0)
                              .arg(rtk.opt.snrmask.mask[0][3], 0).arg(rtk.opt.snrmask.mask[0][4], 0).arg(rtk.opt.snrmask.mask[0][5], 0)
                              .arg(rtk.opt.snrmask.mask[0][6], 0).arg(rtk.opt.snrmask.mask[0][7], 0).arg(rtk.opt.snrmask.mask[0][8], 0)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("SNR Mask L2 (dBHz)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(!rtk.opt.snrmask.ena[0] ? "" :
                              QString("%1, %2, %3, %4, %5, %6, %7, %8, %9")
                              .arg(rtk.opt.snrmask.mask[1][0], 0).arg(rtk.opt.snrmask.mask[1][1], 0).arg(rtk.opt.snrmask.mask[1][2], 0)
                              .arg(rtk.opt.snrmask.mask[1][3], 0).arg(rtk.opt.snrmask.mask[1][4], 0).arg(rtk.opt.snrmask.mask[1][5], 0)
                              .arg(rtk.opt.snrmask.mask[1][6], 0).arg(rtk.opt.snrmask.mask[1][7], 0).arg(rtk.opt.snrmask.mask[1][8], 0)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("SNR Mask L5 (dBHz)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(!rtk.opt.snrmask.ena[0] ? "" :
                              QString("%1, %2, %3, %4, %5, %6, %7, %8, %9")
                              .arg(rtk.opt.snrmask.mask[2][0], 0).arg(rtk.opt.snrmask.mask[2][1], 0).arg(rtk.opt.snrmask.mask[2][2], 0)
                              .arg(rtk.opt.snrmask.mask[2][3], 0).arg(rtk.opt.snrmask.mask[2][4], 0).arg(rtk.opt.snrmask.mask[2][5], 0)
                              .arg(rtk.opt.snrmask.mask[2][6], 0).arg(rtk.opt.snrmask.mask[2][7], 0).arg(rtk.opt.snrmask.mask[2][8], 0)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Rec Dynamic/Earth Tides Correction")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2").arg(rtk.opt.dynamics ? tr("ON") : tr("OFF")).arg(rtk.opt.tidecorr ? tr("ON") : tr("OFF"))));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Ionosphere/Troposphere Model")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2").arg(ionoopt[rtk.opt.ionoopt]).arg(tropopt[rtk.opt.tropopt])));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Satellite Ephemeris")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(ephopt[rtk.opt.sateph]));

    for (j = 1; j <= MAXSAT; j++) {
        if (!rtk.opt.exsats[j - 1]) continue;
        satno2id(j, id);
        if (rtk.opt.exsats[j - 1] == 2) exsats = exsats + "+";
        exsats = exsats + id + " ";
  }
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Excluded Satellites")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(exsats));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Navi Systems")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(navsys));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Accumulated Time to Run")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1:%2:%3").arg(rt[0], 2, 'f', 0, '0').arg(rt[1], 2, 'f', 0, '0').arg(rt[2], 4, 'f', 1, '0')));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("CPU Time for a Processing Cycle (ms)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(cputime)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Missing Obs Data Count")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(prcout)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Bytes in Input Buffer")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(nb[0]).arg(nb[1]).arg(nb[2])));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Input Data Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString(tr("Obs(%1), Nav(%2), Ion(%3), Sbs (%4), Pos(%5), Dgps(%6), Ssr(%7), Err(%8)"))
                              .arg(nmsg[0][0]).arg(nmsg[0][1] + nmsg[0][6]).arg(nmsg[0][2]).arg(nmsg[0][3])
                              .arg(nmsg[0][4]).arg(nmsg[0][5]).arg(nmsg[0][7]).arg(nmsg[0][9])));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Input Data Base/NRTK Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString(tr("Obs(%1), Nav(%2), Ion(%3), Sbs (%4), Pos(%5), Dgps(%6), Ssr(%7), Err(%8)"))
                              .arg(nmsg[1][0]).arg(nmsg[1][1] + nmsg[1][6]).arg(nmsg[1][2]).arg(nmsg[1][3])
                              .arg(nmsg[1][4]).arg(nmsg[1][5]).arg(nmsg[1][7]).arg(nmsg[1][9])));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Input Data Ephemeris")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString(tr("Obs(%1), Nav(%2), Ion(%3), Sbs (%4), Pos(%5), Dgps(%6), Ssr(%7), Err(%8)"))
                              .arg(nmsg[2][0]).arg(nmsg[2][1] + nmsg[2][6]).arg(nmsg[2][2]).arg(nmsg[2][3])
                              .arg(nmsg[2][4]).arg(nmsg[2][5]).arg(nmsg[2][7]).arg(nmsg[2][9])));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Solution Status")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sol[rtkstat]));

    time2str(rtk.sol.time, tstr, 9);
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Time of Receiver Clock Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtk.sol.time.time ? tstr : "-"));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Time System Offset/Receiver Bias (GLO-GPS,GAL-GPS,BDS-GPS,IRN-GPS) (ns)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(rtk.sol.dtr[1] * 1E9, 0, 'f', 3).arg(rtk.sol.dtr[2] * 1E9, 0, 'f', 3).arg(rtk.sol.dtr[3] * 1E9, 0, 'f', 3)
                              .arg(rtk.sol.dtr[4] * 1E9, 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Solution Interval (s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtk.tt, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Age of Differential (s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtk.sol.age, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Ratio for AR Validation")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtk.sol.ratio, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Satellites Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(nsat0)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Satellites Base/NRTK Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(nsat1)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Valid Satellites")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtk.sol.ns)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("GDOP/PDOP/HDOP/VDOP")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(dop[0], 0, 'f', 1).arg(dop[1], 0, 'f', 1).arg(dop[2], 0, 'f', 1).arg(dop[3], 0, 'f', 1)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Real Estimated States")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtk.na)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of All Estimated States")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtk.nx)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Pos X/Y/Z Single (m) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(rtk.sol.rr[0], 0, 'f', 3).arg(rtk.sol.rr[1], 0, 'f', 3).arg(rtk.sol.rr[2], 0, 'f', 3)));

    if (norm(rtk.sol.rr, 3) > 0.0) ecef2pos(rtk.sol.rr, pos); else pos[0] = pos[1] = pos[2] = 0.0;
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Lat/Lon/Height Single (deg,m) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(pos[0] * R2D, 0, 'f', 8).arg(pos[1] * R2D, 0, 'f', 8).arg(pos[2], 0, 'f', 3)));

    ecef2enu(pos, rtk.sol.rr + 3, vel);
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Vel E/N/U (m/s) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(vel[0], 0, 'f', 3).arg(vel[1], 0, 'f', 3).arg(vel[2], 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Pos X/Y/Z Float (m) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3")
                              .arg(rtk.x ? rtk.x[0] : 0, 0, 'f', 3).arg(rtk.x ? rtk.x[1] : 0, 0, 'f', 3).arg(rtk.x ? rtk.x[2] : 0, 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Pos X/Y/Z Float Std (m) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3")
                              .arg(rtk.P ? SQRT(rtk.P[0]) : 0, 0, 'f', 3).arg(rtk.P ? SQRT(rtk.P[1 + 1 * rtk.nx]) : 0, 0, 'f', 3).arg(rtk.P ? SQRT(rtk.P[2 + 2 * rtk.nx]) : 0, 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Pos X/Y/Z Fixed (m) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3")
                              .arg(rtk.xa ? rtk.xa[0] : 0, 0, 'f', 3).arg(rtk.xa ? rtk.xa[1] : 0, 0, 'f', 3).arg(rtk.xa ? rtk.xa[2] : 0, 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Pos X/Y/Z Fixed Std (m) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3")
                              .arg(rtk.Pa ? SQRT(rtk.Pa[0]) : 0, 0, 'f', 3).arg(rtk.Pa ? SQRT(rtk.Pa[1 + 1 * rtk.na]) : 0, 0, 'f', 3).arg(rtk.Pa ? SQRT(rtk.Pa[2 + 2 * rtk.na]) : 0, 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Pos X/Y/Z (m) Base Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(rtk.rb[0], 0, 'f', 3).arg(rtk.rb[1], 0, 'f', 3).arg(rtk.rb[2], 0, 'f', 3)));

    if (norm(rtk.rb, 3) > 0.0) ecef2pos(rtk.rb, pos); else pos[0] = pos[1] = pos[2] = 0.0;
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Lat/Lon/Height (deg,m) Base Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(pos[0] * R2D, 0, 'f', 8).arg(pos[1] * R2D, 0, 'f', 8).arg(pos[2], 0, 'f', 3)));

    ecef2enu(pos, rtk.rb + 3, vel);
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Vel E/N/U (m/s) Base Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(vel[0], 0, 'f', 3).arg(vel[1], 0, 'f', 3).arg(vel[2], 0, 'f', 3)));

    if (norm(rtk.rb,3)>0.0) {
        for (k=0;k<3;k++) rr[k]=rtk.sol.rr[k]-rtk.rb[k];
        ecef2enu(pos,rr,enu);
    }
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Baseline Length/E/N/U (m) Rover-Base Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(norm(rr,3),0,'f',3).arg(enu[0],0,'f',3).arg(enu[1],0,'f',3).arg(enu[2],0,'f',3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of Averaging Single Pos Base Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1").arg(nave)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Antenna Type Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtk.opt.pcvr[0].type));

    for (j=0;j<NFREQ;j++) {
        off=rtk.opt.pcvr[0].off[j];
        tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Antenna Phase Center L%1 E/N/U (m) Rover").arg(j+1)));
        tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(off[0], 0, 'f', 3).arg(off[1], 0, 'f', 3).arg(off[2], 0, 'f', 3)));
    }

    del = rtk.opt.antdel[0];
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Antenna Delta E/N/U (m) Rover")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(del[0], 0, 'f', 3).arg(del[1], 0, 'f', 3).arg(del[2], 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Antenna Type Base Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtk.opt.pcvr[1].type));

    for (j=0;j<NFREQ;j++) {
        off=rtk.opt.pcvr[1].off[0];
        tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Antenna Phase Center L%1 E/N/U (m) Base Station").arg(j+1)));
        tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(off[0], 0, 'f', 3).arg(off[1], 0, 'f', 3).arg(off[2], 0, 'f', 3)));
    }

    del = rtk.opt.antdel[1];
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Antenna Delta E/N/U (m) Base Station")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(del[0], 0, 'f', 3).arg(del[1], 0, 'f', 3).arg(del[2], 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Precise Ephemeris Time/# of Epoch")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1-%2 (%3)").arg(s1).arg(s2).arg(ne)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Precise Ephemeris Download Time")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(s3));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Precise Ephemeris Download File")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(file));
}
//---------------------------------------------------------------------------
void MonitorDialog::setSat()
{
    int i, j = 0;
    const QString label[] = {
        tr("SAT"),	tr("PRN"), tr("PRN"),	tr("Status"), tr("Azimuth (deg)"), tr("Elevation (deg)"), tr("LG (m)"), tr("PHW(cyc)"),
        tr("P1-P2(m)"), tr("P1-C1(m)"), tr("P2-C2(m)")
  };
    int width[] = {25, 25, 30, 45, 45, 60, 60, 40, 40, 40}, nfreq;

    rtksvrlock(&rtksvr);
    nfreq = rtksvr.rtk.opt.nf;
    rtksvrunlock(&rtksvr);

    tWConsole->setColumnCount(9 + nfreq * 8);
    tWConsole->setRowCount(2);
    header.clear();


    j = 0;
    for (i = 0; i < 4; i++) {
        tWConsole->setColumnWidth(j++, width[i] * fontScale / 96);
        header << label[i];
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 30 * fontScale / 96);
        header << QString(tr("L%1")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 40 * fontScale / 96);
        header << QString(tr("Fix%1")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 45 * fontScale / 96);
        header << QString(tr("P%1 Residual(m)")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 45 * fontScale / 96);
        header << QString(tr("L%1 Residual(m)")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 45 * fontScale / 96);
        header << QString(tr("Slip%1")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 45 * fontScale / 96);
        header << QString(tr("Lock%1")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 45 * fontScale / 96);
        header << QString(tr("Outage%1")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 45 * fontScale / 96);
        header << QString(tr("Reject%1")).arg(i+1);
  }
    for (i = 0; i < nfreq; i++) {
        tWConsole->setColumnWidth(j++, 50 * fontScale / 96);
        header << QString(tr("WaveL%1(m)")).arg(i+1);
  }
    for (i = 4; i < 9; i++) {
        tWConsole->setColumnWidth(j++, width[i] * fontScale / 96);
        header << label[i];
  }
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSat()
{
  rtk_t rtk;
  ssat_t *ssat;
    int i, j, k, n, fix, nfreq, sys = sys_tbl[cBSelectNavigationSystems->currentIndex()];
    int vsat[MAXSAT] = {0};
  char id[32];
    double az, el, cbias[MAXSAT][2];

    setSat();

  rtksvrlock(&rtksvr);
    rtk = rtksvr.rtk;

    for (i = 0; i < MAXSAT; i++)
    {
        for (j = 0; j < 2; j++)
            cbias[i][j] = rtksvr.nav.cbias[i][j][0];
    }
    nfreq = rtksvr.rtk.opt.nf;
  rtksvrunlock(&rtksvr);

    lblInformation->setText("");

    for (i = 0; i < MAXSAT; i++) {
        ssat = rtk.ssat + i;
        vsat[i] = ssat->vs;
    }

    for (i = 0, n = 1; i < MAXSAT; i++) {
        if (!(satsys(i + 1, NULL) & sys)) continue;
        if (cBSelectSatellites->currentIndex() == 1 && !vsat[i]) continue;
    n++;
  }
    if (n < 2) {
        tWConsole->setRowCount(0);
    return;
  }
    tWConsole->setRowCount(n - 1);
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0, n = 0; i < MAXSAT; i++) {
        if (!(satsys(i + 1, NULL) & sys)) continue;
        j = 0;
        ssat = rtk.ssat + i;
        if (cBSelectSatellites->currentIndex() == 1 && !vsat[i]) continue;
        satno2id(i + 1, id);
        tWConsole->setItem(n, j++, new QTableWidgetItem(id));
        tWConsole->setItem(n, j++, new QTableWidgetItem(ssat->vs ? tr("OK") : tr("-")));
        az = ssat->azel[0] * R2D; if (az < 0.0) az += 360.0;
        el = ssat->azel[1] * R2D;
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(az, 'f', 1)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(el, 'f', 1)));
        for (k = 0; k < nfreq; k++)
            tWConsole->setItem(n, j++, new QTableWidgetItem(ssat->vsat[k] ? tr("OK") : tr("-")));
        for (k = 0; k < nfreq; k++) {
            fix = ssat->fix[k];
            tWConsole->setItem(n, j++, new QTableWidgetItem(fix == 1 ? tr("FLOAT") : (fix == 2 ? tr("FIX") : (fix == 3 ? tr("HOLD") : tr("-")))));
    }
        for (k = 0; k < nfreq; k++)
            tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->resp[k], 'f', 2)));
        for (k = 0; k < nfreq; k++)
            tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->resc[k], 'f', 4)));
        for (k = 0; k < nfreq; k++)
            tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->slipc[k])));
        for (k = 0; k < nfreq; k++)
            tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->lock[k])));
        for (k = 0; k < nfreq; k++)
            tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->outc[k])));
        for (k = 0; k < nfreq; k++)
            tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->rejc[k])));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->gf[0], 'f', 3)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(ssat->phw, 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(cbias[i][0], 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(cbias[i][1], 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(0, 'f', 2)));
    n++;
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setEstimates()
{
    QString label[] = {
        tr("State"), tr("Estimate Float"), tr("Std Float"), tr("Estimate Fixed"), tr("Std Fixed")
  };
    int i, width[] = {40, 100, 100, 100, 100};

    tWConsole->setColumnCount(5);
    tWConsole->setRowCount(2);
    header.clear();

    for (i = 0; i < tWConsole->columnCount(); i++) {
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
        header << label[i];
    }
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showEstimates()
{
  gtime_t time;
    unsigned int i, nx, na, n;
    double *x, *P = NULL, *xa = NULL, *Pa = NULL;
    QString s0 = "-";
  char tstr[64];

  rtksvrlock(&rtksvr);

    time = rtksvr.rtk.sol.time;
    nx = (unsigned int)rtksvr.rtk.nx;
    na = (unsigned int)rtksvr.rtk.na;
    if ((x = (double *)malloc(sizeof(double) * nx)) &&
        (P = (double *)malloc(sizeof(double) * nx * nx)) &&
        (xa = (double *)malloc(sizeof(double) * na)) &&
        (Pa = (double *)malloc(sizeof(double) * na * na))) {
        memcpy(x, rtksvr.rtk.x, sizeof(double) * nx);
        memcpy(P, rtksvr.rtk.P, sizeof(double) * nx * nx);
        memcpy(xa, rtksvr.rtk.xa, sizeof(double) * na);
        memcpy(Pa, rtksvr.rtk.Pa, sizeof(double) * na * na);
    } else {
        rtksvrunlock(&rtksvr);
        free(x); free(P); free(xa); free(Pa);
        return;
    }
  rtksvrunlock(&rtksvr);

    for (i = 0, n = 0; i < nx; i++) {
        if (cBSelectSatellites->currentIndex() == 1 && x[i] == 0.0) continue;
    n++;
  }
    if (n < 2) {
        tWConsole->setRowCount(0);
        free(x); free(P); free(xa); free(Pa);
    return;
  }
    tWConsole->setRowCount(n);
    tWConsole->setHorizontalHeaderLabels(header);

    time2str(time, tstr, 9);
    lblInformation->setText(time.time ? QString("Time: %1").arg(tstr) : s0);
    for (i = 0, n = 1; i < nx; i++) {
        int j = 0;
        if (cBSelectSatellites->currentIndex() == 1 && x[i] == 0.0) continue;
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString(tr("X_%1")).arg(i + 1)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(x[i] == 0.0 ? s0 : QString::number(x[i], 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(P[i + i * nx] == 0.0 ? s0 : QString::number(SQRT(P[i + i * nx]), 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem((i >= na || qFuzzyCompare(xa[i], 0)) ? s0 : QString::number(xa[i], 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem((i >= na || Pa[i + i * na] == 0.0) ? s0 : QString::number(SQRT(Pa[i + i * na]), 'f', 3)));
    n++;
  }
  free(x); free(P); free(xa); free(Pa);
}
//---------------------------------------------------------------------------
void MonitorDialog::setCovariance()
{
  int i;

    header.clear();

    tWConsole->setColumnCount(2);
    tWConsole->setRowCount(2);

    for (i = 0; i < 2; i++)
        tWConsole->setColumnWidth(i, (i == 0 ? 35 : 45) * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showCovariance()
{
  gtime_t time;
    int i, j, nx, n, m;
    double *x, *P = NULL;
    QString s0 = "-";
  char tstr[64];

  rtksvrlock(&rtksvr);

    time = rtksvr.rtk.sol.time;
    nx = rtksvr.rtk.nx;
    if ((x = (double *)malloc(sizeof(double) * nx)) &&
        (P = (double *)malloc(sizeof(double) * nx * nx))) {
        memcpy(x, rtksvr.rtk.x, sizeof(double) * nx);
        memcpy(P, rtksvr.rtk.P, sizeof(double) * nx * nx);
    } else {
        free(x); free(P);
        rtksvrunlock(&rtksvr);
        return;
    }
  rtksvrunlock(&rtksvr);

    for (i = 0, n = 0; i < nx; i++) {
        if (cBSelectSatellites->currentIndex() == 1 && (x[i] == 0.0 || P[i + i * nx] == 0.0)) continue;
    n++;
  }
    if (n < 2) {
        tWConsole->setColumnCount(0);
        tWConsole->setRowCount(0);
        free(x); free(P);
        return;
  }
    tWConsole->setColumnCount(n);
    tWConsole->setRowCount(n);

    time2str(time, tstr, 9);
    lblInformation->setText(time.time ? QString(tr("Time: %1")).arg(tstr) : s0);
    for (i = 0, n = 0; i < nx; i++) {
        if (cBSelectSatellites->currentIndex() == 1 && (x[i] == 0.0 || P[i + i * nx] == 0.0)) continue;
        tWConsole->setColumnWidth(n, 45 * fontScale / 96);
        tWConsole->setHorizontalHeaderItem(n, new QTableWidgetItem(QString(tr("X_%1")).arg(i + 1)));
        tWConsole->setVerticalHeaderItem(n, new QTableWidgetItem(QString(tr("X_%1")).arg(i + 1)));
        for (j = 0, m = 0; j < nx; j++) {
            if (cBSelectSatellites->currentIndex() == 1 && (x[j] == 0.0 || P[j + j * nx] == 0.0)) continue;
            tWConsole->setItem(n, m, new QTableWidgetItem(P[i + j * nx] == 0.0 ? s0 : QString::number(SQRT(P[i + j * nx]), 'f', 5)));
      m++;
    }
    n++;
  }
  free(x); free(P);
}
//---------------------------------------------------------------------------
void MonitorDialog::setObservations()
{
    const QString label[] = {tr("Trcv (GPST)"), tr("SAT"), tr("STR")};
    int i, j = 0, width[] = {135, 25, 25};
    int nex = observationMode ? NEXOBS : 0;

    tWConsole->setColumnCount(3 + (NFREQ + nex) * 6);
    tWConsole->setRowCount(0);
    header.clear();

    for (i = 0; i < 3; i++) {
        tWConsole->setColumnWidth(j++, width[i] * fontScale / 96);
        header << label[i];
  }
    for (i = 0; i < NFREQ + nex; i++) {
        tWConsole->setColumnWidth(j++, 22 * fontScale / 96);
        header << (i < NFREQ ? QString(tr("C%1")).arg(i+1) : QString(tr("CX%1")).arg(i - NFREQ + 1));
    }
    for (i = 0; i < NFREQ + nex; i++) {
        tWConsole->setColumnWidth(j++, 30 * fontScale / 96);
        header << (i < NFREQ ? QString(tr("S%1")).arg(i+1) : QString(tr("SX%1")).arg(i - NFREQ + 1));
    }
    for (i = 0; i < NFREQ + nex; i++) {
        tWConsole->setColumnWidth(j++, 80 * fontScale / 96);
        header << (i < NFREQ ? QString(tr("P%1 (m)")).arg(i+1) : QString(tr("PX%1 (m)")).arg(i - NFREQ + 1));
    }
    for (i = 0; i < NFREQ + nex; i++) {
        tWConsole->setColumnWidth(j++, 85 * fontScale / 96);
        header << (i < NFREQ ? QString(tr("L%1 (cycle)")).arg(i+1) : QString(tr("LX%1 (cycle)")).arg(i - NFREQ + 1));
  }
    for (i = 0; i < NFREQ + nex; i++) {
        tWConsole->setColumnWidth(j++, 60 * fontScale / 96);
        header << (i < NFREQ ? QString(tr("D%1 (Hz)")).arg(i+1) : QString(tr("DX%1 (Hz)")).arg(i - NFREQ + 1));
  }
    for (i = 0; i < NFREQ + nex; i++) {
        tWConsole->setColumnWidth(j++, 15 * fontScale / 96);
        header << "I";
  }
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showObservations()
{
    obsd_t obs[MAXOBS * 2];
    char tstr[64], id[32], *code;
    int i, k, n = 0, nex = observationMode ? NEXOBS : 0, sys = sys_tbl[cBSelectNavigationSystems->currentIndex()];

  rtksvrlock(&rtksvr);
    for (i = 0; i < rtksvr.obs[0][0].n && n < MAXOBS * 2; i++) {
        if (!(satsys(rtksvr.obs[0][0].data[i].sat, NULL) & sys)) continue;
        obs[n++] = rtksvr.obs[0][0].data[i];
    }
    for (i = 0; i < rtksvr.obs[1][0].n && n < MAXOBS * 2; i++) {
        if (!(satsys(rtksvr.obs[1][0].data[i].sat, NULL) & sys)) continue;
        obs[n++] = rtksvr.obs[1][0].data[i];
    }
  rtksvrunlock(&rtksvr);

    tWConsole->setRowCount(n + 1 < 2 ? 0 : n);
    tWConsole->setColumnCount(3 + (NFREQ + nex) * 6);
    lblInformation->setText("");
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < n; i++) {
        int j = 0;
        time2str(obs[i].time, tstr, 3);
        tWConsole->setItem(i, j++, new QTableWidgetItem(tstr));
        satno2id(obs[i].sat, id);
        tWConsole->setItem(i, j++, new QTableWidgetItem(id));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString("(%1)").arg(obs[i].rcv)));
        for (k = 0; k < NFREQ + nex; k++) {
            code = code2obs(obs[i].code[k]);
            if (*code) tWConsole->setItem(i + 1, j++, new QTableWidgetItem(code));
            else tWConsole->setItem(i + 1, j++, new QTableWidgetItem("-"));
        }
        for (k = 0; k < NFREQ + nex; k++) {
            if (obs[i].SNR[k]) tWConsole->setItem(i + 1, j++, new QTableWidgetItem(QString::number(obs[i].SNR[k]*SNR_UNIT, 'f', 1)));
            else tWConsole->setItem(i + 1, j++, new QTableWidgetItem("-"));
        }
        for (k = 0; k < NFREQ + nex; k++)
            tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(obs[i].P[k], 'f', 3)));
        for (k = 0; k < NFREQ + nex; k++)
            tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(obs[i].L[k], 'f', 3)));
        for (k = 0; k < NFREQ + nex; k++)
            tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(obs[i].D[k], 'f', 3)));
        for (k = 0; k < NFREQ + nex; k++)
            tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(obs[i].LLI[k])));
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setNavigation()
{
    header.clear();
    header	<< tr("SAT") << tr("PRN") << tr("Status") << tr("IODE") << tr("IODC") << tr("URA") << tr("SVH") << tr("Toe") << tr("Toc") << tr("Ttrans")
        << tr("A (m)") << tr("e") << tr("i0 (deg)") << tr("OMEGA0 (deg)") << tr("omega (deg)") << tr("M0 (deg)")
        << tr("deltan (deg/s)") << tr("OMEGAdot (deg/s)") << tr("IDOT (deg/s)")
        << tr("af0 (ns)") << tr("af1 (ns/s)") << tr("af2 (ns/s2)") << tr("TGD (ns)") << tr("BGD5a (ns)") << tr("BGD5b (ns)")
        << tr("Cuc (rad)") << tr("Cus (rad)") << tr("Crc (m)") << tr("Crs (m)") << tr("Cic (rad)") << tr("Cis (rad)") << tr("Code") << tr("Flag");
    int i, width[] = {
        25, 25, 30, 30, 30, 25, 25, 115, 115, 115, 80, 80, 60, 60, 60, 60, 70, 70, 70, 60,
        50, 50, 50, 50, 50, 70, 70, 50,  70,  70, 30, 30
  };
    tWConsole->setColumnCount(32);
    tWConsole->setRowCount(2);

    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showNavigations()
{
  eph_t eph[MAXSAT];
  gtime_t time;
    QString s;
    char tstr[64], id[32];
    int i, k, n, prn, off = cBSelectEphemeris->currentIndex() ? MAXSAT : 0;
    bool valid;
    int sys = sys_tbl[cBSelectSingleNavigationSystem->currentIndex() + 1];

    if (sys == SYS_GLO) {
        setGlonassNavigations();
        showGlonassNavigations();
        return;
    }
    if (sys == SYS_SBS) {
        setSbsNavigations();
        showSbsNavigations();
        return;
    }
    setNavigation();

    rtksvrlock(&rtksvr);
    time = rtksvr.rtk.sol.time;
    for (i = 0; i < MAXSAT; i++) eph[i] = rtksvr.nav.eph[i + off];
    rtksvrunlock(&rtksvr);

    if (sys == SYS_GAL) {
        lblInformation->setText((cBSelectEphemeris->currentIndex() % 2) ? "F/NAV" : "I/NAV");
    }
    else {
        lblInformation->setText("");
    }

    for (k = 0, n = 1; k < MAXSAT; k++) {
        if (!(satsys(k + 1, &prn) & sys)) continue;
        valid = eph[k].toe.time != 0 && !eph[k].svh && fabs(timediff(time, eph[k].toe)) <= MAXDTOE;
        if (cBSelectSatellites->currentIndex() == 1 && !valid) continue;
    n++;
  }
    if (n < 2) {
        tWConsole->setRowCount(0);
    return;
  }
    tWConsole->setRowCount(n);
    tWConsole->setHorizontalHeaderLabels(header);

    for (k = 0, n = 0; k < MAXSAT; k++) {
        int j = 0;
        if (!(satsys(k + 1, &prn) & sys)) continue;
        valid = eph[k].toe.time != 0 && !eph[k].svh && fabs(timediff(time, eph[k].toe)) <= MAXDTOE;
        if (cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        satno2id(k + 1, id);
        tWConsole->setItem(n, j++, new QTableWidgetItem(id));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(prn)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(valid ? tr("OK") : tr("-")));
        if (eph[k].iode < 0) s = "-"; else s = QString::number(eph[k].iode);
        tWConsole->setItem(n, j++, new QTableWidgetItem(s));
        if (eph[k].iodc < 0) s = "-"; else s = QString::number(eph[k].iodc);
        tWConsole->setItem(n, j++, new QTableWidgetItem(s));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].sva)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].svh, 16)));
        if (eph[k].toe.time != 0) time2str(eph[k].toe, tstr, 0); else strcpy(tstr, "-");
        tWConsole->setItem(n, j++, new QTableWidgetItem(tstr));
        if (eph[k].toc.time != 0) time2str(eph[k].toc, tstr, 0); else strcpy(tstr, "-");
        tWConsole->setItem(n, j++, new QTableWidgetItem(tstr));
        if (eph[k].ttr.time != 0) time2str(eph[k].ttr, tstr, 0); else strcpy(tstr, "-");
        tWConsole->setItem(n, j++, new QTableWidgetItem(tstr));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].A, 'f', 3)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].e, 'f', 8)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].i0 * R2D, 'f', 5)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].OMG0 * R2D, 'f', 5)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].omg * R2D, 'f', 5)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].M0 * R2D, 'f', 5)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].deln * R2D, 'E', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].OMGd * R2D, 'E', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].idot * R2D, 'E', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].f0 * 1E9, 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].f1 * 1E9, 'f', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].f2 * 1E9, 'f', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].tgd[0] * 1E9, 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].tgd[1] * 1E9, 'f', 2)));

        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].cuc, 'E', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].cus, 'E', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].crc, 'E', 3)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].crs, 'E', 3)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].cic, 'E', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].cis, 'E', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].code, 16)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(eph[k].flag, 16)));
        n++;
  }
    tWConsole->setRowCount(n);
}
//---------------------------------------------------------------------------
void MonitorDialog::setGlonassNavigations()
{
    header.clear();
    header << tr("SAT") << tr("PRN") << tr("Status") << tr("IOD") << tr("FCN") << tr("SVH") << tr("Age(days)") << tr("Toe") << tr("Tof")
           << tr("X (m)") << tr("Y (m)") << tr("Z (m)") << tr("Vx (m/s)") << tr("Vy (m/s)") << tr("Vz (m/s)")
           << tr("Ax (m/s2)") << tr("Ay (m/s2)") << tr("Az (m/s2)") << tr("Tau (ns)") << tr("Gamma (ns/s)");

    int i, width[] = {
        25, 25, 30, 30, 30, 25, 25, 115, 115, 75, 75, 75, 70, 70, 70, 65, 65, 65, 70, 60, 50
  };
    tWConsole->setColumnCount(21);
    tWConsole->setRowCount(1);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showGlonassNavigations()
{
  geph_t geph[NSATGLO];
  gtime_t time;
    QString s;
    char tstr[64], id[32];
    int i, n, valid, prn, off = cBSelectEphemeris->currentIndex() ? NSATGLO : 0;

  rtksvrlock(&rtksvr);
    time = rtksvr.rtk.sol.time;
    for (i = 0; i < NSATGLO; i++) geph[i] = rtksvr.nav.geph[i + off];
  rtksvrunlock(&rtksvr);

    lblInformation->setText("");

    for (i = 0, n = 0; i < NSATGLO; i++) {
        valid = geph[i].toe.time != 0 && !geph[i].svh &&
            fabs(timediff(time, geph[i].toe)) <= MAXDTOE_GLO;
        if (cBSelectSatellites->currentIndex() == 1 && !valid) continue;
    n++;
  }
    if (n < 1) {
        tWConsole->setRowCount(1);
    return;
  }
    tWConsole->setRowCount(n + 1);
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0, n = 0; i < NSATGLO; i++) {
        int j = 0;
        valid = geph[i].toe.time != 0 && !geph[i].svh &&
            fabs(timediff(time, geph[i].toe)) <= MAXDTOE_GLO;
        if (cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        prn = MINPRNGLO + i;
        satno2id(satno(SYS_GLO, prn), id);
        tWConsole->setItem(n, j++, new QTableWidgetItem(id));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(prn)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(valid ? tr("OK") : tr("-")));
        if (geph[i].iode < 0) s = "-"; else s = QString::number(geph[i].iode);
        tWConsole->setItem(n, j++, new QTableWidgetItem(s));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].frq)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].svh, 16)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].age)));
        if (geph[i].toe.time != 0) time2str(geph[i].toe, tstr, 0); else strcpy(tstr, "-");
        tWConsole->setItem(n, j++, new QTableWidgetItem(tstr));
        if (geph[i].tof.time != 0) time2str(geph[i].tof, tstr, 0); else strcpy(tstr, "-");
        tWConsole->setItem(n, j++, new QTableWidgetItem(tstr));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].pos[0], 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].pos[1], 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].pos[2], 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].vel[0], 'f', 5)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].vel[1], 'f', 5)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].vel[2], 'f', 5)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].acc[0], 'f', 7)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].acc[1], 'f', 7)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].acc[2], 'f', 7)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].taun * 1E9, 'f', 2)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].gamn * 1E9, 'f', 4)));
        tWConsole->setItem(n, j++, new QTableWidgetItem(QString::number(geph[i].dtaun * 1E9, 'f', 2)));
        n++;
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsNavigations()
{
    header.clear();
    header	<< tr("SAT") << tr("PRN") << tr("Status") << tr("T0") << tr("Tof") << tr("SVH") << tr("URA") << tr("X (m)") << tr("Y (m)") << tr("Z (m)") << tr("Vx (m/s)")
        << tr("Vy (m/s)") << tr("Vz (m/s)") << tr("Ax (m/s2)") << tr("Ay (m/s2)") << tr("Az (m/s2)")
        << tr("af0 (ns)") << tr("af1 (ns/s)");

    int i, width[] = {25, 25, 30, 115, 115, 30, 30, 75, 75, 75, 70, 70, 70, 65, 65, 65, 60, 60};

    tWConsole->setColumnCount(18);
    tWConsole->setRowCount(1);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsNavigations()
{
    seph_t seph[MAXPRNSBS - MINPRNSBS + 1];
  gtime_t time;
    int i, n, valid, prn, off = cBSelectEphemeris->currentIndex() ? NSATSBS : 0;
    char tstr[64], id[32];

  rtksvrlock(&rtksvr); // lock
    time = rtksvr.rtk.sol.time;
    for (int i = 0; i < NSATSBS; i++) seph[i] = rtksvr.nav.seph[i + off];
  rtksvrunlock(&rtksvr); // unlock

    lblInformation->setText("");

    for (i = 0, n = 0; i < NSATSBS; i++) {
        valid = fabs(timediff(time, seph[i].t0)) <= MAXDTOE_SBS &&
            seph[i].t0.time && !seph[i].svh;
        if (cBSelectSatellites->currentIndex() == 1 && !valid) continue;
    n++;
  }
    if (n < 1) {
        tWConsole->setRowCount(0);
    return;
  }
    tWConsole->setRowCount(n);
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0, n = 0; i < NSATSBS; i++) {
        int j = 0;
        valid = fabs(timediff(time, seph[i].t0)) <= MAXDTOE_SBS &&
            seph[i].t0.time && !seph[i].svh;
        if (cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        prn = MINPRNSBS + i;
        satno2id(satno(SYS_SBS, prn), id);
        tWConsole->setItem(i, j++, new QTableWidgetItem(id));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(prn)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(valid ? tr("OK") : tr("-")));
        if (seph[i].t0.time) time2str(seph[i].t0, tstr, 0);
        else strcpy(tstr, "-");
        tWConsole->setItem(i, j++, new QTableWidgetItem(tstr));
        if (seph[i].tof.time) time2str(seph[i].tof, tstr, 0);
        else strcpy(tstr, "-");
        tWConsole->setItem(i, j++, new QTableWidgetItem(tstr));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].svh, 16)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].sva)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].pos[0], 'f', 2)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].pos[1], 'f', 2)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].pos[2], 'f', 2)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].vel[0], 'f', 6)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].vel[1], 'f', 6)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].vel[2], 'f', 6)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].acc[0], 'f', 7)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].acc[1], 'f', 7)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].acc[2], 'f', 7)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].af0 * 1E9, 'f', 2)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(seph[i].af1 * 1E9, 'f', 4)));
    n++;
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setIonUtc()
{
    header.clear();
    header << tr("Parameter") << tr("Value");
    int i, width[] = {270, 330};

    tWConsole->setColumnCount(2);
    tWConsole->setRowCount(1);
    for (i = 0; i < tWConsole->columnCount() & i < 2; i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showIonUtc()
{
    double utc_gps[8], utc_glo[8], utc_gal[8], utc_qzs[8], utc_cmp[8],utc_irn[9];
    double ion_gps[8], ion_gal[4], ion_qzs[8], ion_cmp[8], ion_irn[8];
  gtime_t time;
    double tow = 0.0;
  char tstr[64];
    int i, week = 0;

    Q_UNUSED(utc_glo);

  rtksvrlock(&rtksvr);
    time = rtksvr.rtk.sol.time;
    for (i = 0; i < 8; i++) utc_gps[i] = rtksvr.nav.utc_gps[i];
    for (i = 0; i < 8; i++) utc_glo[i] = rtksvr.nav.utc_glo[i];
    for (i = 0; i < 8; i++) utc_gal[i] = rtksvr.nav.utc_gal[i];
    for (i = 0; i < 8; i++) utc_qzs[i] = rtksvr.nav.utc_qzs[i];
    for (i = 0; i < 8; i++) utc_cmp[i] = rtksvr.nav.utc_cmp[i];
    for (i = 0; i < 9; i++) utc_irn[i] = rtksvr.nav.utc_irn[i];
    for (i = 0; i < 8; i++) ion_gps[i] = rtksvr.nav.ion_gps[i];
    for (i = 0; i < 4; i++) ion_gal[i] = rtksvr.nav.ion_gal[i];
    for (i = 0; i < 8; i++) ion_qzs[i] = rtksvr.nav.ion_qzs[i];
    for (i = 0; i < 8; i++) ion_cmp[i] = rtksvr.nav.ion_cmp[i];
    for (i = 0; i < 8; i++) ion_irn[i] = rtksvr.nav.ion_irn[i];
  rtksvrunlock(&rtksvr);

    lblInformation->setText("");

    tWConsole->setRowCount(20);
    tWConsole->setHorizontalHeaderLabels(header);
    i = 0;

    time2str(timeget(), tstr, 3);
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("CPU Time (UTC)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(tstr));

    if (time.time != 0) time2str(gpst2utc(time), tstr, 3); else strcpy(tstr, "-");
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Receiver Time (UTC)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(tstr));

    if (time.time != 0) time2str(time, tstr, 3); else strcpy(tstr, "-");
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Receiver Time (GPST)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(tstr));

    if (time.time != 0) tow = time2gpst(time, &week);
    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("GPS Week/Time (s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2").arg(week).arg(tow, 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Leap Seconds dt_LS(s), WN_LSF,DN, dt_LSF(s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(utc_gps[4], 0, 'f', 0).arg(utc_gps[5], 0, 'f', 0).arg(utc_gps[6], 0, 'f', 0).arg(utc_gps[7], 0, 'f', 0)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("GPST-UTC Reference Week/Time (s), A0(ns), A1(ns/s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(utc_gps[3], 0, 'f', 0).arg(utc_gps[2], 0, 'f', 0).arg(utc_gps[0]*1E9, 0, 'f', 3).arg(utc_gps[1]*1E9, 0, 'E', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("GLOT-UTC Tau, Tau_GPS(ns)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2").arg(utc_glo[0], 0, 'f', 9).arg(utc_glo[1] * 1E9, 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("GTS-UTC Ref Week, Time(s), A0(ns), A1(ns/s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1 ,%2").arg(utc_gal[3], 0, 'f',0).arg(utc_gal[2], 0, 'f', 0).arg(utc_gal[0]*1e9, 0, 'f', 3).arg(utc_gal[1] * 1E9, 0, 'f', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("QZSST-UTC Ref Week, Time(s), A0(ns), A1(ns/s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(tr("%1, %2, %3, %4").arg(utc_qzs[3],0,'f',0).arg(utc_qzs[2],0,'f',0).arg(utc_qzs[0]*1E9, 0, 'f', 3).arg(utc_qzs[1]*1E9, 0, 'f', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("BDST-UTC Ref Week, Time(s), A0(ns), A1(ns/s)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(utc_cmp[3], 0, 'f', 0).arg(utc_cmp[2], 0, 'f', 0).arg(utc_cmp[0]*1e9 , 0, 'f', 3).arg(utc_cmp[1]*1e9, 0, 'f', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("IRNT-UTC Ref Week,Time(s), A0(ns), A1(ns/s), A2(ns/s2)")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(utc_irn[3], 0, 'f', 0).arg(utc_irn[2], 0, 'f', 0).arg(utc_irn[0]*1e9, 0, 'f', 3).arg(utc_irn[1]*1e9, 0, 'f', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("GPS Iono Parameters Alpha0-3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2,%3, %4").arg(ion_gps[0], 0, 'f', 5).arg(ion_gps[1], 0, 'f', 5).arg(ion_gps[2], 0, 'f', 5).arg(ion_gps[3], 0, 'f', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("GPS Iono Parameters Beta0-3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(ion_gps[4], 0, 'f', 5).arg(ion_gps[5], 0, 'f', 5).arg(ion_gps[6], 0, 'f', 5).arg(ion_gps[7], 0, 'f', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Galileo Iono Parameters 0-2")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2,%3").arg(ion_gal[0], 0, 'E', 5).arg(ion_gal[1], 0, 'E', 5).arg(ion_gal[2], 0, 'E', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("QZSS Iono Parameters Alpha0-Alpha3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2,%3,%4").arg(ion_qzs[0], 0, 'E', 5).arg(ion_qzs[1], 0, 'E', 5).arg(ion_qzs[2], 0, 'E', 5).arg(ion_qzs[3], 0, 'E', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("QZSS Iono Parameters Beta0-Beta3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2,%3,%4").arg(ion_qzs[4], 0, 'E', 5).arg(ion_qzs[5], 0, 'E', 5).arg(ion_qzs[6], 0, 'E', 5).arg(ion_qzs[7], 0, 'E', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("BDS Iono Parameters Alpha0-Alpha3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2,%3,%4").arg(ion_cmp[0], 0, 'E', 5).arg(ion_cmp[1], 0, 'E', 5).arg(ion_cmp[2], 0, 'E', 5).arg(ion_cmp[3], 0, 'E', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("BDS Iono Parameters Beta0-Beta3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2,%3,%4").arg(ion_cmp[4], 0, 'E', 5).arg(ion_cmp[5], 0, 'E', 5).arg(ion_cmp[6], 0, 'E', 5).arg(ion_cmp[7], 0, 'E', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("NavIC Iono Parameters Alpha0-Alpha3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2,%3,%4").arg(ion_irn[0], 0, 'E', 5).arg(ion_irn[1], 0, 'E', 5).arg(ion_irn[2], 0, 'E', 5).arg(ion_irn[3], 0, 'E', 5)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("NavIC Iono Parameters Beta0-Beta3")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1,%2,%3,%4").arg(ion_irn[4], 0, 'E', 5).arg(ion_irn[5], 0, 'E', 5).arg(ion_irn[6], 0, 'E', 5).arg(ion_irn[7], 0, 'E', 5)));
}
//---------------------------------------------------------------------------
void MonitorDialog::setStream()
{
    header.clear();
    header	<< tr("STR") << tr("Stream") << tr("Type") << tr("Format") << tr("Mode") << tr("State") << tr("Input (bytes)") << tr("Input (bps)")
        << tr("Output (bytes)") << tr("Output (bps)") << tr("Path") << tr("Message");

    int i, width[] = {25, 95, 70, 80, 35, 35, 70, 70, 70, 70, 220, 220};

    tWConsole->setColumnCount(12);
    tWConsole->setRowCount(1);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showStream()
{
    const QString ch[] = {
        tr("Input Rover"),	 tr("Input Base"), tr("Input Correction"), tr("Output Solution 1"),
        tr("Output Solution 2"), tr("Log Rover"),	tr("Log Base"),   tr("Log Correction"),
        tr("Monitor")
  };
    const QString type[] = {
        tr("-"), tr("Serial"), tr("File"), tr("TCP Server"), tr("TCP Client"), tr("NTRIP Server"),
        tr("NTRIP Client"), tr("FTP"), tr("HTTP"), tr("NTRIP Caster"), tr("UDP Server"),
        tr("UDP Client"), tr("")
  };
    const QString outformat[] = {
        tr("Lat/Lon/Height"), tr("X/Y/Z-ECEF"), tr("E/N/U-Baseline"), tr("NMEA-0183")
  };
    const QString state[] = { tr("Error"), tr("-"), tr("OK") };
    QString mode, form;
  stream_t stream[9];
    int i, format[9] = {0};
    char path[MAXSTRPATH] = "", *p, *q;

  rtksvrlock(&rtksvr); // lock
    for (i = 0; i < 8; i++) stream[i] = rtksvr.stream[i];
    for (i = 0; i < 3; i++) format[i] = rtksvr.format[i];
    for (i = 3; i < 5; i++) format[i] = rtksvr.solopt[i - 3].posf;
    stream[8] = monistr;
    format[8] = SOLF_LLH;
  rtksvrunlock(&rtksvr); // unlock

    tWConsole->setRowCount(9);
    lblInformation->setText("");
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < 9; i++) {
        int j = 0;
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString("(%1)").arg(i+1)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(ch[i]));
        tWConsole->setItem(i, j++, new QTableWidgetItem(type[stream[i].type]));
        if (!stream[i].type) form = "-";
        else if (i < 3) form = formatstrs[format[i]];
        else if (i < 5 || i == 8) form = outformat[format[i]];
        else form = "-";
        tWConsole->setItem(i, j++, new QTableWidgetItem(form));
        if (stream[i].mode & STR_MODE_R) mode = tr("R"); else mode = "";
        if (stream[i].mode & STR_MODE_W) mode = mode + (mode == "" ? "" : "/") + tr("W");
        tWConsole->setItem(i, j++, new QTableWidgetItem(mode));
        tWConsole->setItem(i, j++, new QTableWidgetItem(state[stream[i].state + 1]));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(stream[i].inb)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(stream[i].inr)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(stream[i].outb)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(stream[i].outr)));
        strncpy(path, stream[i].path, 1024);
        char *pp = path;
        if ((p = strchr(path, '@'))) {
            for (q = p - 1; q >= path; q--) if (*q == ':') break;
            if (q >= path) for (q++; q < p; q++) *q = '*';
    }
        if (stream[i].type == STR_TCPCLI || stream[i].type == STR_TCPSVR) {
            if ((p = strchr(path, '/'))) *p = '\0';
            if ((p = strchr(path, '@'))) pp = p + 1;
            if (stream[i].type == STR_TCPSVR) {
                if ((p = strchr(pp, ':'))) pp = p + 1; else *pp = ' ';
      }
    }
        tWConsole->setItem(i, j++, new QTableWidgetItem(pp));
        tWConsole->setItem(i, j++, new QTableWidgetItem(stream[i].msg));
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsMessages()
{
    header.clear();
    header << tr("Trcv") << tr("PRN") << tr("STR") << tr("Type") << tr("Message") << tr("Contents");
    int i, width[] = {115, 25, 25, 25, 420, 200};

    tWConsole->setColumnCount(6);
    tWConsole->setRowCount(1);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsMessages()
{
  sbsmsg_t msg[MAXSBSMSG];
    const QString content[] = {
        tr("For Testing"), tr("PRN Mask"), tr("Fast Corrections"), tr("Fast Corrections"),
        tr("Fast Corrections"), tr("Fast Corrections"), tr("Integrity Information"),
        tr("Fast Correction Degradation Factor"), tr("GEO Navigation Message"),
        tr("Degradation Parameters"), tr("WAAS Network Time/UTC Offset Parameters"),
        tr("GEO Satellite Almanacs"),  tr("Ionospheric Grid Point Masks"),
        tr("Mixed Fast Corrections/Long Term Satellite Error Corrections"),
        tr("Long Term Satellite Error Corrections"), tr("Ionospheric Delay Corrections"),
        tr("WAAS Service Messages"), tr("Clock-Ephemeris Covariance Matrix Message"),
        tr("Internal Test Message"), tr("Null Message"),
        tr("QZSS: DC Report (JMA)"), tr("QZSS: DC Report (Other)"),
        tr("QZSS: Monitoring Station Info"), tr("QZSS: PRN Mask"),
        tr("QZSS: Data Issue Number"), tr("QZSS: DGPS Correction"),
        tr("QZSS: Satellite Health"),
        ""
  };
    const int id[] = {0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 12, 17, 18, 24, 25, 26, 27, 28, 62, 63, 43, 44, 47, 48, 49, 50, 51, -1};
    char str[256];
    QString s;
    int i, k, n;

  rtksvrlock(&rtksvr); // lock
    for (i = n = 0; i < rtksvr.nsbs; i++) {
        msg[n++] = rtksvr.sbsmsg[i];
    }
  rtksvrunlock(&rtksvr); // unlock


    tWConsole->setRowCount(n);
    lblInformation->setText("");
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < n; i++) {
        int j = 0;
        time2str(gpst2time(msg[i].week, msg[i].tow), str, 0);
        tWConsole->setItem(i, j++, new QTableWidgetItem(str));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(msg[i].prn)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString("(%1)").arg(msg[i].rcv)));
        int type = msg[i].msg[1] >> 2;
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(type)));
        for (k = 0; k < 29; k++) s += QString::number(msg[i].msg[k], 16);
        tWConsole->setItem(i, j++, new QTableWidgetItem(s));
        for (k = 0; id[k] >= 0; k++) if (type == id[k]) break;
        tWConsole->setItem(i, j++, new QTableWidgetItem(id[k] < 0 ? "?" : content[k]));
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsLong()
{
    header.clear();
    header	<< tr("SAT") << tr("Status") << tr("IODE") << tr("dX (m)") << tr("dY (m)") << tr("dZ (m)") << tr("dVX (m/s)")
        << tr("dVY (m/s)") << tr("dVZ (m/s)") << tr("daf0 (ns)") << tr("daf1 (ns/s)") << tr("T0");

    int i, width[] = {25, 30, 30, 55, 55, 55, 55, 55, 55, 55, 55, 115};

    tWConsole->setColumnCount(12);
    tWConsole->setRowCount(0);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsLong()
{
  sbssat_t sbssat;
  gtime_t time;
    int i;
    char tstr[64], id[32];

  rtksvrlock(&rtksvr); // lock
    time = rtksvr.rtk.sol.time;
    sbssat = rtksvr.nav.sbssat;
  rtksvrunlock(&rtksvr); // unlock

    tWConsole->setRowCount(sbssat.nsat <= 0 ? 1 : sbssat.nsat);
    lblInformation->setText(QString(tr("IODP:%1  System Latency:%2 s"))
               .arg(sbssat.iodp).arg(sbssat.tlat));
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < sbssat.nsat; i++) {
        int j = 0;
        sbssatp_t *satp = sbssat.sat + i;
        bool valid = timediff(time, satp->lcorr.t0) <= MAXSBSAGEL && satp->lcorr.t0.time;
        satno2id(satp->sat, id);
        tWConsole->setItem(i, j++, new QTableWidgetItem(id));
        tWConsole->setItem(i, j++, new QTableWidgetItem(valid ? tr("OK") : tr("-")));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.iode)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.dpos[0], 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.dpos[1], 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.dpos[2], 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.dvel[0], 'f', 4)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.dvel[1], 'f', 4)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.dvel[2], 'f', 4)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.daf0 * 1E9, 'f', 4)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->lcorr.daf1 * 1E9, 'f', 4)));
        if (satp->lcorr.t0.time) time2str(satp->lcorr.t0, tstr, 0);
        else strcpy(tstr, "-");
        tWConsole->setItem(i, j++, new QTableWidgetItem(tstr));
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsIono()
{
    header.clear();
    header << tr("IODI") << tr("Lat (deg)") << tr("Lon (deg)") << tr("GIVEI") << tr("Delay (m)") << tr("T0");

    int i, width[] = {30, 50, 50, 30, 60, 115};

    tWConsole->setColumnCount(6);
    tWConsole->setRowCount(2);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsIono()
{
    QString s0 = "-";
    sbsion_t sbsion[MAXBAND + 1];
  char tstr[64];
    int i, j, k, n = 0;

  rtksvrlock(&rtksvr); // lock
    for (i = 0; i <= MAXBAND; i++) {
        sbsion[i] = rtksvr.nav.sbsion[i];
        n += sbsion[i].nigp;
    }
    ;
  rtksvrunlock(&rtksvr); // unlock

    tWConsole->setRowCount(n);
    tWConsole->setHorizontalHeaderLabels(header);

    lblInformation->setText("");
    n = 0;
    for (i = 0; i < MAXBAND + 1; i++) {
        sbsion_t *ion = sbsion + i;
        for (j = 0; j < ion->nigp; j++) {
            k = 0;
            tWConsole->setItem(n, k++, new QTableWidgetItem(QString::number(ion->iodi)));
            tWConsole->setItem(n, k++, new QTableWidgetItem(QString::number(ion->igp[j].lat)));
            tWConsole->setItem(n, k++, new QTableWidgetItem(QString::number(ion->igp[j].lon)));
            tWConsole->setItem(n, k++, new QTableWidgetItem(ion->igp[j].give ? QString::number(ion->igp[j].give - 1) : s0));
            tWConsole->setItem(n, k++, new QTableWidgetItem(QString::number(ion->igp[j].delay, 'f', 3)));
            if (ion->igp[j].t0.time) time2str(ion->igp[j].t0, tstr, 0);
            else strcpy(tstr, "-");
            tWConsole->setItem(n, k++, new QTableWidgetItem(tstr));
      n++;
    }
  }
    tWConsole->setRowCount(n <= 0 ? 0 : n + 1);
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsFast()
{
    header.clear();
    header << tr("SAT") << tr("Status") << tr("PRC (m)") << tr("RRC (m)") << tr("IODF") << tr("UDREI") << tr("AI") << tr("Tof");

    int i, width[] = {25, 30, 60, 60, 30, 30, 30, 115};

    tWConsole->setColumnCount(8);
    tWConsole->setRowCount(0);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsFast()
{
    QString s0 = "-";
  sbssat_t sbssat;
  gtime_t time;
    int i;
    char tstr[64], id[32];

  rtksvrlock(&rtksvr); // lock
    time = rtksvr.rtk.sol.time;
    sbssat = rtksvr.nav.sbssat;
  rtksvrunlock(&rtksvr); // unlock

    lblInformation->setText("");
    tWConsole->setRowCount(sbssat.nsat <= 0 ? 1 : sbssat.nsat);
    //lblInformation->setText(QString(tr("IODP:%1  System Latency:%2 s")).arg(sbssat.iodp).arg(sbssat.tlat));
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < sbssat.nsat; i++) {
        int j = 0;
        sbssatp_t *satp = sbssat.sat + i;
        bool valid = fabs(timediff(time, satp->fcorr.t0)) <= MAXSBSAGEF && satp->fcorr.t0.time &&
                 0 <= satp->fcorr.udre - 1 && satp->fcorr.udre - 1 < 14;
        satno2id(satp->sat, id);
        tWConsole->setItem(i, j++, new QTableWidgetItem(id));
        tWConsole->setItem(i, j++, new QTableWidgetItem(valid ? tr("OK") : tr("-")));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->fcorr.prc, 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->fcorr.rrc, 'f', 4)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->fcorr.iodf)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(satp->fcorr.udre ? QString::number(satp->fcorr.udre - 1) : s0));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(satp->fcorr.ai)));
        if (satp->fcorr.t0.time) time2str(satp->fcorr.t0, tstr, 0);
        else strcpy(tstr, "-");
        tWConsole->setItem(i, j++, new QTableWidgetItem(tstr));
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtcm()
{
    header.clear();
    header << tr("Parameter") << tr("Value");
    int i, width[] = {220, 520};

    tWConsole->setColumnCount(2);
    tWConsole->setRowCount(0);
    for (i = 0; i < tWConsole->columnCount() && i < 2; i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showRtcm()
{
    static rtcm_t rtcm;
    int i = 0, j, format;
    QString mstr1, mstr2;
    char tstr[64] = "-";

  rtksvrlock(&rtksvr);
    format = rtksvr.format[inputStream];
    rtcm = rtksvr.rtcm[inputStream];
  rtksvrunlock(&rtksvr);

    if (rtcm.time.time) time2str(rtcm.time, tstr, 3);

    for (j = 1; j < 100; j++) {
        if (rtcm.nmsg2[j] == 0) continue;
        mstr1 += QString("%1%2 (%3)").arg(mstr1.isEmpty() ? "" : ",").arg(j).arg(rtcm.nmsg2[j]);
  }
    if (rtcm.nmsg2[0] > 0) {
        mstr1 += QString("%1other (%2)").arg(mstr1.isEmpty() ? "" : ",").arg(rtcm.nmsg2[0]);
    }
    for (j = 1; j < 300; j++) {
        if (rtcm.nmsg3[j] == 0) continue;
        mstr2 += QString("%1%2(%3)").arg(mstr2.isEmpty() ? "" : ",").arg(j + 1000).arg(rtcm.nmsg3[j]);
  }
    for (j=300;j<399;j++) {
        if (rtcm.nmsg3[j]==0) continue;
        mstr2+=QString("%1%2(%3)").arg(mstr2.isEmpty()?"":",").arg(j+3770).arg(rtcm.nmsg3[j]);
    }
    if (rtcm.nmsg3[0] > 0)
        mstr2 += QString("%1other(%2)").arg(mstr2.isEmpty() ? "" : ",").arg(rtcm.nmsg3[0]);
    lblInformation->setText("");

    tWConsole->setRowCount(15);
    tWConsole->setHorizontalHeaderLabels(header);

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Format")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(format == STRFMT_RTCM2 ? tr("RTCM 2") : tr("RTCM 3")));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Message Time")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(tstr));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Station ID")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtcm.staid)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Station Health")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtcm.stah)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Sequence No")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(rtcm.seqno)));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("RTCM Special Message")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msg));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("Last Message")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msgtype));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("# of RTCM Messages")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(format == STRFMT_RTCM2 ? mstr1 : mstr2));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("MSM Signals for GPS")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msmtype[0]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("MSM Signals for GLONASS")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msmtype[1]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("MSM Signals for Galileo")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msmtype[2]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("MSM Signals for QZSS")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msmtype[3]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("MSM Signals for SBAS")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msmtype[4]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("MSM Signals for BDS")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msmtype[5]));

    tWConsole->setItem(i, 0, new QTableWidgetItem(tr("MSM Signals for NavIC")));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(rtcm.msmtype[6]));
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtcmDgps()
{
    header.clear();
    header << tr("SAT") << tr("Status") << tr("PRC (m)") << tr("RRC (m)") << tr("IOD") << tr("UDRE") << tr("T0");

    int i, width[] = {25, 30, 60, 60, 30, 30, 115};

    tWConsole->setColumnCount(7);
    tWConsole->setRowCount(0);
    for (i = 0; i < tWConsole->columnCount(); i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showRtcmDgps()
{
  gtime_t time;
  dgps_t dgps[MAXSAT];
    int i;
    char tstr[64], id[32];

  rtksvrlock(&rtksvr);
    time = rtksvr.rtk.sol.time;
    for (i = 0; i < MAXSAT; i++) dgps[i] = rtksvr.nav.dgps[i];
  rtksvrunlock(&rtksvr);

    lblInformation->setText("");
    tWConsole->setRowCount(MAXSAT);
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < tWConsole->rowCount(); i++) {
        int j = 0;
        satno2id(i + 1, id);
        bool valid = dgps[i].t0.time && fabs(timediff(time, dgps[i].t0)) <= 1800.0;
        tWConsole->setItem(i, j++, new QTableWidgetItem(id));
        tWConsole->setItem(i, j++, new QTableWidgetItem(valid ? tr("OK") : tr("-")));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(dgps[i].prc, 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(dgps[i].rrc, 'f', 4)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(dgps[i].iod)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(dgps[i].udre)));
        if (dgps[i].t0.time) time2str(dgps[i].t0, tstr, 0); else strcpy(tstr, "-");
        tWConsole->setItem(i, j++, new QTableWidgetItem(tstr));
  }
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtcmSsr()
{
    header.clear();
    header	<< tr("SAT") << tr("Status") << tr("UDI(s)") << tr("UDHR(s)") << tr("IOD") << tr("URA") << tr("Datum") << tr("T0")
            << tr("D0-A(m)") << tr("D0-C(m)") << tr("D0-R(m)") << tr("D1-A(mm/s)") << tr("D1-C(mm/s)") << tr("D1-R(mm/s)")
           << tr("C0(m)") << tr("C1(mm/s)") << tr("C2(mm/s2)") << tr("C-HR(m)") << tr("Code Bias(m)") << tr("Phase Bias(m)");
    int i, width[] = { 25, 30, 30, 30, 30, 25, 15, 115, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };

    tWConsole->setColumnCount(18 + 2 * MAXCODE);
    tWConsole->setRowCount(2);
    for (i = 0; i < 18; i++)
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showRtcmSsr()
{
  gtime_t time;
  ssr_t ssr[MAXSAT];
    int i, k, n, sat[MAXSAT], sys = sys_tbl[cBSelectSingleNavigationSystem->currentIndex() + 1];
    char tstr[64], id[32];
    QString s;

  rtksvrlock(&rtksvr);
    time = rtksvr.rtk.sol.time;
    for (i = n= 0; i < MAXSAT; i++) {
        if (!(satsys(i + 1, NULL) & sys)) continue;
        ssr[n] = rtksvr.rtcm[inputStream].ssr[i];
        sat[n++] = i + 1;
    }
  rtksvrunlock(&rtksvr);

    lblInformation->setText("");
    tWConsole->setRowCount(n);
    tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < n; i++) {
        int j = 0;
        satno2id(sat[i], id);
        tWConsole->setItem(i, j++, new QTableWidgetItem(id));
        bool valid = ssr[i].t0[0].time && fabs(timediff(time, ssr[i].t0[0])) <= 1800.0;
        tWConsole->setItem(i, j++, new QTableWidgetItem(valid ? tr("OK") : tr("-")));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].udi[0], 'f', 0)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].udi[2], 'f', 0)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].iode)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].ura)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].refd)));
        if (ssr[i].t0[0].time) time2str(ssr[i].t0[0], tstr, 0); else strcpy(tstr, "-");
        tWConsole->setItem(i, j++, new QTableWidgetItem(tstr));
        for (k = 0; k < 3; k++)
            tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].deph[k], 'f', 3)));
        for (k = 0; k < 3; k++)
            tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].ddeph[k] * 1E3, 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].dclk[0], 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].dclk[1] * 1E3, 'f', 3)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].dclk[2] * 1E3, 'f', 5)));
        tWConsole->setItem(i, j++, new QTableWidgetItem(QString::number(ssr[i].hrclk, 'f', 3)));
        s = "";
        for (k = 0; k < MAXCODE; k++)
            s += QString("%1: %2 ").arg(code2obs(k+1)).arg(ssr[i].cbias[k], 0, 'f', 3);
        tWConsole->setItem(i, j++, new QTableWidgetItem(s));
        s = "";
        for (k = 0; k < MAXCODE; k++)
            s += QString("%1: %2 ").arg(code2obs(k+1)).arg(ssr[i].pbias[k], 0, 'f', 3);
        tWConsole->setItem(i, j++, new QTableWidgetItem(s));
    }
}
//---------------------------------------------------------------------------
void MonitorDialog::setReferenceStation()
{
    int width[] = {220, 520};
    int i;

    tWConsole->setColumnCount(2);
    tWConsole->setRowCount(2);
    for (i = 0; i < 2; i++) {
        tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    }
    header << tr("Parameter") << tr("Value");
    tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showReferenceStation()
{
    QString s;
    gtime_t time;
    sta_t sta;
    double pos[3] = {0};
    int i = 0, format;
    char tstr[64] = "-";

    rtksvrlock(&rtksvr);
    format = rtksvr.format[inputStream];
    if (format == STRFMT_RTCM2 || format == STRFMT_RTCM3) {
        time = rtksvr.rtcm[inputStream].time;
        sta = rtksvr.rtcm[inputStream].sta;
    }
    else {
        time = rtksvr.raw[inputStream].time;
        sta = rtksvr.raw[inputStream].sta;
    }
    rtksvrunlock(&rtksvr);

    lblInformation->setText("");

    tWConsole->setHorizontalHeaderLabels(header);
    tWConsole->setRowCount(16);


    tWConsole->setItem(i, 0, new QTableWidgetItem("Format"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(formatstrs[format]));

    if (time.time) time2str(time, tstr, 3);
    tWConsole->setItem(i, 0, new QTableWidgetItem("Message Time"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(tstr));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Station Pos X/Y/Z (m)"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(sta.pos[0], 0, 'f', 3).arg(sta.pos[1], 0, 'f', 3).arg(sta.pos[2], 0, 'f', 3)));

    if (norm(sta.pos, 3) > 0.0) ecef2pos(sta.pos, pos);
    tWConsole->setItem(i, 0, new QTableWidgetItem("Station Lat/Lon/Height (deg,m)"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(pos[0]*R2D, 0, 'f', 8).arg(pos[1]*R2D, 0, 'f', 8).arg(pos[2], 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem("ITRF Realization Year"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(sta.itrf)));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Antenna Delta Type"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sta.deltype?"X/Y/Z":"E/N/U"));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Antenna Delta (m)"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3").arg(sta.del[0], 0, 'f', 3).arg(sta.del[1], 0, 'f', 3).arg(sta.del[2], 0, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Antenna Height (m)"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(sta.hgt, 'f', 3)));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Antenna Descriptor"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sta.antdes));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Antenna Setup Id"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString::number(sta.antsetup)));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Antenna Serial No"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sta.antsno));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Receiver Type Descriptor"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sta.rectype));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Receiver Firmware Version"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sta.recver));

    tWConsole->setItem(i, 0, new QTableWidgetItem("Receiver Serial No"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sta.recsno));

    tWConsole->setItem(i, 0, new QTableWidgetItem("GLONASS Code-Phase Alignment"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(sta.glo_cp_align ? "Aligned" : "Not aligned"));

    tWConsole->setItem(i, 0, new QTableWidgetItem("GLONASS Code-Phase Bias C1/P1/C2/P2 (m)"));
    tWConsole->setItem(i++, 1, new QTableWidgetItem(QString("%1, %2, %3, %4").arg(sta.glo_cp_bias[0], 0, 'f', 2).arg(
                                   sta.glo_cp_bias[1], 0, 'f', 2).arg(sta.glo_cp_bias[2], 0, 'f', 2).arg(sta.glo_cp_bias[3], 0, 'f', 2)));

}
//---------------------------------------------------------------------------
