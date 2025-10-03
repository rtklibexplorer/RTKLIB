//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QCloseEvent>
#include <QScrollBar>
#include <QDebug>
#include <QTableView>
#include <QComboBox>
#include <QPushButton>
#include <QDialogButtonBox>

#include "rtklib.h"
#include "mondlg.h"

#include "ui_mondlg.h"

//---------------------------------------------------------------------------

#define SQRT(x)     ((x)<0.0||(x)!=(x)?0.0:sqrt(x))
#define MAXLINE		128
#define MAXLEN		256
#define NMONITEM	17


static const int sys_tbl[] = {
    SYS_ALL, SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN, SYS_SBS
};

//---------------------------------------------------------------------------
MonitorDialog::MonitorDialog(QWidget *parent, rtksvr_t *server, stream_t* stream)
    : QDialog(parent), rtksvr(server), monistr(stream), ui(new Ui::MonitorDialog)
{
    ui->setupUi(this);

    fontScale = QFontMetrics(ui->tWConsole->font()).height() * 4;

    consoleFormat = 1;
    ui->cBSelectFormat->setCurrentIndex(consoleFormat);
    inputStream = solutionStream = 0;

    for (int i = 0; i <= MAXRCVFMT; i++) ui->cBSelectFormat->addItem(formatstrs[i]);

	init_rtcm(&rtcm);
    init_raw(&raw, -1);

    connect(ui->btnClear, &QPushButton::clicked, this, &MonitorDialog::clearOutput);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &MonitorDialog::reject);
    connect(ui->btnDown, &QPushButton::clicked, this, &MonitorDialog::scrollDown);
    connect(ui->cBType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MonitorDialog::displayTypeChanged);
    connect(ui->cBSelectSingleNavigationSystem, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MonitorDialog::displayTypeChanged);
    connect(ui->cBSelectFormat, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MonitorDialog::consoleFormatChanged);
    connect(ui->cBSelectObservation, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MonitorDialog::observationModeChanged);
    connect(ui->cBSelectInputStream, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MonitorDialog::inputStreamChanged);
    connect(ui->cBSelectSolutionStream, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MonitorDialog::solutionStreamChanged);
    connect(ui->cBSelectEphemeris, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MonitorDialog::ephemerisSetChanged);
    connect(&updateTimer, &QTimer::timeout, this, &MonitorDialog::updateDisplays);
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

    ui->lblInformation->setText("");

    clearTable();

    updateTimer.start(100);
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
    int index = getDisplayType() - NMONITEM;

    if (0 <= index) {
        rtksvrlock(rtksvr);
        if (index < 2) rtksvr->npb[index] = 0;
        else if (index < 4) rtksvr->nsb[index - 2] = 0;
        else rtksvr->rtk.neb = 0;
        rtksvrunlock(rtksvr);
	}
    ui->lblInformation->setText("");
    consoleBuffer.clear();
    ui->tWConsole->clear();
    clearTable();
}
//---------------------------------------------------------------------------
int MonitorDialog::getDisplayType()
{
    return ui->cBType->currentIndex();
}
//---------------------------------------------------------------------------
void MonitorDialog::consoleFormatChanged()
{
    if (consoleFormat >= 3 && consoleFormat < 17)
        free_raw(&raw);

    // update format marker
    consoleFormat = ui->cBSelectFormat->currentIndex();

    if (consoleFormat >= 3 && consoleFormat < 17)
        init_raw(&raw, consoleFormat - 2);
}
//---------------------------------------------------------------------------
void MonitorDialog::inputStreamChanged()
{
    inputStream = ui->cBSelectInputStream->currentIndex();
    displayTypeChanged();
}
//---------------------------------------------------------------------------
void MonitorDialog::solutionStreamChanged()
{
    solutionStream = ui->cBSelectSolutionStream->currentIndex();
    consoleBuffer.clear();
    ui->tWConsole->clear();
}
//---------------------------------------------------------------------------
void MonitorDialog::ephemerisSetChanged()
{
    int sys = sys_tbl[ui->cBSelectSingleNavigationSystem->currentIndex() + 1];
    if (sys == SYS_GAL) {
        ui->lblInformation->setText((ui->cBSelectEphemeris->currentIndex() % 2) ? tr("F/NAV") : tr("I/NAV"));
    } else {
        ui->lblInformation->setText("");
    }
}
//---------------------------------------------------------------------------
void MonitorDialog::updateDisplays()
{
    if (!isVisible()) return;
    int sys = sys_tbl[ui->cBSelectSingleNavigationSystem->currentIndex() + 1];

    switch (getDisplayType()) {
        case  0: showRtk(); break;
        case  1: showObservations(); break;
        case  2:
            if (sys == SYS_GLO) {
                showGlonassNavigations();
            } else if (sys == SYS_SBS) {
                showSbsNavigations();
            } else {
                showNavigationsGPS();
            }
            break;
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
    int sys = sys_tbl[ui->cBSelectSingleNavigationSystem->currentIndex() + 1];
    ui->tWConsole->horizontalHeader()->setVisible(true);

    switch (getDisplayType()) {
        case  0: setRtk(); break;
        case  1: setObservations(); break;
        case  2:
            if (sys == SYS_GLO) {
                setGlonassNavigations();
            } else if (sys == SYS_SBS) {
                setSbsNavigations();
            } else {
                setNavigationGPS();
            }
            break;
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
        default:
            console = 1;
            ui->tWConsole->setColumnWidth(0, ui->tWConsole->width());
            ui->tWConsole->setRowCount(0);
            ui->tWConsole->horizontalHeader()->setVisible(false);

            break;
	}
    ui->tWConsole->setVisible(true);
    ui->panelControl->setVisible(console);
    ui->btnPause->setVisible(console != 0);
    ui->btnDown->setVisible(console != 0);
    ui->btnClear->setVisible(console != 0);

    int displayType = getDisplayType();
    ui->cBSelectObservation->setVisible(displayType == 1);
    ui->cBSelectNavigationSystems->setVisible(displayType == 1 || displayType == 5);
    ui->cBSelectSingleNavigationSystem->setVisible(displayType == 2 || displayType == 14);
    ui->cBSelectSatellites->setVisible(displayType == 2 || displayType == 5);
    ui->cBSelectInputStream->setVisible(displayType == 12 || displayType == 14 || displayType == 15 || displayType == 16);
    ui->cBSelectSolutionStream->setVisible(displayType == 17);
    ui->cBSelectFormat->setVisible(displayType == 16);
    ui->cBSelectEphemeris->setVisible(displayType == 2);

    updateDisplays();
}
//---------------------------------------------------------------------------
void MonitorDialog::showBuffers()
{
    unsigned char *msg = 0;
    int i, len;

    int displayType = getDisplayType();

    if (displayType < 16) return;

    rtksvrlock(rtksvr);

    if (displayType == 16) { // input buffer
        if (inputStream >= 3) {
            // Combined
            len = 0;
            for (int i = 0; i < 3; i++)
                len += rtksvr->npb[i];
            if (len > 0 && (msg = (uint8_t *)malloc(size_t(len)))) {
                for (int i = 0, j = 0; i < 3; i++) {
                    int ilen = rtksvr->npb[i];
                    if (ilen > 0) {
                        memcpy(msg + j, rtksvr->pbuf[i], size_t(ilen));
                        rtksvr->npb[i] = 0;
                        j += ilen;
                    }
                }
            }
        } else {
            len = rtksvr->npb[inputStream];
            if (len > 0 && (msg = (uint8_t *)malloc(size_t(len)))) {
                memcpy(msg, rtksvr->pbuf[inputStream], size_t(len));
                rtksvr->npb[inputStream] = 0;
            }
        }
    } else if (displayType == 17) { // solution buffer
        len = rtksvr->nsb[solutionStream];
        if (len > 0 && (msg = (uint8_t*)malloc(size_t(len)))) {
            memcpy(msg, rtksvr->sbuf[solutionStream], size_t(len));
            rtksvr->nsb[solutionStream] = 0;
		}
    } else { // error message buffer
        len = rtksvr->rtk.neb;
        if (len > 0 && (msg = (uint8_t *)malloc(size_t(len)))) {
            memcpy(msg, rtksvr->rtk.errbuf, size_t(len));
            rtksvr->rtk.neb = 0;
		}
	}
    rtksvrunlock(rtksvr);

    if (len <= 0 || !msg) return;

    rtcm.outtype = raw.outtype = 1;

    if (displayType >= 17) {
        addConsole(msg, len, 1, false);
    }
    else if (consoleFormat < 2) {  // ASCII or HEX
        addConsole(msg, len, consoleFormat, false);
    } else if (consoleFormat - 2 == STRFMT_RTCM2) {
        for (i = 0; i < len; i++) {
            input_rtcm2(&rtcm, msg[i]);
			if (rtcm.msgtype[0]) {
                addConsole((uint8_t*)rtcm.msgtype, strlen(rtcm.msgtype), 1, true);
                rtcm.msgtype[0] = '\0';
			}
        }
    } else if (consoleFormat - 2 == STRFMT_RTCM3) {
        for (i = 0; i < len; i++) {
            input_rtcm3(&rtcm, msg[i]);
			if (rtcm.msgtype[0]) {
                addConsole((uint8_t*)rtcm.msgtype, strlen(rtcm.msgtype), 1, true);
                rtcm.msgtype[0] = '\0';
			}
        }
    } else if (consoleFormat < 17) {
        for (i = 0; i < len; i++) {
            input_raw(&raw, consoleFormat - 2, msg[i]);
			if (raw.msgtype[0]) {
                addConsole((uint8_t*)raw.msgtype, strlen(raw.msgtype), 1, true);
                raw.msgtype[0] = '\0';
			}
        }
	}
	free(msg);
}
//---------------------------------------------------------------------------
void MonitorDialog::addConsole(const uint8_t *msg, int n, int mode, bool newline)
{
    char buff[MAXLEN + 16], *p = buff;

    if (ui->btnPause->isChecked()) return;

    if (n <= 0) return;

    if (consoleBuffer.count() == 0) consoleBuffer.append(""); // make sure that there is always at least one line in consoleBuffer

    // fill buffer with last incomplete line
    p += sprintf(p, "%s", qPrintable(consoleBuffer.at(consoleBuffer.count() - 1)));

    for (int i = 0; i < n; i++) {
        int add = 0;
        if (mode) {
            if (msg[i] == '\r') continue;
            if (msg[i] == '\n') add = 1;
            else {
              p += sprintf(p, "%c", isprint(msg[i]) ? msg[i] : '.');
              if (p - buff >= MAXLEN) add = 1;
            }
        } else {
            // Add a space after 16 and a line break after 67 characters.
            if (strlen(buff) % 17 == 16) strcat(p++, " ");
            p += sprintf(p, "%02X", msg[i]);
            if (p - buff >= 67) add = 1;
        }

        if (add) {
            consoleBuffer[consoleBuffer.count() - 1] = QString(buff);
            consoleBuffer.append("");
            *(p = buff) = 0;
            while (consoleBuffer.count() >= MAXLINE) consoleBuffer.removeFirst();
        }
    }
    // store last (incomplete) line
    consoleBuffer[consoleBuffer.count() - 1] = QString(buff);

    // write consoleBuffer to table widget
    ui->tWConsole->setColumnCount(1);
    ui->tWConsole->setRowCount(consoleBuffer.size());
    for (int i = 0; i < consoleBuffer.size(); i++)
        ui->tWConsole->setItem(i, 0, new QTableWidgetItem(consoleBuffer.at(i)));

    if (ui->btnDown->isChecked())
        ui->tWConsole->verticalScrollBar()->setValue(ui->tWConsole->verticalScrollBar()->maximum());

    if (newline)
        consoleBuffer.append("");
}
//---------------------------------------------------------------------------
void MonitorDialog::clearOutput()
{
    consoleBuffer.clear();
    ui->tWConsole->clear();
    ui->tWConsole->setRowCount(0);
}
//---------------------------------------------------------------------------
void MonitorDialog::scrollDown()
{
    ui->tWConsole->verticalScrollBar()->setValue(ui->tWConsole->verticalScrollBar()->maximum());
}
//---------------------------------------------------------------------------
void MonitorDialog::observationModeChanged()
{
    setObservations();
    showObservations();
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtk()
{
    header << tr("Parameter") << tr("Value");
    int width[] = {500, 500};

    ui->tWConsole->setColumnCount(2);
    ui->tWConsole->setRowCount(52 + NFREQ*2);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    for (int i = 0; (i < ui->tWConsole->columnCount()) && (i < 2); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    for (int col = 0; col < ui->tWConsole->columnCount(); col++)
        for (int row = 0; row < ui->tWConsole->rowCount(); row++)
            ui->tWConsole->setItem(row, col, new QTableWidgetItem());

    ui->lblInformation->setText("");
}
//--------------------------------------------------------)-------------------
void MonitorDialog::showRtk()
{
    QString exsats, navsys = "";
    const QString svrstate[] = {tr("Stop"), tr("Run")};
    const QString sol[] = {tr("-"), tr("Fix"), tr("Float"), tr("SBAS"), tr("DGPS"), tr("Single"), tr("PPP"), tr("Dead Reckoning")};
    const QString mode[] = {tr("Single"), tr("DGPS"), tr("Kinematic"), tr("Static"), tr("Static-Start"), tr("Moving-Base"),
                            tr("Fixed"), tr("PPP-Kinematic"), tr("PPP-Static"), tr("PPP-Fixed")};
    const QString freq[] = {tr("-"), tr("L1"), tr("L1+L2"), tr("L1+L2+L5"), tr("L1+L2+L5+L6"), tr("L1+L2+L5+L6+L7")};
    double *del, *off, rt[3] = {0}, dop[4] = {0};
    double azel[MAXSAT * 2], pos[3], vel[3], rr[3] = {0}, enu[3] = {0};
    int row, j, k, cycle, state, rtkstat, nsat0, nsat1, prcout, nave;
    int cputime, nb[3] = {0}, ne;
    unsigned int nmsg[3][10] = {{0}};
    char tstr[40], id[8], s1[40] = "-", s2[40] = "-", s3[40] = "-";
    char file[1024] = "";
    const QString ionoopt[] = {tr("OFF"), tr("Broadcast"), tr("SBAS"), tr("Dual-Frequency"), tr("Estimate STEC"), tr("IONEX TEC"), tr("QZSS LEX")};
    const QString tropopt[] = {tr("OFF"), tr("Saastamoinen"), tr("SBAS"), tr("Estimate ZTD"), tr("Estimate ZTD+Grad")};
    const QString ephopt [] = {tr("Broadcast"), tr("Precise"), tr("Broadcast+SBAS"), tr("Broadcast+SSR APC"), tr("Broadcast+SSR CoM"), tr("QZSS LEX")};

    rtk_t *rtk = static_cast<rtk_t *>(malloc(sizeof(rtk_t)));
    if (rtk == NULL) return;

    rtksvrlock(rtksvr); // lock

    *rtk = rtksvr->rtk;
    cycle = rtksvr->cycle;
    state = rtksvr->state < 0 || rtksvr->state > 1 ? 0 : rtksvr->state;
    rtkstat = rtksvr->rtk.sol.stat > MAXSOLQ ? 0 : rtksvr->rtk.sol.stat;
    nsat0 = rtksvr->obs[0][0].n;
    nsat1 = rtksvr->obs[1][0].n;
    cputime = rtksvr->cputime;
    prcout = rtksvr->prcout;
    nave = rtksvr->nave;

    for (row = 0; row < 3; row++) nb[row] = rtksvr->nb[row];

    for (row = 0; row < 3; row++)
    {
        for (j = 0; j < 10; j++)
            nmsg[row][j] = rtksvr->nmsg[row][j];
    }

    if (rtksvr->state) {
        double runtime;
        runtime = static_cast<double>(tickget() - rtksvr->tick) / 1000.0;
        rt[0] = floor(runtime / 3600.0); runtime -= rt[0] * 3600.0;
        rt[1] = floor(runtime / 60.0); rt[2] = runtime - rt[1] * 60.0;
	}
    if ((ne = rtksvr->nav.ne) > 0) {
        time2str(rtksvr->nav.peph[0].time, s1, 0);
        time2str(rtksvr->nav.peph[ne - 1].time, s2, 0);
        time2str(rtksvr->ftime[2], s3, 0);
	}
    strncpy(file, rtksvr->files[2], 1023);

    rtksvrunlock(rtksvr); // unlock

    for (j = k = 0; j < MAXSAT; j++) {
        if (rtk->opt.mode == PMODE_SINGLE && !rtk->ssat[j].vs) continue;
        if (rtk->opt.mode != PMODE_SINGLE && !rtk->ssat[j].vsat[0]) continue;
        azel[k * 2] = rtk->ssat[j].azel[0];
        azel[k * 2 + 1] = rtk->ssat[j].azel[1];
		k++;
	}
    dops(k, azel, 0.0, dop);

    if (rtk->opt.navsys & SYS_GPS) navsys = navsys + tr("GPS ");
    if (rtk->opt.navsys & SYS_GLO) navsys = navsys + tr("GLONASS ");
    if (rtk->opt.navsys & SYS_GAL) navsys = navsys + tr("Galileo ");
    if (rtk->opt.navsys & SYS_QZS) navsys = navsys + tr("QZSS ");
    if (rtk->opt.navsys & SYS_CMP) navsys = navsys + tr("BDS ");
    if (rtk->opt.navsys & SYS_IRN) navsys = navsys + tr("NavIC ");
    if (rtk->opt.navsys & SYS_SBS) navsys = navsys + tr("SBAS ");

    if (ui->tWConsole->rowCount() < 55) {
      free(rtk);
      return;
    }

    row = 0;

    ui->tWConsole->item(row,   0)->setText(tr("RTKLIB Version"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 %2").arg(VER_RTKLIB, PATCH_LEVEL));

    ui->tWConsole->item(row,   0)->setText(tr("RTK Server State"));
    ui->tWConsole->item(row++, 1)->setText(svrstate[state]);

    ui->tWConsole->item(row,   0)->setText(tr("Processing Cycle (ms)"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(cycle));

    ui->tWConsole->item(row,   0)->setText(tr("Positioning Mode"));
    ui->tWConsole->item(row++, 1)->setText(mode[rtk->opt.mode < 0 || rtk->opt.mode > 9 ? 0 : rtk->opt.mode ]);

    ui->tWConsole->item(row,   0)->setText(tr("Frequencies"));
    ui->tWConsole->item(row++, 1)->setText(freq[rtk->opt.nf > 5 ? 0 : rtk->opt.nf]);

    ui->tWConsole->item(row,   0)->setText(tr("Elevation Mask"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(rtk->opt.elmin * R2D, 'f', 0) + " deg");

    ui->tWConsole->item(row,   0)->setText(tr("SNR Mask L1 (dBHz)"));
    ui->tWConsole->item(row++, 1)->setText(!rtk->opt.snrmask.ena[0] ? "" :
                              QStringLiteral("%1, %2, %3, %4, %5, %6, %7, %8, %9")
                              .arg(rtk->opt.snrmask.mask[0][0], 0).arg(rtk->opt.snrmask.mask[0][1], 0).arg(rtk->opt.snrmask.mask[0][2], 0)
                              .arg(rtk->opt.snrmask.mask[0][3], 0).arg(rtk->opt.snrmask.mask[0][4], 0).arg(rtk->opt.snrmask.mask[0][5], 0)
                              .arg(rtk->opt.snrmask.mask[0][6], 0).arg(rtk->opt.snrmask.mask[0][7], 0).arg(rtk->opt.snrmask.mask[0][8], 0));

    ui->tWConsole->item(row,   0)->setText(tr("SNR Mask L2 (dBHz)"));
    ui->tWConsole->item(row++, 1)->setText(!rtk->opt.snrmask.ena[0] ? "" :
                              QStringLiteral("%1, %2, %3, %4, %5, %6, %7, %8, %9")
                              .arg(rtk->opt.snrmask.mask[1][0], 0).arg(rtk->opt.snrmask.mask[1][1], 0).arg(rtk->opt.snrmask.mask[1][2], 0)
                              .arg(rtk->opt.snrmask.mask[1][3], 0).arg(rtk->opt.snrmask.mask[1][4], 0).arg(rtk->opt.snrmask.mask[1][5], 0)
                              .arg(rtk->opt.snrmask.mask[1][6], 0).arg(rtk->opt.snrmask.mask[1][7], 0).arg(rtk->opt.snrmask.mask[1][8], 0));

    ui->tWConsole->item(row,   0)->setText(tr("SNR Mask L5 (dBHz)"));
    ui->tWConsole->item(row++, 1)->setText(!rtk->opt.snrmask.ena[0] ? "" :
                              QStringLiteral("%1, %2, %3, %4, %5, %6, %7, %8, %9")
                              .arg(rtk->opt.snrmask.mask[2][0], 0).arg(rtk->opt.snrmask.mask[2][1], 0).arg(rtk->opt.snrmask.mask[2][2], 0)
                              .arg(rtk->opt.snrmask.mask[2][3], 0).arg(rtk->opt.snrmask.mask[2][4], 0).arg(rtk->opt.snrmask.mask[2][5], 0)
                              .arg(rtk->opt.snrmask.mask[2][6], 0).arg(rtk->opt.snrmask.mask[2][7], 0).arg(rtk->opt.snrmask.mask[2][8], 0));

    ui->tWConsole->item(row,   0)->setText(tr("Rec Dynamic/Earth Tides Correction"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1, %2").arg(rtk->opt.dynamics ? tr("ON") : tr("OFF"), rtk->opt.tidecorr ? tr("ON") : tr("OFF")));

    ui->tWConsole->item(row,   0)->setText(tr("Ionosphere/Troposphere Model"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1, %2").arg(ionoopt[rtk->opt.ionoopt < 0 || rtk->opt.ionoopt > 6 ? 0 : rtk->opt.ionoopt],
                                                                        tropopt[rtk->opt.tropopt < 0 || rtk->opt.tropopt > 4 ? 0 : rtk->opt.tropopt]));

    ui->tWConsole->item(row,   0)->setText(tr("Satellite Ephemeris"));
    ui->tWConsole->item(row++, 1)->setText(ephopt[rtk->opt.sateph < 0 || rtk->opt.sateph > 5 ? 0 : rtk->opt.sateph]);

    for (j = 1; j <= MAXSAT; j++) {
        if (!rtk->opt.exsats[j - 1]) continue;
        satno2id(j, id);
        if (rtk->opt.exsats[j - 1] == 2) exsats = exsats + "+";
        exsats = exsats + id + " ";
	}
    ui->tWConsole->item(row,   0)->setText(tr("Excluded Satellites"));
    ui->tWConsole->item(row++, 1)->setText(exsats);

    ui->tWConsole->item(row,   0)->setText(tr("Navigation Systems"));
    ui->tWConsole->item(row++, 1)->setText(navsys);

    ui->tWConsole->item(row,   0)->setText(tr("Accumulated Time to Run"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1:%2:%3").arg(rt[0], 2, 'f', 0, '0').arg(rt[1], 2, 'f', 0, '0').arg(rt[2], 4, 'f', 1, '0'));

    ui->tWConsole->item(row,   0)->setText(tr("CPU Time for a Processing Cycle"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(cputime) + " ms");

    ui->tWConsole->item(row,   0)->setText(tr("Missing Observation Data Count"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(prcout));

    ui->tWConsole->item(row,   0)->setText(tr("Bytes in Input Buffer"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1, %2, %3").arg(nb[0]).arg(nb[1]).arg(nb[2]));

    ui->tWConsole->item(row,   0)->setText(tr("# of Input Data Rover"));
    ui->tWConsole->item(row++, 1)->setText(tr("Obs(%1), Nav(%2), Ion(%3), Sbs (%4), Pos(%5), Dgps(%6), Ssr(%7), Err(%8)")
                              .arg(nmsg[0][0]).arg(nmsg[0][1] + nmsg[0][6]).arg(nmsg[0][2]).arg(nmsg[0][3])
                              .arg(nmsg[0][4]).arg(nmsg[0][5]).arg(nmsg[0][7]).arg(nmsg[0][9]));

    ui->tWConsole->item(row,   0)->setText(tr("# of Input Data Base/NRTK Station"));
    ui->tWConsole->item(row++, 1)->setText(tr("Obs(%1), Nav(%2), Ion(%3), Sbs (%4), Pos(%5), Dgps(%6), Ssr(%7), Err(%8)")
                              .arg(nmsg[1][0]).arg(nmsg[1][1] + nmsg[1][6]).arg(nmsg[1][2]).arg(nmsg[1][3])
                              .arg(nmsg[1][4]).arg(nmsg[1][5]).arg(nmsg[1][7]).arg(nmsg[1][9]));

    ui->tWConsole->item(row,   0)->setText(tr("# of Input Data Ephemeris"));
    ui->tWConsole->item(row++, 1)->setText(tr("Obs(%1), Nav(%2), Ion(%3), Sbs (%4), Pos(%5), Dgps(%6), Ssr(%7), Err(%8)")
                              .arg(nmsg[2][0]).arg(nmsg[2][1] + nmsg[2][6]).arg(nmsg[2][2]).arg(nmsg[2][3])
                              .arg(nmsg[2][4]).arg(nmsg[2][5]).arg(nmsg[2][7]).arg(nmsg[2][9]));

    ui->tWConsole->item(row,   0)->setText(tr("Solution Status"));
    ui->tWConsole->item(row++, 1)->setText(sol[rtkstat]);

    time2str(rtk->sol.time, tstr, 9);
    ui->tWConsole->item(row,   0)->setText(tr("Time of Receiver Clock Rover"));
    ui->tWConsole->item(row++, 1)->setText(rtk->sol.time.time ? tstr : "-");

    ui->tWConsole->item(row,   0)->setText(tr("Time System Offset/Receiver Bias\n (GLO-GPS, GAL-GPS, BDS-GPS, IRN-GPS, QZS-GPS) (ns)"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1, %2, %3, %4, %5").arg(rtk->sol.dtr[1] * 1E9, 0, 'f', 3).arg(rtk->sol.dtr[2] * 1E9, 0, 'f', 3)
                                            .arg(rtk->sol.dtr[3] * 1E9, 0, 'f', 3).arg(rtk->sol.dtr[4] * 1E9, 0, 'f', 3).arg(rtk->sol.dtr[5] * 1E9, 0, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("Solution Interval"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(rtk->tt, 'f', 3) + " s");

    ui->tWConsole->item(row,   0)->setText(tr("Age of Differential"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(rtk->sol.age, 'f', 3) + " s");

    ui->tWConsole->item(row,   0)->setText(tr("Ratio for AR Validation"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(rtk->sol.ratio, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("# of Satellites Rover"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(nsat0));

    ui->tWConsole->item(row,   0)->setText(tr("# of Satellites Base/NRTK Station"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(nsat1));

    ui->tWConsole->item(row,   0)->setText(tr("# of Valid Satellites"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(rtk->sol.ns));

    ui->tWConsole->item(row,   0)->setText(tr("GDOP/PDOP/HDOP/VDOP"));
    ui->tWConsole->item(row++, 1)->setText(QString("%1, %2, %3, %4").arg(dop[0], 0, 'f', 1).arg(dop[1], 0, 'f', 1)
                                            .arg(dop[2], 0, 'f', 1).arg(dop[3], 0, 'f', 1));

    ui->tWConsole->item(row,   0)->setText(tr("# of Real Estimated States"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(rtk->na));

    ui->tWConsole->item(row,   0)->setText(tr("# of All Estimated States"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(rtk->nx));

    ui->tWConsole->item(row,   0)->setText(tr("Pos X/Y/Z Single (m) Rover"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1, %2, %3").arg(rtk->sol.rr[0], 0, 'f', 3).arg(rtk->sol.rr[1], 0, 'f', 3).arg(rtk->sol.rr[2], 0, 'f', 3));

    if (norm(rtk->sol.rr, 3) > 0.0)
        ecef2pos(rtk->sol.rr, pos);
    else
        pos[0] = pos[1] = pos[2] = 0.0;
    ui->tWConsole->item(row,   0)->setText(tr("Lat/Lon/Height Single Rover"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 deg, %2 deg, %3 m").arg(pos[0] * R2D, 0, 'f', 8).arg(pos[1] * R2D, 0, 'f', 8).arg(pos[2], 0, 'f', 3));

    ecef2enu(pos, rtk->sol.rr + 3, vel);
    ui->tWConsole->item(row,   0)->setText(tr("Vel E/N/U (m/s) Rover"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m/s, %2 m/s, %3 m/s").arg(vel[0], 0, 'f', 3).arg(vel[1], 0, 'f', 3).arg(vel[2], 0, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("Pos X/Y/Z Float Rover"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m")
                              .arg(rtk->x ? rtk->x[0] : 0, 0, 'f', 3).arg(rtk->x ? rtk->x[1] : 0, 0, 'f', 3).arg(rtk->x ? rtk->x[2] : 0, 0, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("Pos X/Y/Z Float Std Rover"));
    ui->tWConsole->item(row++, 1)->setText(QString("%1 m, %2 m, %3 m")
                              .arg(rtk->P ? SQRT(rtk->P[0]) : 0, 0, 'f', 3).arg(rtk->P ? SQRT(rtk->P[1 + 1 * rtk->nx]) : 0, 0, 'f', 3).arg(rtk->P ? SQRT(rtk->P[2 + 2 * rtk->nx]) : 0, 0, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("Pos X/Y/Z Fixed Rover"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m")
                              .arg(rtk->xa ? rtk->xa[0] : 0, 0, 'f', 3).arg(rtk->xa ? rtk->xa[1] : 0, 0, 'f', 3).arg(rtk->xa ? rtk->xa[2] : 0, 0, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("Pos X/Y/Z Fixed Std Rover"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m")
                              .arg(rtk->Pa ? SQRT(rtk->Pa[0]) : 0, 0, 'f', 3).arg(rtk->Pa ? SQRT(rtk->Pa[1 + 1 * rtk->na]) : 0, 0, 'f', 3).arg(rtk->Pa ? SQRT(rtk->Pa[2 + 2 * rtk->na]) : 0, 0, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("Pos X/Y/Z Base Station"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m").arg(rtk->rb[0], 0, 'f', 3).arg(rtk->rb[1], 0, 'f', 3).arg(rtk->rb[2], 0, 'f', 3));

    if (norm(rtk->rb, 3) > 0.0)
        ecef2pos(rtk->rb, pos);
    else
        pos[0] = pos[1] = pos[2] = 0.0;
    ui->tWConsole->item(row,   0)->setText(tr("Lat/Lon/Height Base Station"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 deg, %2 deg, %3 m").arg(pos[0] * R2D, 0, 'f', 8).arg(pos[1] * R2D, 0, 'f', 8).arg(pos[2], 0, 'f', 3));

    ecef2enu(pos, rtk->rb + 3, vel);
    ui->tWConsole->item(row,   0)->setText(tr("Vel E/N/U Base Station"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m/s, %2 m/s, %3 m/s").arg(vel[0], 0, 'f', 3).arg(vel[1], 0, 'f', 3).arg(vel[2], 0, 'f', 3));

    if (norm(rtk->rb,3)>0.0) {
        for (k = 0; k < 3; k++)
            rr[k] = rtk->sol.rr[k] - rtk->rb[k];
        ecef2enu(pos,rr,enu);
    }
    ui->tWConsole->item(row,   0)->setText(tr("Baseline Length/E/N/U Rover-Base Station"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m, %4 m").arg(norm(rr, 3), 0, 'f', 3).arg(enu[0], 0, 'f', 3).arg(enu[1], 0, 'f', 3).arg(enu[2], 0, 'f', 3));

    ui->tWConsole->item(row,   0)->setText(tr("# of Averaging Single Pos Base Station"));
    ui->tWConsole->item(row++, 1)->setText(QString::number(nave));

    ui->tWConsole->item(row,   0)->setText(tr("Antenna Type Rover"));
    ui->tWConsole->item(row++, 1)->setText(rtk->opt.pcvr[0].type);

    for (j = 0; j < NFREQ; j++) {
        off = rtk->opt.pcvr[0].off[j];
        ui->tWConsole->item(row,   0)->setText(tr("Antenna Phase Center L%1 E/N/U Rover").arg(j+1));
        ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m").arg(off[0], 0, 'f', 3).arg(off[1], 0, 'f', 3).arg(off[2], 0, 'f', 3));
    }

    del = rtk->opt.antdel[0];
    ui->tWConsole->item(row,   0)->setText(tr("Antenna Delta E/N/U Rover"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m").arg(del[0], 0, 'f', 4).arg(del[1], 0, 'f', 4).arg(del[2], 0, 'f', 4));

    ui->tWConsole->item(row,   0)->setText(tr("Antenna Type Base Station"));
    ui->tWConsole->item(row++, 1)->setText(rtk->opt.pcvr[1].type);

    for (j = 0; j < NFREQ; j++) {
        off = rtk->opt.pcvr[1].off[0];
        ui->tWConsole->item(row,   0)->setText(tr("Antenna Phase Center L%1 E/N/U Base Station").arg(j+1));
        ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m").arg(off[0], 0, 'f', 3).arg(off[1], 0, 'f', 3).arg(off[2], 0, 'f', 3));
    }

    del = rtk->opt.antdel[1];
    ui->tWConsole->item(row,   0)->setText(tr("Antenna Delta E/N/U Base Station"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1 m, %2 m, %3 m").arg(del[0], 0, 'f', 4).arg(del[1], 0, 'f', 4).arg(del[2], 0, 'f', 4));

    ui->tWConsole->item(row,   0)->setText(tr("Precise Ephemeris Time/# of Epoch"));
    ui->tWConsole->item(row++, 1)->setText(QStringLiteral("%1-%2 (%3)").arg(s1, s2).arg(ne));

    ui->tWConsole->item(row,   0)->setText(tr("Precise Ephemeris Download Time"));
    ui->tWConsole->item(row++, 1)->setText(s3);

    ui->tWConsole->item(row,   0)->setText(tr("Precise Ephemeris Download File"));
    ui->tWConsole->item(row++, 1)->setText(file);
    free(rtk);
}
//---------------------------------------------------------------------------
void MonitorDialog::setSat()
{
    int i, j = 0;
    const QString label[] = {
        tr("SAT"), tr("Status"), tr("Azimuth (deg)"), tr("Elevation (deg)"), tr("LG (m)"), tr("PHW (cyc)"),
        tr("P1-P2 (m)"), tr("P1-C1 (m)"), tr("P2-C2(m)")
	};
    int width[] = {46, 60, 125, 130, 90, 90, 90, 90, 90}, nfreq;

    rtksvrlock(rtksvr);
    nfreq = rtksvr->rtk.opt.nf > NFREQ ? NFREQ : rtksvr->rtk.opt.nf;
    rtksvrunlock(rtksvr);

    ui->tWConsole->setColumnCount(9 + nfreq * 9);
    ui->tWConsole->setRowCount(0);
    header.clear();

    j = 0;
    for (i = 0; i < 4; i++) {
        ui->tWConsole->setColumnWidth(j++, width[i] * fontScale / 96);
        header << label[i];
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 38 * fontScale / 96);
        header << tr("L%1").arg(i+1);
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 70 * fontScale / 96);
        header << tr("Fix%1").arg(i+1);
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 140 * fontScale / 96);
        header << tr("P%1 Residual (m)").arg(i+1);
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 135 * fontScale / 96);
        header << tr("L%1 Residual (m)").arg(i+1);
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 50 * fontScale / 96);
        header << tr("Slip%1").arg(i+1);
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 55 * fontScale / 96);
        header << tr("Lock%1").arg(i+1);
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 80 * fontScale / 96);
        header << tr("Outage%1").arg(i+1);
	}
    for (i = 0; i < nfreq; i++) {
        ui->tWConsole->setColumnWidth(j++, 70 * fontScale / 96);
        header << tr("Reject%1").arg(i+1);
	}
    for (i = 4; i < 9; i++) {
        ui->tWConsole->setColumnWidth(j++, width[i] * fontScale / 96);
        header << label[i];
	}
    ui->tWConsole->setHorizontalHeaderLabels(header);

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showSat()
{
	ssat_t *ssat;
    int i, j, k, n, nsat, fix, nfreq, sys = sys_tbl[ui->cBSelectNavigationSystems->currentIndex()];
    int vsat[MAXSAT] = {0};
	char id[8];
    double az, el, cbias[MAXSAT][2];

    rtk_t *rtk = static_cast<rtk_t *>(malloc(sizeof(rtk_t)));
    if (rtk == NULL) return;

    rtksvrlock(rtksvr);
    *rtk = rtksvr->rtk;

    for (i = 0; i < MAXSAT; i++)
    {
        for (j = 0; j < 2; j++)
            cbias[i][j] = rtksvr->nav.cbias[i][j][0];
    }
    nfreq = rtksvr->rtk.opt.nf > NFREQ ? NFREQ : rtksvr->rtk.opt.nf;
    rtksvrunlock(rtksvr);

    for (i = 0; i < MAXSAT; i++) {
        ssat = rtk->ssat + i;
        vsat[i] = ssat->vs;
    }

    for (i = 0, nsat = 0; i < MAXSAT; i++) {
        if (!(satsys(i + 1, NULL) & sys)) continue;
        if (ui->cBSelectSatellites->currentIndex() == 1 && !vsat[i]) continue;
        nsat++;
	}

    if (nsat < 1) {
        ui->tWConsole->setRowCount(0);
        free(rtk);
		return;
	}
    if (ui->tWConsole->rowCount() != nsat) {
        ui->tWConsole->setRowCount(nsat);
        for (int row = 0; row < nsat; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0, n = 0; i < MAXSAT; i++) {
        if (!(satsys(i + 1, NULL) & sys)) continue;
        j = 0;
        ssat = rtk->ssat + i;
        if (ui->cBSelectSatellites->currentIndex() == 1 && !vsat[i]) continue;
        satno2id(i + 1, id);
        ui->tWConsole->item(n, j++)->setText(id);
        ui->tWConsole->item(n, j++)->setText(ssat->vs ? tr("OK") : tr("-"));
        az = ssat->azel[0] * R2D; if (az < 0.0) az += 360.0;
        el = ssat->azel[1] * R2D;
        ui->tWConsole->item(n, j++)->setText(QString::number(az, 'f', 1));
        ui->tWConsole->item(n, j++)->setText(QString::number(el, 'f', 1));
        for (k = 0; k < nfreq; k++)
            ui->tWConsole->item(n, j++)->setText(ssat->vsat[k] ? tr("OK") : tr("-"));
        for (k = 0; k < nfreq; k++) {
            fix = ssat->fix[k];
            ui->tWConsole->item(n, j++)->setText(fix == 1 ? tr("FLOAT") : (fix == 2 ? tr("FIX") : (fix == 3 ? tr("HOLD") : tr("-"))));
		}
        for (k = 0; k < nfreq; k++)
            ui->tWConsole->item(n, j++)->setText(QString::number(ssat->resp[k], 'f', 3));
        for (k = 0; k < nfreq; k++)
            ui->tWConsole->item(n, j++)->setText(QString::number(ssat->resc[k], 'f', 4));
        for (k = 0; k < nfreq; k++)
            ui->tWConsole->item(n, j++)->setText(QString::number(ssat->slipc[k]));
        for (k = 0; k < nfreq; k++)
            ui->tWConsole->item(n, j++)->setText(QString::number(ssat->lock[k]));
        for (k = 0; k < nfreq; k++)
            ui->tWConsole->item(n, j++)->setText(QString::number(ssat->outc[k]));
        for (k = 0; k < nfreq; k++)
            ui->tWConsole->item(n, j++)->setText(QString::number(ssat->rejc[k]));
        ui->tWConsole->item(n, j++)->setText(QString::number(ssat->gf[0], 'f', 3));
        ui->tWConsole->item(n, j++)->setText(QString::number(ssat->phw, 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(cbias[i][0], 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(cbias[i][1], 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(0, 'f', 2));
		n++;
	}
    free(rtk);
}
//---------------------------------------------------------------------------
void MonitorDialog::setEstimates()
{
    QString label[] = {
        tr("State"), tr("Estimate Float"), tr("Std Float"), tr("Estimate Fixed"), tr("Std Fixed")
	};
    int i, width[] = {70, 160, 100, 160, 100};

    ui->tWConsole->setColumnCount(5);
    ui->tWConsole->setRowCount(0);
    header.clear();

    for (i = 0; i < ui->tWConsole->columnCount(); i++) {
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
        header << label[i];
    }
    ui->tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showEstimates()
{
	gtime_t time;
    unsigned int i, nx, na, n;
    double *x, *P = NULL, *xa = NULL, *Pa = NULL;
    QString s0 = "-";
	char tstr[40];

    rtksvrlock(rtksvr);

    time = rtksvr->rtk.sol.time;
    nx = (unsigned int)rtksvr->rtk.nx;
    na = (unsigned int)rtksvr->rtk.na;
    if ((x = (double *)malloc(sizeof(double) * nx)) &&
        (P = (double *)malloc(sizeof(double) * nx * nx)) &&
        (xa = (double *)malloc(sizeof(double) * na)) &&
        (Pa = (double *)malloc(sizeof(double) * na * na))) {
        memcpy(x, rtksvr->rtk.x, sizeof(double) * nx);
        memcpy(P, rtksvr->rtk.P, sizeof(double) * nx * nx);
        memcpy(xa, rtksvr->rtk.xa, sizeof(double) * na);
        memcpy(Pa, rtksvr->rtk.Pa, sizeof(double) * na * na);
    } else {
        rtksvrunlock(rtksvr);
        free(x); free(P); free(xa); free(Pa);
        return;
    }
    rtksvrunlock(rtksvr);

    time2str(time, tstr, 9);
    ui->lblInformation->setText(time.time ? tr("Time: %1").arg(tstr) : s0);

    for (i = 0, n = 0; i < nx; i++) {
        if (ui->cBSelectSatellites->currentIndex() == 1 && x[i] == 0.0) continue;
		n++;
	}
    if (n < 1) {
        ui->tWConsole->setRowCount(0);
        free(x); free(P); free(xa); free(Pa);
		return;
	}

    if (ui->tWConsole->rowCount() != (int)n) {
        ui->tWConsole->setRowCount(n);
        for (unsigned int row = 0; row < n; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0, n = 0; i < nx; i++) {
        int j = 0;
        if (ui->cBSelectSatellites->currentIndex() == 1 && x[i] == 0.0) continue;
        ui->tWConsole->item(n, j++)->setText(tr("X_%1").arg(i + 1));
        ui->tWConsole->item(n, j++)->setText(x[i] == 0.0 ? s0 : QString::number(x[i], 'f', 3));
        ui->tWConsole->item(n, j++)->setText(P[i + i * nx] == 0.0 ? s0 : QString::number(SQRT(P[i + i * nx]), 'f', 3));
        ui->tWConsole->item(n, j++)->setText((i >= na || qFuzzyCompare(xa[i], 0)) ? s0 : QString::number(xa[i], 'f', 3));
        ui->tWConsole->item(n, j++)->setText((i >= na || Pa[i + i * na] == 0.0) ? s0 : QString::number(SQRT(Pa[i + i * na]), 'f', 3));
		n++;
	}
	free(x); free(P); free(xa); free(Pa);
}
//---------------------------------------------------------------------------
void MonitorDialog::setCovariance()
{
    header.clear();

    ui->tWConsole->setColumnCount(0);
    ui->tWConsole->setRowCount(0);

    ui->tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showCovariance()
{
	gtime_t time;
    int i, j, nx, n, m;
    double *x, *P = NULL;
    static QString s0 = "-";
	char tstr[40];

    rtksvrlock(rtksvr);

    time = rtksvr->rtk.sol.time;
    nx = rtksvr->rtk.nx;
    if ((x = (double *)malloc(sizeof(double) * nx)) &&
        (P = (double *)malloc(sizeof(double) * nx * nx))) {
        memcpy(x, rtksvr->rtk.x, sizeof(double) * nx);
        memcpy(P, rtksvr->rtk.P, sizeof(double) * nx * nx);
    } else {
        free(x); free(P);
        rtksvrunlock(rtksvr);
        return;
    }
    rtksvrunlock(rtksvr);

    time2str(time, tstr, 9);
    ui->lblInformation->setText(time.time ? tr("Time: %1").arg(tstr) : s0);

    for (i = 0, n = 0; i < nx; i++) {
        if (ui->cBSelectSatellites->currentIndex() == 1 && (x[i] == 0.0 || P[i + i * nx] == 0.0)) continue;
		n++;
	}
    if (n < 1) {
        ui->tWConsole->setColumnCount(0);
        ui->tWConsole->setRowCount(0);
        free(x); free(P);
        return;
	}

    if ((ui->tWConsole->rowCount() != n) || (ui->tWConsole->columnCount() != n) ){
        ui->tWConsole->setColumnCount(n);
        ui->tWConsole->setRowCount(n);
        for (int row = 0; row < n; row++) {
            ui->tWConsole->setHorizontalHeaderItem(row, new QTableWidgetItem());
            ui->tWConsole->setVerticalHeaderItem(row, new QTableWidgetItem());
            for (int col = 0; col < n; col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
        }
    }

    for (i = 0, n = 0; i < nx; i++) {
        if (ui->cBSelectSatellites->currentIndex() == 1 && (x[i] == 0.0 || P[i + i * nx] == 0.0)) continue;

        ui->tWConsole->setColumnWidth(n, 45 * fontScale / 96);
        ui->tWConsole->horizontalHeaderItem(n)->setText(tr("X_%1").arg(i + 1));
        ui->tWConsole->verticalHeaderItem(n)->setText(tr("X_%1").arg(i + 1));

        for (j = 0, m = 0; j < nx; j++) {
            if (ui->cBSelectSatellites->currentIndex() == 1 && (x[j] == 0.0 || P[j + j * nx] == 0.0)) continue;

            ui->tWConsole->item(n, m)->setText(P[i + j * nx] == 0.0 ? s0 : QString::number(SQRT(P[i + j * nx]), 'f', 5));
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
    int i, j = 0, width[] = {230, 46, 40};
    int nex = ui->cBSelectObservation->currentIndex() ? NEXOBS : 0;
    // Show standard deviations when the extended observations are
    // also selected, but this could be a separate option.
    int std = ui->cBSelectObservation->currentIndex();

    ui->tWConsole->setColumnCount(3 + (NFREQ + nex) * (6 + std * 2));
    ui->tWConsole->setRowCount(0);
    header.clear();

    for (i = 0; i < 3; i++) {
        ui->tWConsole->setColumnWidth(j++, width[i] * fontScale / 96);
        header << label[i];
	}
    for (i = 0; i < NFREQ + nex; i++) {
        ui->tWConsole->setColumnWidth(j++, 40 * fontScale / 96);
        header << (i < NFREQ ? tr("C%1").arg(i+1) : tr("CX%1").arg(i - NFREQ + 1));
    }
    for (i = 0; i < NFREQ + nex; i++) {
        ui->tWConsole->setColumnWidth(j++, 50 * fontScale / 96);
        header << (i < NFREQ ? tr("S%1").arg(i+1) : tr("SX%1").arg(i - NFREQ + 1));
    }
    for (i = 0; i < NFREQ + nex; i++) {
        ui->tWConsole->setColumnWidth(j++, 135 * fontScale / 96);
        header << (i < NFREQ ? tr("P%1 (m)").arg(i+1) : tr("PX%1 (m)").arg(i - NFREQ + 1));
        if (std) {
          ui->tWConsole->setColumnWidth(j++, 65 * fontScale / 96);
          header << tr("Std");
        }
    }
    for (i = 0; i < NFREQ + nex; i++) {
        ui->tWConsole->setColumnWidth(j++, 160 * fontScale / 96);
        header << (i < NFREQ ? tr("L%1 (cycle)").arg(i+1) : tr("LX%1 (cycle)").arg(i - NFREQ + 1));
        if (std) {
          ui->tWConsole->setColumnWidth(j++, 75 * fontScale / 96);
          header << tr("Std");
        }
    }
    for (i = 0; i < NFREQ + nex; i++) {
        ui->tWConsole->setColumnWidth(j++, 120 * fontScale / 96);
        header << (i < NFREQ ? tr("D%1 (Hz)").arg(i+1) : tr("DX%1 (Hz)").arg(i - NFREQ + 1));
	}
    for (i = 0; i < NFREQ + nex; i++) {
        ui->tWConsole->setColumnWidth(j++, 15 * fontScale / 96);
        header << "I";
	}
    ui->tWConsole->setHorizontalHeaderLabels(header);

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showObservations()
{
    char tstr[40], id[8], *code;
    int i, k, n = 0, nex = ui->cBSelectObservation->currentIndex() ? NEXOBS : 0;
    int sys = sys_tbl[ui->cBSelectNavigationSystems->currentIndex()];
    int std = ui->cBSelectObservation->currentIndex();

    obsd_t *obs = static_cast<obsd_t *>(calloc(MAXOBS * 2, sizeof(obsd_t)));
    if (obs == NULL) {
      trace(1, "MonitorDialog::showObservations obsd_t alloc failed\n");
      return;
    }

    rtksvrlock(rtksvr);
    for (i = 0; i < rtksvr->obs[0][0].n && n < MAXOBS * 2; i++) {
        if (!(satsys(rtksvr->obs[0][0].data[i].sat, NULL) & sys)) continue;
        obs[n++] = rtksvr->obs[0][0].data[i];
    }
    for (i = 0; i < rtksvr->obs[1][0].n && n < MAXOBS * 2; i++) {
        if (!(satsys(rtksvr->obs[1][0].data[i].sat, NULL) & sys)) continue;
        obs[n++] = rtksvr->obs[1][0].data[i];
    }
    rtksvrunlock(rtksvr);

    if (n < 1) {
        ui->tWConsole->setRowCount(0);
        free(obs);
        return;
    }
    if (ui->tWConsole->rowCount() != n) {
        ui->tWConsole->setRowCount(n);
        for (int row = 0; row < n; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0; i < n; i++) {
        int j = 0;
        time2str(obs[i].time, tstr, 3);
        ui->tWConsole->item(i, j++)->setText(tstr);
        satno2id(obs[i].sat, id);
        ui->tWConsole->item(i, j++)->setText(id);
        ui->tWConsole->item(i, j++)->setText(QString("(%1)").arg(obs[i].rcv));
        for (k = 0; k < NFREQ + nex; k++) {
            code = code2obs(obs[i].code[k]);
            if (*code) ui->tWConsole->item(i, j++)->setText(code);
            else ui->tWConsole->item(i, j++)->setText("-");
        }
        for (k = 0; k < NFREQ + nex; k++) {
            if (obs[i].SNR[k]) ui->tWConsole->item(i, j++)->setText(QString::number(obs[i].SNR[k], 'f', 1));
            else ui->tWConsole->item(i, j++)->setText("-");
        }
        for (k = 0; k < NFREQ + nex; k++) {
            ui->tWConsole->item(i, j++)->setText(QString::number(obs[i].P[k], 'f', 3));
            if (std) {
              if (obs[i].Pstd[k]) ui->tWConsole->item(i, j++)->setText(QString::number(obs[i].Pstd[k], 'f', 3));
              else ui->tWConsole->item(i, j++)->setText("-");
            }
        }
        for (k = 0; k < NFREQ + nex; k++) {
            ui->tWConsole->item(i, j++)->setText(QString::number(obs[i].L[k], 'f', 3));
            if (std) {
              if (obs[i].Lstd[k]) ui->tWConsole->item(i, j++)->setText(QString::number(obs[i].Lstd[k], 'f', 4));
              else ui->tWConsole->item(i, j++)->setText("-");
            }
        }
        for (k = 0; k < NFREQ + nex; k++)
            ui->tWConsole->item(i, j++)->setText(QString::number(obs[i].D[k], 'f', 3));
        for (k = 0; k < NFREQ + nex; k++)
            ui->tWConsole->item(i, j++)->setText(QString::number(obs[i].LLI[k]));
	}
        free(obs);
}
//---------------------------------------------------------------------------
void MonitorDialog::setNavigationGPS()
{
    int sys = sys_tbl[ui->cBSelectSingleNavigationSystem->currentIndex() + 1];
    header.clear();
    header	<< tr("SAT") << tr("PRN") << tr("Status") << tr("IODE") << tr("IODC") << tr("URA") << tr("SVH") << tr("Toe") << tr("Toc") << tr("Ttrans")
        << tr("A (m)") << tr("e") << tr("i0 (deg)") << tr("OMEGA0 (deg)") << tr("omega (deg)") << tr("M0 (deg)")
        << tr("deltan (deg/s)") << tr("OMEGAdot (deg/s)") << tr("IDOT (deg/s)")
        << tr("Cuc (rad)") << tr("Cus (rad)") << tr("Crc (m)") << tr("Crs (m)") << tr("Cic (rad)") << tr("Cis (rad)")
        << tr("af0 (ns)") << tr("af1 (ns/s)") << tr("af2 (ns/s2)");
    if (sys == SYS_GAL)
        header	<< tr("BGD E1-E5a (ns)") << tr("BGD E1-E5b (ns)");
    else if (sys == SYS_CMP)
        header	<< tr("TGD B1I (ns)") << tr("TGD B2I/B2b (ns)") << tr("TGD B1Cp (ns)") << tr("TGD B2ap (ns)") << tr("ISC B1Cd (ns)") << tr("TGD B2ad (ns)");
    else
        header	<< tr("TGD (ns)");

    header << tr("Code") << tr("Flag");
    int i, width[] = {
        46, 46, 60, 30, 30, 25, 25, 200, 200, 200, 140, 110, 90, 140, 130, 90, 140, 140, 120,
        120, 120, 120, 120, 120, 120, 120, 120, 120,
        140, 140, 140, 140, 140, 140, 50, 50
	};
    ui->tWConsole->setColumnCount(header.count());
    ui->tWConsole->setRowCount(0);

    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    if (sys == SYS_GAL) {
        ui->lblInformation->setText((ui->cBSelectEphemeris->currentIndex() % 2) ? tr("F/NAV") : tr("I/NAV"));
    } else {
        ui->lblInformation->setText("");
    }
}
//---------------------------------------------------------------------------
void MonitorDialog::showNavigationsGPS()
{
	eph_t eph[MAXSAT];
	gtime_t time;
    QString s;
    char tstr[40], id[8];
    int i, k, n, nsat, prn, off = ui->cBSelectEphemeris->currentIndex();
    bool valid;
    int sys = sys_tbl[ui->cBSelectSingleNavigationSystem->currentIndex() + 1];

    eph_t eph0;
    memset(&eph0, 0, sizeof(eph_t));

    rtksvrlock(rtksvr);
    time = rtksvr->rtk.sol.time;
    for (i = 0; i < MAXSAT; i++) {
        if (i + off * MAXSAT < rtksvr->nav.n)
            eph[i] = rtksvr->nav.eph[i + off * MAXSAT];
        else
            eph[i] = eph0;
    }
    rtksvrunlock(rtksvr);

    for (k = 0, nsat = 0; k < MAXSAT; k++) {
        int ssys = satsys(k + 1, &prn);
        if ((ssys & sys) == 0) continue;
        // Mask QZS LEX health.
        valid = eph[k].toe.time != 0 && fabs(timediff(time, eph[k].toe)) <= MAXDTOE &&
            (ssys == SYS_QZS ? (eph[k].svh & 0xfe) == 0 : eph[k].svh == 0);
        if (ui->cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        nsat++;
	}
    if (nsat < 1) {
        ui->tWConsole->setRowCount(0);
		return;
	}
    if (ui->tWConsole->rowCount() != nsat) {
        ui->tWConsole->setRowCount(nsat);
        for (int row = 0; row < nsat; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (k = 0, n = 0; k < MAXSAT; k++) {
        int j = 0;
        int ssys = satsys(k + 1, &prn);
        if ((ssys & sys) == 0) continue;
        // Mask QZS LEX health.
        valid = eph[k].toe.time != 0 && fabs(timediff(time, eph[k].toe)) <= MAXDTOE &&
            (ssys == SYS_QZS ? (eph[k].svh & 0xfe) == 0 : eph[k].svh == 0);
        if (ui->cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        satno2id(k + 1, id);
        ui->tWConsole->item(n, j++)->setText(id);
        ui->tWConsole->item(n, j++)->setText(QString::number(prn));
        ui->tWConsole->item(n, j++)->setText(valid ? tr("OK") : tr("-"));
        if (eph[k].iode < 0) s = "-"; else s = QString::number(eph[k].iode);
        ui->tWConsole->item(n, j++)->setText(s);
        if (eph[k].iodc < 0) s = "-"; else s = QString::number(eph[k].iodc);
        ui->tWConsole->item(n, j++)->setText(s);
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].sva));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].svh, 16));
        if (eph[k].toe.time != 0) time2str(eph[k].toe, tstr, 0); else strcpy(tstr, "-");
        ui->tWConsole->item(n, j++)->setText(tstr);
        if (eph[k].toc.time != 0) time2str(eph[k].toc, tstr, 0); else strcpy(tstr, "-");
        ui->tWConsole->item(n, j++)->setText(tstr);
        if (eph[k].ttr.time != 0) time2str(eph[k].ttr, tstr, 0); else strcpy(tstr, "-");
        ui->tWConsole->item(n, j++)->setText(tstr);
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].A, 'f', 3));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].e, 'f', 8));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].i0 * R2D, 'f', 5));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].OMG0 * R2D, 'f', 5));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].omg * R2D, 'f', 5));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].M0 * R2D, 'f', 5));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].deln * R2D, 'E', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].OMGd * R2D, 'E', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].idot * R2D, 'E', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].cuc, 'E', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].cus, 'E', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].crc, 'E', 3));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].crs, 'E', 3));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].cic, 'E', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].cis, 'E', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].f0 * 1E9, 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].f1 * 1E9, 'f', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].f2 * 1E9, 'f', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].tgd[0] * 1E9, 'f', 2));
        if ((sys == SYS_GAL) || (sys == SYS_CMP))
            ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].tgd[1] * 1E9, 'f', 2));
        if (sys == SYS_CMP)
        {
            ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].tgd[2] * 1E9, 'f', 2));
            ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].tgd[3] * 1E9, 'f', 2));
            ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].tgd[4] * 1E9, 'f', 2));
            ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].tgd[5] * 1E9, 'f', 2));
        }
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].code, 16));
        ui->tWConsole->item(n, j++)->setText(QString::number(eph[k].flag, 16));
        n++;
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setGlonassNavigations()
{
    header.clear();
    header << tr("SAT") << tr("PRN") << tr("Status") << tr("IOD") << tr("FCN") << tr("SVH") << tr("Age (days)") << tr("Toe") << tr("Tof")
           << tr("X (m)") << tr("Y (m)") << tr("Z (m)") << tr("Vx (m/s)") << tr("Vy (m/s)") << tr("Vz (m/s)")
           << tr("Ax (m/s2)") << tr("Ay (m/s2)") << tr("Az (m/s2)") << tr("Tau (ns)") << tr("Gamma (ns/s)") << tr("dTau L1-L2 (ns)");

    int i, width[] = {
        46, 46, 60, 30, 30, 25, 110, 200, 200, 140, 140, 140, 130, 130, 130, 95, 95, 95, 110, 140, 140
	};
    ui->tWConsole->setColumnCount(21);
    ui->tWConsole->setRowCount(0);
    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showGlonassNavigations()
{
	geph_t geph[NSATGLO];
	gtime_t time;
    QString s;
    char tstr[40], id[8];
    int i, n, nsat, valid, prn, off = ui->cBSelectEphemeris->currentIndex();

    geph_t geph0;
    memset(&geph0, 0, sizeof(geph_t));

    rtksvrlock(rtksvr);
    time = rtksvr->rtk.sol.time;
    for (i = 0; i < NSATGLO; i++) {
        if (i + off * NSATGLO < rtksvr->nav.ng)
            geph[i] = rtksvr->nav.geph[i + off * NSATGLO];
        else
            geph[i] = geph0;
    }
    rtksvrunlock(rtksvr);

    for (i = 0, nsat = 0; i < NSATGLO; i++) {
        valid = geph[i].toe.time != 0 && fabs(timediff(time, geph[i].toe)) <= MAXDTOE_GLO &&
            (geph[i].svh & 9) == 0 && (geph[i].svh & 6) != 4;
        if (ui->cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        nsat++;
	}
    if (nsat < 1) {
        ui->tWConsole->setRowCount(0);
		return;
	}

    if (ui->tWConsole->rowCount() != nsat) {
        ui->tWConsole->setRowCount(nsat);
        for (int row = 0; row < nsat; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0, n = 0; i < NSATGLO; i++) {
        int j = 0;
        valid = geph[i].toe.time != 0 && fabs(timediff(time, geph[i].toe)) <= MAXDTOE_GLO &&
            (geph[i].svh & 9) == 0 && (geph[i].svh & 6) != 4;
        if (ui->cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        prn = MINPRNGLO + i;
        satno2id(satno(SYS_GLO, prn), id);
                ui->tWConsole->item(n, j++)->setText(id);
        ui->tWConsole->item(n, j++)->setText(QString::number(prn));
        ui->tWConsole->item(n, j++)->setText(valid ? tr("OK") : tr("-"));
        if (geph[i].iode < 0) s = "-";
        else s = QString::number(geph[i].iode);
        ui->tWConsole->item(n, j++)->setText(s);
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].frq));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].svh, 16));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].age));
        if (geph[i].toe.time != 0) time2str(geph[i].toe, tstr, 0);
        else strcpy(tstr, "-");
        ui->tWConsole->item(n, j++)->setText(tstr);
        if (geph[i].tof.time != 0) time2str(geph[i].tof, tstr, 0);
        else strcpy(tstr, "-");
        ui->tWConsole->item(n, j++)->setText(tstr);
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].pos[0], 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].pos[1], 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].pos[2], 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].vel[0], 'f', 5));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].vel[1], 'f', 5));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].vel[2], 'f', 5));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].acc[0], 'f', 7));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].acc[1], 'f', 7));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].acc[2], 'f', 7));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].taun * 1E9, 'f', 2));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].gamn * 1E9, 'f', 4));
        ui->tWConsole->item(n, j++)->setText(QString::number(geph[i].dtaun * 1E9, 'f', 2));
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

    int i, width[] = {46, 46, 60, 200, 200, 30, 30, 150, 150, 150, 100, 100, 100, 100, 100, 100, 80, 95};

    ui->tWConsole->setColumnCount(18);
    ui->tWConsole->setRowCount(0);
    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsNavigations()
{
    seph_t seph[NSATSBS];
	gtime_t time;
    int i, n, nsat, valid, prn, off = ui->cBSelectEphemeris->currentIndex();
    char tstr[40], id[8];

    seph_t seph0;
    memset(&seph0, 0, sizeof(seph_t));

    rtksvrlock(rtksvr); // lock
    time = rtksvr->rtk.sol.time;
    for (int i = 0; i < NSATSBS; i++) {
        if (i + off * NSATSBS < rtksvr->nav.ns)
            seph[i] = rtksvr->nav.seph[i + off * NSATSBS];
        else
            seph[i] = seph0;
    }
    rtksvrunlock(rtksvr); // unlock

    for (i = 0, nsat = 0; i < NSATSBS; i++) {
        valid = fabs(timediff(time, seph[i].t0)) <= MAXDTOE_SBS &&
            seph[i].t0.time && seph[i].svh == 0;
        if (ui->cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        nsat++;
	}
    if (nsat < 1) {
        ui->tWConsole->setRowCount(0);
		return;
	}
    if (ui->tWConsole->rowCount() != nsat) {
        ui->tWConsole->setRowCount(nsat);
        for (int row = 0; row < nsat; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0, n = 0; i < NSATSBS; i++) {
        int j = 0;
        valid = fabs(timediff(time, seph[i].t0)) <= MAXDTOE_SBS &&
            seph[i].t0.time && seph[i].svh == 0;
        if (ui->cBSelectSatellites->currentIndex() == 1 && !valid) continue;
        prn = MINPRNSBS + i;
        satno2id(satno(SYS_SBS, prn), id);
        ui->tWConsole->item(i, j++)->setText(id);
        ui->tWConsole->item(i, j++)->setText(QString::number(prn));
        ui->tWConsole->item(i, j++)->setText(valid ? tr("OK") : tr("-"));
        if (seph[i].t0.time) time2str(seph[i].t0, tstr, 0);
        else strcpy(tstr, "-");
        ui->tWConsole->item(i, j++)->setText(tstr);
        if (seph[i].tof.time) time2str(seph[i].tof, tstr, 0);
        else strcpy(tstr, "-");
        ui->tWConsole->item(i, j++)->setText(tstr);
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].svh, 16));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].sva));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].pos[0], 'f', 2));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].pos[1], 'f', 2));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].pos[2], 'f', 2));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].vel[0], 'f', 6));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].vel[1], 'f', 6));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].vel[2], 'f', 6));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].acc[0], 'f', 7));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].acc[1], 'f', 7));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].acc[2], 'f', 7));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].af0 * 1E9, 'f', 2));
        ui->tWConsole->item(i, j++)->setText(QString::number(seph[i].af1 * 1E9, 'f', 4));
		n++;
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setIonUtc()
{
    header.clear();
    header << tr("Parameter") << tr("Value");
    int i, width[] = {500, 600};

    ui->tWConsole->setColumnCount(2);
    ui->tWConsole->setRowCount(20);
    for (i = 0; (i < ui->tWConsole->columnCount()) && (i < 2); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    for (int row = 0; row < ui->tWConsole->rowCount(); row++)
        for (int col = 0; col < ui->tWConsole->columnCount(); col++)
            ui->tWConsole->setItem(row, col, new QTableWidgetItem());

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showIonUtc()
{
    double utc_gps[8], utc_glo[8], utc_gal[8], utc_qzs[8], utc_cmp[8], utc_irn[9];
    double ion_gps[8], ion_gal[4], ion_qzs[8], ion_cmp[8], ion_irn[8];
	gtime_t time;
    double tow = 0.0;
	char tstr[40];
    int i, week = 0;

    Q_UNUSED(utc_glo);

    rtksvrlock(rtksvr);
    time = rtksvr->rtk.sol.time;
    for (i = 0; i < 8; i++) utc_gps[i] = rtksvr->nav.utc_gps[i];
    for (i = 0; i < 8; i++) utc_glo[i] = rtksvr->nav.utc_glo[i];
    for (i = 0; i < 8; i++) utc_gal[i] = rtksvr->nav.utc_gal[i];
    for (i = 0; i < 8; i++) utc_qzs[i] = rtksvr->nav.utc_qzs[i];
    for (i = 0; i < 8; i++) utc_cmp[i] = rtksvr->nav.utc_cmp[i];
    for (i = 0; i < 9; i++) utc_irn[i] = rtksvr->nav.utc_irn[i];
    for (i = 0; i < 8; i++) ion_gps[i] = rtksvr->nav.ion_gps[i];
    for (i = 0; i < 4; i++) ion_gal[i] = rtksvr->nav.ion_gal[i];
    for (i = 0; i < 8; i++) ion_qzs[i] = rtksvr->nav.ion_qzs[i];
    for (i = 0; i < 8; i++) ion_cmp[i] = rtksvr->nav.ion_cmp[i];
    for (i = 0; i < 8; i++) ion_irn[i] = rtksvr->nav.ion_irn[i];
    rtksvrunlock(rtksvr);

    i = 0;
    time2str(timeget(), tstr, 3);
    ui->tWConsole->item(i,   0)->setText(tr("CPU Time"));
    ui->tWConsole->item(i++, 1)->setText(QString(tstr) + " UTC");

    if (time.time != 0) time2str(gpst2utc(time), tstr, 3); else strcpy(tstr, "-");
    ui->tWConsole->item(i,   0)->setText(tr("Receiver Time"));
    ui->tWConsole->item(i++, 1)->setText(QString(tstr) + " UTC");

    if (time.time != 0) time2str(time, tstr, 3); else strcpy(tstr, "-");
    ui->tWConsole->item(i,   0)->setText(tr("Receiver Time"));
    ui->tWConsole->item(i++, 1)->setText(QString(tstr) + " GPST");

    if (time.time != 0) tow = time2gpst(time, &week);
    ui->tWConsole->item(i,   0)->setText(tr("GPS Week/Time"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2 s").arg(week).arg(tow, 0, 'f', 3));

    ui->tWConsole->item(i,   0)->setText(tr("Leap Seconds dt_LS, WN_LSF, DN, dt_LSF"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1 s, %2, %3, %4 s").arg(utc_gps[4], 0, 'f', 0).arg(utc_gps[5], 0, 'f', 0).arg(utc_gps[6], 0, 'f', 0).arg(utc_gps[7], 0, 'f', 0));

    ui->tWConsole->item(i,   0)->setText(tr("GPST-UTC Reference Week/Time, A0, A1"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2 s, %3 ns, %4 ns/s").arg(utc_gps[3], 0, 'f', 0).arg(utc_gps[2], 0, 'f', 0).arg(utc_gps[0]*1E9, 0, 'f', 3).arg(utc_gps[1]*1E9, 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("GLOT-UTC Tau, Tau_GPS"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1 ns, %2 ns").arg(utc_glo[0], 0, 'f', 9).arg(utc_glo[1] * 1E9, 0, 'f', 3));

    ui->tWConsole->item(i,   0)->setText(tr("GTS-UTC Ref Week, Time, A0, A1"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2 s, %3 ns %4 ns/s").arg(utc_gal[3], 0, 'f',0).arg(utc_gal[2], 0, 'f', 0).arg(utc_gal[0]*1e9, 0, 'f', 3).arg(utc_gal[1] * 1E9, 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("QZSST-UTC Ref Week, Time, A0, A1"));
    ui->tWConsole->item(i++, 1)->setText(tr("%1, %2 s, %3 ns, %4 ns/s").arg(utc_qzs[3],0,'f',0).arg(utc_qzs[2],0,'f',0).arg(utc_qzs[0]*1E9, 0, 'f', 3).arg(utc_qzs[1]*1E9, 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("BDST-UTC Ref Week, Time, A0, A1"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2 s, %3 ns, %4 ns/s").arg(utc_cmp[3], 0, 'f', 0).arg(utc_cmp[2], 0, 'f', 0).arg(utc_cmp[0]*1e9 , 0, 'f', 3).arg(utc_cmp[1]*1e9, 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("IRNT-UTC Ref Week, Time, A0, A1, A2"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2 s, %3 ns, %4 ns/s, %5 ns/s²").arg(utc_irn[3], 0, 'f', 0).arg(utc_irn[2], 0, 'f', 0).arg(utc_irn[0]*1e9, 0, 'f', 3).arg(utc_irn[1]*1e9, 0, 'f', 5).arg(utc_irn[8]*1e9, 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("GPS Iono Parameters Alpha0-3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_gps[0], 0, 'g', 5).arg(ion_gps[1], 0, 'g', 5).arg(ion_gps[2], 0, 'g', 5).arg(ion_gps[3], 0, 'g', 5));

    ui->tWConsole->item(i,   0)->setText(tr("GPS Iono Parameters Beta0-3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_gps[4], 0, 'f', 5).arg(ion_gps[5], 0, 'f', 5).arg(ion_gps[6], 0, 'f', 5).arg(ion_gps[7], 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("Galileo Iono Parameters 0-2"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3").arg(ion_gal[0], 0, 'f', 5).arg(ion_gal[1], 0, 'f', 5).arg(ion_gal[2], 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("QZSS Iono Parameters Alpha0-Alpha3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_qzs[0], 0, 'g', 5).arg(ion_qzs[1], 0, 'g', 5).arg(ion_qzs[2], 0, 'g', 5).arg(ion_qzs[3], 0, 'g', 5));

    ui->tWConsole->item(i,   0)->setText(tr("QZSS Iono Parameters Beta0-Beta3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_qzs[4], 0, 'f', 5).arg(ion_qzs[5], 0, 'f', 5).arg(ion_qzs[6], 0, 'f', 5).arg(ion_qzs[7], 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("BDS Iono Parameters Alpha0-Alpha3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_cmp[0], 0, 'g', 5).arg(ion_cmp[1], 0, 'g', 5).arg(ion_cmp[2], 0, 'g', 5).arg(ion_cmp[3], 0, 'g', 5));

    ui->tWConsole->item(i,   0)->setText(tr("BDS Iono Parameters Beta0-Beta3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_cmp[4], 0, 'f', 5).arg(ion_cmp[5], 0, 'f', 5).arg(ion_cmp[6], 0, 'f', 5).arg(ion_cmp[7], 0, 'f', 5));

    ui->tWConsole->item(i,   0)->setText(tr("NavIC Iono Parameters Alpha0-Alpha3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_irn[0], 0, 'g', 5).arg(ion_irn[1], 0, 'g', 5).arg(ion_irn[2], 0, 'g', 5).arg(ion_irn[3], 0, 'g', 5));

    ui->tWConsole->item(i,   0)->setText(tr("NavIC Iono Parameters Beta0-Beta3"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1, %2, %3, %4").arg(ion_irn[4], 0, 'f', 5).arg(ion_irn[5], 0, 'f', 5).arg(ion_irn[6], 0, 'f', 5).arg(ion_irn[7], 0, 'f', 5));
}
//---------------------------------------------------------------------------
void MonitorDialog::setStream()
{
    header.clear();
    header	<< tr("STR") << tr("Stream") << tr("Type") << tr("Format") << tr("Mode") << tr("State") << tr("Input (bytes)") << tr("Input (bps)")
        << tr("Output (bytes)") << tr("Output (bps)") << tr("Path") << tr("Message");

    int i, width[] = {45, 170, 120, 150, 60, 55, 120, 110, 135, 120, 220, 220};

    ui->tWConsole->setColumnCount(12);
    ui->tWConsole->setRowCount(MAXSTRRTK + 1);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    for (int row = 0; row < ui->tWConsole->rowCount(); row++)
        for (int col = 0; col < ui->tWConsole->columnCount(); col++)
            ui->tWConsole->setItem(row, col, new QTableWidgetItem());

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showStream()
{
    const QString ch[MAXSTRRTK + 1] = {
        tr("Input Rover"), tr("Input Base"), tr("Input Correction"),
        tr("Log Rover"), tr("Log Base"), tr("Log Correction"),
        tr("Output Solution 1"), tr("Output Solution 2"), tr("Output Solution 3"),
        tr("Monitor")
	};
    const QString type[] = {
        tr("-"), tr("Serial"), tr("File"), tr("TCP Server"), tr("TCP Client"), tr("NTRIP Server"),
        tr("NTRIP Client"), tr("FTP"), tr("HTTP"), tr("NTRIP Caster"), tr("UDP Server"),
        tr("UDP Client"), tr("")
	};
    const QString outformat[] = {
        tr("Lat/Lon/Height"), tr("X/Y/Z-ECEF"), tr("E/N/U-Baseline"), tr("NMEA-0183"),
        tr("Solution stats"), tr("GSI F1/F2")};
    const QString state[] = {tr("Error"), tr("-"), tr("OK")};
    QString mode, form;
	stream_t stream[MAXSTRRTK + 1];
    int i, format[MAXSTRRTK + 1] = {0};
    char path[MAXSTRPATH] = "", *p, *q;

    rtksvrlock(rtksvr); // lock
    for (i = 0; i < MAXSTRRTK; i++) stream[i] = rtksvr->stream[i];
    for (i = 0; i < 3; i++) format[i] = rtksvr->format[i];
    for (i = 0; i < RTKSVRNSOL; i++) format[6+i] = rtksvr->solopt[i].posf;
    stream[MAXSTRRTK] = *monistr;
    format[MAXSTRRTK] = SOLF_LLH;
    rtksvrunlock(rtksvr); // unlock

    for (i = 0; i < MAXSTRRTK + 1; i++) {
        int j = 0;
        ui->tWConsole->item(i, j++)->setText(QString("(%1)").arg(i+1));
        ui->tWConsole->item(i, j++)->setText(ch[i]);
        ui->tWConsole->item(i, j++)->setText(type[stream[i].type]);
        if (!stream[i].type) form = "-";
        else if (i < 3) form = formatstrs[format[i]];
        else if (i >= 6) form = outformat[format[i]];
        else form = "-";
        ui->tWConsole->item(i, j++)->setText(form);
        if (stream[i].mode & STR_MODE_R) mode = tr("R"); else mode = "";
        if (stream[i].mode & STR_MODE_W) mode = mode + (mode == "" ? "" : "/") + tr("W");
        ui->tWConsole->item(i, j++)->setText(mode);
        ui->tWConsole->item(i, j++)->setText(state[stream[i].state + 1]);
        ui->tWConsole->item(i, j++)->setText(QString::number(stream[i].inb));
        ui->tWConsole->item(i, j++)->setText(QString::number(stream[i].inr));
        ui->tWConsole->item(i, j++)->setText(QString::number(stream[i].outb));
        ui->tWConsole->item(i, j++)->setText(QString::number(stream[i].outr));
        strncpy(path, stream[i].path, 1023);
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
        ui->tWConsole->item(i, j++)->setText(pp);
        ui->tWConsole->item(i, j++)->setText(stream[i].msg);
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsMessages()
{
    header.clear();
    header << tr("Trcv") << tr("PRN") << tr("STR") << tr("Type") << tr("Message") << tr("Contents");
    int i, width[] = {200, 45, 45, 45, 420, 400};

    ui->tWConsole->setColumnCount(6);
    ui->tWConsole->setRowCount(0);
    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    ui->lblInformation->setText("");
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
        tr("GEO Satellite Almanacs"), tr("Ionospheric Grid Point Masks"),
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
    char tstr[40];
    QString s;
    int i, k, n;

    rtksvrlock(rtksvr); // lock
    for (i = n = 0; i < rtksvr->nsbs; i++) {
        msg[n++] = rtksvr->sbsmsg[i];
    }
    rtksvrunlock(rtksvr); // unlock

    if (n < 1) {
        ui->tWConsole->setRowCount(0);
        return;
    }
    if (ui->tWConsole->rowCount() != n) {
        ui->tWConsole->setRowCount(n);
        for (int row = 0; row < n; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0; i < n; i++) {
        int j = 0;
        time2str(gpst2time(msg[i].week, msg[i].tow), tstr, 0);
        ui->tWConsole->item(i, j++)->setText(tstr);
        ui->tWConsole->item(i, j++)->setText(QString::number(msg[i].prn));
        ui->tWConsole->item(i, j++)->setText(QString("(%1)").arg(msg[i].rcv));
        int type = msg[i].msg[1] >> 2;
        ui->tWConsole->item(i, j++)->setText(QString::number(type));
        for (k = 0; k < 29; k++) s += QString::number(msg[i].msg[k], 16);
        ui->tWConsole->item(i, j++)->setText(s);
        for (k = 0; id[k] >= 0; k++)
            if (type == id[k]) break;
        ui->tWConsole->item(i, j++)->setText(id[k] < 0 ? "?" : content[k]);
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsLong()
{
    header.clear();
    header	<< tr("SAT") << tr("Status") << tr("IODE") << tr("dX (m)") << tr("dY (m)") << tr("dZ (m)") << tr("dVX (m/s)")
        << tr("dVY (m/s)") << tr("dVZ (m/s)") << tr("daf0 (ns)") << tr("daf1 (ns/s)") << tr("T0");

    int i, width[] = {46, 60, 50, 70, 70, 70, 100, 100, 100, 100, 100, 200};

    ui->tWConsole->setColumnCount(12);
    ui->tWConsole->setRowCount(0);
    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
    ui->tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsLong()
{
	sbssat_t sbssat;
	gtime_t time;
    int i;
    char tstr[40], id[8];

    rtksvrlock(rtksvr); // lock
    time = rtksvr->rtk.sol.time;
    sbssat = rtksvr->nav.sbssat;
    rtksvrunlock(rtksvr); // unlock

    ui->lblInformation->setText(tr("IODP: %1,  System Latency: %2 s").arg(sbssat.iodp).arg(sbssat.tlat));

    if (sbssat.nsat < 1) {
        ui->tWConsole->setRowCount(0);
        return;
    }
    if (ui->tWConsole->rowCount() != sbssat.nsat) {
        ui->tWConsole->setRowCount(sbssat.nsat);
        for (int row = 0; row < sbssat.nsat; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0; i < sbssat.nsat; i++) {
        int j = 0;
        sbssatp_t *satp = sbssat.sat + i;
        bool valid = timediff(time, satp->lcorr.t0) <= MAXSBSAGEL && satp->lcorr.t0.time;
        satno2id(satp->sat, id);
        ui->tWConsole->item(i, j++)->setText(id);
        ui->tWConsole->item(i, j++)->setText(valid ? tr("OK") : tr("-"));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.iode));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.dpos[0], 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.dpos[1], 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.dpos[2], 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.dvel[0], 'f', 4));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.dvel[1], 'f', 4));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.dvel[2], 'f', 4));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.daf0 * 1E9, 'f', 4));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->lcorr.daf1 * 1E9, 'f', 4));
        if (satp->lcorr.t0.time) time2str(satp->lcorr.t0, tstr, 0);
        else strcpy(tstr, "-");
        ui->tWConsole->item(i, j++)->setText(tstr);
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsIono()
{
    header.clear();
    header << tr("IODI") << tr("Lat (deg)") << tr("Lon (deg)") << tr("GIVEI") << tr("Delay (m)") << tr("T0");

    int i, width[] = {50, 130, 130, 50, 100, 200};

    ui->tWConsole->setColumnCount(6);
    ui->tWConsole->setRowCount(0);
    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    ui->tWConsole->setHorizontalHeaderLabels(header);
    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsIono()
{
    QString s0 = "-";
    sbsion_t sbsion[MAXBAND + 1];
	char tstr[40];
    int i, j, k, n = 0;

    rtksvrlock(rtksvr); // lock
    for (i = 0; i <= MAXBAND; i++) {
        sbsion[i] = rtksvr->nav.sbsion[i];
        n += sbsion[i].nigp;
    };
    rtksvrunlock(rtksvr); // unlock

    if (n < 1) {
        ui->tWConsole->setRowCount(0);
        return;
    }
    if (ui->tWConsole->rowCount() != n) {
        ui->tWConsole->setRowCount(n);
        for (int row = 0; row < n; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    n = 0;
    for (i = 0; i < MAXBAND + 1; i++) {
        sbsion_t *ion = sbsion + i;
        for (j = 0; j < ion->nigp; j++) {
            k = 0;
            ui->tWConsole->item(n, k++)->setText(QString::number(ion->iodi));
            ui->tWConsole->item(n, k++)->setText(QString::number(ion->igp[j].lat));
            ui->tWConsole->item(n, k++)->setText(QString::number(ion->igp[j].lon));
            ui->tWConsole->item(n, k++)->setText(ion->igp[j].give ? QString::number(ion->igp[j].give - 1) : s0);
            ui->tWConsole->item(n, k++)->setText(QString::number(ion->igp[j].delay, 'f', 3));
            if (ion->igp[j].t0.time) time2str(ion->igp[j].t0, tstr, 0);
            else strcpy(tstr, "-");
            ui->tWConsole->item(n, k++)->setText(tstr);
			n++;
		}
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setSbsFast()
{
    header.clear();
    header << tr("SAT") << tr("Status") << tr("PRC (m)") << tr("RRC (m)") << tr("IODF") << tr("UDREI") << tr("AI") << tr("Tof");

    int i, width[] = {46, 60, 80, 80, 55, 60, 30, 200};

    ui->tWConsole->setColumnCount(8);
    ui->tWConsole->setRowCount(0);
    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    ui->tWConsole->setHorizontalHeaderLabels(header);
}
//---------------------------------------------------------------------------
void MonitorDialog::showSbsFast()
{
    QString s0 = "-";
	sbssat_t sbssat;
	gtime_t time;
    int i;
    char tstr[40], id[8];

    rtksvrlock(rtksvr); // lock
    time = rtksvr->rtk.sol.time;
    sbssat = rtksvr->nav.sbssat;
    rtksvrunlock(rtksvr); // unlock

    ui->lblInformation->setText(tr("IODP: %1,  System Latency: %2 s").arg(sbssat.iodp).arg(sbssat.tlat));

    if (sbssat.nsat < 1) {
        ui->tWConsole->setRowCount(0);
        return;
    }
    if (ui->tWConsole->rowCount() != sbssat.nsat) {
        ui->tWConsole->setRowCount(sbssat.nsat);
        for (int row = 0; row < sbssat.nsat; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0; i < sbssat.nsat; i++) {
        int j = 0;
        sbssatp_t *satp = sbssat.sat + i;
        bool valid = fabs(timediff(time, satp->fcorr.t0)) <= MAXSBSAGEF && satp->fcorr.t0.time &&
                 0 <= satp->fcorr.udre - 1 && satp->fcorr.udre - 1 < 14;
        satno2id(satp->sat, id);
        ui->tWConsole->item(i, j++)->setText(id);
        ui->tWConsole->item(i, j++)->setText(valid ? tr("OK") : tr("-"));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->fcorr.prc, 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->fcorr.rrc, 'f', 4));
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->fcorr.iodf));
        ui->tWConsole->item(i, j++)->setText(satp->fcorr.udre ? QString::number(satp->fcorr.udre - 1) : s0);
        ui->tWConsole->item(i, j++)->setText(QString::number(satp->fcorr.ai));
        if (satp->fcorr.t0.time) time2str(satp->fcorr.t0, tstr, 0);
        else strcpy(tstr, "-");
        ui->tWConsole->item(i, j++)->setText(tstr);
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtcm()
{
    header.clear();
    header << tr("Parameter") << tr("Value");
    int i, width[] = {240, 520};

    ui->tWConsole->setColumnCount(2);
    ui->tWConsole->setRowCount(15);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < ui->tWConsole->columnCount() && i < 2; i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    for (int row = 0; row < ui->tWConsole->rowCount(); row++)
        for (int col = 0; col < ui->tWConsole->columnCount(); col++)
            ui->tWConsole->setItem(row, col, new QTableWidgetItem());

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showRtcm()
{
    static rtcm_t rtcm;
    int i = 0, j, format;
    QString mstr1, mstr2;
    char tstr[40] = "-";

    int effectiveStream = (inputStream < 0 || inputStream > 2) ? 0 : inputStream;

    rtksvrlock(rtksvr);
    format = rtksvr->format[effectiveStream];
    rtcm = rtksvr->rtcm[effectiveStream];
    rtksvrunlock(rtksvr);

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
    for (j = 300; j < 399; j++) {
        if (rtcm.nmsg3[j] == 0) continue;
        mstr2+=QString("%1%2(%3)").arg(mstr2.isEmpty()?"":",").arg(j+3770).arg(rtcm.nmsg3[j]);
    }
    if (rtcm.nmsg3[0] > 0)
        mstr2 += QString("%1other(%2)").arg(mstr2.isEmpty() ? "" : ",").arg(rtcm.nmsg3[0]);

    ui->tWConsole->item(i,   0)->setText(tr("Format"));
    ui->tWConsole->item(i++, 1)->setText(format == STRFMT_RTCM2 ? tr("RTCM 2") : tr("RTCM 3"));

    ui->tWConsole->item(i,   0)->setText(tr("Message Time"));
    ui->tWConsole->item(i++, 1)->setText(tstr);

    ui->tWConsole->item(i,   0)->setText(tr("Station ID"));
    ui->tWConsole->item(i++, 1)->setText(QString::number(rtcm.staid));

    ui->tWConsole->item(i,   0)->setText(tr("Station Health"));
    ui->tWConsole->item(i++, 1)->setText(QString::number(rtcm.stah));

    ui->tWConsole->item(i,   0)->setText(tr("Sequence No"));
    ui->tWConsole->item(i++, 1)->setText(QString::number(rtcm.seqno));

    ui->tWConsole->item(i,   0)->setText(tr("RTCM Special Message"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msg);

    ui->tWConsole->item(i,   0)->setText(tr("Last Message"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msgtype);

    ui->tWConsole->item(i,   0)->setText(tr("# of RTCM Messages"));
    ui->tWConsole->item(i++, 1)->setText(format == STRFMT_RTCM2 ? mstr1 : mstr2);

    ui->tWConsole->item(i,   0)->setText(tr("MSM Signals for GPS"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msmtype[0]);

    ui->tWConsole->item(i,   0)->setText(tr("MSM Signals for GLONASS"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msmtype[1]);

    ui->tWConsole->item(i,   0)->setText(tr("MSM Signals for Galileo"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msmtype[2]);

    ui->tWConsole->item(i,   0)->setText(tr("MSM Signals for QZSS"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msmtype[3]);

    ui->tWConsole->item(i,   0)->setText(tr("MSM Signals for SBAS"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msmtype[4]);

    ui->tWConsole->item(i,   0)->setText(tr("MSM Signals for BDS"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msmtype[5]);

    ui->tWConsole->item(i,   0)->setText(tr("MSM Signals for NavIC"));
    ui->tWConsole->item(i++, 1)->setText(rtcm.msmtype[6]);
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtcmDgps()
{
    header.clear();
    header << tr("SAT") << tr("Status") << tr("PRC (m)") << tr("RRC (m)") << tr("IOD") << tr("UDRE") << tr("T0");

    int i, width[] = {46, 60, 80, 80, 30, 30, 115};

    ui->tWConsole->setColumnCount(7);
    ui->tWConsole->setRowCount(MAXSAT);
    ui->tWConsole->setHorizontalHeaderLabels(header);
    for (i = 0; i < ui->tWConsole->columnCount(); i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    for (int row = 0; row < ui->tWConsole->rowCount(); row++)
        for (int col = 0; col < ui->tWConsole->columnCount(); col++)
            ui->tWConsole->setItem(row, col, new QTableWidgetItem());

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showRtcmDgps()
{
	gtime_t time;
	dgps_t dgps[MAXSAT];
    int i;
    char tstr[40], id[8];

    rtksvrlock(rtksvr);
    time = rtksvr->rtk.sol.time;
    for (i = 0; i < MAXSAT; i++) dgps[i] = rtksvr->nav.dgps[i];
    rtksvrunlock(rtksvr);

    for (i = 0; i < ui->tWConsole->rowCount(); i++) {
        int j = 0;
        satno2id(i + 1, id);
        bool valid = dgps[i].t0.time && fabs(timediff(time, dgps[i].t0)) <= 1800.0;
        ui->tWConsole->item(i, j++)->setText(id);
        ui->tWConsole->item(i, j++)->setText(valid ? tr("OK") : tr("-"));
        ui->tWConsole->item(i, j++)->setText(QString::number(dgps[i].prc, 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(dgps[i].rrc, 'f', 4));
        ui->tWConsole->item(i, j++)->setText(QString::number(dgps[i].iod));
        ui->tWConsole->item(i, j++)->setText(QString::number(dgps[i].udre));
        if (dgps[i].t0.time) time2str(dgps[i].t0, tstr, 0);
        else strcpy(tstr, "-");
        ui->tWConsole->item(i, j++)->setText(tstr);
	}
}
//---------------------------------------------------------------------------
void MonitorDialog::setRtcmSsr()
{
    header.clear();
    header	<< tr("SAT") << tr("Status") << tr("UDI (s)") << tr("UDHR (s)") << tr("IOD") << tr("URA") << tr("Datum") << tr("T0")
            << tr("D0-A (m)") << tr("D0-C (m)") << tr("D0-R (m)") << tr("D1-A (mm/s)") << tr("D1-C (mm/s)") << tr("D1-R (mm/s)")
            << tr("C0 (m)") << tr("C1 (mm/s)") << tr("C2 (mm/s²)") << tr("C-HR (m)") << tr("Code Bias (m)") << tr("Phase Bias (m)");
    int i, width[] = { 46, 60, 70, 90, 30, 25, 70, 115, 90, 90, 90, 120, 120, 120, 90, 120, 120, 120, 200, 200 };

    ui->tWConsole->setColumnCount(20);
    ui->tWConsole->setRowCount(0);
    ui->tWConsole->setHorizontalHeaderLabels(header);

    for (i = 0; i < 20; i++)
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showRtcmSsr()
{
    gtime_t time;
	ssr_t ssr[MAXSAT];
    int i, k, n, sat[MAXSAT], sys = sys_tbl[ui->cBSelectSingleNavigationSystem->currentIndex() + 1];
    char tstr[40], id[8];
    QString s;

    int effectiveStream = (inputStream < 0 || inputStream > 2) ? 0 : inputStream;

    rtksvrlock(rtksvr);
    time = rtksvr->rtk.sol.time;
    for (i = n = 0; i < MAXSAT; i++) {
        if (!(satsys(i + 1, NULL) & sys)) continue;
        ssr[n] = rtksvr->rtcm[effectiveStream].ssr[i];
        sat[n++] = i + 1;
    }
    rtksvrunlock(rtksvr);

    if (ui->tWConsole->rowCount() != n) {
        ui->tWConsole->setRowCount(n);
        for (int row = 0; row < n; row++)
            for (int col = 0; col < ui->tWConsole->columnCount(); col++)
                ui->tWConsole->setItem(row, col, new QTableWidgetItem());
    }

    for (i = 0; i < n; i++) {
        int j = 0;
        satno2id(sat[i], id);
        ui->tWConsole->item(i, j++)->setText(id);
        bool valid = ssr[i].t0[0].time && fabs(timediff(time, ssr[i].t0[0])) <= 1800.0;
        ui->tWConsole->item(i, j++)->setText(valid ? tr("OK") : tr("-"));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].udi[0], 'f', 0));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].udi[2], 'f', 0));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].iode));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].ura));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].refd));
        if (ssr[i].t0[0].time) time2str(ssr[i].t0[0], tstr, 0); else strcpy(tstr, "-");
        ui->tWConsole->item(i, j++)->setText(tstr);
        for (k = 0; k < 3; k++)
            ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].deph[k], 'f', 3));
        for (k = 0; k < 3; k++)
            ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].ddeph[k] * 1E3, 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].dclk[0], 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].dclk[1] * 1E3, 'f', 3));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].dclk[2] * 1E3, 'f', 5));
        ui->tWConsole->item(i, j++)->setText(QString::number(ssr[i].hrclk, 'f', 3));

        s = "";
        for (k = 1; k < MAXCODE; k++) {
            if (ssr[i].cbias[k - 1] == 0.0) continue;
            s += QString("%1: %2 ").arg(code2obs(k)).arg(ssr[i].cbias[k - 1], 0, 'f', 3);
        }
        if (s.isEmpty()) s = QStringLiteral("---");
        ui->tWConsole->item(i, j++)->setText(s);

        s = "";
        for (k = 1; k < MAXCODE; k++) {
            if (ssr[i].pbias[k - 1] == 0.0) continue;
            s += QString("%1: %2 ").arg(code2obs(k)).arg(ssr[i].pbias[k - 1], 0, 'f', 3);
        }
        if (s.isEmpty()) s = QStringLiteral("---");
        ui->tWConsole->item(i, j++)->setText(s);
    }
}
//---------------------------------------------------------------------------
void MonitorDialog::setReferenceStation()
{
    header << tr("Parameter") << tr("Value");
    int width[] = {400, 520};
    int i;

    ui->tWConsole->setColumnCount(2);
    ui->tWConsole->setRowCount(16);
    ui->tWConsole->setHorizontalHeaderLabels(header);
    for (i = 0; i < 2; i++) {
        ui->tWConsole->setColumnWidth(i, width[i] * fontScale / 96);
	}
    for (int row = 0; row < ui->tWConsole->rowCount(); row++)
        for (int col = 0; col < ui->tWConsole->columnCount(); col++)
            ui->tWConsole->setItem(row, col, new QTableWidgetItem());

    ui->lblInformation->setText("");
}
//---------------------------------------------------------------------------
void MonitorDialog::showReferenceStation()
{
    gtime_t time;
    sta_t sta;
    double pos[3] = {0};
    int i = 0, format;
    char tstr[40] = "-";

    int effectiveStream = (inputStream < 0 || inputStream > 2) ? 0 : inputStream;

    rtksvrlock(rtksvr);
    format = rtksvr->format[effectiveStream];
    if (format == STRFMT_RTCM2 || format == STRFMT_RTCM3) {
        time = rtksvr->rtcm[effectiveStream].time;
        sta = rtksvr->rtcm[effectiveStream].sta;
    }
    else {
        time = rtksvr->raw[effectiveStream].time;
        sta = rtksvr->raw[effectiveStream].sta;
    }
    rtksvrunlock(rtksvr);

    ui->tWConsole->item(i,   0)->setText(tr("Format"));
    ui->tWConsole->item(i++, 1)->setText(formatstrs[format]);

    if (time.time) time2str(time, tstr, 3);
    ui->tWConsole->item(i,   0)->setText(tr("Message Time"));
    ui->tWConsole->item(i++, 1)->setText(tstr);

    ui->tWConsole->item(i,   0)->setText(tr("Station Pos X/Y/Z"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1 m, %2 m, %3 m").arg(sta.pos[0], 0, 'f', 3).arg(sta.pos[1], 0, 'f', 3).arg(sta.pos[2], 0, 'f', 3));

    if (norm(sta.pos, 3) > 0.0) ecef2pos(sta.pos, pos);
    ui->tWConsole->item(i  , 0)->setText(tr("Station Lat/Lon/Height"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1 deg, %2 deg, %3 m").arg(pos[0]*R2D, 0, 'f', 8).arg(pos[1]*R2D, 0, 'f', 8).arg(pos[2], 0, 'f', 3));

    ui->tWConsole->item(i,   0)->setText(tr("ITRF Realization Year"));
    ui->tWConsole->item(i++, 1)->setText(QString::number(sta.itrf));

    ui->tWConsole->item(i,   0)->setText(tr("Antenna Delta Type"));
    ui->tWConsole->item(i++, 1)->setText(sta.deltype ? tr("X/Y/Z") : tr("E/N/U"));

    ui->tWConsole->item(i,   0)->setText(tr("Antenna Delta"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1 m, %2 m, %3 m").arg(sta.del[0], 0, 'f', 3).arg(sta.del[1], 0, 'f', 3).arg(sta.del[2], 0, 'f', 3));

    ui->tWConsole->item(i,   0)->setText(tr("Antenna Height (m)"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1 m").arg(sta.hgt, 0, 'f', 3));

    ui->tWConsole->item(i,   0)->setText(tr("Antenna Descriptor"));
    ui->tWConsole->item(i++, 1)->setText(sta.antdes);

    ui->tWConsole->item(i,   0)->setText(tr("Antenna Setup Id"));
    ui->tWConsole->item(i++, 1)->setText(QString::number(sta.antsetup));

    ui->tWConsole->item(i,   0)->setText(tr("Antenna Serial No"));
    ui->tWConsole->item(i++, 1)->setText(sta.antsno);

    ui->tWConsole->item(i,   0)->setText(tr("Receiver Type Descriptor"));
    ui->tWConsole->item(i++, 1)->setText(sta.rectype);

    ui->tWConsole->item(i,   0)->setText(tr("Receiver Firmware Version"));
    ui->tWConsole->item(i++, 1)->setText(sta.recver);

    ui->tWConsole->item(i,   0)->setText(tr("Receiver Serial No"));
    ui->tWConsole->item(i++, 1)->setText(sta.recsno);

    ui->tWConsole->item(i,   0)->setText(tr("GLONASS Code-Phase Alignment"));
    ui->tWConsole->item(i++, 1)->setText(sta.glo_cp_align ? tr("Aligned") : tr("Not aligned"));

    ui->tWConsole->item(i,   0)->setText(tr("GLONASS Code-Phase Bias C1/P1/C2/P2"));
    ui->tWConsole->item(i++, 1)->setText(QString("%1 m, %2 m, %3 m, %4 m").arg(sta.glo_cp_bias[0], 0, 'f', 2).arg(
                                   sta.glo_cp_bias[1], 0, 'f', 2).arg(sta.glo_cp_bias[2], 0, 'f', 2).arg(sta.glo_cp_bias[3], 0, 'f', 2));

}
//---------------------------------------------------------------------------
