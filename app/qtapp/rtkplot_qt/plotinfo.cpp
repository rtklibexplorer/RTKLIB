//---------------------------------------------------------------------------
// plotinfo.c: rtkplot info functions
//---------------------------------------------------------------------------
#include "plotmain.h"
#include "plotopt.h"

#include "ui_plotmain.h"

#include <QComboBox>
#include <QPushButton>
#include <QCheckBox>

#include "rtklib.h"

#define ATAN2(x, y)  ((x) * (x) + (y) * (y) > 1E-12 ? atan2(x, y) : 0.0)

// update information on status-bar -----------------------------------------
void Plot::updateStatusBarInformation()
{
    int showObs = (PLOT_OBS <= plotType && plotType <= PLOT_DOP) ||
                   plotType == PLOT_SNR || plotType == PLOT_SNRE || plotType == PLOT_MPS;

    trace(3, "updateStatusBarInformation:\n");

    if (ui->btnShowTrack->isChecked()) {
        if (showObs)
            updateTimeObservation();
        else
            updateTimeSolution();
    } else {
        if (showObs)
            updateInfoObservation();
        else
            updateInfoSolution();
    }
}
// update time-information for observation-data plot ------------------------
void Plot::updateTimeObservation()
{
    static QStringList legend_freqs = {" #OBS = 5 ", " 4 ", " 3 ", " 2 ", " 1", "", ""};
    static QStringList legend_snr = {" SNR = ...45.", "..40.", "..35.", "..30.", "..25 ", "", " <25 "};
    static QStringList legend_sys = {" SYS = GPS ", "GLO ", "GAL ", "QZS ", "BDS ", "IRN ", "SBS"};
    static QStringList legend_mp = {" MP = ..0.6", "..0.3", "..0.0..", "-0.3..", "-0.6..", "", ""};
    QStringList legend;
    QString msg;
    double azel[MAXOBS * 2], dop[4] = { 0 };
    int i, nsat = 0, nobs = 0;
    QString tstr;

    trace(3, "updateTimeObservation\n");

    // gather azimuth and elevation data
    if (ui->btnSolution1->isChecked() && observationIndex >= 0 && observationIndex < nObservation) {
        for (i = indexObservation[observationIndex]; i < observation.n && i < indexObservation[observationIndex + 1]; i++, nobs++) {
            if (satelliteMask[observation.data[i].sat - 1] || !satelliteSelection[observation.data[i].sat - 1]) continue;
            if (elevation[i] < plotOptDialog->getElevationMask() * D2R) continue;
            if (plotOptDialog->getElevationMaskEnabled() && elevation[i] < elevationMaskData[static_cast<int>(azimuth[i] * R2D + 0.5)]) continue;

            azel[nsat * 2] = azimuth[i];
            azel[1 + nsat * 2] = elevation[i];
            nsat++;
        }
    }

    if (nsat >= 0) {
        tstr = timeString(observation.data[indexObservation[observationIndex]].time, 3, 1);
        msg = QStringLiteral("[1] %1 : N = %2 ").arg(tstr).arg(nobs);

        if (plotType == PLOT_DOP) {
            dops(nsat, azel, plotOptDialog->getElevationMask() * D2R, dop);

            legend.append(QStringLiteral("NSAT = %1").arg(nsat));
            legend.append(QStringLiteral(" GDOP = %1").arg(dop[0], 0, 'f', 1));
            legend.append(QStringLiteral(" PDOP = %1").arg(dop[1], 0, 'f', 1));
            legend.append(QStringLiteral(" HDOP = %1").arg(dop[2], 0, 'f', 1));
            legend.append(QStringLiteral(" VDOP = %1").arg(dop[3], 0, 'f', 1));
        } else if (plotType <= PLOT_SKY && ui->cBObservationType->currentIndex() == 0) {
            msg += QStringLiteral("NSAT = %1 ").arg(nsat);
            legend = simulatedObservation ? legend_sys : legend_freqs;
        } else if (plotType == PLOT_MPS) {
            msg += QStringLiteral("NSAT = %1 ").arg(nsat);
            legend = legend_mp;
        } else {
            msg += QStringLiteral("NSAT = %1 ").arg(nsat);
            legend = simulatedObservation ? legend_sys : legend_snr;
        }
    }

    showMessage(msg);
    showLegend(legend);
}
// update time-information for solution plot --------------------------------
void Plot::updateTimeSolution()
{
    static QStringList units = { "m", "m/s", "m/s²" };
    static QStringList solutionType = { tr(""), tr("FIX"), tr("FLOAT"), tr("SBAS"), tr("DGPS"), tr("Single"), tr("PPP") };
    QString msg;
    QList<QString> legend;

    sol_t *data;
    double pos[3], r, az, el;
    int sel = ui->btnSolution1->isChecked() || !ui->btnSolution2->isChecked() ? 0 : 1, solIndex = solutionIndex[sel];

    trace(3, "updateTimeSolution\n");

    if ((ui->btnSolution1->isChecked() || ui->btnSolution2->isChecked() || ui->btnSolution12->isChecked()) && (data = getsol(solutionData + sel, solIndex))) {
        if (!connectState)
            msg = QStringLiteral("[%1] ").arg(sel + 1);
        else
            msg = "[R] ";

        msg += timeString(data->time, 2, 1) + " : ";

        if (PLOT_SOLP <= plotType && plotType <= PLOT_SOLA) {
            TIMEPOS *p = solutionToPosition(solutionData + sel, solIndex, 0, plotType - PLOT_SOLP);
            msg += QStringLiteral("E = %1%4, N = %2%4, U = %2%4, Q = ")
                       .arg(p->x[0], 7, 'f', 4)
                       .arg(p->y[0], 7, 'f', 4)
                       .arg(p->z[0], 7, 'f', 4)
                       .arg(units[plotType - PLOT_SOLP]);
            delete p;
        } else if (plotType == PLOT_NSAT) {
            msg += QStringLiteral("NS = %1, AGE = %2, RATIO = %3, Q = ").arg(data->ns).arg(data->age, 0, 'f', 1).arg(data->ratio, 0, 'f', 1);
        } else if (!data->type) {
            ecef2pos(data->rr, pos);
            msg += latLonString(pos, 9) + QStringLiteral(", %1 m,  Q = ").arg(pos[2], 9, 'f', 4);
        } else {
            r = norm(data->rr, 3);
            az = norm(data->rr, 2) <= 1E-12 ? 0.0 : atan2(data->rr[0], data->rr[1]) * R2D;
            el = r <= 1E-12 ? 0.0 : asin(data->rr[2] / r) * R2D;
            msg += QStringLiteral("B = %1 m, D = %2°, %3°,  Q = ")
                       .arg(r, 0, 'f', 3)
                       .arg(az < 0.0 ? az + 360.0 : az, 6, 'f', 2)
                       .arg(el, 5, 'f', 2);
        }
        if (1 <= data->stat && data->stat <= 6)
            legend.append(QStringLiteral("%1: %2").arg(data->stat).arg(solutionType[data->stat]));
    }
    showMessage(msg);
    showLegend(legend);
}
// update statistics-information for observation-data plot ------------------
void Plot::updateInfoObservation()
{
    static QStringList legend_dop = {" NSAT", " GDOP", " PDOP", " HDOP", " VDOP", "", ""};
    static QStringList legend_freqs = {" #OBS = 5 ", " 4 ", " 3 ", " 2 ", " 1 ", "", ""};
    static QStringList legend_snr = {" SNR = ...45.", "..40.", "..35.", "..30.", "..25 ", "", " <25 "};
    static QStringList legend_sys = {" SYS = GPS ", "GLO ", "GAL ", "QZS ", "BDS ", "IRN ", "SBS"};
    static QStringList legend_mp = {" MP = ..0.6", "..0.3", "..0.0..", "-0.3..", "-0.6..", "", ""};
    QString msg;
    QStringList legend;
    QString timeStartString, timeEndString;
    gtime_t timeStart = { 0, 0 }, timeEnd = { 0, 0 }, time, previousTime = { 0, 0 };
    int i, nobs = 0, ne = 0, p;

    trace(3, "updateInfoObservation:\n");

    if (ui->btnSolution1->isChecked()) {
        for (i = 0; i < observation.n; i++) {
            time = observation.data[i].time;

            if (timeStart.time == 0) timeStart = time; // save start time
            timeEnd = time;  // update end time

            if (previousTime.time == 0 || timediff(time, previousTime) > TTOL) ne++;

            nobs++;
            previousTime = time;
        }
    }
    if (nobs > 0) {
        timeStartString = timeString(timeStart, 0, 0);
        timeEndString = timeString(timeEnd, 0, 1);

        if (plotOptDialog->getTimeFormat() && (p = timeStartString.indexOf(' '))) timeStartString = timeStartString.left(p);
        timeEndString = timeEndString.mid(plotOptDialog->getTimeFormat() ? 5 : 0);

        msg = QStringLiteral("[1] %1-%2 : EP = %3, N = %4").arg(timeStartString, timeEndString).arg(ne).arg(nobs);

        if (plotType == PLOT_DOP) {
            legend = legend_dop;
        } else if (plotType <= PLOT_SKY && ui->cBObservationType->currentIndex() == 0) {
            legend = simulatedObservation ? legend_sys : legend_freqs;
        } else if (plotType == PLOT_MPS) {
            legend = legend_mp;
        } else if (plotType == PLOT_SNRE) {
            legend = QStringList({"", "", "", "", "", "", ""});
        } else {
            legend = simulatedObservation ? legend_sys : legend_snr;
        }
    }
    showMessage(msg);
    showLegend(legend);
}
// update statistics-information for solution plot --------------------------
void Plot::updateInfoSolution()
{
    QString msg;
    QStringList legend;
    QString timeStartString, timeEndString;
    TIMEPOS *pos = NULL, *pos1, *pos2;
    sol_t *data;
    gtime_t timeStart = { 0, 0 }, timeEnd = { 0, 0 };
    double r[3], baseline, baselineMinMax[2] = { 1E9, 0.0 };
    int i, j, p, n = 0, nq[8] = { 0 }, sel = ui->btnSolution1->isChecked() || !ui->btnSolution2->isChecked() ? 0 : 1;

    trace(3, "updateInfoSolution:\n");

    if (ui->btnSolution1->isChecked() || ui->btnSolution2->isChecked()) {
        pos = solutionToPosition(solutionData + sel, -1, 0, 0);
    } else if (ui->btnSolution12->isChecked()) {
        pos1 = solutionToPosition(solutionData, -1, 0, 0);
        pos2 = solutionToPosition(solutionData + 1, -1, 0, 0);
        pos = pos1->diff(pos2, 0);
        delete pos1;
        delete pos2;
    }    
    if (pos) {
        for (i = 0; i < pos->n; i++) {
            if (timeStart.time == 0) timeStart = pos->t[i];
            timeEnd = pos->t[i];
            nq[pos->q[i]]++;
            n++;
        }
        delete pos;
    }

    for (i = 0; (data = getsol(solutionData + sel, i)) != NULL; i++) {
        if (data->type) {
            baseline = norm(data->rr, 3);
        } else if (norm(solutionData[sel].rb, 3) > 0.0) {
            for (j = 0; j < 3; j++)
                r[j] = data->rr[j] - solutionData[sel].rb[j];
            baseline = norm(r, 3);
        } else {
            baseline = 0.0;
        }
        if (baseline < baselineMinMax[0]) baselineMinMax[0] = baseline;
        if (baseline > baselineMinMax[1]) baselineMinMax[1] = baseline;
    }

    if (n > 0) {
        if (!connectState)
            msg = QStringLiteral("[%1] ").arg(sel + 1);
        else
            msg = "[R] ";

        timeStartString = timeString(timeStart, 0, 0);
        timeEndString = timeString(timeEnd, 0, 1);
        if (plotOptDialog->getTimeFormat() && (p = timeStartString.lastIndexOf(' '))) timeStartString = timeStartString.left(p);

        msg += QStringLiteral("%1 - %2: N = %3").arg(timeStartString, timeEndString).arg(n);

        if (baselineMinMax[0] - baselineMinMax[1] > 100)
            msg += QStringLiteral(", B = %1-%2 km").arg(baselineMinMax[0] / 1E3, 0, 'f', 1).arg(baselineMinMax[1] / 1E3, 0, 'f', 1);
        else
            msg += QStringLiteral(", B = %1 km").arg(baselineMinMax[0] / 1E3, 0, 'f', 1);

        msg += ", Q = ";

        for (i = 1; i <= 6; i++) {
            if (nq[i] <= 0)
                legend.append("");
            else
                legend.append(QStringLiteral("%1: %2 (%3%) ").arg(i).arg(nq[i]).arg(static_cast<double>(nq[i]) / n * 100.0, 0, 'f', 1));
        }
    }
    showMessage(msg);
    showLegend(legend);
}
// update plot-type pull-down menu ------------------------------------------
void Plot::updatePlotTypeMenu()
{
    trace(3, "updatePlotType\n");
    
    ui->cBPlotTypeSelection->blockSignals(true);
    ui->cBPlotTypeSelection->clear();

    if (solutionData[0].n > 0 || solutionData[1].n > 0 || (nObservation <= 0 && solutionStat[0].n <= 0 && solutionStat[1].n <= 0)) {
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_TRK], PLOT_TRK);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_SOLP], PLOT_SOLP);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_SOLV], PLOT_SOLV);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_SOLA], PLOT_SOLA);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_NSAT], PLOT_NSAT);
    }

    if (nObservation > 0) {
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_OBS], PLOT_OBS);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_SKY], PLOT_SKY);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_DOP], PLOT_DOP);
    }

    if (solutionStat[0].n > 0 || solutionStat[1].n > 0) {
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_RES], PLOT_RES);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_RESE], PLOT_RESE);
    }

    if ((nObservation > 0)  && (!simulatedObservation)) {
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_SNR], PLOT_SNR);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_SNRE], PLOT_SNRE);
        ui->cBPlotTypeSelection->addItem(PTypes[PLOT_MPS], PLOT_MPS);
    }

    // select previously selected item again
    for (int i = 0; i < ui->cBPlotTypeSelection->count(); i++) {
        if (ui->cBPlotTypeSelection->itemData(i) != plotType) continue;
        ui->cBPlotTypeSelection->setCurrentIndex(i);
        ui->cBPlotTypeSelection->blockSignals(false);
        return;
    }

    ui->cBPlotTypeSelection->setCurrentIndex(0);
    
    ui->cBPlotTypeSelection->blockSignals(false);
}
// update satellite-list pull-down menu -------------------------------------
void Plot::updateSatelliteList()
{
    int i, j, sys, previousSys = 0, sat, satMask[MAXSAT] = { 0 };
    char s[8];

    trace(3, "updateSatelliteList\n");

    // populate satMask from solutions and observations
    for (i = 0; i < 2; i++)
        for (j = 0; j < solutionStat[i].n; j++) {
            sat = solutionStat[i].data[j].sat;
            if (1 <= sat && sat <= MAXSAT) satMask[sat - 1] = 1;
        }
    for (j = 0; j < observation.n; j++) {
        sat = observation.data[j].sat;
        if (1 <= sat && sat <= MAXSAT) satMask[sat - 1] = 1;
    }

    // update combobox
    ui->cBSatelliteList->clear();
    ui->cBSatelliteList->addItem("ALL");

    // add satellite systems
    for (sat = 1; sat <= MAXSAT; sat++) {
        if (satelliteMask[sat - 1] || !satMask[sat - 1]) continue;
        if ((sys = satsys(sat, NULL)) == previousSys) continue;
        switch (sys) {
            case SYS_GPS: ui->cBSatelliteList->addItem("G"); break;
            case SYS_GLO: ui->cBSatelliteList->addItem("R"); break;
            case SYS_GAL: ui->cBSatelliteList->addItem("E"); break;
            case SYS_QZS: ui->cBSatelliteList->addItem("J"); break;
            case SYS_CMP: ui->cBSatelliteList->addItem("C"); break;
            case SYS_IRN: ui->cBSatelliteList->addItem("I"); break;
            case SYS_SBS: ui->cBSatelliteList->addItem("S"); break;
        };
        previousSys = sys;
    }

    // add individual satellites
    for (sat = 1; sat <= MAXSAT; sat++) {
        if (satelliteMask[sat - 1] || !satMask[sat - 1]) continue;
        satno2id(sat, s);
        ui->cBSatelliteList->addItem(s);
    }
    ui->cBSatelliteList->setCurrentIndex(0);

    updateSatelliteSelection();
}
// string compare --------------------------------------------------------------
static int _strcmp(const void *str1, const void *str2)
{
    return strcmp(*(const char **)str1, *(const char **)str2);
}
// update observation type pull-down menu --------------------------------------
void Plot::updateObservationType()
{
    char *codes[MAXCODE + 1], *obs;
    int i, j, n = 0, codeMask[MAXCODE + 1] = { 0 };

    trace(3, "updateObservationType\n");

    // populate codeMask
    for (i = 0; i < observation.n; i++)
        for (j = 0; j < NFREQ + NEXOBS; j++)
            codeMask[observation.data[i].code[j]] = 1;

    // count codes
    for (uint8_t c = 1; c <= MAXCODE; c++) {
        if (!codeMask[c]) continue;
        if (!*(obs = code2obs(c))) continue;
        codes[n++] = obs;
    }
    qsort(codes, n, sizeof(char *), _strcmp);

    // update combo boxes
    ui->cBObservationType->clear();
    ui->cBObservationTypeSNR->clear();
    ui->cBObservationType->addItem(tr("ALL"));

    for (i = 0; i < NFREQ; i++) {
        ui->cBObservationType->addItem(QStringLiteral("L%1").arg(i+1), i+1);
        ui->cBObservationTypeSNR->addItem(QStringLiteral("L%1").arg(i+1), i+1);
    }
    for (i = 0; i < n; i++) {
        ui->cBObservationType->addItem(QStringLiteral("L%1").arg(codes[i]), QString(codes[i]));
        ui->cBObservationTypeSNR->addItem(QStringLiteral("L%1").arg(codes[i]), QString(codes[i]));
    }
    ui->cBObservationType->setCurrentIndex(0);
    ui->cBObservationTypeSNR->setCurrentIndex(0);
}
// update information for current-cursor position ---------------------------
void Plot::updatePoint(int x, int y)
{
    gtime_t time;
    QPoint p = ui->lblDisplay->mapFromGlobal(QPoint(x, y));
    double enu[3] = { 0 }, rr[3], pos[3], xx, yy, r, xl[2], yl[2], q[2], az, el;
    int i;
    QString msg;

    trace(4, "updatePoint: x=%d y=%d\n", x, y);

    if (plotType == PLOT_TRK) { // track-plot
        graphTrack->toPos(p, enu[0], enu[1]);

        if (pointCoordinateType == 1 || norm(originPosition, 3) <= 0.0) {
            msg = tr("E: %1 m, N: %2 m").arg(enu[0], 0, 'f', 4).arg(enu[1], 0, 'f', 4);
        } else if (pointCoordinateType == 2) {
            r = norm(enu, 2);
            az = r <= 0.0 ? 0.0 : ATAN2(enu[0], enu[1]) * R2D;
            if (az < 0.0) az += 360.0;

            msg = tr("R: %1 m, D: %2°").arg(r, 0, 'f', 4).arg(az, 5, 'f', 1);
        } else {
            ecef2pos(originPosition, pos);
            enu2ecef(pos, enu, rr);
            for (i = 0; i < 3; i++) rr[i] += originPosition[i];
            ecef2pos(rr, pos);

            msg = latLonString(pos, 8);
        }
    } else if (plotType == PLOT_SKY || plotType == PLOT_MPS) { // sky-plot
        graphSky->getLimits(xl, yl);
        graphSky->toPos(p, q[0], q[1]);
        r = (xl[1] - xl[0] < yl[1] - yl[0] ? xl[1] - xl[0] : yl[1] - yl[0]) * 0.45;

        if ((el = 90.0 - 90.0 * norm(q, 2) / r) > 0.0) {
            az = el >= 90.0 ? 0.0 : ATAN2(q[0], q[1]) * R2D;
            if (az < 0.0) az += 360.0;

            msg = tr("Az: %1°, El: %2°").arg(az, 5, 'f', 1).arg(el, 4, 'f', 1);
        }
    } else if (plotType == PLOT_SNRE || plotType==PLOT_RESE) { // snr-el-plot
        graphDual[0]->toPos(p, q[0], q[1]);

        msg = tr("El: %1°").arg(q[0], 4, 'f', 1);
    } else {
        graphTriple[0]->toPos(p, xx, yy);
        time = gpst2time(week, xx);
        if (plotOptDialog->getTimeFormat() == 2)
            time = utc2gpst(time);                         // UTC
        else if (plotOptDialog->getTimeFormat() == 3)
            time = timeadd(gpst2utc(time), -9 * 3600.0);   // JST

        msg = timeString(time, 0, 1);
    }
    ui->lblMessage2->setVisible(true);
    ui->lblMessage2->setText(msg);
}
//---------------------------------------------------------------------------
QString Plot::plotDetails(const QPoint &globalPos) {
    QPoint pos = ui->lblDisplay->mapFromGlobal(globalPos);

    if (ui->lblDisplay->geometry().contains(pos)) {
        double xx, yy, xs, ys, xt, yp[MAXSAT] = { 0 };
        int i, j, sats[MAXSAT] = { 0 }, nSats = 0;
        QPoint p1, p2;
        gtime_t time;
        char id[16];

        switch (plotType) {
        case PLOT_OBS: // TODO: add other plot types
            // count used satellites
            for (i = 0; i < observation.n; i++) {
                if (!satelliteSelection[observation.data[i].sat - 1]) continue;
                sats[observation.data[i].sat - 1] = 1;
            }
            for (i = 0; i < MAXSAT; i++)
                if (sats[i]) nSats++;

            for (i = 0, j = 0; i < MAXSAT; i++) {
                if (!sats[i]) continue;
                yp[nSats - (++j)] = i;
            }

            graphSingle->toPos(pos, xx, yy);
            graphSingle->getExtent(p1, p2);
            graphSingle->getScale(xs, ys);

            for (i = 0; i < observation.n; i++) {
                obsd_t *obs = &observation.data[i];

                xt = timePosition(obs->time);
                if (fabs(xt - xx) / xs > 0.9) {
                    int sat = static_cast<int>((yy/ys) / (p2.y() - p1.y()) * nSats - 0.5);
                    satno2id(yp[sat] + 1, id);

                    time = gpst2time(week, xx);
                    if (plotOptDialog->getTimeFormat() == 2)
                        time = utc2gpst(time);                         // UTC
                    else if (plotOptDialog->getTimeFormat() == 3)
                        time = timeadd(gpst2utc(time), -9 * 3600.0);   // JST

                    return QString("%1: %2").arg(timeString(time, 0, 1)).arg(id);
                }
            }
        };
    };
    return QString();
}
