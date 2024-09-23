//---------------------------------------------------------------------------
// plotcmn.c: rtkplot common functions
//---------------------------------------------------------------------------
#include <QString>
#include <QProcess>
#include <QColor>
#include <QStringList>
#include <QDebug>

#include "plotmain.h"
#include "helper.h"
#include "plotopt.h"

#include "ui_plotmain.h"

#include "rtklib.h"

//---------------------------------------------------------------------------
extern "C" {
    int showmsg(const char *format, ...)
    {
        Q_UNUSED(format); return 0;
    }
}
//---------------------------------------------------------------------------
const QString PTypes[] = {
    QT_TR_NOOP("Gnd Trk"), QT_TR_NOOP("Position"), QT_TR_NOOP("Velocity"), QT_TR_NOOP("Accel"),	QT_TR_NOOP("NSat"),	 QT_TR_NOOP("Residuals"), QT_TR_NOOP("Resid-EL"),
    QT_TR_NOOP("Sat Vis"), QT_TR_NOOP("Skyplot"),  QT_TR_NOOP("DOP/NSat"), QT_TR_NOOP("SNR/MP/EL"), QT_TR_NOOP("SNR/MP-EL"), QT_TR_NOOP("MP-Skyplot"), ""
};
// show message in status-bar -----------------------------------------------
void Plot::showMessage(const QString &msg)
{
    ui->lblMessage1->setText(msg);
    ui->lblMessage1->adjustSize();
    ui->wgStatus->updateGeometry();
}
// execute command ----------------------------------------------------------
int Plot::execCmd(const QString &cmd, const QStringList &opt)
{
    return QProcess::startDetached(cmd, opt);
}
// get time span and time interval ------------------------------------------
void Plot::timeSpan(gtime_t *ts, gtime_t *te, double *tint)
{
    gtime_t t0 = { 0, 0 };

    trace(3, "timeSpan\n");

    *ts = *te = t0;
    *tint = 0.0;

    if (timeEnabled[0]) *ts = timeStart;
    if (timeEnabled[1]) *te = timeEnd;
    if (timeEnabled[2]) *tint = timeInterval;
}
// get position regarding time ----------------------------------------------
double Plot::timePosition(gtime_t time)
{
    double tow;
    int week;

    if (plotOptDialog->getTimeFormat() <= 1)             // www/ssss or gpst
        tow = time2gpst(time, &week);
    else if (plotOptDialog->getTimeFormat() == 2)        // utc
        tow = time2gpst(gpst2utc(time), &week);
    else                                                 // jst
        tow = time2gpst(timeadd(gpst2utc(time), 9 * 3600.0), &week);

    return tow + (week - week) * 86400.0 * 7;
}
// show legand in status-bar ------------------------------------------------
void Plot::showLegend(const QStringList &msgs)
{
    QLabel *lblQL[] = {ui->lblQL1, ui->lblQL2, ui->lblQL3, ui->lblQL4, ui->lblQL5, ui->lblQL6, ui->lblQL7};
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "showLegend\n");

    for (int i = 0; (i < 7) & (i < msgs.count()); i++) {
        lblQL[i]->setText(msgs[i] + " ");
        int col = i + 1; // Default coloring follows the marker color index
        if (msgs[0] == " #OBS = 5 ") {
            // Match the legend color to the observation color, see observationColor()
            int n = i < 5 ? 5 - i : 0;
            col = 3 - n + (n > 2 ? 5 : 0);
        }
        setWidgetTextColor(lblQL[i], plotOptDialog->getMarkerColor(sel, col));
        auto policy = lblQL[i]->sizePolicy();
        if (msgs[i].isEmpty())
        {
            policy.setHorizontalPolicy(QSizePolicy::Ignored);
        } else {
            policy.setHorizontalPolicy(QSizePolicy::Preferred);
            lblQL[i]->adjustSize();
            lblQL[i]->updateGeometry();
        }
        lblQL[i]->setSizePolicy(policy);
    }
    ui->wgStatus->updateGeometry();
}
// get current cursor position ----------------------------------------------
bool Plot::getCurrentPosition(double *rr)
{
    sol_t *data;
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "getCurrentPosition\n");

    if (PLOT_OBS <= plotType && plotType <= PLOT_DOP) return false;
    if (!(data = getsol(solutionData + sel, solutionIndex[sel]))) return false;
    if (data->type) return false;

    for (int i = 0; i < 3; i++)
        rr[i] = data->rr[i];

    return true;
}
// get center position of plot ----------------------------------------------
bool Plot::getCenterPosition(double *rr)
{
    double xc, yc, opos[3], pos[3], enu[3] = { 0 }, dr[3];
    int i, j;

    trace(3, "getCenterPosition\n");

    if (PLOT_OBS <= plotType && plotType <= PLOT_DOP && plotType != PLOT_TRK) return false;
    if (norm(originPosition, 3) <= 0.0) return false;
    
    graphTrack->getCenter(xc, yc);
    ecef2pos(originPosition, opos);
    enu[0] = xc;
    enu[1] = yc;

    // iteratively approach center position
    for (i = 0; i < 6; i++) {
        enu2ecef(opos, enu, dr);
        for (j = 0; j < 3; j++)
            rr[j] = originPosition[j] + dr[j];
        ecef2pos(rr, pos);
        enu[2] -= pos[2];
    }

    return true;
}
// get position, velocity or accel from solutions ---------------------------
TIMEPOS *Plot::solutionToPosition(solbuf_t *sol, int index, int qflag, int type)
{
    TIMEPOS *pos, *vel, *acc;
    gtime_t ts = { 0, 0 };
    sol_t *data;
    double tint, xyz[3], xyzs[4];
    int i;

    trace(3, "solutionToPosition: n=%d\n", sol->n);

    pos = new TIMEPOS(index < 0 ? sol->n : 3, 1);

    if (index >= 0) {
        if (type == 1 && index > sol->n - 2) index = sol->n - 2;  // velocity
        if (type == 2 && index > sol->n - 3) index = sol->n - 3;  // acceleration
    }

    tint = timeEnabled[2] ? timeInterval : 0.0;

    for (i = index < 0 ? 0 : index; (data = getsol(sol, i)) != NULL; i++) {
        if (index < 0 && !screent(data->time, ts, ts, tint)) continue;
        if (qflag && data->stat != qflag) continue;

        positionToXyz(data->time, data->rr, data->type, xyz);
        covarianceToXyz(data->rr, data->qr, data->type, xyzs);
        if (xyz[2]<-RE_WGS84) continue;

        pos->t  [pos->n] = data->time;
        pos->x  [pos->n] = xyz[0];
        pos->y  [pos->n] = xyz[1];
        pos->z  [pos->n] = xyz[2];
        pos->xs [pos->n] = xyzs[0];     // var x^2
        pos->ys [pos->n] = xyzs[1];     // var y^2
        pos->zs [pos->n] = xyzs[2];     // var z^2
        pos->xys[pos->n] = xyzs[3];     // cov xy
        pos->q  [pos->n] = data->stat;
        pos->n++;

        if (index >= 0 && pos->n >= 3) break;
    }
    if (type == 0) return pos; // position

    vel = pos->tdiff();
    delete pos;
    if (type == 1) return vel; // velocity

    acc = vel->tdiff();
    delete vel;
    return acc; // acceleration
}
// get number of satellites, age and ratio from solutions -------------------
TIMEPOS *Plot::solutionToNsat(solbuf_t *sol, int index, int qflag)
{
    TIMEPOS *ns;
    sol_t *data;
    int i;

    trace(3, "solutionToNsat: n=%d\n", sol->n);

    ns = new TIMEPOS(index < 0 ? sol->n : 3, 1);

    for (i = index < 0 ? 0 : index; (data = getsol(sol, i)) != NULL; i++) {
        if (qflag && data->stat != qflag) continue;

        ns->t[ns->n] = data->time;
        ns->x[ns->n] = data->ns;
        ns->y[ns->n] = data->age;
        ns->z[ns->n] = data->ratio;
        ns->q[ns->n] = data->stat;
        ns->n++;

        if (index >= 0 && i >= 2) break;
    }
    return ns;
}
// transform solution to xyz-terms ------------------------------------------
void Plot::positionToXyz(gtime_t time, const double *rr, int type, double *xyz)
{
    double originPos[3], originGeodetic[3], r[3], enu[3], timeDiff;
    int i;

    trace(4, "positionToXyz:\n");

    if (type == 0) { // xyz
        if (time.time == 0.0 || originEpoch.time == 0.0)
            timeDiff = 0;
        else
            timeDiff = timediff(time, originEpoch);

        // interpolate origin position
        for (i = 0; i < 3; i++) {
            originPos[i] = originPosition[i];
            originPos[i] += originVelocity[i] * timeDiff;
        }

        for (i = 0; i < 3; i++)
            r[i] = rr[i] - originPos[i];
        ecef2pos(originPos, originGeodetic);
        ecef2enu(originGeodetic, r, enu);

        xyz[0] = enu[0];
        xyz[1] = enu[1];
        xyz[2] = enu[2];
    } else { // enu
        xyz[0] = rr[0];
        xyz[1] = rr[1];
        xyz[2] = rr[2];
    }
}
// transform covariance to xyz-terms ----------------------------------------
void Plot::covarianceToXyz(const double *rr, const float *qr, int type, double *xyzs)
{
    double pos[3], P[9], Q[9];

    trace(4, "covarianceToXyz:\n");

    if (type == 0) { // xyz
        ecef2pos(rr, pos);
        P[0] = qr[0];
        P[4] = qr[1];
        P[8] = qr[2];
        P[1] = P[3] = qr[3];
        P[5] = P[7] = qr[4];
        P[2] = P[6] = qr[5];

        covenu(pos, P, Q);

        xyzs[0] = Q[0];
        xyzs[1] = Q[4];
        xyzs[2] = Q[8];
        xyzs[3] = Q[1];
    } else { // enu
        xyzs[0] = qr[0];
        xyzs[1] = qr[1];
        xyzs[2] = qr[2];
        xyzs[3] = qr[3];
    }
}
// computes solution statistics ---------------------------------------------
void Plot::calcStats(const double *x, int n, double ref, double &ave, double &std, double &rms)
{
    double sum = 0.0, sumsq = 0.0;
    int i;

    trace(3, "calcStats: n=%d\n", n);

    ave = std = rms = 0.0;

    if (n <= 0) {
        return;
    }

    for (i = 0; i < n; i++) {
        sum += x[i];
        sumsq += x[i] * x[i];
    }
    ave = sum / n;
    std = n > 1 ? SQRT((sumsq - 2.0 * sum * ave + ave * ave * n) / (n - 1)) : 0.0;
    rms = SQRT((sumsq - 2.0 * sum * ref + ref * ref * n) / n);
}
// get system color ---------------------------------------------------------
QColor Plot::sysColor(int sat)
{
    switch (satsys(sat, NULL)) {
        case SYS_GPS: return plotOptDialog->getMarkerColor(0, 1);
        case SYS_GLO: return plotOptDialog->getMarkerColor(0, 2);
        case SYS_GAL: return plotOptDialog->getMarkerColor(0, 3);
        case SYS_QZS: return plotOptDialog->getMarkerColor(0, 4);
        case SYS_CMP: return plotOptDialog->getMarkerColor(0, 5);
        case SYS_IRN: return plotOptDialog->getMarkerColor(0, 6);
        case SYS_SBS: return plotOptDialog->getMarkerColor(0, 0);
        default: return plotOptDialog->getMarkerColor(0, 0);
    }
}
// get observation data color -----------------------------------------------
QColor Plot::observationColor(const obsd_t *obs, double az, double el, QVariant obstype)
{
    QColor color;
    int i, n, freq;

    trace(4, "observationColor\n");

    if (!satelliteSelection[obs->sat - 1]) return Qt::black;

    if (simulatedObservation) {
        color = sysColor(obs->sat);
    } else if (obstype.isNull()) {  // "ALL" selected
        for (i = n = 0; i < NFREQ + NEXOBS && n < 5; i++) {
            if (obs->L[i] != 0.0 || obs->P[i] != 0.0) n++;
        }
        if (n == 0) {
            return Qt::black;
        }
        color = plotOptDialog->getMarkerColor(0, 3 - n + (n > 2 ? 5 : 0));
    } else if (strcmp(obstype.typeName(), "int") == 0) {  // frequency
        freq = obstype.toInt();
        if (obs->L[freq-1] == 0.0 && obs->P[freq-1] == 0.0) {
            return Qt::black;
        }
        color = snrColor(obs->SNR[freq-1] * SNR_UNIT);
    } else {  // code
        for (i = 0; i < NFREQ + NEXOBS; i++) {
            if (!strcmp(code2obs(obs->code[i]), qPrintable(obstype.toString()))) break;
        }
        if (i >= NFREQ+NEXOBS) {
            return Qt::black;
        }
        if (obs->L[i] == 0.0 && obs->P[i] == 0.0) {
            return Qt::black;
        }
        color = snrColor(obs->SNR[i] * SNR_UNIT);
    }

    // check against elevation mask
    if (el < plotOptDialog->getElevationMask() * D2R || (plotOptDialog->getElevationMaskEnabled() && el < elevationMaskData[static_cast<int>(az * R2D + 0.5)]))
        return plotOptDialog->getHideLowSatellites() ? Qt::black : plotOptDialog->getMarkerColor(0, 0);

    return color;
}
// get observation data color -----------------------------------------------
QColor Plot::snrColor(double snr)
{
    QColor c1, c2;
    uint32_t r1, b1, g1;
    double remainder, a;
    int i;

    if (snr < 25.0) return plotOptDialog->getMarkerColor(0, 7);
    if (snr < 27.5) return plotOptDialog->getMarkerColor(0, 5);
    if (snr > 47.5) return plotOptDialog->getMarkerColor(0, 1);

    a = (snr - 27.5) / 5.0;
    i = static_cast<int>(a);
    remainder = a - i;
    c1 = plotOptDialog->getMarkerColor(0, 4 - i);
    c2 = plotOptDialog->getMarkerColor(0, 5 - i);
    r1 = static_cast<uint32_t>(remainder * c1.red() + (1.0 - remainder) * c2.red()) & 0xFF;
    g1 = static_cast<uint32_t>(remainder * c1.green() + (1.0 - remainder) * c2.green()) & 0xFF;
    b1 = static_cast<uint32_t>(remainder * c1.blue() + (1.0 - remainder) * c2.blue()) & 0xFF;

    return QColor(r1, g1, b1);
}
// get mp color -------------------------------------------------------------
QColor Plot::mpColor(double mp)
{
    QColor colors[5];
    QColor c1, c2;
    uint32_t r1, b1, g1;
    double a, remainder;
    int i;

    colors[4] = plotOptDialog->getMarkerColor(0, 5);       /*      mp> 0.6 */
    colors[3] = plotOptDialog->getMarkerColor(0, 4);       /*  0.6>mp> 0.2 */
    colors[2] = plotOptDialog->getMarkerColor(0, 3);       /*  0.2>mp>-0.2 */
    colors[1] = plotOptDialog->getMarkerColor(0, 2);       /* -0.2>mp>-0.6 */
    colors[0] = plotOptDialog->getMarkerColor(0, 1);       /* -0.6>mp      */

    if (mp >= 0.6) return colors[4];
    if (mp <= -0.6) return colors[0];

    a = (mp+0.8) / 0.4;
    i = static_cast<int>(a);
    remainder = a - i;
    c1 = colors[i];
    c2 = colors[i + 1];
    r1 = static_cast<uint32_t>(remainder * c2.red() + (1.0 - remainder) * c1.red()) & 0xFF;
    g1 = static_cast<uint32_t>(remainder * c2.green() + (1.0 - remainder) * c1.green()) & 0xFF;
    b1 = static_cast<uint32_t>(remainder * c2.blue() + (1.0 - remainder) * c1.blue()) & 0xFF;

    return QColor(r1, g1, b1);
}
// search solution by xy-position in plot -----------------------------------
int Plot::searchPosition(int x, int y)
{
    sol_t *data;
    QPoint p(x, y);
    double xp, yp, xs, ys, r, xyz[3];
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "searchPosition: x=%d y=%d\n", x, y);

    if (!ui->btnShowTrack->isChecked() || (!ui->btnSolution1->isChecked() && !ui->btnSolution2->isChecked())) return -1;
    
    graphTrack->toPos(ui->lblDisplay->mapFromGlobal(p), xp, yp);
    graphTrack->getScale(xs, ys);
    r = (plotOptDialog->getMarkSize() / 2 + 2) * xs;  // search radius

    for (int i = 0; (data = getsol(solutionData + sel, i)) != NULL; i++) {
        if (ui->cBQFlag->currentIndex() && data->stat != ui->cBQFlag->currentIndex()) continue;

        positionToXyz(data->time, data->rr, data->type, xyz);

        if (xyz[2] < -RE_WGS84) continue;
        if (SQR(xp - xyz[0]) + SQR(yp - xyz[1]) <= SQR(r)) return i;
    }

    return -1;
}
// generate time-string -----------------------------------------------------
QString Plot::timeString(gtime_t time, int n, int tsys)
{
    struct tm *t;
    QString tstr;
    char temp[64];
    QString label = "";
    double tow;
    int week;

    Q_UNUSED(tsys);

    if (plotOptDialog->getTimeFormat() == 0) { // www/ssss
        tow = time2gpst(time, &week);
        tstr = QStringLiteral("%1/%2s").arg(week, 4).arg(tow, (n > 0 ? 6 : 5) + n, 'f', n);
    } else if (plotOptDialog->getTimeFormat() == 1) { // gpst
        time2str(time, temp, n);
        tstr = temp;
        label = " GPST";
    } else if (plotOptDialog->getTimeFormat() == 2) { // utc
        time2str(gpst2utc(time), temp, n);
        tstr = temp;
        label = " UTC";
    } else { // lt
        time = gpst2utc(time);
        if (!(t = localtime(&time.time))) tstr = QStringLiteral(u"2000/01/01 00:00:00.0");  // TODO: use Qt to convert it local time convention
        else tstr = QStringLiteral(u"%1/%2/%3 %4:%5:%6.%7").arg(t->tm_year + 1900, 4, 10, QChar('0'))
                         .arg(t->tm_mon + 1, 2, 10, QChar('0')).arg(t->tm_mday, 2, 10, QChar('0')).arg(t->tm_hour, 2, 10, QChar('0')).arg(t->tm_min, 2, 10, QChar('0'))
                         .arg(t->tm_sec, 2, 10, QChar('0')).arg(static_cast<int>(time.sec * pow(10.0, n)), n, 10);
        label = " LT";
    }
    return tstr + label;
}
// latitude/longitude/height string -----------------------------------------
QString Plot::latLonString(const double *pos, int ndec)
{
    QString s;
    double dms1[3], dms2[3];

    if (plotOptDialog->getLatLonFormat() == 0) {
        s = QStringLiteral(u"%L1°%2, %L3°%4").arg(fabs(pos[0] * R2D), ndec + 4, 'f', ndec).arg(pos[0] < 0.0 ? tr("S") : tr("N"))
                .arg(fabs(pos[1] * R2D), ndec + 5, 'f', ndec).arg(pos[1] < 0.0 ? tr("W") : tr("E"));
    } else {
        deg2dms(pos[0] * R2D, dms1, ndec - 5);
        deg2dms(pos[1] * R2D, dms2, ndec - 5);
        s = QStringLiteral(u"%L1° %L2' %L3\" %4, %L5 %L6' %L7\" %8")
                .arg(fabs(dms1[0]), 3, 'f', 0).arg(dms1[1], 2, 'f', 0, QChar('0')).arg(dms1[2], ndec - 2, 'f', ndec - 5, QChar('0')).arg(pos[0] < 0.0 ? tr("S") : tr("N"))
                .arg(fabs(dms2[0]), 4, 'f', 0).arg(dms2[1], 2, 'f', 0, QChar('0')).arg(dms2[2], ndec - 2, 'f', ndec - 5, QChar('0')).arg(pos[1] < 0.0 ? tr("W") : tr("E"));
    }
    return s;
}

//---------------------------------------------------------------------------
// time-taged xyz-position class implementation
//---------------------------------------------------------------------------
// constructor --------------------------------------------------------------
TIMEPOS::TIMEPOS(int nmax, int sflg)
{
    nmax_ = nmax;
    n = 0;
    t = new gtime_t[nmax];
    x = new double [nmax];
    y = new double [nmax];
    z = new double [nmax];
    if (sflg) {
        xs = new double [nmax];
        ys = new double [nmax];
        zs = new double [nmax];
        xys = new double [nmax];
    } else {
        xs = ys = zs = xys = NULL;
    }
    q = new int [nmax];
}
// destructor ---------------------------------------------------------------
TIMEPOS::~TIMEPOS()
{
    delete [] t;
    delete [] x;
    delete [] y;
    delete [] z;
    if (xs) {
        delete [] xs;
        delete [] ys;
        delete [] zs;
        delete [] xys;
    }
    delete [] q;
}
// xyz-position difference from previous ------------------------------------
TIMEPOS *TIMEPOS::tdiff()
{
    TIMEPOS *pos = new TIMEPOS(n, 1);
    double tt;
    int i;

    for (i = 0; i < n - 1; i++) {
        tt = timediff(t[i + 1], t[i]);

        if (tt == 0.0 || fabs(tt) > MAXTDIFF) continue;

        pos->t[pos->n] = timeadd(t[i], tt / 2.0);
        pos->x[pos->n] = (x[i + 1] - x[i]) / tt;
        pos->y[pos->n] = (y[i + 1] - y[i]) / tt;
        pos->z[pos->n] = (z[i + 1] - z[i]) / tt;
        if (xs) {
            pos->xs [pos->n] = SQR(xs [i + 1]) + SQR(xs [i]);
            pos->ys [pos->n] = SQR(ys [i + 1]) + SQR(ys [i]);
            pos->zs [pos->n] = SQR(zs [i + 1]) + SQR(zs [i]);
            pos->xys[pos->n] = SQR(xys[i + 1]) + SQR(xys[i]);
        }
        pos->q[pos->n] = qMax(q[i], q[i + 1]);
        pos->n++;
    }
    return pos;
}
// xyz-position difference between TIMEPOS ----------------------------------
TIMEPOS *TIMEPOS::diff(const TIMEPOS *pos2, int qflag)
{
    TIMEPOS *pos1 = this;
    TIMEPOS *pos = new TIMEPOS(qMin(n, pos2->n), 1);
    double tt;
    int i, j, q;

    for (i = 0, j = 0; i < pos1->n && j < pos2->n; i++, j++) {
        tt = timediff(pos1->t[i], pos2->t[j]);

        if (tt < -TTOL) {
            j--; continue;
        } else if (tt > TTOL) {
            i--; continue;
        }

        pos->t[pos->n] = pos1->t[i];
        pos->x[pos->n] = pos1->x[i] - pos2->x[j];
        pos->y[pos->n] = pos1->y[i] - pos2->y[j];
        pos->z[pos->n] = pos1->z[i] - pos2->z[j];
        if (pos->xs) {
            pos->xs [pos->n] = SQR(pos1->xs [i]) + SQR(pos2->xs [j]);
            pos->ys [pos->n] = SQR(pos1->ys [i]) + SQR(pos2->ys [j]);
            pos->zs [pos->n] = SQR(pos1->zs [i]) + SQR(pos2->zs [j]);
            pos->xys[pos->n] = SQR(pos1->xys[i]) + SQR(pos2->xys[j]);
        }
        q = qMax(pos1->q[i], pos2->q[j]);

        if (!qflag || qflag == q)
            pos->q[pos->n++] = q;
    }
    return pos;
}
//---------------------------------------------------------------------------
