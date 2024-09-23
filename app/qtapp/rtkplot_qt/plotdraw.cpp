//---------------------------------------------------------------------------
// plotdraw.c: rtkplot draw functions
//---------------------------------------------------------------------------
#include <QColor>
#include <QPainter>
#include <QDebug>

#include "rtklib.h"
#include "plotmain.h"
#include "graph.h"
#include "mapview.h"
#include "plotopt.h"
#include "vmapdlg.h"
#include "skydlg.h"
#include "mapoptdlg.h"

#include "ui_plotmain.h"

#define COL_ELMASK  Qt::red
#define ATAN2(x, y)  ((x) * (x) + (y) * (y) > 1E-12 ? atan2(x, y) : 0.0)

// update plot --------------------------------------------------------------
void Plot::updatePlot()
{
    trace(3, "updatePlot\n");
    
    updateStatusBarInformation();
    refresh();
}
// refresh plot -------------------------------------------------------------
void Plot::refresh()
{
    trace(3, "refresh\n");

    flush = 1;
    updateDisplay();
}
// draw plot ----------------------------------------------------------------
void Plot::updateDisplay()
{
    int level = dragState ? 0 : 1;

    trace(3, "updateDisplay\n");

    if (flush) {
        QGuiApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        QApplication::processEvents();
        if (buffer.size() != ui->lblDisplay->size())
            buffer = QPixmap(ui->lblDisplay->size());

        if (buffer.isNull()) return;
        buffer.fill(plotOptDialog->getCColor(0));

        QPainter c(&buffer);
        c.setRenderHint(QPainter::Antialiasing);

        c.setFont(plotOptDialog->getFont());
        c.setPen(plotOptDialog->getCColor(0));
        c.setBrush(plotOptDialog->getCColor(0));

        switch (plotType) {
            case  PLOT_TRK: drawTrack(c, level);   break;
            case  PLOT_SOLP: drawSolution(c, level, 0); break;
            case  PLOT_SOLV: drawSolution(c, level, 1); break;
            case  PLOT_SOLA: drawSolution(c, level, 2); break;
            case  PLOT_NSAT: drawNsat(c, level);   break;
            case  PLOT_OBS: drawObservation(c, level);   break;
            case  PLOT_SKY: drawSky(c, level);   break;
            case  PLOT_DOP: drawDop(c, level);   break;
            case  PLOT_RES: drawResidual(c, level);   break;
            case  PLOT_RESE: drawResidualE(c, level);   break;
            case  PLOT_SNR: drawSnr(c, level);   break;
            case  PLOT_SNRE: drawSnrE(c, level);   break;
            case  PLOT_MPS: drawMpSky(c, level);   break;
        }

        ui->lblDisplay->setPixmap(buffer);
        QGuiApplication::restoreOverrideCursor();
    }

    flush = 0;
}
// draw track-plot ----------------------------------------------------------
void Plot::drawTrack(QPainter &c, int level)
{
    QString header;
    TIMEPOS *pos, *pos1, *pos2, *vel;
    gtime_t time1 = { 0, 0 }, time2 = { 0, 0 };
    sol_t *sol;
    QPoint p1, p2;
    double xt, yt, sx, sy, opos[3], pnt[3], rr[3], enu[3]={}, cent[3];
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0, p = 0;

    trace(3, "drawTrack: level=%d\n", level);

    if (ui->btnShowTrack->isChecked() && ui->btnFixCenter->isChecked()) {
        // set map position to center
        if (!ui->btnSolution12->isChecked()) {
            pos = solutionToPosition(solutionData + sel, solutionIndex[sel], 0, 0);

            if (pos->n > 0) graphTrack->setCenter(pos->x[0], pos->y[0]);

            delete pos;
        } else {
            pos1 = solutionToPosition(solutionData, solutionIndex[0], 0, 0);
            pos2 = solutionToPosition(solutionData + 1, solutionIndex[1], 0, 0);
            pos = pos1->diff(pos2, 0);

            if (pos->n > 0) graphTrack->setCenter(pos->x[0], pos->y[0]);

            delete pos;
            delete pos1;
            delete pos2;
        }
    }

    // background image
    if (!ui->btnSolution12->isChecked() && ui->btnShowImage->isChecked())
        drawTrackImage(c, level);

    // gis map
    if (ui->btnShowMap->isChecked())
        drawTrackGis(c, level);

    // grid
    if (ui->btnShowGrid->isChecked()) {
        graphTrack->setLabelUnits(tr(" m"), tr(" m"));

        if (level) { // draw "+" at center
            graphTrack->getExtent(p1, p2);
            p1.setX((p1.x() + p2.x()) / 2);
            p1.setY((p1.y() + p2.y()) / 2);
            drawMark(graphTrack, c, p1, Graph::MarkerTypes::Plus, plotOptDialog->getCColor(1), 20, 0);
        }
        if (plotOptDialog->getShowGridLabel() >= 3) { // circles
            graphTrack->xLabelPosition = Graph::LabelPosition::Axis;
            graphTrack->yLabelPosition = Graph::LabelPosition::Axis;
            graphTrack->drawCircles(c, plotOptDialog->getShowGridLabel() == 4);
        }
        else if (plotOptDialog->getShowGridLabel() >= 1) { // grid
            graphTrack->xLabelPosition = Graph::LabelPosition::Inner;
            graphTrack->yLabelPosition = Graph::LabelPosition::InnerRot;
            graphTrack->drawAxis(c, plotOptDialog->getShowGridLabel(), plotOptDialog->getShowGridLabel() == 2);
        }
    }

    if (norm(originPosition, 3) > 0.0) {
        ecef2pos(originPosition, opos);
        header = tr("ORI = %1, %L2 m").arg(latLonString(opos, 9)).arg(opos[2], 0, 'f', 4);
    }

    // draw tracks
    if (ui->btnSolution1->isChecked()) {
        pos = solutionToPosition(solutionData, -1, ui->cBQFlag->currentIndex(), 0);
        drawTrackPoint(c, pos, level, 0);
        if (ui->btnShowMap->isChecked() && norm(solutionData[0].rb, 3) > 1E-3)
            drawTrackPosition(c, solutionData[0].rb, 0, 8, plotOptDialog->getCColor(2), tr("Base Station 1"));
        drawTrackStatistics(c, pos, header, p++);
        header = "";
        delete pos;
    }

    if (ui->btnSolution2->isChecked()) {
        pos = solutionToPosition(solutionData + 1, -1, ui->cBQFlag->currentIndex(), 0);
        drawTrackPoint(c, pos, level, 1);
        if (ui->btnShowMap->isChecked() && norm(solutionData[1].rb, 3) > 1E-3)
            drawTrackPosition(c, solutionData[1].rb, 0, 8, plotOptDialog->getCColor(2), tr("Base Station 2"));
        drawTrackStatistics(c, pos, header, p++);
        delete pos;
    }

    if (ui->btnSolution12->isChecked()) {
        pos1 = solutionToPosition(solutionData    , -1, 0, 0);
        pos2 = solutionToPosition(solutionData + 1, -1, 0, 0);
        pos = pos1->diff(pos2, ui->cBQFlag->currentIndex());
        drawTrackPoint(c, pos, level, 0);
        drawTrackStatistics(c, pos, "", p++);
        delete pos;
        delete pos1;
        delete pos2;
    }

    if (ui->btnShowTrack->isChecked() && ui->btnSolution1->isChecked()) {
        pos = solutionToPosition(solutionData, solutionIndex[0], 0, 0);

        if ((sol = getsol(solutionData, solutionIndex[0]))) time1 = sol->time;

        if (pos->n) {
            pos->n = 1;
            drawTrackError(c, pos, 0);
            graphTrack->toPoint(pos->x[0], pos->y[0], p1);

            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 12, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 10, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Plus, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 14, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 6, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(0, pos->q[0]), plotOptDialog->getMarkSize() * 2 + 4, 0);

            if (ui->btnSolution2->isChecked()) {  // label solution if both solutions are shown
                p1.rx() += plotOptDialog->getMarkSize() + 8;
                drawLabel(graphTrack, c, p1, "1", Graph::Alignment::Left, Graph::Alignment::Center);
            }
        }
        delete pos;
    }

    if (ui->btnShowTrack->isChecked() && ui->btnSolution2->isChecked()) {
        pos = solutionToPosition(solutionData + 1, solutionIndex[1], 0, 0);

        if ((sol = getsol(solutionData + 1, solutionIndex[1]))) time2 = sol->time;

        if (pos->n > 0 && (time1.time == 0 || fabs(timediff(time1, time2)) < DTTOL * 2.0)) {  // additionally check time difference w.r.t. first solution
            pos->n = 1;
            drawTrackError(c, pos, 1);
            graphTrack->toPoint(pos->x[0], pos->y[0], p1);

            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 12, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(1), plotOptDialog->getMarkSize() * 2 + 10, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Plus, plotOptDialog->getCColor(1), plotOptDialog->getMarkSize() * 2 + 14, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 6, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(1, pos->q[0]), plotOptDialog->getMarkSize() * 2 + 4, 0);

            if (ui->btnSolution1->isChecked()) { // label solution if both solutions are shown
                p1.setX(p1.x() + plotOptDialog->getMarkSize() + 8);
                drawLabel(graphTrack, c, p1, "2", Graph::Alignment::Left, Graph::Alignment::Center);
            }
        }
        delete pos;
    }
    if (ui->btnShowTrack->isChecked() && ui->btnSolution12->isChecked()) {
        pos1 = solutionToPosition(solutionData, solutionIndex[0], 0, 0);
        pos2 = solutionToPosition(solutionData + 1, solutionIndex[1], 0, 0);
        pos = pos1->diff(pos2, 0);

        if (pos->n > 0) {
            pos->n = 1;
            drawTrackError(c, pos, 1);
            graphTrack->toPoint(pos->x[0], pos->y[0], p1);

            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 12, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 10, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Plus, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 14, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 6, 0);
            graphTrack->drawMark(c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(0, pos->q[0]), plotOptDialog->getMarkSize() * 2 + 4, 0);
        }
        delete pos;
        delete pos1;
        delete pos2;
    }

    // draw waypoints
    if (level && ui->btnShowMap->isChecked()) {
        for (int i = 0; i < wayPoints.size(); i++) {

            pnt[0] = wayPoints[i].position[0] * D2R;
            pnt[1] = wayPoints[i].position[1] * D2R;
            pnt[2] = wayPoints[i].position[2];
            pos2ecef(pnt, rr);

            if (i == selectedWayPoint)  // highlight selected waypoint
                drawTrackPosition(c, rr, 0, 16, Qt::red, wayPoints[selectedWayPoint].name);
            else
                drawTrackPosition(c, rr, 0, 8, plotOptDialog->getCColor(2), wayPoints[i].name);
        }
    }

    // draw compass
    if (plotOptDialog->getShowCompass()) {
        graphTrack->getExtent(p1, p2);
        p1.rx() += SIZE_COMPASS / 2 + 25;
        p1.ry() += SIZE_COMPASS / 2 + 35;
        drawMark(graphTrack, c, p1, Graph::MarkerTypes::Compass, plotOptDialog->getCColor(2), SIZE_COMPASS, 0);
    }

    // draw velocity indicator
    if (plotOptDialog->getShowArrow() && ui->btnShowTrack->isChecked()) {
        vel = solutionToPosition(solutionData + sel, solutionIndex[sel], 0, 1);
        drawTrackVelocityIndicator(c, vel);
        delete vel;
    }

    // draw map scaling
    if (plotOptDialog->getShowScale()) {
        QString label;
        graphTrack->getExtent(p1, p2);
        graphTrack->getTick(xt, yt);
        graphTrack->getScale(sx, sy);

        // draw horizontal scale
        p2.rx() -= 70;
        p2.ry() -= 25;
        drawMark(graphTrack, c, p2, Graph::MarkerTypes::HScale, plotOptDialog->getCColor(2), static_cast<int>(xt / sx + 0.5), 0);

        // draw text
        p2.ry() -= 3;
        if (xt < 0.0099)
            label = QStringLiteral("%1 mm").arg(xt * 1000.0, 0, 'f', 0);
        else if (xt < 0.999)
            label = QStringLiteral("%1 cm").arg(xt * 100.0, 0, 'f', 0);
        else if (xt < 999.0)
            label = QStringLiteral("%1 m").arg(xt, 0, 'f', 0);
        else
            label = QStringLiteral("%1 km").arg(xt / 1000.0, 0, 'f', 0);
        drawLabel(graphTrack, c, p2, label, Graph::Alignment::Center, Graph::Alignment::Bottom);
    }

    if (!level) { // "+" at center
        graphTrack->getCenter(xt, yt);
        graphTrack->toPoint(xt, yt, p1);
        drawMark(graphTrack, c, p1, Graph::MarkerTypes::Plus, plotOptDialog->getCColor(2), 20, 0);
    }

    // update center of "map view"
    if (level) {
        if (norm(originPosition, 3) > 0.0) {
            graphTrack->getCenter(xt, yt);
            graphTrack->toPoint(xt, yt, p1);
            graphTrack->toPos(p1, enu[0], enu[1]);
            ecef2pos(originPosition, opos);
            enu2ecef(opos, enu, rr);
            for (int i = 0; i < 3; i++) rr[i] += originPosition[i];
            ecef2pos(rr, cent);
            mapView->setCenter(cent[0] * R2D, cent[1] * R2D);
        }
        refresh_MapView();
    }
}
// draw map-image on track-plot ---------------------------------------------
void Plot::drawTrackImage(QPainter &c, int level)
{
    gtime_t time = {0, 0};
    QPoint p1, p2;
    double pos[3] = {0}, rr[3], xyz[3] = {0}, x1, x2, y1, y2;

    trace(3, "drawTrackImage: level=%d\n", level);

    pos[0] = mapOptDialog->getMapLatitude() * D2R;
    pos[1] = mapOptDialog->getMapLongitude() * D2R;
    pos2ecef(pos, rr);
    if (norm(originPosition, 3) > 0.0)
        positionToXyz(time, rr, 0, xyz);

    x1 = xyz[0] - mapOptDialog->getMapSize(0) * 0.5 * mapOptDialog->getMapScaleX();
    x2 = xyz[0] + mapOptDialog->getMapSize(0) * 0.5 * mapOptDialog->getMapScaleX();
    y1 = xyz[1] - mapOptDialog->getMapSize(1) * 0.5 * (mapOptDialog->getMapScaleEqual() ? mapOptDialog->getMapScaleX() : mapOptDialog->getMapScaleY());
    y2 = xyz[1] + mapOptDialog->getMapSize(1) * 0.5 * (mapOptDialog->getMapScaleEqual() ? mapOptDialog->getMapScaleX() : mapOptDialog->getMapScaleY());

    graphTrack->toPoint(x1, y2, p1);
    graphTrack->toPoint(x2, y1, p2);
    QRect r(p1, p2);
    if (mapImageScaled.size() != r.size())  // avoid scaling the image each time it is drawn
        mapImageScaled = mapImageOriginal.scaled(r.size());
    c.drawImage(r, mapImageScaled);
}
// check in boundary --------------------------------------------------------
#define P_IN_B(pos, bound) \
    (pos[0] >= bound[0] && pos[0] <= bound[1] && pos[1] >= bound[2] && pos[1] <= bound[3])

#define B_IN_B(bound1, bound2) \
    (bound1[0] <= bound2[1] && bound1[1] >= bound2[0] && bound1[2] <= bound2[3] && bound1[3] >= bound2[2])

// draw gis-map on track-plot ----------------------------------------------
void Plot::drawTrackGis(QPainter &c, int level)
{
    gisd_t *data;
    gis_pnt_t *pnt;
    gis_poly_t *poly;
    gis_polygon_t *polygon;
    gtime_t time = { 0, 0 };
    QColor color;
    QPoint *p, p1;
    double xyz[3], S, xl[2], yl[2], enu[8][3] = { { 0 } }, opos[3], pos[3], rr[3];
    double bound[4] = { PI / 2.0, -PI / 2.0, PI, -PI };
    int i, j, n, m;

    trace(3, "drawTrackGis: level=%d\n", level);

    // get map boundary
    graphTrack->getLimits(xl, yl);
    enu[0][0] = xl[0]; enu[0][1] = yl[0];  // top left
    enu[1][0] = xl[1]; enu[1][1] = yl[0];  // bottom left
    enu[2][0] = xl[0]; enu[2][1] = yl[1];  // top right
    enu[3][0] = xl[1]; enu[3][1] = yl[1];  // bottom right
    enu[4][0] = (xl[0] + xl[1]) / 2.0; enu[4][1] = yl[0];  // top mid
    enu[5][0] = (xl[0] + xl[1]) / 2.0; enu[5][1] = yl[1];  // bottom mid
    enu[6][0] = xl[0]; enu[6][1] = (yl[0] + yl[1]) / 2.0;  // mid left
    enu[7][0] = xl[1]; enu[7][1] = (yl[0] + yl[1]) / 2.0;  // mid right

    ecef2pos(originPosition, opos);

    // derive boundaries from the 8 enu position
    for (i = 0; i < 8; i++) {
        if (norm(enu[i], 2) >= 1000000.0) {
            bound[0] =-PI / 2.0;
            bound[1] = PI / 2.0;
            bound[2] =-PI;
            bound[3] = PI;
            break;
        }
        enu2ecef(opos, enu[i], rr);
        for (j = 0; j < 3; j++) rr[j] += originPosition[j];
        ecef2pos(rr, pos);
        if (pos[0] < bound[0]) bound[0] = pos[0];       // min lat
        if (pos[0] > bound[1]) bound[1] = pos[0];       // max lat
        if (pos[1] < bound[2]) bound[2] = pos[1];       // min lon
        if (pos[1] > bound[3]) bound[3] = pos[1];       // max lon
    }

    for (i = MAXMAPLAYER - 1; i >= 0; i--) {
        if (!gis.flag[i]) continue;  // not enabled

        for (data = gis.data[i]; data; data = data->next) {
            if (data->type == 1) { // point
                pnt = static_cast<gis_pnt_t *>(data->data);
                if (!P_IN_B(pnt->pos, bound)) continue;

                positionToXyz(time, pnt->pos, 0, xyz);
                if (xyz[2] < -RE_WGS84) continue;

                graphTrack->toPoint(xyz[0], xyz[1], p1);
                drawMark(graphTrack, c, p1, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 6, 0);
                drawMark(graphTrack, c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), 2, 0);
            } else if (level && data->type == 2) { // polyline
                poly = static_cast<gis_poly_t *>(data->data);
                if ((n = poly->npnt) <= 0 || !B_IN_B(poly->bound, bound))
                    continue;

                p = new QPoint[n];
                for (j = m = 0; j < n; j++) {
                    positionToXyz(time, poly->pos + j * 3, 0, xyz);
                    if (xyz[2] < -RE_WGS84) { // interrupt polyline
                        if (m > 1) {
                            // draw polyline so far and start a new one
                            graphTrack->drawPoly(c, p, m, vecMapDialog->getMapColor(i), 0);
                            m = 0;
                        }
                        continue;
                    }
                    graphTrack->toPoint(xyz[0], xyz[1], p1);
                    if (m == 0 || p1.x() != p[m - 1].x() || p1.y() != p[m - 1].y())
                        p[m++] = p1;
                }
                graphTrack->drawPoly(c, p, m, vecMapDialog->getMapColor(i), 0);
                delete [] p;
            } else if (level && data->type == 3) { // polygon
                polygon = (gis_polygon_t *)data->data;
                if ((n = polygon->npnt) <= 0 || !B_IN_B(polygon->bound, bound))
                    continue;

                p = new QPoint[n];
                for (j = m = 0; j < n; j++) {
                    positionToXyz(time, polygon->pos + j * 3, 0, xyz);
                    if (xyz[2] < -RE_WGS84) {
                        continue;  // skip point
                    }
                    graphTrack->toPoint(xyz[0], xyz[1], p1);
                    if (m == 0 || p1.x() != p[m - 1].x() || p1.y() != p[m - 1].y())
                        p[m++] = p1;
                }
                // judge hole
                for (j = 0, S = 0.0; j < m - 1; j++)
                    S += static_cast<double>(p[j].x() * p[j + 1].y() - p[j + 1].x() * p[j].y());
                color = S < 0.0 ? plotOptDialog->getCColor(0) : vecMapDialog->getMapColorF(i);
                graphTrack->drawPatch(c, p, m, vecMapDialog->getMapColor(i), color, 0);
                delete [] p;
            }
        }
    }
}
// draw track-points on track-plot ------------------------------------------
void Plot::drawTrackPoint(QPainter &c, const TIMEPOS *pos, int level, int style)
{
    int i;

    trace(3, "drawTrackPoint: level=%d style=%d\n", level, style);

    if (level) drawTrackArrow(c, pos);

    if (level && plotOptDialog->getPlotStyle() <= 1 && !ui->btnShowTrack->isChecked()) // error circle
        drawTrackError(c, pos, style);

    if ((plotOptDialog->getPlotStyle() % 2) == 0)  // a style with lines
        graphTrack->drawPoly(c, pos->x, pos->y, pos->n, plotOptDialog->getCColor(3), style);

    if (level && plotOptDialog->getPlotStyle() < 2) {  // a style with markers
        QColor* color = new QColor[pos->n];

        if (ui->btnShowImage->isChecked()) {
            for (i = 0; i < pos->n; i++) color[i] = plotOptDialog->getCColor(0);
            graphTrack->drawMarks(c, pos->x, pos->y, color, pos->n, Graph::MarkerTypes::Dot, plotOptDialog->getMarkSize() + 2, 0);
        }

        for (i = 0; i < pos->n; i++)
            color[i] = plotOptDialog->getMarkerColor(style, pos->q[i]);
        graphTrack->drawMarks(c, pos->x, pos->y, color, pos->n, Graph::MarkerTypes::Dot, plotOptDialog->getMarkSize(), 0);
        delete[] color;
    }

}
// draw point with label on track-plot --------------------------------------
void Plot::drawTrackPosition(QPainter &c, const double *rr, int type, int siz, QColor color, const QString &label)
{
    gtime_t time = { 0, 0 };
    QPoint p1;
    double xyz[3], xs, ys;

    trace(3,"drawTrackPosition: type=%d rr=%.3f %.3f %.3f\n", type, rr[0], rr[1], rr[2]);

    if (norm(rr, 3) > 0.0) {
        graphTrack->getScale(xs, ys);
        positionToXyz(time, rr, type, xyz);
        graphTrack->toPoint(xyz[0], xyz[1], p1);

        // draw point
        drawMark(graphTrack, c, p1, Graph::MarkerTypes::Plus, color, siz + 6, 0);
        drawMark(graphTrack, c, p1, Graph::MarkerTypes::Circle, color, siz, 0);
        drawMark(graphTrack, c, p1, Graph::MarkerTypes::Circle, color, siz - 6, 0);

        // draw label
        p1.ry() += 10;
        drawLabel(graphTrack, c, p1, label, Graph::Alignment::Center, Graph::Alignment::Top);
    }
}
// draw statistics on track-plot --------------------------------------------
void Plot::drawTrackStatistics(QPainter &c, const TIMEPOS *pos, const QString &header, int p)
{
    QString s[6];
    QPoint p1, p2;
    double *d, ave[4], std[4], rms[4];
    int i, n = 0, fonth = (int)(QFontMetrics(plotOptDialog->getFont()).height() * 1.2);

    trace(3, "drawTrackStatistics: p=%d\n", p);

    if (!plotOptDialog->getShowStats()) return;

    if (p == 0 && !header.isEmpty()) s[n++] = header;

    if (pos->n > 0) {
        d = new double[pos->n];

        for (i = 0; i < pos->n; i++)
            d[i] = SQRT(SQR(pos->x[i]) + SQR(pos->y[i]));

        calcStats(pos->x, pos->n, 0.0, ave[0], std[0], rms[0]);
        calcStats(pos->y, pos->n, 0.0, ave[1], std[1], rms[1]);
        calcStats(pos->z, pos->n, 0.0, ave[2], std[2], rms[2]);
        calcStats(d, pos->n, 0.0, ave[3], std[3], rms[3]);

        s[n++] = tr("AVE = E: %L1 m, N: %L2 m, U: %L3 m").arg(ave[0], 7, 'f', 4).arg(ave[1], 7, 'f', 4).arg(ave[2], 7, 'f', 4);
        s[n++] = tr("STD = E: %L1 m, N: %L2 m, U: %L3 m").arg(std[0], 7, 'f', 4).arg(std[1], 7, 'f', 4).arg(std[2], 7, 'f', 4);
        s[n++] = tr("RMS = E: %L1 m, N: %L2 m, U: %L3 m, 2D: %L4 m").arg(rms[0], 7, 'f', 4).arg(rms[1], 7, 'f', 4).arg(rms[2], 7, 'f', 4).arg(2.0 * rms[3], 7, 'f', 4);

        delete [] d;
    }
    graphTrack->getExtent(p1, p2);
    p1.rx() = p2.x() - 10;
    p1.ry() += 8 + fonth * 4 * p;

    for (i = 0; i < n; i++, p1.ry() += fonth)
        drawLabel(graphTrack, c, p1, s[i], Graph::Alignment::Right, Graph::Alignment::Top);
}
// draw error-circle on track-plot ------------------------------------------
void Plot::drawTrackError(QPainter &c, const TIMEPOS *pos, int style)
{
    static const double sint[36] = {  // sinus table
        0.0000,	 0.1736,  0.3420,  0.5000,  0.6428,  0.7660,  0.8660,  0.9397,	0.9848,
        1.0000,	 0.9848,  0.9397,  0.8660,  0.7660,  0.6428,  0.5000,  0.3420,	0.1736,
        0.0000,	 -0.1736, -0.3420, -0.5000, -0.6428, -0.7660, -0.8660, -0.9397, -0.9848,
        -1.0000, -0.9848, -0.9397, -0.8660, -0.7660, -0.6428, -0.5000, -0.3420, -0.1736
    };
    double xc[37], yc[37], a, b, s, cc;
    int i, j;

    trace(3, "drawTrackError: style=%d\n", style);

    if (!plotOptDialog->getShowError()) return;

    for (i = 0; i < pos->n; i++) {
        if (pos->xs[i] <= 0.0 || pos->ys[i] <= 0.0) continue;

        a = pos->xys[i] / SQRT(pos->xs[i]);

        if ((b = pos->ys[i] - a * a) >= 0.0) b = SQRT(b);
        else continue;

        for (j = 0; j < 37; j++) {
            s = sint[j % 36];
            cc = sint[(45 - j) % 36];
            xc[j] = pos->x[i] + SQRT(pos->xs[i]) * cc;
            yc[j] = pos->y[i] + a * cc + b * s;
        }
        graphTrack->drawPoly(c, xc, yc, 37, plotOptDialog->getCColor(1), plotOptDialog->getShowError() == 1 ? 0 : 1);
    }
}
// draw direction-arrow on track-plot ---------------------------------------
void Plot::drawTrackArrow(QPainter &c, const TIMEPOS *pos)
{
    QPoint p;
    double tt, d[2], dist, dt, vel;
    int i, off = 8;

    trace(3, "drawTrackArrow\n");

    if (!plotOptDialog->getShowArrow()) return;

    for (i = 1; i < pos->n - 1; i++) {
        tt = time2gpst(pos->t[i], NULL);

        d[0] = pos->x[i + 1] - pos->x[i - 1];
        d[1] = pos->y[i + 1] - pos->y[i - 1];
        dist = norm(d, 2);

        dt = timediff(pos->t[i + 1], pos->t[i - 1]);
        vel = dt == 0.0 ? 0.0 : dist / dt;

        if (vel < 0.5 || fmod(tt + 0.005, INTARROW) >= 0.01) continue;

        graphTrack->toPoint(pos->x[i], pos->y[i], p);
        p.rx() -= static_cast<int>(off * d[1] / dist);
        p.ry() -= static_cast<int>(off * d[0] / dist);
        drawMark(graphTrack, c, p, Graph::MarkerTypes::Arrow, plotOptDialog->getCColor(3), 15, static_cast<int>(ATAN2(d[1], d[0]) * R2D));
    }
}
// draw velocity-indicator on track-plot ------------------------------------
void Plot::drawTrackVelocityIndicator(QPainter &c, const TIMEPOS *vel)
{
    QString label;
    QPoint p1, p2;
    double v = 0.0, dir = 0.0;

    trace(3, "drawTrackVelocityIndicator\n");

    if (vel && vel->n > 0) {
        if ((v = SQRT(SQR(vel->x[0]) + SQR(vel->y[0]))) > 1.0)
            dir = ATAN2(vel->x[0], vel->y[0]) * R2D;
    }

    graphTrack->getExtent(p1, p2);
    p1.rx() += SIZE_VELC / 2 + 30;
    p1.ry() = p2.y() - SIZE_VELC / 2 - 30;
    drawMark(graphTrack, c, p1, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), SIZE_VELC, 0);

    p1.ry() += SIZE_VELC / 2;
    label = QStringLiteral("%1 km/h").arg(v * 3600.0 / 1000.0, 0, 'f', 0);
    drawLabel(graphTrack, c, p1, label, Graph::Alignment::Center, Graph::Alignment::Top);

    p1.ry() -= SIZE_VELC / 2;
    if (v >= 1.0)
        drawMark(graphTrack, c, p1, Graph::MarkerTypes::Arrow, plotOptDialog->getCColor(2), SIZE_VELC, 90 - static_cast<int>(dir));
    drawMark(graphTrack, c, p1, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), 8, 0);
    drawMark(graphTrack, c, p1, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 8, 0);
}
// draw solution-plot -------------------------------------------------------
void Plot::drawSolution(QPainter &c, int level, int type)
{
    QString label[] = { tr("E-W"), tr("N-S"), tr("U-D") }, unit[] = { "m", "m/s", QString("m/s²") };
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3 };
    TIMEPOS *pos, *pos1, *pos2;
    gtime_t time1 = {0, 0}, time2 = {0, 0};
    QPoint p1, p2;
    double xc, yc, xl[2], yl[2], off, y;
    int panel, k, p = 0, bottomPanel;
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "drawSolution: level=%d\n", level);

    // update panel center
    if (ui->btnShowTrack->isChecked() && (ui->btnFixHorizontal->isChecked() || ui->btnFixVertical->isChecked())) {
        pos = solutionToPosition(solutionData + sel, solutionIndex[sel], 0, type);

        for (int panel = 0; panel < 3 && pos->n > 0; panel++) {
            graphTriple[panel]->getCenter(xc, yc);

            if (ui->btnFixVertical->isChecked())
                yc = (panel == 0) ? pos->x[0] : (panel == 1 ? pos->y[0] : pos->z[0]);

            if (ui->btnFixHorizontal->isChecked()) {
                graphTriple[panel]->getLimits(xl, yl);
                off = centX * (xl[1] - xl[0]) / 2.0;
                graphTriple[panel]->setCenter(timePosition(pos->t[0]) - off, yc);
            } else {
                graphTriple[panel]->setCenter(xc, yc);
            }
        }
        delete pos;
    }

    bottomPanel = -1;
    for (int i = 0; i < 3; i++)
        if (btn[i]->isChecked()) bottomPanel = i;

    for (int panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        graphTriple[panel]->setLabelUnits("", " " + unit[type]);
        graphTriple[panel]->xLabelPosition = plotOptDialog->getTimeFormat() ? (panel == bottomPanel ? Graph::LabelPosition::Time : Graph::LabelPosition::None) : \
            (panel == bottomPanel ? Graph::LabelPosition::Outer : Graph::LabelPosition::Off);
        graphTriple[panel]->week = week;
        graphTriple[panel]->drawAxis(c, plotOptDialog->getShowGridLabel(), plotOptDialog->getShowGridLabel());
    }

    // draw solution 1
    if (ui->btnSolution1->isChecked()) {
        pos = solutionToPosition(solutionData, -1, ui->cBQFlag->currentIndex(), type);
        drawSolutionPoint(c, pos, level, 0);
        drawSolutionStat(c, pos, unit[type], p++);

        delete pos;
    }

    // draw solution 2
    if (ui->btnSolution2->isChecked()) {
        pos = solutionToPosition(solutionData + 1, -1, ui->cBQFlag->currentIndex(), type);
        drawSolutionPoint(c, pos, level, 1);
        drawSolutionStat(c, pos, unit[type], p++);

        delete pos;
    }

    // draw solution difference
    if (ui->btnSolution12->isChecked()) {
        pos1 = solutionToPosition(solutionData, -1, 0, type);
        pos2 = solutionToPosition(solutionData + 1, -1, 0, type);
        pos = pos1->diff(pos2, ui->cBQFlag->currentIndex());
        drawSolutionPoint(c, pos, level, 0);
        drawSolutionStat(c, pos, unit[type], p++);

        delete pos;
        delete pos1;
        delete pos2;
    }

    if (ui->btnShowTrack->isChecked() && (ui->btnSolution1->isChecked() || ui->btnSolution2->isChecked() || ui->btnSolution12->isChecked())) {
        pos = solutionToPosition(solutionData + sel, solutionIndex[sel], 0, type);
        pos1 = solutionToPosition(solutionData, solutionIndex[0], 0, type);
        pos2 = solutionToPosition(solutionData + 1, solutionIndex[1], 0, type);
        if (pos1->n > 0) time1 = pos1->t[0];
        if (pos2->n > 0) time2 = pos2->t[0];

        for (panel = k = 0; panel < 3 && pos->n > 0; panel++) {
            if (!btn[panel]->isChecked()) continue;

            // draw outline
            graphTriple[panel]->getLimits(xl, yl);
            xl[0] = xl[1] = timePosition(pos->t[0]);
            graphTriple[panel]->drawPoly(c, xl, yl, 2, plotOptDialog->getCColor(2), 0);

            // draw current position
            if (ui->btnSolution2->isChecked() && pos2->n > 0 && (time1.time == 0 || fabs(timediff(time1, time2)) < DTTOL * 2.0)) {
                xl[0] = xl[1] = timePosition(time2);
                y = (panel == 0) ? pos2->x[0] : (panel == 1 ? pos2->y[0] : pos2->z[0]);

                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 6, 0);
                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(1), plotOptDialog->getMarkSize() * 2 + 6, 0);
                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 2, 0);
                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(1, pos->q[0]), plotOptDialog->getMarkSize() * 2, 0);

                // draw solution number if both solutions are shown
                if (ui->btnSolution1->isChecked() && pos1->n > 0 && graphTriple[panel]->toPoint(xl[0], y, p1)) {
                    p1.rx() += plotOptDialog->getMarkSize() + 4;
                    drawLabel(graphTriple[panel], c, p1, QStringLiteral("2"), Graph::Alignment::Left, Graph::Alignment::Center);
                }
            }
            if (ui->btnSolution1->isChecked() && pos1->n > 0) {
                xl[0] = xl[1] = timePosition(time1);
                y = panel == 0 ? pos1->x[0] : (panel == 1 ? pos1->y[0] : pos1->z[0]);

                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 6, 0);
                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 6, 0);
                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 2, 0);
                graphTriple[panel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(0, pos->q[0]), plotOptDialog->getMarkSize() * 2, 0);

                // draw solution number if both solutions are shown
                if (ui->btnSolution2->isChecked() && pos2->n > 0 && graphTriple[panel]->toPoint(xl[0], y, p1)) {
                    p1.rx() += plotOptDialog->getMarkSize() + 4;
                    drawLabel(graphTriple[panel], c, p1, QStringLiteral("1"), Graph::Alignment::Left, Graph::Alignment::Center);
                }
            }

            xl[0] = xl[1] = timePosition(pos->t[0]);
            if (k++ == 0) {  // first time only
                graphTriple[panel]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), 5, 0);

                if (!ui->btnFixHorizontal->isChecked())
                    graphTriple[panel]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 9, 0);
            }
        }

        delete pos;
        delete pos1;
        delete pos2;
    }

    // draw labels
    for (int i = 0; i < 3; i++) {
        if (!btn[i]->isChecked()) continue;
        graphTriple[i]->getExtent(p1, p2);
        p1.rx() += 5;
        p1.ry() += 3;
        drawLabel(graphTriple[i], c, p1, label[i], Graph::Alignment::Left, Graph::Alignment::Top);
    }
}
// draw points and line on solution-plot ------------------------------------
void Plot::drawSolutionPoint(QPainter &c, const TIMEPOS *pos, int level, int style)
{
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3 };
    double *x, *y, *s, xs, ys, *yy;
    int j;

    trace(3, "drawSolutionPoint: level=%d style=%d\n", level, style);

    x = new double[pos->n];

    for (int i = 0; i < pos->n; i++)
        x[i] = timePosition(pos->t[i]);

    for (int panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        y = panel == 0 ? pos->x : (panel == 1 ? pos->y : pos->z);
        s = panel == 0 ? pos->xs : (panel == 1 ? pos->ys : pos->zs);

        if (!level || (plotOptDialog->getPlotStyle() % 2) == 0) // a style with lines
            drawPolyS(graphTriple[panel], c, x, y, pos->n, plotOptDialog->getCColor(3), style);

        // draw errors
        if (level && plotOptDialog->getShowError() != 0 && plotType <= PLOT_SOLA && plotOptDialog->getPlotStyle() < 2) {
            graphTriple[panel]->getScale(xs, ys);

            if (plotOptDialog->getShowError() == 1) {  // bar error style
                for (j = 0; j < pos->n; j++)
                    graphTriple[panel]->drawMark(c, x[j], y[j], Graph::MarkerTypes::VScale, plotOptDialog->getCColor(1), static_cast<int>(SQRT(s[j]) * 2.0 / ys), 0);
            } else {  // getShowError() == 2: dots error style
                yy = new double [pos->n];

                // negative std-dev
                for (j = 0; j < pos->n; j++) yy[j] = y[j] - SQRT(s[j]);
                drawPolyS(graphTriple[panel], c, x, yy, pos->n, plotOptDialog->getCColor(1), 1);

                // position std-dev
                for (j = 0; j < pos->n; j++) yy[j] = y[j] + SQRT(s[j]);
                drawPolyS(graphTriple[panel], c, x, yy, pos->n, plotOptDialog->getCColor(1), 1);

                delete [] yy;
            }
        }

        // draw markers
        if (level && plotOptDialog->getPlotStyle() < 2) {  // a style with markers
            QColor * color = new QColor[pos->n];

            for (j = 0; j < pos->n; j++)
                color[j] = plotOptDialog->getMarkerColor(style, pos->q[j]);
            graphTriple[panel]->drawMarks(c, x, y, color, pos->n, Graph::MarkerTypes::Dot, plotOptDialog->getMarkSize(), 0);

            delete[] color;
        }
    }
    delete [] x;
}
// draw statistics on solution-plot -----------------------------------------
void Plot::drawSolutionStat(QPainter &c, const TIMEPOS *pos, const QString &unit, int p)
{
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3 };
    QPoint p1, p2;
    double ave, std, rms, *y, opos[3];
    int j = 0, k = 0, fonth = (int)(QFontMetrics(plotOptDialog->getFont()).height() * 1.2);
    QString label, s;

    trace(3, "drawSolutionStat: p=%d\n", p);

    if (!plotOptDialog->getShowStats() || pos->n <= 0) return;

    for (int panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        y = panel == 0 ? pos->x : (panel == 1 ? pos->y : pos->z);

        calcStats(y, pos->n, 0.0, ave, std, rms);

        graphTriple[panel]->getExtent(p1, p2);
        p1.rx() = p2.x() - 5;
        p1.ry() += 3 + fonth * (p + (k++ == 0 && p > 0 ? 1 : 0));

        if (j == 0 && p == 0) {
            if (norm(originPosition, 3) > 0.0) {
                ecef2pos(originPosition, opos);
                label = tr("ORI = %1, %L2 m").arg(latLonString(opos, 9) ).arg(opos[2], 0, 'f', 4);
                drawLabel(graphTriple[j], c, p1, label, Graph::Alignment::Right, Graph::Alignment::Top);
                p1.ry() += fonth;
                j++;
            }
        }
        s = tr("AVE = %L1 %2, STD = %L3 %2, RMS = %L4 %2").arg(ave, 0, 'f', 4).arg(unit).arg(std, 0, 'f', 4).arg(rms, 0, 'f', 4);
        drawLabel(graphTriple[panel], c, p1, s, Graph::Alignment::Right, Graph::Alignment::Top);
    }
}
// draw number-of-satellite plot --------------------------------------------
void Plot::drawNsat(QPainter &c, int level)
{
    QString label[] = {
        tr("# of Valid Satellites"),
        tr("Age of Differential"),
        tr("Ratio Factor for AR Validation")
    };
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3 };
    TIMEPOS *ns;
    QPoint p1, p2;
    double xc, yc, y, xl[2], yl[2], off;
    int panel, bottomPanel, k, sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;

    trace(3, "drawNsat: level=%d\n", level);

    // update panel center
    if (ui->btnShowTrack->isChecked() && ui->btnFixHorizontal->isChecked()) {
        ns = solutionToNsat(solutionData + sel, solutionIndex[sel], 0);

        for (panel = 0; panel < 3; panel++) {
            if (ui->btnFixHorizontal->isChecked()) {
                graphTriple[panel]->getLimits(xl, yl);
                off = centX * (xl[1] - xl[0]) / 2.0;
                graphTriple[panel]->getCenter(xc, yc);
                graphTriple[panel]->setCenter(timePosition(ns->t[0]) - off, yc);
            } else {
                graphTriple[panel]->getRight(xc, yc);
                graphTriple[panel]->setRight(timePosition(ns->t[0]), yc);
            }
        }
        delete ns;
    }

    // set axis labels
    bottomPanel = -1;
    for (panel = 0; panel < 3; panel++)
        if (btn[panel]->isChecked()) bottomPanel = panel;
    for (panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        graphTriple[panel]->setLabelUnits("", panel == 1 ? tr(" s") : "" );
        graphTriple[panel]->xLabelPosition = plotOptDialog->getTimeFormat() ? (panel == bottomPanel ? Graph::LabelPosition::Time : Graph::LabelPosition::None) :
                                                 (panel == bottomPanel ? Graph::LabelPosition::Outer : Graph::LabelPosition::Off);
        graphTriple[panel]->week = week;
        graphTriple[panel]->drawAxis(c, plotOptDialog->getShowGridLabel(), plotOptDialog->getShowGridLabel());
    }

    // draw solutions
    if (ui->btnSolution1->isChecked()) {
        ns = solutionToNsat(solutionData, -1, ui->cBQFlag->currentIndex());
        drawSolutionPoint(c, ns, level, 0);
        delete ns;
    }

    if (ui->btnSolution2->isChecked()) {
        ns = solutionToNsat(solutionData + 1, -1, ui->cBQFlag->currentIndex());
        drawSolutionPoint(c, ns, level, 1);
        delete ns;
    }

    // draw current position
    if (ui->btnShowTrack->isChecked() && (ui->btnSolution1->isChecked() || ui->btnSolution2->isChecked())) {
        ns = solutionToNsat(solutionData + sel, solutionIndex[sel], 0);

        for (bottomPanel = k = 0; bottomPanel < 3 && ns->n > 0; bottomPanel++) {
            if (!btn[bottomPanel]->isChecked()) continue;

            y = bottomPanel == 0 ? ns->x[0] : (bottomPanel == 1 ? ns->y[0] : ns->z[0]);
            graphTriple[bottomPanel]->getLimits(xl, yl);
            xl[0] = xl[1] = timePosition(ns->t[0]);

            graphTriple[bottomPanel]->drawPoly(c, xl, yl, 2, plotOptDialog->getCColor(2), 0);
            graphTriple[bottomPanel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 6, 0);
            graphTriple[bottomPanel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 6, 0);
            graphTriple[bottomPanel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 2, 0);
            graphTriple[bottomPanel]->drawMark(c, xl[0], y, Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(sel, ns->q[0]), plotOptDialog->getMarkSize() * 2, 0);

            if (k++ == 0) {  // first time only
                graphTriple[bottomPanel]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), 5, 0);

                if (!ui->btnFixHorizontal->isChecked())
                    graphTriple[bottomPanel]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 9, 0);
            }
        }
        delete ns;
    }

    // draw labels
    for (panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;
        graphTriple[panel]->getExtent(p1, p2);
        p1.rx() += 5;
        p1.ry() += 3;
        drawLabel(graphTriple[panel], c, p1, label[panel], Graph::Alignment::Left, Graph::Alignment::Top);
    }
}
// draw observation-data-plot -----------------------------------------------
void Plot::drawObservation(QPainter &c, int level)
{
    QPoint p1, p2, p;
    QVariant obstype = ui->cBObservationType->currentData();
    gtime_t time;
    obsd_t *obs;
    double xs, ys, xt, xl[2], yl[2], tt[MAXSAT] = { 0 }, xp, xc, yc, yp[MAXSAT] = { 0 };
    int i, j, nSats = 0, sats[MAXSAT] = { 0 }, ind = observationIndex;
    char id[16];

    trace(3, "drawObservation: level=%d\n", level);

    // count used satellites
    for (i = 0; i < observation.n; i++) {
        if (!satelliteSelection[observation.data[i].sat - 1]) continue;
        sats[observation.data[i].sat - 1] = 1;
    }
    for (i = 0; i < MAXSAT; i++)
        if (sats[i]) nSats++;
    
    graphSingle->xLabelPosition = plotOptDialog->getTimeFormat() ? Graph::LabelPosition::Time : Graph::LabelPosition::Outer;
    graphSingle->yLabelPosition = Graph::LabelPosition::Off;
    graphSingle->week = week;

    graphSingle->getLimits(xl, yl);
    yl[0] = 0.5;
    yl[1] = nSats > 0 ? nSats + 0.5 : nSats + 10.5;
    graphSingle->setLimits(xl, yl);
    graphSingle->setTick(0.0, 1.0);

    // update plot center
    if (0 <= ind && ind < nObservation && ui->btnShowTrack->isChecked() && ui->btnFixHorizontal->isChecked()) {
        xp = timePosition(observation.data[indexObservation[ind]].time);
        if (ui->btnFixHorizontal->isChecked()) {
            double xl[2], yl[2], off;
            graphSingle->getLimits(xl, yl);
            off = centX * (xl[1] - xl[0]) / 2.0;
            graphSingle->getCenter(xc, yc);
            graphSingle->setCenter(xp - off, yc);
        } else {
            graphSingle->getRight(xc, yc);
            graphSingle->setRight(xp, yc);
        }
    }

    graphSingle->drawAxis(c, true, true);
    graphSingle->getExtent(p1, p2);

    // draw sat labels
    for (i = 0, j = 0; i < MAXSAT; i++) {
        if (!sats[i]) continue;

        p.setX(p1.x() - 2);
        p.setY(p1.y() + static_cast<int>((p2.y() - p1.y()) * (j + 0.5) / nSats));
        yp[i] = nSats - (j++);
        satno2id(i + 1, id);
        graphSingle->drawText(c, p, id, plotOptDialog->getCColor(2), Graph::Alignment::Right, Graph::Alignment::Center, 0);
    }

    // draw title
    p1.setX(plotOptDialog->getFont().pointSize());
    p1.setY((p1.y() + p2.y()) / 2);
    graphSingle->drawText(c, p1, tr("Satellite No."), plotOptDialog->getCColor(2), Graph::Alignment::Center, Graph::Alignment::Center, 90);

    if (!ui->btnSolution1->isChecked()) return;

    if (level && plotOptDialog->getPlotStyle() <= 2)  // plot style not "none"
        drawObservationEphemeris(c, yp);

    // draw observations
    if (level && plotOptDialog->getPlotStyle() <= 2) {
        graphSingle->getScale(xs, ys);

        for (i = 0; i < observation.n; i++) {
            obs = &observation.data[i];

            QColor col = observationColor(obs, azimuth[i], elevation[i], obstype);
            if (col == Qt::black) continue;

            xt = timePosition(obs->time);
            if (fabs(xt - tt[obs->sat - 1]) / xs > 0.9) {
                graphSingle->drawMark(c, xt, yp[obs->sat - 1], Graph::MarkerTypes::Dot, plotOptDialog->getPlotStyle() < 2 ? col : plotOptDialog->getCColor(3),
                         plotOptDialog->getPlotStyle() < 2 ? plotOptDialog->getMarkSize() : 0, 0);
                tt[obs->sat - 1] = xt;
            }
        }
    }

    // draw slip cycles
    if (level && plotOptDialog->getPlotStyle() <= 2)  // plot style not "none"
        drawObservationSlips(c, yp);

    if (ui->btnShowTrack->isChecked() && 0 <= ind && ind < nObservation) {
        int idx = indexObservation[ind];
        time = observation.data[idx].time;

        // draw vertical line at current position
        graphSingle->getLimits(xl, yl);
        xl[0] = xl[1] = timePosition(observation.data[idx].time);
        graphSingle->drawPoly(c, xl, yl, 2, plotOptDialog->getCColor(2), 0);

        // draw observation for current position
        for (; idx < observation.n && timediff(observation.data[idx].time, time) == 0.0; idx++) {
            obs = &observation.data[idx];

            QColor col = observationColor(obs, azimuth[idx], elevation[idx], obstype);
            if (col == Qt::black) continue;

            graphSingle->drawMark(c, xl[0], yp[obs->sat - 1], Graph::MarkerTypes::Dot, col, plotOptDialog->getMarkSize() * 2 + 2, 0);
        }

        graphSingle->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), 5, 0);
        if (!ui->btnFixHorizontal->isChecked())
            graphSingle->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 9, 0);
    }
}
// generate observation-data-slips ---------------------------------------
void Plot::generateObservationSlips(int *LLI)
{
    QVariant obstype = ui->cBObservationType->currentData();
    double prev_gf[MAXSAT][NFREQ+NEXOBS] = {{0}};
    double freq1, freq2, gf;
    int j, k;

    for (int i = 0; i < observation.n; i++) {
        obsd_t *obs = observation.data + i;

        LLI[i] = 0; // clear LLI

        // check elevation mask(s)
        if (elevation[i] < plotOptDialog->getElevationMask() * D2R || !satelliteSelection[obs->sat - 1]) continue;
        if (plotOptDialog->getElevationMaskEnabled() && elevation[i] < elevationMaskData[(int)(azimuth[i] * R2D + 0.5)]) continue;

        if (plotOptDialog->getShowSlip() == 1) { // LG jump
            if (obstype.isNull()) {  // "ALL" selected
                if ((freq1 = sat2freq(obs->sat, obs->code[0], &navigation)) == 0.0) continue;

                LLI[i] = obs->LLI[0] & 2;
                for (j = 1; j < NFREQ + NEXOBS; j++) {
                    LLI[i] |= obs->LLI[j] & 2;

                    if ((freq2 = sat2freq(obs->sat, obs->code[j], &navigation)) == 0.0) continue;

                    gf = CLIGHT * (obs->L[0] / freq1-obs->L[j] / freq2); // geometry-free
                    if (fabs(prev_gf[obs->sat-1][j] - gf) > THRESLIP) LLI[i] |= 1;

                    prev_gf[obs->sat-1][j] = gf;
                }
            } else {
                if (strcmp(obstype.typeName(), "int") == 0) {  // frequency selected
                    k = obstype.toInt();
                    j = k > 2 ? k - 3 : k - 1;  /* L1,L2,L5,L6 ... */
                } else {  // code selected
                    for (j = 0; j < NFREQ + NEXOBS; j++) {
                        if (!strcmp(code2obs(obs->code[j]), qPrintable(obstype.toString()))) break;
                    }
                    if (j >= NFREQ + NEXOBS) continue;
                }

                LLI[i] = obs->LLI[j] & 2;
                k = (j == 0) ? 1 : 0;  // selected reference for geometry-free solution

                if ((freq1 = sat2freq(obs->sat, obs->code[k], &navigation)) == 0.0) continue;
                if ((freq2 = sat2freq(obs->sat, obs->code[j], &navigation)) == 0.0) continue;

                gf = CLIGHT * (obs->L[k] / freq1-obs->L[j] / freq2);
                if (fabs(prev_gf[obs->sat-1][j] - gf) > THRESLIP) LLI[i] |= 1;

                prev_gf[obs->sat-1][j] = gf;
            }
        } else {  // LLI flags
            if (obstype.isNull()) {  // "ALL" selected
                for (j = 0; j < NFREQ + NEXOBS; j++)
                    LLI[i] |= obs->LLI[j];

            } else {
                if (strcmp(obstype.typeName(), "int") == 0) {  // frequency selected
                    k = obstype.toInt();
                    j = k > 2 ? k - 3 : k - 1;  /* L1,L2,L5,L6 ... */
                } else {  // code selected
                    for (j = 0; j < NFREQ + NEXOBS; j++)
                        if (!strcmp(code2obs(obs->code[j]), qPrintable(obstype.toString()))) break;

                    if (j >= NFREQ + NEXOBS) continue;
                }
                LLI[i] = obs->LLI[j];
            }
        }
    }
}
// draw slip on observation-data-plot ---------------------------------------
void Plot::drawObservationSlips(QPainter &c, double *yp)
{
    trace(3,"drawObservationSlips\n");

    if (observation.n <= 0 || (plotOptDialog->getShowSlip() == 0 && plotOptDialog->getShowHalfC() == 0)) return;

    QPoint ps[2];
    int *LLI = new int [observation.n];

    generateObservationSlips(LLI);

    // draw slips
    for (int i = 0; i < observation.n; i++) {
        if (!LLI[i]) continue;

        obsd_t *obs = observation.data + i;
        if (graphSingle->toPoint(timePosition(obs->time), yp[obs->sat-1], ps[0])) {
            ps[1].setX(ps[0].x());

            ps[1].setY(ps[0].y() + plotOptDialog->getMarkSize() * 3 / 2 + 1);
            ps[0].setY(ps[0].y() - plotOptDialog->getMarkSize() * 3 / 2);

            if (plotOptDialog->getShowHalfC() && (LLI[i] & 2))
                graphSingle->drawPoly(c, ps, 2, plotOptDialog->getMarkerColor(0, 0), 0);
            if (plotOptDialog->getShowSlip() && (LLI[i] & 1))
                graphSingle->drawPoly(c, ps, 2, plotOptDialog->getMarkerColor(0, 5), 0);
        }
    }
    delete [] LLI;
}
// draw ephemeris on observation-data-plot ----------------------------------
void Plot::drawObservationEphemeris(QPainter &c, double *yp)
{
    QPoint ps[3];
    int sat, i, k, in, svh, offset[MAXSAT] = { 0 };

    trace(3, "drawObservationEphemeris\n");

    if (!plotOptDialog->getShowEphemeris()) return;

    for (sat = 0; sat < MAXSAT; sat++) {
        if (!satelliteSelection[sat]) continue;

        // GPS/Galileo/... ephemeris
        for (i = 0; i < navigation.n; i++) {
            if (navigation.eph[i].sat != sat + 1) continue;

            graphSingle->toPoint(timePosition(navigation.eph[i].ttr), yp[sat], ps[0]);  // start position
            ps[1] = ps[0];
            in = graphSingle->toPoint(timePosition(navigation.eph[i].toe), yp[sat], ps[2]);  // end position

            offset[navigation.eph[i].sat - 1] = offset[navigation.eph[i].sat - 1] ? 0 : 3;

            for (k = 0; k < 3; k++)
                ps[k].ry() += plotOptDialog->getMarkSize() + 2 + offset[navigation.eph[i].sat - 1];
            ps[0].ry() -= 2;

            svh = navigation.eph[i].svh;
            if (satsys(sat + 1, NULL) == SYS_QZS)
                svh &= 0xFE; /* mask QZS LEX health */

            graphSingle->drawPoly(c, ps, 3, svh ? plotOptDialog->getMarkerColor(0, 5) : plotOptDialog->getCColor(1), 0);

            if (in) graphSingle->drawMark(c, ps[2], Graph::MarkerTypes::Dot, svh ? plotOptDialog->getMarkerColor(0, 5) : plotOptDialog->getCColor(1), svh ? 4 : 3, 0);
        }
        // Glonass ephemeris
        for (i = 0; i < navigation.ng; i++) {
            if (navigation.geph[i].sat != sat + 1) continue;

            graphSingle->toPoint(timePosition(navigation.geph[i].tof), yp[sat], ps[0]);
            ps[1] = ps[0];
            in = graphSingle->toPoint(timePosition(navigation.geph[i].toe), yp[sat], ps[2]);

            offset[navigation.geph[i].sat - 1] = offset[navigation.geph[i].sat - 1] ? 0 : 3;

            for (k = 0; k < 3; k++)
                ps[k].ry() += plotOptDialog->getMarkSize() + 2 + offset[navigation.geph[i].sat - 1];
            ps[0].ry() -= 2;

            svh = navigation.geph[i].svh;
            graphSingle->drawPoly(c, ps, 3, svh ? plotOptDialog->getMarkerColor(0, 5) : plotOptDialog->getCColor(1), 0);

            if (in) graphSingle->drawMark(c, ps[2], Graph::MarkerTypes::Dot, svh ? plotOptDialog->getMarkerColor(0, 5) : plotOptDialog->getCColor(1), svh ? 4 : 3, 0);
        }
        // SBAS ephemeris
        for (i = 0; i < navigation.ns; i++) {
            if (navigation.seph[i].sat != sat + 1) continue;

            graphSingle->toPoint(timePosition(navigation.seph[i].tof), yp[sat], ps[0]);
            ps[1] = ps[0];
            in = graphSingle->toPoint(timePosition(navigation.seph[i].t0), yp[sat], ps[2]);

            offset[navigation.seph[i].sat - 1] = offset[navigation.seph[i].sat - 1] ? 0 : 3;

            for (k = 0; k < 3; k++)
                ps[k].ry() += plotOptDialog->getMarkSize() + 2 + offset[navigation.seph[i].sat - 1];
            ps[0].ry() -= 2;

            svh = navigation.seph[i].svh;
            graphSingle->drawPoly(c, ps, 3, svh ? plotOptDialog->getMarkerColor(0, 5) : plotOptDialog->getCColor(1), 0);

            if (in) graphSingle->drawMark(c, ps[2], Graph::MarkerTypes::Dot, svh ? plotOptDialog->getMarkerColor(0, 5) : plotOptDialog->getCColor(1), svh ? 4 : 3, 0);
        }
    }
}
// draw sky-image on sky-plot -----------------------------------------------
void Plot::drawSkyImage(QPainter &c, int level)
{
    QPoint topLeft, bottomRight;
    double xl[2], yl[2], radius, scale, mx[190], my[190];

    trace(3, "drawSkyImage: level=%d\n", level);

    if (skyImgDialog->getSkySize()[0] <= 0 || skyImgDialog->getSkySize()[1] <= 0) return;

    graphSky->getLimits(xl, yl);
    radius = qMin(xl[1] - xl[0], yl[1] - yl[0]) * 0.45;
    scale = radius * skyImageResampled.width() / 2.0 / skyImgDialog->getSkyScaleR();

    graphSky->toPoint(-scale, scale, topLeft);
    graphSky->toPoint(scale, -scale, bottomRight);
    c.drawImage(QRect(topLeft, bottomRight), skyImageResampled);

    if (skyImgDialog->getSkyElevationMask()) {
        int n = 0;

        mx[n] = 0.0; my[n++] = yl[1];
        for (int i = 0; i <= 180; i++) {
            mx[n] = radius * sin(i * 2.0 * D2R);
            my[n++] = radius * cos(i * 2.0 * D2R);
        }
        mx[n] = 0.0;   my[n++] = yl[1];
        mx[n] = xl[0]; my[n++] = yl[1];
        mx[n] = xl[0]; my[n++] = yl[0];
        mx[n] = xl[1]; my[n++] = yl[0];
        mx[n] = xl[1]; my[n++] = yl[1];
        graphSky->drawPatch(c, mx, my, n, plotOptDialog->getCColor(0), plotOptDialog->getCColor(0), 0);
    }
}
// draw sky-plot ------------------------------------------------------------
void Plot::drawSky(QPainter &c, int level)
{
    QPoint p1, p2;
    QString s;
    QVariant obstype = ui->cBObservationType->currentData();
    obsd_t *obs;
    gtime_t prevTime[MAXSAT] = {{0, 0}};
    double prevPoint[MAXSAT][2] = {{0}}, p0[MAXSAT][2] = {{0}};
    double x, y, xp, yp, xs, ys, dt, dx, dy, xl[2], yl[2], radius;
    int i, j, freq, ind = observationIndex;
    int hh = (int)(QFontMetrics(plotOptDialog->getFont()).height() * 1.5);
    char satId[16];

    trace(3, "drawSky: level=%d\n", level);

    graphSky->getLimits(xl, yl);
    radius = qMin(xl[1] - xl[0], yl[1] - yl[0]) * 0.45;

    if (ui->btnShowImage->isChecked())
        drawSkyImage(c, level);

    if (ui->btnShowSkyplot->isChecked())
        graphSky->drawSkyPlot(c, 0.0, 0.0, plotOptDialog->getCColor(1), plotOptDialog->getCColor(2), plotOptDialog->getCColor(0), radius * 2.0);

    if (!ui->btnSolution1->isChecked()) return;

    graphSky->getScale(xs, ys);

    if (plotOptDialog->getPlotStyle() <= 2) {  // plot style not "none"
        for (i = 0; i < observation.n; i++) {
            obs = &observation.data[i];
            if (satelliteMask[obs->sat - 1] || !satelliteSelection[obs->sat - 1] || elevation[i] <= 0.0) continue;

            QColor col = observationColor(obs, azimuth[i], elevation[i], obstype);
            if (col == Qt::black) continue;

            x = radius * sin(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            y = radius * cos(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            xp = prevPoint[obs->sat - 1][0];
            yp = prevPoint[obs->sat - 1][1];

            if ((x - xp) * (x - xp) + (y - yp) * (y - yp) >= xs * xs) {
                int siz = plotOptDialog->getPlotStyle() < 2 ? plotOptDialog->getMarkSize() : 1;
                graphSky->drawMark(c, x, y, Graph::MarkerTypes::Dot, plotOptDialog->getPlotStyle() < 2 ? col : plotOptDialog->getCColor(3), siz, 0);
                prevPoint[obs->sat - 1][0] = x;
                prevPoint[obs->sat - 1][1] = y;
            }
            if (xp == 0.0 && yp == 0.0) {  // save first point
                p0[obs->sat - 1][0] = x;
                p0[obs->sat - 1][1] = y;
            }
        }
    }

    if ((plotOptDialog->getPlotStyle() == 0 || plotOptDialog->getPlotStyle() == 2) && !ui->btnShowTrack->isChecked()) {  // plot style with lines
        for (int sat = 0; sat < MAXSAT; sat++) {
            if (p0[sat][0] != 0.0 || p0[sat][1] != 0.0) {
                QPoint pnt;
                if (graphSky->toPoint(p0[sat][0], p0[sat][1], pnt)) {
                    satno2id(sat + 1, satId);
                    drawLabel(graphSky, c, pnt, QString(satId), Graph::Alignment::Left, Graph::Alignment::Center);
                }
            }
        }
    }
    if (!level) return;

    if (plotOptDialog->getShowSlip() && plotOptDialog->getPlotStyle() <= 2) {  // plot style not "none"
        int *LLI = new int [observation.n];

        generateObservationSlips(LLI);

        for (i = 0; i < observation.n; i++) {
            if (!(LLI[i] & 1)) continue;
            obs = observation.data + i;

            x = radius * sin(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            y = radius * cos(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            dx = x - prevPoint[obs->sat - 1][0];
            dy = y - prevPoint[obs->sat - 1][1];

            dt = timediff(obs->time, prevTime[obs->sat - 1]);

            prevTime[obs->sat - 1] = obs->time;
            prevPoint[obs->sat - 1][0] = x;
            prevPoint[obs->sat - 1][1] = y;

            if (fabs(dt) > 300.0) continue;  // don't connect observations by a line if the time difference is too large

            graphSky->drawMark(c, x, y, Graph::MarkerTypes::Line, plotOptDialog->getMarkerColor(0, 5), plotOptDialog->getMarkSize() * 3 + 2, static_cast<int>(ATAN2(dy, dx) * R2D + 90));
        }
        delete [] LLI;
    }

    if (plotOptDialog->getElevationMaskEnabled()) {  // draw elevation mask
        double *x = new double [361];
        double *y = new double [361];

        for (i = 0; i <= 360; i++) {
            x[i] = radius * sin(i * D2R) * (1.0 - 2.0 * elevationMaskData[i] / PI);
            y[i] = radius * cos(i * D2R) * (1.0 - 2.0 * elevationMaskData[i] / PI);
        }
        QPen pen = c.pen(); pen.setWidth(2); c.setPen(pen);  // set prn width
        graphSky->drawPoly(c, x, y, 361, COL_ELMASK, 0);
        pen.setWidth(1); c.setPen(pen);

        delete [] x;
        delete [] y;
    }

    // draw current observation
    if (ui->btnShowTrack->isChecked() && 0 <= ind && ind < nObservation) {
        int fontsize = plotOptDialog->getFont().pointSize();

        for (i = indexObservation[ind]; i < observation.n && i < indexObservation[ind + 1]; i++) {
            obs = &observation.data[i];

            if (satelliteMask[obs->sat - 1] || !satelliteSelection[obs->sat - 1] || elevation[i] <= 0.0) continue;

            QColor col = observationColor(obs, azimuth[i], elevation[i], obstype);
            if (col == Qt::black) continue;

            x = radius * sin(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            y = radius * cos(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);

            satno2id(obs->sat, satId);

            graphSky->drawMark(c, x, y, Graph::MarkerTypes::Dot, col, fontsize * 2 + 5, 0);
            graphSky->drawMark(c, x, y, Graph::MarkerTypes::Circle, col == Qt::black ? plotOptDialog->getMarkerColor(0, 0) : plotOptDialog->getCColor(2),
                             fontsize * 2 + 5, 0);
            graphSky->drawText(c, x, y, QString(satId), plotOptDialog->getCColor(0), 0, Graph::Alignment::Center, Graph::Alignment::Center);
        }
    }

    graphSky->getExtent(p1, p2);
    p1.rx() += 10; p1.ry() += 8;
    p2.rx() -= 10; p2.ry() = p1.y();

    // show station data
    if (plotOptDialog->getShowStats() && !simulatedObservation) {
        s = tr("MARKER: %1 %2").arg(station.name, station.marker);
        drawLabel(graphSky, c, p1, s, Graph::Alignment::Left, Graph::Alignment::Top); p1.ry() += hh;
        s = tr("REC: %1 %2 %3").arg(station.rectype, station.recver, station.recsno);
        drawLabel(graphSky, c, p1, s, Graph::Alignment::Left, Graph::Alignment::Top); p1.ry() += hh;
        s = tr("ANT: %1 %2").arg(station.antdes, station.antsno);
        drawLabel(graphSky, c, p1, s, Graph::Alignment::Left, Graph::Alignment::Top); p1.ry() += hh;
    }

    // show statistics
    if (plotOptDialog->getShowStats() && ui->btnShowTrack->isChecked() && 0 <= ind && ind < nObservation && !simulatedObservation) {

        if (obstype.isNull()) {  // "ALL" selected
            s = QString::asprintf("%3s: %*s %*s%*s %*s", "SAT", NFREQ, "PR", NFREQ, "CP",
                         NFREQ*3, "CN0", NFREQ, "LLI");
        } else {
            s = tr("SAT: SIG  OBS   CN0 LLI");
        }
        QFont monoFont("Monospace");
        monoFont.setStyleHint(QFont::TypeWriter);
        graphSky->drawText(c, p2, s, Qt::black, Graph::Alignment::Right, Graph::Alignment::Top, 0, monoFont);

        p2.ry() += 3;

        for (i = indexObservation[ind]; i < observation.n && i < indexObservation[ind + 1]; i++) {
            obs = &observation.data[i];
            if (satelliteMask[obs->sat - 1] || !satelliteSelection[obs->sat - 1]) continue;
            if (plotOptDialog->getHideLowSatellites() && elevation[i] < plotOptDialog->getElevationMask() * D2R) continue;
            if (plotOptDialog->getHideLowSatellites() && plotOptDialog->getElevationMaskEnabled() && elevation[i] < elevationMaskData[static_cast<int>(azimuth[i] * R2D + 0.5)]) continue;

            satno2id(obs->sat, satId);
            s = QStringLiteral("%1: ").arg(satId, 3, QChar('-'));

            if (obstype.isNull()) {  // "ALL"
                for (j = 0; j < NFREQ; j++)
                    s += obs->P[j] == 0.0 ? "-" : "C";
                s += " ";

                for (j = 0; j < NFREQ; j++)
                    s += obs->L[j] == 0.0 ? "-" : "L";
                s += " ";

                // SNR
                for (j = 0; j < NFREQ; j++) {
                    if (obs->P[j] == 0.0 && obs->L[j] == 0.0)
                        s += "-- ";
                    else
                        s += QStringLiteral("%1 ").arg(obs->SNR[j] * SNR_UNIT, 2, 'f', 0, QChar('0'));
                }

                // LLI
                for (j = 0; j < NFREQ; j++) {
                    if (obs->L[j] == 0.0) s += "-";
                    else s += QString::number(obs->LLI[j]);
                }
            } else if (strcmp(obstype.typeName(), "int") == 0) {  // frequency
                freq = obstype.toInt();
                freq -= freq > 2 ? 2 : 0;  /* L1,L2,L5,L6 ... */

                if (!obs->code[freq-1]) continue;

                s += QStringLiteral("%1  %2 %3 %4  ").arg(code2obs(obs->code[freq - 1]), 3).arg(
                              obs->P[freq - 1] == 0.0 ? "-" : "C",
                              obs->L[freq - 1] == 0.0 ? "-" : "L",
                              obs->D[freq - 1] == 0.0 ? "-" : "D");

                // SNR
                if (obs->P[freq - 1] == 0.0 && obs->L[freq - 1] == 0.0)
                    s += "---- ";
                else
                    s += QStringLiteral("%1 ").arg(obs->SNR[freq - 1] * SNR_UNIT, 4, 'f', 1);

                // LLI
                if (obs->L[freq-1] == 0.0)
                    s += " -";
                else
                    s += QString("%1").arg(obs->LLI[freq - 1], 2);

            } else {  // code
                for (j = 0; j < NFREQ + NEXOBS; j++)
                    if (!strcmp(code2obs(obs->code[j]), qPrintable(obstype.toString()))) break;
                if (j >= NFREQ + NEXOBS) continue;

                s += QStringLiteral("%1  %2 %3 %4 ").arg(code2obs(obs->code[j]), 3).arg(
                                 obs->P[j] == 0.0 ? "-" : "C",
                                 obs->L[j] == 0.0 ? "-" : "L",
                                 obs->D[j] == 0.0 ? "-" : "D");

                // SNR
                if (obs->P[j] == 0.0 && obs->L[j] == 0.0)
                    s += "---- ";
                else
                    s += QStringLiteral("%1 ").arg(obs->SNR[j] * SNR_UNIT, 4, 'f', 1);

                // LLI
                if (obs->L[j] == 0.0)
                    s += " -";
                else
                    s += QString("%1").arg(obs->LLI[j], 2);
            }

            QColor col = observationColor(obs, azimuth[i], elevation[i], obstype);
            p2.ry() += hh;

            graphSky->drawText(c, p2, s, col == Qt::black ? plotOptDialog->getMarkerColor(0, 0) : col, Graph::Alignment::Right, Graph::Alignment::Top, 0, monoFont);
        }
    }

    if (navigation.n <= 0 && navigation.ng <= 0 && !simulatedObservation) {  // indicate if no navigation data is available
        graphSky->getExtent(p1, p2);
        p2.rx() -= 10;
        p2.ry() -= 3;
        drawLabel(graphSky, c, p2, tr("No navigation data"), Graph::Alignment::Right, Graph::Alignment::Bottom);
    }
}
// draw DOP and number-of-satellite plot ------------------------------------
void Plot::drawDop(QPainter &c, int level)
{
    QString label;
    QPoint p1, p2;
    double xp, xc, yc, xl[2], yl[2], azel[MAXSAT * 2], *dop, *x, *y;
    int i, j, *ns, n = 0;
    int ind = observationIndex, doptype = ui->cBDopType->currentIndex();

    trace(3, "drawDop: level=%d\n", level);
    
    graphSingle->xLabelPosition = plotOptDialog->getTimeFormat() ? Graph::LabelPosition::Time : Graph::LabelPosition::Outer;
    graphSingle->yLabelPosition = Graph::LabelPosition::Outer;
    graphSingle->week = week;

    // adjest plot limits
    graphSingle->getLimits(xl, yl);
    yl[0] = 0.0;
    yl[1] = plotOptDialog->getMaxDop();
    graphSingle->setLimits(xl, yl);
    graphSingle->setTick(0.0, 0.0);

    // update plot center to current position
    if (0 <= ind && ind < nObservation && ui->btnShowTrack->isChecked() && ui->btnFixHorizontal->isChecked()) {
        double xl[2], yl[2], offset;
        graphSingle->getLimits(xl, yl);
        offset = centX * (xl[1] - xl[0]) / 2.0;
        xp = timePosition(observation.data[indexObservation[ind]].time);
        graphSingle->getCenter(xc, yc);
        graphSingle->setCenter(xp - offset, yc);
    }

    graphSingle->drawAxis(c, true, true);

    // draw title
    graphSingle->getExtent(p1, p2);
    p1.setX((int)(QFontMetrics(plotOptDialog->getFont()).height()));
    p1.setY((p1.y() + p2.y()) / 2);    
    if (doptype == 0)  // ALL
        label = tr("# of Satellites / DOP (El>=%1°)").arg(plotOptDialog->getElevationMask(), 0, 'f', 0);
    else if (doptype == 1)  // NSAT
        label = tr("# of Satellites (El>=%1°)").arg(plotOptDialog->getElevationMask(), 0, 'f', 0);
    else
        label = tr("DOP (El>=%1°)").arg(plotOptDialog->getElevationMask(), 0, 'f', 0);
    graphSingle->drawText(c, p1, label, plotOptDialog->getCColor(2), Graph::Alignment::Center, Graph::Alignment::Center, 90);

    if (!ui->btnSolution1->isChecked()) return;

    x = new double[nObservation];
    y = new double[nObservation];
    dop = new double[nObservation * 4];
    ns = new int[nObservation];

    // calculate DOP
    for (i = 0; i < nObservation; i++) {
        ns[n] = 0;
        for (j = indexObservation[i]; j < observation.n && j < indexObservation[i + 1]; j++) {
            if (satelliteMask[observation.data[j].sat - 1] || !satelliteSelection[observation.data[j].sat - 1]) continue;
            if (elevation[j] < plotOptDialog->getElevationMask() * D2R) continue;
            if (plotOptDialog->getElevationMaskEnabled() && elevation[j] < elevationMaskData[static_cast<int>(azimuth[j] * R2D + 0.5)]) continue;

            azel[ns[n] * 2] = azimuth[j];
            azel[ns[n] * 2 + 1] = elevation[j];
            ns[n]++;
        }
        dops(ns[n], azel, plotOptDialog->getElevationMask() * D2R, dop + n * 4);
        x[n++] = timePosition(observation.data[indexObservation[i]].time);
    }

    for (i = 0; i < 4; i++) { // for all DOP type (GDOP, PDOP, HDOP, VDOP)
        if (doptype != 0 && doptype != i + 2) continue;

        for (j = 0; j < n; j++) y[j] = dop[i + j * 4] * 10;

        if ((plotOptDialog->getPlotStyle() % 2) == 0)  // line style
            drawPolyS(graphSingle, c, x, y, n, plotOptDialog->getCColor(3), 0);

        if (level && plotOptDialog->getPlotStyle() < 2) {  // marker style
            for (j = 0; j < n; j++) {
                if (y[j] == 0.0) continue;
                graphSingle->drawMark(c, x[j], y[j], Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(0, i + 2), plotOptDialog->getMarkSize(), 0);
            }
        }
    }

    if (doptype == 0 || doptype == 1) {  // ALL or NSAT
        for (i = 0; i < n; i++) y[i] = ns[i];

        if ((plotOptDialog->getPlotStyle() % 2) == 0)  // line style
            drawPolyS(graphSingle, c, x, y, n, plotOptDialog->getCColor(3), 1);

        if (level && plotOptDialog->getPlotStyle() < 2) {  // marker style
            for (i = 0; i < n; i++)
                graphSingle->drawMark(c, x[i], y[i], Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(0, 1), plotOptDialog->getMarkSize(), 0);
        }
    }

    // draw currently selected data
    if (ui->btnShowTrack->isChecked() && 0 <= ind && ind < nObservation) {
        graphSingle->getLimits(xl, yl);
        xl[0] = xl[1] = timePosition(observation.data[indexObservation[ind]].time);

        graphSingle->drawPoly(c, xl, yl, 2, plotOptDialog->getCColor(2), 0);  // vertical line at current position

        ns[0] = 0;
        for (i = indexObservation[ind]; i < observation.n && i < indexObservation[ind + 1]; i++) {
            if (satelliteMask[observation.data[i].sat - 1] || !satelliteSelection[observation.data[i].sat - 1]) continue;
            if (elevation[i] < plotOptDialog->getElevationMask() * D2R) continue;
            if (plotOptDialog->getElevationMaskEnabled() && elevation[i] < elevationMaskData[static_cast<int>(azimuth[i] * R2D + 0.5)]) continue;

            azel[ns[0] * 2] = azimuth[i];
            azel[ns[0] * 2 + 1] = elevation[i];
            ns[0]++;
        }
        dops(ns[0], azel, plotOptDialog->getElevationMask() * D2R, dop);

        for (i = 0; i < 4; i++) { // for all DOP type (GDOP, PDOP, HDOP, VDOP)
            if ((doptype != 0 && doptype != i + 2) || dop[i] <= 0.0) continue;

            graphSingle->drawMark(c, xl[0], dop[i]*10.0, Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(0, i + 2), plotOptDialog->getMarkSize() * 2 + 2, 0);
        }

        if (doptype == 0 || doptype == 1)  // ALL or NSAT
            graphSingle->drawMark(c, xl[0], ns[0], Graph::MarkerTypes::Dot, plotOptDialog->getMarkerColor(0, 1), plotOptDialog->getMarkSize() * 2 + 2, 0);

        graphSingle->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), 5, 0);
        if (!ui->btnFixHorizontal->isChecked())
            graphSingle->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 9, 0);
    } else {
        drawDopStat(c, dop, ns, n);
    }

    if (navigation.n <= 0 && navigation.ng <= 0 && (doptype == 0 || doptype >= 2) && !simulatedObservation) {  // indicate if no navigation data is available
        graphSingle->getExtent(p1, p2);
        p2.rx() -= 10;
        p2.ry() -= 3;
        drawLabel(graphSingle, c, p2, tr("No navigation data"), Graph::Alignment::Right, Graph::Alignment::Bottom);
    }

    delete [] x;
    delete [] y;
    delete [] dop;
    delete [] ns;
}
// draw statistics on DOP and number-of-satellite plot ----------------------
void Plot::drawDopStat(QPainter &c, double *dop, int *ns, int n)
{
    QString s0[MAXOBS + 2], s1[MAXOBS + 2], s2[MAXOBS + 2];
    double ave[4] = {0};
    int nsat[MAXOBS] = {0}, ndop[4] = {0}, m = 0;

    trace(3, "drawDopStat: n=%d\n", n);

    if (!plotOptDialog->getShowStats()) return;

    for (int i = 0; i < n; i++) {
        nsat[ns[i]]++;
    }

    // calculate average DOP
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < n; j++) {
            if (dop[i + j * 4] <= 0.0 || dop[i + j * 4] > plotOptDialog->getMaxDop()) continue;
            ave[i] += dop[i + j * 4];
            ndop[i]++;
        }
        if (ndop[i] > 0) ave[i] /= ndop[i];
    }

    if (ui->cBDopType->currentIndex() == 0 || ui->cBDopType->currentIndex() >= 2) {  // DopType != NSAT
        s2[m++] = tr("AVE = GDOP: %L1, PDOP: %L2, HDOP: %L3, VDOP: %L4")
              .arg(ave[0], 4, 'f', 1).arg(ave[1], 4, 'f', 1).arg(ave[2], 4, 'f', 1).arg(ave[3], 4, 'f', 1);
        s2[m++] = tr("NDOP = %1 (%L2%), %3 (%L4%), %5 (%L6%), %7 (%L8%)")
              .arg(ndop[0]).arg(n > 0 ? ndop[0] * 100.0 / n : 0.0, 4, 'f', 1)
              .arg(ndop[1]).arg(n > 0 ? ndop[1] * 100.0 / n : 0.0, 4, 'f', 1)
              .arg(ndop[2]).arg(n > 0 ? ndop[2] * 100.0 / n : 0.0, 4, 'f', 1)
              .arg(ndop[3]).arg(n > 0 ? ndop[3] * 100.0 / n : 0.0, 4, 'f', 1);
    }
    if (ui->cBDopType->currentIndex() <= 1) {  // DopType == NSAT (or ALL)
        for (int i = 0, j = 0; i < MAXOBS; i++) {
            if (nsat[i] <= 0) continue;

            s0[m] = QStringLiteral("%1%2: ").arg(j++ == 0 ? QStringLiteral("NSAT = ") : QStringLiteral("")).arg(i, 2);
            s1[m] = QStringLiteral("%1").arg(nsat[i], 7);
            s2[m++] = QStringLiteral("(%1%)").arg(nsat[i] * 100.0 / n, 4, 'f', 1);
        }
    }
    QPoint p1, p2, p3;
    int fonth = (int)(QFontMetrics(plotOptDialog->getFont()).height() * 1.2);

    // calculate text position on the right side of the plot
    graphSingle->getExtent(p1, p2);
    p1.setX(p2.x() - 10);
    p1.ry() += 8;
    p2 = p1; p2.rx() -= fonth * 3;
    p3 = p2; p3.rx() -= fonth * 4;

    for (int i = 0; i < m; i++) {
        drawLabel(graphSingle, c, p3, s0[i], Graph::Alignment::Right, Graph::Alignment::Top);
        drawLabel(graphSingle, c, p2, s1[i], Graph::Alignment::Right, Graph::Alignment::Top);
        drawLabel(graphSingle, c, p1, s2[i], Graph::Alignment::Right, Graph::Alignment::Top);
        p1.ry() += fonth;
        p2.ry() += fonth;
        p3.ry() += fonth;
    }
}
// draw SNR, MP and elevation-plot ---------------------------------------------
void Plot::drawSnr(QPainter &c, int level)
{
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3};
    const QString label[] = {tr("SNR"), tr("Multipath"), tr("Elevation")};
    static const QString unit[] = { "dBHz", "m", "°"};
    gtime_t time_selected = {0, 0};
    int idx;

    trace(3, "drawSnr: level=%d\n", level);

    if (0 <= observationIndex && observationIndex < nObservation && ui->btnShowTrack->isChecked())
        time_selected = observation.data[indexObservation[observationIndex]].time;

    // update plot center
    if (0 <= observationIndex && observationIndex < nObservation && ui->btnShowTrack->isChecked() && ui->btnFixHorizontal->isChecked()) {
        double xc, yc, xp = timePosition(time_selected);
        double xl[2], yl[2];

        graphTriple[0]->getLimits(xl, yl);
        xp -= centX * (xl[1] - xl[0]) / 2.0;
        for (int panel=0; panel < 3; panel++) {
            graphTriple[panel]->getCenter(xc,yc);
            graphTriple[panel]->setCenter(xp,yc);
        }
    }

    // draw axes
    int bottomPanel = 0;
    for (int panel = 0; panel < 3; panel++)
        if (btn[panel]->isChecked()) bottomPanel = panel;

    for (int panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        graphTriple[panel]->setLabelUnits("", " " + unit[panel]);
        graphTriple[panel]->xLabelPosition = plotOptDialog->getTimeFormat() ? (panel == bottomPanel ? Graph::LabelPosition::Time : Graph::LabelPosition::None) :
                                        (panel == bottomPanel ? Graph::LabelPosition::Outer : Graph::LabelPosition::Off);
        graphTriple[panel]->week = week;
        graphTriple[panel]->drawAxis(c, plotOptDialog->getShowGridLabel(), plotOptDialog->getShowGridLabel());
    }

    if (nObservation > 0 && ui->btnSolution1->isChecked()) {
        QVariant obstype = ui->cBObservationTypeSNR->currentData();
        double *x = new double[nObservation];
        double *y = new double[nObservation];
        QColor *col = new QColor[nObservation];

        for (int panel = 0, l = 0; panel < 3; panel++) {
            QColor col_selected[MAXSAT];
            double y_selected[MAXSAT], ave = 0.0, rms = 0.0;
            int np = 0, nrms = 0;
            int n;

            if (!btn[panel]->isChecked()) continue;

            for (uint8_t sat = 1, n_selected = 0; sat <= MAXSAT; sat++) {
                if (satelliteMask[sat - 1] || !satelliteSelection[sat - 1]) continue;

                for (int j = n = 0; j < observation.n; j++) {
                    obsd_t *obs = observation.data + j;

                    if (obs->sat != sat) continue;

                    // find array index corresponding to the selected data
                    if (strcmp(obstype.typeName(), "int") == 0) {
                        int freq = obstype.toInt();
                        idx = freq > 2 ? freq - 3 : freq - 1;
                    } else {
                        for (idx = 0; idx < NFREQ + NEXOBS; idx++) {
                            if (!strcmp(code2obs(obs->code[idx]), qPrintable(obstype.toString()))) break;
                        }
                        if (idx >= NFREQ + NEXOBS) continue;
                    }
                    if (obs->SNR[idx] * SNR_UNIT <= 0.0) continue;  // skip negative SNR

                    // calculate position
                    x[n] = timePosition(obs->time);
                    if (panel == 0) {  // SNR
                        y[n] = obs->SNR[idx] * SNR_UNIT;
                        col[n] = plotOptDialog->getMarkerColor(0, 4);
                    } else if (panel == 1) {  // multipath
                        if (!multipath[idx] || multipath[idx][j] == 0.0) continue;

                        y[n] = multipath[idx][j];
                        col[n] = plotOptDialog->getMarkerColor(0, 4);
                    } else {  // elevation
                        y[n] = elevation[j] * R2D;
                        if (simulatedObservation)
                            col[n] = sysColor(obs->sat);
                        else
                            col[n] = snrColor(obs->SNR[idx] * SNR_UNIT);

                        if (elevation[j] > 0.0 && elevation[j] < plotOptDialog->getElevationMask() * D2R) col[n] = plotOptDialog->getMarkerColor(0, 0);
                    }
                    // save values and colors at currently selected position
                    if (timediff(time_selected, obs->time) == 0.0 && n_selected < MAXSAT) {
                        y_selected[n_selected] = y[n];
                        col_selected[n_selected++] = col[n];
                    }
                    if (n < nObservation) n++;
                }

                if (!level || !(plotOptDialog->getPlotStyle() % 2)) { // line style
                    for (int j = 0, k = 0; j < n; j = k) {
                        for (k = j + 1; k < n; k++)
                            if (fabs(y[k - 1] - y[k]) > 30.0) break; // interrupt line segment if y difference is too large
                        drawPolyS(graphTriple[panel], c, x + j, y + j, k - j, plotOptDialog->getCColor(3), 0);
                    }
                }
                if (level && plotOptDialog->getPlotStyle() < 2) {  // marker style
                    for (int j  = 0; j < n; j++) {
                        if (panel != 1 && y[j] <= 0.0) continue; // don't draw SNR or elevation below 0
                        graphTriple[panel]->drawMark(c, x[j], y[j], Graph::MarkerTypes::Dot, col[j], plotOptDialog->getMarkSize(), 0);
                    }
                }
                // calculate average and RMS
                for (int j = 0; j < n; j++) {
                    if (y[j] == 0.0) continue;
                    ave += y[j];
                    rms += SQR(y[j]);
                    nrms++;
                }
            }

            // draw statistics for multipath panel
            if (level && panel == 1 && nrms > 0 && plotOptDialog->getShowStats() && !ui->btnShowTrack->isChecked()) {
                QPoint p1, p2;
                ave = ave / nrms;
                rms = SQRT(rms / nrms);

                graphTriple[panel]->getExtent(p1, p2);
                p1.rx() = p2.x() - 8;
                p1.ry() += 3;
                drawLabel(graphTriple[panel], c, p1, tr("AVE = %L1 m, RMS = %L2 m").arg(ave, 0, 'f', 4).arg(rms, 0, 'f', 4),
                          Graph::Alignment::Right, Graph::Alignment::Top);
            }

            // highlight current position
            if (ui->btnShowTrack->isChecked() && 0 <= observationIndex && observationIndex < nObservation && ui->btnSolution1->isChecked()) {
                if (!btn[panel]->isChecked()) continue;

                QPoint p1, p2;
                double xl[2], yl[2];

                // draw vertical line at selected time
                graphTriple[panel]->getLimits(xl, yl);
                xl[0] = xl[1] = timePosition(time_selected);
                graphTriple[panel]->drawPoly(c, xl, yl, 2, plotOptDialog->getCColor(2), 0);

                if (l++ == 0) {  // indicate current position on x axis
                    graphTriple[panel]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), 5, 0);

                    if (!ui->btnFixHorizontal->isChecked())
                        graphTriple[panel]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 9, 0);
                }

                // highlight current position
                for (int k = 0; k < np; k++) {
                    if (panel != 1 && y_selected[k] <= 0.0) continue;
                    graphTriple[panel]->drawMark(c, xl[0], y_selected[k], Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 4, 0);
                    graphTriple[panel]->drawMark(c, xl[0], y_selected[k], Graph::MarkerTypes::Dot, col_selected[k], plotOptDialog->getMarkSize() * 2 + 2, 0);
                }

                if (np <= 0 || np > 1 || (panel != 1 && y_selected[0] <= 0.0)) continue;

                // draw value of 1st curve at current position
                graphTriple[panel]->getExtent(p1, p2);
                p1.rx() = p2.x() - 8;
                p1.ry() += 3;
                drawLabel(graphTriple[panel], c, p1, QStringLiteral("%1 %2").arg(y_selected[0], 0, 'f', panel == 1 ? 4 : 1).arg(unit[panel]),
                          Graph::Alignment::Right, Graph::Alignment::Top);
            }
        }
        delete [] x;
        delete [] y;
        delete [] col;
    }

    // draw labels with units
    for (int panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        QPoint p1, p2;
        graphTriple[panel]->getExtent(p1, p2);
        p1.rx() += 5;
        p1.ry() += 3;
        drawLabel(graphTriple[panel], c, p1, QStringLiteral("%1").arg(label[panel]), Graph::Alignment::Left, Graph::Alignment::Top);
    }
}
// draw SNR, MP to elevation-plot ----------------------------------------------
void Plot::drawSnrE(QPainter &c, int level)
{
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3};
    QString s;
    const QString label[] = {tr("SNR"), tr("Multipath")};
    gtime_t time_selected = {0, 0};
    double ave = 0.0, rms = 0.0;
    int nrms = 0;

    trace(3, "drawSnrE: level=%d\n", level);

    int bottomPanel = 0;
    for (int panel = 0; panel < 2; panel++)
        if (btn[panel]->isChecked()) bottomPanel = panel;

    for (int panel = 0; panel < 2; panel++) {
        if (!btn[panel]->isChecked()) continue;

        QPoint p1, p2;
        double xl[2] = {-0.001, 90.0};
        double yl[2][2] = {{10.0, 65.0}, {-plotOptDialog->getMaxMP(), plotOptDialog->getMaxMP()}};

        // draw axis
        graphDual[panel]->setLabelUnits(panel == bottomPanel ? "°" : "", panel == 0 ? tr(" dBHz") : tr("m"));
        graphDual[panel]->xLabelPosition = panel == bottomPanel ? Graph::LabelPosition::Outer : Graph::LabelPosition::Off;
        graphDual[panel]->yLabelPosition = Graph::LabelPosition::Outer;
        graphDual[panel]->setLimits(xl, yl[panel]);
        graphDual[panel]->setTick(0.0, 0.0);
        graphDual[panel]->drawAxis(c, true, true);

        graphDual[panel]->getExtent(p1, p2);
        p1.setX(0);
        p1.setY((p1.y() + p2.y()) / 2); // center
        graphDual[panel]->drawText(c, p1, label[panel], plotOptDialog->getCColor(2), Graph::Alignment::Center, Graph::Alignment::Top, 90);

        // draw label for x axis
        if (panel == bottomPanel) {
            p2.rx() -= 8;
            p2.ry() -= 6;
            graphDual[panel]->drawText(c, p2, tr("Elevation"), plotOptDialog->getCColor(2),
                                Graph::Alignment::Right, Graph::Alignment::Bottom, 0);
        }
    }

    if (0 <= observationIndex && observationIndex < nObservation && ui->btnShowTrack->isChecked())
        time_selected = observation.data[indexObservation[observationIndex]].time;

    if (nObservation > 0 && ui->btnSolution1->isChecked()) {
        QColor *col[2], col_selected[2][MAXSAT];
        QVariant obstype = ui->cBObservationTypeSNR->currentData();
        double *x[2], *y[2], x_selected[2][MAXSAT], y_selected[2][MAXSAT];
        int n[2], n_selected[2] = {0};

        for (int panel = 0; panel < 2; panel++) {
            x[panel] = new double[nObservation],
            y[panel] = new double[nObservation];
            col[panel] = new QColor[nObservation];
        }

        for (uint8_t sat = 1; sat <= MAXSAT; sat++) {
            if (satelliteMask[sat - 1] || !satelliteSelection[sat - 1]) continue;
            n[0] = n[1] = 0;

            for (int j = 0; j < observation.n; j++) {
                obsd_t *obs = observation.data + j;
                int idx;

                if (obs->sat != sat || elevation[j] <= 0.0) continue;

                if (strcmp(obstype.typeName(), "int") == 0) {
                    int freq = obstype.toInt();
                    idx = freq > 2 ? freq-3 : freq-1;  /* L1,L2,L5,L6 ... */
                } else {
                    for (idx = 0; idx < NFREQ + NEXOBS; idx++) {
                        if (!strcmp(code2obs(obs->code[idx]), qPrintable(obstype.toString()))) break;
                    }
                    if (idx >= NFREQ + NEXOBS) continue;
                }
                if (obs->SNR[idx] * SNR_UNIT <= 0.0) continue;

                x[0][n[0]] = x[1][n[1]] = elevation[j] * R2D;
                y[0][n[0]] = obs->SNR[idx] * SNR_UNIT;
                y[1][n[1]] = !multipath[idx] ? 0.0 : multipath[idx][j];

                col[0][n[0]] = col[1][n[1]] = (elevation[j] > 0.0 && elevation[j] < plotOptDialog->getElevationMask() * D2R) ?\
                    plotOptDialog->getMarkerColor(0, 0) : plotOptDialog->getMarkerColor(0, 4);

                // SNR plot
                if (y[0][n[0]] > 0.0) {
                    if (timediff(time_selected, observation.data[j].time) == 0.0) {
                        x_selected[0][n_selected[0]] = x[0][n[0]];
                        y_selected[0][n_selected[0]] = y[0][n[0]];
                        col_selected[0][n_selected[0]] = observationColor(observation.data + j, azimuth[j], elevation[j], obstype);
                        if (n_selected[0] < MAXSAT && col_selected[0][n_selected[0]] != Qt::black) n_selected[0]++;
                    }
                    if (n[0] < nObservation) n[0]++;
                }

                // multipath plot
                if (y[1][n[1]] != 0.0) {
                    // calculate statistics
                    if (elevation[j] >= plotOptDialog->getElevationMask() * D2R) {
                        ave += y[1][n[1]];
                        rms += SQR(y[1][n[1]]);
                        nrms++;
                    }
                    if (timediff(time_selected, observation.data[j].time) == 0.0) {
                        x_selected[1][n_selected[1]] = x[1][n[1]];
                        y_selected[1][n_selected[1]] = y[1][n[1]];
                        col_selected[1][n_selected[1]] = observationColor(observation.data + j, azimuth[j], elevation[j], obstype);

                        if (n_selected[1] < MAXSAT && col_selected[1][n_selected[1]] != Qt::black) n_selected[1]++;
                    }
                    if (n[1] < nObservation) n[1]++;
                }
            }
            if (!level || !(plotOptDialog->getPlotStyle() % 2)) {  // line plot style
                for (int panel = 0; panel < 2; panel++) {
                    if (!btn[panel]->isChecked()) continue;

                    drawPolyS(graphDual[panel], c, x[panel], y[panel], n[panel], plotOptDialog->getCColor(3), 0);
                }
            }
            if (level && plotOptDialog->getPlotStyle() < 2) {  // marker style
                for (int panel = 0; panel < 2; panel++) {
                    if (!btn[panel]->isChecked()) continue;

                    for (int j = 0; j < n[panel]; j++)
                        graphDual[panel]->drawMark(c, x[panel][j], y[panel][j], Graph::MarkerTypes::Dot, col[panel][j], plotOptDialog->getMarkSize(), 0);
                }
            }
        }
        for (int panel = 0; panel < 2; panel++) {
            delete[] x[panel];
            delete[] y[panel];
            delete[] col[panel];
        }

        // highlight selected position
        if (ui->btnShowTrack->isChecked() && 0 <= observationIndex && observationIndex < nObservation && ui->btnSolution1->isChecked()) {
            for (int panel = 0; panel < 2; panel++) {
                if (!btn[panel]->isChecked()) continue;

                for (int j = 0; j < n_selected[panel]; j++) {
                    graphDual[panel]->drawMark(c, x_selected[panel][j], y_selected[panel][j], Graph::MarkerTypes::Dot, plotOptDialog->getCColor(0), plotOptDialog->getMarkSize() * 2 + 8, 0);
                    graphDual[panel]->drawMark(c, x_selected[panel][j], y_selected[panel][j], Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), plotOptDialog->getMarkSize() * 2 + 6, 0);
                    graphDual[panel]->drawMark(c, x_selected[panel][j], y_selected[panel][j], Graph::MarkerTypes::Dot, col_selected[panel][j], plotOptDialog->getMarkSize() * 2 + 2, 0);
                }
            }
        }
    }

    // show statistics
    if (plotOptDialog->getShowStats()) {
        int topPanel;
        for (topPanel = 0; topPanel < 2; topPanel++)
            if (btn[topPanel]->isChecked()) break;

        if (topPanel < 2) {
            QPoint p1, p2;
            int hh = (int)(QFontMetrics(plotOptDialog->getFont()).height() * 1.5);

            // get top left position to draw station properties
            graphDual[topPanel]->getExtent(p1, p2);
            p1.rx() += 8;
            p1.ry() += 6;
            s = tr("MARKER: %1 %2").arg(station.name, station.marker);
            drawLabel(graphDual[topPanel], c, p1, s, Graph::Alignment::Left, Graph::Alignment::Top); p1.ry() += hh;
            s = tr("REC: %1 %2 %3").arg(station.rectype, station.recver, station.recsno);
            drawLabel(graphDual[topPanel], c, p1, s, Graph::Alignment::Left, Graph::Alignment::Top); p1.ry() += hh;
            s = tr("ANT: %1 %2").arg(station.antdes, station.antsno);
            drawLabel(graphDual[topPanel], c, p1, s, Graph::Alignment::Left, Graph::Alignment::Top); p1.ry() += hh;
        }

        // draw statistics for multipath panel
        if (btn[1]->isChecked() && nrms > 0 && !ui->btnShowTrack->isChecked()) {
            QPoint p1, p2;
            ave = ave / nrms;
            rms = SQRT(rms / nrms);

            graphDual[1]->getExtent(p1, p2);
            p1.setX(p2.x() - 8);
            p1.ry() += 6;
            drawLabel(graphDual[1], c, p1, tr("AVE = %L1 m, RMS = %L2 m").arg(ave, 0, 'f', 4).arg(rms, 0, 'f', 4),
                      Graph::Alignment::Right, Graph::Alignment::Top);
        }
    }
}
// draw MP-skyplot ----------------------------------------------------------
void Plot::drawMpSky(QPainter &c, int level)
{
    QVariant obstype = ui->cBObservationTypeSNR->currentData();
    double radius, xl[2], yl[2], xs, ys;

    trace(3, "drawMpSky: level=%d\n", level);

    graphSky->getLimits(xl, yl);
    radius = qMin(xl[1] - xl[0], yl[1] - yl[0]) * 0.45;

    if (ui->btnShowImage->isChecked())
        drawSkyImage(c, level);

    if (ui->btnShowSkyplot->isChecked())
        graphSky->drawSkyPlot(c, 0.0, 0.0, plotOptDialog->getCColor(1), plotOptDialog->getCColor(2), plotOptDialog->getCColor(0), radius * 2.0);

    if (!ui->btnSolution1->isChecked() || nObservation <= 0 || simulatedObservation) return;

    graphSky->getScale(xs, ys);

    for (uint8_t sat = 1; sat <= MAXSAT; sat++) {
        double previous[MAXSAT][2] = {{0}};

        if (satelliteMask[sat - 1] || !satelliteSelection[sat - 1]) continue;

        int siz = plotOptDialog->getPlotStyle() < 2 ? plotOptDialog->getMarkSize() : 1;

        for (int i = 0; i < observation.n; i++) {
            obsd_t *obs = observation.data+i;
            int idx;

            if (obs->sat != sat || elevation[i] <= 0.0) continue;

            if (strcmp(obstype.typeName(), "int") == 0) {
                int freq = obstype.toInt();
                idx = freq > 2 ? freq - 3 : freq - 1;  /* L1, L2, L5, L6 ... */
            } else {
                for (idx = 0; idx < NFREQ + NEXOBS; idx++) {
                    if (!strcmp(code2obs(obs->code[idx]), qPrintable(obstype.toString()))) break;
                }
                if (idx >= NFREQ+NEXOBS) continue;
            }

            // calculate position
            double x = radius * sin(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            double y = radius * cos(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            double xp = previous[sat-1][0];
            double yp = previous[sat-1][1];
            QColor col = mpColor(!multipath[idx] ? 0.0 : multipath[idx][i]);

            if ((x - xp) * (x - xp) + (y - yp) * (y - yp) >= xs * xs) {
                graphSky->drawMark(c, x, y, Graph::MarkerTypes::Dot, col, siz, 0);
                graphSky->drawMark(c, x, y, Graph::MarkerTypes::Dot, plotOptDialog->getPlotStyle() < 2 ? col : plotOptDialog->getCColor(3), siz, 0);
                previous[sat - 1][0] = x;
                previous[sat - 1][1] = y;
            }
        }
    }

    // highlight selected data
    if (ui->btnShowTrack->isChecked() && 0 <= observationIndex && observationIndex < nObservation) {
        char id[32];
        int fontsize = plotOptDialog->getFont().pointSize();
        for (int i = indexObservation[observationIndex]; i < observation.n && i < indexObservation[observationIndex + 1]; i++) {
            obsd_t *obs = observation.data+i;
            int idx;

            if (satelliteMask[obs->sat-1] || !satelliteSelection[obs->sat-1] || elevation[i] <= 0.0) continue;

            if (strcmp(obstype.typeName(), "int") == 0) {
                int freq = obstype.toInt();
                idx = freq > 2 ? freq - 3 : freq - 1;  /* L1,L2,L5,L6 ... */
            } else {
                for (idx = 0; idx < NFREQ + NEXOBS; idx++) {
                    if (!strcmp(code2obs(obs->code[idx]), qPrintable(obstype.toString()))) break;
                }
                if (idx >= NFREQ + NEXOBS) continue;
            }
            QColor col = mpColor(!multipath[idx] ? 0.0 : multipath[idx][i]);
            double x = radius * sin(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);
            double y = radius * cos(azimuth[i]) * (1.0 - 2.0 * elevation[i] / PI);

            satno2id(obs->sat, id);

            graphSky->drawMark(c, x, y, Graph::MarkerTypes::Dot, col, fontsize * 2 + 5, 0);
            graphSky->drawMark(c, x, y, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), fontsize * 2 + 5, 0);
            graphSky->drawText(c, x, y, QString(id), plotOptDialog->getCColor(0), Graph::Alignment::Center, Graph::Alignment::Center, 0);
        }
    }
}
// draw residuals and SNR/elevation plot ------------------------------------
void Plot::drawResidual(QPainter &c, int level)
{
    const QString label[] = {
        tr("Pseudorange Residuals"),
        tr("Carrier-Phase Residuals"),
        tr("Elevation Angle (deg) / Signal Strength (dBHz)")
    };
    QString str;
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3};
    int sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;
    int ind = solutionIndex[sel];
    uint8_t frq = ui->cBFrequencyType->currentIndex() + 1;
    int n = solutionStat[sel].n;

    trace(3, "drawResidual: level=%d\n", level);

    // update panel center position
    if (0 <= ind && ind < solutionData[sel].n && ui->btnShowTrack->isChecked() && ui->btnFixHorizontal->isChecked()) {
        gtime_t t = solutionData[sel].data[ind].time;

        for (int panel = 0; panel < 3; panel++) {
            double offset, xc, yc, xl[2], yl[2];

            if (ui->btnFixHorizontal->isChecked()) {
                graphTriple[panel]->getLimits(xl, yl);
                offset = centX * (xl[1] - xl[0]) / 2.0;
                graphTriple[panel]->getCenter(xc, yc);
                graphTriple[panel]->getCenter(xc, yc);
                graphTriple[panel]->setCenter(timePosition(t) - offset, yc);
            } else {
                graphTriple[panel]->getRight(xc, yc);
                graphTriple[panel]->setRight(timePosition(t), yc);
            }
        }
    }

    // draw axis
    int bottomPanel = -1;
    for (int panel = 0; panel < 3; panel++)
        if (btn[panel]->isChecked()) bottomPanel = panel;
    for (int panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        graphTriple[panel]->setLabelUnits("", panel == 2 ? tr("") : tr(" m"));
        graphTriple[panel]->xLabelPosition = plotOptDialog->getTimeFormat() ? (panel == bottomPanel ? Graph::LabelPosition::Time : Graph::LabelPosition::None) :
                                        (panel == bottomPanel ? Graph::LabelPosition::Outer : Graph::LabelPosition::Off);
        graphTriple[panel]->week = week;
        graphTriple[panel]->drawAxis(c, plotOptDialog->getShowGridLabel(), plotOptDialog->getShowGridLabel());
    }

    if (n > 0 && ((sel == 0 && ui->btnSolution1->isChecked()) || (sel == 1 && ui->btnSolution2->isChecked()))) {
        QColor *col[4];
        double *x[4], *y[4], sum[2] = {0}, sum2[2] = {0};
        int ns[2] = {0};

        for (int i = 0; i < 4; i++) {  // for pseudorange residuals, carrier-phase residuals, elevation angle, SNR
            x[i] = new double[n],
            y[i] = new double[n];
            col[i] = new QColor[n];
        }

        for (uint8_t sat = 1; sat <= MAXSAT; sat++) {
            if (satelliteMask[sat - 1] || !satelliteSelection[sat - 1]) continue;

            int m[4] = {0};

            for (int i = 0; i < n; i++) {
                solstat_t *solstat = solutionStat[sel].data + i;
                int q;

                if (solstat->sat != sat || solstat->frq != frq) continue;
                if (solstat->resp == 0.0 && solstat->resc == 0.0) continue;

                x[0][m[0]] = x[1][m[1]] = x[2][m[2]] = x[3][m[3]] = timePosition(solstat->time);
                y[0][m[0]] = solstat->resp;
                y[1][m[1]] = solstat->resc;
                y[2][m[2]] = solstat->el * R2D;
                y[3][m[3]] = solstat->snr * SNR_UNIT;

                if (!(solstat->flag >> 5)) q = 0;          // invalid
                else if ((solstat->flag & 7) <= 1) q = 2;  // float
                else if ((solstat->flag & 7) <= 3) q = 1;  // fixed
                else q = 6;                                // ppp

                col[0][m[0]] = plotOptDialog->getMarkerColor(0, q);
                col[1][m[1]] = ((solstat->flag >> 3) & 1) ? Qt::red : plotOptDialog->getMarkerColor(0, q);
                col[2][m[2]] = plotOptDialog->getMarkerColor(0, 1);
                col[3][m[3]] = plotOptDialog->getMarkerColor(0, 4);        // slip

                if (solstat->resp != 0.0) {  // residul pseudorange
                    sum [0] += solstat->resp;
                    sum2[0] += solstat->resp * solstat->resp;
                    ns[0]++;
                }
                if (solstat->resc != 0.0) {  // residual carrier-phase
                    sum [1] += solstat->resc;
                    sum2[1] += solstat->resc * solstat->resc;
                    ns[1]++;
                }
                m[0]++; m[1]++; m[2]++; m[3]++;
            }

            for (int panel = 0; panel < 3; panel++) {
                if (!btn[panel]->isChecked()) continue;

                if (!level || !(plotOptDialog->getPlotStyle() % 2)) {  // line style
                    drawPolyS(graphTriple[panel], c, x[panel], y[panel], m[panel], plotOptDialog->getCColor(3), 0);
                    if (panel == 2) drawPolyS(graphTriple[panel], c, x[3], y[3], m[3], plotOptDialog->getCColor(3), 0);
                }
                if (level && plotOptDialog->getPlotStyle() < 2) {  // marker style
                    for (int j = 0; j < m[panel]; j++) {
                        graphTriple[panel]->drawMark(c, x[panel][j], y[panel][j], Graph::MarkerTypes::Dot, col[panel][j], plotOptDialog->getMarkSize(), 0);
                    }
                    if (panel == 2) {  // additionally plot SNR values in elevation/SNR panel
                        for (int j = 0; j < m[3]; j++) {
                            graphTriple[panel]->drawMark(c, x[3][j], y[3][j], Graph::MarkerTypes::Dot, col[3][j], plotOptDialog->getMarkSize(), 0);
                        }
                    }
                }
            }
        }
        for (int i = 0; i < 4; i++) {
            delete [] x[i];
            delete [] y[i];
            delete [] col[i];
        }

        // draw statistics
        if (plotOptDialog->getShowStats()) {
            for (int panel = 0; panel < 2; panel++) {
                if (!btn[panel]->isChecked()) continue;

                QPoint p1, p2;
                double ave, std, rms;
                ave = ns[panel] <= 0 ? 0.0 : sum[panel] / ns[panel];
                std = ns[panel] <= 1 ? 0.0 : SQRT((sum2[panel] - 2.0 * sum[panel] * ave + ns[panel] * ave * ave) / (ns[panel] - 1));
                rms = ns[panel] <= 0 ? 0.0 : SQRT(sum2[panel] / ns[panel]);

                graphTriple[panel]->getExtent(p1, p2);
                p1.setX(p2.x() - 5);
                p1.ry() += 3;
                str = tr("AVE = %L1 m, STD = %L2 m, RMS = %L3 m").arg(ave, 0, 'f', 3).arg(std, 0, 'f', 3).arg(rms, 0, 'f', 3);
                drawLabel(graphTriple[panel], c, p1, str, Graph::Alignment::Right, Graph::Alignment::Top);
            }
        }

        // highlight selected data
        if (ui->btnShowTrack->isChecked() && 0 <= ind && ind < solutionData[sel].n && (ui->btnSolution1->isChecked() || ui->btnSolution2->isChecked())) {
            for (int i = 0, j = 0; i < 3; i++) {
                double xl[2], yl[2];

                if (!btn[i]->isChecked()) continue;

                gtime_t t = solutionData[sel].data[ind].time;

                graphTriple[i]->getLimits(xl, yl);
                xl[0] = xl[1] = timePosition(t);
                graphTriple[i]->drawPoly(c, xl, yl, 2, ind == 0 ? plotOptDialog->getCColor(1) : plotOptDialog->getCColor(2), 0);

                if (j++ == 0) {
                    graphTriple[i]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Dot, plotOptDialog->getCColor(2), 5, 0);
                    graphTriple[i]->drawMark(c, xl[0], yl[1] - 1E-6, Graph::MarkerTypes::Circle, plotOptDialog->getCColor(2), 9, 0);
                }
            }
        }
    }

    // draw labels
    for (int panel = 0; panel < 3; panel++) {
        if (!btn[panel]->isChecked()) continue;

        QPoint p1, p2;
        graphTriple[panel]->getExtent(p1, p2);
        p1.rx() += 5;
        p1.ry() += 3;
        drawLabel(graphTriple[panel], c, p1, label[panel], Graph::Alignment::Left, Graph::Alignment::Top);
    }
}
// draw residuals - elevation plot ------------------------------------------
void Plot::drawResidualE(QPainter &c, int level)
{
    QPushButton *btn[] = {ui->btnOn1, ui->btnOn2, ui->btnOn3};
    const QString label[] = {tr("Pseudorange Residuals"), tr("Carrier-Phase Residuals")};
    int bottomPanel, sel = !ui->btnSolution1->isChecked() && ui->btnSolution2->isChecked() ? 1 : 0;
    int frq = ui->cBFrequencyType->currentIndex() + 1;
    int n = solutionStat[sel].n;

    trace(3,"drawResidualE: level=%d\n", level);

    // draw axes
    bottomPanel = 0;
    for (int panel = 0; panel < 2; panel++)
        if (btn[panel]->isChecked()) bottomPanel = panel;

    for (int panel = 0; panel < 2; panel++) {
        if (!btn[panel]->isChecked()) continue;

        QPoint p1, p2;
        double xl[2] = {-0.001, 90.0};
        double yl[2][2] = {{-plotOptDialog->getMaxMP(), plotOptDialog->getMaxMP()}, {-plotOptDialog->getMaxMP() / 100.0, plotOptDialog->getMaxMP() / 100.0}};
        
        graphDual[panel]->xLabelPosition = (panel == bottomPanel) ? Graph::LabelPosition::Outer : Graph::LabelPosition::Off;
        graphDual[panel]->yLabelPosition = Graph::LabelPosition::Outer;

        graphDual[panel]->setLabelUnits(panel == bottomPanel ? "°" :"", tr(" m"));
        graphDual[panel]->setLimits(xl, yl[panel]);
        graphDual[panel]->setTick(0.0, 0.0);
        graphDual[panel]->drawAxis(c, true, true);

        graphDual[panel]->getExtent(p1, p2);
        p1.setX((int)(QFontMetrics(plotOptDialog->getFont()).height()));
        p1.setY((p1.y() + p2.y()) / 2);
        graphDual[panel]->drawText(c, p1, label[panel], plotOptDialog->getCColor(2), Graph::Alignment::Center, Graph::Alignment::Center, 90);

        if (panel == bottomPanel) {
            p2.rx() -= 8;
            p2.ry() -= 6;
            graphDual[panel]->drawText(c, p2, tr("Elevation"), plotOptDialog->getCColor(2), Graph::Alignment::Right, Graph::Alignment::Bottom, 0);
        }
    }

    // draw data
    if (n > 0 && ((sel == 0 && ui->btnSolution1->isChecked()) || (sel == 1 && ui->btnSolution2->isChecked()))) {
        QColor *col[2];
        double *x[2], *y[2], sum[2] = {0}, sum2[2] = {0};
        int ns[2] = {0};

        for (int i = 0; i < 2; i++) {
            x[i] = new double[n],
            y[i] = new double[n];
            col[i] = new QColor[n];
        }
        for (int sat = 1; sat <= MAXSAT; sat++) {
            if (satelliteMask[sat - 1] || !satelliteSelection[sat - 1]) continue;
            int q, m[2] = {0};

            for (int i = 0; i < n; i++) {
                solstat_t *solstat = solutionStat[sel].data + i;
                if (solstat->sat != sat || solstat->frq != frq) continue;

                x[0][m[0]] = x[1][m[1]] = solstat->el*R2D;
                y[0][m[0]] = solstat->resp;  // pseudorange residuals
                y[1][m[1]] = solstat->resc;  // carrier-phase residuals

                if (!(solstat->flag >> 5))  q = 0; // invalid
                else if ((solstat->flag & 0x7) <= 1) q = 2; // float
                else if ((solstat->flag & 0x7) <= 3) q = 1; // fixed
                else q = 6; // ppp

                col[0][m[0]] = plotOptDialog->getMarkerColor(0, q);
                col[1][m[1]] = ((solstat->flag >> 3) & 1) ? Qt::red : plotOptDialog->getMarkerColor(0, q);

                // calculate statistics
                if (solstat->resp != 0.0) {
                    sum[0] += solstat->resp;
                    sum2[0] += solstat->resp * solstat->resp;
                    ns[0]++;
                }
                if (solstat->resc != 0.0) {
                    sum[1] += solstat->resc;
                    sum2[1] += solstat->resc * solstat->resc;
                    ns[1]++;
                }
                m[0]++;
                m[1]++;
            }
            for (int panel = 0; panel < 2; panel++) {
                if (!btn[panel]->isChecked()) continue;

                if (!level || !(plotOptDialog->getPlotStyle() % 2)) {  // line style
                    drawPolyS(graphDual[panel], c, x[panel], y[panel], m[panel], plotOptDialog->getCColor(3), 0);
                }
                if (level && plotOptDialog->getPlotStyle() < 2) {  // marker style
                    for (int j = 0; j < m[panel]; j++) {
                        graphDual[panel]->drawMark(c, x[panel][j], y[panel][j], Graph::MarkerTypes::Dot, col[panel][j], plotOptDialog->getMarkSize(), 0);
                    }
                }
            }
        }
        for (int i = 0; i < 2; i++) {
            delete[] x[i];
            delete[] y[i];
            delete[] col[i];
        }

        // draw statistics
        if (plotOptDialog->getShowStats()) {
            for (int panel = 0; panel < 2; panel++) {
                if (!btn[panel]->isChecked()) continue;

                QString str;
                QPoint p1, p2;
                double ave,std,rms;

                ave = ns[panel] <= 0 ? 0.0 : sum[panel] / ns[panel];
                std = ns[panel] <= 1 ? 0.0 : SQRT((sum2[panel] - 2.0 * sum[panel] * ave + ns[panel] * ave * ave) / (ns[panel] - 1));
                rms = ns[panel] <= 0 ? 0.0 : SQRT(sum2[panel] / ns[panel]);

                graphDual[panel]->getExtent(p1, p2);
                p1.setX(p2.x() - 5);
                p1.ry() += 3;
                str = tr("AVE = %L1 m, STD = %L2 m, RMS = %L3 m").arg(ave, 0, 'f', 3).arg(std, 0, 'f', 3).arg(rms, 0, 'f', 3);
                drawLabel(graphTriple[panel], c, p1, str, Graph::Alignment::Right, Graph::Alignment::Top);
            }
        }
    }
}
// draw polyline without time-gaps ------------------------------------------
void Plot::drawPolyS(Graph *graph, QPainter &c, double *x, double *y, int n, const QColor &color, int style)
{
    // draw connected line segments except their distance is greater TBRK
    int i, j;

    for (i = 0; i < n; i = j) {
        for (j = i + 1; j < n; j++)
            if (fabs(x[j] - x[j - 1]) > TBRK) break;
        graph->drawPoly(c, x + i, y + i, j - i, color, style);
    }
}
// draw label with hemming --------------------------------------------------
void Plot::drawLabel(Graph *g, QPainter &c, const QPoint &p, const QString &label, int ha, int va)
{
    g->drawText(c, p, label, plotOptDialog->getCColor(2), plotOptDialog->getCColor(0), ha, va, 0);
}
// draw mark with hemming ---------------------------------------------------
void Plot::drawMark(Graph *g, QPainter &c, const QPoint &p, int mark, const QColor &color, int size, int rot)
{
    g->drawMark(c, p, mark, color, plotOptDialog->getCColor(0), size, rot);
}
// refresh map view --------------------------------------------------
void Plot::refresh_MapView()
{
    sol_t *sol;
    double pos[3] = {0};

    if (ui->btnShowTrack->isChecked()) {
        if (ui->btnSolution2->isChecked() && solutionData[1].n > 0 &&
                (sol = getsol(solutionData + 1, solutionIndex[1]))) {
            ecef2pos(sol->rr, pos);
            mapView->setMark(2, tr("Solution 2"), pos[0] * R2D, pos[1] * R2D);
            mapView->showMark(2);
        } else {
            mapView->hideMark(2);
        }

        if (ui->btnSolution1->isChecked() && solutionData[0].n > 0 &&
                (sol = getsol(solutionData, solutionIndex[0]))) {
            ecef2pos(sol->rr, pos);
            mapView->setMark(1, tr("Solution 1"), pos[0] * R2D, pos[1] * R2D);
            mapView->showMark(1);
        } else {
            mapView->hideMark(1);
        }
    } else {
        mapView->hideMark(1);
        mapView->hideMark(2);
    }

}
