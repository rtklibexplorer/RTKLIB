//---------------------------------------------------------------------------
// graph.cpp: graph plot subfunctions
//---------------------------------------------------------------------------
#include <QPainter>
#include <QBrush>
#include <QPen>
#include <QLocale>

#include <math.h>
#include "rtklib.h"
#include "graph.h"

#define MINSIZE		10			// min width/height
#define MINSCALE	2E-5		// min scale factor (pixel/unit)
#define MAXSCALE	1E7			// max scale factor (pixel/unit)
#define MAXCIRCLES	100			// max number of circles
#define SIZEORIGIN	6

#define SQR(x)		((x)*(x))
#define MIN(x,y)	((x)<(y)?(x):(y))
#define MAX(x,y)	((x)>(y)?(x):(y))

// constructor --------------------------------------------------------------
Graph::Graph(QPaintDevice *p)
{
    parent = p;
    x = y = 0;
    setSize(parent->width(), parent->height());

    xCenter = yCenter = 0.0;
    xScale = yScale = 0.02;
    box = true;
    fit = true;
    xGrid = yGrid = true;
    xTick = yTick = 0.0;
    xLabelPosition = yLabelPosition = LabelPosition::Outer;
    week = 0;
    title = xLabel = yLabel = "";
    color[0] = Qt::black;           // background color
    color[1] = Qt::gray;            // grid color
    color[2] = Qt::black;           // title/label color
}
// --------------------------------------------------------------------------
void Graph::setSize(int w, int h)
{
    width = w;
    height = h;
}
// --------------------------------------------------------------------------
bool Graph::isInArea(const QPoint &p) // check whether point is within graph area
{
    return x <= p.x() && p.x() < x + width && y <= p.y() && p.y() < y + height;
}
//---------------------------------------------------------------------------
int Graph::toPoint(double x_, double y_, QPoint &p)
{
    const double xt = 0.1;

    x_ = x + (width - 1) / 2.0 + (x_ - xCenter) / xScale;
    y_ = y + (height - 1) / 2.0 - (y_ - yCenter) / yScale;
    if      (x_ < -1E6) x_ = -1E6; // clip to avoid numerical exception
    else if (x_ >  1E6) x_ =  1E6;
    if      (y_ < -1E6) y_ = -1E6;
    else if (y_ >  1E6) y_ =  1E6;
    p.setX((int)floor((x_ + 0.5)));
    p.setY((int)floor((y_ + 0.5)));

    return (x - xt < x_) && (x_ < x + width - 1 + xt) &&
        (y - xt < y_) && (y_ < y + height - 1 + xt);
}
//---------------------------------------------------------------------------
void Graph::resize()
{
    width = parent->width();
    height = parent->height();
}
//---------------------------------------------------------------------------
void Graph::toPos(const QPoint &p, double &x_, double &y_)
{
    x_ = xCenter + (p.x() - x - (width - 1) / 2.0) * xScale;
    y_ = yCenter - (p.y() - y - (height - 1) / 2.0) * yScale;
}
//---------------------------------------------------------------------------
void Graph::setPosition(const QPoint &p1, const QPoint &p2)
{
    int w = p2.x() - p1.x() + 1, h = p2.y() - p1.y() + 1;

    if (w < MINSIZE) w = MINSIZE;
    if (h < MINSIZE) h = MINSIZE;
	if (fit) {
        xScale *= (double)(width - 1) / (w - 1);
        yScale *= (double)(height - 1) / (h - 1);
	}
    x = p1.x();
    y = p1.y();
    width = w;
    height = h;
}
//---------------------------------------------------------------------------
void Graph::getExtent(QPoint &p1, QPoint &p2)
{
    p1.setX(x);
    p1.setY(y);
    p2.setX(x + width - 1);
    p2.setY(y + height - 1);
}
//---------------------------------------------------------------------------
void Graph::setCenter(const double x, const double y)
{
    xCenter = x;
    yCenter = y;
}
//---------------------------------------------------------------------------
void Graph::getCenter(double &x_, double &y_)
{
    x_ = xCenter;
    y_ = yCenter;
}
//---------------------------------------------------------------------------
void Graph::setRight(const double x_, const double y_)
{
//	XCent=x-(double)(Width-1)*XScale*0.5; YCent=y;
    xCenter = x_ - (double)(width - 13) * xScale * 0.5;
    yCenter = y_;
}
//---------------------------------------------------------------------------
void Graph::getRight(double &x_, double &y_)
{
//	x=XCent+(double)(Width-1)*XScale*0.5; y=YCent;
    x_ = xCenter + (double)(width - 13) * xScale * 0.5;
    y_ = yCenter;
}
//---------------------------------------------------------------------------
void Graph::setScale(const double xs_, const double ys_)
{
    double xs = xs_, ys = ys_;
    if (xs < MINSCALE) xs = MINSCALE; else if (MAXSCALE < xs) xs = MAXSCALE;
    if (ys < MINSCALE) ys = MINSCALE; else if (MAXSCALE < ys) ys = MAXSCALE;
    xScale = xs;
    yScale = ys;
}
//---------------------------------------------------------------------------
void Graph::getScale(double &xs, double &ys)
{
    xs = xScale;
    ys = yScale;
}
//---------------------------------------------------------------------------
void Graph::setLimits(const double *xl, const double *yl)
{
    if (xl[0] < xl[1]) {
        xCenter = (xl[0] + xl[1]) / 2.0;
        xScale = (xl[1] - xl[0]) / (width - 1);
    }
    if (yl[0] < yl[1]) {
        yCenter = (yl[0] + yl[1]) / 2.0;
        yScale = (yl[1] - yl[0]) / (height - 1);
    }
}
//---------------------------------------------------------------------------
void Graph::getLimits(double *xl, double *yl)
{
    QPoint p0(x, y), p1(x + width - 1, y + height - 1);

    toPos(p0, xl[0], yl[1]);
    toPos(p1, xl[1], yl[0]);
}
//---------------------------------------------------------------------------
void Graph::setTick(const double xt, const double yt)
{
    xTick = xt;
    yTick = yt;
}
//---------------------------------------------------------------------------
void Graph::getTick(double &xt, double &yt)
{
    xt = xTick > 0.0 ? xTick : (xLabelPosition == LabelPosition::None || xLabelPosition == LabelPosition::Time ? autoTickTime(xScale) : autoTick(xScale));
    yt = yTick > 0.0 ? yTick : autoTick(yScale);
}
//---------------------------------------------------------------------------
void Graph::setLabelUnits(const QString & xUnit, const QString & yUnit)
{
    xLabelUnit = xUnit;
    yLabelUnit = yUnit;
}
//---------------------------------------------------------------------------
void Graph::getLabelUnits(QString &xUnit, QString &yUnit)
{
    xUnit = xLabelUnit;
    yUnit = yLabelUnit;
};
//---------------------------------------------------------------------------
double Graph::autoTick(double scale)
{
    static const double t[] = { 1.0, 2.0, 5.0, 10.0 };
    double tick = 30.0 * scale;
    double order = pow(10.0, floor(log10(tick)));

    for (int i = 0; i < 4; i++) {
        if (tick <= t[i] * order)
            return t[i] * order;
    }
	return 10.0;
}
//---------------------------------------------------------------------------
double Graph::autoTickTime(double scale)
{
    static const double t[] = { 0.1, 0.2, 0.5, 1.0, 3.0, 6.0, 12.0, 30.0, 60.0, 300.0, 900.0, 1800.0, 3600.0,
                                7200.0, 10800.0, 21600.0, 43200.0, 86400.0, 86400.0 * 2, 86400.0 * 7, 86400.0 * 14,
                                86400.0 * 35, 86400.0 * 70 };
    double tick = 60.0 * scale;

    for (int i = 0; i < (int)(sizeof(t) / sizeof(*t)); i++) {
        if (tick <= t[i])
            return t[i];
    }
    return 86400.0 * 140;
}
//---------------------------------------------------------------------------
QString Graph::numText(double x, double dx)
{
    int n = (int)(0.9 - log10(dx));

    return QLocale().toString(x, 'f', n < 0 ? 0 : n);
}
//---------------------------------------------------------------------------
QString Graph::timeText(double x, double dx)
{
    char str[64];

    time2str(gpst2time(week, x), str, 1);
    // decide what part of the string to use
    int b = dx < 86400.0 ? 11 : (dx < 86400.0 * 30 ? 5 : 2);
    int w = dx < 60.0 ? (dx < 1.0 ? 10 : 8) : 5;
    return QString("%1").arg(str + b, w);
}
//---------------------------------------------------------------------------
void Graph::drawGrid(QPainter &c, double xt, double yt)
{
    double xl[2], yl[2];
    QPoint p;
    QPen pen = c.pen();

    getLimits(xl, yl);
    pen.setColor(color[1]);
    c.setPen(pen);
    c.setBrush(Qt::NoBrush);
	if (xGrid) {
        for (int i = (int)ceil(xl[0] / xt); i * xt <= xl[1]; i++) {
            toPoint(i * xt, 0.0, p);
            pen.setStyle(i != 0 ? Qt::DotLine : Qt::SolidLine); c.setPen(pen);
            c.drawLine(p.x(), y, p.x(), y + height - 1);
		}
	}
	if (yGrid) {
        for (int i = (int)ceil(yl[0] / yt); i * yt <= yl[1]; i++) {
            toPoint(0.0, i * yt, p);
            pen.setStyle(i != 0 ? Qt::DotLine : Qt::SolidLine); c.setPen(pen);
            c.drawLine(x, p.y(), x + width - 1, p.y());
		}
	}
    pen.setStyle(Qt::SolidLine);
    c.setPen(pen);
    drawMark(c, 0.0, 0.0, 0, color[1], SIZEORIGIN, 0);
}
//---------------------------------------------------------------------------
void Graph::drawGridLabel(QPainter &c, double xt, double yt)
{
    double xl[2], yl[2];
    QPoint p;

    getLimits(xl, yl);
    if (xLabelPosition) {
        for (int i = (int)ceil(xl[0] / xt); i * xt <= xl[1]; i++) {
            if (xLabelPosition <= LabelPosition::InnerRot) {
                toPoint(i * xt, yl[0], p);
                if (xLabelPosition == LabelPosition::Outer) p.ry() -= 1;
                int ha = xLabelPosition <= LabelPosition::Inner ? Alignment::Center :
                             (xLabelPosition == LabelPosition::OuterRot ? Alignment::Right : Alignment::Left);
                int va = xLabelPosition >= LabelPosition::OuterRot ? Alignment::Center :
                             (xLabelPosition == LabelPosition::Outer ? Alignment::Right : Alignment::Left);
                drawText(c, p, numText(i * xt, xt) + xLabelUnit, color[2], ha, va, xLabelPosition >= LabelPosition::OuterRot ? 90 : 0);
            } else if (xLabelPosition == LabelPosition::Time) {
                toPoint(i * xt, yl[0], p);
                drawText(c, p, timeText(i * xt, xt), color[2], Alignment::Center, Alignment::Right, 0);
            } else if (xLabelPosition == LabelPosition::Axis) {
                if (i == 0) continue;
                toPoint(i * xt, 0.0, p);
                drawText(c, p, numText(i * xt, xt) + xLabelUnit, color[2], Alignment::Center, Alignment::Right, 0);
			}
		}
	}
    if (yLabelPosition) {
        for (int i = (int)ceil(yl[0] / yt); i * yt <= yl[1]; i++) {
            if (yLabelPosition <= LabelPosition::InnerRot) {
                toPoint(xl[0], i * yt, p);
                if (yLabelPosition == LabelPosition::Outer) p.rx() -= 1;
                int ha = yLabelPosition >= LabelPosition::OuterRot ? Alignment::Center :
                             (yLabelPosition == LabelPosition::Outer ? Alignment::Right : Alignment::Left);
                int va = yLabelPosition <= LabelPosition::Inner ? Alignment::Center :
                             (yLabelPosition == LabelPosition::OuterRot ? Alignment::Left : Alignment::Right);
                drawText(c, p, numText(i * yt, yt) + yLabelUnit, color[2], ha, va, yLabelPosition >= LabelPosition::OuterRot ? 90 : 0);
            } else if (yLabelPosition == LabelPosition::Axis) {
                if (i == 0) continue;
                toPoint(0.0, i * yt, p);
                p.rx() += 2;
                drawText(c, p, numText(i * yt, yt) + yLabelUnit, color[2], Alignment::Left, Alignment::Center, 0);
			}
		}
	}
}
//---------------------------------------------------------------------------
void Graph::drawBox(QPainter &c)
{
	if (box) {
        QPen pen = c.pen();
        pen.setColor(color[1]);
        pen.setStyle(Qt::SolidLine);
        c.setPen(pen);
        c.setBrush(Qt::NoBrush);

        c.drawRect(x, y, width - 1, height - 1);
	}
}
//---------------------------------------------------------------------------
void Graph::drawLabel(QPainter &c)
{
    if (!xLabel.isEmpty()) {
        QPoint p(x + width / 2, y + height + ((xLabelPosition % 2) ? 10 : 2));
        drawText(c, p, xLabel, color[2], Alignment::Center, Alignment::Left, 0);
	}
    if (!yLabel.isEmpty()) {
        QPoint p(x - ((yLabelPosition % 2) ? 20 : 2), y + height / 2);
        drawText(c, p, yLabel, color[2], Alignment::Center, Alignment::Left, 90);
	}
    if (!title.isEmpty()) {
        QPoint p(x + width / 2, y - 1);
        drawText(c, p, title, color[2], Alignment::Center, Alignment::Left, 0);
	}
}
//---------------------------------------------------------------------------
void Graph::drawAxis(QPainter &c, bool label, bool gridLabel)
{
    double xt, yt;

    getTick(xt, yt);
    QPen pen = c.pen();

    pen.setColor(color[0]);
    c.setPen(pen);
    c.setBrush(color[0]);

    drawGrid(c, xt, yt);

    if (xt / xScale < 50.0 && xLabelPosition <= LabelPosition::Inner) xt *= xLabelPosition == LabelPosition::None ? 4.0 : 2.0;
    if (yt / yScale < 50.0 && yLabelPosition >= LabelPosition::OuterRot) yt *= 2.0;
    if (gridLabel) drawGridLabel(c, xt, yt);

    drawBox(c);

    if (label) drawLabel(c);
}
//---------------------------------------------------------------------------
void Graph::rotatePoints(QPoint *ps, int n, const QPoint &pc, int rot, QPoint *pr)
{
    double cos_rot = cos(rot*D2R), sin_rot = sin(rot*D2R);
    for (int i = 0; i < n; i++) {
        pr[i].setX(pc.x() + (int)floor(ps[i].x() * cos_rot - ps[i].y() * sin_rot + 0.5));
        pr[i].setY(pc.y() - (int)floor(ps[i].x() * sin_rot + ps[i].y() * cos_rot + 0.5));
	}
}
//---------------------------------------------------------------------------
void Graph::drawMark(QPainter &c, const QPoint &p, int mark, const QColor &color, int size, int rot)
{
	// mark = mark ( 0: dot  (.), 1: circle (o),  2: rect  (#), 3: cross (x)
	//               4: line (-), 5: plus   (+), 10: arrow (->),
	//              11: hscale,  12: vscale,     13: compass)
	// rot  = rotation angle (deg)
    if (size < 1) size = 1;
    int n, s = size / 2;
    int x = p.x() - s;
    int w = size + 1;
    int y = p.y() - s;
    int h = size + 1;
    int xs1[] = { -7, 0, -7, 0 }, ys1[] = { 2, 0, -2, 0 };
    int xs2[] = { -1, -1, -1, 1, 1, 1 }, ys2[] = { -1, 1, 0, 0, -1, 1 };
    int xs3[] = { 3, -4, 0, 0, 0, -8, 8 }, ys3[] = { 0, 5, 20, -20, -10, -10, -10 };
    int xs4[] = { 0, 0, 0, 1, -1}, ys4[] = { 1, -1, 0, 0, 0};
    QPoint ps[8], pr[8], pd(0, size / 2 + 12), pt;

    QPen pen = c.pen();
    pen.setColor(color);
    pen.setStyle(Qt::SolidLine);
    c.setPen(pen);
    QBrush brush(color);

	switch (mark) {
    case 0:         // dot
        brush.setStyle(Qt::SolidPattern);
        c.setBrush(brush);

        c.drawEllipse(x, y, w, h);
        return;
    case 1:         // circle
        brush.setStyle(Qt::NoBrush);
        c.setBrush(brush);

        c.drawEllipse(x, y, w, h);
        return;
    case 2:         // rectangle
        brush.setStyle(Qt::NoBrush);
        c.setBrush(brush);

        c.drawRect(x, y, w, h);
        return;
    case 3:         // cross
        brush.setStyle(Qt::NoBrush);
        c.setBrush(brush);

        c.drawLine(x, y, x + w, y + h);
        c.drawLine(x, y + h, x + w, y);
        return;
    case 4:         // line
        n = 2;
        ps[0].setX(-size / 2);
        ps[0].setY(0);
        ps[1].setX(size / 2);
        ps[1].setY(0);
        break;
    case 5:         // plus
        n = 5;
        for (int i=0;i<n;i++) {
            ps[i].setX(xs4[i] * s);
            ps[i].setY(ys4[i] * s);
        }
        break;
    case 10:         // arrow
        n = 6;
        ps[0].setX(-size / 2);
        ps[0].setY(0);
        ps[1].setX(size / 2);
        ps[1].setY(0);
        for (int i = 2; i < n; i++) {
            ps[i].setX(size / 2 + xs1[i - 2]);
            ps[i].setY(ys1[i - 2]);
        }
        break;
    case 11:        // hscale
    case 12:        // vscale
        n = 6;
        for (int i = 0; i < n; i++) {
            int x = xs2[i] * size / 2;
            int y = ys2[i] * 5;
            ps[i].setX(mark == 11 ? x : y); ps[i].setY(mark == 11 ? y : x);
        }
        break;
    case 13:         // compass
        n = 7;
        for (int i = 0; i < n; i++) {
            ps[i].setX(xs3[i] * size / 40);
            ps[i].setY(ys3[i] * size / 40);
        }
        rotatePoints(&pd, 1, p, rot, &pt);
        drawText(c, pt, "N", color, Alignment::Center, Alignment::Center, rot);
        break;
    default:
        return;
	}
    brush.setStyle(Qt::NoBrush);
    c.setBrush(brush);

    rotatePoints(ps, n, p, rot, pr);

    drawPoly(c, pr, n, color, 0);
}
//---------------------------------------------------------------------------
void Graph::drawMark(QPainter &c, double x, double y, int mark, const QColor &color, int size, int rot)
{
    QPoint p;

    if (toPoint(x, y, p)) drawMark(c, p, mark, color, size, rot);
}
//---------------------------------------------------------------------------
void Graph::drawMark(QPainter &c, const QPoint &p, int mark, const QColor &color, const QColor &bgcolor, int size, int rot)
{
    QPoint p1;

    p1 = p; p1.rx() -= 1; drawMark(c, p1, mark, bgcolor, size, rot); // draw with hemming
    p1 = p; p1.rx() += 1; drawMark(c, p1, mark, bgcolor, size, rot);
    p1 = p; p1.ry() -= 1; drawMark(c, p1, mark, bgcolor, size, rot);
    p1 = p; p1.ry() += 1; drawMark(c, p1, mark, bgcolor, size, rot);

    drawMark(c, p, mark, color, size, rot);
}
//---------------------------------------------------------------------------
void Graph::drawMark(QPainter &c, double x, double y, int mark, const QColor &color, const QColor &bgcolor, int size, int rot)
{
    QPoint p;

    if (toPoint(x, y, p)) drawMark(c, p, mark, color, bgcolor, size, rot);
}
//---------------------------------------------------------------------------
void Graph::drawMarks(QPainter &c, const double *x, const double *y, const QColor *colors,
                      int n, int mark, int size, int rot)
{
    QPoint p, pp;

    for (int i = 0; i < n; i++) {
        if (!toPoint(x[i], y[i], p) || (pp == p)) continue;
        drawMark(c, p, mark, colors[i], size, rot);
        pp = p;
	}
}
//---------------------------------------------------------------------------
void Graph::drawText(QPainter &c, const QPoint &p, const QString &str, const QColor &color, int ha, int va,
                     int rot)
{
    // str = UTF-8 string
    // ha  = horizontal alignment (0: center, 1: left,   2: right)
    // va  = vertical alignment   (0: center, 1: bottom, 2: top  )
    // rot = rotation angle (deg)
    int flags = 0;

    switch (ha) {
        case Graph::Alignment::Center: flags |= Qt::AlignHCenter; break;
        case Graph::Alignment::Left: flags |= Qt::AlignLeft; break;
        case Graph::Alignment::Right: flags |= Qt::AlignRight; break;
    }
    switch (va) {
        case Graph::Alignment::Center: flags |= Qt::AlignVCenter; break;
        case Graph::Alignment::Bottom: flags |= Qt::AlignBottom; break;
        case Graph::Alignment::Top: flags |= Qt::AlignTop; break;
    }

    QPen pen = c.pen();
    c.setBrush(Qt::NoBrush);
    pen.setColor(color);
    c.setPen(pen);

    QRectF off = c.boundingRect(QRectF(), flags, str);

    c.translate(p);
    c.rotate(-rot);
    c.drawText(off, str);
    c.rotate(rot);
    c.translate(-p);
}
//---------------------------------------------------------------------------
void Graph::drawText(QPainter &c, const QPoint &p, const QString &str, const QColor &color, int ha, int va,
                     int rot, const QFont &font)
{
	// str = UTF-8 string
	// ha  = horizontal alignment (0: center, 1: left,   2: right)
	// va  = vertical alignment   (0: center, 1: bottom, 2: top  )
	// rot = rotation angle (deg)
    int flags = 0;

    switch (ha) {
        case 0: flags |= Qt::AlignHCenter; break;
        case 1: flags |= Qt::AlignLeft; break;
        case 2: flags |= Qt::AlignRight; break;
    }
    switch (va) {
        case 0: flags |= Qt::AlignVCenter; break;
        case 1: flags |= Qt::AlignBottom; break;
        case 2: flags |= Qt::AlignTop; break;
    }

    QFont old_font = c.font();
    c.setFont(font);

    QPen pen = c.pen();
    c.setBrush(Qt::NoBrush);
    pen.setColor(color);
    c.setPen(pen);

    QRectF off = c.boundingRect(QRectF(), flags, str);

    c.translate(p);
    c.rotate(-rot);
    c.drawText(off, str);
    c.rotate(rot);
    c.translate(-p);
    c.setFont(old_font);
}
//---------------------------------------------------------------------------
void Graph::drawText(QPainter &c, double x, double y, const QString &str, const QColor &color,
                     int ha, int va, int rot)
{
    QPoint p;

    toPoint(x, y, p);
    drawText(c, p, str, color, ha, va, rot);
}
//---------------------------------------------------------------------------
void Graph::drawText(QPainter &c, double x, double y, const QString &str, const QColor &color,
                     int ha, int va, int rot, const QFont &font)
{
    QPoint p;

    toPoint(x, y, p);
    drawText(c, p, str, color, ha, va, rot, font);
}//---------------------------------------------------------------------------
void Graph::drawText(QPainter &c, const QPoint &p, const QString &str, const QColor &color, const QColor &bgcolor,
                     int ha, int va, int rot)
{
    QPoint p1;

    p1 = p; p1.rx() -= 1; drawText(c, p1, str, bgcolor, ha, va, rot); // draw with hemming
    p1 = p; p1.rx() += 1; drawText(c, p1, str, bgcolor, ha, va, rot);
    p1 = p; p1.ry() -= 1; drawText(c, p1, str, bgcolor, ha, va, rot);
    p1 = p; p1.ry() += 1; drawText(c, p1, str, bgcolor, ha, va, rot);

    drawText(c, p, str, color, ha, va, rot);
}
//---------------------------------------------------------------------------
void Graph::drawText(QPainter &c, double x, double y, const QString &str, const QColor &color,
                     const QColor &bgcolor, int ha, int va, int rot)
{
    QPoint p;

    toPoint(x, y, p);
    drawText(c, p, str, color, bgcolor, ha, va, rot);
}
//---------------------------------------------------------------------------
void Graph::drawCircle(QPainter &c, const QPoint &p, const QColor &color, int rx, int ry, int style)
{
    static const Qt::PenStyle ps[] = { Qt::SolidLine, Qt::DotLine, Qt::DashLine, Qt::DashDotLine, Qt::DashDotDotLine };
    int x = p.x() - rx, w = 2 * rx, y = p.y() - ry, h = 2 * ry;

    QPen pen = c.pen();
    pen.setColor(color);
    pen.setStyle(ps[style]);
    c.setPen(pen);
    c.setBrush(Qt::NoBrush);

    c.drawEllipse(x, y, w, h);
    pen.setStyle(Qt::SolidLine);
    c.setPen(pen);
}
//---------------------------------------------------------------------------
void Graph::drawCircle(QPainter &c, double x, double y, const QColor &color, double rx,
               double ry, int style)
{
    QPoint p;

    toPoint(x, y, p);
    drawCircle(c, p, color, (int)(rx / xScale + 0.5), (int)(ry / yScale + 0.5), style);
}
//---------------------------------------------------------------------------
void Graph::drawCircles(QPainter &c, int label)
{
    QPoint p;
    double xl[2], yl[2], xt, yt, r[4], rmin = 1E99, rmax = 0.0;
    int imin, imax;

    getLimits(xl, yl);
    getTick(xt, yt);

    r[0] = sqrt(SQR(xl[0]) + SQR(yl[0]));
    r[1] = sqrt(SQR(xl[0]) + SQR(yl[1]));
    r[2] = sqrt(SQR(xl[1]) + SQR(yl[0]));
    r[3] = sqrt(SQR(xl[1]) + SQR(yl[1]));
    for (int i = 0; i < 4; i++) {
        if (r[i] < rmin) rmin = r[i];
        if (r[i] > rmax) rmax = r[i];
    }
    if (xl[0] <= 0.0 && xl[1] >= 0.0 && yl[0] <= 0.0 && yl[1] >= 0.0) {
        imin = 0;
        imax = (int)ceil(rmax / xt);
    } else if (xl[0] <= 0.0 && xl[1] >= 0.0) {
        imin = (int)floor((yl[1] < 0.0 ? -yl[1] : yl[0]) / xt);
        imax = (int)ceil(rmax / xt);
    } else if (yl[0] <= 0.0 && yl[1] >= 0.0) {
        imin = (int)floor((xl[1] < 0.0 ? -xl[1] : xl[0]) / xt);
        imax = (int)ceil(rmax / xt);
    } else {
        imin = (int)floor(rmin / xt);
        imax = (int)ceil(rmax / xt);
	}
    for (int i = imin; i <= imax; i++)
        drawCircle(c, 0.0, 0.0, color[1], i * xt, i * xt, 1);
    toPoint(0.0, 0.0, p);

    QPen pen = c.pen();
    pen.setStyle(Qt::SolidLine);
    c.setPen(pen);

    c.drawLine(p.x(), y, p.x(), y + height - 1);
    c.drawLine(x, p.y(), x + width - 1, p.y());

    drawMark(c, 0.0, 0.0, 0, color[1], SIZEORIGIN, 0);

    if (xt / xScale < 50.0) xt *= 2.0;
    if (yt / yScale < 50.0) yt *= 2.0;
    if (label) drawGridLabel(c, xt, yt);

    drawBox(c);
}
//---------------------------------------------------------------------------
int Graph::onAxis(const QPoint &p)
{
	// area code :  5  4  6
	//              1  0  2
	//              9  8 10
    int xmin = x, xmax = x + width - 1, ymin = y, ymax = y + height - 1;

    return (p.x() < xmin ? 1 : (p.x() <= xmax ? 0 : 2)) + (p.y() < ymin ? 4 : (p.y() <= ymax ? 0 : 8));
}
//---------------------------------------------------------------------------
int Graph::clipPoint(QPoint *p0, int area, QPoint *p1)
{
    int x_, y_, xmin = x, xmax = x + width - 1, ymin = y, ymax = y + height - 1;

    if (area & 1) { // left
        if (p0->x() == p1->x()) return 0;
        y_ = p0->y() + (p1->y() - p0->y()) * (xmin - p0->x()) / (p1->x() - p0->x());
        if (ymin <= y_ && y_ <= ymax) {
            p0->setX(xmin);
            p0->setY(y_);
            return 1;
        }
	}
    if (area & 2) { // right
        if (p0->x() == p1->x()) return 0;
        y_ = p0->y() + (p1->y() - p0->y()) * (xmax - p0->x()) / (p1->x() - p0->x());
        if (ymin <= y_ && y_ <= ymax) {
            p0->setX(xmax);
            p0->setY(y_);
            return 1;
        }
	}
    if (area & 4) { // upper
        if (p0->y() == p1->y()) return 0;
        x_ = p0->x() + (p1->x() - p0->x()) * (ymin - p0->y()) / (p1->y() - p0->y());
        if (xmin <= x_ && x_ <= xmax) {
            p0->setX(x_);
            p0->setY(ymin);
            return 1;
        }
	}
    if (area & 8) { // lower
        if (p0->y()==p1->y()) return 0;
        x_ = p0->x() + (p1->x() - p0->x()) * (ymax - p0->y()) / (p1->y() - p0->y());
        if (xmin <= x_ && x_ <= xmax) {
            p0->setX(x_);
            p0->setY(ymax);
            return 1;
        }
	}
	return 0;
}
//---------------------------------------------------------------------------
void Graph::drawPolyline(QPainter &c, QPoint *p, int n)
{
#if 1
    c.drawPolyline(p, n);
#else
    // avoid overflow of points
    for (int i = 0; i < n - 1; i += 30000, p += 30000)
        c.drawPolyline(p, n - i > 30000 ? 30000 : n - i);

#endif
}
//---------------------------------------------------------------------------
void Graph::drawPoly(QPainter &c, QPoint *p, int n, const QColor &color, int style)
{
    static const Qt::PenStyle ps[] = { Qt::SolidLine, Qt::DotLine, Qt::DashLine, Qt::DashDotLine, Qt::DashDotDotLine };
    QPoint pc[2];

    QPen pen = c.pen();
    pen.setColor(color);
    pen.setStyle(ps[style]);
    c.setPen(pen);
    c.setBrush(Qt::NoBrush);

    int i, j, area0 = 11, area1;
    for (i = j = 0; j < n; j++, area0 = area1) {
        if ((area1 = onAxis(p[j])) == area0) continue;
        if (!area1) i = j;
        else if (!area0) drawPolyline(c, p + i, j - i);
        if (j <= 0 || (area0 & area1)) continue;

        pc[0] = p[j - 1];
        pc[1] = p[j];
        if (area0 && !clipPoint(pc, area0, p + j)) continue;
        if (area1 && !clipPoint(pc + 1, area1, p + j - 1)) continue;

        drawPolyline(c, pc, 2);
	}
    if (!area0) drawPolyline(c, p + i, j - i);

    pen.setStyle(Qt::SolidLine);
    c.setPen(pen);
}
//---------------------------------------------------------------------------
void Graph::drawPoly(QPainter &c, double *x, double *y, int n, const QColor &color, int style)
{
    QPoint *p = new QPoint[n];
    int m = 0;

    for (int i = 0; i < n; i++) {
        toPoint(x[i], y[i], p[m]);
        if (m == 0 || p[m - 1] != p[m]) m++;
	}

    drawPoly(c, p, m, color, style);

	delete [] p;
}
//---------------------------------------------------------------------------
void Graph::drawPatch(QPainter &c, QPoint *p, int n, const QColor &color1, const QColor &color2,
              int style)
{
    static const Qt::PenStyle ps[] = {Qt::SolidLine, Qt::DotLine, Qt::DashLine, Qt::DashDotLine, Qt::DashDotDotLine};
    int xmin = 1000000, xmax = 0, ymin = 1000000, ymax = 0;

    if (n > 30000) return; // # of points overflow

    for (int i = 0; i < n - 1; i++) {
        if (p[i].x() < xmin) xmin = p[i].x();
        if (p[i].x() > xmax) xmax = p[i].x();
        if (p[i].y() < ymin) ymin = p[i].y();
        if (p[i].y() > ymax) ymax = p[i].y();
    }
    if (xmax < x || xmin > x + width - 1 || ymax < y || ymin > y + height - 1)
        return;

    QPen pen = c.pen();
    pen.setColor(color1);
    pen.setStyle(ps[style]);
    c.setPen(pen);
    c.setBrush(color2);

    c.drawPolygon(p, n);

    pen.setStyle(Qt::SolidLine);
    c.setPen(pen);
}
//---------------------------------------------------------------------------
void Graph::drawPatch(QPainter &c, double *x, double *y, int n, const QColor &color1,
              const QColor &color2, int style)
{
    QPoint *p = new QPoint[n];

    for (int i = 0; i < n; i++)
        toPoint(x[i], y[i], p[i]);

    drawPatch(c, p, n, color1, color2, style);

	delete [] p;
}
//---------------------------------------------------------------------------
void Graph::drawSkyPlot(QPainter &c, const QPoint &p, const QColor &color1, const QColor &color2, int size)
{
    QPen pen = c.pen();

    pen.setColor(color1);
    c.setPen(pen);
    c.setBrush(Qt::NoBrush);
    QString s, dir = QT_TR_NOOP("NESW");
    QPoint ps;
    int r = size / 2;
    for (int el = 0; el < 90; el += 15) {
        int ys = r - r * el / 90;
        pen.setStyle(el == 0 ? Qt::SolidLine : Qt::DotLine);
        c.setPen(pen);
        c.drawEllipse(p.x() - ys, p.y() - ys, 2 * ys, 2 * ys);
        if (el <= 0) continue;
        ps.setX(p.x());
        ps.setY(p.y() - ys);
        drawText(c, ps, QString::number(el), color2, Alignment::Left, Alignment::Center, 0);
	}
    pen.setColor(color2);
    for (int az = 0, i = 0; az < 360; az += 30) {
        ps.setX((int)(r * sin(az * D2R) + 0.5));
        ps.setY((int)(-r * cos(az * D2R) + 0.5));
        pen.setStyle(Qt::DotLine);
        c.setPen(pen);
        c.drawLine(p.x(), p.y(), ps.x(), ps.y());
        ps.rx() +=  3 * sin(az * D2R);
        ps.ry() += -3 * cos(az * D2R);
        s = QString::number(az);
        if (!(az % 90)) s = dir[i++];
        drawText(c, ps, s, color2, Alignment::Center, Alignment::Bottom, -az);
	}
    pen.setStyle(Qt::SolidLine);
    c.setPen(pen);
}
//---------------------------------------------------------------------------
void Graph::drawSkyPlot(QPainter &c, double x, double y, const QColor &color1, const QColor &color2,
            double size)
{
    QPoint p;

    toPoint(x, y, p);

    drawSkyPlot(c, p, color1, color2, size / xScale);
}
//---------------------------------------------------------------------------
void Graph::drawSkyPlot(QPainter &c, const QPoint &p, const QColor &color1, const QColor &color2,
            const QColor &bgcolor, int size)
{
    QPen pen = c.pen();

    pen.setColor(color1);
    c.setPen(pen);
    c.setBrush(Qt::NoBrush);

    QString s, dir = QT_TR_NOOP("NESW");
    QPoint ps;
    int r = size / 2;

    for (int el = 0; el < 90; el += 15) {
        int ys = r - r * el / 90;
        pen.setStyle(el == 0 ? Qt::SolidLine : Qt::DotLine);
        c.setPen(pen);
        c.drawEllipse(p.x() - ys, p.y() - ys, 2 * ys, 2 * ys);
        if (el <= 0) continue;

        ps.setX(p.x());
        ps.setY(p.y() - ys);

        drawText(c, ps, QString::number(el) + "°", color2, bgcolor, Alignment::Left, Alignment::Center, 0);
	}
    pen.setStyle(Qt::DotLine);
    pen.setColor(color2);
    c.setPen(pen);
    for (int az = 0, i = 0; az < 360; az += 30) {
        ps.setX((int)(p.x() + r * sin(az * D2R) + 0.5));
        ps.setY((int)(p.y() - r * cos(az * D2R) + 0.5));
        c.drawLine(p.x(), p.y(), ps.x(), ps.y());
        ps.rx() +=  3 * sin(az * D2R);
        ps.ry() += -3 * cos(az * D2R);
        s = QString::number(az) + "°";
        if (!(az % 90)) s = dir[i++];
        drawText(c, ps, s, color2, bgcolor, Alignment::Center, Alignment::Bottom, -az);
	}
    pen.setStyle(Qt::SolidLine);
}
//---------------------------------------------------------------------------
void Graph::drawSkyPlot(QPainter &c, double x, double y, const QColor &color1, const QColor &color2,
            const QColor &bgcolor, double size)
{
    QPoint p;

    toPoint(x, y, p);
    drawSkyPlot(c, p, color1, color2, bgcolor, size / xScale);
}
//---------------------------------------------------------------------------
