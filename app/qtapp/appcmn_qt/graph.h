//---------------------------------------------------------------------------
#ifndef graphH
#define graphH

#include <QString>
#include <QColor>
#include <QPoint>
#include <QLabel>

//---------------------------------------------------------------------------
class Graph // graph class
{
public:
    explicit Graph(QPaintDevice *parent);

    enum MarkerTypes {
        Dot = 0, // (.)
        Circle = 1, // (o)
        Rect = 2,  // (#)
        Cross = 3, // (x)
        Line = 4, // (-)
        Plus = 5, // (+)
        Arrow = 10, // (->)
        HScale = 11,
        VScale = 12,
        Compass = 13
    };
    enum LabelPosition {
        Off = 0,
        Outer = 1,
        Inner = 2,
        OuterRot = 3,
        InnerRot = 4,
        None = 5,
        Time = 6,
        Axis = 7
    };
    enum Alignment {
        Center = 0,
        Left = 1,
        Right = 2,
        Bottom = 1,
        Top = 2,
    };

    bool isInArea(const QPoint &p);
    int toPoint(double x, double y, QPoint &p);
    int onAxis(const QPoint &p);
    QString numText(double x, double dx);
    QString timeText(double x, double dx);
    void toPos(const QPoint &p, double &x, double &y);
    void setSize(int width, int height);
    void resize();
    void setPosition(const QPoint &p1, const QPoint &p2);
    void getExtent(QPoint &p1, QPoint &p2);
    void setCenter(const double x, const double y);
    void getCenter(double &x, double &y);
    void setRight(const double x, const double y);
    void getRight(double &x, double &y);
    void setScale(const double xs, const double ys);
    void getScale(double &xs, double &ys);
    void setLimits(const double *xl, const double *yl);
    void getLimits(double *xl, double *yl);
    void setTick(const double xt, const double yt);
    void getTick(double &xt, double &yt);
    void setLabelUnits(const QString & xUnit, const QString & yUnit);
    void getLabelUnits(QString &xUnit, QString &yUnit);
    void drawAxis(QPainter &c, bool label, bool glabel);
    void drawMark(QPainter &c, const QPoint &p, int mark, const QColor &color, int size, int rot);
    void drawMark(QPainter &c, double x, double y, int mark, const QColor &color, int size, int rot);
    void drawMark(QPainter &c, const QPoint &p, int mark, const QColor &color, const QColor &bgcolor, int size,int rot);
    void drawMark(QPainter &c, double x, double y, int mark, const QColor &color, const QColor &bgcolor,int size, int rot);
    void drawMarks(QPainter &c, const double *x, const double *y,const QColor *colors, int n,
               int mark, int size, int rot);
    void drawCircle(QPainter &c, const QPoint &p, const QColor &color, int rx, int ry, int style);
    void drawCircle(QPainter &c, double x, double y, const QColor &color, double rx, double ry, int style);
    void drawCircles(QPainter &c, int label);
    void drawText(QPainter &c, double x, double y, const QString &str, const QColor &color, int ha, int va, int rot);
    void drawText(QPainter &c, double x, double y, const QString &str, const QColor &color, int ha, int va, int rot, const QFont &font);
    void drawText(QPainter &c, const QPoint &p, const QString &str, const QColor &color, int ha, int va,int rot);
    void drawText(QPainter &c, const QPoint &p, const QString &str, const QColor &color, int ha, int va,int rot, const QFont &font);
    void drawText(QPainter &c, double x, double y, const QString &str, const QColor &color, const QColor &bgcolor,int ha, int va, int rot);
    void drawText(QPainter &c, const QPoint &p, const QString &str, const QColor &color, const QColor &bgcolor,int ha, int va, int rot);
    void drawPoly(QPainter &c, QPoint *p, int n, const QColor &color, int style);
    void drawPoly(QPainter &c, double *x, double *y, int n, const QColor &color, int style);
    void drawPolyline(QPainter &c, QPoint *p, int n);
    void drawPatch(QPainter &c, QPoint *p, int n, const QColor &color1, const QColor &color2, int style);
    void drawPatch(QPainter &c, double *x, double *y, int n, const QColor &color1, const QColor &color2, int style);
    void drawSkyPlot(QPainter &c, const QPoint &p, const QColor &color1, const QColor &color2, int size);
    void drawSkyPlot(QPainter &c, double x, double y, const QColor &color1, const QColor &color2, double size);
    void drawSkyPlot(QPainter &c, const QPoint &p, const QColor &color1, const QColor &color2, const QColor &bgcolor, int size);
    void drawSkyPlot(QPainter &c, double x, double y, const QColor &color1, const QColor &color2, const QColor &bgcolor,double size);

    bool box;  // show box
    bool fit;  // fit scale on resize
    bool xGrid, yGrid;  // show grid
    int xLabelPosition, yLabelPosition; // grid label pos (0:off, 1:outer, 2:inner, 3:outer-rot, 4:inner-rot, 5/6:time, 7:axis)
    int week;  // gpsweek no. for time label
    QString title, xLabel, yLabel;  // lable string ("":no label)
    QColor color[3];  // background color, grid color, and title/label color

private:
    double autoTick(double scale);
    double autoTickTime(double scale);
    void drawBox(QPainter &c);
    void drawLabel(QPainter &c);
    void drawGrid(QPainter &c,double xt, double yt);
    void drawGridLabel(QPainter &c, double xt, double yt);
    void rotatePoints(QPoint *ps, int n, const QPoint &pc, int rot, QPoint *pr);
    int clipPoint(QPoint *p0, int area, QPoint *p1);

    QPoint p_;
    QPaintDevice *parent;
    int x, y, width, height;
    double xCenter, yCenter;  // center coordinate (unit)
    double xScale, yScale;  // scale factor (unit/pixel)
    double xTick, yTick;   // grid interval (unit) (0:auto)
    QString xLabelUnit, yLabelUnit;
};
#endif
