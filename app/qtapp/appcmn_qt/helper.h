#ifndef helperH
#define helperH

#include <QColor>
class QWidget;
class QComboBox;

QString color2String(const QColor &c);
void setWidgetBackgroundColor(QWidget *, const QColor &color);
void setWidgetTextColor(QWidget *, const QColor &color);
void degtodms(double deg, double *dms);
void setComboBoxItemEnabled(QComboBox * comboBox, int index, bool enabled);
bool check_compression(const QString &filename);

namespace Color {
static const QColor Orange(0xff, 0xaa, 0x00);
static const QColor Aqua(0xff, 0xff, 0x00);
static const QColor Silver(0xc0, 0xc0, 0xc0);
static const QColor Teal(0x80, 0x80, 0x00);
static const QColor Fuchsia(0xff, 0x00, 0xff);
static const QColor Lime(0x00, 0xff, 0x00);
//static const QColor Window(0xa0, 0xa0, 0xa4);
static const QColor LightGray(0xdd, 0xdd, 0xdd);
}

#endif
