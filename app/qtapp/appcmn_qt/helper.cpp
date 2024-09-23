#include "helper.h"

#include "rtklib.h"

#include <math.h>

#include <QLabel>
#include <QComboBox>
#include <QStandardItemModel>
#include <QMessageBox>


//---------------------------------------------------------------------------
QString color2String(const QColor &c)
{
    return QStringLiteral("rgb(%1,%2,%3)").arg(c.red()).arg(c.green()).arg(c.blue());
}
//---------------------------------------------------------------------------
void setWidgetBackgroundColor(QWidget * label, const QColor &color)
{
    // use palett instead of stylesheet here for speed reasons
    if (true) {
        label->setAutoFillBackground(true);
        QPalette palette = label->palette();
        palette.setColor(label->backgroundRole(), color);
        label->setPalette(palette);
    } else
        label->setStyleSheet(QStringLiteral("QLabel {background-color: %1}").arg(color2String(color)));
}
//---------------------------------------------------------------------------
void setWidgetTextColor(QWidget * label, const QColor &color)
{
    // use palett instead of stylesheet here for speed reasons
    if (true) {
        QPalette palette = label->palette();
        palette.setColor(label->foregroundRole(), color);
        label->setPalette(palette);
    } else
        label->setStyleSheet(QStringLiteral("QLabel {color: %1}").arg(color2String(color)));
}

// convert degree to deg-min-sec --------------------------------------------
void degtodms(double deg, double *dms)
{
    double sgn = 1.0;

    if (deg < 0.0) {
        deg = -deg;
        sgn = -1.0;
    }
    dms[0] = floor(deg);
    dms[1] = floor((deg - dms[0]) * 60.0);
    dms[2] = (deg - dms[0] - dms[1] / 60.0) * 3600;
    dms[0] *= sgn;
}

void setComboBoxItemEnabled(QComboBox * comboBox, int index, bool enabled)
{
    auto * model = qobject_cast<QStandardItemModel*>(comboBox->model());
    assert(model);
    if(!model) return;

    auto * item = model->item(index);
    assert(item);
    if(!item) return;
    item->setEnabled(enabled);
}

bool check_compression(const QString &filename)
{
    char tmpfile[1024];
    int cstat;
    bool ret = true;

    if ((cstat=rtk_uncompress(qPrintable(filename), tmpfile)) == -1){
        QMessageBox::information(NULL,
                                 QObject::tr("Decompression tools missing"),
                                 QObject::tr("You need to have <i>gzip</i>, <i>tar</i>, and <i>crx2rnx</i> installed to work with compressed rinex files.<br><i>crx2rnx</i> can e.g. be found at "
                                             "<a href = \"https://terras.gsi.go.jp/ja/crx2rnx.html\">https://terras.gsi.go.jp/ja/crx2rnx.html</a>."));
        ret = false;
    }
    /* delete temporary file */
    if (cstat) remove(tmpfile);

    return ret;
}
