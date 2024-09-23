#include "labelstretcher.h"

// https://github.com/KubaO/stackoverflown/tree/master/questions/label-text-size-vert-40861305
// adapted by Jens Reimann to support vertical streching of widgets
#include <QWidget>
#include <QEvent>
#include <QLayout>
#include <QVariant>
#include <QDebug>

#include <math.h>

LabelStretcher::LabelStretcher(QObject *parent) : QObject(parent) {
    apply(qobject_cast<QWidget*>(parent));
}

void LabelStretcher::apply(QWidget *widget) {
    if (!widget) return;
    setManaged(widget);
    setMinimumSize(widget);
    widget->installEventFilter(this);
}

void LabelStretcher::setManaged(QWidget *w, bool managed) {
    w->setProperty(kStretcherManaged, managed);
}

bool LabelStretcher::eventFilter(QObject * obj, QEvent * ev) {
    auto widget = qobject_cast<QWidget*>(obj);

    if (widget && (ev->type() == QEvent::Resize || ev->type() == QEvent::Show))
        resized(widget);

    return false;
}

void LabelStretcher::onLayout(QLayout *layout, const std::function<void(QWidget*)> &onWidget) {
    if (!layout) return;

    auto N = layout->count();
    for (int i = 0; i < N; ++i) {
        auto item = layout->itemAt(i);
        onWidget(item->widget());
        onLayout(item->layout(), onWidget);
    }
}

void LabelStretcher::setFont(QLayout *layout, const QFont &font) {
    onLayout(layout, [&](QWidget *widget){ setFont(widget, font); });
}

void LabelStretcher::setFont(QWidget *widget, const QFont &font) {
    if (!widget || !widget->property(kStretcherManaged).toBool())
        return;

    float scale = 1;
    if (widget->sizePolicy().verticalStretch() != 0) {
        scale = (double)widget->sizePolicy().verticalStretch()/10.;
    }

    QFont f(font);
    f.setPointSizeF(font.pointSizeF()*scale);

    widget->setFont(f);
    setFont(widget->layout(), font);
}

void LabelStretcher::setMinimumSize(QWidget *widget) {
    if (!widget)
        return;
    if (widget->layout())
        return;
    widget->setMinimumSize(widget->minimumSizeHint());
}

int LabelStretcher::dSize(const QSizeF & inner, const QSizeF & outer) {
    auto dy = inner.height() - outer.height();
    auto dx = inner.width() - outer.width();
    return std::max(dx, dy);
}

qreal LabelStretcher::f(qreal fontSize, QWidget *widget) {
    auto font = widget->font();
    font.setPointSizeF(fontSize);
    setFont(widget, font);
    auto d = dSize(widget->sizeHint(), widget->size());
    qDebug() << "f:" << fontSize << "d" << d;
    return d;
}

qreal LabelStretcher::df(qreal fontSize, qreal dStep, QWidget *widget) {
    fontSize = std::max(dStep + 1.0, fontSize);
    return (f(fontSize + dStep, widget) - f(fontSize - dStep, widget)) / dStep;
}

void LabelStretcher::resized(QWidget *widget) {
    qDebug() << "pre: " << widget->minimumSizeHint() << widget->sizeHint() << widget->size();
    if (!widget->property(kMinimumsAcquired).toBool()) {
        onLayout(widget->layout(), [=](QWidget *widget){ setMinimumSize(widget); });
        widget->setProperty(kMinimumsAcquired, true);
    }

    if (!widget->isVisible())
        return;

    // Newton's method
    auto font = widget->font();
    auto fontSize = font.pointSizeF();
    qreal dStep = 1.0;
    int i;
    for (i = 0; i < 5; ++i) {
        auto prevFontSize = fontSize;
        auto d = df(fontSize, dStep, widget);
        if (d == 0) {
            dStep *= 2.0;
            continue;
        }
        fontSize -= f(fontSize, widget)/d;
        fontSize = std::max(dStep + 1.0, fontSize);
        auto change = fabs(prevFontSize - fontSize)/fontSize;
        qDebug() << "d:" << d << " delta" << change;
        if (change < 0.01) break; // we're within 1% of target
    }
    font.setPointSizeF(fontSize);  // use a font smaller than max
    setFont(widget, font);
    qDebug() << "post:" << i << widget->minimumSizeHint() << widget->sizeHint() << widget->size();
}

constexpr const char LabelStretcher::kMinimumsAcquired[];
constexpr const char LabelStretcher::kStretcherManaged[];
