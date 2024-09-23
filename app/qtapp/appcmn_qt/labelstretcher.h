#ifndef LABELSTRETCHER_H
#define LABELSTRETCHER_H

#include <QObject>

class QLayout;

class LabelStretcher : public QObject {
    Q_OBJECT
    static constexpr const char kMinimumsAcquired[] = "ls_minimumsAcquired";
    static constexpr const char kStretcherManaged[] = "ls_stretcherManaged";
public:
    LabelStretcher(QObject *parent = 0);
    void apply(QWidget *widget);
    void setManaged(QWidget *w, bool managed = true);
    void resized(QWidget *widget);

protected:
    bool eventFilter(QObject * obj, QEvent * ev) override;

private:
    void onLayout(QLayout *layout, const std::function<void(QWidget*)> &onWidget);
    void setFont(QLayout *layout, const QFont &font);
    void setFont(QWidget *widget, const QFont &font);
    void setMinimumSize(QWidget *widget);
    static int dSize(const QSizeF & inner, const QSizeF & outer);
    qreal f(qreal fontSize, QWidget *widget);
    qreal df(qreal fontSize, qreal dStep, QWidget *widget);
};

#endif // LABELSTRETCHER_H
