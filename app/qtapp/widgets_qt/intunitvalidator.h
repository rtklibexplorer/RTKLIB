#ifndef DOUBLEUNITVALIDATOR_H
#define DOUBLEUNITVALIDATOR_H

#include <QIntValidator>

class IntUnitValidator : public QIntValidator
{
    Q_OBJECT
public:
    IntUnitValidator(const QString &suffix = "", QObject *parent = nullptr);
    IntUnitValidator(int minimum, int maximum, const QString &suffix, QObject *parent = nullptr);

    virtual QValidator::State validate(QString &input, int &pos) const override;
    void fixup(QString &input) const override;
protected:
    QString stripped(const QString &) const;

    QString m_suffix;
};

#endif // DOUBLEUNITVALIDATOR_H
