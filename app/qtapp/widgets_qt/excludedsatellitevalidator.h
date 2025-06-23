#ifndef EXCLUDEDSATELLITEVALIDATOR_H
#define EXCLUDEDSATELLITEVALIDATOR_H

#include <QObject>
#include <QValidator>

class ExcludedSatelliteValidator: public QValidator
{
public:
    ExcludedSatelliteValidator(QObject *parent = nullptr);
    void fixup(QString &input) const;
    QValidator::State 	validate(QString &input, int &pos) const;
};

#endif // EXCLUDEDSATELLITEVALIDATOR_H
