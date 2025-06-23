#include "excludedsatellitevalidator.h"

#include "rtklib.h"

ExcludedSatelliteValidator::ExcludedSatelliteValidator(QObject *parent) : QValidator(parent) {

}

void ExcludedSatelliteValidator::fixup(QString &input) const
{

}
QValidator::State ExcludedSatelliteValidator::validate(QString &input, int &pos) const
{
    if (!input.isEmpty()) {
        foreach (QString sat, input.split(' ')) {
            unsigned char ex;
            int satNo;
            if (sat.length() == 0) continue;
            if (sat[0] == '+')
            {
                sat = sat.mid(1);
            }
            if (!(satNo = satid2no(qPrintable(sat)))) return QValidator::State::Intermediate;
        }
        return QValidator::State::Acceptable;
    }
    return QValidator::State::Acceptable;
}
