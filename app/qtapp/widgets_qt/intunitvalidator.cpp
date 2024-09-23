#include "intunitvalidator.h"

IntUnitValidator::IntUnitValidator(const QString &suffix, QObject *parent): QIntValidator(parent), m_suffix(suffix)
{
};

IntUnitValidator::IntUnitValidator(int minimum, int maximum, const QString &suffix, QObject *parent)
    : QIntValidator(minimum, maximum, parent), m_suffix(suffix)
{
};

QValidator::State IntUnitValidator::validate(QString &input, int &pos) const
{
    QString text = stripped(input);

    return QIntValidator::validate(text, pos);
}

void IntUnitValidator::fixup(QString &input) const
{
    QString text = stripped(input);

    QIntValidator::fixup(text);

    input = text + m_suffix;
}

QString IntUnitValidator::stripped(const QString &input) const
{
    QString text = input;
    int size;

    if (m_suffix.size() && text.endsWith(m_suffix)) {
        size = text.size() - m_suffix.size();
        text = text.mid(0, size).trimmed();
    }

    return text;
}
