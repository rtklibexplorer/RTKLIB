#include "scientificspinbox.h"
#include <QLineEdit>
#include <qlocale.h>
#include <math.h>

#include <limits>

ScientificSpinBox::ScientificSpinBox(QWidget * parent)
    : QDoubleSpinBox(parent)
{
    setDecimals(2);
    QDoubleSpinBox::setDecimals(1000);  // internal number of decimals

    setRange(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());

    m_validator = new QDoubleValidator(this);
    m_validator->setDecimals(1000);
    m_validator->setNotation(QDoubleValidator::ScientificNotation);
    this->lineEdit()->setValidator(m_validator);
}

int ScientificSpinBox::decimals() const
{
    return m_displayDecimals;
}

void ScientificSpinBox::setDecimals(int value)
{
    m_displayDecimals = value;
}

// overwrites virtual function of QAbstractSpinBox
void ScientificSpinBox::stepBy(int steps)
{
    setValue(value() * pow(10, steps));
}

QString ScientificSpinBox::textFromValue(double value) const
{
    QString str = locale().toString(value, 'g', m_displayDecimals);

    // remove thousand sign
    if (qAbs(value) >= 1000.0) {
        str.remove(locale().groupSeparator());
    }
    return str;
}

double ScientificSpinBox::valueFromText(const QString &text) const
{
    QString copy = text;
    int pos = this->lineEdit()->cursorPosition();
    QValidator::State state = QValidator::Acceptable;
    return validateAndInterpret(copy, pos, state).toDouble();
}

// overwrites virtual function of QAbstractSpinBox
QValidator::State ScientificSpinBox::validate(QString &text, int &pos) const
{
    QValidator::State state;
    validateAndInterpret(text, pos, state);
    return state;
}

// overwrites virtual function of QAbstractSpinBox
void ScientificSpinBox::fixup(QString &input) const
{
    input.remove(locale().groupSeparator());
}

/*!
    \internal Multi purpose function that parses input, sets state to
    the appropriate state and returns the value it will be interpreted
    as.
*/
// reimplemented function, copied from QDoubleSpinBoxPrivate::validateAndInterpret
QVariant ScientificSpinBox::validateAndInterpret(QString &input, int &pos, QValidator::State &state) const
{
    QLocale loc(locale());
    static QString cachedText;
    static QValidator::State cachedState;
    static QVariant cachedValue;

    if (cachedText == input && !input.isEmpty()) {
        state = cachedState;
        return cachedValue;
    }
    const double max = maximum();
    const double min = minimum();

    // removes prefix & suffix
    QString copy = stripped(input, &pos);

    int len = copy.size();
    double num = min;
    const bool plus = max >= 0;
    const bool minus = min <= 0;

    // Test possible 'Intermediate' reasons
    switch (len)
    {
    case 0:
        // Length 0 is always 'Intermediate', except for min=max
        state = max != min ? QValidator::Intermediate : QValidator::Invalid;
        goto end;
    case 1:
        // if only char is '+' or '-'
        if (copy.at(0) == loc.decimalPoint()
            || (plus && copy.at(0) == QLatin1Char('+'))
            || (minus && copy.at(0) == QLatin1Char('-'))) {
            state = QValidator::Intermediate;
            goto end;
        }
        break;
    case 2:
        // if only chars are '+' or '-' followed by Comma seperator (delimiter)
        if (copy.at(1) == loc.decimalPoint()
            && ((plus && copy.at(0) == QLatin1Char('+')) || (minus && copy.at(0) == QLatin1Char('-')))) {
            state = QValidator::Intermediate;
            goto end;
        }
        break;
    default: break;
    } // end switch


    // First char must not be thousand-char
    if (copy.at(0) == loc.groupSeparator())
    {
        state = QValidator::Invalid;
        goto end;
    }
    // Test possible 'Invalid' reasons
    else if (len > 1)
    {
        const int dec = copy.indexOf(loc.decimalPoint()); // position of delimiter
        // if decimal separator exists
        if (dec != -1) {
            // reject more than one delimiter
            if (dec + 1 < copy.size() && copy.at(dec + 1) == loc.decimalPoint() && pos == dec + 1) {
                copy.remove(dec + 1, 1); // typing a delimiter when you are on the delimiter
            }							 // should be treated as typing right arrow
            // too many decimal points
            if (copy.size() - dec > QDoubleSpinBox::decimals() + 1) {
                state = QValidator::Invalid;
                goto end;
            }
            // after decimal separator no group separator
            for (int i=dec + 1; i<copy.size(); ++i) {
                if (copy.at(i).isSpace() || copy.at(i) == loc.groupSeparator()) {
                    state = QValidator::Invalid;
                    goto end;
                }
            }
            // if no decimal separator exists
        } else {
            const QChar &last = copy.at(len - 1);
            const QChar &secondLast = copy.at(len - 2);
            // group of two thousand or space chars is invalid
            if ((last == loc.groupSeparator() || last.isSpace())
                && (secondLast == loc.groupSeparator() || secondLast.isSpace())) {
                state = QValidator::Invalid;
                goto end;
            }
            // two space chars is invalid
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
            else if (last.isSpace() && (!loc.groupSeparator().simplified().isEmpty() || secondLast.isSpace())) {
#else
            else if (last.isSpace() && (!loc.groupSeparator().isNull() || secondLast.isSpace())) {
#endif
                state = QValidator::Invalid;
                goto end;
            }
        }
    } // end if (len > 1)

    // block of remaining test before 'end' mark
    {
        bool ok = false;
        bool notAcceptable = false;

        // convert 'copy' to double, and check if that was 'ok'
        num = loc.toDouble(copy, &ok);

        // conversion to double did fail
        if (!ok) {
            // maybe group separator caused failure
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
            if (loc.groupSeparator().at(0).isPrint())
#else
            if (loc.groupSeparator().isPrint())
#endif
            {
                // if no group separator is possible but exist
                if (max < 1000 && min > -1000 && copy.contains(loc.groupSeparator())) {
                    state = QValidator::Invalid;
                    goto end;
                }

                // two thousand-chars after each others
                const int len = copy.size();
                for (int i=0; i<len- 1; ++i) {
                    if (copy.at(i) == loc.groupSeparator() && copy.at(i + 1) == loc.groupSeparator()) {
                        state = QValidator::Invalid;
                        goto end;
                    }
                }

                // remove thousand-chars
                QString copy2 = copy;
                copy2.remove(loc.groupSeparator());
                num = loc.toDouble(copy2, &ok);

                // if conversion still not valid, then reason unknown -> Invalid
                if (!ok) {
                    state = QValidator::Invalid;
                    goto end;
                }
                notAcceptable = true; // -> state = Intermediate
            }
        }

        // no thousand sign, but still invalid for unknown reason
        if (!ok) {
            state = QValidator::Invalid;
        }
        // number valid and within valid range
        else if (num >= min && num <= max) {
            if (notAcceptable) {
                state = QValidator::Intermediate; // conversion to num initially failed
            } else {
                state = QValidator::Acceptable;
            }
        }
        // when max and min is the same the only non-Invalid input is max (or min)
        else if (max == min) {
            state = QValidator::Invalid;
        } else {
            // value out of valid range (coves only special cases)
            if ((num >= 0 && num > max) || (num < 0 && num < min)) {
                state = QValidator::Invalid;
            } else {
                state =  QValidator::Intermediate;
            }
        }
    }

end:
    // if something went wrong, set num to something valid
    if (state != QValidator::Acceptable) {
        num = max > 0 ? min : max;
    }

    // save (private) cache values
    cachedText = prefix() + copy + suffix();
    cachedState = state;
    cachedValue = QVariant(num);
    // return resulting valid num
    return QVariant(num);
}

/*!
    \internal
    Strips any prefix/suffix from \a text.
*/
// reimplemented function, copied from QAbstractSpinBoxPrivate::stripped
QString ScientificSpinBox::stripped(const QString &t, int *pos) const
{
    QString text = t;
    QString prefixtext = prefix();
    QString suffixtext = suffix();

    if (specialValueText().size() == 0 || text != specialValueText()) {
        int from = 0;
        int size = text.size();
        bool changed = false;
        if (prefixtext.size() && text.startsWith(prefixtext)) {
            from += prefixtext.size();
            size -= from;
            changed = true;
        }
        if (suffixtext.size() && text.endsWith(suffixtext)) {
            size -= suffixtext.size();
            changed = true;
        }
        if (changed)
            text = text.mid(from, size);
    }

    const int s = text.size();
    text = text.trimmed();
    if (pos)
        (*pos) -= (s - text.size());
    return text;
}
