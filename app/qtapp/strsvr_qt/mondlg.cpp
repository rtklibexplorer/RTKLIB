//---------------------------------------------------------------------------
#include <ctype.h>
#include <QScrollBar>

#include <stdio.h>

#include "mondlg.h"

#include "ui_mondlg.h"

//---------------------------------------------------------------------------

#define MAXLEN		200
#define MAXLINE		2048

//---------------------------------------------------------------------------
StrMonDialog::StrMonDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::StrMonDialog)
{
    ui->setupUi(this);

    connect(ui->btnClose, &QPushButton::clicked, this, &StrMonDialog::accept);
    connect(ui->btnClear, &QPushButton::clicked, this, &StrMonDialog::clearConsole);
    connect(ui->btnDown, &QPushButton::clicked, this, &StrMonDialog::scrollDown);
    connect(ui->cBSelectFormat, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &StrMonDialog::changeFormat);

    clearConsole();

    for (int i = 0; i <= MAXRCVFMT; i++) {
        ui->cBSelectFormat->addItem(formatstrs[i]);
    }
    memset(&rtcm, 0, sizeof(rtcm));
    memset(&raw, 0, sizeof(raw));
    rtcm.outtype = raw.outtype = 1;
}
//---------------------------------------------------------------------------
void StrMonDialog::changeFormat()
{
    int streamFormat = getStreamFormat();

    if ((streamFormat-3 == STRFMT_RTCM2) || (streamFormat-3 == STRFMT_RTCM3)) {
        free_rtcm(&rtcm);
    } else if (streamFormat >= 3) {
        free_raw(&raw);
    }

    streamFormat = ui->cBSelectFormat->currentIndex();
    clearConsole();

    if ((streamFormat - 3 == STRFMT_RTCM2) || (streamFormat - 3 == STRFMT_RTCM3)) {
        init_rtcm(&rtcm);
        rtcm.outtype = 1;
    } else if (streamFormat >= 3) {
        init_raw(&raw, streamFormat - 3);
        raw.outtype = 1;
    }
}
//---------------------------------------------------------------------------
void StrMonDialog::addMessage(unsigned char *msg, int len)
{
    int streamFormat = getStreamFormat();
    int i;

    if (len <= 0) {
        return;
    }else if (streamFormat - 3 == STRFMT_RTCM2) {
        for (i = 0; i < len; i++) {
            input_rtcm2(&rtcm, msg[i]);
            if (rtcm.msgtype[0]) {
                addConsole((unsigned char*)rtcm.msgtype, strlen(rtcm.msgtype), 1, true);
                rtcm.msgtype[0] = '\0';
            }
        }
    } else if (streamFormat - 3 == STRFMT_RTCM3) {
        for (i = 0; i < len; i++) {
            input_rtcm3(&rtcm, msg[i]);
            if (rtcm.msgtype[0]) {
                addConsole((unsigned char*)rtcm.msgtype, strlen(rtcm.msgtype), 1, true);
                rtcm.msgtype[0] = '\0';
            }
        }
    } else if (streamFormat >= 3) { // raw
        for (i = 0; i < len; i++) {
            input_raw(&raw, streamFormat - 3, msg[i]);
            if (raw.msgtype[0]) {
                addConsole((unsigned char*)raw.msgtype, strlen(raw.msgtype), 1, true);
                raw.msgtype[0] = '\0';
            }
        }
    } else if (streamFormat >= 1 ) { // HEX/ASC
        addConsole(msg, len, streamFormat - 1, false);
    } else { // Streams
        consoleBuffer.clear();
        addConsole(msg, len, 1, false);
    }
}
//---------------------------------------------------------------------------
void StrMonDialog::addConsole(unsigned char *msg, int n, int mode, bool newline)
{
    char buff[MAXLEN + 16], *p = buff;

    if (ui->btnStop->isChecked()) return;
    if (n <= 0) return;

    if (consoleBuffer.count() == 0) consoleBuffer.append(""); // make sure that there is always at least one line in consoleBuffer

    // fill buffer with last incomplete line
    p += sprintf(p, "%s", qPrintable(consoleBuffer.at(consoleBuffer.count() - 1)));

    for (int i = 0; i < n; i++) {
        if (mode) {
            if (msg[i] == '\r') continue;
            p += sprintf(p, "%c", (msg[i] == '\n' || isprint(msg[i])) ? msg[i] : '.');
        } else {  // add a space after 16 and a line break after 67 characters
            p += sprintf(p, "%s%02X", (p - buff) % 17 == 16 ? " " : "", msg[i]);
            if (p - buff >= 67) p += sprintf(p, "\n");
        }
        if (p - buff >= MAXLEN) p += sprintf(p, "\n");

        if (*(p - 1) == '\n') {
            consoleBuffer[consoleBuffer.count() - 1] = QString(buff).remove(QString(buff).size()-1, 1);
            consoleBuffer.append("");
            *(p = buff) = 0;
            while (consoleBuffer.count() >= MAXLINE) consoleBuffer.removeFirst();
        }
    }
    // store last (incomplete) line
    consoleBuffer[consoleBuffer.count() - 1] = QString(buff);

    // write consoleBuffer to table widget
    ui->tWConsole->setColumnCount(1);
    ui->tWConsole->setRowCount(consoleBuffer.size());
    for (int i = 0; i < consoleBuffer.size(); i++)
        ui->tWConsole->setItem(i, 0, new QTableWidgetItem(consoleBuffer.at(i)));

    if (ui->btnDown->isChecked())
       ui->tWConsole->verticalScrollBar()->setValue(ui->tWConsole->verticalScrollBar()->maximum());

    if (newline)
        consoleBuffer.append("");
}
//---------------------------------------------------------------------------
void StrMonDialog::clearConsole()
{
    consoleBuffer.clear();
    ui->tWConsole->clear();
    ui->tWConsole->setRowCount(0);
}
//---------------------------------------------------------------------------
void StrMonDialog::scrollDown()
{
    ui->tWConsole->verticalScrollBar()->setValue(ui->tWConsole->verticalScrollBar()->maximum());
    if (ui->btnDown->isChecked())
        ui->tWConsole->verticalScrollBar()->setEnabled(false);
    else
        ui->tWConsole->verticalScrollBar()->setEnabled(true);
}
//---------------------------------------------------------------------------

int StrMonDialog::getStreamFormat()
{
    return ui->cBSelectFormat->currentIndex();
}
//---------------------------------------------------------------------------
