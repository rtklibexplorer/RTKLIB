//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include <stdio.h>

#include <QProcess>
#include <QUrl>

#include "tcpoptdlg.h"
#include "mntpoptdlg.h"

#include "ui_tcpoptdlg.h"

#include "rtklib.h"

//---------------------------------------------------------------------------

#define NTRIP_TIMEOUT   10000                           // response timeout (ms)
#define NTRIP_CYCLE             50                      // processing cycle (ms)
#define MAXSRCTBL               512000                  // max source table size (bytes)
#define ENDSRCTBL               "ENDSOURCETABLE"        // end marker of table
#define MAXLINE                 1024                    // max line size (byte)

//---------------------------------------------------------------------------
TcpOptDialog::TcpOptDialog(QWidget *parent, int options)
    : QDialog(parent), ui(new Ui::TcpOptDialog)
{
    ui->setupUi(this);

    mntpOptDialog = new MntpOptDialog(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &TcpOptDialog::btnOkClicked);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &TcpOptDialog::reject);
    connect(ui->btnNtrip, &QPushButton::clicked, this, &TcpOptDialog::btnNtripClicked);
    connect(ui->btnMountpointOptions, &QPushButton::clicked, this, &TcpOptDialog::btnMountpointClicked);

    setOptions(options);
}
//---------------------------------------------------------------------------
void TcpOptDialog::setOptions(int options)
{
    QString ti[] = {tr("TCP Server Options "),
                    tr("TCP Client Options"),
                    tr("NTRIP Server Options"),
                    tr("NTRIP Client Options"),
                    tr("NTRIP Caster Client Options"),
                    tr("NTRIP Caster Server Options"),
                    tr("UDP Server Options"),
                    tr("UDP Client Options")};

    ui->lblAddress->setText((options >= 2 && options <= 5) ? tr("NTRIP Caster Address") : tr("Server Address"));
    ui->lblAddress->setEnabled((options >= 1 && options <= 3) || options == 7);

    ui->cBAddress->setEnabled((options >= 1 && options <= 3) || options == 7);
    ui->lblMountPoint->setEnabled(options >= 2 && options <= 4);
    ui->cBMountPoint->setEnabled(options >= 2 && options <= 4);
    ui->lblUser->setEnabled(options >= 3 && options <= 4);
    ui->lEUser->setEnabled(options >= 3 && options <= 4);
    ui->lblPassword->setEnabled(options >= 2 && options <= 5);
    ui->lEPassword->setEnabled(options >= 2 && options <= 5);
    ui->btnNtrip->setVisible((options == 3));
    ui->btnBrowse->setVisible((options == 3));
    ui->btnMountpointOptions->setVisible((options >= 2 && options <= 4));
    ui->btnNtrip->setVisible(options == 2 || options == 3);
    setWindowTitle(ti[options]);

    showOptions = options;
}

//---------------------------------------------------------------------------
void TcpOptDialog::setHistory(QString tcpHistory[], int size)
{
    ui->cBAddress->clear();

    for (int i = 0; i < qMin(size, MAXHIST); i++) {
        this->history[i] = tcpHistory[i];
        if (!history[i].isEmpty())
            ui->cBAddress->addItem(history[i]);
    }
}
//---------------------------------------------------------------------------
QString* TcpOptDialog::getHistory()
{
    return history;
}
//---------------------------------------------------------------------------
void TcpOptDialog::setPath(QString path)
{
    int userpwEnd = path.lastIndexOf("@");
    int addrStart = userpwEnd < 0 ? 0 : userpwEnd + 1;
    int mntpntStart = path.indexOf("/", addrStart);

    QString addrport;
    QString mntpnt;
    QString mntpntstr;
    if (mntpntStart >= 0) {
        int pathStart = mntpntStart + 1;
        if (showOptions == OPT_NTRIP_SERVER || showOptions == OPT_NTRIP_CASTER_CLIENT) {
            int mntpntEnd = path.indexOf(":", pathStart);
            if (mntpntEnd >= pathStart) {
                mntpnt = path.mid(pathStart, mntpntEnd - pathStart);
                mntpntstr = path.mid(mntpntEnd + 1);
            } else {
                mntpnt = path.mid(pathStart);
            }
        } else {
            mntpnt = path.mid(pathStart);
        }
        ui->cBMountPoint->setCurrentText(mntpnt);
        addrport = path.mid(addrStart, mntpntStart - addrStart);
    } else {
        addrport = path.mid(addrStart);
    }
    ui->cBMountPoint->insertItem(0, mntpnt, mntpntstr);
    ui->cBMountPoint->setCurrentText(mntpnt);

    QString user, password;
    if (userpwEnd >= 0) {
        QString userpasswd = path.mid(0, userpwEnd);
        int userEnd = userpasswd.indexOf(":");
        if (userEnd >= 0 ) {
            user = userpasswd.mid(0, userEnd);
            password = userpasswd.mid(userEnd + 1);
        } else {
            user = userpasswd;
        }
    }
    ui->lEUser->setText(user);
    ui->lEPassword->setText(password);

    int port = 0;
    int portSep = addrport.indexOf(":");
    if (portSep >= 0)
        port = addrport.mid(portSep + 1).toInt();
    ui->sBPort->setValue(port);

    QString addr = addrport.mid(0, portSep);
    ui->cBAddress->insertItem(0, addr);
    ui->cBAddress->setCurrentText(addr);
    addHistory(ui->cBAddress, history);
}
//---------------------------------------------------------------------------
QString TcpOptDialog::getPath() {
    QString path;
    QString user = ui->lEUser->text();
    QString password = ui->lEPassword->text();
    if (!user.isEmpty() || !password.isEmpty()) {
        path = user;
        if (!password.isEmpty())
            path = QString("%1:%2").arg(path, password);
        path += "@";
    }
    path = QString("%1%2").arg(path, ui->cBAddress->currentText());
    QString port = ui->sBPort->text();
    if (!port.isEmpty())
        path = QString("%1:%2").arg(path, port);
    QString mntpnt = ui->cBMountPoint->currentText();
    QString str = ui->cBMountPoint->currentData().toString();
    if (!mntpnt.isEmpty() || !str.isEmpty()) {
        path = QString("%1/%2").arg(path, mntpnt);
        if (!str.isEmpty())
            path = QString("%1:%2").arg(path, str);
    }
    return path;
}

//---------------------------------------------------------------------------
void TcpOptDialog::btnOkClicked()
{
    addHistory(ui->cBAddress, history);

    accept();
}
//---------------------------------------------------------------------------
void TcpOptDialog::addHistory(QComboBox *list, QString *hist)
{
    for (int i = 0; i < MAXHIST; i++) {
        if (list->currentText() != hist[i]) continue;
        for (int j = i + 1; j < MAXHIST; j++)
            hist[j - 1] = hist[j];
        hist[MAXHIST - 1] = "";
	}
    for (int i = MAXHIST - 1; i > 0; i--)
        hist[i] = hist[i - 1];
    hist[0] = list->currentText();

    list->clear();
    for (int i = 0; i < MAXHIST; i++)
        if (!hist[i].isEmpty()) list->addItem(hist[i]);
}
//---------------------------------------------------------------------------
void TcpOptDialog::btnNtripClicked()
{
    QPushButton *btn = (QPushButton *)sender();
    QString path = ui->cBAddress->currentText() + ":" + ui->sBPort->text();
    stream_t str;
    uint32_t tick = tickget();
    static char buff[MAXSRCTBL];
    char *p = buff, mntpnt[256];

    strinit(&str);
    if (!stropen(&str, STR_NTRIPCLI, STR_MODE_R, qPrintable(path))) return;

    btn->setEnabled(false);
    *p = '\0';
    while (p < buff + MAXSRCTBL - 1) {
        p += strread(&str, (uint8_t *)p, buff + MAXSRCTBL - p - 1);
        *p = '\0';
        sleepms(NTRIP_CYCLE);
        if (strstr(buff, ENDSRCTBL)) break;
        if ((int)(tickget() - tick) > NTRIP_TIMEOUT) break;
    }
    strclose(&str);

    ui->cBMountPoint->clear();
    for (p = buff; (p  = strstr(p, "STR;")); p+=4) {
        if (sscanf(p, "STR;%255[^;]", mntpnt) == 1) {
            ui->cBMountPoint->addItem(mntpnt, QString(p).split('\n', Qt::SkipEmptyParts).first());
        }
    }
    btn->setEnabled(true);
}
//---------------------------------------------------------------------------
void TcpOptDialog::btnBrowseClicked()
{
    QString addrText = ui->cBAddress->currentText();
    QString portText = ui->sBPort->text();

    if (!portText.isEmpty()) addrText += ":" + portText;
    execCommand("srctblbrows_qt", QStringList(addrText), 1);
}
//---------------------------------------------------------------------------
void TcpOptDialog::btnMountpointClicked()
{
    mntpOptDialog->setMountPoint(ui->cBMountPoint->currentText());
    mntpOptDialog->setMountPointString(ui->cBMountPoint->currentData().toString());

    mntpOptDialog->exec();
    if (mntpOptDialog->result()!=QDialog::Accepted) return;

    ui->cBMountPoint->setCurrentText(mntpOptDialog->getMountPoint());
    ui->cBMountPoint->setItemData(ui->cBMountPoint->currentIndex(), mntpOptDialog->getMountPointString());
}
//---------------------------------------------------------------------------
int TcpOptDialog::execCommand(const QString &cmd, const QStringList &opt, int show)
{
    QProcess prog;

    Q_UNUSED(show);

    prog.start(cmd, opt); /* FIXME: show option not yet supported */
    return 1;
}
//---------------------------------------------------------------------------
