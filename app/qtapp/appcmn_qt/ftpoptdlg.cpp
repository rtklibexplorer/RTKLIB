//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include <QShowEvent>
#include <QUrl>
#include <intunitvalidator.h>

#include "ftpoptdlg.h"
#include "keydlg.h"

#include "ui_ftpoptdlg.h"


//---------------------------------------------------------------------------
FtpOptDialog::FtpOptDialog(QWidget *parent, int options)
    : QDialog(parent), ui(new Ui::FtpOptDialog)
{
    ui->setupUi(this);

    keyDlg = new KeyDialog(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &FtpOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &FtpOptDialog::reject);
    connect(ui->btnKey, &QPushButton::clicked, this, &FtpOptDialog::showKeyDialog);

    ui->cBPathOffset->setValidator(new IntUnitValidator(" h", this));
    ui->cBInterval->setValidator(new IntUnitValidator(" h", this));
    ui->cBOffset->setValidator(new IntUnitValidator(" h", this));

    setOptions(options);
}
//---------------------------------------------------------------------------
void FtpOptDialog::setOptions(int options)
{
    QString cap[] = { tr("FTP Option"), tr("HTTP Option") };
    this->options = options;

    setWindowTitle(cap[options]);

    updateEnable();
}
//---------------------------------------------------------------------------
void FtpOptDialog::setPath(const QString &path)
{
    int topts[4] = { 0, 3600, 0, 0 };

    QStringList tokens = path.split("::");
    if (tokens.size() > 1) {
        QString t = tokens.at(1);
        if (t.startsWith("T=")) {
            QStringList values = t.mid(2).split(",");  // remove "T=" and split list:
            for (int i = 0; (i < 4) || (i < values.size()); i++)
                topts[i] = values.at(i).toInt();
        }
    }
    QUrl url(QString("ftp://") + tokens.at(0));

    ui->cBAddress->clear();
    ui->cBAddress->addItem(url.host() + url.path());
    for (int i = 0; i < MAXHIST; i++)
        if (history[i] != "") ui->cBAddress->addItem(history[i]);

    ui->cBAddress->setCurrentIndex(0);
    ui->lEUser->setText(url.userName());
    ui->lEPassword->setText(url.password());
    ui->cBPathOffset->setCurrentText(QString("%1 h").arg(topts[0] / 3600.0, 0, 'g', 2));
    ui->cBInterval->setCurrentText(QString("%1 h").arg(topts[1] / 3600.0, 0, 'g', 2));
    ui->cBOffset->setCurrentText(QString("%1 h").arg(topts[2] / 3600.0, 0, 'g', 2));
    ui->sBRetryInterval->setValue(topts[3]);
}
//---------------------------------------------------------------------------
QString FtpOptDialog::getPath()
{
    QString pathOffsetText = ui->cBPathOffset->currentText().split(" ").first();
    QString intervalText = ui->cBInterval->currentText().split(" ").first();
    QString offsetText = ui->cBOffset->currentText().split(" ").first();
    QString userText = ui->lEUser->text();
    QString passwordText = ui->lEPassword->text();
    QString addressText = ui->cBAddress->currentText();
    int topts[4] = {0, 0, 0, 0};
    bool ok;

    topts[0] = pathOffsetText.toInt(&ok) * 3600.0;
    topts[1] = intervalText.toInt(&ok) * 3600.0;
    topts[2] = offsetText.toInt(&ok) * 3600.0;
    topts[3] = ui->sBRetryInterval->value();

    return QString("%1:%2@%3::T=%4,%5,%6,%7")
        .arg(userText)
        .arg(passwordText)
        .arg(addressText)
        .arg(topts[0])
        .arg(topts[1])
        .arg(topts[2])
        .arg(topts[3]);
}
//---------------------------------------------------------------------------
void FtpOptDialog::accept()
{
    addHistory(ui->cBAddress, history);

    QDialog::accept();
}
//---------------------------------------------------------------------------
void FtpOptDialog::showKeyDialog()
{
    keyDlg->exec();
}
//---------------------------------------------------------------------------
void FtpOptDialog::addHistory(QComboBox *list, QString *hist)
{
    for (int i = 0; i < MAXHIST; i++) {
        if (list->currentText() != hist[i]) continue;
        for (int j = i + 1; j < MAXHIST; j++) hist[j - 1] = hist[j];
        hist[MAXHIST - 1] = "";
	}
    for (int i = MAXHIST - 1; i > 0; i--) hist[i] = hist[i - 1];
    hist[0] = list->currentText();

    list->clear();
    for (int i = 0; i < MAXHIST; i++)
        if (hist[i] != "") list->addItem(hist[i]);
    list->setCurrentIndex(0);
}
//---------------------------------------------------------------------------
void FtpOptDialog::setHistory(QString tcpHistory[], int size)
{
    ui->cBAddress->clear();

    for (int i = 0; i < qMin(size, MAXHIST); i++) {
        this->history[i] = tcpHistory[i];
        if (!history[i].isEmpty())
            ui->cBAddress->addItem(history[i]);
    }
}
//---------------------------------------------------------------------------
QString* FtpOptDialog::getHistory()
{
    return history;
}
//---------------------------------------------------------------------------
void FtpOptDialog::updateEnable()
{
    ui->lEUser->setEnabled(options == 0);
    ui->lEPassword->setEnabled(options == 0);
    ui->lbUser->setEnabled(options == 0);
    ui->lbPassword->setEnabled(options == 0);
}
