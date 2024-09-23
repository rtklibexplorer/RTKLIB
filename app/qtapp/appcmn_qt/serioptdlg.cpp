//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann
#include "serioptdlg.h"

#include <QShowEvent>
#include <QComboBox>
#include <QSerialPortInfo>

#include "ui_serioptdlg.h"

//---------------------------------------------------------------------------
SerialOptDialog::SerialOptDialog(QWidget *parent, int options)
    : QDialog(parent), ui(new Ui::SerialOptDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &SerialOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &SerialOptDialog::reject);
    connect(ui->cBOutputTcpPort, &QCheckBox::clicked, this, &SerialOptDialog::updateEnable);

    updateEnable();
    setOptions(options);
}
//---------------------------------------------------------------------------
void SerialOptDialog::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;
    updatePortList();
}
//---------------------------------------------------------------------------
void SerialOptDialog::setOptions(int options)
{
    ui->cBOutputTcpPort->setVisible(options);
    ui->cBTcpPort->setVisible(options);
}
//---------------------------------------------------------------------------
void SerialOptDialog::setPath(const QString &path)
{
    QStringList tokens = path.split(':');

    ui->cBPort->setCurrentIndex(ui->cBPort->findText(tokens.first()));

    if (tokens.size() < 2) return;
    ui->cBBitRate->setCurrentIndex(ui->cBBitRate->findText(tokens.at(1) + " bps"));

    if (tokens.size() < 3) return;
    ui->cBByteSize->setCurrentIndex(tokens.at(2) == "7" ? 0 : 1);

    if (tokens.size() < 4) return;
    ui->cBParity->setCurrentIndex(tokens.at(3) == "n" ? 0 : tokens.at(3) == "e" ? 1 : 2);

    if (tokens.size() < 5) return;
    ui->cBStopBits->setCurrentIndex(tokens.at(4) == "1" ? 0 : 1);

    if (tokens.size() < 6) return;
    ui->cBFlowControl->setCurrentIndex(tokens.at(5).contains("off") ? 0 : tokens.at(5).contains("rts") ? 1 : 2);

    QStringList tokens2 = tokens.at(5).split('#');

    bool okay;
    if (tokens2.size() == 2) {
        int port = tokens2.at(1).toInt(&okay);
        if (okay) {
            ui->cBOutputTcpPort->setChecked(true);
            ui->cBTcpPort->setValue(port);
        }
    } else
    {
        ui->cBOutputTcpPort->setChecked(false);
        ui->cBTcpPort->setValue(-1);
    }
}
//---------------------------------------------------------------------------
QString SerialOptDialog::getPath()
{
    const char *parity[] = {"n", "e", "o"}, *fctr[] = {"off", "rts", "xon"};
    QString Port_Text = ui->cBPort->currentText();
    QString BitRate_Text = ui->cBBitRate->currentText().split(" ").first();
    QString path;

    path = QStringLiteral("%1:%2:%3:%4:%5:%6")
               .arg(Port_Text)
               .arg(BitRate_Text)
               .arg(ui->cBByteSize->currentIndex() ? 8 : 7)
               .arg(parity[ui->cBParity->currentIndex()])
               .arg(ui->cBStopBits->currentIndex() ? 2 : 1)
               .arg(fctr[ui->cBFlowControl->currentIndex()]);

    if (ui->cBOutputTcpPort->isChecked())
        path += QStringLiteral("#%1").arg(ui->cBTcpPort->value());

    return path;
}
//---------------------------------------------------------------------------
void SerialOptDialog::updatePortList()
{
    ui->cBPort->clear();
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

    for (int i = 0; i < ports.size(); i++)
        ui->cBPort->addItem(ports.at(i).portName());
}
//---------------------------------------------------------------------------
void SerialOptDialog::updateEnable()
{
    ui->cBTcpPort->setEnabled(ui->cBOutputTcpPort->isChecked());
}
//---------------------------------------------------------------------------
