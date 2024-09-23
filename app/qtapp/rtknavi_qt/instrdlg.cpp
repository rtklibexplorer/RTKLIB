//---------------------------------------------------------------------------

#include <QComboBox>
#include <QFileInfo>
#include <QShowEvent>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QCompleter>

#include "refdlg.h"
#include "navimain.h"
#include "serioptdlg.h"
#include "cmdoptdlg.h"
#include "tcpoptdlg.h"
#include "ftpoptdlg.h"
#include "rcvoptdlg.h"
#include "instrdlg.h"

#include "ui_instrdlg.h"

#include "rtklib.h"

//---------------------------------------------------------------------------
InputStrDialog::InputStrDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::InputStrDialog)
{
    ui->setupUi(this);

    noFormats = 0;

    // add fill format combo box with all available formats
    for (int i = 0; i <= MAXRCVFMT; i++) {
        ui->cBFormat1->addItem(formatstrs[i]);
        ui->cBFormat2->addItem(formatstrs[i]);
        ui->cBFormat3->addItem(formatstrs[i]);
        noFormats++;
	}
    ui->cBFormat3->addItem(tr("SP3"));

    cmdOptDialog = new CmdOptDialog(this);
    rcvOptDialog = new RcvOptDialog(this);
    refDialog = new RefDialog(this);
    serialOptDialog = new SerialOptDialog(this);;
    tcpOptDialog = new TcpOptDialog(this);
    ftpOptDialog = new FtpOptDialog(this);

    // setup completers
    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lEFilePath1->setCompleter(fileCompleter);
    ui->lEFilePath2->setCompleter(fileCompleter);
    ui->lEFilePath3->setCompleter(fileCompleter);

    // line edit actions
    QAction *aclEFilePath1Select = ui->lEFilePath1->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath1Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath2Select = ui->lEFilePath2->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath2Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath3Select = ui->lEFilePath3->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath3Select->setToolTip(tr("Select File"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &InputStrDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &InputStrDialog::reject);
    connect(ui->btnPosition, &QPushButton::clicked, this, &InputStrDialog::selectPosition);
    connect(ui->btnCmd1, &QPushButton::clicked, this, &InputStrDialog::showCommandDialog1);
    connect(ui->btnCmd2, &QPushButton::clicked, this, &InputStrDialog::showCommandDialog2);
    connect(ui->btnCmd3, &QPushButton::clicked, this, &InputStrDialog::showCommandDialog3);
    connect(aclEFilePath1Select, &QAction::triggered, this, &InputStrDialog::selectFile1);
    connect(aclEFilePath2Select, &QAction::triggered, this, &InputStrDialog::selectFile2);
    connect(aclEFilePath3Select, &QAction::triggered, this, &InputStrDialog::selectFile3);
    connect(ui->btnReceiverOptions1, &QPushButton::clicked, this, &InputStrDialog::showReceiverOptions1);
    connect(ui->btnReceiverOptions2, &QPushButton::clicked, this, &InputStrDialog::showReceiverOptions2);
    connect(ui->btnReceiverOptions3, &QPushButton::clicked, this, &InputStrDialog::showReceiverOptions3);
    connect(ui->btnStream1, &QPushButton::clicked, this, &InputStrDialog::showStreamOptions1);
    connect(ui->btnStream2, &QPushButton::clicked, this, &InputStrDialog::showStreamOptions2);
    connect(ui->btnStream3, &QPushButton::clicked, this, &InputStrDialog::showStreamOptions3);
    connect(ui->cBStreamC1, &QCheckBox::clicked, this, &InputStrDialog::updateEnable);
    connect(ui->cBStreamC2, &QCheckBox::clicked, this, &InputStrDialog::updateEnable);
    connect(ui->cBStreamC3, &QCheckBox::clicked, this, &InputStrDialog::updateEnable);
    connect(ui->cBStream1, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &InputStrDialog::updateEnable);
    connect(ui->cBStream2, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &InputStrDialog::updateEnable);
    connect(ui->cBStream3, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &InputStrDialog::updateEnable);
    connect(ui->cBNmeaRequestType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &InputStrDialog::updateEnable);
    connect(ui->cBTimeTag, &QCheckBox::clicked, this, &InputStrDialog::updateEnable);
}
//---------------------------------------------------------------------------
void InputStrDialog::setStreamEnabled(int stream, int enabled)
{
    QCheckBox *cBStreamC[] = {ui->cBStreamC1, ui->cBStreamC2, ui->cBStreamC3};
    if ((stream < 0 ) || (stream > 2)) return;
    cBStreamC[stream]->setChecked(enabled);
    updateEnable();
}
//---------------------------------------------------------------------------
int InputStrDialog::getStreamEnabled(int stream)
{
    QCheckBox *cBStreamC[] = {ui->cBStreamC1, ui->cBStreamC2, ui->cBStreamC3};
    if ((stream < 0 ) || (stream > 2)) return false;
    return cBStreamC[stream]->isChecked();
}
//---------------------------------------------------------------------------
void InputStrDialog::setStreamType(int stream, int type)
{
    QComboBox *cBStream[] = {ui->cBStream1, ui->cBStream2, ui->cBStream3};
    if ((stream < 0 ) || (stream > 2)) return;
    cBStream[stream]->setCurrentIndex(type);
    updateEnable();
}
//---------------------------------------------------------------------------
int InputStrDialog::getStreamType(int stream)
{
    QComboBox *cBStream[] = {ui->cBStream1, ui->cBStream2, ui->cBStream3};
    if ((stream < 0 ) || (stream > 2)) return STR_NONE;
    return cBStream[stream]->currentIndex();
}
//---------------------------------------------------------------------------
void InputStrDialog::setStreamFormat(int stream, int format)
{
    QComboBox *cBFormat[] = {ui->cBFormat1, ui->cBFormat2, ui->cBFormat3};
    if ((stream < 0 ) || (stream > 2)) return;
    if (stream == 0)
        cBFormat[stream]->setCurrentIndex(format);
    else
        cBFormat[stream]->setCurrentIndex(format < noFormats ? format : noFormats + format - STRFMT_SP3);

    updateEnable();
}
//---------------------------------------------------------------------------
int InputStrDialog::getStreamFormat(int stream)
{
    QComboBox *cBFormat[] = {ui->cBFormat1, ui->cBFormat2, ui->cBFormat3};
    if ((stream < 0 ) || (stream > 2)) return STRFMT_RTCM2;  // should never happen
    if (stream == 0)
        return cBFormat[stream]->currentIndex();
    else
        return cBFormat[stream]->currentIndex() < noFormats ? cBFormat[stream]->currentIndex() : STRFMT_SP3 + cBFormat[stream]->currentIndex() - noFormats;
}
//---------------------------------------------------------------------------
void InputStrDialog::setReceiverOptions(int stream, const QString & options)
{
    if ((stream < 0 ) || (stream > 2)) return;
    receiverOptions[stream] = options;
}
//---------------------------------------------------------------------------
QString InputStrDialog::getReceiverOptions(int stream)
{
    if ((stream < 0 ) || (stream > 2)) return "";
    return receiverOptions[stream];
}
//---------------------------------------------------------------------------
void InputStrDialog::setResetCommand(const QString & command)
{
    ui->lEResetCmd->setText(command);
}
//---------------------------------------------------------------------------
QString InputStrDialog::getResetCommand()
{
    return ui->lEResetCmd->text();
}
//---------------------------------------------------------------------------
void InputStrDialog::setNMeaPosition(int i, double value)
{
    QDoubleSpinBox *widget[] = {ui->sBNmeaPosition1, ui->sBNmeaPosition2, ui->sBNmeaPosition3};
    if ((i < 0 ) || (i > 2)) return;
    widget[i]->setValue(value);
}
//---------------------------------------------------------------------------
double InputStrDialog::getNMeaPosition(int i)
{
    QDoubleSpinBox *widget[] = {ui->sBNmeaPosition1, ui->sBNmeaPosition2, ui->sBNmeaPosition3};
    if ((i < 0 ) || (i > 2)) return NAN;
    return widget[i]->value();
}
//---------------------------------------------------------------------------
void InputStrDialog::setNmeaRequestType(int nmeaMode)
{
    ui->cBNmeaRequestType->setCurrentIndex(nmeaMode);
    updateEnable();
}
//---------------------------------------------------------------------------
int InputStrDialog::getNmeaRequestType()
{
    return ui->cBNmeaRequestType->currentIndex();
}
//---------------------------------------------------------------------------
void InputStrDialog::setMaxBaseLine(double baseLine)
{
    ui->sBMaxBaseLine->setValue(baseLine);
}
//---------------------------------------------------------------------------
double InputStrDialog::getMaxBaseLine()
{
    return ui->sBMaxBaseLine->value();
}
//---------------------------------------------------------------------------
void InputStrDialog::setHistory(int i, const QString &history)
{
    if ((i < 0) || (i > 9)) return;
    this->history[i] = history;
}
//---------------------------------------------------------------------------
const QString &InputStrDialog::getHistory(int i)
{
    if ((i < 0) || (i > 9)) QString();
    return history[i];
}
//---------------------------------------------------------------------------
void InputStrDialog::setTimeTagEnabled(bool tag)
{
    ui->cBTimeTag->setChecked(tag);

    updateEnable();
}
//---------------------------------------------------------------------------
bool InputStrDialog::getTimeTagEnabled()
{
    return ui->cBTimeTag->isChecked();
}
//---------------------------------------------------------------------------
void InputStrDialog::setTimeTag64bit(bool enabled)
{
    ui->cB64Bit->setChecked(enabled);
}
//---------------------------------------------------------------------------
bool InputStrDialog::getTimeTag64bit()
{
    return ui->cB64Bit->isChecked();
}
//---------------------------------------------------------------------------
void InputStrDialog::setTimeStart(double time)
{
    ui->sBTimeStart->setValue(time);
}
//---------------------------------------------------------------------------
double InputStrDialog::getTimeStart()
{
    return ui->sBTimeStart->value();
}
//---------------------------------------------------------------------------
void InputStrDialog::setTimeSpeed(const QString & timeSpeed)
{
    ui->cBTimeSpeed->setCurrentIndex(ui->cBTimeSpeed->findText(timeSpeed));
}
//---------------------------------------------------------------------------
QString InputStrDialog::getTimeSpeed()
{
    return ui->cBTimeSpeed->currentText();
}
//---------------------------------------------------------------------------
void InputStrDialog::setStationPositionFile(const QString & file)
{
    stationPositionFile = file;
}
//---------------------------------------------------------------------------
QString InputStrDialog::getStationPositionFile()
{
    return stationPositionFile;
}
//---------------------------------------------------------------------------
void InputStrDialog::setCommands(int stream, int type, const QString &cmd)
{
    if ((stream < 0) || (stream > 2)) return;
    if ((type < 0) || (type > 2)) return;

    commands[stream][type]= cmd;
}
//---------------------------------------------------------------------------
QString InputStrDialog::getCommands(int stream, int type)
{
    if ((stream < 0) || (stream > 2)) return QString();
    if ((type < 0) || (type > 2)) return QString();

    return commands[stream][type];
}
//---------------------------------------------------------------------------
void InputStrDialog::setCommandsEnabled(int stream, int type, bool ena)
{
    if ((stream < 0) || (stream > 2)) return;
    if ((type < 0) || (type > 2)) return;

    commandEnable[stream][type] = ena;
};
//---------------------------------------------------------------------------
bool InputStrDialog::getCommandsEnabled(int stream, int type)
{
    if ((stream < 0) || (stream > 2)) return false;
    if ((type < 0) || (type > 2)) return false;

    return commandEnable[stream][type];
};
//---------------------------------------------------------------------------
void InputStrDialog::setCommandsTcp(int stream, int type, const QString &cmd)
{
    if ((stream < 0) || (stream > 2)) return;
    if ((type < 0) || (type > 2)) return;

    commandsTcp[stream][type] = cmd;
}
//---------------------------------------------------------------------------
QString InputStrDialog::getCommandsTcp(int stream, int type)
{
    if ((stream < 0) || (stream > 2)) return QString();
    if ((type < 0) || (type > 2)) return QString();

    return commandsTcp[stream][type];
}
//---------------------------------------------------------------------------
void InputStrDialog::setCommandsTcpEnabled(int stream, int type, bool ena)
{
    if ((stream < 0) || (stream > 2)) return;
    if ((type < 0) || (type > 2)) return;

    commandEnableTcp[stream][type] = ena;
};
//---------------------------------------------------------------------------
bool InputStrDialog::getCommandsTcpEnabled(int stream, int type)
{
    if ((stream < 0) || (stream > 2)) return false;
    if ((type < 0) || (type > 2)) return false;

    return commandEnableTcp[stream][type];
};
//---------------------------------------------------------------------------
void InputStrDialog::setPath(int stream, int type, const QString & path)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3};

    if ((stream < 0) || (stream > 2)) return;
    if ((type < 0) || (type > 3)) return;

    paths[stream][type] = path;
    ui->cBTimeTag->setChecked(path.contains("::T"));
    ui->cB64Bit->setChecked(path.contains("::P=8"));
    if (path.contains("::+"))
    {
        int startPos = path.indexOf("::+")+3;
        QString startTime = path.mid(startPos, path.indexOf("::", startPos)-startPos);
        ui->sBTimeStart->setValue(startTime.toInt());
    };
    if (path.contains("::x"))
    {
        int startPos = path.indexOf("::x")+3;
        QString speed = path.mid(startPos, path.indexOf("::", startPos)-startPos);
        ui->cBTimeSpeed->setCurrentText(speed);
    }
    if (type == 2)
    {
        edits[stream]->setText(path.mid(0, path.indexOf("::")));
    };
}
//---------------------------------------------------------------------------
QString InputStrDialog::getPath(int stream, int type)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3};
    if ((stream < 0) || (stream > 2)) return QString();
    if ((type < 0) || (type > 3)) return QString();

    if (type == 2)
        return makePath(edits[stream]->text());

    return paths[stream][type];
}
//---------------------------------------------------------------------------
QString InputStrDialog::extractFilePath(const QString &path)
{
    return path.mid(0, path.indexOf("::"));
}
//---------------------------------------------------------------------------
QString InputStrDialog::makePath(const QString &p)
{
    QString path = p;

    if (ui->cBTimeTag->isChecked()) path += "::T";
    if (ui->sBTimeStart->value() != 0) path += "::+" + QString::number(ui->sBTimeStart->value());
    path += "::" + ui->cBTimeSpeed->currentText();
    if (ui->cB64Bit->isChecked()) path += "::P=8";
    return path;
}
//---------------------------------------------------------------------------
void InputStrDialog::showStreamOptions1()
{
    switch (ui->cBStream1->currentIndex()) {
        case 0: showSerialOptionsDialog(0, 0); break;
        case 1: showTcpOptionsDialog(0, TcpOptDialog::OPT_TCP_CLIENT); break;
        case 2: showTcpOptionsDialog(0, TcpOptDialog::OPT_TCP_SERVER); break;
        case 3: showTcpOptionsDialog(0, TcpOptDialog::OPT_NTRIP_CLIENT); break;
	}
}
//---------------------------------------------------------------------------
void InputStrDialog::showStreamOptions2()
{
    switch (ui->cBStream2->currentIndex()) {
        case 0: showSerialOptionsDialog(1, 0); break;
        case 1: showTcpOptionsDialog(1, TcpOptDialog::OPT_TCP_CLIENT); break;
        case 2: showTcpOptionsDialog(1, TcpOptDialog::OPT_TCP_SERVER); break;
        case 3: showTcpOptionsDialog(1, TcpOptDialog::OPT_NTRIP_CLIENT); break;
        case 5: showFtpOptionsDialog(1, 0); break;
        case 6: showFtpOptionsDialog(1, 1); break;
	}
}
//---------------------------------------------------------------------------
void InputStrDialog::showStreamOptions3()
{
    switch (ui->cBStream3->currentIndex()) {
        case 0: showSerialOptionsDialog(2, 0); break;
        case 1: showTcpOptionsDialog(2, TcpOptDialog::OPT_TCP_CLIENT); break;
        case 2: showTcpOptionsDialog(2, TcpOptDialog::OPT_TCP_SERVER); break;
        case 3: showTcpOptionsDialog(2, TcpOptDialog::OPT_NTRIP_CLIENT); break;
        case 5: showFtpOptionsDialog(2, 0); break;
        case 6: showFtpOptionsDialog(2, 1); break;
	}
}
//---------------------------------------------------------------------------
void InputStrDialog::showCommandDialog(int streamNo)
{
    if ((streamNo < 0) || (streamNo > 2)) return;

    for (int i = 0; i < 3; i++) {
        if (ui->cBStream1->currentIndex() == 0) {
            cmdOptDialog->setCommands(i, commands[streamNo][i]);
            cmdOptDialog->setCommandsEnabled(i, commandEnable[streamNo][i]);
        }
        else {
            cmdOptDialog->setCommands(i, commandsTcp[streamNo][i]);
            cmdOptDialog->setCommandsEnabled(i, commandEnableTcp[streamNo][i]);
        }
    }

    cmdOptDialog->exec();
    if (cmdOptDialog->result() != QDialog::Accepted) return;

    for (int i = 0; i < 3; i++) {
        if (ui->cBStream1->currentIndex() == 0) {
            commands[streamNo][i] = cmdOptDialog->getCommands(i);
            commandEnable[streamNo][i] = cmdOptDialog->getCommandsEnabled(i);
        }
        else {
            commandsTcp[streamNo][i] = cmdOptDialog->getCommands(i);
            commandEnableTcp[streamNo][i] = cmdOptDialog->getCommandsEnabled(i);
        }
    }
}
//---------------------------------------------------------------------------
void InputStrDialog::showCommandDialog1()
{
    this->showCommandDialog(0);
}
//---------------------------------------------------------------------------
void InputStrDialog::showCommandDialog2()
{
    this->showCommandDialog(1);
}
//---------------------------------------------------------------------------
void InputStrDialog::showCommandDialog3()
{
    this->showCommandDialog(2);
}
//---------------------------------------------------------------------------
void InputStrDialog::showReceiverOptionDialog(int streamNo)
{
    if ((streamNo < 0) || (streamNo > 2)) return;

    rcvOptDialog->setOptions(receiverOptions[streamNo]);

    rcvOptDialog->exec();
    if (rcvOptDialog->result() != QDialog::Accepted) return;

    receiverOptions[streamNo] = rcvOptDialog->getOptions();
}
//---------------------------------------------------------------------------
void InputStrDialog::showReceiverOptions1()
{
    this->showReceiverOptionDialog(0);
}
//---------------------------------------------------------------------------
void InputStrDialog::showReceiverOptions2()
{
    this->showReceiverOptionDialog(1);
}
//---------------------------------------------------------------------------
void InputStrDialog::showReceiverOptions3()
{
    this->showReceiverOptionDialog(2);
}
//---------------------------------------------------------------------------
void InputStrDialog::selectPosition()
{
    refDialog->setRoverPosition(ui->sBNmeaPosition1->value(), ui->sBNmeaPosition2->value(), ui->sBNmeaPosition3->value());
    refDialog->stationPositionFile = stationPositionFile;

    refDialog->exec();
    if (refDialog->result() != QDialog::Accepted) return;

    ui->sBNmeaPosition1->setValue(refDialog->getPosition()[0]);
    ui->sBNmeaPosition2->setValue(refDialog->getPosition()[1]);
    ui->sBNmeaPosition3->setValue(refDialog->getPosition()[2]);
}
//---------------------------------------------------------------------------
void InputStrDialog::selectFile1()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath1->text());
    if (dir.isEmpty()) return;
    ui->lEFilePath1->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void InputStrDialog::selectFile2()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath2->text());
    if (dir.isEmpty()) return;
    ui->lEFilePath2->setText(QDir::toNativeSeparators(dir));}
//---------------------------------------------------------------------------
void InputStrDialog::selectFile3()
{
    QString dir = QFileDialog::getOpenFileName(this, tr("Open..."), ui->lEFilePath3->text());
    if (dir.isEmpty()) return;
    ui->lEFilePath3->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void InputStrDialog::showSerialOptionsDialog(int index, int opt)
{
    serialOptDialog->setOptions(opt);
    serialOptDialog->setPath(getPath(index, 0));

    serialOptDialog->exec();
    if (serialOptDialog->result() != QDialog::Accepted) return;

    setPath(index, 0, serialOptDialog->getPath());
}
//---------------------------------------------------------------------------
void InputStrDialog::showTcpOptionsDialog(int index, int opt)
{
    tcpOptDialog->setOptions(opt);
    tcpOptDialog->setHistory(history, 10);
    tcpOptDialog->setPath(getPath(index, 1));

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    setPath(index, 1, tcpOptDialog->getPath());
    for (int i = 0; i < 10; i++) {
        history[i] = tcpOptDialog->getHistory()[i];
	}
}
//---------------------------------------------------------------------------
void InputStrDialog::showFtpOptionsDialog(int index, int opt)
{
    ftpOptDialog->setOptions(opt);
    ftpOptDialog->setPath(getPath(index, 3));

    ftpOptDialog->exec();
    if (ftpOptDialog->result() != QDialog::Accepted) return;

    setPath(index, 3, ftpOptDialog->getPath());
}
//---------------------------------------------------------------------------
void InputStrDialog::updateEnable()
{
    // (cBStream->currentIndex() == 4) -> File stream
    int enaFile = (ui->cBStreamC1->isChecked() && (ui->cBStream1->currentIndex() == 4)) ||
                  (ui->cBStreamC2->isChecked() && (ui->cBStream2->currentIndex() == 4)) ||
                  (ui->cBStreamC3->isChecked() && (ui->cBStream3->currentIndex() == 4));
    int enaNmea = ui->cBStreamC2->isChecked() && (ui->cBStream2->currentIndex() <= 3);

    ui->cBStream1->setEnabled(ui->cBStreamC1->isChecked());
    ui->cBStream2->setEnabled(ui->cBStreamC2->isChecked());
    ui->cBStream3->setEnabled(ui->cBStreamC3->isChecked());
    ui->btnStream1->setEnabled(ui->cBStreamC1->isChecked() && ui->cBStream1->currentIndex() != 4);
    ui->btnStream2->setEnabled(ui->cBStreamC2->isChecked() && ui->cBStream2->currentIndex() != 4);
    ui->btnStream3->setEnabled(ui->cBStreamC3->isChecked() && ui->cBStream3->currentIndex() != 4);
    ui->btnCmd1->setEnabled(ui->cBStreamC1->isChecked() && ui->cBStream1->currentIndex() != 4);
    ui->btnCmd2->setEnabled(ui->cBStreamC2->isChecked() && ui->cBStream2->currentIndex() != 4);
    ui->btnCmd3->setEnabled(ui->cBStreamC3->isChecked() && ui->cBStream3->currentIndex() != 4);
    ui->cBFormat1->setEnabled(ui->cBStreamC1->isChecked());
    ui->cBFormat2->setEnabled(ui->cBStreamC2->isChecked());
    ui->cBFormat3->setEnabled(ui->cBStreamC3->isChecked());
    ui->btnReceiverOptions1->setEnabled(ui->cBStreamC1->isChecked());
    ui->btnReceiverOptions2->setEnabled(ui->cBStreamC2->isChecked());
    ui->btnReceiverOptions3->setEnabled(ui->cBStreamC3->isChecked());

    ui->lblNmea->setEnabled(enaNmea);
    ui->cBNmeaRequestType->setEnabled(enaNmea);
    ui->sBNmeaPosition1->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 1);
    ui->sBNmeaPosition2->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 1);
    ui->sBNmeaPosition3->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 1);
    ui->btnPosition->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 1);

    ui->lblResetCmd->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 3);
    ui->lEResetCmd->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 3);
    ui->lblMaxBaseLine->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 3);
    ui->sBMaxBaseLine->setEnabled(enaNmea && ui->cBNmeaRequestType->currentIndex() == 3);

    ui->lblInputFilePath->setEnabled(enaFile);
    ui->lEFilePath1->setEnabled(ui->cBStreamC1->isChecked() && ui->cBStream1->currentIndex() == 4);
    ui->lEFilePath2->setEnabled(ui->cBStreamC2->isChecked() && ui->cBStream2->currentIndex() == 4);
    ui->lEFilePath3->setEnabled(ui->cBStreamC3->isChecked() && ui->cBStream3->currentIndex() == 4);
    ui->cBTimeTag->setEnabled(enaFile);
    ui->sBTimeStart->setEnabled(enaFile && ui->cBTimeTag->isChecked());
    ui->cBTimeSpeed->setEnabled(enaFile && ui->cBTimeTag->isChecked());
    ui->lblPlus->setEnabled(enaFile && ui->cBTimeTag->isChecked());
    ui->cB64Bit->setEnabled(enaFile && ui->cBTimeTag->isChecked());
}
//---------------------------------------------------------------------------
