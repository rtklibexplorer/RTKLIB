//---------------------------------------------------------------------------
#include <QShortcutEvent>
#include <QCompleter>
#include <QFileSystemModel>
#include <QFileDialog>
#include <QAction>

#include "serioptdlg.h"
#include "fileoptdlg.h"
#include "tcpoptdlg.h"
#include "outstrdlg.h"
#include "keydlg.h"

#include "ui_outstrdlg.h"

#include "rtklib.h"


//---------------------------------------------------------------------------
OutputStrDialog::OutputStrDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::OutputStrDialog)
{
    ui->setupUi(this);

    ui->cBFormat1->setItemData(0, "Latitude, longitude and height", Qt::ToolTipRole);
    ui->cBFormat1->setItemData(1, "X/Y/Z components of ECEF coordinates", Qt::ToolTipRole);
    ui->cBFormat1->setItemData(2, "E/N/U components of baseline vector", Qt::ToolTipRole);
    ui->cBFormat1->setItemData(3, "NMEA GPRMC, GPGGA, GPGSA, GLGSA, GAGSA, GPGSV, GLGSV and GAGSV", Qt::ToolTipRole);
    ui->cBFormat1->setItemData(4, "Solution status", Qt::ToolTipRole);

    ui->cBFormat2->setItemData(0, "Latitude, longitude and height", Qt::ToolTipRole);
    ui->cBFormat2->setItemData(1, "X/Y/Z components of ECEF coordinates", Qt::ToolTipRole);
    ui->cBFormat2->setItemData(2, "E/N/U components of baseline vector", Qt::ToolTipRole);
    ui->cBFormat2->setItemData(3, "NMEA GPRMC, GPGGA, GPGSA, GLGSA, GAGSA, GPGSV, GLGSV and GAGSV", Qt::ToolTipRole);
    ui->cBFormat2->setItemData(4, "Solution status", Qt::ToolTipRole);

    ui->cBFormat3->setItemData(0, "Latitude, longitude and height", Qt::ToolTipRole);
    ui->cBFormat3->setItemData(1, "X/Y/Z components of ECEF coordinates", Qt::ToolTipRole);
    ui->cBFormat3->setItemData(2, "E/N/U components of baseline vector", Qt::ToolTipRole);
    ui->cBFormat3->setItemData(3, "NMEA GPRMC, GPGGA, GPGSA, GLGSA, GAGSA, GPGSV, GLGSV and GAGSV", Qt::ToolTipRole);
    ui->cBFormat3->setItemData(4, "Solution status", Qt::ToolTipRole);

    keyDialog = new KeyDialog(this);
    serialOptDialog = new SerialOptDialog(this);
    tcpOptDialog = new TcpOptDialog(this);

    QFileSystemModel *fileModel = new QFileSystemModel(this);
    fileModel->setRootPath("");
    ui->lEFilePath1->setCompleter(new QCompleter(fileModel, this));
    ui->lEFilePath2->setCompleter(new QCompleter(fileModel, this));
    ui->lEFilePath3->setCompleter(new QCompleter(fileModel, this));

    // line edit actions
    QAction *aclEFilePath1Select = ui->lEFilePath1->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath1Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath2Select = ui->lEFilePath2->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath2Select->setToolTip(tr("Select File"));
    QAction *aclEFilePath3Select = ui->lEFilePath3->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    aclEFilePath3Select->setToolTip(tr("Select File"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &OutputStrDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &OutputStrDialog::reject);
    connect(aclEFilePath1Select, &QAction::triggered, this, &OutputStrDialog::selectFile1);
    connect(aclEFilePath2Select, &QAction::triggered, this, &OutputStrDialog::selectFile2);
    connect(aclEFilePath3Select, &QAction::triggered, this, &OutputStrDialog::selectFile3);
    connect(ui->btnKey, &QPushButton::clicked, this, &OutputStrDialog::showKeyDialog);
    connect(ui->btnStream1, &QPushButton::clicked, this, &OutputStrDialog::showStream1Options);
    connect(ui->btnStream2, &QPushButton::clicked, this, &OutputStrDialog::showStream2Options);
    connect(ui->btnStream3, &QPushButton::clicked, this, &OutputStrDialog::showStream2Options);
    connect(ui->cBStream1, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OutputStrDialog::updateEnable);
    connect(ui->cBStream2, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OutputStrDialog::updateEnable);
    connect(ui->cBStream3, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OutputStrDialog::updateEnable);
    connect(ui->cBStream1C, &QCheckBox::clicked, this, &OutputStrDialog::updateEnable);
    connect(ui->cBStream2C, &QCheckBox::clicked, this, &OutputStrDialog::updateEnable);
    connect(ui->cBStream3C, &QCheckBox::clicked, this, &OutputStrDialog::updateEnable);

    ui->cBSwapInterval->setValidator(new QDoubleValidator(this));
}
//---------------------------------------------------------------------------
void OutputStrDialog::selectFile1()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Load..."), ui->lEFilePath1->text());

    if (!filename.isEmpty())
        ui->lEFilePath1->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OutputStrDialog::selectFile2()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Load..."), ui->lEFilePath2->text());
    if (!filename.isEmpty())
        ui->lEFilePath2->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OutputStrDialog::selectFile3()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Load..."), ui->lEFilePath3->text());
    if (!filename.isEmpty())
        ui->lEFilePath3->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OutputStrDialog::showKeyDialog()
{
    keyDialog->show();
}
//---------------------------------------------------------------------------
void OutputStrDialog::showStream1Options()
{
    switch (ui->cBStream1->currentIndex()) {
        case 0: showSerialOptions(0, 0); break;
        case 1: showTcpOptions(0, TcpOptDialog::OPT_TCP_CLIENT); break;
        case 2: showTcpOptions(0, TcpOptDialog::OPT_TCP_SERVER); break;
        case 3: showTcpOptions(0, TcpOptDialog::OPT_NTRIP_SERVER); break;
        case 4: showTcpOptions(0, TcpOptDialog::OPT_NTRIP_CASTER_CLIENT); break;
    }
}
//---------------------------------------------------------------------------
void OutputStrDialog::showStream2Options()
{
    switch (ui->cBStream2->currentIndex()) {
        case 0: showSerialOptions(1, 0); break;
        case 1: showTcpOptions(1, TcpOptDialog::OPT_TCP_CLIENT); break;
        case 2: showTcpOptions(1, TcpOptDialog::OPT_TCP_SERVER); break;
        case 3: showTcpOptions(1, TcpOptDialog::OPT_NTRIP_SERVER); break;
        case 4: showTcpOptions(0, TcpOptDialog::OPT_NTRIP_CASTER_CLIENT); break;
    }
}
//---------------------------------------------------------------------------
void OutputStrDialog::showStream3Options()
{
    switch (ui->cBStream3->currentIndex()) {
        case 0: showSerialOptions(1, 0); break;
        case 1: showTcpOptions(1, TcpOptDialog::OPT_TCP_CLIENT); break;
        case 2: showTcpOptions(1, TcpOptDialog::OPT_TCP_SERVER); break;
        case 3: showTcpOptions(1, TcpOptDialog::OPT_NTRIP_SERVER); break;
        case 4: showTcpOptions(0, TcpOptDialog::OPT_NTRIP_CASTER_CLIENT); break;
    }
}
//---------------------------------------------------------------------------
QString OutputStrDialog::getFilePath(const QString &path)
{
    return path.mid(0, path.indexOf("::"));
}
//---------------------------------------------------------------------------
QString OutputStrDialog::setFilePath(const QString &p)
{
    QString path = p;
    QString str;
    bool okay;

    if (ui->cBTimeTag->isChecked()) path += "::T";
    str = ui->cBSwapInterval->currentText().split(' ').first();
    str.toDouble(&okay);
    if (okay) path += "::S=" + str;
    return path;
}
//---------------------------------------------------------------------------
void OutputStrDialog::showSerialOptions(int index, int opt)
{
    if ((index < 0) || (index > 1)) return;

    serialOptDialog->setOptions(opt);
    serialOptDialog->setPath(paths[index][0]);

    serialOptDialog->exec();
    if (serialOptDialog->result() != QDialog::Accepted) return;

    paths[index][0] = serialOptDialog->getPath();
}
//---------------------------------------------------------------------------
void OutputStrDialog::showTcpOptions(int index, int opt)
{
    if ((index < 0) || (index > 1)) return;

    tcpOptDialog->setOptions(opt);
    tcpOptDialog->setHistory(history, 10);
    tcpOptDialog->setPath(paths[index][1]);

    tcpOptDialog->exec();
    if (tcpOptDialog->result() != QDialog::Accepted) return;

    paths[index][1] = tcpOptDialog->getPath();
    for (int i = 0; i < 10; i++) {
        history[i] = tcpOptDialog->getHistory()[i];
	}
}
//---------------------------------------------------------------------------
void OutputStrDialog::updateEnable()
{
    int ena = (ui->cBStream1C->isChecked() && (ui->cBStream1->currentIndex() == 5)) ||
              (ui->cBStream2C->isChecked() && (ui->cBStream2->currentIndex() == 5)) ||
              (ui->cBStream3C->isChecked() && (ui->cBStream3->currentIndex() == 5));

    ui->cBStream1->setEnabled(ui->cBStream1C->isChecked());
    ui->cBStream2->setEnabled(ui->cBStream2C->isChecked());
    ui->cBStream3->setEnabled(ui->cBStream3C->isChecked());
    ui->btnStream1->setEnabled(ui->cBStream1C->isChecked() && ui->cBStream1->currentIndex() <= 4);
    ui->btnStream2->setEnabled(ui->cBStream2C->isChecked() && ui->cBStream2->currentIndex() <= 4);
    ui->btnStream3->setEnabled(ui->cBStream3C->isChecked() && ui->cBStream3->currentIndex() <= 4);
    ui->lEFilePath1->setEnabled(ui->cBStream1C->isChecked() && ui->cBStream1->currentIndex() == 5);
    ui->lEFilePath2->setEnabled(ui->cBStream2C->isChecked() && ui->cBStream2->currentIndex() == 5);
    ui->lEFilePath3->setEnabled(ui->cBStream3C->isChecked() && ui->cBStream3->currentIndex() == 5);
    ui->cBFormat1->setEnabled(ui->cBStream1C->isChecked());
    ui->cBFormat2->setEnabled(ui->cBStream2C->isChecked());
    ui->cBFormat3->setEnabled(ui->cBStream3C->isChecked());
    ui->lblF1->setEnabled(ena);
    ui->lblSwapInterval->setEnabled(ena);
    ui->cBTimeTag->setEnabled(ena);
    ui->cBSwapInterval->setEnabled(ena);
    ui->btnKey->setEnabled(ena);
}
//---------------------------------------------------------------------------
void OutputStrDialog::setStreamEnabled(int stream, int enabled)
{
    QCheckBox *cBStreamC[] = {ui->cBStream1C, ui->cBStream2C, ui->cBStream3C};
    if ((stream < 0 ) || (stream > 2)) return;

    cBStreamC[stream]->setChecked(enabled);

    updateEnable();
}
//---------------------------------------------------------------------------
int OutputStrDialog::getStreamEnabled(int stream)
{
    QCheckBox *cBStreamC[] = {ui->cBStream1C, ui->cBStream2C, ui->cBStream3C};
    if ((stream < 0 ) || (stream > 2)) return false;

    return cBStreamC[stream]->isChecked();
}
//---------------------------------------------------------------------------
void OutputStrDialog::setStreamType(int stream, int type)
{
    QComboBox *cBStream[] = {ui->cBStream1, ui->cBStream2, ui->cBStream3};
    if ((stream < 0 ) || (stream > 2)) return;

    cBStream[stream]->setCurrentIndex(type);

    updateEnable();
}
//---------------------------------------------------------------------------
void OutputStrDialog::setStreamFormat(int stream, int format)
{
    QComboBox *cBFormat[] = {ui->cBFormat1, ui->cBFormat2, ui->cBFormat3};
    if ((stream < 0 ) || (stream > 2)) return;

    cBFormat[stream]->setCurrentIndex(format);
    updateEnable();
}
//---------------------------------------------------------------------------
int OutputStrDialog::getStreamFormat(int stream)
{
    QComboBox *cBFormat[] = {ui->cBFormat1, ui->cBFormat2, ui->cBFormat3};
    if ((stream < 0 ) || (stream > 2)) return STRFMT_RTCM2;

    return cBFormat[stream]->currentIndex();
}

//---------------------------------------------------------------------------
int OutputStrDialog::getStreamType(int stream)
{
    QComboBox *cBStream[] = {ui->cBStream1, ui->cBStream2, ui->cBStream3};
    if ((stream < 0 ) || (stream > 2)) return STR_NONE;

    return cBStream[stream]->currentIndex();
};
//---------------------------------------------------------------------------
void OutputStrDialog::setPath(int stream, int type, const QString &path)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3};
    if ((stream < 0 ) || (stream > 2)) return;
    if ((type < 0) || (type > 3)) return;

    paths[stream][type] = path;
    ui->cBTimeTag->setChecked(path.contains("::T"));
    if (path.contains("::S="))
    {
        int startPos = path.indexOf("::S=")+4;
        QString startTime = path.mid(startPos, path.indexOf("::", startPos)-startPos);
        ui->cBSwapInterval->setCurrentText(startTime + " h");
    };

    if (type == 2)
    {
        edits[stream]->setText(path.mid(0, path.indexOf("::")));
    };
}
//---------------------------------------------------------------------------
QString OutputStrDialog::getPath(int stream, int type)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3};
    if ((stream < 0 ) || (stream > 2)) return "";
    if ((type < 0) || (type > 3)) return "";

    if (type == 2)
        return setFilePath(edits[stream]->text());

    return paths[stream][type];
}
//---------------------------------------------------------------------------
void OutputStrDialog::setTimeTagEnabled(bool ena)
{
    ui->cBTimeTag->setChecked(ena);
}
//---------------------------------------------------------------------------
bool OutputStrDialog::getTimeTagEnabled(){
    return ui->cBTimeTag->isChecked();
}
//---------------------------------------------------------------------------
void OutputStrDialog::setSwapInterval(const QString & swapInterval)
{
    QString interval_str = swapInterval;
    if (!interval_str.isEmpty()) interval_str += " h";
    if (ui->cBSwapInterval->findText(interval_str) == -1)
        ui->cBSwapInterval->insertItem(0, interval_str);
    ui->cBSwapInterval->setCurrentText(interval_str);
}
//---------------------------------------------------------------------------
QString OutputStrDialog::getSwapInterval()
{
    QStringList tokens = ui->cBSwapInterval->currentText().split(' ', Qt::SkipEmptyParts);
    if (tokens.size() > 1)
        return tokens.first();
    else return "";
};
//---------------------------------------------------------------------------
void OutputStrDialog::setHistory(int i, const QString &history)
{
    if ((i < 0) || (i > 9)) return;

    this->history[i] = history;
}
//---------------------------------------------------------------------------
const QString OutputStrDialog::getHistory(int i)
{
    if ((i < 0) || (i > 9)) return "";

    return history[i];
}
//---------------------------------------------------------------------------
