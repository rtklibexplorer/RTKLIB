//---------------------------------------------------------------------------
#include <QDialog>
#include <QShowEvent>
#include <QFileDialog>
#include <QCompleter>
#include <QFileSystemModel>
#include <QAction>

#include "serioptdlg.h"
#include "fileoptdlg.h"
#include "tcpoptdlg.h"
#include "logstrdlg.h"
#include "keydlg.h"

#include "ui_logstrdlg.h"

#include "rtklib.h"


//---------------------------------------------------------------------------
LogStrDialog::LogStrDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::LogStrDialog)
{
    ui->setupUi(this);

    keyDialog = new KeyDialog(this);
    serialOptDialog = new SerialOptDialog(this);
    tcpOptDialog = new TcpOptDialog(this);

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

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &LogStrDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &LogStrDialog::reject);
    connect(ui->btnKey, &QPushButton::clicked, this, &LogStrDialog::showKeyDialog);
    connect(aclEFilePath1Select, &QAction::triggered, this, &LogStrDialog::selectFile1);
    connect(aclEFilePath2Select, &QAction::triggered, this, &LogStrDialog::selectFile2);
    connect(aclEFilePath3Select, &QAction::triggered, this, &LogStrDialog::selectFile3);
    connect(ui->btnStream1, &QPushButton::clicked, this, &LogStrDialog::showStreamOptions1);
    connect(ui->btnStream2, &QPushButton::clicked, this, &LogStrDialog::showStreamOptions2);
    connect(ui->btnStream3, &QPushButton::clicked, this, &LogStrDialog::showStreamOptions3);
    connect(ui->cBStream1, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &LogStrDialog::updateEnable);
    connect(ui->cBStream2, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &LogStrDialog::updateEnable);
    connect(ui->cBStream3, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &LogStrDialog::updateEnable);
    connect(ui->cBStream1C, &QCheckBox::clicked, this, &LogStrDialog::updateEnable);
    connect(ui->cBStream2C, &QCheckBox::clicked, this, &LogStrDialog::updateEnable);
    connect(ui->cBStream3C, &QCheckBox::clicked, this, &LogStrDialog::updateEnable);

    ui->cBSwapInterval->setValidator(new QDoubleValidator(this));
}
//---------------------------------------------------------------------------
void LogStrDialog::selectFile1()
{
    ui->lEFilePath1->setText(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Open..."), ui->lEFilePath1->text())));
}
//---------------------------------------------------------------------------
void LogStrDialog::selectFile2()
{
    ui->lEFilePath2->setText(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Open..."), ui->lEFilePath2->text())));
}
//---------------------------------------------------------------------------
void LogStrDialog::selectFile3()
{
    ui->lEFilePath3->setText(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Open..."), ui->lEFilePath3->text())));
}
//---------------------------------------------------------------------------
void LogStrDialog::showKeyDialog()
{
    keyDialog->show();
}
//---------------------------------------------------------------------------
void LogStrDialog::showStreamOptions1()
{
    switch (ui->cBStream1->currentIndex()) {
        case 0: showSerialOptions(0, 0); break;
        case 1: showTcpOptions(0, 1); break;
        case 2: showTcpOptions(0, 0); break;
        case 3: showTcpOptions(0, 2); break;
        case 4: showTcpOptions(0, 4); break;
    }
}
//---------------------------------------------------------------------------
void LogStrDialog::showStreamOptions2()
{
    switch (ui->cBStream2->currentIndex()) {
        case 0: showSerialOptions(1, 0); break;
        case 1: showTcpOptions(1, 1); break;
        case 2: showTcpOptions(1, 0); break;
        case 3: showTcpOptions(1, 2); break;
        case 4: showTcpOptions(0, 4); break;
	}
}
//---------------------------------------------------------------------------
void LogStrDialog::showStreamOptions3()
{
    switch (ui->cBStream3->currentIndex()) {
        case 0: showSerialOptions(2, 0); break;
        case 1: showTcpOptions(2, 1); break;
        case 2: showTcpOptions(2, 0); break;
        case 3: showTcpOptions(2, 2); break;
    case 4: showTcpOptions(0, 4); break;
    }
}
//---------------------------------------------------------------------------
QString LogStrDialog::getFilePath(const QString &path)
{
    return path.mid(0, path.indexOf("::"));
}
//---------------------------------------------------------------------------
QString LogStrDialog::setFilePath(const QString &p)
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
void LogStrDialog::showSerialOptions(int index, int opt)
{
    if ((index < 0) || (index > 2)) return;

    serialOptDialog->setOptions(opt);
    serialOptDialog->setPath(paths[index][0]);

    serialOptDialog->exec();
    if (serialOptDialog->result() != QDialog::Accepted) return;

    paths[index][0] = serialOptDialog->getPath();
}
//---------------------------------------------------------------------------
void LogStrDialog::showTcpOptions(int index, int opt)
{
    if ((index < 0) || (index > 2)) return;

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
void LogStrDialog::updateEnable()
{
    int ena = (ui->cBStream1C->isChecked() && ui->cBStream1->currentIndex() == 5) ||
              (ui->cBStream2C->isChecked() && ui->cBStream2->currentIndex() == 5) ||
              (ui->cBStream3C->isChecked() && ui->cBStream3->currentIndex() == 5);

    ui->cBStream1->setEnabled(ui->cBStream1C->isChecked());
    ui->cBStream2->setEnabled(ui->cBStream2C->isChecked());
    ui->cBStream3->setEnabled(ui->cBStream3C->isChecked());
    ui->btnStream1->setEnabled(ui->cBStream1C->isChecked() && ui->cBStream1->currentIndex() <= 4);
    ui->btnStream2->setEnabled(ui->cBStream2C->isChecked() && ui->cBStream2->currentIndex() <= 4);
    ui->btnStream3->setEnabled(ui->cBStream3C->isChecked() && ui->cBStream3->currentIndex() <= 4);
    ui->lEFilePath1->setEnabled(ui->cBStream1C->isChecked() && ui->cBStream1->currentIndex() == 5);
    ui->lEFilePath2->setEnabled(ui->cBStream2C->isChecked() && ui->cBStream2->currentIndex() == 5);
    ui->lEFilePath3->setEnabled(ui->cBStream3C->isChecked() && ui->cBStream3->currentIndex() == 5);
    ui->lblSwapInterval->setEnabled(ena);
    ui->lblF1->setEnabled(ena);
    ui->cBTimeTag->setEnabled(ena);
    ui->cBSwapInterval->setEnabled(ena);
    ui->btnKey->setEnabled(ena);
}
//---------------------------------------------------------------------------
void LogStrDialog::setStreamEnabled(int stream, int enabled)
{
    QCheckBox *cBStreamC[] = {ui->cBStream1C, ui->cBStream2C, ui->cBStream3C};
    if ((stream < 0 ) || (stream > 2)) return;

    cBStreamC[stream]->setChecked(enabled);
    updateEnable();
}
//---------------------------------------------------------------------------
int LogStrDialog::getStreamEnabled(int stream)
{
    QCheckBox *cBStreamC[] = {ui->cBStream1C, ui->cBStream2C, ui->cBStream3C};
    if ((stream < 0 ) || (stream > 2)) return false;

    return cBStreamC[stream]->isChecked();
}
//---------------------------------------------------------------------------
void LogStrDialog::setStreamType(int stream, int type)
{
    QComboBox *cBStream[] = {ui->cBStream1, ui->cBStream2, ui->cBStream3};
    if ((stream < 0 ) || (stream > 2)) return;

    cBStream[stream]->setCurrentIndex(type);

    updateEnable();
}
//---------------------------------------------------------------------------
int LogStrDialog::getStreamType(int stream)
{
    QComboBox *cBStream[] = {ui->cBStream1, ui->cBStream2, ui->cBStream3};
    if ((stream < 0 ) || (stream > 2)) return STR_NONE;

    return cBStream[stream]->currentIndex();
};
//---------------------------------------------------------------------------
void LogStrDialog::setPath(int stream, int type, const QString &path)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3};
    if ((stream < 0 ) || (stream > 2)) return;
    if ((type < 0 ) || (type > 3)) return;

    paths[stream][type] = path;
    ui->cBTimeTag->setChecked(path.contains("::T"));
    if (path.contains("::S="))
    {
        int startPos = path.indexOf("::S=")+4;
        QString startTime = path.mid(startPos, path.indexOf("::", startPos)-startPos);
        ui->cBSwapInterval->setCurrentText(startTime);
    };
    if (type == 2)
    {
        edits[stream]->setText(path.mid(0, path.indexOf("::")));
    };
}
//---------------------------------------------------------------------------
QString LogStrDialog::getPath(int stream, int type)
{
    QLineEdit *edits[] = {ui->lEFilePath1, ui->lEFilePath2, ui->lEFilePath3};
    if ((stream < 0 ) || (stream > 2)) return "";
    if ((type < 0 ) || (type > 3)) return "";

    if (type == 2)
        return setFilePath(edits[stream]->text());

    return paths[stream][type];
}
//---------------------------------------------------------------------------
void LogStrDialog::setLogTimeTagEnabled(bool ena)
{
    ui->cBTimeTag->setChecked(ena);
}
//---------------------------------------------------------------------------
bool LogStrDialog::getLogTimeTagEnabled(){
    return ui->cBTimeTag->isChecked();
}
//---------------------------------------------------------------------------
void LogStrDialog::setSwapInterval(const QString & swapInterval)
{
    QString interval_str = swapInterval;
    if (!interval_str.isEmpty()) interval_str += " h";
    if (ui->cBSwapInterval->findText(interval_str) == -1)
        ui->cBSwapInterval->insertItem(0, interval_str);
    ui->cBSwapInterval->setCurrentText(interval_str);
}
//---------------------------------------------------------------------------
QString LogStrDialog::getSwapInterval()
{
    return ui->cBSwapInterval->currentText().split(' ').first();
};
//---------------------------------------------------------------------------
void LogStrDialog::setHistory(int i, const QString &history)
{
    if ((i < 0) || (i > 9)) return;

    this->history[i] = history;
}
//---------------------------------------------------------------------------
const QString LogStrDialog::getHistory(int i)
{
    if ((i < 0) || (i > 9)) return QStringLiteral("");

    return history[i];
}
//---------------------------------------------------------------------------
