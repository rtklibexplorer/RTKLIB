//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include "keydlg.h"
#include "fileoptdlg.h"

#include <QShowEvent>
#include <QFileDialog>
#include <QIntValidator>
#include <QFileSystemModel>
#include <QCompleter>
#include <QAction>

#include "ui_fileoptdlg.h"


//---------------------------------------------------------------------------
FileOptDialog::FileOptDialog(QWidget *parent, int options, int pathEnabled)
    : QDialog(parent), ui(new Ui::FileOptDialog)
{
    ui->setupUi(this);

    keyDialog = new KeyDialog(this);

    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lEFilePath->setCompleter(fileCompleter);

    QAction *acFilePath = ui->lEFilePath->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acFilePath->setToolTip(tr("Select file path"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &FileOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &FileOptDialog::reject);
    connect(ui->btnKey, &QPushButton::clicked, this, &FileOptDialog::keyDialogShow);
    connect(ui->cBTimeTag, &QCheckBox::clicked, this, &FileOptDialog::updateEnable);
    connect(ui->cBPathEnable, &QCheckBox::clicked, this, &FileOptDialog::updateEnable);
    connect(acFilePath, &QAction::triggered, this, &FileOptDialog::filePathSelect);

    ui->cBSwapInterval->setValidator(new QIntValidator());

    setOptions(options);
    setPathEnabled(pathEnabled);
    updateEnable();
}

//---------------------------------------------------------------------------
void FileOptDialog::setOptions(int options)
{
    this->options = options;

    ui->cBTimeTag->setText(options ? tr("TimeTag") : tr("Time"));
    ui->lbFilePath->setVisible(options != 2);
    ui->cBPathEnable->setVisible(options == 2);
    ui->cBPathEnable->setChecked(options != 2 || pathEnabled);
    ui->cBTimeSpeed->setVisible(!options);
    ui->sBTimeStart->setVisible(!options);
    ui->lbFilePath->setText(options ? tr("Output File Path") : tr("Input File Path"));
    ui->lbPlus->setVisible(!options);
    ui->lbSwapInterval->setVisible(options);
    ui->cBSwapInterval->setVisible(options);
    ui->btnKey->setVisible(options);
}
//---------------------------------------------------------------------------
void FileOptDialog::setPathEnabled(int pathEnabled)
{
    this->pathEnabled = pathEnabled;

    ui->cBPathEnable->setChecked(options != 2 || pathEnabled);
}
//---------------------------------------------------------------------------
void FileOptDialog::setPath(const QString &path)
{
    ui->cBTimeTag->setChecked(false);
    if (!options) { // input
        double speed = 1.0, start = 0.0;
        int size_fpos = 4;

        QStringList tokens = path.split("::");

        QString token;
        foreach(token, tokens){
            if (token == 'T') ui->cBTimeTag->setChecked(true);
            if (token.contains('+')) start = token.toDouble();
            if (token.contains('x')) speed = QStringView{token}.mid(1).toDouble();
            if (token.contains('P')) size_fpos = QStringView{token}.mid(2).toInt();
        }

        if (start <= 0.0) start = 0.0;
        if (speed <= 0.0) speed = 1.0;

        int index = ui->cBTimeSpeed->findText(QStringLiteral("x%1").arg(speed));
        if (index != -1) {
            ui->cBTimeSpeed->setCurrentIndex(index);
        } else {
            ui->cBTimeSpeed->addItem(QStringLiteral("x%1").arg(speed), speed);
            ui->cBTimeSpeed->setCurrentIndex(ui->cBTimeSpeed->count());
        }
        ui->sBTimeStart->setValue(start);
        ui->cB64Bit->setChecked(size_fpos == 8);

        ui->lEFilePath->setText(tokens.at(0));
    } else { // output
        double intv = 0.0;

        QStringList tokens = path.split("::");

        QString token;
        foreach(token, tokens){
            if (token == "T") ui->cBTimeTag->setChecked(true);
            if (token.contains("S=")) intv = QStringView{token}.mid(2).toDouble();
        };
        int index = ui->cBSwapInterval->findText(QStringLiteral("%1 h").arg(intv, 0, 'g', 3));
        if (index != -1) {
            ui->cBSwapInterval->setCurrentIndex(index);
        } else {
            if (intv == 0) {
                ui->cBSwapInterval->setCurrentIndex(0);
            } else {
                ui->cBSwapInterval->addItem(QStringLiteral("%1 h").arg(intv, 0, 'g', 3), intv);
                ui->cBSwapInterval->setCurrentIndex(ui->cBTimeSpeed->count());
            }
        }

        ui->lEFilePath->setText(tokens.at(0));
        setPathEnabled(ui->cBPathEnable->isChecked());
	}
    updateEnable();
}
//---------------------------------------------------------------------------
QString FileOptDialog::getPath()
{
    QString str, path;
    bool okay;

    if (!options) {  // input
        path = ui->lEFilePath->text();
        if (ui->cBTimeTag->isChecked())
            path = path + "::T" + "::" + ui->cBTimeSpeed->currentText() + "::+" + QString::number(ui->sBTimeStart->value());
        if (ui->cB64Bit->isChecked()) {
            path = path + "::P=8";
        }
    } else { // output
        path = ui->lEFilePath->text();
        if (ui->cBTimeTag->isChecked()) path += "::T";
        str = ui->cBSwapInterval->currentText();
        str = str.split(" ").first();
        str.toDouble(&okay);
        if (okay)
            path += "::S=" + str;
    }
    return path;
}
//---------------------------------------------------------------------------
void FileOptDialog::filePathSelect()
{
    if (!options)
        ui->lEFilePath->setText(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, QString(), ui->lEFilePath->text())));
    else
        ui->lEFilePath->setText(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, QString(), ui->lEFilePath->text())));
}
//---------------------------------------------------------------------------
void FileOptDialog::keyDialogShow()
{
    keyDialog->exec();
}
//---------------------------------------------------------------------------
void FileOptDialog::updateEnable()
{

    ui->cBTimeSpeed->setEnabled(ui->cBTimeTag->isChecked());
    ui->sBTimeStart->setEnabled(ui->cBTimeTag->isChecked());
    ui->lbPlus->setEnabled(ui->cBTimeTag->isChecked());
    ui->cB64Bit->setEnabled(ui->cBTimeTag->isChecked());

    ui->lEFilePath->setEnabled(ui->cBPathEnable->isChecked());
    ui->lbSwapInterval->setEnabled(ui->cBPathEnable->isChecked());
    ui->cBSwapInterval->setEnabled(ui->cBPathEnable->isChecked());
    ui->lbFilePath->setEnabled(ui->cBPathEnable->isChecked());
    ui->cBTimeTag->setEnabled(ui->cBPathEnable->isChecked());
    ui->btnKey->setEnabled(ui->cBPathEnable->isChecked());
}
