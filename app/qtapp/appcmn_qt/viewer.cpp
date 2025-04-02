//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann
#include <QShowEvent>
#include <QFileDialog>
#include <QPalette>
#include <QDebug>

#include "rtklib.h"
#include "viewer.h"
#include "vieweropt.h"

#include "ui_viewer.h"

//---------------------------------------------------------------------------
TextViewer::TextViewer(QWidget *parent, int option)
    : QDialog(parent), ui(new Ui::TextViewer)
{
    ui->setupUi(this);

    colorText = Qt::black;
    colorBackground = Qt::white;

    viewerOptDialog = new ViewerOptDialog(this);

    connect(ui->btnClose, &QPushButton::clicked, this, &TextViewer::accept);
    connect(ui->btnFind, &QPushButton::clicked, this, &TextViewer::findText);
    connect(ui->btnOptions, &QPushButton::clicked, this, &TextViewer::showOptions);
    connect(ui->btnReadSave, &QPushButton::clicked, this, &TextViewer::readSaveFile);
    connect(ui->btnReload, &QPushButton::clicked, this, &TextViewer::reloadText);
    connect(ui->findStr, &QLineEdit::editingFinished, this, &TextViewer::findText);

    setOption(option);
}
//---------------------------------------------------------------------------
void TextViewer::setOption(int option)
{
    if (option == 0) {
        ui->btnReload->setVisible(false);
        ui->btnReadSave->setVisible(false);
    } else if (option == 2) {
        ui->btnReload->setVisible(false);
        ui->btnReadSave->setText(tr("&Save..."));
    }
}
//---------------------------------------------------------------------------
void TextViewer::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

	updateText();
}
//---------------------------------------------------------------------------
void TextViewer::reloadText()
{
	read(file);
}
//---------------------------------------------------------------------------
void TextViewer::readSaveFile()
{
    if (ui->btnReadSave->text() == tr("&Save..."))
        save(QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, QString(), file)));
    else
        read(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, QString(), file)));
}
//---------------------------------------------------------------------------
void TextViewer::showOptions()
{
    viewerOptDialog->setFont(font);
    viewerOptDialog->setTextColor(colorText);
    viewerOptDialog->setBackgroundColor(colorBackground);

    viewerOptDialog->move(this->size().width() / 2 - viewerOptDialog->size().width() / 2,
                          this->size().height() / 2 - viewerOptDialog->size().height() / 2);
    viewerOptDialog->exec();

    if (viewerOptDialog->result() != QDialog::Accepted) return;

    font = viewerOptDialog->getFont();
    colorText = viewerOptDialog->getTextColor();
    colorBackground = viewerOptDialog->getBackgroundColor();

	updateText();
}
//---------------------------------------------------------------------------
void TextViewer::findText()
{
    ui->textEdit->find(ui->findStr->text());
}
//---------------------------------------------------------------------------
bool TextViewer::read(const QString &path)
{
    char filename[1024], *p[] = {filename};

    if (expath(qPrintable(path), p, 1) < 1) return false;

    QFile f(filename);
    if (!f.open(QIODevice::ReadOnly)) return false;

    ui->textEdit->setPlainText(f.readAll());

    file = filename;
    setWindowTitle(file);

    return true;
}
//---------------------------------------------------------------------------
bool TextViewer::save(const QString &filename)
{
    QFile f(filename);

    if (!f.open(QIODevice::WriteOnly)) return false;

    f.write(ui->textEdit->toPlainText().toLocal8Bit());
    file = filename;

    return true;
}
//---------------------------------------------------------------------------
void TextViewer::updateText()
{
    QPalette pal;

    ui->textEdit->setFont(font);
    pal = ui->textEdit->palette();
    pal.setColor(QPalette::Text, colorText);
    pal.setColor(QPalette::Base, colorBackground);
    ui->textEdit->setPalette(pal);
}
//---------------------------------------------------------------------------
void TextViewer::loadOptions(QSettings & ini)
{
    colorText = ini.value("viewer/color1", QColor(Qt::black)).value<QColor>();
    colorBackground = ini.value("viewer/color2", QColor(Qt::white)).value<QColor>();
    font.setFamily(ini.value ("viewer/fontname", "Courier New").toString());
    font.setPointSize(ini.value("viewer/fontsize", 9).toInt());
}
//---------------------------------------------------------------------------
void TextViewer::saveOptions(QSettings & ini)
{
    ini.setValue ("viewer/color1", colorText);
    ini.setValue ("viewer/color2", colorBackground);
    ini.setValue ("viewer/fontname", font.family());
    ini.setValue ("viewer/fontsize", font.pointSize());
}
//---------------------------------------------------------------------------
