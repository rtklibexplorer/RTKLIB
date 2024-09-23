//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QFileDialog>

#include "staoptdlg.h"

#include "ui_staoptdlg.h"

//---------------------------------------------------------------------------
StaListDialog::StaListDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::StaListDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &StaListDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &StaListDialog::reject);
    connect(ui->btnLoad, &QPushButton::clicked, this, &StaListDialog::loadFile);
    connect(ui->btnSave, &QPushButton::clicked, this, &StaListDialog::saveFile);
}
//---------------------------------------------------------------------------
void StaListDialog::setStationList(const QStringList &list)
{
    ui->lWStationList->clear();

    for (int i = 0; i < list.count(); i++)
        ui->lWStationList->addItem(list.at(i));
}
//---------------------------------------------------------------------------
QStringList StaListDialog::getStationList()
{
    QStringList list;

    for (int i = 0; i < ui->lWStationList->count(); i++)
        list.append(ui->lWStationList->item(i)->text());

    return list;
}
//---------------------------------------------------------------------------
void StaListDialog::loadFile()
{
    QString filename;
    QFile file;
    QByteArray buffer;

    filename = QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Open...")));

    file.setFileName(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;

    ui->lWStationList->clear();
    ui->lWStationList->setVisible(false);

    while (!file.atEnd()) {
        buffer = file.readLine();
        buffer = buffer.mid(buffer.indexOf('#'));
        ui->lWStationList->addItem(buffer.trimmed());
    }

    ui->lWStationList->setVisible(true);
}
//---------------------------------------------------------------------------
void StaListDialog::saveFile()
{
    QString filename = QDir::toNativeSeparators(QFileDialog::getSaveFileName(this, tr("Save...")));
    QFile file;

    file.setFileName(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;

    for (int i = 0; i < ui->lWStationList->count(); i++)
        file.write((ui->lWStationList->item(i)->text() + '\n').toLatin1());
}
//---------------------------------------------------------------------------
