//---------------------------------------------------------------------------
#include <stdio.h>

#include <QShowEvent>

#include "mntpoptdlg.h"
#include "ui_mntpoptdlg.h"


//---------------------------------------------------------------------------
MntpOptDialog::MntpOptDialog(QWidget* parent, int option)
    : QDialog(parent), ui(new Ui::MntpOptDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &MntpOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &MntpOptDialog::reject);

    setOption(option);
}
//---------------------------------------------------------------------------
void MntpOptDialog::setOption(int option)
{
    ui->lEMountPoint->setReadOnly(option == 0);
    if (option == 0)
        ui->buttonBox->setStandardButtons(QDialogButtonBox::Ok);
    else
        ui->buttonBox->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
}
//---------------------------------------------------------------------------
QString MntpOptDialog::getMountPointString()
{
    return "STR;" + getMountPoint() +
           ui->lESourceTable3->text() + ";" +
           ui->lESourceTable4->text() + ";" +
           ui->lESourceTable5->text() + ";" +
           QString::number(ui->cBSourceTable6->currentIndex()) + ";" +
           ui->lESourceTable7->text() + ";" +
           ui->lESourceTable8->text() + ";" +
           ui->lESourceTable9->text() + ";" +
           QString::number(ui->sBSourceTable10->value()) + ";" +
           QString::number(ui->sBSourceTable11->value()) + ";" +
           QString::number(ui->cBSourceTable12->currentIndex()) + ";" +
           QString::number(ui->cBSourceTable13->currentIndex()) + ";" +
           ui->lESourceTable14->text() + ";" +
           ui->lESourceTable15->text() + ";" +
           QString::number(ui->cBSourceTable16->currentIndex()) + ";" +
           QString::number(ui->cBSourceTable17->currentIndex()) + ";" +
           QString::number(ui->sBSourceTable18->value());
}
//---------------------------------------------------------------------------
void MntpOptDialog::setMountPointString(QString mountPointString)
{
    QLineEdit *edit[] = {NULL, NULL,
        ui->lESourceTable3, ui->lESourceTable4, ui->lESourceTable5, NULL, ui->lESourceTable7,
        ui->lESourceTable8, ui->lESourceTable9, NULL, NULL,
        NULL, NULL, ui->lESourceTable14, ui->lESourceTable15, NULL, NULL, NULL
    };
    QComboBox *box[] = {NULL, NULL,
        NULL, NULL, NULL, ui->cBSourceTable6, NULL, NULL, NULL, NULL, NULL, ui->cBSourceTable12,
        ui->cBSourceTable13, NULL, NULL, ui->cBSourceTable16, ui->cBSourceTable17, NULL
    };
    QDoubleSpinBox *spinbox[] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, ui->sBSourceTable10,
        ui->sBSourceTable11, NULL, NULL, NULL, NULL, NULL, NULL, ui->sBSourceTable18
    };

    QStringList tokens = mountPointString.split(";");

    for (int i = 2; i < 18; i++) {
        if (edit[i] != NULL) edit[i]->setText("");
        else if (spinbox[i] != NULL) spinbox[i]->setValue(0);
        else if (box[i] != NULL) {box[i]->setCurrentIndex(0);box[i]->setDisabled(true);}
    }

    for (int i = 2; i < tokens.size() && i < 18; i++) {
        if (edit[i] != NULL) edit[i]->setText(tokens.at(i));
        else if (spinbox[i] != NULL) spinbox[i]->setValue(tokens.at(i).toDouble());
        else if (box[i] != NULL) box[i]->setCurrentText(tokens.at(i));
    }
}
//---------------------------------------------------------------------------
QString MntpOptDialog::getMountPoint()
{
    return ui->lEMountPoint->text();
}
//---------------------------------------------------------------------------
void MntpOptDialog::setMountPoint(QString mountPoint)
{
    ui->lEMountPoint->setText(mountPoint);
}
//---------------------------------------------------------------------------
