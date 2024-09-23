//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include "keydlg.h"

#include "ui_keydlg.h"


//---------------------------------------------------------------------------
KeyDialog::KeyDialog(QWidget *parent, int flag)
    : QDialog(parent), ui (new Ui::KeyDialog)
{
    ui->setupUi(this);

    setFlag(flag);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &KeyDialog::accept);
}
//---------------------------------------------------------------------------
void KeyDialog::setFlag(int flag)
{
    ui->labelS->setVisible(flag != 3);
    ui->labelSecond->setVisible(flag != 3);
    ui->labelha->setVisible(flag != 3);
    ui->label3H_Hour->setVisible(flag != 3);
    ui->labelhb->setVisible(flag != 3);
    ui->label6H_Hour->setVisible(flag != 3);
    ui->labelhc->setVisible(flag != 3);
    ui->label12H_Hour->setVisible(flag != 3);
    ui->labelt->setVisible(flag != 3);
    ui->label15_min->setVisible(flag != 3);

    ui->labelr->setVisible(flag >= 1);
    ui->labelStationID->setVisible(flag >= 1);

    ui->labels->setVisible(flag == 3);
    ui->labelStationID_L->setVisible(flag == 3);
    ui->labelS_2->setVisible(flag == 3);
    ui->labelStationID_U->setVisible(flag == 3);
    ui->labelN->setVisible(flag == 3);
    ui->labelSequenceNumber->setVisible(flag == 3);

    ui->labelb->setVisible(flag == 2);
    ui->labelBaseStationID->setVisible(flag == 2);
}
//---------------------------------------------------------------------------
