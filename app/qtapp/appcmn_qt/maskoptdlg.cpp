//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include <QShowEvent>

#include "maskoptdlg.h"

#include "ui_maskoptdlg.h"

//---------------------------------------------------------------------------
MaskOptDialog::MaskOptDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::MaskOptDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &MaskOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &MaskOptDialog::reject);
    connect(ui->cBMaskEnabled1, &QCheckBox::clicked, this, &MaskOptDialog::updateEnable);
    connect(ui->cBMaskEnabled2, &QCheckBox::clicked, this, &MaskOptDialog::updateEnable);

    updateEnable();
}
//---------------------------------------------------------------------------
void MaskOptDialog::setSnrMask(snrmask_t mask)
{
    QDoubleSpinBox *widgets[][9] = {
        { ui->sBMask_1_1, ui->sBMask_1_2, ui->sBMask_1_3, ui->sBMask_1_4, ui->sBMask_1_5, ui->sBMask_1_6, ui->sBMask_1_7, ui->sBMask_1_8, ui->sBMask_1_9 },
        { ui->sBMask_2_1, ui->sBMask_2_2, ui->sBMask_2_3, ui->sBMask_2_4, ui->sBMask_2_5, ui->sBMask_2_6, ui->sBMask_2_7, ui->sBMask_2_8, ui->sBMask_2_9 },
        { ui->sBMask_3_1, ui->sBMask_3_2, ui->sBMask_3_3, ui->sBMask_3_4, ui->sBMask_3_5, ui->sBMask_3_6, ui->sBMask_3_7, ui->sBMask_3_8, ui->sBMask_3_9 },
        { ui->sBMask_4_1, ui->sBMask_4_2, ui->sBMask_4_3, ui->sBMask_4_4, ui->sBMask_4_5, ui->sBMask_4_6, ui->sBMask_4_7, ui->sBMask_4_8, ui->sBMask_4_9 },
        { ui->sBMask_5_1, ui->sBMask_5_2, ui->sBMask_5_3, ui->sBMask_5_4, ui->sBMask_5_5, ui->sBMask_5_6, ui->sBMask_5_7, ui->sBMask_5_8, ui->sBMask_5_9 },
        { ui->sBMask_6_1, ui->sBMask_6_2, ui->sBMask_6_3, ui->sBMask_6_4, ui->sBMask_6_5, ui->sBMask_6_6, ui->sBMask_6_7, ui->sBMask_6_8, ui->sBMask_6_9 }
    };
    
    ui->cBMaskEnabled1->setChecked(mask.ena[0]);
    ui->cBMaskEnabled2->setChecked(mask.ena[1]);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 9; j++) {
            widgets[i][j]->setValue(mask.mask[i][j]);
        }
    }
    updateEnable();
}
//---------------------------------------------------------------------------
snrmask_t MaskOptDialog::getSnrMask()
{
    snrmask_t mask;
    QDoubleSpinBox *widgets[][9] = {
        { ui->sBMask_1_1, ui->sBMask_1_2, ui->sBMask_1_3, ui->sBMask_1_4, ui->sBMask_1_5, ui->sBMask_1_6, ui->sBMask_1_7, ui->sBMask_1_8, ui->sBMask_1_9 },
        { ui->sBMask_2_1, ui->sBMask_2_2, ui->sBMask_2_3, ui->sBMask_2_4, ui->sBMask_2_5, ui->sBMask_2_6, ui->sBMask_2_7, ui->sBMask_2_8, ui->sBMask_2_9 },
        { ui->sBMask_3_1, ui->sBMask_3_2, ui->sBMask_3_3, ui->sBMask_3_4, ui->sBMask_3_5, ui->sBMask_3_6, ui->sBMask_3_7, ui->sBMask_3_8, ui->sBMask_3_9 },
        { ui->sBMask_4_1, ui->sBMask_4_2, ui->sBMask_4_3, ui->sBMask_4_4, ui->sBMask_4_5, ui->sBMask_4_6, ui->sBMask_4_7, ui->sBMask_4_8, ui->sBMask_4_9 },
        { ui->sBMask_5_1, ui->sBMask_5_2, ui->sBMask_5_3, ui->sBMask_5_4, ui->sBMask_5_5, ui->sBMask_5_6, ui->sBMask_5_7, ui->sBMask_5_8, ui->sBMask_5_9 },
        { ui->sBMask_6_1, ui->sBMask_6_2, ui->sBMask_6_3, ui->sBMask_6_4, ui->sBMask_6_5, ui->sBMask_6_6, ui->sBMask_6_7, ui->sBMask_6_8, ui->sBMask_6_9 }
    };
    
    mask.ena[0] = ui->cBMaskEnabled1->isChecked();
    mask.ena[1] = ui->cBMaskEnabled2->isChecked();
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 9; j++)
            mask.mask[i][j] = widgets[i][j]->value();

    return mask;
}
//---------------------------------------------------------------------------
void MaskOptDialog::updateEnable()
{
    QDoubleSpinBox *widgets[][9] = {
        { ui->sBMask_1_1, ui->sBMask_1_2, ui->sBMask_1_3, ui->sBMask_1_4, ui->sBMask_1_5, ui->sBMask_1_6, ui->sBMask_1_7, ui->sBMask_1_8, ui->sBMask_1_9 },
        { ui->sBMask_2_1, ui->sBMask_2_2, ui->sBMask_2_3, ui->sBMask_2_4, ui->sBMask_2_5, ui->sBMask_2_6, ui->sBMask_2_7, ui->sBMask_2_8, ui->sBMask_2_9 },
        { ui->sBMask_3_1, ui->sBMask_3_2, ui->sBMask_3_3, ui->sBMask_3_4, ui->sBMask_3_5, ui->sBMask_3_6, ui->sBMask_3_7, ui->sBMask_3_8, ui->sBMask_3_9 },
        { ui->sBMask_4_1, ui->sBMask_4_2, ui->sBMask_4_3, ui->sBMask_4_4, ui->sBMask_4_5, ui->sBMask_4_6, ui->sBMask_4_7, ui->sBMask_4_8, ui->sBMask_4_9 },
        { ui->sBMask_5_1, ui->sBMask_5_2, ui->sBMask_5_3, ui->sBMask_5_4, ui->sBMask_5_5, ui->sBMask_5_6, ui->sBMask_5_7, ui->sBMask_5_8, ui->sBMask_5_9 },
        { ui->sBMask_6_1, ui->sBMask_6_2, ui->sBMask_6_3, ui->sBMask_6_4, ui->sBMask_6_5, ui->sBMask_6_6, ui->sBMask_6_7, ui->sBMask_6_8, ui->sBMask_6_9 }
	};

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 9; j++)
            widgets[i][j]->setEnabled(ui->cBMaskEnabled1->isChecked() || ui->cBMaskEnabled2->isChecked());

}
//---------------------------------------------------------------------------
