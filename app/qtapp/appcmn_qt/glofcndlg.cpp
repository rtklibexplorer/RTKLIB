//---------------------------------------------------------------------------

#include <QFileDialog>
#include <QDir>

#include "rtklib.h"
#include "glofcndlg.h"

#include "ui_glofcndlg.h"

//---------------------------------------------------------------------------
GloFcnDialog::GloFcnDialog(QWidget* parent) : QDialog(parent), ui(new Ui::GloFcnDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &GloFcnDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &GloFcnDialog::reject);
    connect(ui->btnClear, &QPushButton::clicked, this, &GloFcnDialog::clearFrequencies);
    connect(ui->btnRead, &QPushButton::clicked, this, &GloFcnDialog::readRinex);
    connect(ui->cBEnableFcn, &QCheckBox::stateChanged, this, &GloFcnDialog::updateEnable);

    updateEnable();
}
//---------------------------------------------------------------------------
void GloFcnDialog::readRinex()
{
    QString filename;
    nav_t nav;
	int prn;

    memset(&nav, 0, sizeof(nav_t));

    filename = QFileDialog::getOpenFileName(this, tr("Open RINEX file"), "", tr("Rinex (*.obs *.*O);;All (*.*)"));

    if (filename.isEmpty()) return;
    
    if (!readrnx(qPrintable(filename), 0, "", NULL, &nav, NULL)) return;
	
    for (int i = 0; i < nav.ng; i++) {
        if (satsys(nav.geph[i].sat, &prn) != SYS_GLO) continue;
        getFcn(prn)->setValue(nav.geph[i].frq);
	}
    freenav(&nav, 0xFF);
}
//---------------------------------------------------------------------------
void GloFcnDialog::clearFrequencies()
{
    for (int i = 0; i < 27; i++) {
        getFcn(i + 1)->setValue(getFcn(i + 1)->minimum());
	}
}
//---------------------------------------------------------------------------
void GloFcnDialog::updateEnable()
{
    ui->btnClear->setEnabled(ui->cBEnableFcn->isChecked());
    ui->btnRead ->setEnabled(ui->cBEnableFcn->isChecked());
    ui->lblDescription1->setEnabled(ui->cBEnableFcn->isChecked());
    ui->lblDescription1->setEnabled(ui->cBEnableFcn->isChecked());
	
    for (int i = 0; i < 27; i++) {
        getFcn(i + 1)->setEnabled(ui->cBEnableFcn->isChecked());
	}
}
//---------------------------------------------------------------------------
QSpinBox * GloFcnDialog::getFcn(int prn)
{
    QSpinBox *fcn[] = {
        ui->sBFcn01, ui->sBFcn02, ui->sBFcn03, ui->sBFcn04, ui->sBFcn05, ui->sBFcn06, ui->sBFcn07, ui->sBFcn08, ui->sBFcn09, ui->sBFcn10,
        ui->sBFcn11, ui->sBFcn12, ui->sBFcn13, ui->sBFcn14, ui->sBFcn15, ui->sBFcn16, ui->sBFcn17, ui->sBFcn18, ui->sBFcn19, ui->sBFcn20,
        ui->sBFcn21, ui->sBFcn22, ui->sBFcn23, ui->sBFcn24, ui->sBFcn25, ui->sBFcn26, ui->sBFcn27
	};
    return fcn[prn - 1];
}
//---------------------------------------------------------------------------
int GloFcnDialog::getGloFcnEnable()
{
    return ui->cBEnableFcn->isChecked();
}
//---------------------------------------------------------------------------
void GloFcnDialog::setGloFcnEnable(int enable)
{
    ui->cBEnableFcn->setChecked(enable);
}
//---------------------------------------------------------------------------
int GloFcnDialog::getGloFcn(int i)
{
    int no = getFcn(i + 1)->value();
    if (no >= -7 && no <= 6) {
        return no + 8;
    } else return 0; // GLONASS FCN+8 (0:none)
}
//---------------------------------------------------------------------------
void GloFcnDialog::setGloFcn(int i, int fcn)
{
    if (fcn)
        getFcn(i + 1)->setValue(fcn - 8);
    else
        getFcn(i + 1)->setValue(getFcn(i + 1)->minimum());
}
//---------------------------------------------------------------------------
