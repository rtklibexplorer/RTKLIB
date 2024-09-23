//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include "aboutdlg.h"
#include "rtklib.h"

#include "ui_aboutdlg.h"


//---------------------------------------------------------------------------
AboutDialog::AboutDialog(QWidget *parent, QPixmap icon, QString aboutString)
    : QDialog(parent), ui(new Ui::AboutDlg)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &AboutDialog::accept);

    ui->lbIcon->setPixmap(icon.scaled(128, 128));
    ui->lbAbout->setText(QStringLiteral("<b>%1</b>").arg(aboutString));
    ui->lbVersion->setText(tr("with RTKLIB Version %1 %2").arg(VER_RTKLIB, PATCH_LEVEL));
    ui->lbCopyright->setText(COPYRIGHT_RTKLIB);
    ui->lbCopyrightQt->setText(tr("Qt Version\nCopyright (C) 2016-2024 Jens Reimann"));
    ui->lbWebsite->setText(tr("Support: <a href=\"https://github.com/rtklibexplorer\">https://github.com/rtklibexplorer</a>"));
}
