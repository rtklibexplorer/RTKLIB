//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann

#include <QDialog>
#include <QShowEvent>
#include <QFont>
#include <QPalette>
#include <QColorDialog>
#include <QFontDialog>
#include <QLocale>

#include "helper.h"
#include "viewer.h"
#include "vieweropt.h"

#include "ui_vieweropt.h"


//---------------------------------------------------------------------------
ViewerOptDialog::ViewerOptDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ViewerOptDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ViewerOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &ViewerOptDialog::reject);
    connect(ui->btnFont, &QPushButton::clicked, this, &ViewerOptDialog::selectFont);
    connect(ui->btnColorText, &QPushButton::clicked, this, &ViewerOptDialog::selectTextColor);
    connect(ui->btnColorBackground, &QPushButton::clicked, this, &ViewerOptDialog::selectBackgroundColor);
}
//---------------------------------------------------------------------------
void ViewerOptDialog::setFont(const QFont &font)
{
    ui->fontLabel->setFont(font);
    ui->fontLabel->setText(QStringLiteral("%1 %2 px").arg(font.family()).arg(font.pointSize()));
}
//---------------------------------------------------------------------------
void ViewerOptDialog::setTextColor(const QColor &color)
{
    colorText = color;
    setWidgetBackgroundColor(ui->lbColorText, colorText);
}
//---------------------------------------------------------------------------
void ViewerOptDialog::setBackgroundColor(const QColor &color)
{
    colorBackground = color;
    setWidgetBackgroundColor(ui->lbColorBackground, colorBackground);
}
//---------------------------------------------------------------------------
void ViewerOptDialog::selectTextColor()
{
    QColorDialog d;

    d.setCurrentColor(colorText);
    d.exec();
    colorText = d.selectedColor();

    setWidgetBackgroundColor(ui->lbColorText, colorText);
}
//---------------------------------------------------------------------------
void ViewerOptDialog::selectBackgroundColor()
{
    QColorDialog d;

    d.setCurrentColor(colorBackground);
    d.exec();
    colorBackground = d.selectedColor();

    setWidgetBackgroundColor(ui->lbColorBackground, colorBackground);
}
//---------------------------------------------------------------------------
void ViewerOptDialog::selectFont()
{
    QFontDialog d;

    d.setCurrentFont(font);
    d.exec();

    font = d.selectedFont();

    ui->fontLabel->setFont(font);
    ui->fontLabel->setText(font.family() + QLocale().toString(font.pointSize()) + "pt");
}
//---------------------------------------------------------------------------
