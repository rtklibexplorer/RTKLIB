//---------------------------------------------------------------------------
#include <QDebug>

#include "convopt.h"
#include "codeopt.h"

#include "ui_codeopt.h"

#include "rtklib.h"

//---------------------------------------------------------------------------
CodeOptDialog::CodeOptDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::CodeOptDialog)
{
    ui->setupUi(this);
    
    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &CodeOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &CodeOptDialog::reject);
    connect(ui->btnSetAll, &QPushButton::clicked, this, &CodeOptDialog::setUnsetAll);
}
//---------------------------------------------------------------------------
void CodeOptDialog::setCodeMask(int i, const QString &mask)
{
    if (i == 0) { // GPui->S
        ui->G01->setChecked(mask[0] == '1');
        ui->G02->setChecked(mask[1] == '1');
        ui->G03->setChecked(mask[2] == '1');
        ui->G04->setChecked(mask[3] == '1');
        ui->G05->setChecked(mask[4] == '1');
        ui->G06->setChecked(mask[5] == '1');
        ui->G07->setChecked(mask[6] == '1');
        ui->G08->setChecked(mask[7] == '1');
        ui->G12->setChecked(mask[11] == '1');
        ui->G14->setChecked(mask[13] == '1');
        ui->G15->setChecked(mask[14] == '1');
        ui->G16->setChecked(mask[15] == '1');
        ui->G17->setChecked(mask[16] == '1');
        ui->G18->setChecked(mask[17] == '1');
        ui->G19->setChecked(mask[18] == '1');
        ui->G20->setChecked(mask[19] == '1');
        ui->G21->setChecked(mask[20] == '1');
        ui->G22->setChecked(mask[21] == '1');
        ui->G23->setChecked(mask[22] == '1');
        ui->G24->setChecked(mask[23] == '1');
        ui->G25->setChecked(mask[24] == '1');
        ui->G26->setChecked(mask[25] == '1');
    } else if (i == 1) { // GLONASS
        ui->R01->setChecked(mask[0] == '1');
        ui->R02->setChecked(mask[1] == '1');
        ui->R14->setChecked(mask[13] == '1');
        ui->R19->setChecked(mask[18] == '1');
        ui->R30->setChecked(mask[29] == '1');
        ui->R31->setChecked(mask[30] == '1');
        ui->R33->setChecked(mask[32] == '1');
        ui->R44->setChecked(mask[43] == '1');
        ui->R45->setChecked(mask[44] == '1');
        ui->R46->setChecked(mask[45] == '1');
        ui->R66->setChecked(mask[65] == '1');
        ui->R67->setChecked(mask[66] == '1');
        ui->R68->setChecked(mask[67] == '1');
    } else if (i == 2) { // Galileo
        ui->E01->setChecked(mask[0] == '1');
        ui->E10->setChecked(mask[9] == '1');
        ui->E11->setChecked(mask[10] == '1');
        ui->E12->setChecked(mask[11] == '1');
        ui->E13->setChecked(mask[12] == '1');
        ui->E24->setChecked(mask[23] == '1');
        ui->E25->setChecked(mask[24] == '1');
        ui->E26->setChecked(mask[25] == '1');
        ui->E27->setChecked(mask[26] == '1');
        ui->E28->setChecked(mask[27] == '1');
        ui->E29->setChecked(mask[28] == '1');
        ui->E30->setChecked(mask[29] == '1');
        ui->E31->setChecked(mask[30] == '1');
        ui->E32->setChecked(mask[31] == '1');
        ui->E33->setChecked(mask[32] == '1');
        ui->E34->setChecked(mask[33] == '1');
        ui->E37->setChecked(mask[36] == '1');
        ui->E38->setChecked(mask[37] == '1');
        ui->E39->setChecked(mask[38] == '1');
    } else if (i==3) { // QZSS
        ui->J01->setChecked(mask[0] == '1');
        ui->J07->setChecked(mask[6] == '1');
        ui->J08->setChecked(mask[7] == '1');
        ui->J11->setChecked(mask[11] == '1');
        ui->J12->setChecked(mask[11] == '1');
        ui->J13->setChecked(mask[12] == '1');
        ui->J16->setChecked(mask[15] == '1');
        ui->J17->setChecked(mask[16] == '1');
        ui->J18->setChecked(mask[17] == '1');
        ui->J24->setChecked(mask[23] == '1');
        ui->J25->setChecked(mask[24] == '1');
        ui->J26->setChecked(mask[25] == '1');
        ui->J33->setChecked(mask[32] == '1');
        ui->J34->setChecked(mask[33] == '1');
        ui->J35->setChecked(mask[34] == '1');
        ui->J36->setChecked(mask[35] == '1');
        ui->J57->setChecked(mask[56] == '1');
        ui->J58->setChecked(mask[57] == '1');
        ui->J59->setChecked(mask[58] == '1');
        ui->J60->setChecked(mask[59] == '1');
    } else if (i == 4){ // SBAS
        ui->S01->setChecked(mask[0] == '1');
        ui->S24->setChecked(mask[23] == '1');
        ui->S25->setChecked(mask[24] == '1');
        ui->S26->setChecked(mask[25] == '1');
    } else if (i == 5) { //BDS
        ui->C02->setChecked(mask[1] == '1');
        ui->C08->setChecked(mask[7] == '1');
        ui->C09->setChecked(mask[8] == '1');
        ui->C12->setChecked(mask[11] == '1');
        ui->C13->setChecked(mask[12] == '1');
        ui->C18->setChecked(mask[17] == '1');
        ui->C26->setChecked(mask[25] == '1');
        ui->C27->setChecked(mask[26] == '1');
        ui->C28->setChecked(mask[27] == '1');
        ui->C29->setChecked(mask[28] == '1');
        ui->C33->setChecked(mask[32] == '1');
        ui->C39->setChecked(mask[38] == '1');
        ui->C40->setChecked(mask[39] == '1');
        ui->C41->setChecked(mask[40] == '1');
        ui->C42->setChecked(mask[41] == '1');
        ui->C43->setChecked(mask[42] == '1');
        ui->C56->setChecked(mask[55] == '1');
        ui->C57->setChecked(mask[56] == '1');
        ui->C58->setChecked(mask[57] == '1');
        ui->C61->setChecked(mask[60] == '1');
        ui->C62->setChecked(mask[61] == '1');
        ui->C63->setChecked(mask[62] == '1');
        ui->C64->setChecked(mask[63] == '1');
        ui->C65->setChecked(mask[64] == '1');
    } else if (i == 6) { // NavIC
        ui->I26->setChecked(mask[25] == '1');
        ui->I49->setChecked(mask[48] == '1');
        ui->I50->setChecked(mask[49] == '1');
        ui->I51->setChecked(mask[50] == '1');
        ui->I52->setChecked(mask[51] == '1');
        ui->I53->setChecked(mask[52] == '1');
        ui->I54->setChecked(mask[53] == '1');
        ui->I55->setChecked(mask[54] == '1');
    }
}
//---------------------------------------------------------------------------
QString CodeOptDialog::getCodeMask(int i)
{
    QString mask;

    mask.fill('0', MAXCODE);

    if (i == 0) { // GPS
        if (ui->G01->isChecked()) mask[0] = '1';
        if (ui->G02->isChecked()) mask[1] = '1';
        if (ui->G03->isChecked()) mask[2] = '1';
        if (ui->G04->isChecked()) mask[3] = '1';
        if (ui->G05->isChecked()) mask[4] = '1';
        if (ui->G06->isChecked()) mask[5] = '1';
        if (ui->G07->isChecked()) mask[6] = '1';
        if (ui->G08->isChecked()) mask[7] = '1';
        if (ui->G12->isChecked()) mask[11] = '1';
        if (ui->G14->isChecked()) mask[13] = '1';
        if (ui->G15->isChecked()) mask[14] = '1';
        if (ui->G16->isChecked()) mask[15] = '1';
        if (ui->G17->isChecked()) mask[16] = '1';
        if (ui->G18->isChecked()) mask[17] = '1';
        if (ui->G19->isChecked()) mask[18] = '1';
        if (ui->G20->isChecked()) mask[19] = '1';
        if (ui->G21->isChecked()) mask[20] = '1';
        if (ui->G22->isChecked()) mask[21] = '1';
        if (ui->G23->isChecked()) mask[22] = '1';
        if (ui->G24->isChecked()) mask[23] = '1';
        if (ui->G25->isChecked()) mask[24] = '1';
        if (ui->G26->isChecked()) mask[25] = '1';
    } else if (i == 1) { // GLONASS
        if (ui->R01->isChecked()) mask[0] = '1';
        if (ui->R02->isChecked()) mask[1] = '1';
        if (ui->R14->isChecked()) mask[13] = '1';
        if (ui->R19->isChecked()) mask[18] = '1';
        if (ui->R30->isChecked()) mask[29] = '1';
        if (ui->R31->isChecked()) mask[30] = '1';
        if (ui->R33->isChecked()) mask[32] = '1';
        if (ui->R44->isChecked()) mask[43] = '1';
        if (ui->R45->isChecked()) mask[44] = '1';
        if (ui->R46->isChecked()) mask[45] = '1';
        if (ui->R66->isChecked()) mask[65] = '1';
        if (ui->R67->isChecked()) mask[66] = '1';
        if (ui->R68->isChecked()) mask[67] = '1';
    } else if (i == 2) { // Galileo
        if (ui->E01->isChecked()) mask[0] = '1';
        if (ui->E10->isChecked()) mask[9] = '1';
        if (ui->E11->isChecked()) mask[10] = '1';
        if (ui->E12->isChecked()) mask[11] = '1';
        if (ui->E13->isChecked()) mask[12] = '1';
        if (ui->E24->isChecked()) mask[23] = '1';
        if (ui->E25->isChecked()) mask[24] = '1';
        if (ui->E26->isChecked()) mask[25] = '1';
        if (ui->E27->isChecked()) mask[26] = '1';
        if (ui->E28->isChecked()) mask[27] = '1';
        if (ui->E29->isChecked()) mask[28] = '1';
        if (ui->E30->isChecked()) mask[29] = '1';
        if (ui->E31->isChecked()) mask[30] = '1';
        if (ui->E32->isChecked()) mask[31] = '1';
        if (ui->E33->isChecked()) mask[32] = '1';
        if (ui->E34->isChecked()) mask[33] = '1';
        if (ui->E37->isChecked()) mask[36] = '1';
        if (ui->E38->isChecked()) mask[37] = '1';
        if (ui->E39->isChecked()) mask[38] = '1';
    } else if (i == 3) { // QZSS
        if (ui->J01->isChecked()) mask[0] = '1';
        if (ui->J07->isChecked()) mask[6] = '1';
        if (ui->J08->isChecked()) mask[7] = '1';
        if (ui->J11->isChecked()) mask[11] = '1';
        if (ui->J12->isChecked()) mask[11] = '1';
        if (ui->J13->isChecked()) mask[12] = '1';
        if (ui->J16->isChecked()) mask[15] = '1';
        if (ui->J17->isChecked()) mask[16] = '1';
        if (ui->J18->isChecked()) mask[17] = '1';
        if (ui->J24->isChecked()) mask[23] = '1';
        if (ui->J25->isChecked()) mask[24] = '1';
        if (ui->J26->isChecked()) mask[25] = '1';
        if (ui->J33->isChecked()) mask[32] = '1';
        if (ui->J34->isChecked()) mask[33] = '1';
        if (ui->J35->isChecked()) mask[34] = '1';
        if (ui->J36->isChecked()) mask[35] = '1';
        if (ui->J57->isChecked()) mask[56] = '1';
        if (ui->J58->isChecked()) mask[57] = '1';
        if (ui->J59->isChecked()) mask[58] = '1';
        if (ui->J60->isChecked()) mask[59] = '1';
    } else if (i == 4) { // SBAS
        if (ui->S01->isChecked()) mask[0] = '1';
        if (ui->S24->isChecked()) mask[23] = '1';
        if (ui->S25->isChecked()) mask[24] = '1';
        if (ui->S26->isChecked()) mask[25] = '1';
    } else if (i == 5) { // BDS
        if (ui->C02->isChecked()) mask[1] = '1';
        if (ui->C08->isChecked()) mask[7] = '1';
        if (ui->C09->isChecked()) mask[8] = '1';
        if (ui->C12->isChecked()) mask[11] = '1';
        if (ui->C13->isChecked()) mask[12] = '1';
        if (ui->C18->isChecked()) mask[17] = '1';
        if (ui->C26->isChecked()) mask[25] = '1';
        if (ui->C27->isChecked()) mask[26] = '1';
        if (ui->C28->isChecked()) mask[27] = '1';
        if (ui->C29->isChecked()) mask[28] = '1';
        if (ui->C33->isChecked()) mask[32] = '1';
        if (ui->C39->isChecked()) mask[38] = '1';
        if (ui->C40->isChecked()) mask[39] = '1';
        if (ui->C41->isChecked()) mask[40] = '1';
        if (ui->C42->isChecked()) mask[41] = '1';
        if (ui->C43->isChecked()) mask[42] = '1';
        if (ui->C56->isChecked()) mask[55] = '1';
        if (ui->C57->isChecked()) mask[56] = '1';
        if (ui->C58->isChecked()) mask[57] = '1';
        if (ui->C61->isChecked()) mask[60] = '1';
        if (ui->C62->isChecked()) mask[61] = '1';
        if (ui->C63->isChecked()) mask[62] = '1';
        if (ui->C64->isChecked()) mask[63] = '1';
        if (ui->C65->isChecked()) mask[64] = '1';
    } else if (i == 6) { // NavIC
        if (ui->I26->isChecked()) mask[25] = '1';
        if (ui->I49->isChecked()) mask[48] = '1';
        if (ui->I50->isChecked()) mask[49] = '1';
        if (ui->I51->isChecked()) mask[50] = '1';
        if (ui->I52->isChecked()) mask[51] = '1';
        if (ui->I53->isChecked()) mask[52] = '1';
        if (ui->I54->isChecked()) mask[53] = '1';
        if (ui->I55->isChecked()) mask[54] = '1';
    }

    return mask;
}
//---------------------------------------------------------------------------
void CodeOptDialog::setUnsetAll()
{
    bool set = (ui->btnSetAll->text() == tr("&Set All"));

    ui->G01->setChecked(set);
    ui->G02->setChecked(set);
    ui->G03->setChecked(set);
    ui->G04->setChecked(set);
    ui->G05->setChecked(set);
    ui->G06->setChecked(set);
    ui->G07->setChecked(set);
    ui->G08->setChecked(set);
    ui->G12->setChecked(set);
    ui->G14->setChecked(set);
    ui->G15->setChecked(set);
    ui->G16->setChecked(set);
    ui->G17->setChecked(set);
    ui->G18->setChecked(set);
    ui->G19->setChecked(set);
    ui->G20->setChecked(set);
    ui->G21->setChecked(set);
    ui->G22->setChecked(set);
    ui->G23->setChecked(set);
    ui->G24->setChecked(set);
    ui->G25->setChecked(set);
    ui->G26->setChecked(set);

    ui->R01->setChecked(set);
    ui->R02->setChecked(set);
    ui->R14->setChecked(set);
    ui->R19->setChecked(set);
    ui->R30->setChecked(set);
    ui->R31->setChecked(set);
    ui->R33->setChecked(set);
    ui->R44->setChecked(set);
    ui->R45->setChecked(set);
    ui->R46->setChecked(set);
    ui->R66->setChecked(set);
    ui->R67->setChecked(set);
    ui->R68->setChecked(set);

    ui->E01->setChecked(set);
    ui->E10->setChecked(set);
    ui->E11->setChecked(set);
    ui->E12->setChecked(set);
    ui->E13->setChecked(set);
    ui->E24->setChecked(set);
    ui->E25->setChecked(set);
    ui->E26->setChecked(set);
    ui->E27->setChecked(set);
    ui->E28->setChecked(set);
    ui->E29->setChecked(set);
    ui->E30->setChecked(set);
    ui->E31->setChecked(set);
    ui->E32->setChecked(set);
    ui->E33->setChecked(set);
    ui->E34->setChecked(set);
    ui->E37->setChecked(set);
    ui->E38->setChecked(set);
    ui->E39->setChecked(set);

    ui->J01->setChecked(set);
    ui->J07->setChecked(set);
    ui->J08->setChecked(set);
    ui->J11->setChecked(set);
    ui->J12->setChecked(set);
    ui->J13->setChecked(set);
    ui->J16->setChecked(set);
    ui->J17->setChecked(set);
    ui->J18->setChecked(set);
    ui->J24->setChecked(set);
    ui->J25->setChecked(set);
    ui->J26->setChecked(set);
    ui->J33->setChecked(set);
    ui->J34->setChecked(set);
    ui->J35->setChecked(set);
    ui->J36->setChecked(set);
    ui->J57->setChecked(set);
    ui->J58->setChecked(set);
    ui->J59->setChecked(set);
    ui->J60->setChecked(set);

    ui->S01->setChecked(set);
    ui->S24->setChecked(set);
    ui->S25->setChecked(set);
    ui->S26->setChecked(set);

    ui->C02->setChecked(set);
    ui->C08->setChecked(set);
    ui->C09->setChecked(set);
    ui->C12->setChecked(set);
    ui->C13->setChecked(set);
    ui->C18->setChecked(set);
    ui->C26->setChecked(set);
    ui->C27->setChecked(set);
    ui->C28->setChecked(set);
    ui->C29->setChecked(set);
    ui->C33->setChecked(set);
    ui->C39->setChecked(set);
    ui->C40->setChecked(set);
    ui->C41->setChecked(set);
    ui->C42->setChecked(set);
    ui->C43->setChecked(set);
    ui->C56->setChecked(set);
    ui->C57->setChecked(set);
    ui->C58->setChecked(set);
    ui->C61->setChecked(set);
    ui->C62->setChecked(set);
    ui->C63->setChecked(set);
    ui->C64->setChecked(set);
    ui->C65->setChecked(set);

    ui->I26->setChecked(set);
    ui->I49->setChecked(set);
    ui->I50->setChecked(set);
    ui->I51->setChecked(set);
    ui->I52->setChecked(set);
    ui->I53->setChecked(set);
    ui->I54->setChecked(set);
    ui->I55->setChecked(set);

    ui->btnSetAll->setText(ui->btnSetAll->text() == tr("&Set All") ? tr("Un&set All") : tr("&Set All"));
}
//---------------------------------------------------------------------------
void CodeOptDialog::setNavSystem(int navSystem)
{
    this->navSystem = navSystem;

    updateEnable();
}
//---------------------------------------------------------------------------
void CodeOptDialog::setFrequencyType(int frequencyType)
{
    this->frequencyType = frequencyType;

    updateEnable();
}
//---------------------------------------------------------------------------
void CodeOptDialog::updateEnable()
{
    ui->G01->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G02->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G03->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G04->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G05->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G06->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G07->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G08->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G12->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L1));
    ui->G14->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G15->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G16->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G17->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G18->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G19->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G20->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G21->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G22->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G23->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L2));
    ui->G24->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L3));
    ui->G25->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L3));
    ui->G26->setEnabled((navSystem & SYS_GPS) && (frequencyType & FREQTYPE_L3));

    ui->R01->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L1));
    ui->R02->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L1));
    ui->R14->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L2));
    ui->R19->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L2));
    ui->R30->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L2));
    ui->R31->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L2));
    ui->R33->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L2));
    ui->R44->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L3));
    ui->R45->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L3));
    ui->R46->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L3));
    ui->R66->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L1));
    ui->R67->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L1));
    ui->R68->setEnabled((navSystem & SYS_GLO) && (frequencyType & FREQTYPE_L1));

    ui->E01->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L1));
    ui->E10->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L1));
    ui->E11->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L1));
    ui->E12->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L1));
    ui->E13->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L1));
    ui->E24->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L3));
    ui->E25->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L3));
    ui->E26->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L3));
    ui->E27->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L2));
    ui->E28->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L2));
    ui->E29->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L2));
    ui->E30->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L4));
    ui->E31->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L4));
    ui->E32->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L4));
    ui->E33->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L4));
    ui->E34->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L4));
    ui->E37->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L5));
    ui->E38->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L5));
    ui->E39->setEnabled((navSystem & SYS_GAL) && (frequencyType & FREQTYPE_L5));

    ui->J01->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L1));
    ui->J07->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L1));
    ui->J08->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L1));
    ui->J11->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L1));
    ui->J12->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L1));
    ui->J13->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L1));
    ui->J16->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L2));
    ui->J17->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L2));
    ui->J18->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L2));
    ui->J24->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L3));
    ui->J25->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L3));
    ui->J26->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L3));
    ui->J33->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L4));
    ui->J34->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L4));
    ui->J35->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L4));
    ui->J36->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L4));
    ui->J57->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L3));
    ui->J58->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L3));
    ui->J59->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L3));
    ui->J60->setEnabled((navSystem & SYS_QZS) && (frequencyType & FREQTYPE_L4));    

    ui->S01->setEnabled((navSystem & SYS_SBS) && (frequencyType & FREQTYPE_L1));
    ui->S24->setEnabled((navSystem & SYS_SBS) && (frequencyType & FREQTYPE_L3));
    ui->S25->setEnabled((navSystem & SYS_SBS) && (frequencyType & FREQTYPE_L3));
    ui->S26->setEnabled((navSystem & SYS_SBS) && (frequencyType & FREQTYPE_L3));

    ui->C02->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C08->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C09->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C12->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C13->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C18->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C26->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L3));
    ui->C27->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L2));
    ui->C28->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L2));
    ui->C29->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L2));
    ui->C33->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L4));
    ui->C39->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L5));
    ui->C40->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C41->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C42->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L4));
    ui->C43->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L4));
    ui->C56->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L1));
    ui->C57->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L3));
    ui->C58->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L3));
    ui->C61->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L2));
    ui->C62->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L2));
    ui->C63->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L2));
    ui->C64->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L5));
    ui->C65->setEnabled((navSystem & SYS_CMP) && (frequencyType & FREQTYPE_L5));

    ui->I26->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L1));
    ui->I49->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L1));
    ui->I50->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L1));
    ui->I51->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L1));
    ui->I52->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L2));
    ui->I53->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L2));
    ui->I54->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L2));
    ui->I55->setEnabled((navSystem & SYS_IRN) && (frequencyType & FREQTYPE_L2));

    if ((navSystem == (SYS_GPS | SYS_GAL | SYS_GLO | SYS_SBS | SYS_CMP | SYS_IRN | SYS_QZS)) &&
        (frequencyType == (FREQTYPE_L1 | FREQTYPE_L2 | FREQTYPE_L3 | FREQTYPE_L4 | FREQTYPE_L5)))
        ui->btnSetAll->setText(tr("Un&set All"));
}
//---------------------------------------------------------------------------
