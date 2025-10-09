//---------------------------------------------------------------------------
// ported to Qt by Jens Reimann
#include <QShowEvent>
#include <QScreen>
#include <QLabel>
#include <QFileDialog>

#include "refdlg.h"
#include "rtklib.h"
#include "ui_refdlg.h"

static const QChar degreeChar(0260);           // character code of degree (UTF-8)


//---------------------------------------------------------------------------
RefDialog::RefDialog(QWidget *parent, int options)
    : QDialog(parent), ui(new Ui::RefDialog)
{
    ui->setupUi(this);
    this->options = options;
    roverPosition[0] = roverPosition[1] = roverPosition[2] = 0.0;

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &RefDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &RefDialog::reject);
    connect(ui->tWStationList, &QTableWidget::cellDoubleClicked, this, &RefDialog::stationSelected);
    connect(ui->btnFind, &QPushButton::clicked, this, &RefDialog::findList);
    connect(ui->btnLoad, &QPushButton::clicked, this, &RefDialog::loadStations);
    connect(ui->lEFind, &QLineEdit::returnPressed, this, &RefDialog::findList);
}
//---------------------------------------------------------------------------
void RefDialog::showEvent(QShowEvent *event)
{
    int width[] = { 30, 80, 90, 65, 50, 80, 55 };

    if (event->spontaneous()) return;

    ui->btnLoad->setVisible(options);

    QStringList columns;
    columns << tr("No") << tr("Latitude (%1)").arg(degreeChar) << tr("Longitude (%1)").arg(degreeChar) << tr("Height (m)") << tr("Id") << tr("Name") << tr("Distance (km)");
    ui->tWStationList->setColumnCount(columns.size());
    ui->tWStationList->setRowCount(1);
    ui->tWStationList->setHorizontalHeaderLabels(columns);

    for (int i = 0; i < columns.size(); i++)
        for (int j = 0; j < 1; j++)
            ui->tWStationList->setItem(i, j, new QTableWidgetItem(""));

    fontScale = 2 * physicalDpiX();
    for (int i = 0; i < columns.size(); i++)
        ui->tWStationList->setColumnWidth(i, width[i] * fontScale / 96);

    loadList(stationPositionFile);

    ui->tWStationList->sortItems(6);  // sort by distance
}
//---------------------------------------------------------------------------
void RefDialog::stationSelected(int, int)
{
    if (validReference())
        accept();
    else
        reject();
}
//---------------------------------------------------------------------------
void RefDialog::accept()
{
    if (validReference())
        QDialog::accept();
    else
        QDialog::reject();
}
//---------------------------------------------------------------------------
void RefDialog::loadStations()
{
    stationPositionFile = QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Load Station List..."), stationPositionFile, tr("Position File (*.pos *.snx);;All (*.*)")));

    loadList(stationPositionFile);
}
//---------------------------------------------------------------------------
void RefDialog::findList()
{
    QString str = ui->lEFind->text();

    QList<QTableWidgetItem *> f = ui->tWStationList->findItems(str, Qt::MatchContains);

    if (f.empty()) return;

    ui->tWStationList->setCurrentItem(f.first());
}
//---------------------------------------------------------------------------
void RefDialog::loadList(QString filename)
{
    QByteArray buff;

    double pos[3];
    int n = 0;

    ui->tWStationList->setRowCount(0);

	// check format
    QFile fp(filename);
    if (!fp.open(QIODevice::ReadOnly)) return;

    buff = fp.readAll();
    if (buff.contains("%=SNX")) {
        fp.close();
        loadSinex(filename);
		return;
	}

    fp.seek(0);
    while (!fp.atEnd()) {
        buff = fp.readLine();
        buff = buff.mid(0, buff.indexOf('%')); /* remove comments */

        QList<QByteArray> tokens = buff.trimmed().split(' ');

        if (tokens.size() < 3) continue;

        ui->tWStationList->setRowCount(++n);

        for (int i = 0; i < 3; i++)
            pos[i] = tokens.at(i).toDouble();

        addReference(n, pos, tokens.size() > 3 ? tokens.at(3) : "",
                     tokens.size() > 4 ? tokens.at(4) : "");
    }
    if (n == 0) ui->tWStationList->setRowCount(0);

    updateDistances();
    setWindowTitle(filename);
}
//---------------------------------------------------------------------------
void RefDialog::loadSinex(QString filename)
{
    int n = 0, sol = 0;
    double rr[3], pos[3];
    bool okay;
    QFile file(filename);
    QByteArray buff, code;

    if (!file.open(QIODevice::ReadOnly)) return;

    while (!file.atEnd()) {
        buff = file.readLine();

        if (buff.contains("+SOLUTION/ESTIMATE")) sol = 1;
        if (buff.contains("-SOLUTION/ESTIMATE")) sol = 0;

        if (!sol || buff.size() < 68) continue;

        if (buff.mid(7, 4) == "STAX") {
            rr[0] = buff.mid(47, 21).toDouble(&okay);
            if (!okay) continue;
            code = buff.mid(14, 4);
        }
        if (buff.mid(7, 4) == "STAY") {
            rr[1] = buff.mid(47, 21).toDouble(&okay);
            if (!okay) continue;
            if (buff.mid(14, 4) != code) continue;
        }
        if (buff.mid(7, 4) == "STAZ") {
            rr[2] = buff.mid(47, 21).toDouble(&okay);
            if (!okay) continue;
            if (buff.mid(14, 4) != code) continue;
            ecef2pos(rr, pos);
            pos[0] *= R2D;
            pos[1] *= R2D;
            ui->tWStationList->setRowCount(++n);
            addReference(n, pos, code, "");
        }
    }
    ;
    if (n == 0)
        ui->tWStationList->setRowCount(0);

    updateDistances();
    setWindowTitle(stationPositionFile);
}
//---------------------------------------------------------------------------
void RefDialog::addReference(int n, double *pos, const QString code,
               const QString name)
{
    int row = ui->tWStationList->rowCount() - 1;

    ui->tWStationList->setItem(row, 0, new QTableWidgetItem(QString::number(n)));
    ui->tWStationList->setItem(row, 1, new QTableWidgetItem(QString::number(pos[0], 'f', 9)));
    ui->tWStationList->setItem(row, 2, new QTableWidgetItem(QString::number(pos[1], 'f', 9)));
    ui->tWStationList->setItem(row, 3, new QTableWidgetItem(QString::number(pos[2], 'f', 4)));
    ui->tWStationList->setItem(row, 4, new QTableWidgetItem(code));
    ui->tWStationList->setItem(row, 5, new QTableWidgetItem(name));
    ui->tWStationList->setItem(row, 6, new QTableWidgetItem(""));
}
//---------------------------------------------------------------------------
int RefDialog::validReference()
{
    QList<QTableWidgetItem *> sel = ui->tWStationList->selectedItems();
    if (sel.isEmpty()) return 0;
    else return 1;
}
//---------------------------------------------------------------------------
void RefDialog::updateDistances()
{
    double pos[3], ru[3], rr[3];
    bool ok;

    matcpy(pos, roverPosition, 3, 1);

    if (norm(pos, 3) <= 0.0) return;

    pos[0] *= D2R;
    pos[1] *= D2R;
    pos2ecef(pos, ru);

    for (int i = 0; i < ui->tWStationList->rowCount(); i++) {
        if (ui->tWStationList->item(i, 1)->text() == "") continue;

        pos[0] = ui->tWStationList->item(i, 1)->text().toDouble(&ok) * D2R;
        pos[1] = ui->tWStationList->item(i, 2)->text().toDouble(&ok) * D2R;
        pos[2] = ui->tWStationList->item(i, 3)->text().toDouble(&ok);
        pos2ecef(pos, rr);
        for (int j = 0; j < 3; j++) rr[j] -= ru[j];

        ui->tWStationList->item(i, 6)->setText(QString::number(norm(rr, 3) / 1E3, 'f', 1));
	}
}
//---------------------------------------------------------------------------
void RefDialog::setRoverPosition(double lat, double lon, double height)
{
    roverPosition[0] = lat;
    roverPosition[1] = lon;
    roverPosition[2] = height;

    updateDistances();
}
//---------------------------------------------------------------------------
double* RefDialog::getPosition()
{
    static double position[3] = {0, 0, 0};
    bool ok;

    QList<QTableWidgetItem *> sel = ui->tWStationList->selectedItems();
    if (sel.isEmpty()) return position;

    int row = ui->tWStationList->row(sel.first());
    position[0] = ui->tWStationList->item(row, 1)->text().toDouble(&ok); if (!ok) return 0;
    position[1] = ui->tWStationList->item(row, 2)->text().toDouble(&ok); if (!ok) return 0;
    position[2] = ui->tWStationList->item(row, 3)->text().toDouble(&ok); if (!ok) return 0;

    return position;
}

//---------------------------------------------------------------------------
QString RefDialog::getStationId()
{
    QList<QTableWidgetItem *> sel = ui->tWStationList->selectedItems();
    if (sel.isEmpty()) return "";

    int row = ui->tWStationList->row(sel.first());

    return ui->tWStationList->item(row, 4)->text();
}
//---------------------------------------------------------------------------
QString RefDialog::getStationName()
{
    QList<QTableWidgetItem *> sel = ui->tWStationList->selectedItems();
    if (sel.isEmpty()) return "";

    int row = ui->tWStationList->row(sel.first());

    return ui->tWStationList->item(row, 5)->text();
}
//---------------------------------------------------------------------------
void RefDialog::setLoadEnabled(bool enabled)
{
    ui->btnLoad->setEnabled(enabled);
}
//---------------------------------------------------------------------------
bool RefDialog::getLoadEnabled()
{
    return ui->btnLoad->isEnabled();
}
//---------------------------------------------------------------------------
