//---------------------------------------------------------------------------

#include <QShowEvent>
#include <QFileDialog>
#include <QFile>
#include <QDebug>

#include "rtklib.h"
#include "pntdlg.h"
#include "plotmain.h"

#include "ui_pntdlg.h"

//---------------------------------------------------------------------------
PntDialog::PntDialog(Plot *_plot, QWidget *parent)
    : QDialog(parent), plot(_plot), ui(new Ui::PntDialog)
{
    ui->setupUi(this);
    int width[] = {150, 150, 120, 150};
    int fontScale = QFontMetrics(ui->tWPntList->font()).height() * 4;
    QStringList labels;

    labels << tr("Latitude (°)") << tr("Longitude (°)") << tr("Height (m)") << tr("Name");

    ui->tWPntList->setColumnCount(4);
    ui->tWPntList->setHorizontalHeaderLabels(labels);
    for (int i = 0; i < 4; i++)
        ui->tWPntList->setColumnWidth(i, width[i] * fontScale / 96);

    noUpdate = false;

    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &PntDialog::reject);
    connect(ui->btnUpdate, &QPushButton::clicked, this, &PntDialog::updatePoints);
    connect(ui->btnAdd, &QPushButton::clicked, this, &PntDialog::addPoint);
    connect(ui->btnDelete, &QPushButton::clicked, this, &PntDialog::deletePoint);
    connect(ui->tWPntList, &QTableWidget::itemChanged, this, &PntDialog::updatePoints);
    connect(ui->tWPntList, &QTableWidget::itemSelectionChanged, this, &PntDialog::selectPoint);
    connect(ui->tWPntList, &QTableWidget::itemDoubleClicked, this, &PntDialog::centerPoint);
}
//---------------------------------------------------------------------------
void PntDialog::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    setPoints();
}
//---------------------------------------------------------------------------
void PntDialog::addPoint()
{
    double rr[3], pos[3] = {0};

    if (!plot->getCenterPosition(rr)) return;
    if (norm(rr, 3) <= 0.0) return;
    ecef2pos(rr, pos);

    noUpdate = true;
    ui->tWPntList->setRowCount(ui->tWPntList->rowCount() + 1);
    ui->tWPntList->setItem(ui->tWPntList->rowCount() - 1, 0, new QTableWidgetItem(QString("%1").arg(pos[0] * R2D, 0, 'f', 9)));
    ui->tWPntList->setItem(ui->tWPntList->rowCount() - 1, 1, new QTableWidgetItem(QString("%1").arg(pos[1] * R2D, 0, 'f', 9)));
    ui->tWPntList->setItem(ui->tWPntList->rowCount() - 1, 2, new QTableWidgetItem(QString("%1").arg(pos[2], 0, 'f', 4)));
    ui->tWPntList->setItem(ui->tWPntList->rowCount() - 1, 3, new QTableWidgetItem(tr("Point%1").arg(ui->tWPntList->rowCount(), 2, 10, QChar('0'))));
    noUpdate = false;

    updatePoints();
}
//---------------------------------------------------------------------------
void PntDialog::deletePoint()
{
    QList<QTableWidgetItem *> selections = ui->tWPntList->selectedItems();
    if (selections.isEmpty()) return;

    QTableWidgetItem *sel = selections.first();
    if (!sel) return;

    noUpdate = true;
    ui->tWPntList->removeRow(sel->row());
    noUpdate = false;

    updatePoints();
}

//---------------------------------------------------------------------------
void PntDialog::updatePoints()
{
    QList<WayPoint> wayPoints;
    WayPoint wayPnt;

    if (noUpdate) return;

    for (int i = 0; i < ui->tWPntList->rowCount(); i++) {
        if (!ui->tWPntList->item(i, 0)) continue;
        if (!ui->tWPntList->item(i, 1)) continue;
        if (!ui->tWPntList->item(i, 2)) continue;
        if (ui->tWPntList->item(i, 2)->text().isEmpty()) continue;
        wayPnt.position[0] = ui->tWPntList->item(i, 0)->text().toDouble();
        wayPnt.position[1] = ui->tWPntList->item(i, 1)->text().toDouble();
        wayPnt.position[2] = ui->tWPntList->item(i, 2)->text().toDouble();
        wayPnt.name = ui->tWPntList->item(i, 3)->text();
        wayPoints.append(wayPnt);
    }
    plot->setWayPoints(wayPoints);

    plot->updatePlot();
}
//---------------------------------------------------------------------------
void PntDialog::setPoints()
{
    noUpdate = true;
    ui->tWPntList->setRowCount(plot->getWayPoints().size());
    for (int i = 0; i < plot->getWayPoints().size(); i++) {
        ui->tWPntList->setItem(i, 0, new QTableWidgetItem(QString::number(plot->getWayPoints()[i].position[0], 'f', 9)));
        ui->tWPntList->setItem(i, 1, new QTableWidgetItem(QString::number(plot->getWayPoints()[i].position[1], 'f', 9)));
        ui->tWPntList->setItem(i, 2, new QTableWidgetItem(QString::number(plot->getWayPoints()[i].position[2], 'f', 4)));
        ui->tWPntList->setItem(i, 3, new QTableWidgetItem(plot->getWayPoints()[i].name));
    }
    noUpdate = false;
}

//---------------------------------------------------------------------------
void PntDialog::selectPoint()
{
    plot->setSelectedWayPoint(-1);
    QList<QTableWidgetItem *> selections = ui->tWPntList->selectedItems();
    if (selections.isEmpty()) return;

    QTableWidgetItem *item = selections.first();
    if (!item) return;

    int sel = ui->tWPntList->row(item);
    plot->setSelectedWayPoint(sel);

    plot->updatePlot();
}
//---------------------------------------------------------------------------
void PntDialog::centerPoint(QTableWidgetItem *w)
{
    int sel = ui->tWPntList->row(w);

    if ((sel >= plot->getWayPoints().size()) ||(sel < 0)) return;
    plot->setTrackCenter(plot->getWayPoints()[sel].position[0], plot->getWayPoints()[sel].position[1]);
}
//---------------------------------------------------------------------------
