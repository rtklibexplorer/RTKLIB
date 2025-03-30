//---------------------------------------------------------------------------
#include <QShowEvent>
#include <QFile>
#include <QDir>
#include <QAction>

#include "viewer.h"
#include "mapviewopt.h"

#include "ui_mapviewopt.h"


//---------------------------------------------------------------------------
MapViewOptDialog::MapViewOptDialog(QWidget* parent)
    : QDialog(parent), ui(new Ui::MapViewOpt)
{
    ui->setupUi(this);

    QAction *acNotes = new QAction("?", this);
    ui->lEApiKey->addAction(acNotes, QLineEdit::TrailingPosition);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &MapViewOptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &MapViewOptDialog::reject);
    connect(acNotes, &QAction::triggered, this, &MapViewOptDialog::showNotes);
}
//---------------------------------------------------------------------------
void MapViewOptDialog::showNotes()
{
    QString filename, dir;
    TextViewer *viewer;
    
    dir = qApp->applicationDirPath(); // exe directory
    filename = QDir(dir).absoluteFilePath("gmview_notes.txt");
    if (!QFile::exists(filename))
        filename = QDir(dir).absoluteFilePath("../appcmn_qt/gmview_notes.txt");

    viewer = new TextViewer(this);
    viewer->setWindowTitle(filename);
    viewer->setOption(0);
    viewer->exec();
    viewer->read(filename);
}
//---------------------------------------------------------------------------
void MapViewOptDialog::setApiKey(const QString & apiKey)
{
    ui->lEApiKey->setText(apiKey);
}
//---------------------------------------------------------------------------
QString MapViewOptDialog::getApiKey()
{
    return ui->lEApiKey->text();
}
//---------------------------------------------------------------------------
void MapViewOptDialog::setMapStrings(int i, const QString &title, const QString &url, const QString &attr)
{
    QLineEdit *titles[] = {
        ui->lEMapTitle1, ui->lEMapTitle2, ui->lEMapTitle3, ui->lEMapTitle4, ui->lEMapTitle5, ui->lEMapTitle6
    };
    QLineEdit *urls[] = {
        ui->lEMapTile1, ui->lEMapTile2, ui->lEMapTile3, ui->lEMapTile4, ui->lEMapTile5, ui->lEMapTile6
    };
    if ((i < 0) || (i > 5))
        return;

    titles[i]->setText(title);
    urls[i]->setText(url);

    attrs[i] = attr;
}
//---------------------------------------------------------------------------
void MapViewOptDialog::getMapStrings(int i, QString &title, QString &url, QString &attr)
{
    QLineEdit *titles[] = {
        ui->lEMapTitle1, ui->lEMapTitle2, ui->lEMapTitle3, ui->lEMapTitle4, ui->lEMapTitle5, ui->lEMapTitle6
    };
    QLineEdit *urls[] = {
        ui->lEMapTile1, ui->lEMapTile2, ui->lEMapTile3, ui->lEMapTile4, ui->lEMapTile5, ui->lEMapTile6
    };
    if ((i < 0) || (i > 5))
        return;

    title = titles[i]->text();
    url = urls[i]->text();
    attr = attrs[i];
}
//---------------------------------------------------------------------------


