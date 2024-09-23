//---------------------------------------------------------------------------
#ifndef postmainH
#define postmainH
//---------------------------------------------------------------------------
#include <QString>
#include <QDialog>
#include <QThread>

#include "rtklib.h"

namespace Ui {
class MainForm;
}

class QShowEvent;
class QCloseEvent;
class QSettings;
class OptDialog;
class TextViewer;
class ConvDialog;
class QComboBox;


//Helper Class ------------------------------------------------------------------

class ProcessingThread : public QThread
{
    Q_OBJECT

public:
    prcopt_t prcopt;
    solopt_t solopt;
    filopt_t filopt;
    gtime_t ts, te;
    double ti, tu;
    int n, stat;
    char *infile[6], outfile[1024];
    QString rov, base;

    explicit ProcessingThread(QObject *parent);
    ~ProcessingThread();

    bool addInput(const QString &);
    static QString toList(const QString & list);

protected:
    void run();

signals:
    void done(int);
};
//---------------------------------------------------------------------------

class MainForm : public QDialog
{
    Q_OBJECT

protected slots:
    void callRtkPlot();
    void viewOutputFile();
    void convertToKML();
    void showOptionsDialog();
    void startPostProcessing();
    void abortProcessing();
    void showAboutDialog();
	
    void showStartTimeDialog();
    void showStopTimeDialog();
    void selectInputFile1();
    void selectInputFile3();
    void selectInputFile2();
    void selectInputFile4();
    void selectInputFile5();
    void selectOutputFile();
    void viewInputFile1();
    void viewInputFile3();
    void viewInputFile2();
    void viewInputFile4();
    void viewInputFile5();
    void viewOutputFileStat();
    void viewOutputFileTrace();
    void plotInputFile1();
    void plotInputFile2();
    void showKeyDialog();
		
    void outputDirectoryEnableClicked();
    void selectOutputDirectory();
    void selectInputFile6();
    void viewInputFile6();

    void processingFinished(int);

public slots:
    void showMessage(const QString  &msg);
    void setProgress(int);

protected:
    void showEvent(QShowEvent*);
    void closeEvent(QCloseEvent*);
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private:
    OptDialog  *optDialog;
    ConvDialog *convDialog;
    TextViewer *textViewer;

    ProcessingThread *processingThread;

    void execProcessing();
    bool getOption(prcopt_t &prcopt, solopt_t &solopt, filopt_t &filopt);
    bool obsToNav (const QString &obsfile, QString &navfile);
	
    QString filePath(const QString &file);
    void readList(QComboBox *, QSettings *ini,  const QString &key);
    void writeList(QSettings *ini, const QString &key, const QComboBox *combo);
    void addHistory(QComboBox *combo);
    bool execCommand(const QString &cmd, const QStringList &opt);
	
    gtime_t getTimeStart();
    gtime_t getTimeStop();
    void setOutputFile();
    void setTimeStart(gtime_t time);
    void setTimeStop(gtime_t time);
    void updateEnable();
    void loadOptions();
    void saveOptions();

    Ui::MainForm *ui;

public:
    explicit MainForm(QWidget *parent = 0);

    QString iniFile;
    bool abortFlag;
	
    void viewFile(const QString &file);
};

//---------------------------------------------------------------------------
#endif
