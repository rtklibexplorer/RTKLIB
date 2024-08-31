//---------------------------------------------------------------------------
#include <QFileDialog>
#include <QLineEdit>
#include <QPoint>
#include <QFontDialog>
#include <QFont>
#include <QShowEvent>
#include <QFileSystemModel>
#include <QCompleter>
#include <QMessageBox>
#include <QStandardItemModel>
#include <QAction>

#include "rtklib.h"
#include "navi_post_opt.h"
#include "viewer.h"
#include "refdlg.h"
#include "maskoptdlg.h"
#include "freqdlg.h"
#include "keydlg.h"
#include "helper.h"
#include "doubleunitvalidator.h"

#include "ui_navi_post_opt.h"

//---------------------------------------------------------------------------
#define MAXSTR      1024                /* max length of a string */
#define PANELFONTNAME "Tahoma"
#define PANELFONTSIZE 8
#define POSFONTNAME "Palatino Linotype"
#define POSFONTSIZE 10
#define DEFAULTPORT 52001               // default monitor port number

#define MSGOPT  "0:all,1:rover,2:base,3:corr"

//---------------------------------------------------------------------------
OptDialog::OptDialog(QWidget *parent, int opts)
    : QDialog(parent), options(opts), ui(new Ui::OptDialog)
{
    QString label;
    static int freq[] = {1, 2, 5, 6, 7, 8, 9};

    ui->setupUi(this);

    // add tooltips to items of combo boxes
    ui->cBPositionMode->setItemData(PMODE_SINGLE, tr("Single point positioning or SBAS DGPS"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_DGPS, tr("Code-based differential GPS"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_KINEMA, tr("Carrier-based Kinematic positioning"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_STATIC, tr("Carrier-based Static positioning"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_STATIC_START, tr("Static till first fix, then Kinematic"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_MOVEB, tr("Moving baseline"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_FIXED, tr("Rover receiver position fixed (for residual analysis)"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_PPP_KINEMA, tr("Precise Point Positioning with kinematic mode"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_PPP_STATIC, tr("Precise Point Positioning with static mode"), Qt::ToolTipRole);
    ui->cBPositionMode->setItemData(PMODE_PPP_FIXED, tr("Rover receiver position is fixed with PPP mode (for residual analysis)"), Qt::ToolTipRole);

    ui->cBSolution->setItemData(SOLTYPE_FORWARD, tr("Forward filter solution"), Qt::ToolTipRole);
    ui->cBSolution->setItemData(SOLTYPE_BACKWARD, tr("Backward filter solution"), Qt::ToolTipRole);
    ui->cBSolution->setItemData(SOLTYPE_COMBINED, tr("Smoother combined solution with forward and backward filter solutions, \n"
                                                     "phase bias states reset between forward and backward solutions"), Qt::ToolTipRole);
    ui->cBSolution->setItemData(SOLTYPE_COMBINED_NORESET, tr("Smoother combined solution with forward and backward filter solutions, \n"
                                                             "phase bias states not reset between forward and backward solutions."), Qt::ToolTipRole);

    ui->cBDynamicModel->setItemData(0, tr("Dynamics is not used"), Qt::ToolTipRole);
    ui->cBDynamicModel->setItemData(1, tr("Receiver velocity and acceleration are estimated"), Qt::ToolTipRole);

    ui->cBTideCorrection->setItemData(0, tr("Not apply earth tides correction"), Qt::ToolTipRole);
    ui->cBTideCorrection->setItemData(1, tr("Apply solid earth tides correction"), Qt::ToolTipRole);
    ui->cBTideCorrection->setItemData(2, tr("Apply solid earth tides, OTL (ocean tide loading) and pole tide corrections"), Qt::ToolTipRole);

    ui->cBIonosphereOption->setItemData(IONOOPT_OFF, tr("Not apply ionospheric correction"), Qt::ToolTipRole);
    ui->cBIonosphereOption->setItemData(IONOOPT_BRDC, tr("Apply broadcast ionospheric model"), Qt::ToolTipRole);
    ui->cBIonosphereOption->setItemData(IONOOPT_SBAS, tr("Apply SBAS ionospheric model"), Qt::ToolTipRole);
    ui->cBIonosphereOption->setItemData(IONOOPT_IFLC, tr("Ionosphere-free linear combination with dual frequency (L1-L2 for GPS/ GLONASS/QZSS \n"
                                                         "or L1-L5 for Galileo) measurements is used for ionospheric correction"), Qt::ToolTipRole);
    ui->cBIonosphereOption->setItemData(IONOOPT_EST, tr("Estimate ionospheric parameter STEC (slant total electron content)"), Qt::ToolTipRole);
    ui->cBIonosphereOption->setItemData(IONOOPT_TEC, tr("Use IONEX TEC grid data"), Qt::ToolTipRole);
    ui->cBIonosphereOption->setItemData(IONOOPT_QZS, tr("Apply broadcast ionosphere model provided by QZSS"), Qt::ToolTipRole);

    ui->cBTroposphereOption->setItemData(TROPOPT_OFF, tr("Not apply troposphere correction"), Qt::ToolTipRole);
    ui->cBTroposphereOption->setItemData(TROPOPT_SAAS, tr("Apply Saastamoinen model"), Qt::ToolTipRole);
    ui->cBTroposphereOption->setItemData(TROPOPT_SBAS, tr("Apply SBAS tropospheric model (MOPS)"), Qt::ToolTipRole);
    ui->cBTroposphereOption->setItemData(TROPOPT_EST, tr("Estimate ZTD (zenith total delay) parameters as EKF states"), Qt::ToolTipRole);
    ui->cBTroposphereOption->setItemData(TROPOPT_ESTG, tr("ZTD and horizontal gradient parameters as EKF states"), Qt::ToolTipRole);
    ui->cBTroposphereOption->setItemData(TROPOPT_ESTG, tr("ZTD and horizontal gradient parameters as EKF states"), Qt::ToolTipRole);

    ui->cBSatelliteEphemeris->setItemData(EPHOPT_BRDC, tr("Use broadcast ephemeris"), Qt::ToolTipRole);
    ui->cBSatelliteEphemeris->setItemData(EPHOPT_PREC, tr("Use precise ephemeris"), Qt::ToolTipRole);
    ui->cBSatelliteEphemeris->setItemData(EPHOPT_SBAS, tr("Broadcast ephemeris with SBAS long-term and fast correction"), Qt::ToolTipRole);
    ui->cBSatelliteEphemeris->setItemData(EPHOPT_SSRAPC, tr("Broadcast ephemeris with RTCM SSR correction (antenna phase center value)"), Qt::ToolTipRole);
    ui->cBSatelliteEphemeris->setItemData(EPHOPT_SSRCOM, tr("Broadcast ephemeris with RTCM SSR correction (satellite center of mass value)"), Qt::ToolTipRole);

    ui->cBAmbiguityResolutionGPS->setItemData(0, tr("No ambiguity resolution"), Qt::ToolTipRole);
    ui->cBAmbiguityResolutionGPS->setItemData(1, tr("Continuously static integer ambiguities are estimated and resolved"), Qt::ToolTipRole);
    ui->cBAmbiguityResolutionGPS->setItemData(2, tr("Integer ambiguity is estimated and resolved by epoch-by-epoch basis"), Qt::ToolTipRole);
    ui->cBAmbiguityResolutionGPS->setItemData(3, tr("Continuously static integer ambiguities are estimated and resolved.\n"
                                                    "If the validation OK, the ambiguities are constrained to the resolved values."), Qt::ToolTipRole);

    ui->cBAmbiguityResolutionGLO->setItemData(0, tr("No ambiguity resolution"), Qt::ToolTipRole);
    ui->cBAmbiguityResolutionGLO->setItemData(1, tr("Ambiguities are fixed. Usually the ambiguity of only the same types receiver pair for the rover and\n"
                                                    "the base station can be fixed. If he different receiver types have IFB (inter-frequency bias),\n"
                                                    "they cannot be canceled by DD."), Qt::ToolTipRole);
    ui->cBAmbiguityResolutionGLO->setItemData(2, tr("Nulls out inter-frequency biases after first fix-and-hold of GPS satellites"), Qt::ToolTipRole);
    ui->cBAmbiguityResolutionGLO->setItemData(3, tr("Receiver inter-frequency biases auto-calibrate but require\n"
                                                    "reasonably accurate initial estimates (see GLO HW Bias)"), Qt::ToolTipRole);

    ui->cBSolutionFormat->setItemData(0, tr("Latitude, longitude and height"), Qt::ToolTipRole);
    ui->cBSolutionFormat->setItemData(1, tr("X/Y/Z components of ECEF coordinates"), Qt::ToolTipRole);
    ui->cBSolutionFormat->setItemData(2, tr("E/N/U components of baseline vector"), Qt::ToolTipRole);
    ui->cBSolutionFormat->setItemData(3, tr("NMEA GPRMC, GPGGA, GPGSA, GLGSA, GAGSA, GPGSV, GLGSV and GAGSV"), Qt::ToolTipRole);

    ui->cBOutputGeoid->setItemData(0, tr("Internal geoid model"), Qt::ToolTipRole);
    ui->cBOutputGeoid->setItemData(1, tr("EGM96 (15\" x 15\" grid)"), Qt::ToolTipRole);
    ui->cBOutputGeoid->setItemData(2, tr("EGM2008 (2.5\" x 2.5\" grid)"), Qt::ToolTipRole);
    ui->cBOutputGeoid->setItemData(3, tr("EGM2008 (1\" x 1\" grid)"), Qt::ToolTipRole);
    ui->cBOutputGeoid->setItemData(4, tr("GSI2000 (1\"x1.5\" grid)"), Qt::ToolTipRole);

    ui->cBReferencePositionType->setItemData(0, tr("Latitude/longitude/height in degree and m"), Qt::ToolTipRole);
    ui->cBReferencePositionType->setItemData(1, tr("Latitude/longitude/height in degree/minute/second and m"), Qt::ToolTipRole);
    ui->cBReferencePositionType->setItemData(2, tr("X/Y/Z components in ECEF frame."), Qt::ToolTipRole);
    ui->cBReferencePositionType->setItemData(3, tr("Use the average of single point solutions"), Qt::ToolTipRole);
    ui->cBReferencePositionType->setItemData(4, tr("Use the position in the position file. The station is searched by using the\n"
                                                   "head 4-character ID of the rover observation data file path"), Qt::ToolTipRole);
    ui->cBReferencePositionType->setItemData(5, tr("Use the approximate position in RINEX OBS header"), Qt::ToolTipRole);
    ui->cBReferencePositionType->setItemData(6, tr("Use the antenna position included in RTCM messages"), Qt::ToolTipRole);

    // inspired by https://stackoverflow.com/questions/18321779/degrees-minutes-and-seconds-regex
    // and https://stackoverflow.com/questions/3518504/regular-expression-for-matching-latitude-longitude-coordinates
    QString posSign = QRegularExpression::escape(QLocale().positiveSign());
    QString negSign = QRegularExpression::escape(QLocale().negativeSign());
    QString decSep = QRegularExpression::escape(QLocale().decimalPoint());

    regExDMSLat = QRegularExpression(QString("^\\s*(?:(?<deg1>[%0%1]?90)[°\\s]\\s*(?<min1>0{1,2})['\\s]\\s*(?<sec1>0{1,2}(?:[%2]0*)?)\"?\\s*)|"
                                             "(?:(?<deg2>[%0%1]?(?:[1-8][0-9]|[0-9]))[°\\s]\\s*(?<min2>(?:[0-5][0-9]|[0-9]))['\\s]\\s*(?<sec2>(?:[0-5][0-9]|[0-9])(?:[%2][0-9]*)?)\"?)\\s*$").arg(posSign, negSign, decSep));
    regExDMSLon = QRegularExpression(QString("^\\s*(?:(?<deg1>[%0%1]?180)[°\\s]\\s*(?<min1>0{1,2})['\\s]\\s*(?<sec1>0{1,2}(?:[%2]0*)?)\"?\\s*)|"
                                             "(?:(?<deg2>[%0%1]?(?:1[0-7][0-9]|[0-9][0-9]|[0-9]))[°\\s]\\s*(?<min2>(?:[0-5][0-9]|[0-9]))['\\s]\\s*(?<sec2>(?:[0-5][0-9]|[0-9])(?:[%2][0-9]*)?)\"?)\\s*$").arg(posSign, negSign, decSep));


    processingOptions = prcopt_default;
    solutionOptions = solopt_default;
    fileOptions.blq[0] = '\0';
    fileOptions.dcb[0] = '\0';
    fileOptions.eop[0] = '\0';
    fileOptions.geexe[0] = '\0';
    fileOptions.geoid[0] = '\0';
    fileOptions.iono[0] = '\0';
    fileOptions.rcvantp[0] = '\0';
    fileOptions.satantp[0] = '\0';
    fileOptions.solstat[0] = '\0';
    fileOptions.stapos[0] = '\0';
    fileOptions.tempdir[0] = '\0';
    fileOptions.trace[0] = '\0';

    appOptions = nullptr;

    serverCycle = serverBufferSize = 0;
    solutionBufferSize = 1000;
    panelStacking = 0;

    serverCycle = 10;
    timeoutTime = 10000;             /* timeout time (ms) */
    reconnectTime = 10000;           /* reconnect interval (ms) */
    nmeaCycle = 5000;                /* nmea request cycle (ms) */
    fileSwapMargin = 30;             /* file swap marign (s) */
    serverBufferSize = 32768;        /* input buffer size (bytes) */
    navSelect = 0;                   /* navigation mesaage select */
    proxyaddr[0] = '\0';             /* proxy address */

    static opt_t _naviopt[] = {
        { "misc-svrcycle",    0, (void *)&serverCycle,      "ms"    },
        { "misc-timeout",     0, (void *)&timeoutTime,      "ms"    },
        { "misc-reconnect",   0, (void *)&reconnectTime,    "ms"    },
        { "misc-nmeacycle",   0, (void *)&nmeaCycle,        "ms"    },
        { "misc-buffsize",    0, (void *)&serverBufferSize, "bytes" },
        { "misc-navmsgsel",   3, (void *)&navSelect,        MSGOPT  },
        { "misc-proxyaddr",   2, (void *)proxyaddr,         ""      },
        { "misc-fswapmargin", 0, (void *)&fileSwapMargin,   "s"     },
        { "",		          0, NULL,		                ""      }
    };
    this->naviopts = _naviopt;

    textViewer = new TextViewer(this);
    freqDialog = new FreqDialog(this);
    
    QStringList freq_tooltips = {tr("Single Frequency"),
                                 tr("L1 and L2 Dual-Frequency"),
                                 tr("L1, L2 and L5 Triple-Frequency"),
                                 tr("Experimental, not fully supported")};

    for (int i = 0; i < NFREQ; i++) {
        label = label + (i > 0 ? "+" : "") + QString("L%1").arg(freq[i]);
        ui->cBFrequencies->addItem(label, i);
        ui->cBFrequencies->setItemData(i, freq_tooltips.at(i) , Qt::ToolTipRole);
	}

    // set up completers
    QCompleter *fileCompleter = new QCompleter(this);
    QFileSystemModel *fileModel = new QFileSystemModel(fileCompleter);
    fileModel->setRootPath("");
    fileCompleter->setModel(fileModel);
    ui->lEStationPositionFile->setCompleter(fileCompleter);
    ui->lEAntennaPcvFile->setCompleter(fileCompleter);
    ui->lESatellitePcvFile->setCompleter(fileCompleter);
    ui->lEDCBFile->setCompleter(fileCompleter);
    ui->lEGeoidDataFile->setCompleter(fileCompleter);
    ui->lEEOPFile->setCompleter(fileCompleter);
    ui->lEBLQFile->setCompleter(fileCompleter);

    QCompleter *dirCompleter = new QCompleter(this);
    QFileSystemModel *dirModel = new QFileSystemModel(dirCompleter);
    dirModel->setRootPath("");
    dirModel->setFilter(QDir::AllDirs | QDir::Drives | QDir::NoDotAndDotDot);
    dirCompleter->setModel(dirModel);
    ui->lELocalDirectory->setCompleter(dirCompleter);

    // station position file line edit actions
    QAction *acStationPositionFileSelect = ui->lEStationPositionFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acStationPositionFileSelect->setToolTip(tr("Select File"));
    QAction *acStationPositionFileView = ui->lEStationPositionFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acStationPositionFileView->setToolTip(tr("View File"));
    acStationPositionFileView->setEnabled(false);

    // satllite PCV line edit actions
    QAction *acSatellitePcvFileSelect = ui->lESatellitePcvFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acSatellitePcvFileSelect->setToolTip(tr("Select File"));
    QAction *acSatellitePcvFileView = ui->lESatellitePcvFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acSatellitePcvFileView->setToolTip(tr("View File"));
    acSatellitePcvFileView->setEnabled(false);

    // antenna PCV line edit actions
    QAction *acAntennaPcvFileSelect = ui->lEAntennaPcvFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acAntennaPcvFileSelect->setToolTip(tr("Select File"));
    QAction *acAntennaPcvFileView = ui->lEAntennaPcvFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acAntennaPcvFileView->setToolTip(tr("View File"));
    acAntennaPcvFileView->setEnabled(false);

    // DCB file line edit actions
    QAction *acDCBFileSelect = ui->lEDCBFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acDCBFileSelect->setToolTip(tr("Select File"));
    QAction *acDCBFileView = ui->lEDCBFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acDCBFileView->setToolTip(tr("View File"));
    acDCBFileView->setEnabled(false);

    // BLQ file line edit actions
    QAction *acBLQFileSelect = ui->lEBLQFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acBLQFileSelect->setToolTip(tr("Select File"));
    QAction *acBLQFileView = ui->lEBLQFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acBLQFileView->setToolTip(tr("View File"));
    acBLQFileView->setEnabled(false);

    // EOP file line edit actions
    QAction *acEOPFileSelect = ui->lEEOPFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acEOPFileSelect->setToolTip(tr("Select File"));
    QAction *acEOPFileView = ui->lEEOPFile->addAction(QIcon(":/buttons/doc"), QLineEdit::TrailingPosition);
    acEOPFileView->setToolTip(tr("View File"));
    acEOPFileView->setEnabled(false);

    // geoid data file line edit actions
    QAction *acGeoidDataFileSelect = ui->lEGeoidDataFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acGeoidDataFileSelect->setToolTip(tr("Select File"));

    // BLQ file line edit actions
    QAction *acLocalDirectorySelect = ui->lELocalDirectory->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acLocalDirectorySelect->setToolTip(tr("Select Directory"));

    // Ionosphere file line edit actions
    QAction *acIonosphereFileSelect = ui->lEIonosphereFile->addAction(QIcon(":/buttons/folder"), QLineEdit::TrailingPosition);
    acIonosphereFileSelect->setToolTip(tr("Select File"));

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &OptDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &OptDialog::reject);
    connect(ui->btnLoad, &QPushButton::clicked, this, &OptDialog::loadSettings);
    connect(ui->btnSave, &QPushButton::clicked, this, &OptDialog::saveSettings);
    connect(acAntennaPcvFileSelect, &QAction::triggered, this, &OptDialog::selectAntennaPcvFile);
    connect(acAntennaPcvFileView, &QAction::triggered, this, &OptDialog::viewAntennaPcvFile);
    connect(ui->lEAntennaPcvFile, &QLineEdit::textChanged, this, [acAntennaPcvFileView, this]()
            {acAntennaPcvFileView->setEnabled(QFile::exists(ui->lEAntennaPcvFile->text()));});
    connect(acSatellitePcvFileSelect, &QAction::triggered, this, &OptDialog::selectSatellitePcvFile);
    connect(acSatellitePcvFileView, &QAction::triggered, this, &OptDialog::viewSatellitePcvFile);
    connect(ui->lESatellitePcvFile, &QLineEdit::textChanged, this, [acSatellitePcvFileView, this]()
            {acSatellitePcvFileView->setEnabled(QFile::exists(ui->lESatellitePcvFile->text()));});
    connect(acDCBFileSelect, &QAction::triggered, this, &OptDialog::selectDCBFile);
    connect(acDCBFileView, &QAction::triggered, this, &OptDialog::viewDCBFile);
    connect(ui->lEDCBFile, &QLineEdit::textChanged, this, [acDCBFileView, this]()
            {acDCBFileView->setEnabled(QFile::exists(ui->lEDCBFile->text()));});
    connect(acEOPFileSelect, &QAction::triggered, this, &OptDialog::selectEOPFile);
    connect(acEOPFileView, &QAction::triggered, this, &OptDialog::viewEOPFile);
    connect(ui->lEEOPFile, &QLineEdit::textChanged, this, [acEOPFileView, this]()
            {acEOPFileView->setEnabled(QFile::exists(ui->lEEOPFile->text()));});
    connect(acGeoidDataFileSelect, &QAction::triggered, this, &OptDialog::selectGeoidDataFile);
    connect(acLocalDirectorySelect, &QAction::triggered, this, &OptDialog::selectLocalDirectory);
    connect(acIonosphereFileSelect, &QAction::triggered, this, &OptDialog::selectIonosphereFile);
    connect(acBLQFileSelect, &QAction::triggered, this, &OptDialog::selectBLQFile);
    connect(acBLQFileView, &QAction::triggered, this, &OptDialog::viewBLQFile);
    connect(ui->lEBLQFile, &QLineEdit::textChanged, this, [acBLQFileView, this]()
            {acBLQFileView->setEnabled(QFile::exists(ui->lEBLQFile->text()));});
    connect(acStationPositionFileSelect, &QAction::triggered, this, &OptDialog::selectStationPositionFile);
    connect(acStationPositionFileView, &QAction::triggered, this, &OptDialog::viewStationPositionFile);
    connect(ui->lEStationPositionFile, &QLineEdit::textChanged, this, [acStationPositionFileView, this]()
            {acStationPositionFileView->setEnabled(QFile::exists(ui->lEStationPositionFile->text()));});
    connect(ui->btnReferencePosition, &QPushButton::clicked, this, &OptDialog::selectReferencePosition);
    connect(ui->btnRoverPosition, &QPushButton::clicked, this, &OptDialog::selectRoverPosition);
    connect(ui->btnSnrMask, &QPushButton::clicked, this, &OptDialog::showSnrMaskDialog);
    connect(ui->btnFontPanel, &QPushButton::clicked, this, &OptDialog::selectPanelFont);
    connect(ui->btnFontSolution, &QPushButton::clicked, this, &OptDialog::selectSolutionFont);
    connect(ui->cBPositionMode, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBSolutionFormat, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBReferencePositionType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::referencePositionTypeChanged);
    connect(ui->cBRoverPositionType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::roverPositionTypeChanged);
    connect(ui->lEReferencePosition1, &QLineEdit::textChanged, this, &OptDialog::checkLineEditValidator);
    connect(ui->lEReferencePosition2, &QLineEdit::textChanged, this, &OptDialog::checkLineEditValidator);
    connect(ui->lEReferencePosition3, &QLineEdit::textChanged, this, &OptDialog::checkLineEditValidator);
    connect(ui->lERoverPosition1, &QLineEdit::textChanged, this, &OptDialog::checkLineEditValidator);
    connect(ui->lERoverPosition2, &QLineEdit::textChanged, this, &OptDialog::checkLineEditValidator);
    connect(ui->lERoverPosition3, &QLineEdit::textChanged, this, &OptDialog::checkLineEditValidator);
    connect(ui->cBAmbiguityResolutionGPS, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBRoverAntennaPcv, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBReferenceAntennaPcv, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBOutputHeight, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBNavSys1, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBNavSys2, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBNavSys3, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBNavSys4, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBNavSys5, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBNavSys6, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->cBBaselineConstrain, &QCheckBox::clicked, this, &OptDialog::updateEnable);
    connect(ui->btnFrequencies, &QPushButton::clicked, this, &OptDialog::showFrequenciesDialog);
    connect(ui->cBReferenceAntenna, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBRoverAntenna, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBIonosphereOption, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBTroposphereOption, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBDynamicModel, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->cBSatelliteEphemeris, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);
    connect(ui->btnHelp, &QPushButton::clicked, this, &OptDialog::showKeyDialog);
    connect(ui->cBFrequencies, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &OptDialog::updateEnable);

    QStandardItemModel *refPosModel =
        qobject_cast<QStandardItemModel *>(ui->cBReferencePositionType->model());
    QStandardItemModel *rovPosModel =
        qobject_cast<QStandardItemModel *>(ui->cBRoverPositionType->model());

    if (options == PostOptions) {
        refPosModel->item(6)->setFlags(refPosModel->item(6)->flags() & ~Qt::ItemIsEnabled); // disable "RTCM/Raw Antenna Position"
    } else if (options == NaviOptions) {
        refPosModel->item(4)->setFlags(refPosModel->item(4)->flags() & ~Qt::ItemIsEnabled); // disable "Get from Position File"
        refPosModel->item(5)->setFlags(refPosModel->item(5)->flags() & ~Qt::ItemIsEnabled); // disable "RINEX Header Position"

        rovPosModel->item(3)->setFlags(rovPosModel->item(3)->flags() & ~Qt::ItemIsEnabled); // disable "Average of Single Pos"
        rovPosModel->item(4)->setFlags(rovPosModel->item(4)->flags() & ~Qt::ItemIsEnabled); // disable "Get from Position File"
        rovPosModel->item(5)->setFlags(rovPosModel->item(5)->flags() & ~Qt::ItemIsEnabled); // disable "RINEX Header Position"
    }

    if (MAXPRNGPS <= 0) ui->cBNavSys1->setEnabled(false);
    if (MAXPRNGLO <= 0) ui->cBNavSys2->setEnabled(false);
    if (MAXPRNGAL <= 0) ui->cBNavSys3->setEnabled(false);
    if (MAXPRNQZS <= 0) ui->cBNavSys4->setEnabled(false);
    if (MAXPRNCMP <= 0) ui->cBNavSys6->setEnabled(false);
    if (MAXPRNIRN <= 0) ui->cBNavSys7->setEnabled(false);
}
//---------------------------------------------------------------------------
void OptDialog::showEvent(QShowEvent *event)
{
    if (event->spontaneous()) return;

    ui->tWOptions->setCurrentIndex(0);

    updateEnable();
}
//---------------------------------------------------------------------------
void OptDialog::loadSettings()
{
    QString fileName;

    fileName = QFileDialog::getOpenFileName(this, tr("Load Options..."), QString(), tr("Options File (*.conf);;All (*.*)"));

    if (!fileName.isEmpty())
        load(QDir::toNativeSeparators(fileName));
}
//---------------------------------------------------------------------------
void OptDialog::saveSettings()
{
    QString file;

    file = QFileDialog::getSaveFileName(this, tr("Save Options..."), QString(), tr("Options File (*.conf);;All (*.*)"));

    if (file.isEmpty()) return;

    QFileInfo f(QDir::toNativeSeparators(file));
    if (f.suffix().isEmpty()) file = file + ".conf";

    save(file);
}
//---------------------------------------------------------------------------
void OptDialog::viewStationPositionFile()
{
    if (ui->lEStationPositionFile->text().isEmpty()) return;

    textViewer->read(ui->lEStationPositionFile->text());

    textViewer->show();
}
//---------------------------------------------------------------------------
void OptDialog::selectStationPositionFile()
{
    QString fileName;

    fileName = QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Station Position File"), ui->lEStationPositionFile->text(), tr("Position File (*.pos);;All (*.*)")));

    if (!fileName.isEmpty())
        ui->lEStationPositionFile->setText(fileName);
}
//---------------------------------------------------------------------------
void OptDialog::showSnrMaskDialog()
{
    MaskOptDialog maskOptDialog(this);
    
    maskOptDialog.setSnrMask(snrmask);

    maskOptDialog.exec();
    if (maskOptDialog.result() != QDialog::Accepted) return;

    snrmask = maskOptDialog.getSnrMask();
}
//---------------------------------------------------------------------------
void OptDialog::roverPositionTypeChanged(int)
{
    QLineEdit *edit[] = {ui->lERoverPosition1, ui->lERoverPosition2, ui->lERoverPosition3};
	double pos[3];

    // update position value in accordance to type
    int error = getPosition(current_roverPositionType, edit, pos);
    if (error) {
        ui->cBRoverPositionType->setCurrentIndex(current_roverPositionType); // don't allow to change the type
    } else {
        setPosition(ui->cBRoverPositionType->currentIndex(), edit, pos);
        current_roverPositionType = ui->cBRoverPositionType->currentIndex();
    }
	updateEnable();
}
//---------------------------------------------------------------------------
void OptDialog::referencePositionTypeChanged(int)
{
    QLineEdit *edit[] = {ui->lEReferencePosition1, ui->lEReferencePosition2, ui->lEReferencePosition3};
	double pos[3];

    // update position value in accordance to type
    int error = getPosition(current_referencePositionType, edit, pos);
    if (error) {
        ui->cBReferencePositionType->setCurrentIndex(current_referencePositionType); // don't allow to change the type
    } else {
        setPosition(ui->cBReferencePositionType->currentIndex(), edit, pos);
        current_referencePositionType = ui->cBReferencePositionType->currentIndex();
    }

	updateEnable();
}
//---------------------------------------------------------------------------
void OptDialog::selectRoverPosition()
{
    RefDialog refDialog(this);
    QLineEdit *edit[] = {ui->lERoverPosition1, ui->lERoverPosition2, ui->lERoverPosition3};
    double p[3], posi[3];

    getPosition(ui->cBRoverPositionType->currentIndex(), edit, p);
    ecef2pos(p, posi);
    
    refDialog.setRoverPosition(posi[0] * R2D, posi[1] * R2D, posi[2]);
    refDialog.stationPositionFile = ui->lEStationPositionFile->text();
    refDialog.move(pos().x() + size().width() / 2 - refDialog.size().width() / 2,
               pos().y() + size().height() / 2 - refDialog.size().height() / 2);

    refDialog.exec();
    if (refDialog.result() != QDialog::Accepted) return;

    posi[0] = refDialog.getPosition()[0] * D2R;
    posi[1] = refDialog.getPosition()[1] * D2R;
    posi[2] = refDialog.getPosition()[2];

    pos2ecef(posi, p);
    setPosition(ui->cBRoverPositionType->currentIndex(), edit, p);
}
//---------------------------------------------------------------------------
void OptDialog::selectReferencePosition()
{
    RefDialog refDialog(this);
    QLineEdit *edit[] = {ui->lEReferencePosition1, ui->lEReferencePosition2, ui->lEReferencePosition3};
    double p[3], posi[3];

    getPosition(ui->cBReferencePositionType->currentIndex(), edit, p);
    ecef2pos(p, posi);
    refDialog.setRoverPosition(posi[0] * R2D, posi[1] * R2D, posi[2]);
    refDialog.stationPositionFile = ui->lEStationPositionFile->text();
    refDialog.move(pos().x() + size().width() / 2 - refDialog.size().width() / 2,
               pos().y() + size().height() / 2 - refDialog.size().height() / 2);

    refDialog.exec();
    if (refDialog.result() != QDialog::Accepted) return;

    posi[0] = refDialog.getPosition()[0] * D2R;
    posi[1] = refDialog.getPosition()[1] * D2R;
    posi[2] = refDialog.getPosition()[2];

    pos2ecef(posi, p);
    setPosition(ui->cBReferencePositionType->currentIndex(), edit, p);
}
//---------------------------------------------------------------------------
void OptDialog::viewSatellitePcvFile()
{
    if (ui->lESatellitePcvFile->text().isEmpty()) return;

    textViewer->read(ui->lESatellitePcvFile->text());

    textViewer->show();
}
//---------------------------------------------------------------------------
void OptDialog::selectSatellitePcvFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Satellite Antenna PCV File"), ui->lESatellitePcvFile->text(), tr("PCV File (*.pcv *.atx);Position File (*.pcv *.snx);All (*.*)"));

    if (!filename.isEmpty())
        ui->lESatellitePcvFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OptDialog::viewAntennaPcvFile()
{
    if (ui->lEAntennaPcvFile->text().isEmpty()) return;

    textViewer->read(ui->lEAntennaPcvFile->text());

    textViewer->show();
}
//---------------------------------------------------------------------------
void OptDialog::selectAntennaPcvFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Receiver Antenna PCV File"), ui->lEAntennaPcvFile->text(), tr("PCV File (*.pcv *.atx);Position File (*.pcv *.snx);All (*.*)"));

    if (filename.isEmpty()) return;

    ui->lEAntennaPcvFile->setText(QDir::toNativeSeparators(filename));

    readAntennaList();
}
//---------------------------------------------------------------------------
void OptDialog::selectGeoidDataFile()
{

    QString filename = QFileDialog::getOpenFileName(this, tr("Geoid Data File"), ui->lEGeoidDataFile->text(), tr("All (*.*)"));

    if (!filename.isEmpty())
        ui->lEGeoidDataFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OptDialog::selectDCBFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("DCB Data File"), ui->lEDCBFile->text(), tr("DCB Data File (*.dcb *.DCB);;All (*.*)"));

    if (!filename.isEmpty())
        ui->lEDCBFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OptDialog::viewDCBFile()
{
    if (ui->lEDCBFile->text().isEmpty()) return;

    textViewer->read(ui->lEDCBFile->text());

    textViewer->show();
}
//---------------------------------------------------------------------------
void OptDialog::selectEOPFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("EOP Date File"), ui->lEEOPFile->text(), tr("EOP Data File (*.eop *.erp);;All (*.*)"));

    if (!filename.isEmpty())
        ui->lEEOPFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OptDialog::viewEOPFile()
{
    if (ui->lEEOPFile->text().isEmpty()) return;

    textViewer->read(ui->lEEOPFile->text());

    textViewer->show();
}
//---------------------------------------------------------------------------
void OptDialog::selectBLQFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Ocean Tide Loading BLQ File"), ui->lEBLQFile->text(), tr("OTL BLQ File (*.blq);;All (*.*)"));

    if (!filename.isEmpty())
        ui->lEBLQFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OptDialog::viewBLQFile()
{
    QString blqFilename = ui->lEBLQFile->text();

    if (blqFilename.isEmpty()) return;

    textViewer->read(blqFilename);
    textViewer->show();
}
//---------------------------------------------------------------------------
void OptDialog::selectLocalDirectory()
{
    QString dir = ui->lELocalDirectory->text();

    dir = QFileDialog::getExistingDirectory(this, tr("FTP/HTTP Local Directory"), dir);

    if (!dir.isEmpty())
        ui->lELocalDirectory->setText(QDir::toNativeSeparators(dir));
}
//---------------------------------------------------------------------------
void OptDialog::selectIonosphereFile()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Ionosphere Data File"), ui->lEIonosphereFile->text(), tr("Ionosphere Data File (*.*i,*stec);;All (*.*)"));

    if (!filename.isEmpty())
        ui->lEIonosphereFile->setText(QDir::toNativeSeparators(filename));
}
//---------------------------------------------------------------------------
void OptDialog::selectPanelFont()
{
    QFontDialog dialog(this);

    dialog.setCurrentFont(ui->fontLabelPanel->font());
    dialog.exec();

    ui->fontLabelPanel->setFont(dialog.selectedFont());
    ui->fontLabelPanel->setText(tr("%1 %2 pt").arg(ui->fontLabelPanel->font().family()).arg(ui->fontLabelPanel->font().pointSize()));
    setWidgetTextColor(ui->fontLabelPanel, panelFontColor);
}
//---------------------------------------------------------------------------
void OptDialog::selectSolutionFont()
{
    QFontDialog dialog(this);

    dialog.setCurrentFont(ui->fontLabelSolution->font());
    dialog.exec();

    ui->fontLabelSolution->setFont(dialog.selectedFont());
    ui->fontLabelSolution->setText(tr("%1 %2 pt").arg(ui->fontLabelSolution->font().family()).arg(ui->fontLabelSolution->font().pointSize()));
    setWidgetTextColor(ui->fontLabelSolution, positionFontColor);
}
//---------------------------------------------------------------------------
void OptDialog::updateOptions()
{
    double rovPos[3], refPos[3];
    QLineEdit *editu[] = {ui->lERoverPosition1, ui->lERoverPosition2, ui->lERoverPosition3 };
    QLineEdit *editr[] = {ui->lEReferencePosition1, ui->lEReferencePosition2, ui->lEReferencePosition3 };
    pcvs_t pcvr;
    pcv_t *pcv, pcv0;
    gtime_t time = timeget();

    memset(&pcvr, 0, sizeof(pcvs_t));
    memset(&pcv0, 0, sizeof(pcv_t));

    // file options
    strncpy(fileOptions.satantp, qPrintable(ui->lESatellitePcvFile->text()), MAXSTRPATH-1);
    strncpy(fileOptions.rcvantp, qPrintable(ui->lEAntennaPcvFile->text()), MAXSTRPATH-1);
    strncpy(fileOptions.stapos, qPrintable(ui->lEStationPositionFile->text()), MAXSTRPATH-1);
    strncpy(fileOptions.geoid, qPrintable(ui->lEGeoidDataFile->text()), MAXSTRPATH-1);
    if (options == PostOptions)
        strncpy(fileOptions.iono, qPrintable(ui->lEIonosphereFile->text()), MAXSTRPATH-1);
    strncpy(fileOptions.dcb, qPrintable(ui->lEDCBFile->text()), MAXSTRPATH-1);
    strncpy(fileOptions.eop, qPrintable(ui->lEEOPFile->text()), MAXSTRPATH-1);
    strncpy(fileOptions.blq, qPrintable(ui->lEBLQFile->text()), MAXSTRPATH-1);
    if (options == NaviOptions)
        strncpy(fileOptions.tempdir, qPrintable(ui->lELocalDirectory->text()), MAXSTRPATH-1);
    // fileOptions.geexe
    // fileOptions.solstat
    // fileOptions.trace

    processingOptions.mode = ui->cBPositionMode->currentIndex();
    processingOptions.soltype = ui->cBSolution->currentIndex();
    processingOptions.nf = ui->cBFrequencies->currentIndex() + 1 > NFREQ ? NFREQ : ui->cBFrequencies->currentIndex() + 1;
    processingOptions.navsys = 0;
    if (ui->cBNavSys1->isChecked()) processingOptions.navsys |= SYS_GPS;
    if (ui->cBNavSys2->isChecked()) processingOptions.navsys |= SYS_GLO;
    if (ui->cBNavSys3->isChecked()) processingOptions.navsys |= SYS_GAL;
    if (ui->cBNavSys4->isChecked()) processingOptions.navsys |= SYS_QZS;
    if (ui->cBNavSys5->isChecked()) processingOptions.navsys |= SYS_SBS;
    if (ui->cBNavSys6->isChecked()) processingOptions.navsys |= SYS_CMP;
    if (ui->cBNavSys7->isChecked()) processingOptions.navsys |= SYS_IRN;
    processingOptions.elmin = ui->cBElevationMask->currentText().toDouble() * D2R;
    // snrmask: already set by calling mask dialog
    processingOptions.sateph = ui->cBSatelliteEphemeris->currentIndex();
    processingOptions.modear = ui->cBAmbiguityResolutionGPS->currentIndex();
    processingOptions.glomodear = ui->cBAmbiguityResolutionGLO->currentIndex();
    processingOptions.bdsmodear = ui->cBAmbiguityResolutionBDS->currentIndex();
    processingOptions.arfilter = ui->cBARFilter->currentIndex();
    processingOptions.maxout = ui->sBOutageCountResetAmbiguity->value();
    processingOptions.minlock = ui->sBLockCountFixAmbiguity->value();
    processingOptions.minfixsats = ui->sBMinFixSats->value();
    processingOptions.minholdsats = ui->sBMinHoldSats->value();
    processingOptions.mindropsats = ui->sBMinDropSats->value();
    processingOptions.minfix = ui->sBFixCountHoldAmbiguity->value();
    // armaxiter
    processingOptions.ionoopt = ui->cBIonosphereOption->currentIndex();
    processingOptions.tropopt = ui->cBTroposphereOption->currentIndex();
    processingOptions.dynamics = ui->cBDynamicModel->currentIndex();
    processingOptions.tidecorr = ui->cBTideCorrection->currentIndex();
    processingOptions.niter = ui->sBNumIteration->value();
    // codesmooth
    processingOptions.intpref = ui->cBIntputReferenceObservation->currentIndex();
    // sbascorr
    if (options == NaviOptions)
        processingOptions.sbassatsel = ui->sBSbasSatellite->value();
    else if (options == PostOptions)
        processingOptions.sbassatsel = ui->sBSbasSat->value();

    processingOptions.eratio[0] = ui->sBMeasurementErrorR1->value();
    processingOptions.eratio[1] = ui->sBMeasurementErrorR2->value();
    processingOptions.eratio[2] = ui->sBMeasurementErrorR5->value();
    processingOptions.err[0] = 0;
    processingOptions.err[1] = ui->sBMeasurementError2->value();
    processingOptions.err[2] = ui->sBMeasurementError3->value();
    processingOptions.err[3] = ui->sBMeasurementError4->value();
    processingOptions.err[4] = ui->sBMeasurementError5->value();
    processingOptions.err[5] = ui->sBMeasurementErrorSNR_Max->value();
    processingOptions.err[6] = ui->sBMeasurementErrorSNR->value();
    processingOptions.err[7] = ui->sBMeasurementErrorReceiver->value();
    // std
    processingOptions.prn[0] = ui->sBProcessNoise1->value();
    processingOptions.prn[1] = ui->sBProcessNoise2->value();
    processingOptions.prn[2] = ui->sBProcessNoise3->value();
    processingOptions.prn[3] = ui->sBProcessNoise4->value();
    processingOptions.prn[4] = ui->sBProcessNoise5->value();
    processingOptions.sclkstab = ui->sBSatelliteClockStability->value();
    processingOptions.thresar[0] = ui->sBValidThresAR->value();
    processingOptions.thresar[1] = ui->sBMaxPositionVarAR->value();
    processingOptions.thresar[2] = ui->sBGlonassHwBias->value();
    processingOptions.thresar[5] = ui->sBValidThresARMin->value();
    processingOptions.thresar[6] = ui->sBValidThresARMax->value();
    processingOptions.elmaskar = ui->sBElevationMaskAR->value() * D2R;
    processingOptions.elmaskhold = ui->sBElevationMaskHold->value() * D2R;
    processingOptions.thresslip = ui->sBSlipThreshold->value();
    processingOptions.thresdop = ui->sBDopplerThreshold->value();
    processingOptions.varholdamb = ui->sBVarHoldAmb->value();
    processingOptions.gainholdamb = ui->sBGainHoldAmb->value();
    processingOptions.maxtdiff = ui->sBMaxAgeDifferences->value();
    processingOptions.maxinno[0] = ui->sBRejectPhase->value();
    processingOptions.maxinno[1] = ui->sBRejectCode->value();
    if (ui->cBBaselineConstrain->isChecked()) {
        processingOptions.baseline[0] = ui->sBBaselineLen->value();
        processingOptions.baseline[1] = ui->sBBaselineSig->value();
    } else {
        processingOptions.baseline[0] = 0.0;
        processingOptions.baseline[1] = 0.0;
    }

    getPosition(ui->cBRoverPositionType->currentIndex(), editu, rovPos);
    getPosition(ui->cBReferencePositionType->currentIndex(), editr, refPos);

    for (int i = 0; i < 3; i++) processingOptions.rb[i] = refPos[i];
    processingOptions.refpos = POSOPT_POS_LLH;  /* Fallback */
    if (ui->cBReferencePositionType->currentIndex() < 2) processingOptions.refpos = POSOPT_POS_LLH;
    else if (ui->cBReferencePositionType->currentIndex() == 2) processingOptions.refpos = POSOPT_POS_XYZ;
    else if (options == NaviOptions) {
        if (ui->cBReferencePositionType->currentIndex() == 3) processingOptions.refpos = POSOPT_SINGLE;
        else if (ui->cBReferencePositionType->currentIndex() == 6) processingOptions.refpos = POSOPT_RTCM;
    } else if (options == PostOptions) {
        processingOptions.refpos = ui->cBReferencePositionType->currentIndex() - 1; /* 2:single, 3:posfile, 4:rinex, 5: rtcm */
    }

    for (int i = 0; i < 3; i++) processingOptions.ru[i] = rovPos[i];
    processingOptions.rovpos = POSOPT_POS_LLH;
    if (ui->cBRoverPositionType->currentIndex() < 2) processingOptions.rovpos = POSOPT_POS_LLH;
    else if (ui->cBRoverPositionType->currentIndex() == 2) processingOptions.rovpos = POSOPT_POS_XYZ;
    else if (options == PostOptions) {
        processingOptions.rovpos = ui->cBRoverPositionType->currentIndex() - 1; /* 2:single, 3:posfile, 4:rinex */
    }

    if (ui->cBRoverAntennaPcv->isChecked()) {
        strncpy(processingOptions.anttype[0], qPrintable(ui->cBRoverAntenna->currentText()), 63);
        processingOptions.antdel[0][0] = ui->sBRoverAntennaE->value();
        processingOptions.antdel[0][1] = ui->sBRoverAntennaN->value();
        processingOptions.antdel[0][2] = ui->sBRoverAntennaU->value();
    } else {
        strncpy(processingOptions.anttype[0], "", sizeof(processingOptions.anttype[0]) - 1);
        processingOptions.antdel[0][0] = 0;
        processingOptions.antdel[0][1] = 0;
        processingOptions.antdel[0][2] = 0;
    }

    if (ui->cBReferenceAntennaPcv->isChecked()) {
        strncpy(processingOptions.anttype[1], qPrintable(ui->cBReferenceAntenna->currentText()), 63);
        processingOptions.antdel[1][0] = ui->sBReferenceAntennaE->value();
        processingOptions.antdel[1][1] = ui->sBReferenceAntennaN->value();
        processingOptions.antdel[1][2] = ui->sBReferenceAntennaU->value();
    } else {
        strncpy(processingOptions.anttype[1], "", sizeof(processingOptions.anttype[1]) - 1);
        processingOptions.antdel[1][0] = 0;
        processingOptions.antdel[1][1] = 0;
        processingOptions.antdel[1][2] = 0;
    }

    processingOptions.pcvr[0] = processingOptions.pcvr[1] = pcv0; // initialize antenna PCV
    if ((ui->cBRoverAntennaPcv->isChecked() || ui->cBReferenceAntennaPcv->isChecked()) && !readpcv(fileOptions.rcvantp, &pcvr)) {
        QMessageBox::warning(this, tr("Error"), tr("Antenna file read error: \"%1\"").arg(fileOptions.rcvantp));
        return;
    }
    if (ui->cBRoverAntennaPcv->isChecked() && (processingOptions.anttype[0] != QStringLiteral("*"))) {
        if ((pcv = searchpcv(0, processingOptions.anttype[0], time, &pcvr)))
            processingOptions.pcvr[0] = *pcv;
        else
            QMessageBox::warning(this, tr("Error"), tr("No rover antenna PCV: \"%1\"").arg(processingOptions.anttype[0]));
    }
    if (ui->cBReferenceAntennaPcv->isChecked()&& (processingOptions.anttype[1] != QStringLiteral("*"))) {
        if ((pcv = searchpcv(0, processingOptions.anttype[1], time, &pcvr)))
            processingOptions.pcvr[1] = *pcv;
        else
            QMessageBox::warning(this, tr("Error"), tr("No reference station antenna PCV: \"%1\"").arg(processingOptions.anttype[1]));
    }
    if (ui->cBRoverAntennaPcv->isChecked() || ui->cBReferenceAntennaPcv->isChecked())
        free(pcvr.pcv);
    fillExcludedSatellites(&processingOptions, ui->lEExcludedSatellites->text());
    processingOptions.maxaveep = ui->sBMaxAveEp->value();
    processingOptions.initrst = ui->cBInitRestart->isChecked();
    processingOptions.outsingle = ui->cBOutputSingle->currentIndex();
    if (options == PostOptions) {
        strncpy(processingOptions.rnxopt[0], qPrintable(ui->lERnxOptions1->text()), 255);
        strncpy(processingOptions.rnxopt[1], qPrintable(ui->lERnxOptions2->text()), 255);
    }
    processingOptions.posopt[0] = ui->cBPositionOption1->isChecked();
    processingOptions.posopt[1] = ui->cBPositionOption2->isChecked();
    processingOptions.posopt[2] = ui->cBPositionOption3->isChecked();
    processingOptions.posopt[3] = ui->cBPositionOption4->isChecked();
    processingOptions.posopt[4] = ui->cBPositionOption5->isChecked();
    processingOptions.posopt[5] = ui->cBPositionOption6->isChecked();
    if (options == NaviOptions)
        processingOptions.syncsol = ui->cBSyncSolution->currentIndex();
    /// processingOptions.odisp
    /// processingOptions.freqopt
    if (options == PostOptions)
        strncpy(processingOptions.pppopt, qPrintable(ui->lEPPPOptions->text()), 255);

    // solution options
    solutionOptions.posf = ui->cBSolutionFormat->currentIndex();
    solutionOptions.times = ui->cBTimeFormat->currentIndex() == 0 ? TIMES_GPST : ui->cBTimeFormat->currentIndex() - 1;
    solutionOptions.timef = ui->cBTimeFormat->currentIndex() == 0 ? 0 : 1;
    solutionOptions.timeu =ui->sBTimeDecimal->value();
    solutionOptions.degf = ui->cBLatLonFormat->currentIndex();
    solutionOptions.outhead = ui->cBOutputHeader->currentIndex();
    solutionOptions.outopt = ui->cBOutputOptions->currentIndex();
    solutionOptions.outvel = ui->cBOutputVelocity->currentIndex();
    solutionOptions.datum = ui->cBOutputDatum->currentIndex();
    solutionOptions.height = ui->cBOutputHeight->currentIndex();
    solutionOptions.geoid = ui->cBOutputGeoid->currentIndex();
    solutionOptions.solstatic = ui->cBSolutionStatic->currentIndex();
    solutionOptions.sstat = ui->cBDebugStatus->currentIndex();
    solutionOptions.trace = ui->cBDebugTrace->currentIndex();
    if (options == NaviOptions) {
        solutionOptions.nmeaintv[0] = ui->sBNmeaInterval1->value();
        solutionOptions.nmeaintv[1] = ui->sBNmeaInterval2->value();
    }
    strncpy(solutionOptions.sep, ui->lEFieldSeperator->text().isEmpty() ? " " : qPrintable(ui->lEFieldSeperator->text()), 63);
    solutionOptions.maxsolstd = ui->sBMaxSolutionStd->value();

    if (options == NaviOptions) {
        serverCycle = ui->sBServerCycle->value();
        timeoutTime = ui->sBTimeoutTime->value();
        reconnectTime = ui->sBReconnectTime->value();
        nmeaCycle = ui->sBNmeaCycle->value();
        fileSwapMargin = ui->sBFileSwapMargin->value();
        serverBufferSize = ui->sBServerBufferSize->value();
        solutionBufferSize = ui->sBSolutionBufferSize->value();
        savedSolutions = ui->sBSavedSolution->value();
        navSelect = ui->cBNavSelect->currentIndex();

        proxyAddress = ui->lEProxyAddress->text();
        monitorPort = ui->sBMonitorPort->value();
        panelStacking = ui->cBPanelStack->currentIndex();

        panelFont = ui->fontLabelPanel->font();
        positionFont = ui->fontLabelSolution->font();
    } else if (options == PostOptions) {
        roverList = ui->tERoverList->toPlainText();
        baseList = ui->tEBaseList->toPlainText();
    }
}

//---------------------------------------------------------------------------
void OptDialog::accept()
{
    updateOptions();

    QDialog::accept();
}
//---------------------------------------------------------------------------
void OptDialog::load(const QString &file)
{
    QLineEdit *editu[] = {ui->lERoverPosition1, ui->lERoverPosition2, ui->lERoverPosition3};
    QLineEdit *editr[] = {ui->lEReferencePosition1, ui->lEReferencePosition2, ui->lEReferencePosition3};
    prcopt_t prcopt = prcopt_default;
    solopt_t solopt = solopt_default;
    filopt_t filopt;

    memset(&filopt, 0, sizeof(filopt_t));

	resetsysopts();
    if (!loadopts(qPrintable(file), sysopts)) return;
    if (appOptions && !loadopts(qPrintable(file), appOptions)) return;
    if (!loadopts(qPrintable(file), naviopts)) return;
    getsysopts(&prcopt, &solopt, &filopt);
    proxyAddress = proxyaddr;

    ui->sBSbasSatellite->setValue(prcopt.sbassatsel);

    ui->cBPositionMode->setCurrentIndex(prcopt.mode);
    ui->cBSolution->setCurrentIndex(prcopt.soltype);
    ui->cBFrequencies->setCurrentIndex(prcopt.nf > NFREQ - 1 ? NFREQ - 1 : prcopt.nf - 1);
    ui->cBNavSys1->setChecked(prcopt.navsys & SYS_GPS);
    ui->cBNavSys2->setChecked(prcopt.navsys & SYS_GLO);
    ui->cBNavSys3->setChecked(prcopt.navsys & SYS_GAL);
    ui->cBNavSys4->setChecked(prcopt.navsys & SYS_QZS);
    ui->cBNavSys5->setChecked(prcopt.navsys & SYS_SBS);
    ui->cBNavSys6->setChecked(prcopt.navsys & SYS_CMP);
    ui->cBNavSys7->setChecked(prcopt.navsys & SYS_IRN);
    ui->cBElevationMask->setCurrentIndex(ui->cBElevationMask->findText(QString::number(prcopt.elmin * R2D, 'f', 0)));
    snrmask = prcopt.snrmask;
    ui->cBSatelliteEphemeris->setCurrentIndex(prcopt.sateph);
    ui->cBAmbiguityResolutionGPS->setCurrentIndex(prcopt.modear);
    ui->cBAmbiguityResolutionGLO->setCurrentIndex(prcopt.glomodear);
    ui->cBAmbiguityResolutionBDS->setCurrentIndex(prcopt.bdsmodear);
    ui->cBARFilter->setCurrentIndex(prcopt.arfilter);
    ui->sBOutageCountResetAmbiguity->setValue(prcopt.maxout);
    ui->sBLockCountFixAmbiguity->setValue(prcopt.minlock);
    ui->sBMinFixSats->setValue(prcopt.minfixsats);
    ui->sBMinHoldSats->setValue(prcopt.minholdsats);
    ui->sBMinDropSats->setValue(prcopt.mindropsats);
    ui->sBFixCountHoldAmbiguity->setValue(prcopt.minfix);
    //BARIter->setValue(prcopt.armaxiter);
    ui->cBIonosphereOption->setCurrentIndex(prcopt.ionoopt);
    ui->cBTroposphereOption->setCurrentIndex(prcopt.tropopt);
    ui->cBDynamicModel->setCurrentIndex(prcopt.dynamics);
    ui->cBTideCorrection->setCurrentIndex(prcopt.tidecorr);
    ui->sBNumIteration->setValue(prcopt.niter);
    //prcopt.codesmooth
    if (options == PostOptions) {
        ui->cBIntputReferenceObservation->setCurrentIndex(prcopt.intpref);
        ui->sBSbasSat->setValue(prcopt.sbassatsel);
        ui->cBRoverPositionType->setCurrentIndex(prcopt.rovpos == POSOPT_POS_LLH ? 0 : prcopt.rovpos == POSOPT_POS_XYZ ? 2 : prcopt.rovpos + 1);
        ui->cBReferencePositionType->setCurrentIndex(prcopt.refpos == POSOPT_POS_LLH ? 0 : prcopt.refpos == POSOPT_POS_XYZ ? 2 : prcopt.refpos + 1);
    } else if (options == NaviOptions) {
        ui->cBRoverPositionType->setCurrentIndex(0);
        if (prcopt.rovpos == POSOPT_POS_LLH) ui->cBRoverPositionType->setCurrentIndex(0);
        else if (prcopt.rovpos == POSOPT_POS_XYZ) ui->cBRoverPositionType->setCurrentIndex(2);

        ui->cBReferencePositionType->setCurrentIndex(0);
        if (prcopt.refpos == POSOPT_POS_LLH) ui->cBReferencePositionType->setCurrentIndex(0);
        else if (prcopt.refpos == POSOPT_POS_XYZ) ui->cBReferencePositionType->setCurrentIndex(2);
        else if (prcopt.refpos == POSOPT_SINGLE) ui->cBReferencePositionType->setCurrentIndex(3);
        else if (prcopt.refpos == POSOPT_RTCM) ui->cBReferencePositionType->setCurrentIndex(6);
    }
    current_roverPositionType = ui->cBRoverPositionType->currentIndex();
    current_referencePositionType = ui->cBReferencePositionType->currentIndex();
    ui->sBMeasurementErrorR1->setValue(prcopt.eratio[0]);
    ui->sBMeasurementErrorR2->setValue(prcopt.eratio[1]);
    ui->sBMeasurementErrorR5->setValue(prcopt.eratio[2]);
    ui->sBMeasurementError2->setValue(prcopt.err[1]);
    ui->sBMeasurementError3->setValue(prcopt.err[2]);
    ui->sBMeasurementError4->setValue(prcopt.err[3]);
    ui->sBMeasurementError5->setValue(prcopt.err[4]);
    ui->sBMeasurementErrorSNR_Max->setValue(prcopt.err[5]);
    ui->sBMeasurementErrorSNR->setValue(prcopt.err[6]);
    ui->sBMeasurementErrorReceiver->setValue(prcopt.err[7]);
    // std
    ui->sBProcessNoise1->setValue(prcopt.prn[0]);
    ui->sBProcessNoise2->setValue(prcopt.prn[1]);
    ui->sBProcessNoise3->setValue(prcopt.prn[2]);
    ui->sBProcessNoise4->setValue(prcopt.prn[3]);
    ui->sBProcessNoise5->setValue(prcopt.prn[4]);
    ui->sBSatelliteClockStability->setValue(prcopt.sclkstab);
    ui->sBValidThresAR->setValue(prcopt.thresar[0]);
    ui->sBMaxPositionVarAR->setValue(prcopt.thresar[1]);
    ui->sBGlonassHwBias->setValue(prcopt.thresar[2]);
    ui->sBValidThresARMin->setValue(prcopt.thresar[5]);
    ui->sBValidThresARMax->setValue(prcopt.thresar[6]);
    ui->sBElevationMaskAR->setValue(prcopt.elmaskar * R2D);
    ui->sBElevationMaskHold->setValue(prcopt.elmaskhold * R2D);
    ui->sBSlipThreshold->setValue(prcopt.thresslip);
    ui->sBDopplerThreshold->setValue(prcopt.thresdop);
    ui->sBVarHoldAmb->setValue(prcopt.varholdamb);
    ui->sBGainHoldAmb->setValue(prcopt.gainholdamb);
    ui->sBMaxAgeDifferences->setValue(prcopt.maxtdiff);
    ui->sBRejectPhase->setValue(prcopt.maxinno[0]);
    ui->sBRejectCode->setValue(prcopt.maxinno[1]);
    ui->sBBaselineLen->setValue(prcopt.baseline[0]);
    ui->sBBaselineSig->setValue(prcopt.baseline[1]);
    ui->cBBaselineConstrain->setChecked(prcopt.baseline[0] > 0.0);
    setPosition(ui->cBRoverPositionType->currentIndex(), editu, prcopt.ru);
    setPosition(ui->cBReferencePositionType->currentIndex(), editr, prcopt.rb);
    ui->cBRoverAntennaPcv->setChecked(*prcopt.anttype[0]);
    ui->cBReferenceAntennaPcv->setChecked(*prcopt.anttype[1]);
    ui->cBRoverAntenna->setCurrentIndex(ui->cBRoverAntenna->findText(prcopt.anttype[0]));
    ui->cBReferenceAntenna->setCurrentIndex(ui->cBReferenceAntenna->findText(prcopt.anttype[1]));
    ui->sBRoverAntennaE->setValue(prcopt.antdel[0][0]);
    ui->sBRoverAntennaN->setValue(prcopt.antdel[0][1]);
    ui->sBRoverAntennaU->setValue(prcopt.antdel[0][2]);
    ui->sBReferenceAntennaE->setValue(prcopt.antdel[1][0]);
    ui->sBReferenceAntennaN->setValue(prcopt.antdel[1][1]);
    ui->sBReferenceAntennaU->setValue(prcopt.antdel[1][2]);
    // prcopt.pcvr

    ui->lEExcludedSatellites->setText(excludedSatellitesString(&prcopt));
    ui->sBMaxAveEp->setValue(prcopt.maxaveep);
    ui->cBInitRestart->setChecked(prcopt.initrst);
    ui->cBOutputSingle->setCurrentIndex(prcopt.outsingle);
    ui->lERnxOptions1->setText(prcopt.rnxopt[0]);
    ui->lERnxOptions2->setText(prcopt.rnxopt[1]);
    ui->cBPositionOption1->setChecked(prcopt.posopt[0]);
    ui->cBPositionOption2->setChecked(prcopt.posopt[1]);
    ui->cBPositionOption3->setChecked(prcopt.posopt[2]);
    ui->cBPositionOption4->setChecked(prcopt.posopt[3]);
    ui->cBPositionOption5->setChecked(prcopt.posopt[4]);
    ui->cBPositionOption6->setChecked(prcopt.posopt[5]);
    ui->cBSyncSolution->setCurrentIndex(prcopt.syncsol);
    /// prcopt.odisp
    /// prcopt.freqopt
    ui->lEPPPOptions->setText(prcopt.pppopt);

    // solution options
    ui->cBSolutionFormat->setCurrentIndex(solopt.posf);
    ui->cBTimeFormat->setCurrentIndex(solopt.timef == 0 ? 0 : solopt.times + 1);
    ui->sBTimeDecimal->setValue(solopt.timeu);
    ui->cBLatLonFormat->setCurrentIndex(solopt.degf);
    ui->cBOutputHeader->setCurrentIndex(solopt.outhead);
    ui->cBOutputOptions->setCurrentIndex(solopt.outopt);
    ui->cBOutputVelocity->setCurrentIndex(solopt.outvel);
    ui->cBOutputDatum->setCurrentIndex(solopt.datum);
    ui->cBOutputHeight->setCurrentIndex(solopt.height);
    ui->cBOutputGeoid->setCurrentIndex(solopt.geoid);
    ui->cBSolutionStatic->setCurrentIndex(solopt.solstatic);
    ui->cBDebugStatus->setCurrentIndex(solopt.sstat);
    ui->cBDebugTrace->setCurrentIndex(solopt.trace);
    ui->sBNmeaInterval1->setValue(solopt.nmeaintv[0]);
    ui->sBNmeaInterval2->setValue(solopt.nmeaintv[1]);
    ui->lEFieldSeperator->setText(solopt.sep);
    ui->sBMaxSolutionStd->setValue(solopt.maxsolstd);

    // file options
    ui->lESatellitePcvFile->setText(filopt.satantp);
    ui->lEAntennaPcvFile->setText(filopt.rcvantp);
    ui->lEStationPositionFile->setText(filopt.stapos);
    ui->lEGeoidDataFile->setText(filopt.geoid);
    if (options == PostOptions)
        ui->lEIonosphereFile->setText(filopt.iono);
    ui->lEDCBFile->setText(filopt.dcb);
    ui->lEEOPFile->setText(filopt.eop);
    ui->lEBLQFile->setText(filopt.blq);
    if (options == NaviOptions)
        ui->lELocalDirectory->setText(filopt.tempdir);
    // filopt.geexe
    // filopt.solstat
    // filopt.trace

    readAntennaList();
	updateEnable();
}
//---------------------------------------------------------------------------
void OptDialog::save(const QString &file)
{
    QLineEdit *editu[] = {ui->lERoverPosition1, ui->lERoverPosition2, ui->lERoverPosition3};
    QLineEdit *editr[] = {ui->lEReferencePosition1, ui->lEReferencePosition2, ui->lEReferencePosition3};
    char comment[256], s[64];
    prcopt_t procOpts = prcopt_default;
    solopt_t solOpts = solopt_default;
    filopt_t filopt;

    memset(&filopt, 0, sizeof(filopt_t));

    if (options == NaviOptions) {
        serverCycle = ui->sBServerCycle->value();
        timeoutTime = ui->sBTimeoutTime->value();
        reconnectTime = ui->sBReconnectTime->value();
        nmeaCycle = ui->sBNmeaCycle->value();
        serverBufferSize = ui->sBServerBufferSize->value();
        navSelect = ui->cBNavSelect->currentIndex();
        proxyAddress = ui->lEProxyAddress->text();
        fileSwapMargin = ui->sBFileSwapMargin->value();
    }

    procOpts.mode = ui->cBPositionMode->currentIndex();
    procOpts.soltype = ui->cBSolution->currentIndex();
    procOpts.nf = ui->cBFrequencies->currentIndex() + 1 > NFREQ ? NFREQ : ui->cBFrequencies->currentIndex() + 1;
    procOpts.navsys = (ui->cBNavSys1->isChecked() ? SYS_GPS : 0) |
                      (ui->cBNavSys2->isChecked() ? SYS_GLO : 0) |
                      (ui->cBNavSys3->isChecked() ? SYS_GAL : 0) |
                      (ui->cBNavSys4->isChecked() ? SYS_QZS : 0) |
                      (ui->cBNavSys5->isChecked() ? SYS_SBS : 0) |
                      (ui->cBNavSys6->isChecked() ? SYS_CMP : 0) |
                      (ui->cBNavSys7->isChecked() ? SYS_IRN : 0);
    procOpts.elmin = ui->cBElevationMask->currentText().toDouble() * D2R;
    procOpts.snrmask = snrmask;
    procOpts.sateph = ui->cBSatelliteEphemeris->currentIndex();
    procOpts.modear = ui->cBAmbiguityResolutionGPS->currentIndex();
    procOpts.glomodear = ui->cBAmbiguityResolutionGLO->currentIndex();
    procOpts.bdsmodear = ui->cBAmbiguityResolutionBDS->currentIndex();
    procOpts.arfilter = ui->cBARFilter->currentIndex();
    procOpts.maxout = ui->sBOutageCountResetAmbiguity->value();
    procOpts.minlock = ui->sBLockCountFixAmbiguity->value();
    procOpts.minfixsats = ui->sBMinFixSats->value();
    procOpts.minholdsats = ui->sBMinHoldSats->value();
    procOpts.mindropsats = ui->sBMinDropSats->value();
    procOpts.minfix = ui->sBFixCountHoldAmbiguity->value();
    //prcopt.armaxiter = sBARIter->value();
    procOpts.ionoopt = ui->cBIonosphereOption->currentIndex();
    procOpts.tropopt = ui->cBTroposphereOption->currentIndex();
    procOpts.dynamics = ui->cBDynamicModel->currentIndex();
    procOpts.tidecorr = ui->cBTideCorrection->currentIndex();
    procOpts.niter = ui->sBNumIteration->value();
    // procOpts.codesmooth
    procOpts.intpref = ui->cBIntputReferenceObservation->currentIndex();
    // procOpts.sbascorr
    if (options == NaviOptions)
        procOpts.sbassatsel = ui->sBSbasSatellite->value();
    else if (options == PostOptions)
        procOpts.sbassatsel = ui->sBSbasSat->value();

    procOpts.rovpos = ui->cBRoverPositionType->currentIndex() < 2 ? POSOPT_POS_LLH : ui->cBRoverPositionType->currentIndex() == 2 ? POSOPT_POS_XYZ : ui->cBRoverPositionType->currentIndex() - 1;
    if (options == NaviOptions) {
        procOpts.refpos = POSOPT_POS_LLH;
        if      (ui->cBReferencePositionType->currentIndex() < 2) procOpts.refpos = POSOPT_POS_LLH;
        else if (ui->cBReferencePositionType->currentIndex() == 2) procOpts.refpos = POSOPT_POS_XYZ;
        else if (ui->cBReferencePositionType->currentIndex() == 3) procOpts.refpos = POSOPT_SINGLE;
        else if (ui->cBReferencePositionType->currentIndex() == 6) procOpts.refpos = POSOPT_RTCM;
    } else if (options == PostOptions) {
        procOpts.refpos = ui->cBReferencePositionType->currentIndex() < 2 ? POSOPT_POS_LLH : ui->cBReferencePositionType->currentIndex() == 2 ? POSOPT_POS_XYZ : ui->cBReferencePositionType->currentIndex() - 1;
    }
    procOpts.eratio[0] = ui->sBMeasurementErrorR1->value();
    procOpts.eratio[1] = ui->sBMeasurementErrorR2->value();
    procOpts.eratio[2] = ui->sBMeasurementErrorR5->value();
    procOpts.err[0] = 0;
    procOpts.err[1] = ui->sBMeasurementError2->value();
    procOpts.err[2] = ui->sBMeasurementError3->value();
    procOpts.err[3] = ui->sBMeasurementError4->value();
    procOpts.err[4] = ui->sBMeasurementError5->value();
    procOpts.err[5] = ui->sBMeasurementErrorSNR_Max->value();
    procOpts.err[6] = ui->sBMeasurementErrorSNR->value();
    procOpts.err[7] = ui->sBMeasurementErrorReceiver->value();
    // std
    procOpts.prn[0] = ui->sBProcessNoise1->value();
    procOpts.prn[1] = ui->sBProcessNoise2->value();
    procOpts.prn[2] = ui->sBProcessNoise3->value();
    procOpts.prn[3] = ui->sBProcessNoise4->value();
    procOpts.prn[4] = ui->sBProcessNoise5->value();
    procOpts.sclkstab = ui->sBSatelliteClockStability->value();
    procOpts.thresar[0] = ui->sBValidThresAR->value();
    procOpts.thresar[1] = ui->sBMaxPositionVarAR->value();
    procOpts.thresar[2] = ui->sBGlonassHwBias->value();
    procOpts.thresar[5] = ui->sBValidThresARMin->value();
    procOpts.thresar[6] = ui->sBValidThresARMax->value();
    procOpts.elmaskar = ui->sBElevationMaskAR->value() * D2R;
    procOpts.elmaskhold = ui->sBElevationMaskHold->value() * D2R;
    procOpts.thresslip = ui->sBSlipThreshold->value();
    procOpts.thresdop = ui->sBDopplerThreshold->value();
    procOpts.varholdamb = ui->sBVarHoldAmb->value();
    procOpts.gainholdamb = ui->sBGainHoldAmb->value();
    procOpts.maxtdiff = ui->sBMaxAgeDifferences->value();
    procOpts.maxinno[0] = ui->sBRejectPhase->value();
    procOpts.maxinno[1] = ui->sBRejectCode->value();
    if (procOpts.mode == PMODE_MOVEB && ui->cBBaselineConstrain->isChecked()) {
        procOpts.baseline[0] = ui->sBBaselineLen->value();
        procOpts.baseline[1] = ui->sBBaselineSig->value();
    }
    if (procOpts.rovpos == POSOPT_POS_LLH || procOpts.rovpos == POSOPT_POS_XYZ) getPosition(ui->cBRoverPositionType->currentIndex(), editu, procOpts.ru);
    if (procOpts.refpos == POSOPT_POS_LLH || procOpts.refpos == POSOPT_POS_XYZ) getPosition(ui->cBReferencePositionType->currentIndex(), editr, procOpts.rb);
    if (ui->cBRoverAntennaPcv->isChecked()) strncpy(procOpts.anttype[0], qPrintable(ui->cBRoverAntenna->currentText()), MAXANT-1);
    if (ui->cBReferenceAntennaPcv->isChecked()) strncpy(procOpts.anttype[1], qPrintable(ui->cBReferenceAntenna->currentText()), MAXANT-1);
    procOpts.antdel[0][0] = ui->sBRoverAntennaE->value();
    procOpts.antdel[0][1] = ui->sBRoverAntennaN->value();
    procOpts.antdel[0][2] = ui->sBRoverAntennaU->value();
    procOpts.antdel[1][0] = ui->sBReferenceAntennaE->value();
    procOpts.antdel[1][1] = ui->sBReferenceAntennaN->value();
    procOpts.antdel[1][2] = ui->sBReferenceAntennaU->value();
    // procOpts.pcvr
    fillExcludedSatellites(&procOpts, ui->lEExcludedSatellites->text());
    procOpts.maxaveep = ui->sBMaxAveEp->value();
    procOpts.initrst = ui->cBInitRestart->isChecked();
    procOpts.outsingle = ui->cBOutputSingle->currentIndex();
    if (options == PostOptions) {
        strncpy(procOpts.rnxopt[0], qPrintable(ui->lERnxOptions1->text()), 255);
        strncpy(procOpts.rnxopt[1], qPrintable(ui->lERnxOptions2->text()), 255);
    }
    procOpts.posopt[0] = ui->cBPositionOption1->isChecked();
    procOpts.posopt[1] = ui->cBPositionOption2->isChecked();
    procOpts.posopt[2] = ui->cBPositionOption3->isChecked();
    procOpts.posopt[3] = ui->cBPositionOption4->isChecked();
    procOpts.posopt[4] = ui->cBPositionOption5->isChecked();
    procOpts.posopt[5] = ui->cBPositionOption6->isChecked();
    procOpts.syncsol = ui->cBSyncSolution->currentIndex();
    /// proc.Opts.odisp
    /// procOptsfreqopt
    if (options == PostOptions)
        strncpy(procOpts.pppopt, qPrintable(ui->lEPPPOptions->text()), 255);

    solOpts.posf = ui->cBSolutionFormat->currentIndex();
    solOpts.times = ui->cBTimeFormat->currentIndex() == 0 ? TIMES_GPST : ui->cBTimeFormat->currentIndex() - 1;
    solOpts.timef = ui->cBTimeFormat->currentIndex() == 0 ? 0 : 1;
    solOpts.timeu = ui->sBTimeDecimal->value();
    solOpts.degf = ui->cBLatLonFormat->currentIndex();
    solOpts.outhead = ui->cBOutputHeader->currentIndex();
    solOpts.outopt = ui->cBOutputOptions->currentIndex();
    solOpts.outvel = ui->cBOutputVelocity->currentIndex();
    solOpts.datum = ui->cBOutputDatum->currentIndex();
    solOpts.height = ui->cBOutputHeight->currentIndex();
    solOpts.geoid = ui->cBOutputGeoid->currentIndex();
    solOpts.solstatic = ui->cBSolutionStatic->currentIndex();
    solOpts.trace = ui->cBDebugTrace->currentIndex();
    solOpts.sstat = ui->cBDebugStatus->currentIndex();
    solOpts.nmeaintv[0] = ui->sBNmeaInterval1->value();
    solOpts.nmeaintv[1] = ui->sBNmeaInterval2->value();
    strncpy(solOpts.sep, qPrintable(ui->lEFieldSeperator->text()), 63);
    solOpts.maxsolstd = ui->sBMaxSolutionStd->value();

    strncpy(filopt.satantp, qPrintable(ui->lESatellitePcvFile->text()), MAXSTRPATH-1);
    strncpy(filopt.rcvantp, qPrintable(ui->lEAntennaPcvFile->text()), MAXSTRPATH-1);
    strncpy(filopt.stapos, qPrintable(ui->lEStationPositionFile->text()), MAXSTRPATH-1);
    strncpy(filopt.geoid, qPrintable(ui->lEGeoidDataFile->text()), MAXSTRPATH-1);
    if (options == PostOptions)
        strncpy(filopt.iono, qPrintable(ui->lEIonosphereFile->text()), MAXSTRPATH-1);
    strncpy(filopt.dcb, qPrintable(ui->lEDCBFile->text()), MAXSTRPATH-1);
    strncpy(filopt.eop, qPrintable(ui->lEEOPFile->text()), MAXSTRPATH-1);
    strncpy(filopt.blq, qPrintable(ui->lEBLQFile->text()), MAXSTRPATH-1);
    if (options == NaviOptions)
        strncpy(filopt.tempdir, qPrintable(ui->lELocalDirectory->text()), MAXSTRPATH-1);
    // filopt.geexe
    // filopt.solstat
    // filopt.trace

    time2str(utc2gpst(timeget()), s, 0);
    sprintf(comment, "rtk options (%s, v.%s %s)", s, VER_RTKLIB, PATCH_LEVEL);

    setsysopts(&procOpts, &solOpts, &filopt);

    if (!saveopts(qPrintable(file), "w", comment, sysopts)) return;
    if (appOptions && !saveopts(qPrintable(file), "a", "", appOptions)) return;

    if (options == NaviOptions) {
        strncpy(proxyaddr, qPrintable(proxyAddress), 1023);
        saveopts(qPrintable(file), "a", "", naviopts);
    }
}
//---------------------------------------------------------------------------
void OptDialog::saveOptions(QSettings &settings)
{
    double rovPos[3], refPos[3];
    QLineEdit *editu[] = {ui->lERoverPosition1, ui->lERoverPosition2, ui->lERoverPosition3};
    QLineEdit *editr[] = {ui->lEReferencePosition1, ui->lEReferencePosition2, ui->lEReferencePosition3};

    settings.setValue("prcopt/mode", ui->cBPositionMode->currentIndex());
    if (options == PostOptions)
        settings.setValue("prcopt/soltype", ui->cBSolution->currentIndex());
    settings.setValue("prcopt/nf", ui->cBFrequencies->currentIndex() + 1);
    settings.setValue("prcopt/navsys", (ui->cBNavSys1->isChecked() ? SYS_GPS : 0) |
                                       (ui->cBNavSys2->isChecked() ? SYS_GLO : 0) |
                                       (ui->cBNavSys3->isChecked() ? SYS_GAL : 0) |
                                       (ui->cBNavSys4->isChecked() ? SYS_QZS : 0) |
                                       (ui->cBNavSys5->isChecked() ? SYS_SBS : 0) |
                                       (ui->cBNavSys6->isChecked() ? SYS_CMP : 0) |
                                       (ui->cBNavSys7->isChecked() ? SYS_IRN : 0));
    settings.setValue("prcopt/elmin", ui->cBElevationMask->currentText().toDouble() * D2R);
    settings.setValue("prcopt/snrmask_ena1", snrmask.ena[0]);
    settings.setValue("prcopt/snrmask_ena2", snrmask.ena[1]);
    for (int i = 0; i < NFREQ; i++)
        for (int j = 0; j < 9; j++)
            settings.setValue(QStringLiteral("prcopt/snrmask_%1_%2").arg(i + 1).arg(j + 1),
                              snrmask.mask[i][j]);

    settings.setValue("prcopt/ephopt", ui->cBSatelliteEphemeris->currentIndex());
    settings.setValue("prcopt/modear", ui->cBAmbiguityResolutionGPS->currentIndex());
    settings.setValue("prcopt/glomodear", ui->cBAmbiguityResolutionGLO->currentIndex());
    settings.setValue("prcopt/bdsmodear", ui->cBAmbiguityResolutionBDS->currentIndex());
    settings.setValue("prcopt/arfilter", ui->cBARFilter->currentIndex());
    settings.setValue("prcopt/maxout", ui->sBOutageCountResetAmbiguity->value());
    settings.setValue("prcopt/minlock", ui->sBLockCountFixAmbiguity->value());
    settings.setValue("prcopt/minfixsats", ui->sBMinFixSats->value());
    settings.setValue("prcopt/minholdsats", ui->sBMinHoldSats->value());
    settings.setValue("prcopt/mindropsats", ui->sBMinDropSats->value());
    settings.setValue("prcopt/minfix", ui->sBFixCountHoldAmbiguity->value());
    // settings.setValue("prcopt/armaxiter", sBARIter->value());
    settings.setValue("prcopt/ionoopt", ui->cBIonosphereOption->currentIndex());
    settings.setValue("prcopt/tropopt", ui->cBTroposphereOption->currentIndex());
    settings.setValue("prcopt/dynamics", ui->cBDynamicModel->currentIndex());
    settings.setValue("prcopt/tidecorr", ui->cBTideCorrection->currentIndex());
    settings.setValue("prcopt/niter", ui->sBNumIteration->value());
    // settings.setValue("prcopt/codesmooth", processingOptions.codesmooth);
    settings.setValue("prcopt/intpref", ui->cBIntputReferenceObservation->currentIndex());
    if (options == NaviOptions)
        settings.setValue("setting/sbassat", ui->sBSbasSatellite->value());
    else if (options == PostOptions)
        settings.setValue("setting/sbassat", ui->sBSbasSat->value());

    settings.setValue("setting/rovpostype", ui->cBRoverPositionType->currentIndex());
    settings.setValue("setting/refpostype", ui->cBReferencePositionType->currentIndex());
    if (ui->cBRoverPositionType->currentIndex() < 3) getPosition(ui->cBRoverPositionType->currentIndex(), editu, rovPos);
    if (ui->cBReferencePositionType->currentIndex() < 3) getPosition(ui->cBReferencePositionType->currentIndex(), editr, refPos);

    for (int i = 0; i < 3; i++) {
        settings.setValue(QString("setting/rovpos_%1").arg(i), rovPos[i]);
        settings.setValue(QString("setting/refpos_%1").arg(i), refPos[i]);
    }
    settings.setValue("prcopt/eratio0", ui->sBMeasurementErrorR1->value());
    settings.setValue("prcopt/eratio1", ui->sBMeasurementErrorR2->value());
    settings.setValue("prcopt/eratio5", ui->sBMeasurementErrorR5->value());
    settings.setValue("prcopt/err1", ui->sBMeasurementError2->value());
    settings.setValue("prcopt/err2", ui->sBMeasurementError3->value());
    settings.setValue("prcopt/err3", ui->sBMeasurementError4->value());
    settings.setValue("prcopt/err4", ui->sBMeasurementError5->value());
    settings.setValue("prcopt/err5", ui->sBMeasurementErrorSNR_Max->value());
    settings.setValue("prcopt/err6", ui->sBMeasurementErrorSNR->value());
    settings.setValue("prcopt/err7", ui->sBMeasurementErrorReceiver->value());
    // std
    settings.setValue("prcopt/prn0", ui->sBProcessNoise1->value());
    settings.setValue("prcopt/prn1", ui->sBProcessNoise2->value());
    settings.setValue("prcopt/prn2", ui->sBProcessNoise3->value());
    settings.setValue("prcopt/prn3", ui->sBProcessNoise4->value());
    settings.setValue("prcopt/prn4", ui->sBProcessNoise5->value());
    settings.setValue("prcopt/sclkstab", ui->sBSatelliteClockStability->value());
    settings.setValue("prcopt/thresar0", ui->sBValidThresAR->value());
    settings.setValue("prcopt/thresar1", ui->sBMaxPositionVarAR->value());
    settings.setValue("prcopt/thresar2", ui->sBGlonassHwBias->value());
    settings.setValue("prcopt/thresar5", ui->sBValidThresARMin->value());
    settings.setValue("prcopt/thresar6", ui->sBValidThresARMax->value());
    settings.setValue("prcopt/elmaskar", ui->sBElevationMaskAR->value() * D2R);
    settings.setValue("prcopt/elmaskhold", ui->sBElevationMaskHold->value() * D2R);
    settings.setValue("prcopt/thresslip", ui->sBSlipThreshold->value());
    settings.setValue("prcopt/thresdop", ui->sBDopplerThreshold->value());
    settings.setValue("prcopt/varholdamb", ui->sBVarHoldAmb->value());
    settings.setValue("prcopt/gainholdamb", ui->sBGainHoldAmb->value());
    settings.setValue("prcopt/maxtdiff", ui->sBMaxAgeDifferences->value());
    settings.setValue("prcopt/maxinno1", ui->sBRejectPhase->value());
    settings.setValue("prcopt/maxinno2", ui->sBRejectCode->value());
    settings.setValue("prcopt/baselinec", int(ui->cBBaselineConstrain->isChecked()));
    settings.setValue("prcopt/baseline1", ui->sBBaselineLen->value());
    settings.setValue("prcopt/baseline2", ui->sBBaselineSig->value());
    // ru
    // rb
    settings.setValue("prcopt/anttype1", ui->cBRoverAntennaPcv->isChecked() ? ui->cBRoverAntenna->currentText() : "");
    settings.setValue("prcopt/anttype2", ui->cBReferenceAntennaPcv->isChecked() ? ui->cBReferenceAntenna->currentText() : "");
    settings.setValue(QString("setting/rovantdel_0"), ui->sBRoverAntennaE->value());
    settings.setValue(QString("setting/rovantdel_1"), ui->sBRoverAntennaN->value());
    settings.setValue(QString("setting/rovantdel_2"), ui->sBRoverAntennaU->value());
    settings.setValue(QString("setting/refantdel_0"), ui->sBReferenceAntennaE->value());
    settings.setValue(QString("setting/refantdel_1"), ui->sBReferenceAntennaN->value());
    settings.setValue(QString("setting/refantdel_2"), ui->sBReferenceAntennaU->value());
    // pcvr
    settings.setValue("prcopt/exsats", ui->lEExcludedSatellites->text());
    settings.setValue("prcopt/maxaveep", ui->sBMaxAveEp->value());
    settings.setValue("prcopt/initrst", int(ui->cBInitRestart->isChecked()));
    settings.setValue("prcopt/outsingle", ui->cBOutputSingle->currentIndex());
    if (options == PostOptions) {
        settings.setValue("opt/rnxopts1", ui->lERnxOptions1->text());
        settings.setValue("opt/rnxopts2", ui->lERnxOptions2->text());
    }
    settings.setValue("prcopt/posopt1", int(ui->cBPositionOption1->isChecked()));
    settings.setValue("prcopt/posopt2", int(ui->cBPositionOption2->isChecked()));
    settings.setValue("prcopt/posopt3", int(ui->cBPositionOption3->isChecked()));
    settings.setValue("prcopt/posopt4", int(ui->cBPositionOption4->isChecked()));
    settings.setValue("prcopt/posopt5", int(ui->cBPositionOption5->isChecked()));
    settings.setValue("prcopt/posopt6", int(ui->cBPositionOption6->isChecked()));
    settings.setValue("prcopt/syncsol", ui->cBSyncSolution->currentIndex());
    // odisp
    // freqopt
    if (options == PostOptions)
        settings.setValue("opt/pppopts", ui->lEPPPOptions->text());

    // solution options
    settings.setValue("solopt/posf", ui->cBSolutionFormat->currentIndex());
    settings.setValue("solopt/times", ui->cBTimeFormat->currentIndex() == 0 ? 0 : ui->cBTimeFormat->currentIndex() - 1);
    settings.setValue("solopt/timef", ui->cBTimeFormat->currentIndex() == 0 ? 0 : 1);
    settings.setValue("solopt/timeu", ui->sBTimeDecimal->value());
    settings.setValue("solopt/degf", ui->cBLatLonFormat->currentIndex());
    settings.setValue("solopt/outhead", ui->cBOutputHeader->currentIndex());
    settings.setValue("solopt/outopt", ui->cBOutputOptions->currentIndex());
    settings.setValue("solopt/outvel", ui->cBOutputVelocity->currentIndex());
    settings.setValue("solopt/datum", ui->cBOutputDatum->currentIndex());
    settings.setValue("solopt/height", ui->cBOutputHeight->currentIndex());
    settings.setValue("solopt/geoid", ui->cBOutputGeoid->currentIndex());
    settings.setValue("solopt/solstatic", ui->cBSolutionStatic->currentIndex());
    settings.setValue("setting/debugtrace", ui->cBDebugTrace->currentIndex());
    settings.setValue("setting/debugstatus", ui->cBDebugStatus->currentIndex());
    settings.setValue("solopt/nmeaintv1", ui->sBNmeaInterval1->value());
    settings.setValue("solopt/nmeaintv2", ui->sBNmeaInterval2->value());
    settings.setValue("solopt/sep", ui->lEFieldSeperator->text());
    settings.setValue("prcopt/outsingle", processingOptions.outsingle);
    settings.setValue("solopt/maxsolstd", ui->sBMaxSolutionStd->value());

    settings.setValue("setting/satpcvfile", ui->lESatellitePcvFile->text());
    settings.setValue("setting/antpcvfile", ui->lEAntennaPcvFile->text());
    settings.setValue("setting/staposfile", ui->lEStationPositionFile->text());
    settings.setValue("setting/geoiddatafile", ui->lEGeoidDataFile->text());
    if (options == PostOptions)
        settings.setValue("setting/ionosphereFile", ui->lEIonosphereFile->text());
    settings.setValue("setting/dcbfile", ui->lEDCBFile->text());
    settings.setValue("setting/eopfile", ui->lEEOPFile->text());
    settings.setValue("setting/blqfile", ui->lEBLQFile->text());
    settings.setValue("setting/localdirectory", ui->lELocalDirectory->text());

    if (options == NaviOptions) {
        settings.setValue("setting/svrcycle", ui->sBServerCycle->value());
        settings.setValue("setting/timeouttime", ui->sBTimeoutTime->value());
        settings.setValue("setting/recontime", ui->sBReconnectTime->value());
        settings.setValue("setting/nmeacycle", ui->sBNmeaCycle->value());
        settings.setValue("setting/svrbuffsize", ui->sBServerBufferSize->value());
        settings.setValue("setting/solbuffsize", ui->sBSolutionBufferSize->value());
        settings.setValue("setting/savedsol", ui->sBSavedSolution->value());
        settings.setValue("setting/navselect", ui->cBNavSelect->currentIndex());

        settings.setValue("setting/proxyaddr", ui->lEProxyAddress->text());
        settings.setValue("setting/moniport", ui->sBMonitorPort->value());
        settings.setValue("setting/panelstack", ui->cBPanelStack->currentIndex());
        settings.setValue("setting/fswapmargin", ui->sBFileSwapMargin->value());

        QFont panelFnt = ui->fontLabelPanel->font();
        QFont positionFnt = ui->fontLabelSolution->font();
        settings.setValue("setting/panelfontname", panelFnt.family());
        settings.setValue("setting/panelfontsize", panelFnt.pointSize());
        settings.setValue("setting/panelfontcolor", static_cast<int>(panelFontColor.rgb()));
        settings.setValue("setting/panelfontbold", panelFnt.bold());
        settings.setValue("setting/paneöfontitalic", panelFnt.italic());

        settings.setValue("setting/posfontname", positionFnt.family());
        settings.setValue("setting/posfontsize", positionFnt.pointSize());
        settings.setValue("setting/posfontcolor", static_cast<int>(positionFontColor.rgb()));
        settings.setValue("setting/posfontbold", positionFnt.bold());
        settings.setValue("setting/posfontitalic", positionFnt.italic());
    } else if (options == PostOptions) {        
        QString rovList = ui->tERoverList->toPlainText();
        QString refList = ui->tEBaseList->toPlainText();

        rovList.replace("\n", "@@");
        for (int i = 0; i < 10; i++) {
            settings.setValue(QStringLiteral("opt/rovlist%1").arg(i + 1), rovList.mid(i*2000, 2000));
        }

        refList.replace("\n", "@@");
        for (int i = 0; i < 10; i++) {
            settings.setValue(QStringLiteral("opt/baselist%1").arg(i + 1), refList.mid(i*2000, 2000));
        }
    }
}
//---------------------------------------------------------------------------
void OptDialog::loadOptions(QSettings &settings)
{
    double rovPos[3], refPos[3];
    QLineEdit *editu[] = {ui->lERoverPosition1, ui->lERoverPosition2, ui->lERoverPosition3};
    QLineEdit *editr[] = {ui->lEReferencePosition1, ui->lEReferencePosition2, ui->lEReferencePosition3};

    // processing options
    ui->cBPositionMode->setCurrentIndex(settings.value("prcopt/mode", 0).toInt());
    ui->cBSolution->setCurrentIndex(settings.value("prcopt/soltype", 0).toInt());
    ui->cBFrequencies->setCurrentIndex(settings.value("prcopt/nf", 2).toInt() > NFREQ - 1 ? NFREQ - 1 : settings.value("prcopt/nf", 2).toInt() - 1);
    int navsys = settings.value("prcopt/navsys", SYS_GPS).toInt();
    ui->cBNavSys1->setChecked(navsys & SYS_GPS);
    ui->cBNavSys2->setChecked(navsys & SYS_GLO);
    ui->cBNavSys3->setChecked(navsys & SYS_GAL);
    ui->cBNavSys4->setChecked(navsys & SYS_QZS);
    ui->cBNavSys5->setChecked(navsys & SYS_SBS);
    ui->cBNavSys6->setChecked(navsys & SYS_CMP);
    ui->cBNavSys7->setChecked(navsys & SYS_IRN);
    ui->cBElevationMask->setCurrentIndex(ui->cBElevationMask->findText(QString::number(settings.value("prcopt/elmin", 15.0 * D2R).toFloat() * R2D, 'f', 0)));
    snrmask.ena[0] = settings.value("prcopt/snrmask_ena1", 0).toInt();
    snrmask.ena[1] = settings.value("prcopt/snrmask_ena2", 0).toInt();
    for (int i = 0; i < NFREQ; i++)
        for (int j = 0; j < 9; j++)
            snrmask.mask[i][j] =
                settings.value(QString("prcopt/snrmask_%1_%2").arg(i + 1).arg(j + 1), 0.0).toInt();
    ui->cBSatelliteEphemeris->setCurrentIndex(settings.value("prcopt/ephopt", EPHOPT_BRDC).toInt());
    ui->cBAmbiguityResolutionGPS->setCurrentIndex(settings.value("prcopt/modear", 1).toInt());
    ui->cBAmbiguityResolutionGLO->setCurrentIndex(settings.value("prcopt/glomodear", 0).toInt());
    ui->cBAmbiguityResolutionBDS->setCurrentIndex(settings.value("prcopt/bdsmodear", 0).toInt());
    ui->cBARFilter->setCurrentIndex(settings.value("prcopt/arfilter", 1).toInt());
    ui->sBOutageCountResetAmbiguity->setValue(settings.value("prcopt/maxout", 5).toInt());
    ui->sBLockCountFixAmbiguity->setValue(settings.value("prcopt/minlock", 0).toInt());
    ui->sBMinFixSats->setValue(settings.value("prcopt/minfixsats", 4).toInt());
    ui->sBMinHoldSats->setValue(settings.value("prcopt/minholdsats", 5).toInt());
    ui->sBMinDropSats->setValue(settings.value("prcopt/mindropsats", 10).toInt());
    ui->sBFixCountHoldAmbiguity->setValue(settings.value("prcopt/minfix", 10).toInt());
    // BARIter->setValue(settings.value("prcopt/ariter", 1).toInt());
    ui->cBIonosphereOption->setCurrentIndex(settings.value("prcopt/ionoopt", IONOOPT_BRDC).toInt());
    ui->cBTroposphereOption->setCurrentIndex(settings.value("prcopt/tropopt", TROPOPT_SAAS).toInt());
    ui->cBDynamicModel->setCurrentIndex(settings.value("prcopt/dynamics", 0).toInt());
    ui->cBTideCorrection->setCurrentIndex(settings.value("prcopt/tidecorr", 0).toInt());
    ui->sBNumIteration->setValue(settings.value("prcopt/niter", 1).toInt());
    // processingOptions.codesmooth = settings.value("prcopt/codesmooth", 0).toInt();
    if (options == PostOptions) {
        ui->cBIntputReferenceObservation->setCurrentIndex(settings.value("prcopt/intpref", 0).toInt());
        // sbassatsel
        ui->sBSbasSat->setValue(settings.value("setting/sbassat", 0).toInt());
    } else if (options == NaviOptions) {
        ui->cBRoverPositionType->setCurrentIndex(0);
        ui->cBReferencePositionType->setCurrentIndex(0);
    }
    ui->cBRoverPositionType->setCurrentIndex(settings.value("setting/rovpostype", 0).toInt());
    ui->cBReferencePositionType->setCurrentIndex(settings.value("setting/refpostype", 0).toInt());
    current_roverPositionType = settings.value("setting/rovpostype", 0).toInt();
    current_referencePositionType = settings.value("setting/refpostype", 0).toInt();
    ui->sBMeasurementErrorR1->setValue(settings.value("prcopt/eratio0", 100.0).toDouble());
    ui->sBMeasurementErrorR2->setValue(settings.value("prcopt/eratio1", 100.0).toDouble());
    ui->sBMeasurementErrorR5->setValue(settings.value("prcopt/eratio5", 100.0).toDouble());
    ui->sBMeasurementError2->setValue(settings.value("prcopt/err1", 0.003).toDouble());
    ui->sBMeasurementError3->setValue(settings.value("prcopt/err2", 0.003).toDouble());
    ui->sBMeasurementError4->setValue(settings.value("prcopt/err3", 0.0).toDouble());
    ui->sBMeasurementError5->setValue(settings.value("prcopt/err4", 1.0).toDouble());
    ui->sBMeasurementErrorSNR_Max->setValue(settings.value("prcopt/err5", 1.0).toDouble());
    ui->sBMeasurementErrorSNR->setValue(settings.value("prcopt/err6", 1.0).toDouble());
    ui->sBMeasurementErrorReceiver->setValue(settings.value("prcopt/err7", 1.0).toDouble());
    // std
    ui->sBProcessNoise1->setValue(settings.value("prcopt/prn0", 1E-4).toDouble());
    ui->sBProcessNoise2->setValue(settings.value("prcopt/prn1", 1E-3).toDouble());
    ui->sBProcessNoise3->setValue(settings.value("prcopt/prn2", 1E-4).toDouble());
    ui->sBProcessNoise4->setValue(settings.value("prcopt/prn3", 10.0).toDouble());
    ui->sBProcessNoise5->setValue(settings.value("prcopt/prn4", 10.0).toDouble());
    ui->sBSatelliteClockStability->setValue(settings.value("prcopt/sclkstab", 5E-12).toDouble());
    ui->sBValidThresAR->setValue(settings.value("prcopt/thresar0", 100.0).toDouble());
    ui->sBMaxPositionVarAR->setValue(settings.value("prcopt/thresar1", 100.0).toDouble());
    ui->sBGlonassHwBias->setValue(settings.value("prcopt/thresar2", 100.0).toDouble());
    ui->sBValidThresARMin->setValue(settings.value("prcopt/thresar5", 100.0).toDouble());
    ui->sBValidThresARMax->setValue(settings.value("prcopt/thresar6", 100.0).toDouble());
    ui->sBElevationMaskAR->setValue(settings.value("prcopt/elmaskar", 0.0).toDouble());
    ui->sBElevationMaskHold->setValue(settings.value("prcopt/elmaskhold", 0.0).toDouble());
    ui->sBSlipThreshold->setValue(settings.value("prcopt/thresslip", 0.05).toDouble());
    ui->sBDopplerThreshold->setValue(settings.value("prcopt/thresdop", 0.05).toDouble());
    ui->sBVarHoldAmb->setValue(settings.value("prcopt/varholdamb", 0.1).toDouble());
    ui->sBGainHoldAmb->setValue(settings.value("prcopt/gainholdamb", 0.01).toDouble());
    ui->sBMaxAgeDifferences->setValue(settings.value("prcopt/maxtdiff", 30.0).toDouble());
    ui->sBRejectPhase->setValue(settings.value("prcopt/maxinno1", 30.0).toDouble());
    ui->sBRejectCode->setValue(settings.value("prcopt/maxinno2", 30.0).toDouble());
    ui->cBBaselineConstrain->setChecked(settings.value("prcopt/baselinec", 0).toInt());
    ui->sBBaselineLen->setValue(settings.value("prcopt/baseline1", 0.0).toDouble());
    ui->sBBaselineSig->setValue(settings.value("prcopt/baseline2", 0.0).toDouble());

    ui->lESatellitePcvFile->setText(settings.value("setting/satpcvfile", "").toString());
    ui->lEAntennaPcvFile->setText(settings.value("setting/antpcvfile", "").toString());
    readAntennaList();
    ui->cBRoverAntennaPcv->setChecked(!settings.value("prcopt/anttype1", "").toString().isEmpty());
    ui->cBReferenceAntennaPcv->setChecked(!settings.value("prcopt/anttype2", "").toString().isEmpty());
    ui->cBRoverAntenna->setCurrentIndex(ui->cBRoverAntenna->findText(settings.value("prcopt/anttype1", "").toString()));
    ui->cBReferenceAntenna->setCurrentIndex(ui->cBReferenceAntenna->findText(settings.value("prcopt/anttype2", "").toString()));

    for (int i = 0; i < 3; i++) {
        rovPos[i] = settings.value(QStringLiteral("setting/rovpos_%1").arg(i), 0.0).toDouble();
        refPos[i] = settings.value(QStringLiteral("setting/refpos_%1").arg(i), 0.0).toDouble();
    }
    setPosition(ui->cBRoverPositionType->currentIndex(), editu, rovPos);
    setPosition(ui->cBReferencePositionType->currentIndex(), editr, refPos);
    ui->sBRoverAntennaE->setValue(settings.value("setting/rovantdel_0", 0.0).toDouble());
    ui->sBRoverAntennaN->setValue(settings.value("setting/rovantdel_1", 0.0).toDouble());
    ui->sBRoverAntennaU->setValue(settings.value("setting/rovantdel_2", 0.0).toDouble());
    ui->sBReferenceAntennaE->setValue(settings.value("setting/refantdel_0", 0.0).toDouble());
    ui->sBReferenceAntennaN->setValue(settings.value("setting/refantdel_1", 0.0).toDouble());
    ui->sBReferenceAntennaU->setValue(settings.value("setting/refantdel_2", 0.0).toDouble());
    // pcvr
    ui->lEExcludedSatellites->setText(settings.value("prcopt/exsats", "").toString());
    ui->sBMaxAveEp->setValue(settings.value("prcopt/maxaveep", 3600).toInt());
    ui->cBInitRestart->setChecked(settings.value("prcopt/initrst", 1).toInt());
    ui->cBOutputSingle->setCurrentIndex(settings.value("prcopt/outsingle", 0).toInt());
    if (options == PostOptions) {
        ui->lERnxOptions1->setText(settings.value("opt/rnxopts1", "").toString());
        ui->lERnxOptions2->setText(settings.value("opt/rnxopts2", "").toString());
    }
    ui->cBPositionOption1->setChecked(settings.value("prcopt/posopt1", 0).toInt());
    ui->cBPositionOption2->setChecked(settings.value("prcopt/posopt2", 0).toInt());
    ui->cBPositionOption3->setChecked(settings.value("prcopt/posopt3", 0).toInt());
    ui->cBPositionOption4->setChecked(settings.value("prcopt/posopt4", 0).toInt());
    ui->cBPositionOption5->setChecked(settings.value("prcopt/posopt5", 0).toInt());
    ui->cBPositionOption6->setChecked(settings.value("prcopt/posopt6", 0).toInt());
    ui->cBSyncSolution->setCurrentIndex(settings.value("prcopt/syncsol", 0).toInt());
    // odisp
    // freqopt
    if (options == PostOptions)
        ui->lEPPPOptions->setText(settings.value("opt/pppopts", "").toString());

    // solution options
    ui->cBSolutionFormat->setCurrentIndex(settings.value("solopt/posf", 0).toInt());
    ui->cBTimeFormat->setCurrentIndex(settings.value("solopt/timef", 1).toInt() == 0 ? 0 : settings.value("solopt/times", 0).toInt() + 1);
    ui->sBTimeDecimal->setValue(settings.value("solopt/timeu", 3).toInt());
    ui->cBLatLonFormat->setCurrentIndex(settings.value("solopt/degf", 0).toInt());
    ui->cBOutputHeader->setCurrentIndex(settings.value("solopt/outhead", 0).toInt());
    ui->cBOutputOptions->setCurrentIndex(settings.value("solopt/outopt", 0).toInt());
    ui->cBOutputVelocity->setCurrentIndex(settings.value("solopt/outvel", 0).toInt());
    ui->cBOutputDatum->setCurrentIndex(settings.value("solopt/datum", 0).toInt());
    ui->cBOutputHeight->setCurrentIndex(settings.value("solopt/height", 0).toInt());
    ui->cBOutputGeoid->setCurrentIndex(settings.value("solopt/geoid", 0).toInt());
    ui->cBSolutionStatic->setCurrentIndex(settings.value("solopt/solstatic", 0).toInt());
    ui->cBDebugStatus->setCurrentIndex(settings.value("setting/debugstatus", 0).toInt());
    ui->cBDebugTrace->setCurrentIndex(settings.value("setting/debugtrace", 0).toInt());
    ui->sBNmeaInterval1->setValue(settings.value("solopt/nmeaintv1", 0.0).toDouble());
    ui->sBNmeaInterval2->setValue(settings.value("solopt/nmeaintv2", 0.0).toDouble());
    ui->lEFieldSeperator->setText(settings.value("solopt/sep", " ").toString());
    ui->sBMaxSolutionStd->setValue(settings.value("solopt/maxsolstd", 0).toInt());

    // settings
    ui->lEStationPositionFile->setText(settings.value("setting/staposfile", "").toString());
    ui->lEGeoidDataFile->setText(settings.value("setting/geoiddatafile", "").toString());
    ui->lEIonosphereFile->setText(settings.value("setting/ionosphereFile", "").toString());
    ui->lEDCBFile->setText(settings.value("setting/dcbfile", "").toString());
    ui->lEEOPFile->setText(settings.value("setting/eopfile", "").toString());
    ui->lEBLQFile->setText(settings.value("setting/blqfile", "").toString());
    if (options == NaviOptions)
        ui->lELocalDirectory->setText(settings.value("setting/localdirectory", QDir::tempPath()).toString());

    if (options == NaviOptions) {
        ui->sBServerCycle->setValue(settings.value("setting/svrcycle", 10).toInt());
        ui->sBTimeoutTime->setValue(settings.value("setting/timeouttime", 10000).toInt());
        ui->sBReconnectTime->setValue(settings.value("setting/recontime", 10000).toInt());
        ui->sBNmeaCycle->setValue(settings.value("setting/nmeacycle", 5000).toInt());
        ui->sBFileSwapMargin->setValue(settings.value("setting/fswapmargin", 30).toInt());
        ui->sBServerBufferSize->setValue(settings.value("setting/svrbuffsize", 32768).toInt());
        ui->sBSolutionBufferSize->setValue(settings.value("setting/solbuffsize", 1000).toInt());
        ui->sBSavedSolution->setValue(settings.value("setting/savedsol", 100).toInt());
        ui->cBNavSelect->setCurrentIndex(settings.value("setting/navselect", 0).toInt());
        ui->lEProxyAddress->setText(settings.value("setting/proxyaddr", "").toString());
        ui->sBMonitorPort->setValue(settings.value("setting/moniport", DEFAULTPORT).toInt());
        ui->cBPanelStack->setCurrentIndex(settings.value("setting/panelstack", 0).toInt());

        QFont panelFnt, positionFnt;
        panelFnt.setFamily(settings.value("setting/panelfontname", POSFONTNAME).toString());
        panelFnt.setPointSize(settings.value("setting/panelfontsize", POSFONTSIZE).toInt());
        panelFontColor = QColor(static_cast<QRgb>(settings.value("setting/panelfontcolor", static_cast<int>(Qt::black)).toInt()));  // nowhere used
        if (settings.value("setting/panelfontbold", 0).toInt()) panelFnt.setBold(true);
        if (settings.value("setting/panelfontitalic", 0).toInt()) panelFnt.setItalic(true); ;
        positionFnt.setFamily(settings.value("setting/posfontname", POSFONTNAME).toString());
        positionFnt.setPointSize(settings.value("setting/posfontsize", POSFONTSIZE).toInt());
        positionFontColor = QColor(static_cast<QRgb>(settings.value("setting/posfontcolor", static_cast<int>(Qt::black)).toInt()));
        if (settings.value("setting/posfontbold", 0).toInt()) positionFnt.setBold(true);
        if (settings.value("setting/posfontitalic", 0).toInt()) positionFnt.setItalic(true);

        ui->fontLabelPanel->setFont(panelFnt);
        ui->fontLabelPanel->setText(tr("%1 %2 pt").arg(ui->fontLabelPanel->font().family()).arg(ui->fontLabelPanel->font().pointSize()));
        setWidgetTextColor(ui->fontLabelPanel, panelFontColor);
        ui->fontLabelSolution->setFont(positionFnt);
        ui->fontLabelSolution->setText(tr("%1 %2 pt").arg(ui->fontLabelSolution->font().family()).arg(ui->fontLabelSolution->font().pointSize()));
        setWidgetTextColor(ui->fontLabelSolution, positionFontColor);
    } else if (options == PostOptions) {
        QString rovList;
        for (int i = 0; i < 10; i++) {
            rovList += settings.value(QStringLiteral("opt/rovlist%1").arg(i + 1), "").toString();
        }
        QString refList;
        for (int i = 0; i < 10; i++) {
            refList += settings.value(QStringLiteral("opt/baselist%1").arg(i + 1), "").toString();
        }
        rovList.replace("@@", "\n");
        refList.replace("@@", "\n");

        ui->tERoverList->setPlainText(rovList);
        ui->tEBaseList->setPlainText(refList);
    }

    updateOptions();
    updateEnable();
}
//---------------------------------------------------------------------------
void OptDialog::updateEnable()
{
    bool rel = PMODE_DGPS <= ui->cBPositionMode->currentIndex() && ui->cBPositionMode->currentIndex() <= PMODE_FIXED;
    bool rtk = PMODE_KINEMA <= ui->cBPositionMode->currentIndex() && ui->cBPositionMode->currentIndex() <= PMODE_FIXED;
    bool ppp = ui->cBPositionMode->currentIndex() >= PMODE_PPP_KINEMA;
    bool ar = rtk || ppp;

    // processing options
    ui->cBSolution->setEnabled(options == PostOptions ? rel || ppp : false);
    ui->cBFrequencies->setEnabled(rel || ppp);
    ui->cBPositionOption1->setEnabled(ui->cBPositionMode->currentIndex() != PMODE_SINGLE);
    ui->cBPositionOption2->setEnabled(ui->cBPositionMode->currentIndex() != PMODE_SINGLE);
    ui->cBPositionOption3->setEnabled(ppp);
    ui->cBPositionOption4->setEnabled(ppp);
    ui->cBPositionOption5->setEnabled(true);
    ui->cBPositionOption6->setEnabled(ppp);

    ui->cBAmbiguityResolutionGPS->setEnabled(ar);
    ui->cBAmbiguityResolutionGLO->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() > 0 && ui->cBNavSys2->isChecked());
    ui->cBAmbiguityResolutionBDS->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() > 0 && ui->cBNavSys6->isChecked());
    ui->cBARFilter->setEnabled(ar || ppp);
    ui->sBOutageCountResetAmbiguity->setEnabled(ar || ppp);
    ui->sBLockCountFixAmbiguity->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() >= 1);
    ui->sBMinFixSats->setEnabled(ar || ppp);
    ui->sBMinHoldSats->setEnabled(ar || ppp);
    ui->sBMinDropSats->setEnabled(ar || ppp);
    ui->sBFixCountHoldAmbiguity->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() == 3);
    //sBARIter->setEnabled(ppp);

    ui->cBDynamicModel->setEnabled(ui->cBPositionMode->currentIndex() == PMODE_KINEMA ||
                                   ui->cBPositionMode->currentIndex() == PMODE_PPP_KINEMA);
    ui->cBTideCorrection->setEnabled(options != NaviOptions && ui->cBPositionMode->currentIndex() != PMODE_SINGLE);
    if (options == NaviOptions)
        setComboBoxItemEnabled(ui->cBTideCorrection, 2, false);  // OTL option is not available in RtkNavi

    setComboBoxItemEnabled(ui->cBIonosphereOption, IONOOPT_EST, rel);
    setComboBoxItemEnabled(ui->cBTroposphereOption, TROPOPT_EST, ui->cBPositionMode->currentIndex() != PMODE_SINGLE);
    setComboBoxItemEnabled(ui->cBTroposphereOption, TROPOPT_ESTG, ui->cBPositionMode->currentIndex() != PMODE_SINGLE);
    setComboBoxItemEnabled(ui->cBSatelliteEphemeris, EPHOPT_PREC, options != NaviOptions);

    ui->sBNumIteration->setEnabled(rel || ppp);

    ui->sBValidThresAR->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() >= 1 && ui->cBAmbiguityResolutionGPS->currentIndex() < 4);
    ui->sBMaxPositionVarAR->setEnabled(ar && !ppp);
    ui->sBGlonassHwBias->setEnabled(ar && ui->cBAmbiguityResolutionGLO->currentIndex() == 2);
    ui->sBValidThresARMin->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() >= 1 && ui->cBAmbiguityResolutionGPS->currentIndex() < 4);
    ui->sBValidThresARMax->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() >= 1 && ui->cBAmbiguityResolutionGPS->currentIndex() < 4);

    ui->sBElevationMaskAR->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() >= 1);
    ui->sBElevationMaskHold->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() == 3);
    ui->sBSlipThreshold->setEnabled(ar || ppp);
    ui->sBDopplerThreshold->setEnabled(ar || ppp);
    ui->sBVarHoldAmb->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() == 3);
    ui->sBGainHoldAmb->setEnabled(ar && ui->cBAmbiguityResolutionGPS->currentIndex() == 3);
    ui->sBMaxAgeDifferences->setEnabled(rel);
    ui->sBRejectCode->setEnabled(rel || ppp);
    ui->sBRejectPhase->setEnabled(rel || ppp);
    ui->cBBaselineConstrain->setEnabled(ui->cBPositionMode->currentIndex() == PMODE_MOVEB);
    ui->sBBaselineLen->setEnabled(ui->cBBaselineConstrain->isChecked() && ui->cBPositionMode->currentIndex() == PMODE_MOVEB);
    ui->sBBaselineSig->setEnabled(ui->cBBaselineConstrain->isChecked() && ui->cBPositionMode->currentIndex() == PMODE_MOVEB);
    ui->cBRoverPositionType->setEnabled(ui->cBPositionMode->currentIndex() == PMODE_FIXED || ui->cBPositionMode->currentIndex() == PMODE_PPP_FIXED);
    ui->lERoverPosition1->setEnabled(ui->cBRoverPositionType->isEnabled() && ui->cBRoverPositionType->currentIndex() <= 2);
    ui->lERoverPosition2->setEnabled(ui->cBRoverPositionType->isEnabled() && ui->cBRoverPositionType->currentIndex() <= 2);
    ui->lERoverPosition3->setEnabled(ui->cBRoverPositionType->isEnabled() && ui->cBRoverPositionType->currentIndex() <= 2);
    ui->btnRoverPosition->setEnabled(ui->cBRoverPositionType->isEnabled() && ui->cBRoverPositionType->currentIndex() <= 2);

    ui->cBReferencePositionType->setEnabled(rel && ui->cBPositionMode->currentIndex() != PMODE_MOVEB);
    setComboBoxItemEnabled(ui->cBReferencePositionType, 3, options == NaviOptions);
    setComboBoxItemEnabled(ui->cBReferencePositionType, 4, options == PostOptions);
    setComboBoxItemEnabled(ui->cBReferencePositionType, 5, options == PostOptions);
    setComboBoxItemEnabled(ui->cBReferencePositionType, 6, options == NaviOptions);
    ui->lEReferencePosition1->setEnabled(ui->cBReferencePositionType->isEnabled() && ui->cBReferencePositionType->currentIndex() <= 2);
    ui->lEReferencePosition2->setEnabled(ui->cBReferencePositionType->isEnabled() && ui->cBReferencePositionType->currentIndex() <= 2);
    ui->lEReferencePosition3->setEnabled(ui->cBReferencePositionType->isEnabled() && ui->cBReferencePositionType->currentIndex() <= 2);
    ui->btnReferencePosition->setEnabled(ui->cBReferencePositionType->isEnabled() && ui->cBReferencePositionType->currentIndex() <= 2);
    ui->cBRoverAntennaPcv->setEnabled(rel || ppp);
    ui->cBRoverAntenna->setEnabled((rel || ppp) && ui->cBRoverAntennaPcv->isChecked());
    ui->sBRoverAntennaE->setEnabled((rel || ppp) && ui->cBRoverAntennaPcv->isChecked() && ui->cBRoverAntenna->currentText()!="*");
    ui->sBRoverAntennaN->setEnabled((rel || ppp) && ui->cBRoverAntennaPcv->isChecked() && ui->cBRoverAntenna->currentText()!="*");
    ui->sBRoverAntennaU->setEnabled((rel || ppp) && ui->cBRoverAntennaPcv->isChecked() && ui->cBRoverAntenna->currentText()!="*");
    ui->lblRoverAntennaD->setEnabled((rel || ppp) && ui->cBRoverAntennaPcv->isChecked() && ui->cBRoverAntenna->currentText()!="*");
    ui->cBReferenceAntennaPcv->setEnabled(rel);
    ui->cBReferenceAntenna->setEnabled(rel && ui->cBReferenceAntennaPcv->isChecked());
    ui->sBReferenceAntennaE->setEnabled(rel && ui->cBReferenceAntennaPcv->isChecked() && ui->cBReferenceAntenna->currentText()!="*");
    ui->sBReferenceAntennaN->setEnabled(rel && ui->cBReferenceAntennaPcv->isChecked() && ui->cBReferenceAntenna->currentText()!="*");
    ui->sBReferenceAntennaU->setEnabled(rel && ui->cBReferenceAntennaPcv->isChecked() && ui->cBReferenceAntenna->currentText()!="*");
    ui->lblReferenceAntennaD->setEnabled(rel && ui->cBReferenceAntennaPcv->isChecked() && ui->cBReferenceAntenna->currentText()!="*");
    ui->lblMaxAveEp->setVisible(ui->cBReferencePositionType->currentIndex() == 3);
    ui->sBMaxAveEp->setVisible(ui->cBReferencePositionType->currentIndex() == 3);
    ui->cBInitRestart->setVisible(ui->cBReferencePositionType->currentIndex() == 3);
    ui->cBOutputSingle->setEnabled(ui->cBPositionMode->currentIndex() != 0);
    ui->cBSyncSolution->setEnabled(options == NaviOptions ? rel || ppp : false);

    // solution options
    ui->cBTimeFormat->setEnabled(ui->cBSolutionFormat->currentIndex() < 3);
    ui->sBTimeDecimal->setEnabled(ui->cBSolutionFormat->currentIndex() < 3);
    ui->cBLatLonFormat->setEnabled(ui->cBSolutionFormat->currentIndex() == 0);
    ui->cBOutputHeader->setEnabled(ui->cBSolutionFormat->currentIndex() < 3);
    ui->cBOutputOptions->setEnabled(options == PostOptions ? ui->cBSolutionFormat->currentIndex() < 3: false);
    ui->lEFieldSeperator->setEnabled(ui->cBSolutionFormat->currentIndex() < 3);
    ui->cBOutputDatum->setEnabled(ui->cBSolutionFormat->currentIndex() == 0);
    ui->cBOutputHeight->setEnabled(ui->cBSolutionFormat->currentIndex() == 0);
    ui->cBOutputGeoid->setEnabled(ui->cBSolutionFormat->currentIndex() == 0 && ui->cBOutputHeight->currentIndex() == 1);
    ui->Label21->setEnabled(ui->cBPositionMode->currentIndex() == PMODE_STATIC ||
                            ui->cBPositionMode->currentIndex() == PMODE_PPP_STATIC);
    ui->cBSolutionStatic->setEnabled(ui->cBPositionMode->currentIndex() == PMODE_STATIC ||
                                     ui->cBPositionMode->currentIndex() == PMODE_PPP_STATIC);

    if (options == NaviOptions) {
        ui->sBSbasSatellite->setEnabled(ui->cBPositionMode->currentIndex() == 0);
        ui->lEIonosphereFile->setVisible(false);
        ui->lblIonosphereFile->setVisible(false);
        ui->lELocalDirectory->setVisible(true);
        ui->lblLocalDirectory->setVisible(true);

        ui->tWOptions->setTabVisible(6, true);
        ui->tWOptions->setTabVisible(7, false);
    } else if (options == PostOptions) {
        ui->lEIonosphereFile->setVisible(true);
        ui->lblIonosphereFile->setVisible(true);
        ui->lELocalDirectory->setVisible(false);
        ui->lblLocalDirectory->setVisible(false);

        ui->tWOptions->setTabVisible(6, false);
        ui->tWOptions->setTabVisible(7, true);
    }
}
//---------------------------------------------------------------------------
int OptDialog::getPosition(int type, QLineEdit **edit, double *pos)
{
    double p[3] = { 0 };
    bool okay;
    int ret = 0;

    if (type == 1) { /* lat/lon/height dms/m */
        auto lat = regExDMSLat.match(edit[0]->text());
        if (lat.hasMatch()) {
          double deg1 = QLocale().toDouble(lat.captured("deg1"));
          double min1 = QLocale().toDouble(lat.captured("min1"));
          double sec1 = QLocale().toDouble(lat.captured("sec1"));
          double deg2 = QLocale().toDouble(lat.captured("deg2"));
          double min2 = QLocale().toDouble(lat.captured("min2"));
          double sec2 = QLocale().toDouble(lat.captured("sec2"));
          if (fabs(fabs(deg1) - 90) < 1e-12 && fabs(min1) < 1e-12 && fabs(sec1) < 1e-12)
            p[0] = (deg1 < 0 ? -1 : 1) * fabs(deg1) * D2R;
          else
            p[0] = (deg2 < 0 ? -1 : 1) * (fabs(deg2) + min2 / 60 + sec2 / 3600) * D2R;
        } else
          p[0] = 0;

        auto lon = regExDMSLon.match(edit[1]->text());
        if (lon.hasMatch()) {
          double deg1 = QLocale().toDouble(lon.captured("deg1"));
          double min1 = QLocale().toDouble(lon.captured("min1"));
          double sec1 = QLocale().toDouble(lon.captured("sec1"));
          double deg2 = QLocale().toDouble(lon.captured("deg2"));
          double min2 = QLocale().toDouble(lon.captured("min2"));
          double sec2 = QLocale().toDouble(lon.captured("sec2"));
          if (fabs(fabs(deg1) - 180) < 1e-12 && fabs(min1) < 1e-12 && fabs(sec1) < 1e-12)
            p[1] = (deg1 < 0 ? -1 : 1) * fabs(deg1) * D2R;
          else
            p[1] = (deg2 < 0 ? -1 : 1) * (fabs(deg2) + min2 / 60 + sec2 / 3600) * D2R;
        } else
          p[1] = 0;

        auto height = stripped(edit[2]->text(), "m");
        p[2] = QLocale().toDouble(height, &okay);
        if (!okay) {ret |= 0x4; p[2] = 0;};

        pos2ecef(p, pos);
    } else if (type == 2) { /* x/y/z-ecef */
        auto x = stripped(edit[0]->text(), "m");
        auto y = stripped(edit[1]->text(), "m");
        auto z = stripped(edit[2]->text(), "m");

        pos[0] = QLocale().toDouble(x, &okay);
        if (!okay) {ret |= 0x1; pos[0] = 0;}
        pos[1] = QLocale().toDouble(y, &okay);
        if (!okay) {ret |= 0x2; pos[1] = 0;}
        pos[2] = QLocale().toDouble(z, &okay);
        if (!okay) {ret |= 0x4; pos[2] = 0;}
    } else {   /* lat/lon/hight decimal */
        auto lat = stripped(edit[0]->text(), "°");
        auto lon = stripped(edit[1]->text(), "°");
        auto height = stripped(edit[2]->text(), "m");
        
        p[0] = QLocale().toDouble(lat, &okay) * D2R;
        if (!okay) {ret |= 0x1; p[0] = 0;}
        p[1] = QLocale().toDouble(lon, &okay) * D2R;
        if (!okay) {ret |= 0x2; p[1] = 0;}
        p[2] = QLocale().toDouble(height, &okay);
        if (!okay) {ret |= 0x4; p[2] = 0;}

        pos2ecef(p, pos);
    }

    return ret;
}
//---------------------------------------------------------------------------
void OptDialog::setPosition(int type, QLineEdit **edit, double *pos)
{
    double p[3], dms1[3], dms2[3], s1, s2;

    if (type == 1) { /* lat/lon/height dms/m */
        ecef2pos(pos, p);
        s1 = p[0] < 0 ? -1 : 1;
        s2 = p[1] < 0 ? -1 : 1;
        p[0] = fabs(p[0]) * R2D + 1E-12;
        p[1] = fabs(p[1]) * R2D + 1E-12;
        dms1[0] = floor(p[0]); p[0] = (p[0] - dms1[0]) * 60.0;
        dms1[1] = floor(p[0]); dms1[2] = (p[0] - dms1[1]) * 60.0;
        dms2[0] = floor(p[1]); p[1] = (p[1] - dms2[0]) * 60.0;
        dms2[1] = floor(p[1]); dms2[2] = (p[1] - dms2[1]) * 60.0;

        edit[0]->setValidator(new QRegularExpressionValidator(regExDMSLat, this));
        edit[1]->setValidator(new QRegularExpressionValidator(regExDMSLon, this));
        edit[2]->setValidator(new DoubleUnitValidator(-100, 10000, -1, " m", this));

        edit[0]->setText(QStringLiteral("%1° %2' %3\"").arg(QLocale().toString(s1 * dms1[0], 'f', 0))
                             .arg(QLocale().toString(dms1[1], 'f', 0)).arg(QLocale().toString(dms1[2],'f', 6)));
        edit[1]->setText(QStringLiteral("%1° %2' %3\"").arg(QLocale().toString(s2 * dms2[0], 'f', 0))
                             .arg(QLocale().toString(dms2[1], 'f', 0)).arg(QLocale().toString(dms2[2], 'f', 6)));
        edit[2]->setText(QStringLiteral("%1 m").arg(QLocale().toString(p[2], 'f', 4)));

    } else if (type == 2) { /* x/y/z-ecef */
        edit[0]->setValidator(new DoubleUnitValidator(-INFINITY, INFINITY, -1, " m", this));
        edit[1]->setValidator(new DoubleUnitValidator(-INFINITY, INFINITY, -1, " m", this));
        edit[2]->setValidator(new DoubleUnitValidator(-INFINITY, INFINITY, -1, " m", this));

        edit[0]->setText(QStringLiteral("%1 m").arg(QLocale().toString(pos[0], 'f', 4)));
        edit[1]->setText(QStringLiteral("%1 m").arg(QLocale().toString(pos[1], 'f', 4)));
        edit[2]->setText(QStringLiteral("%1 m").arg(QLocale().toString(pos[2], 'f', 4)));
    } else {   /* lat/lon/hight decimal */
        edit[0]->setValidator(new DoubleUnitValidator(-90, 90, -1, "°", this));
        edit[1]->setValidator(new DoubleUnitValidator(-180, 180, -1, "°", this));
        edit[2]->setValidator(new DoubleUnitValidator(-100, 10000, -1, " m", this));

        ecef2pos(pos, p);
        edit[0]->setText(QStringLiteral("%1°").arg(QLocale().toString(p[0] * R2D, 'f', 9)));
        edit[1]->setText(QStringLiteral("%1°").arg(QLocale().toString(p[1] * R2D, 'f', 9)));
        edit[2]->setText(QStringLiteral("%1 m").arg(QLocale().toString(p[2], 'f', 4)));
    }
}
//---------------------------------------------------------------------------
void OptDialog::readAntennaList()
{
    pcvs_t pcvs = { 0, 0, 0 };
    char *p;
    QString currentRoverAntenna, currentReferenceAntenna;
    int i;

    if (!readpcv(qPrintable(ui->lEAntennaPcvFile->text()), &pcvs)) return;

    /* Save currently defined antennas */
    currentRoverAntenna = ui->cBRoverAntenna->currentText();
    currentReferenceAntenna = ui->cBReferenceAntenna->currentText();

    /* Clear and add antennas from ANTEX file */
    ui->cBRoverAntenna->clear();
    ui->cBReferenceAntenna->clear();

    ui->cBRoverAntenna->addItem(""); ui->cBReferenceAntenna->addItem("");
    ui->cBRoverAntenna->addItem("*"); ui->cBReferenceAntenna->addItem("*");

    for (int i = 0; i < pcvs.n; i++) {
        if (pcvs.pcv[i].sat) continue;
        if ((p = strchr(pcvs.pcv[i].type, ' '))) *p = '\0';
        if (i > 0 && !strcmp(pcvs.pcv[i].type, pcvs.pcv[i - 1].type)) continue;
        ui->cBRoverAntenna->addItem(pcvs.pcv[i].type);
        ui->cBReferenceAntenna->addItem(pcvs.pcv[i].type);
    }

    /* Restore previously defined antennas */
    i = ui->cBRoverAntenna->findText(currentRoverAntenna);
    ui->cBRoverAntenna->setCurrentIndex(i == -1 ? 0 : i);
    i = ui->cBReferenceAntenna->findText(currentReferenceAntenna);
    ui->cBReferenceAntenna->setCurrentIndex(i == -1 ? 0 : i);

    free(pcvs.pcv);
}
//---------------------------------------------------------------------------
void OptDialog::showKeyDialog()
{
    KeyDialog *keyDialog = new KeyDialog(this);

    keyDialog->setFlag(2);
    keyDialog->exec();

    delete keyDialog;
}

//---------------------------------------------------------------------------
void OptDialog::showFrequenciesDialog()
{
    FreqDialog *freqDialog = new FreqDialog(this);

    freqDialog->exec();

    delete freqDialog;
}
//---------------------------------------------------------------------------
QString OptDialog::excludedSatellitesString(prcopt_t *prcopt)
{
    QString buff;
    char id[32];
    int sat;

    buff = "";
    for (sat = 1; sat <= MAXSAT; sat++) {
        if (!prcopt->exsats[sat - 1]) continue;
        satno2id(sat, id);
        buff += QStringLiteral("%1%2%3").arg(buff.isEmpty() ? "" : " ").arg(prcopt->exsats[sat - 1] == 2 ? "+" : "").arg(id);
    }

    return buff;
}
//---------------------------------------------------------------------------
bool OptDialog::fillExcludedSatellites(prcopt_t *prcopt, const QString &excludedSatellites)
{
    if (!excludedSatellites.isEmpty()) {
        foreach (QString sat, excludedSatellites.split(' ')) {
            unsigned char ex;
            int satNo;
            if (sat[0] == '+')
            {
                ex = 2;
                sat = sat.mid(1);
            } else ex = 1;
            if (!(satNo = satid2no(qPrintable(sat)))) continue;
            prcopt->exsats[satNo - 1] = ex;
        }
        return true;
    }
    return false;
};
//---------------------------------------------------------------------------
QString OptDialog::stripped(const QString input, const QString suffix) const
{
    QString text = input;
    int size;

    if (suffix.size() && text.endsWith(suffix)) {
        size = text.size() - suffix.size();
        text = text.mid(0, size).trimmed();
    }

    return text;
}
//---------------------------------------------------------------------------
void OptDialog::checkLineEditValidator()
{
    QLineEdit *edit = (QLineEdit*)this->sender();

    const QValidator *validator = edit->validator();

    if (!validator) return;

    QString text = edit->text();
    int pos = edit->cursorPosition();
    setWidgetTextColor(edit, validator->validate(text, pos) == QValidator::Acceptable ? QColor() : QColor(Qt::red));

    edit->setText(text);
    edit->setCursorPosition(pos);
}
