object OptDialog: TOptDialog
  Left = 0
  Top = 0
  BorderIcons = [biSystemMenu]
  BorderStyle = bsDialog
  Caption = 'Options'
  ClientHeight = 384
  ClientWidth = 413
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  Position = poMainFormCenter
  OnShow = FormShow
  TextHeight = 13
  object BtnLoad: TButton
    Left = 2
    Top = 355
    Width = 100
    Height = 28
    Caption = '&Load...'
    TabOrder = 2
    OnClick = BtnLoadClick
  end
  object BtnSave: TButton
    Left = 104
    Top = 355
    Width = 100
    Height = 28
    Caption = '&Save'
    TabOrder = 3
    OnClick = BtnSaveClick
  end
  object BtnOk: TButton
    Left = 206
    Top = 355
    Width = 100
    Height = 28
    Caption = '&OK'
    ModalResult = 1
    TabOrder = 0
    OnClick = BtnOkClick
  end
  object BtnCancel: TButton
    Left = 308
    Top = 355
    Width = 100
    Height = 28
    Caption = '&Cancel'
    ModalResult = 2
    TabOrder = 1
  end
  object Options: TPageControl
    Left = 0
    Top = 0
    Width = 413
    Height = 353
    ActivePage = TabSheet1
    Align = alTop
    TabOrder = 4
    ExplicitWidth = 407
    object TabSheet1: TTabSheet
      Caption = 'Setting&1'
      object LabelPosMode: TLabel
        Left = 24
        Top = 7
        Width = 80
        Height = 13
        Caption = 'Positioning Mode'
      end
      object LabelFreq: TLabel
        Left = 24
        Top = 29
        Width = 58
        Height = 13
        Caption = 'Frequencies'
      end
      object LabelElMask: TLabel
        Left = 24
        Top = 51
        Width = 179
        Height = 13
        Caption = 'Elevation Mask ('#176') / SNR Mask (dBHz)'
      end
      object Label3: TLabel
        Left = 24
        Top = 139
        Width = 183
        Height = 13
        Caption = 'Rec Dynamics / Earth Tides Correction'
      end
      object Label8: TLabel
        Left = 24
        Top = 161
        Width = 108
        Height = 13
        Caption = 'Ionosphere Correction'
      end
      object Label9: TLabel
        Left = 24
        Top = 183
        Width = 114
        Height = 13
        Caption = 'Troposphere Correction'
      end
      object Label32: TLabel
        Left = 24
        Top = 205
        Width = 119
        Height = 13
        Caption = 'Satellite Ephemeris/Clock'
      end
      object Label35: TLabel
        Left = 24
        Top = 249
        Width = 176
        Height = 13
        Caption = 'Excluded Satellites (+PRN: Included)'
      end
      object PosMode: TComboBox
        Left = 248
        Top = 4
        Width = 152
        Height = 21
        Style = csDropDownList
        DropDownCount = 10
        ItemIndex = 2
        TabOrder = 0
        Text = 'Kinematic'
        OnChange = PosModeChange
        Items.Strings = (
          'Single'
          'DGPS/DGNSS'
          'Kinematic'
          'Static'
          'Static-Start'
          'Moving-Base'
          'Fixed'
          'PPP Kinematic'
          'PPP Static'
          'PPP Fixed')
      end
      object BtnFreq: TSpeedButton
        Left = 102
        Top = 26
        Width = 18
        Height = 22
        Caption = '?'
        Flat = True
        Spacing = 0
        OnClick = BtnFreqClick
      end
      object Freq: TComboBox
        Left = 248
        Top = 26
        Width = 152
        Height = 21
        Style = csDropDownList
        TabOrder = 1
        OnChange = FreqChange
      end
      object ElMask: TComboBox
        Left = 248
        Top = 48
        Width = 75
        Height = 21
        AutoComplete = False
        DropDownCount = 16
        TabOrder = 2
        Text = '15'
        Items.Strings = (
          '0'
          '5'
          '10'
          '15'
          '20'
          '25'
          '30'
          '35'
          '40'
          '45'
          '50'
          '55'
          '60'
          '65'
          '70')
      end
      object BtnSnrMask: TButton
        Left = 323
        Top = 48
        Width = 77
        Height = 21
        Caption = '...'
        TabOrder = 3
        OnClick = BtnSnrMaskClick
      end
      object DynamicModel: TComboBox
        Left = 248
        Top = 136
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 4
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object TideCorr: TComboBox
        Left = 325
        Top = 136
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 5
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'Solid')
      end
      object IonoOpt: TComboBox
        Left = 248
        Top = 158
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 6
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'Broadcast'
          'SBAS'
          'Iono-Free LC'
          'Estimate STEC'
          'IONEX TEC'
          'QZSS Broadcast')
      end
      object TropOpt: TComboBox
        Left = 248
        Top = 180
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 7
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'Saastamoinen'
          'SBAS'
          'Estimate ZTD'
          'Estimate ZTD+Grad')
      end
      object SatEphem: TComboBox
        Left = 248
        Top = 202
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 8
        Text = 'Broadcast'
        Items.Strings = (
          'Broadcast'
          'Precise'
          'Broadcast+SBAS'
          'Broadcast+SSR APC'
          'Broadcast+SSR CoM'
          'QZSS LEX')
      end
      object PosOpt1: TCheckBox
        Left = 24
        Top = 224
        Width = 61
        Height = 19
        Caption = 'Sat PCV'
        TabOrder = 9
      end
      object PosOpt2: TCheckBox
        Left = 84
        Top = 224
        Width = 62
        Height = 19
        Caption = 'Rec PCV'
        TabOrder = 10
      end
      object PosOpt3: TCheckBox
        Left = 146
        Top = 224
        Width = 69
        Height = 19
        Caption = 'PhWU'
        TabOrder = 11
      end
      object PosOpt4: TCheckBox
        Left = 197
        Top = 224
        Width = 68
        Height = 19
        Caption = 'Rej Ecl'
        TabOrder = 12
      end
      object PosOpt5: TCheckBox
        Left = 254
        Top = 224
        Width = 68
        Height = 19
        Caption = 'RAIM FDE'
        TabOrder = 13
      end
      object PosOpt6: TCheckBox
        Left = 326
        Top = 224
        Width = 68
        Height = 19
        Caption = 'DBCorr'
        TabOrder = 14
      end
      object ExSatsE: TEdit
        Left = 221
        Top = 244
        Width = 179
        Height = 19
        TabOrder = 15
      end
      object NavSys1: TCheckBox
        Left = 24
        Top = 268
        Width = 49
        Height = 19
        Caption = 'GPS'
        Checked = True
        State = cbChecked
        TabOrder = 16
      end
      object NavSys2: TCheckBox
        Left = 65
        Top = 268
        Width = 71
        Height = 19
        Caption = 'GLONASS'
        TabOrder = 17
        OnClick = NavSys2Click
      end
      object NavSys3: TCheckBox
        Left = 134
        Top = 268
        Width = 61
        Height = 19
        Caption = 'Galileo'
        TabOrder = 18
      end
      object NavSys4: TCheckBox
        Left = 190
        Top = 268
        Width = 45
        Height = 19
        Caption = 'QZSS'
        TabOrder = 19
      end
      object NavSys6: TCheckBox
        Left = 241
        Top = 268
        Width = 69
        Height = 19
        Caption = 'BDS'
        TabOrder = 20
        OnClick = NavSys6Click
      end
      object NavSys7: TCheckBox
        Left = 284
        Top = 268
        Width = 48
        Height = 19
        Caption = 'NavIC'
        TabOrder = 21
        OnClick = NavSys6Click
      end
      object NavSys5: TCheckBox
        Left = 338
        Top = 268
        Width = 51
        Height = 19
        Caption = 'SBAS'
        TabOrder = 22
      end
    end
    object TabSheet2: TTabSheet
      Caption = 'Setting&2'
      ImageIndex = 1
      object Label25: TLabel
        Left = 24
        Top = 7
        Width = 184
        Height = 13
        Caption = 'Integer Ambiguity Res (GPS/GLO/BDS)'
      end
      object Label24: TLabel
        Left = 24
        Top = 29
        Width = 181
        Height = 13
        Caption = 'Ratio to Fix Ambiguity (Min/Nom/Max)'
      end
      object Label15: TLabel
        Left = 24
        Top = 51
        Width = 62
        Height = 13
        Caption = 'GLO HW Bias'
      end
      object Label13: TLabel
        Left = 24
        Top = 73
        Width = 164
        Height = 13
        Caption = 'Min Lock / Elevation ('#176') to Fix Amb'
      end
      object LabelHold: TLabel
        Left = 24
        Top = 95
        Width = 164
        Height = 13
        Caption = 'Min Fix / Elevation ('#176') to Hold Amb'
      end
      object Label22: TLabel
        Left = 24
        Top = 117
        Width = 206
        Height = 13
        Caption = 'Slip Threshs: Doppler (Hz) / Geom-Free (m)'
      end
      object Label14: TLabel
        Left = 24
        Top = 139
        Width = 192
        Height = 13
        Caption = 'Max Age of Diff (s) / Outs to Reset Amb'
      end
      object Label11: TLabel
        Left = 24
        Top = 161
        Width = 179
        Height = 13
        Caption = 'Outlier Threshold for Code/Phase (m)'
      end
      object Label37: TLabel
        Left = 24
        Top = 183
        Width = 143
        Height = 13
        Caption = '# of Filter Iter / Sync Solution'
      end
      object Label54: TLabel
        Left = 24
        Top = 227
        Width = 131
        Height = 13
        Caption = 'Min Fix Sats / Min Hold Sats'
      end
      object Label51: TLabel
        Left = 24
        Top = 249
        Width = 66
        Height = 13
        Caption = 'Min Drop Sats'
      end
      object Label50: TLabel
        Left = 24
        Top = 271
        Width = 144
        Height = 13
        Caption = 'Max Pos Var for AR / AR Filter'
      end
      object Label52: TLabel
        Left = 24
        Top = 293
        Width = 143
        Height = 13
        Caption = 'Hold Amb Var / Hold Amb Gain'
      end
      object AmbRes: TComboBox
        Left = 248
        Top = 4
        Width = 52
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 0
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'Continuous'
          'Instantaneous'
          'Fix and Hold')
      end
      object GloAmbRes: TComboBox
        Left = 302
        Top = 4
        Width = 52
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 1
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'ON'
          'Auto Calibration'
          'Fix and Hold')
      end
      object BdsAmbRes: TComboBox
        Left = 356
        Top = 4
        Width = 45
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 2
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'ON')
      end
      object ValidThresARMin: TEdit
        Left = 248
        Top = 26
        Width = 50
        Height = 21
        TabOrder = 3
        Text = '3.0'
      end
      object ValidThresAR: TEdit
        Left = 300
        Top = 26
        Width = 49
        Height = 21
        TabOrder = 4
        Text = '3.0'
      end
      object ValidThresARMax: TEdit
        Left = 351
        Top = 26
        Width = 50
        Height = 21
        TabOrder = 5
        Text = '3.0'
      end
      object GloHwBias: TEdit
        Left = 248
        Top = 48
        Width = 75
        Height = 21
        ParentShowHint = False
        ShowHint = True
        TabOrder = 6
        Text = '0.0'
      end
      object LockCntFixAmb: TEdit
        Left = 248
        Top = 70
        Width = 75
        Height = 21
        TabOrder = 7
        Text = '0'
      end
      object ElMaskAR: TEdit
        Left = 325
        Top = 70
        Width = 75
        Height = 21
        TabOrder = 8
        Text = '0'
      end
      object FixCntHoldAmb: TEdit
        Left = 248
        Top = 92
        Width = 75
        Height = 21
        TabOrder = 9
        Text = '10'
      end
      object ElMaskHold: TEdit
        Left = 325
        Top = 92
        Width = 75
        Height = 21
        TabOrder = 10
        Text = '10'
      end
      object DopThres: TEdit
        Left = 248
        Top = 114
        Width = 75
        Height = 21
        TabOrder = 11
        Text = '0.0'
      end
      object SlipThres: TEdit
        Left = 325
        Top = 114
        Width = 75
        Height = 21
        TabOrder = 12
        Text = '0.05'
      end
      object MaxAgeDiff: TEdit
        Left = 248
        Top = 136
        Width = 75
        Height = 21
        TabOrder = 13
        Text = '30'
      end
      object OutCntResetAmb: TEdit
        Left = 325
        Top = 136
        Width = 75
        Height = 21
        TabOrder = 14
        Text = '5'
      end
      object RejectCode: TEdit
        Left = 248
        Top = 158
        Width = 75
        Height = 21
        TabOrder = 15
        Text = '30'
      end
      object RejectPhase: TEdit
        Left = 325
        Top = 158
        Width = 75
        Height = 21
        TabOrder = 16
        Text = '30'
      end
      object ARIter: TEdit
        Left = 248
        Top = 180
        Width = 75
        Height = 21
        TabOrder = 17
        Text = '1'
      end
      object NumIter: TEdit
        Left = 248
        Top = 180
        Width = 75
        Height = 21
        TabOrder = 18
        Text = '1'
      end
      object SyncSol: TComboBox
        Left = 325
        Top = 180
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 19
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'ON')
      end
      object BaselineConst: TCheckBox
        Left = 24
        Top = 202
        Width = 181
        Height = 19
        Caption = 'Baseline Length Constraint (m)'
        TabOrder = 20
        OnClick = BaselineConstClick
      end
      object BaselineLen: TEdit
        Left = 248
        Top = 202
        Width = 75
        Height = 21
        TabOrder = 21
        Text = '0.000'
      end
      object BaselineSig: TEdit
        Left = 325
        Top = 202
        Width = 75
        Height = 21
        TabOrder = 22
        Text = '0.000'
      end
      object MinFixSats: TEdit
        Left = 248
        Top = 224
        Width = 75
        Height = 21
        TabOrder = 23
        Text = '3'
      end
      object MinHoldSats: TEdit
        Left = 325
        Top = 224
        Width = 75
        Height = 21
        TabOrder = 24
        Text = '5'
      end
      object MinDropSats: TEdit
        Left = 248
        Top = 246
        Width = 75
        Height = 21
        TabOrder = 25
        Text = '20'
      end
      object MaxPosVarAR: TEdit
        Left = 248
        Top = 268
        Width = 75
        Height = 21
        TabOrder = 26
        Text = '0.1'
      end
      object ARFilter: TComboBox
        Left = 325
        Top = 268
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 27
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'ON')
      end
      object VarHoldAmb: TEdit
        Left = 248
        Top = 290
        Width = 75
        Height = 21
        TabOrder = 28
        Text = '0.001'
      end
      object GainHoldAmb: TEdit
        Left = 325
        Top = 290
        Width = 75
        Height = 21
        TabOrder = 29
        Text = '0.01'
      end
    end
    object TabSheet3: TTabSheet
      Caption = 'O&utput'
      ImageIndex = 2
      object LabelSolFormat: TLabel
        Left = 24
        Top = 7
        Width = 75
        Height = 13
        Caption = 'Solution Format'
      end
      object Label20: TLabel
        Left = 24
        Top = 29
        Width = 194
        Height = 13
        Caption = 'Output Header / Proc. Options / Velocity'
      end
      object LabelTimeFormat: TLabel
        Left = 24
        Top = 51
        Width = 134
        Height = 13
        Caption = 'Time Format / # of Decimals'
      end
      object LabelLatLonFormat: TLabel
        Left = 24
        Top = 73
        Width = 209
        Height = 13
        Caption = 'Latitude Longitude Format / Field Separator'
      end
      object LabelFieldSep: TLabel
        Left = 24
        Top = 95
        Width = 215
        Height = 13
        Caption = 'Output Single if Sol Outage / Max Sol Std (m)'
      end
      object Label2: TLabel
        Left = 24
        Top = 117
        Width = 72
        Height = 13
        Caption = 'Datum / Height'
      end
      object Label18: TLabel
        Left = 24
        Top = 139
        Width = 58
        Height = 13
        Caption = 'Geoid Model'
      end
      object Label21: TLabel
        Left = 24
        Top = 161
        Width = 114
        Height = 13
        Caption = 'Solution for Static Mode'
        Enabled = False
      end
      object Label17: TLabel
        Left = 24
        Top = 183
        Width = 185
        Height = 13
        Caption = 'NMEA Interval (s) RMC/GGA, GSA/GSV'
      end
      object Label36: TLabel
        Left = 24
        Top = 205
        Width = 217
        Height = 13
        Caption = 'Output Solution Status / Output Debug Trace'
      end
      object SolFormat: TComboBox
        Left = 248
        Top = 4
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 0
        Text = 'Lat/Lon/Height'
        OnChange = SolFormatChange
        Items.Strings = (
          'Lat/Lon/Height'
          'X/Y/Z-ECEF'
          'E/N/U-Baseline'
          'NMEA0183')
      end
      object OutputHead: TComboBox
        Left = 248
        Top = 26
        Width = 50
        Height = 21
        Style = csDropDownList
        ItemIndex = 1
        TabOrder = 1
        Text = 'ON'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object OutputOpt: TComboBox
        Left = 300
        Top = 26
        Width = 49
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 2
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object OutputVel: TComboBox
        Left = 351
        Top = 26
        Width = 49
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 3
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object TimeFormat: TComboBox
        Left = 248
        Top = 48
        Width = 118
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 4
        Text = 'ww ssss GPST'
        Items.Strings = (
          'ww ssss GPST'
          'hh:mm:ss GPST'
          'hh:mm:ss UTC'
          'hh:mm:ss JST')
      end
      object TimeDecimal: TEdit
        Left = 368
        Top = 48
        Width = 32
        Height = 21
        TabOrder = 5
        Text = '3'
      end
      object LatLonFormat: TComboBox
        Left = 248
        Top = 70
        Width = 118
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 6
        Text = 'ddd.ddddddddd'
        Items.Strings = (
          'ddd.ddddddddd'
          'ddd mm ss.sss')
      end
      object FieldSep: TEdit
        Left = 368
        Top = 70
        Width = 32
        Height = 21
        TabOrder = 7
      end
      object OutputSingle: TComboBox
        Left = 248
        Top = 92
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 8
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object MaxSolStd: TEdit
        Left = 325
        Top = 92
        Width = 75
        Height = 21
        TabOrder = 9
        Text = '0'
      end
      object OutputDatum: TComboBox
        Left = 248
        Top = 114
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 10
        Text = 'WGS84'
        Items.Strings = (
          'WGS84')
      end
      object OutputHeight: TComboBox
        Left = 325
        Top = 114
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 11
        Text = 'Ellipsoidal'
        OnClick = OutputHeightClick
        Items.Strings = (
          'Ellipsoidal'
          'Geodetic')
      end
      object OutputGeoid: TComboBox
        Left = 248
        Top = 136
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 12
        Text = 'Internal'
        Items.Strings = (
          'Internal'
          'EGM96-BE (15")'
          'EGM2008-SE (2.5")'
          'EGM2008-SE (1")'
          'GSI2000 (1x1.5")')
      end
      object SolStatic: TComboBox
        Left = 248
        Top = 158
        Width = 152
        Height = 21
        Style = csDropDownList
        Enabled = False
        ItemIndex = 0
        TabOrder = 13
        Text = 'All'
        Items.Strings = (
          'All'
          'Single')
      end
      object NmeaIntv1: TEdit
        Left = 248
        Top = 180
        Width = 75
        Height = 21
        TabOrder = 14
        Text = '0'
      end
      object NmeaIntv2: TEdit
        Left = 325
        Top = 180
        Width = 75
        Height = 21
        TabOrder = 15
        Text = '1'
      end
      object DebugStatus: TComboBox
        Left = 248
        Top = 202
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 16
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'States'
          'Residuals')
      end
      object DebugTrace: TComboBox
        Left = 325
        Top = 202
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 17
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'Level 1'
          'Level 2'
          'Level 3'
          'Level 4'
          'Level 5')
      end
    end
    object TabSheet4: TTabSheet
      Caption = 'S&tatistics'
      ImageIndex = 3
      object GroupBox3: TGroupBox
        Left = 3
        Top = 0
        Width = 397
        Height = 153
        Caption = 'Measurement Errors (1-sigma)'
        TabOrder = 0
        object Label6: TLabel
          Left = 33
          Top = 16
          Width = 194
          Height = 13
          Caption = 'Code/Phase Error Ratio L1/L2/L5/L6'
        end
        object Label7: TLabel
          Left = 33
          Top = 38
          Width = 160
          Height = 13
          Caption = 'Carrier-Phase Error a+b/sinEl (m)'
        end
        object Label16: TLabel
          Left = 33
          Top = 60
          Width = 184
          Height = 13
          Caption = 'Carrier-Phase Error/Baseline (m/10km)'
        end
        object Label12: TLabel
          Left = 33
          Top = 82
          Width = 185
          Height = 13
          Caption = 'Carrier Phase Error: SNR / SNR maxDb'
        end
        object Label10: TLabel
          Left = 33
          Top = 104
          Width = 140
          Height = 13
          Caption = 'Carrier-Phase Error: Rcv Errs'
        end
        object Label53: TLabel
          Left = 33
          Top = 126
          Width = 112
          Height = 13
          Caption = 'Doppler Freq Error (Hz)'
        end
        object MeasErrR1: TEdit
          Left = 242
          Top = 13
          Width = 35
          Height = 21
          TabOrder = 0
          Text = '100.0'
        end
        object MeasErrR2: TEdit
          Left = 280
          Top = 13
          Width = 35
          Height = 21
          TabOrder = 1
          Text = '100.0'
        end
        object MeasErrR5: TEdit
          Left = 318
          Top = 13
          Width = 35
          Height = 21
          TabOrder = 2
          Text = '100.0'
        end
        object MeasErrR6: TEdit
          Left = 356
          Top = 13
          Width = 35
          Height = 21
          TabOrder = 3
          Text = '100.0'
        end
        object MeasErr2: TEdit
          Left = 242
          Top = 35
          Width = 73
          Height = 21
          TabOrder = 4
          Text = '0.003'
        end
        object MeasErr3: TEdit
          Left = 318
          Top = 35
          Width = 73
          Height = 21
          TabOrder = 5
          Text = '0.003'
        end
        object MeasErr4: TEdit
          Left = 242
          Top = 57
          Width = 73
          Height = 21
          TabOrder = 6
          Text = '0.000'
        end
        object MeasErr7: TEdit
          Left = 242
          Top = 79
          Width = 73
          Height = 21
          TabOrder = 7
          Text = '0'
        end
        object MeasErr6: TEdit
          Left = 318
          Top = 79
          Width = 73
          Height = 21
          TabOrder = 8
          Text = '52'
        end
        object MeasErr8: TEdit
          Left = 242
          Top = 101
          Width = 73
          Height = 21
          TabOrder = 9
          Text = '0'
        end
        object MeasErr5: TEdit
          Left = 242
          Top = 123
          Width = 73
          Height = 21
          TabOrder = 10
          Text = '0'
        end
      end
      object GroupBox4: TGroupBox
        Left = 3
        Top = 156
        Width = 396
        Height = 105
        Caption = 'Process Noises (1-sigma/sqrt(s))'
        TabOrder = 1
        object Label33: TLabel
          Left = 33
          Top = 16
          Width = 170
          Height = 13
          Caption = 'Receiver Accel Horiz/Vertical (m/s2)'
        end
        object Label26: TLabel
          Left = 33
          Top = 38
          Width = 123
          Height = 13
          Caption = 'Carrier-Phase Bias (cycle)'
        end
        object Label27: TLabel
          Left = 33
          Top = 60
          Width = 172
          Height = 13
          Caption = 'Vertical Ionospheric Delay (m/10km)'
        end
        object Label28: TLabel
          Left = 33
          Top = 82
          Width = 144
          Height = 13
          Caption = 'Zenith Tropospheric Delay (m)'
        end
        object PrNoise4: TEdit
          Left = 242
          Top = 13
          Width = 73
          Height = 21
          TabOrder = 0
          Text = '1.0E-04'
        end
        object PrNoise5: TEdit
          Left = 318
          Top = 13
          Width = 73
          Height = 21
          TabOrder = 1
          Text = '1.0E-04'
        end
        object PrNoise1: TEdit
          Left = 242
          Top = 35
          Width = 149
          Height = 21
          TabOrder = 2
          Text = '1.0E-04'
        end
        object PrNoise2: TEdit
          Left = 242
          Top = 57
          Width = 149
          Height = 21
          TabOrder = 3
          Text = '1.0E-03'
        end
        object PrNoise3: TEdit
          Left = 242
          Top = 79
          Width = 149
          Height = 21
          TabOrder = 4
          Text = '1.0E-04'
        end
      end
      object Label29: TLabel
        Left = 37
        Top = 274
        Width = 132
        Height = 13
        Caption = 'Satellite Clock Stability (s/s)'
      end
      object SatClkStab: TEdit
        Left = 245
        Top = 267
        Width = 149
        Height = 21
        TabOrder = 2
        Text = '5.0E-12'
      end
    end
    object TabSheet5: TTabSheet
      Caption = '&Positions'
      ImageIndex = 4
      object GroupRovAnt: TGroupBox
        Left = 2
        Top = 0
        Width = 397
        Height = 99
        Caption = 'Rover'
        TabOrder = 0
        object RovPosTypeP: TComboBox
          Left = 6
          Top = 14
          Width = 137
          Height = 21
          Style = csDropDownList
          Enabled = False
          ItemIndex = 0
          TabOrder = 0
          Text = 'Lat/Lon/Height (deg/m)'
          OnChange = RovPosTypePChange
          Items.Strings = (
            'Lat/Lon/Height (deg/m)'
            'Lat/Lon/Height (dms/m)'
            'X/Y/Z-ECEF (m)')
        end
        object BtnRovPos: TButton
          Left = 371
          Top = 14
          Width = 21
          Height = 21
          Caption = '...'
          Font.Charset = DEFAULT_CHARSET
          Font.Color = clWindowText
          Font.Height = -9
          Font.Name = 'Tahoma'
          Font.Style = []
          ParentFont = False
          TabOrder = 1
          OnClick = BtnRovPosClick
        end
        object RovPos1: TEdit
          Left = 6
          Top = 36
          Width = 126
          Height = 21
          TabOrder = 2
          Text = '0'
        end
        object RovPos2: TEdit
          Left = 133
          Top = 36
          Width = 129
          Height = 21
          TabOrder = 3
          Text = '0'
        end
        object RovPos3: TEdit
          Left = 263
          Top = 36
          Width = 129
          Height = 21
          TabOrder = 4
          Text = '0'
        end
        object RovAntPcv: TCheckBox
          Left = 6
          Top = 58
          Width = 161
          Height = 17
          Caption = 'Antenna Type (*: Auto)'
          TabOrder = 5
          OnClick = RovAntPcvClick
        end
        object LabelRovAntD: TLabel
          Left = 239
          Top = 60
          Width = 76
          Height = 13
          Caption = 'Delta-E/N/U (m)'
        end
        object RovAnt: TComboBox
          Left = 6
          Top = 74
          Width = 230
          Height = 21
          DropDownCount = 16
          TabOrder = 6
          OnClick = RovAntClick
        end
        object RovAntE: TEdit
          Left = 238
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 7
          Text = '0'
        end
        object RovAntN: TEdit
          Left = 290
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 8
          Text = '0'
        end
        object RovAntU: TEdit
          Left = 342
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 9
          Text = '0'
        end
      end
      object GroupRefAnt: TGroupBox
        Left = 2
        Top = 100
        Width = 397
        Height = 121
        Caption = 'Base Station'
        TabOrder = 1
        object RefPosTypeP: TComboBox
          Left = 6
          Top = 14
          Width = 137
          Height = 21
          Style = csDropDownList
          ItemIndex = 0
          TabOrder = 0
          Text = 'Lat/Lon/Height (deg/m)'
          OnChange = RefPosTypePChange
          Items.Strings = (
            'Lat/Lon/Height (deg/m)'
            'Lat/Lon/Height (dms/m)'
            'X/Y/Z-ECEF (m)'
            'RTCM/Raw Antenna Position'
            'Average of Single Position')
        end
        object BtnRefPos: TButton
          Left = 371
          Top = 14
          Width = 21
          Height = 21
          Caption = '...'
          Font.Charset = DEFAULT_CHARSET
          Font.Color = clWindowText
          Font.Height = -9
          Font.Name = 'Tahoma'
          Font.Style = []
          ParentFont = False
          TabOrder = 1
          OnClick = BtnRefPosClick
        end
        object LabelMaxAveEp: TLabel
          Left = 10
          Top = 39
          Width = 76
          Height = 13
          Caption = 'Max # Ave (ep)'
        end
        object MaxAveEp: TEdit
          Left = 89
          Top = 36
          Width = 43
          Height = 21
          TabOrder = 2
          Text = '3600'
        end
        object ChkInitRestart: TCheckBox
          Left = 135
          Top = 38
          Width = 89
          Height = 17
          Caption = 'Init by Restart'
          TabOrder = 3
        end
        object RefPos1: TEdit
          Left = 6
          Top = 58
          Width = 126
          Height = 21
          TabOrder = 4
          Text = '0'
        end
        object RefPos2: TEdit
          Left = 133
          Top = 58
          Width = 129
          Height = 21
          TabOrder = 5
          Text = '0'
        end
        object RefPos3: TEdit
          Left = 263
          Top = 58
          Width = 129
          Height = 21
          TabOrder = 6
          Text = '0'
        end
        object RefAntPcv: TCheckBox
          Left = 6
          Top = 80
          Width = 165
          Height = 17
          Caption = 'Antenna Type (*: Auto)'
          TabOrder = 7
          OnClick = RovAntPcvClick
        end
        object LabelRefAntD: TLabel
          Left = 241
          Top = 82
          Width = 76
          Height = 13
          Caption = 'Delta-E/N/U (m)'
        end
        object RefAnt: TComboBox
          Left = 6
          Top = 96
          Width = 231
          Height = 21
          DropDownCount = 16
          TabOrder = 8
        end
        object RefAntE: TEdit
          Left = 239
          Top = 96
          Width = 51
          Height = 21
          TabOrder = 9
          Text = '0'
        end
        object RefAntN: TEdit
          Left = 291
          Top = 96
          Width = 51
          Height = 21
          TabOrder = 10
          Text = '0'
        end
        object RefAntU: TEdit
          Left = 343
          Top = 96
          Width = 51
          Height = 21
          TabOrder = 11
          Text = '0'
        end
      end
      object Label30: TLabel
        Left = 10
        Top = 222
        Width = 93
        Height = 13
        Caption = 'Station Position File'
      end
      object StaPosFile: TEdit
        Left = 3
        Top = 236
        Width = 352
        Height = 21
        TabOrder = 2
      end
      object BtnStaPosView: TSpeedButton
        Left = 355
        Top = 236
        Width = 21
        Height = 21
        Flat = True
        Glyph.Data = {
          3E020000424D3E0200000000000036000000280000000D0000000D0000000100
          1800000000000802000000000000000000000000000000000000FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080FFFFFFFFFFFFFFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF000000FFFFFF808080808080808080FFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080808080808080FFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00}
        OnClick = BtnStaPosViewClick
      end
      object BtnStaPosFile: TButton
        Left = 378
        Top = 235
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 3
        OnClick = BtnStaPosFileClick
      end
    end
    object TabSheet7: TTabSheet
      Caption = '&Files'
      ImageIndex = 6
      object Label38: TLabel
        Left = 6
        Top = 2
        Width = 250
        Height = 13
        Caption = 'Satellite/Receiver Antenna PCV File ANTEX/NGS PCV'
      end
      object BtnSatPcvView: TSpeedButton
        Left = 355
        Top = -2
        Width = 21
        Height = 17
        Flat = True
        Glyph.Data = {
          3E020000424D3E0200000000000036000000280000000D0000000D0000000100
          1800000000000802000000000000000000000000000000000000FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080FFFFFFFFFFFFFFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF000000FFFFFF808080808080808080FFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080808080808080FFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00}
        OnClick = BtnSatPcvViewClick
      end
      object BtnAntPcvView: TSpeedButton
        Left = 381
        Top = -2
        Width = 21
        Height = 17
        Flat = True
        Glyph.Data = {
          3E020000424D3E0200000000000036000000280000000D0000000D0000000100
          1800000000000802000000000000000000000000000000000000FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080FFFFFFFFFFFFFFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF000000FFFFFF808080808080808080FFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080808080808080FFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00}
        OnClick = BtnAntPcvViewClick
      end
      object SatPcvFile: TEdit
        Left = 2
        Top = 16
        Width = 378
        Height = 21
        TabOrder = 0
      end
      object BtnSatPcvFile: TButton
        Left = 381
        Top = 16
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 1
        OnClick = BtnSatPcvFileClick
      end
      object AntPcvFile: TEdit
        Left = 2
        Top = 37
        Width = 378
        Height = 21
        TabOrder = 2
      end
      object BtnAntPcvFile: TButton
        Left = 381
        Top = 37
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 3
        OnClick = BtnAntPcvFileClick
      end
      object Label48: TLabel
        Left = 6
        Top = 58
        Width = 72
        Height = 13
        Caption = 'Geoid Data File'
      end
      object GeoidDataFile: TEdit
        Left = 2
        Top = 72
        Width = 378
        Height = 21
        TabOrder = 4
      end
      object BtnGeoidDataFile: TButton
        Left = 381
        Top = 72
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 5
        OnClick = BtnGeoidDataFileClick
      end
      object Label1: TLabel
        Left = 6
        Top = 93
        Width = 65
        Height = 13
        Caption = 'DCB Data File'
      end
      object BtnDCBView: TSpeedButton
        Left = 381
        Top = 91
        Width = 21
        Height = 17
        Flat = True
        Glyph.Data = {
          3E020000424D3E0200000000000036000000280000000D0000000D0000000100
          1800000000000802000000000000000000000000000000000000FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080FFFFFFFFFFFFFFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF000000FFFFFF808080808080808080FFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080808080808080FFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00}
        OnClick = BtnDCBViewClick
      end
      object DCBFile: TEdit
        Left = 2
        Top = 107
        Width = 378
        Height = 21
        TabOrder = 6
      end
      object BtnDCBFile: TButton
        Left = 381
        Top = 107
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 7
        OnClick = BtnDCBFileClick
      end
      object Label23: TLabel
        Left = 6
        Top = 128
        Width = 65
        Height = 13
        Caption = 'EOP Data File'
      end
      object BtnEOPView: TSpeedButton
        Left = 381
        Top = 125
        Width = 21
        Height = 17
        Flat = True
        Glyph.Data = {
          3E020000424D3E0200000000000036000000280000000D0000000D0000000100
          1800000000000802000000000000000000000000000000000000FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080FFFFFFFFFFFFFFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF000000FFFFFF808080808080808080FFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080808080808080FFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00}
        OnClick = BtnEOPViewClick
      end
      object EOPFile: TEdit
        Left = 2
        Top = 142
        Width = 378
        Height = 21
        TabOrder = 8
      end
      object BtnEOPFile: TButton
        Left = 381
        Top = 142
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 9
        OnClick = BtnEOPFileClick
      end
      object Label34: TLabel
        Left = 6
        Top = 163
        Width = 130
        Height = 13
        Caption = 'Ocean Tide Loading BLQ File'
      end
      object BtnBLQFileView: TSpeedButton
        Left = 381
        Top = 160
        Width = 21
        Height = 17
        Flat = True
        Glyph.Data = {
          3E020000424D3E0200000000000036000000280000000D0000000D0000000100
          1800000000000802000000000000000000000000000000000000FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080FFFFFFFFFFFFFFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF000000FFFFFF808080808080808080FFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000FFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000FFFFFF00FFFFFF000000
          FFFFFF808080808080808080808080808080808080808080FFFFFF000000FFFF
          FF00FFFFFF000000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFF000000FFFFFF00FFFFFF00000000000000000000000000000000000000
          0000000000000000000000000000FFFFFF00FFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00FFFFFFFFFFFF
          FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
          FF00}
        OnClick = BtnBLQViewClick
      end
      object BLQFile: TEdit
        Left = 2
        Top = 177
        Width = 378
        Height = 21
        TabOrder = 10
      end
      object BtnBLQFile: TButton
        Left = 381
        Top = 177
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 11
        OnClick = BtnBLQFileClick
      end
      object Label31: TLabel
        Left = 6
        Top = 197
        Width = 121
        Height = 13
        Caption = 'FTP/HTTP Local Directory'
      end
      object LocalDir: TEdit
        Left = 2
        Top = 211
        Width = 378
        Height = 21
        TabOrder = 12
      end
      object BtnLocalDir: TButton
        Left = 381
        Top = 211
        Width = 21
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 13
        OnClick = BtnLocalDirClick
      end
    end
    object TabSheet8: TTabSheet
      Caption = '&Misc'
      ImageIndex = 6
      DesignSize = (
        405
        325)
      object Label19: TLabel
        Left = 32
        Top = 7
        Width = 190
        Height = 13
        Caption = 'Process Cycle (ms) / Buffer Size (bytes)'
      end
      object Label44: TLabel
        Left = 32
        Top = 29
        Width = 164
        Height = 13
        Caption = 'Timeout / Reconnect Interval (ms)'
      end
      object Label46: TLabel
        Left = 32
        Top = 51
        Width = 187
        Height = 13
        Caption = 'NMEA Cycle (ms) / File Swap Margin (s)'
      end
      object Label41: TLabel
        Left = 32
        Top = 73
        Width = 162
        Height = 13
        Caption = 'Solution Buffer/ Log Size (epochs)'
      end
      object Label42: TLabel
        Left = 32
        Top = 95
        Width = 142
        Height = 13
        Caption = 'Navigation Message Selection'
      end
      object Label5: TLabel
        Left = 32
        Top = 117
        Width = 193
        Height = 13
        Caption = 'SBAS Sat Selection (0: all) / Monitor Port'
      end
      object Label45: TLabel
        Left = 32
        Top = 139
        Width = 96
        Height = 13
        Caption = 'HTTP / NTRIP Proxy'
      end
      object Label49: TLabel
        Left = 32
        Top = 161
        Width = 51
        Height = 13
        Caption = 'Panel Font'
      end
      object FontLabel1: TLabel
        Left = 148
        Top = 161
        Width = 200
        Height = 18
        Caption = 'Font Label 1'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -15
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
      end
      object Label40: TLabel
        Left = 32
        Top = 188
        Width = 63
        Height = 13
        Caption = 'Solution Font'
      end
      object FontLabel2: TLabel
        Left = 148
        Top = 188
        Width = 200
        Height = 18
        Caption = 'Font Label 2'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -15
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
      end
      object Label43: TLabel
        Left = 32
        Top = 215
        Width = 62
        Height = 13
        Caption = 'Panel Layout'
      end
      object SvrCycleE: TEdit
        Left = 248
        Top = 4
        Width = 66
        Height = 21
        TabOrder = 0
        Text = '10'
      end
      object SvrBuffSizeE: TEdit
        Left = 316
        Top = 4
        Width = 66
        Height = 21
        TabOrder = 1
        Text = '32768'
      end
      object TimeoutTimeE: TEdit
        Left = 248
        Top = 26
        Width = 66
        Height = 21
        TabOrder = 2
        Text = '10000'
      end
      object ReconTimeE: TEdit
        Left = 316
        Top = 26
        Width = 66
        Height = 21
        TabOrder = 3
        Text = '10000'
      end
      object NmeaCycleE: TEdit
        Left = 248
        Top = 48
        Width = 66
        Height = 21
        TabOrder = 4
        Text = '5000'
      end
      object FileSwapMarginE: TEdit
        Left = 316
        Top = 48
        Width = 66
        Height = 21
        TabOrder = 5
        Text = '30'
      end
      object SolBuffSizeE: TEdit
        Left = 248
        Top = 70
        Width = 66
        Height = 21
        TabOrder = 6
        Text = '1000'
      end
      object SavedSolE: TEdit
        Left = 316
        Top = 70
        Width = 66
        Height = 21
        TabOrder = 7
        Text = '100'
      end
      object NavSelectS: TComboBox
        Left = 248
        Top = 92
        Width = 134
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 8
        Text = 'All'
        Items.Strings = (
          'All'
          '(1) Rover'
          '(2) Base Station'
          '(3) Correction')
      end
      object SbasSatE: TEdit
        Left = 248
        Top = 114
        Width = 66
        Height = 21
        TabOrder = 9
        Text = '0'
      end
      object MoniPortE: TEdit
        Left = 316
        Top = 114
        Width = 66
        Height = 21
        TabOrder = 10
        Text = '0'
      end
      object ProxyAddrE: TEdit
        Left = 148
        Top = 136
        Width = 234
        Height = 21
        TabOrder = 11
      end
      object BtnFont1: TButton
        Left = 358
        Top = 158
        Width = 25
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 12
        OnClick = BtnFont1Click
      end
      object BtnFont2: TButton
        Left = 358
        Top = 185
        Width = 25
        Height = 21
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 13
        OnClick = BtnFont2Click
      end
      object PanelStackE: TComboBox
        Left = 248
        Top = 212
        Width = 134
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 14
        Text = 'Horizontal'
        Items.Strings = (
          'Horizontal'
          'Vertical')
      end
    end
  end
  object OpenDialog: TOpenDialog
    Filter = 
      'All (*.*)|*.*|PCV File (*.pcv,*.atx)|*.pcv;*.atx|Position File (' +
      '*.pos,*.snx)|*.pos;*.snx|Options File (*.conf)|*.conf|DCB Data '  +
      'File (*.dcb)|*.dcb|EOP Data File (*.eop,*.erp)|*.eop;*.erp|OTL '  +
      'BLQ File (*.blq)|*.blq'
    Options = [ofHideReadOnly, ofNoChangeDir, ofEnableSizing]
    Title = 'Load File'
    Left = 164
    Top = 308
  end
  object SaveDialog: TSaveDialog
    Filter = 'All (*.*)|*.*|Options File (*.conf)|*.conf'
    Title = 'Save File'
    Left = 120
    Top = 309
  end
  object FontDialog: TFontDialog
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = []
    Left = 211
    Top = 307
  end
end
