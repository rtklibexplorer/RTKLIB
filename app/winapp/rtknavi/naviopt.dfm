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
  object Label47: TLabel
    Left = 10
    Top = 183
    Width = 93
    Height = 13
    Caption = 'Station Position File'
  end
  object BtnCancel: TButton
    Left = 310
    Top = 353
    Width = 99
    Height = 29
    Caption = '&Cancel'
    ModalResult = 2
    TabOrder = 1
  end
  object BtnOk: TButton
    Left = 205
    Top = 353
    Width = 99
    Height = 29
    Caption = '&OK'
    ModalResult = 1
    TabOrder = 0
    OnClick = BtnOkClick
  end
  object BtnSave: TButton
    Left = 100
    Top = 354
    Width = 99
    Height = 28
    Caption = '&Save'
    TabOrder = 3
    OnClick = BtnSaveClick
  end
  object Options: TPageControl
    Left = 0
    Top = 0
    Width = 413
    Height = 353
    ActivePage = TabSheet2
    Align = alTop
    TabOrder = 4
    ExplicitWidth = 407
    object TabSheet1: TTabSheet
      Caption = 'Setting&1'
      object Label3: TLabel
        Left = 24
        Top = 137
        Width = 183
        Height = 13
        Caption = 'Rec Dynamics / Earth Tides Correction'
      end
      object Label9: TLabel
        Left = 24
        Top = 181
        Width = 114
        Height = 13
        Caption = 'Troposphere Correction'
      end
      object Label8: TLabel
        Left = 24
        Top = 159
        Width = 108
        Height = 13
        Caption = 'Ionosphere Correction'
      end
      object LabelPosMode: TLabel
        Left = 24
        Top = 7
        Width = 80
        Height = 13
        Caption = 'Positioning Mode'
      end
      object LabelFreq: TLabel
        Left = 24
        Top = 32
        Width = 58
        Height = 13
        Caption = 'Frequencies'
      end
      object LabelElMask: TLabel
        Left = 24
        Top = 51
        Width = 179
        Height = 13
        Caption = 'Elevation Mask ('#176') / SNR Mask (dbHz)'
      end
      object Label32: TLabel
        Left = 24
        Top = 203
        Width = 119
        Height = 13
        Caption = 'Satellite Ephemeris/Clock'
      end
      object Label35: TLabel
        Left = 24
        Top = 252
        Width = 176
        Height = 13
        Caption = 'Excluded Satellites (+PRN: Included)'
      end
      object BtnFreq: TSpeedButton
        Left = 102
        Top = 27
        Width = 18
        Height = 22
        Caption = '?'
        Flat = True
        Spacing = 0
        OnClick = BtnFreqClick
      end
      object NavSys1: TCheckBox
        Left = 24
        Top = 274
        Width = 49
        Height = 17
        Caption = 'GPS'
        Checked = True
        State = cbChecked
        TabOrder = 7
      end
      object DynamicModel: TComboBox
        Left = 248
        Top = 134
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 2
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object IonoOpt: TComboBox
        Left = 248
        Top = 156
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 8
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
        Top = 178
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 4
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'Saastamoinen'
          'SBAS'
          'Estimate ZTD'
          'Estimate ZTD+Grad')
      end
      object PosMode: TComboBox
        Left = 248
        Top = 3
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
      object Freq: TComboBox
        Left = 248
        Top = 26
        Width = 152
        Height = 21
        Style = csDropDownList
        TabOrder = 1
        OnChange = FreqChange
      end
      object SatEphem: TComboBox
        Left = 248
        Top = 200
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 5
        Text = 'Broadcast'
        Items.Strings = (
          'Broadcast'
          'Precise'
          'Broadcast+SBAS'
          'Broadcast+SSR APC'
          'Broadcast+SSR CoM'
          'QZSS LEX')
      end
      object ExSatsE: TEdit
        Left = 221
        Top = 248
        Width = 179
        Height = 21
        TabOrder = 6
      end
      object NavSys2: TCheckBox
        Left = 68
        Top = 274
        Width = 71
        Height = 17
        Caption = 'GLONASS'
        TabOrder = 12
        OnClick = NavSys2Click
      end
      object NavSys3: TCheckBox
        Left = 134
        Top = 274
        Width = 61
        Height = 17
        Caption = 'Galileo'
        TabOrder = 9
      end
      object NavSys4: TCheckBox
        Left = 190
        Top = 271
        Width = 44
        Height = 23
        Caption = 'QZSS'
        TabOrder = 10
      end
      object NavSys5: TCheckBox
        Left = 338
        Top = 274
        Width = 51
        Height = 17
        Caption = 'SBAS'
        TabOrder = 11
      end
      object TideCorr: TComboBox
        Left = 325
        Top = 134
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 3
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'Solid')
      end
      object NavSys6: TCheckBox
        Left = 241
        Top = 273
        Width = 69
        Height = 19
        Caption = 'BDS'
        TabOrder = 22
        OnClick = NavSys6Click
      end
      object ElMask: TComboBox
        Left = 248
        Top = 48
        Width = 75
        Height = 21
        AutoComplete = False
        DropDownCount = 16
        TabOrder = 13
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
      object PosOpt1: TCheckBox
        Left = 24
        Top = 226
        Width = 61
        Height = 17
        Caption = 'Sat PCV'
        TabOrder = 14
      end
      object PosOpt2: TCheckBox
        Left = 82
        Top = 226
        Width = 62
        Height = 17
        Caption = 'Rec PCV'
        TabOrder = 15
      end
      object PosOpt3: TCheckBox
        Left = 142
        Top = 226
        Width = 69
        Height = 17
        Caption = 'PhWU'
        TabOrder = 16
      end
      object PosOpt4: TCheckBox
        Left = 191
        Top = 226
        Width = 68
        Height = 17
        Caption = 'Rej Ecl'
        TabOrder = 17
      end
      object PosOpt5: TCheckBox
        Left = 244
        Top = 226
        Width = 68
        Height = 17
        Caption = 'RAIM FDE'
        TabOrder = 18
      end
      object BtnSnrMask: TButton
        Left = 323
        Top = 47
        Width = 77
        Height = 21
        Caption = '...'
        TabOrder = 20
        OnClick = BtnSnrMaskClick
      end
      object PosOpt6: TCheckBox
        Left = 313
        Top = 226
        Width = 68
        Height = 17
        Caption = 'DBCorr'
        TabOrder = 19
      end
      object NavSys7: TCheckBox
        Left = 284
        Top = 273
        Width = 48
        Height = 19
        Caption = 'NavIC'
        TabOrder = 21
        OnClick = NavSys6Click
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
        Top = 33
        Width = 181
        Height = 13
        Caption = 'Ratio to Fix Ambiguity (Min/Nom/Max)'
      end
      object Label13: TLabel
        Left = 24
        Top = 77
        Width = 164
        Height = 13
        Caption = 'Min Lock / Elevation ('#176') to Fix Amb'
      end
      object LabelHold: TLabel
        Left = 24
        Top = 99
        Width = 164
        Height = 13
        Caption = 'Min Fix / Elevation ('#176') to Hold Amb'
      end
      object Label22: TLabel
        Left = 24
        Top = 123
        Width = 206
        Height = 13
        Caption = 'Slip Threshs: Doppler (Hz) / Geom-Free (m)'
      end
      object Label14: TLabel
        Left = 24
        Top = 147
        Width = 192
        Height = 13
        Caption = 'Max Age of Diff (s) / Outs to Reset Amb'
      end
      object Label11: TLabel
        Left = 24
        Top = 169
        Width = 179
        Height = 13
        Caption = 'Outlier Threshold for Code/Phase (m)'
      end
      object Label37: TLabel
        Left = 24
        Top = 193
        Width = 143
        Height = 13
        Caption = '# of Filter Iter / Sync Solution'
      end
      object Label54: TLabel
        Left = 24
        Top = 236
        Width = 131
        Height = 13
        Caption = 'Min Fix Sats / Min Hold Sats'
      end
      object Label50: TLabel
        Left = 24
        Top = 278
        Width = 147
        Height = 13
        Caption = 'Max Pos Var for AR  / AR Filter'
      end
      object Label51: TLabel
        Left = 26
        Top = 258
        Width = 66
        Height = 13
        Caption = 'Min Drop Sats'
      end
      object Label52: TLabel
        Left = 24
        Top = 300
        Width = 143
        Height = 13
        Caption = 'Hold Amb Var / Hold Amb Gain'
      end
      object Label15: TLabel
        Left = 26
        Top = 54
        Width = 62
        Height = 13
        Caption = 'GLO HW Bias'
      end
      object AmbRes: TComboBox
        Left = 248
        Top = 4
        Width = 62
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
      object ValidThresAR: TEdit
        Left = 300
        Top = 28
        Width = 49
        Height = 21
        TabOrder = 2
        Text = '3.0'
      end
      object LockCntFixAmb: TEdit
        Left = 248
        Top = 74
        Width = 75
        Height = 21
        TabOrder = 3
        Text = '0'
      end
      object OutCntResetAmb: TEdit
        Left = 326
        Top = 144
        Width = 75
        Height = 21
        TabOrder = 5
        Text = '5'
      end
      object ElMaskAR: TEdit
        Left = 325
        Top = 74
        Width = 75
        Height = 21
        TabOrder = 4
        Text = '0'
      end
      object SlipThres: TEdit
        Left = 326
        Top = 120
        Width = 75
        Height = 21
        TabOrder = 6
        Text = '0.05'
      end
      object MaxAgeDiff: TEdit
        Left = 248
        Top = 144
        Width = 75
        Height = 21
        TabOrder = 7
        Text = '30'
      end
      object RejectPhase: TEdit
        Left = 325
        Top = 166
        Width = 75
        Height = 21
        TabOrder = 10
        Text = '30'
      end
      object NumIter: TEdit
        Left = 248
        Top = 193
        Width = 75
        Height = 21
        TabOrder = 11
        Text = '1'
      end
      object GloAmbRes: TComboBox
        Left = 312
        Top = 4
        Width = 43
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
      object BaselineConst: TCheckBox
        Left = 24
        Top = 212
        Width = 181
        Height = 17
        Caption = 'Baseline Length Constraint (m)'
        TabOrder = 12
        OnClick = BaselineConstClick
      end
      object BaselineLen: TEdit
        Left = 248
        Top = 210
        Width = 75
        Height = 21
        TabOrder = 13
        Text = '0.000'
      end
      object BaselineSig: TEdit
        Left = 325
        Top = 209
        Width = 75
        Height = 21
        TabOrder = 14
        Text = '0.000'
      end
      object FixCntHoldAmb: TEdit
        Left = 248
        Top = 96
        Width = 75
        Height = 21
        TabOrder = 15
        Text = '10'
      end
      object ElMaskHold: TEdit
        Left = 325
        Top = 96
        Width = 75
        Height = 21
        TabOrder = 16
        Text = '10'
      end
      object RejectCode: TEdit
        Left = 248
        Top = 166
        Width = 75
        Height = 21
        TabOrder = 9
        Text = '30'
      end
      object SyncSol: TComboBox
        Left = 326
        Top = 188
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 8
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'ON')
      end
      object BdsAmbRes: TComboBox
        Left = 357
        Top = 4
        Width = 43
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 17
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'ON')
      end
      object MinFixSats: TEdit
        Left = 248
        Top = 232
        Width = 75
        Height = 21
        TabOrder = 18
        Text = '3'
      end
      object MinHoldSats: TEdit
        Left = 326
        Top = 232
        Width = 75
        Height = 21
        TabOrder = 19
        Text = '5'
      end
      object ARFilter: TComboBox
        Left = 325
        Top = 278
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 20
        Text = 'OFF'
        OnChange = AmbResChange
        Items.Strings = (
          'OFF'
          'ON')
      end
      object MaxPosVarAR: TEdit
        Left = 247
        Top = 278
        Width = 75
        Height = 21
        TabOrder = 21
        Text = '0.1'
      end
      object MinDropSats: TEdit
        Left = 248
        Top = 255
        Width = 75
        Height = 21
        TabOrder = 22
        Text = '20'
      end
      object VarHoldAmb: TEdit
        Left = 247
        Top = 301
        Width = 75
        Height = 21
        TabOrder = 23
        Text = '0.001'
      end
      object GainHoldAmb: TEdit
        Left = 326
        Top = 301
        Width = 75
        Height = 21
        TabOrder = 24
        Text = '0.01'
      end
      object GloHwBias: TEdit
        Left = 249
        Top = 51
        Width = 75
        Height = 21
        ParentShowHint = False
        ShowHint = True
        TabOrder = 25
        Text = '0.0'
      end
      object ARIter: TEdit
        Left = 248
        Top = 188
        Width = 75
        Height = 21
        TabOrder = 26
        Text = '1'
      end
      object ValidThresARMin: TEdit
        Left = 248
        Top = 28
        Width = 50
        Height = 21
        TabOrder = 27
        Text = '3.0'
      end
      object ValidThresARMax: TEdit
        Left = 350
        Top = 28
        Width = 50
        Height = 21
        TabOrder = 28
        Text = '3.0'
      end
      object DopThres: TEdit
        Left = 249
        Top = 120
        Width = 73
        Height = 21
        TabOrder = 29
        Text = '0.0'
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
        Top = 97
        Width = 215
        Height = 13
        Caption = 'Output Single if Sol Outage / Max Sol Std (m)'
      end
      object Label2: TLabel
        Left = 24
        Top = 119
        Width = 72
        Height = 13
        Caption = 'Datum / Height'
      end
      object Label18: TLabel
        Left = 24
        Top = 141
        Width = 58
        Height = 13
        Caption = 'Geoid Model'
      end
      object Label20: TLabel
        Left = 23
        Top = 29
        Width = 194
        Height = 13
        Caption = 'Output Header / Proc. Options / Velocity'
      end
      object Label36: TLabel
        Left = 24
        Top = 211
        Width = 217
        Height = 13
        Caption = 'Output Solution Status / Output Debug Trace'
      end
      object Label17: TLabel
        Left = 24
        Top = 187
        Width = 185
        Height = 13
        Caption = 'NMEA Interval (s) RMC/GGA, GSA/GSV'
      end
      object Label21: TLabel
        Left = 24
        Top = 163
        Width = 114
        Height = 13
        Caption = 'Solution for Static Mode'
        Enabled = False
      end
      object SolFormat: TComboBox
        Left = 248
        Top = 4
        Width = 152
        Height = 21
        Style = csDropDownList
        Enabled = False
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
      object TimeFormat: TComboBox
        Left = 248
        Top = 50
        Width = 118
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 17
        Text = 'ww ssss GPST'
        Items.Strings = (
          'ww ssss GPST'
          'hh:mm:ss GPST'
          'hh:mm:ss UTC'
          'hh:mm:ss JST')
      end
      object LatLonFormat: TComboBox
        Left = 248
        Top = 72
        Width = 118
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 5
        Text = 'ddd.ddddddd'
        Items.Strings = (
          'ddd.ddddddd'
          'ddd mm ss.sss')
      end
      object FieldSep: TEdit
        Left = 368
        Top = 72
        Width = 32
        Height = 21
        TabOrder = 6
      end
      object OutputDatum: TComboBox
        Left = 248
        Top = 116
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 9
        Text = 'WGS84'
        Items.Strings = (
          'WGS84')
      end
      object OutputHeight: TComboBox
        Left = 325
        Top = 116
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 10
        Text = 'Ellipsoidal'
        OnClick = OutputHeightClick
        Items.Strings = (
          'Ellipsoidal'
          'Geodetic')
      end
      object OutputGeoid: TComboBox
        Left = 248
        Top = 138
        Width = 152
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 11
        Text = 'Internal'
        Items.Strings = (
          'Internal'
          'EGM96-BE (15")'
          'EGM2008-SE (2.5")'
          'EGM2008-SE (1")'
          'GSI2000 (1x1.5")')
      end
      object OutputHead: TComboBox
        Left = 248
        Top = 28
        Width = 75
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
        Left = 325
        Top = 28
        Width = 75
        Height = 21
        Style = csDropDownList
        Enabled = False
        ItemIndex = 0
        TabOrder = 2
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object TimeDecimal: TEdit
        Left = 368
        Top = 50
        Width = 32
        Height = 21
        TabOrder = 4
        Text = '3'
      end
      object DebugStatus: TComboBox
        Left = 248
        Top = 208
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 12
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'States'
          'Residuals')
      end
      object NmeaIntv1: TEdit
        Left = 248
        Top = 184
        Width = 75
        Height = 21
        TabOrder = 14
        Text = '0'
      end
      object NmeaIntv2: TEdit
        Left = 325
        Top = 184
        Width = 75
        Height = 21
        TabOrder = 15
        Text = '1'
      end
      object DebugTrace: TComboBox
        Left = 325
        Top = 208
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 13
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'Level 1'
          'Level 2'
          'Level 3'
          'Level 4'
          'Level 5')
      end
      object SolStatic: TComboBox
        Left = 248
        Top = 160
        Width = 152
        Height = 21
        Style = csDropDownList
        Enabled = False
        ItemIndex = 0
        TabOrder = 16
        Text = 'All'
        Items.Strings = (
          'All'
          'Single')
      end
      object OutputSingle: TComboBox
        Left = 248
        Top = 94
        Width = 75
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 7
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
      object MaxSolStd: TEdit
        Left = 325
        Top = 94
        Width = 75
        Height = 21
        TabOrder = 8
        Text = '0'
      end
      object OutputVel: TComboBox
        Left = 350
        Top = 28
        Width = 50
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 3
        Text = 'OFF'
        Items.Strings = (
          'OFF'
          'ON')
      end
    end
    object TabSheet4: TTabSheet
      Caption = 'S&tatistics'
      ImageIndex = 3
      object Label29: TLabel
        Left = 37
        Top = 274
        Width = 132
        Height = 13
        Caption = 'Satellite Clock Stability (s/s)'
      end
      object GroupBox3: TGroupBox
        Left = 3
        Top = 0
        Width = 397
        Height = 153
        Caption = 'Measurement Errors (1-sigma)'
        TabOrder = 0
        object Label6: TLabel
          Left = 33
          Top = 19
          Width = 194
          Height = 13
          Caption = 'Code/Carrier-Phase Error Ratio L1/L2/L5'
        end
        object Label7: TLabel
          Left = 33
          Top = 38
          Width = 160
          Height = 13
          Caption = 'Carrier-Phase Error a+b/sinEl (m)'
        end
        object Label16: TLabel
          Left = 34
          Top = 60
          Width = 184
          Height = 13
          Caption = 'Carrier-Phase Error/Baseline (m/10km)'
        end
        object Label10: TLabel
          Left = 33
          Top = 105
          Width = 140
          Height = 13
          Caption = 'Carrier-Phase Error: Rcv Errs'
        end
        object Label12: TLabel
          Left = 33
          Top = 82
          Width = 185
          Height = 13
          Caption = 'Carrier Phase Error: SNR / SNR maxDb'
        end
        object Label39: TLabel
          Left = -269
          Top = 247
          Width = 140
          Height = 13
          Caption = 'Carrier-Phase Error: Rcv Errs'
        end
        object Label53: TLabel
          Left = 33
          Top = 127
          Width = 112
          Height = 13
          Caption = 'Doppler Freq Error (Hz)'
        end
        object MeasErrR1: TEdit
          Left = 242
          Top = 12
          Width = 36
          Height = 21
          TabOrder = 0
          Text = '100.0'
        end
        object MeasErr2: TEdit
          Left = 242
          Top = 35
          Width = 73
          Height = 21
          TabOrder = 2
          Text = '0.003'
        end
        object MeasErr3: TEdit
          Left = 318
          Top = 35
          Width = 73
          Height = 21
          TabOrder = 3
          Text = '0.003'
        end
        object MeasErr4: TEdit
          Left = 242
          Top = 57
          Width = 73
          Height = 21
          TabOrder = 4
          Text = '0.000'
        end
        object MeasErrR2: TEdit
          Left = 279
          Top = 12
          Width = 36
          Height = 21
          TabOrder = 1
          Text = '100.0'
        end
        object MeasErrR5: TEdit
          Left = 318
          Top = 12
          Width = 34
          Height = 21
          TabOrder = 5
          Text = '100.0'
        end
        object MeasErrR6: TEdit
          Left = 355
          Top = 12
          Width = 36
          Height = 21
          TabOrder = 6
          Text = '100.0'
        end
      end
      object GroupBox4: TGroupBox
        Left = 6
        Top = 156
        Width = 396
        Height = 105
        Caption = 'Process Noises (1-sigma/sqrt(s))'
        TabOrder = 1
        object Label26: TLabel
          Left = 32
          Top = 35
          Width = 123
          Height = 13
          Caption = 'Carrier-Phase Bias (cycle)'
        end
        object Label27: TLabel
          Left = 32
          Top = 60
          Width = 172
          Height = 13
          Caption = 'Vertical Ionospheric Delay (m/10km)'
        end
        object Label28: TLabel
          Left = 32
          Top = 84
          Width = 144
          Height = 13
          Caption = 'Zenith Tropospheric Delay (m)'
        end
        object Label33: TLabel
          Left = 32
          Top = 16
          Width = 170
          Height = 13
          Caption = 'Receiver Accel Horiz/Vertical (m/s2)'
        end
        object PrNoise1: TEdit
          Left = 246
          Top = 36
          Width = 148
          Height = 21
          TabOrder = 2
          Text = '1.0E-04'
        end
        object PrNoise2: TEdit
          Left = 246
          Top = 58
          Width = 148
          Height = 21
          TabOrder = 3
          Text = '1.0E-03'
        end
        object PrNoise3: TEdit
          Left = 246
          Top = 80
          Width = 148
          Height = 21
          TabOrder = 4
          Text = '1.0E-04'
        end
        object PrNoise4: TEdit
          Left = 246
          Top = 14
          Width = 73
          Height = 21
          TabOrder = 0
          Text = '1.0E-04'
        end
        object PrNoise5: TEdit
          Left = 321
          Top = 14
          Width = 73
          Height = 21
          TabOrder = 1
          Text = '1.0E-04'
        end
      end
      object SatClkStab: TEdit
        Left = 248
        Top = 267
        Width = 148
        Height = 21
        TabOrder = 2
        Text = '5.0E-12'
      end
      object MeasErr7: TEdit
        Left = 245
        Top = 79
        Width = 73
        Height = 21
        TabOrder = 3
        Text = '0'
      end
      object MeasErr6: TEdit
        Left = 321
        Top = 79
        Width = 73
        Height = 21
        TabOrder = 4
        Text = '52'
      end
      object MeasErr8: TEdit
        Left = 245
        Top = 102
        Width = 73
        Height = 21
        TabOrder = 5
        Text = '0'
      end
      object MeasErr5: TEdit
        Left = 245
        Top = 124
        Width = 73
        Height = 21
        TabOrder = 6
        Text = '0'
      end
    end
    object TabSheet5: TTabSheet
      Caption = '&Positions'
      ImageIndex = 4
      object Label4: TLabel
        Left = 12
        Top = 12
        Width = 3
        Height = 13
      end
      object Label30: TLabel
        Left = 6
        Top = 195
        Width = 93
        Height = 13
        Caption = 'Station Position File'
      end
      object BtnStaPosView: TSpeedButton
        Left = 351
        Top = 209
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
      object GroupRefAnt: TGroupBox
        Left = 3
        Top = 96
        Width = 397
        Height = 99
        Caption = 'Base Station'
        TabOrder = 1
        object LabelRefAntD: TLabel
          Left = 241
          Top = 58
          Width = 76
          Height = 13
          Caption = 'Delta-E/N/U (m)'
        end
        object LabelMaxAveEp: TLabel
          Left = 153
          Top = 18
          Width = 76
          Height = 13
          Caption = 'Max # Ave (ep)'
          Visible = False
        end
        object RefAntE: TEdit
          Left = 239
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 7
          Text = '0'
        end
        object RefAntN: TEdit
          Left = 291
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 8
          Text = '0'
        end
        object RefAntU: TEdit
          Left = 343
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 9
          Text = '0'
        end
        object RefPos1: TEdit
          Left = 6
          Top = 36
          Width = 126
          Height = 21
          TabOrder = 1
          Text = '0'
        end
        object RefPos2: TEdit
          Left = 133
          Top = 36
          Width = 129
          Height = 21
          TabOrder = 2
          Text = '0'
        end
        object RefPos3: TEdit
          Left = 263
          Top = 36
          Width = 129
          Height = 21
          TabOrder = 3
          Text = '0'
        end
        object BtnRefPos: TButton
          Left = 368
          Top = 13
          Width = 25
          Height = 23
          Caption = '...'
          Font.Charset = DEFAULT_CHARSET
          Font.Color = clWindowText
          Font.Height = -9
          Font.Name = 'Tahoma'
          Font.Style = []
          ParentFont = False
          TabOrder = 4
          OnClick = BtnRefPosClick
        end
        object RefAntPcv: TCheckBox
          Left = 6
          Top = 58
          Width = 165
          Height = 17
          Caption = 'Antenna Type (*: Auto)'
          TabOrder = 5
          OnClick = RovAntPcvClick
        end
        object RefAnt: TComboBox
          Left = 6
          Top = 74
          Width = 231
          Height = 21
          DropDownCount = 16
          TabOrder = 6
        end
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
        object MaxAveEp: TEdit
          Left = 232
          Top = 14
          Width = 43
          Height = 21
          TabOrder = 10
          Text = '3600'
          Visible = False
        end
        object ChkInitRestart: TCheckBox
          Left = 280
          Top = 16
          Width = 89
          Height = 17
          Caption = 'Init by Restart'
          TabOrder = 11
        end
      end
      object StaPosFile: TEdit
        Left = 2
        Top = 209
        Width = 356
        Height = 21
        TabOrder = 2
      end
      object BtnStaPosFile: TButton
        Left = 381
        Top = 208
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
      object GroupRovAnt: TGroupBox
        Left = 3
        Top = -2
        Width = 397
        Height = 99
        Caption = 'Rover'
        TabOrder = 0
        object LabelRovAntD: TLabel
          Left = 241
          Top = 58
          Width = 76
          Height = 13
          Caption = 'Delta-E/N/U (m)'
        end
        object RovAntE: TEdit
          Left = 239
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 7
          Text = '0'
        end
        object RovAntN: TEdit
          Left = 291
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 8
          Text = '0'
        end
        object RovAntU: TEdit
          Left = 343
          Top = 74
          Width = 51
          Height = 21
          TabOrder = 9
          Text = '0'
        end
        object RovPos1: TEdit
          Left = 6
          Top = 36
          Width = 126
          Height = 21
          TabOrder = 1
          Text = '0'
        end
        object RovPos2: TEdit
          Left = 133
          Top = 36
          Width = 129
          Height = 21
          TabOrder = 2
          Text = '0'
        end
        object RovPos3: TEdit
          Left = 263
          Top = 36
          Width = 129
          Height = 21
          TabOrder = 3
          Text = '0'
        end
        object BtnRovPos: TButton
          Left = 368
          Top = 13
          Width = 25
          Height = 23
          Caption = '...'
          Font.Charset = DEFAULT_CHARSET
          Font.Color = clWindowText
          Font.Height = -9
          Font.Name = 'Tahoma'
          Font.Style = []
          ParentFont = False
          TabOrder = 4
          OnClick = BtnRovPosClick
        end
        object RovAntPcv: TCheckBox
          Left = 6
          Top = 58
          Width = 153
          Height = 17
          Caption = 'Antenna Type (*: Auto)'
          TabOrder = 5
          OnClick = RovAntPcvClick
        end
        object RovAnt: TComboBox
          Left = 6
          Top = 74
          Width = 231
          Height = 21
          DropDownCount = 16
          TabOrder = 6
          OnClick = RovAntClick
        end
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
      end
    end
    object TabSheet7: TTabSheet
      Caption = '&Files'
      ImageIndex = 6
      object Label1: TLabel
        Left = 6
        Top = 93
        Width = 65
        Height = 13
        Caption = 'DCB Data File'
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
      object Label48: TLabel
        Left = 6
        Top = 58
        Width = 72
        Height = 13
        Caption = 'Geoid Data File'
      end
      object Label31: TLabel
        Left = 6
        Top = 197
        Width = 121
        Height = 13
        Caption = 'FTP/HTTP Local Directory'
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
        OnClick = BtnStaPosViewClick
      end
      object Label34: TLabel
        Left = 6
        Top = 163
        Width = 130
        Height = 13
        Caption = 'Ocean Loading BLQ Format'
      end
      object BtnOLFileView: TSpeedButton
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
      object AntPcvFile: TEdit
        Left = 2
        Top = 37
        Width = 378
        Height = 21
        TabOrder = 2
      end
      object BtnAntPcvFile: TButton
        Left = 381
        Top = 36
        Width = 21
        Height = 23
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
      object BtnDCBFile: TButton
        Left = 381
        Top = 106
        Width = 21
        Height = 23
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
      object DCBFile: TEdit
        Left = 2
        Top = 107
        Width = 378
        Height = 21
        TabOrder = 6
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
        Top = 15
        Width = 21
        Height = 23
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
      object GeoidDataFile: TEdit
        Left = 2
        Top = 72
        Width = 378
        Height = 21
        TabOrder = 4
      end
      object BtnGeoidDataFile: TButton
        Left = 381
        Top = 71
        Width = 21
        Height = 23
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
      object LocalDir: TEdit
        Left = 2
        Top = 211
        Width = 378
        Height = 21
        TabOrder = 12
      end
      object BtnLocalDir: TButton
        Left = 381
        Top = 210
        Width = 21
        Height = 23
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
      object OLFile: TEdit
        Left = 2
        Top = 177
        Width = 378
        Height = 21
        TabOrder = 10
      end
      object BtnOLFile: TButton
        Left = 381
        Top = 176
        Width = 21
        Height = 23
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 11
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
        Top = 140
        Width = 21
        Height = 23
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
      object Label40: TLabel
        Left = 147
        Top = 167
        Width = 63
        Height = 13
        Caption = 'Solution Font'
      end
      object FontLabel2: TLabel
        Left = 253
        Top = 188
        Width = 80
        Height = 18
        Alignment = taRightJustify
        Anchors = [akRight, akBottom]
        Caption = 'Font Label 2'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -15
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        Layout = tlCenter
        ExplicitLeft = 265
      end
      object Label41: TLabel
        Left = 32
        Top = 74
        Width = 162
        Height = 13
        Caption = 'Solution Buffer/ Log Size (epochs)'
      end
      object Label42: TLabel
        Left = 32
        Top = 96
        Width = 142
        Height = 13
        Caption = 'Navigation Message Selection'
      end
      object Label5: TLabel
        Left = 32
        Top = 118
        Width = 193
        Height = 13
        Caption = 'SBAS Sat Selection (0: all) / Monitor Port'
      end
      object Label46: TLabel
        Left = 32
        Top = 51
        Width = 187
        Height = 13
        Caption = 'NMEA Cycle (ms) / File Swap Margin (s)'
      end
      object Label44: TLabel
        Left = 32
        Top = 29
        Width = 164
        Height = 13
        Caption = 'Timeout / Reconnect Interval (ms)'
      end
      object Label45: TLabel
        Left = 32
        Top = 142
        Width = 96
        Height = 13
        Caption = 'HTTP / NTRIP Proxy'
      end
      object Label43: TLabel
        Left = 32
        Top = 214
        Width = 62
        Height = 13
        Caption = 'Panel Layout'
      end
      object Label49: TLabel
        Left = 32
        Top = 167
        Width = 51
        Height = 13
        Caption = 'Panel Font'
      end
      object FontLabel1: TLabel
        Left = 253
        Top = 164
        Width = 80
        Height = 18
        Alignment = taRightJustify
        Anchors = [akRight, akBottom]
        Caption = 'Font Label 1'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -15
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        Layout = tlCenter
        ExplicitLeft = 265
      end
      object SvrCycleE: TEdit
        Left = 248
        Top = 5
        Width = 66
        Height = 21
        TabOrder = 0
        Text = '10'
      end
      object SvrBuffSizeE: TEdit
        Left = 316
        Top = 5
        Width = 66
        Height = 21
        TabOrder = 5
        Text = '32768'
      end
      object BtnFont2: TButton
        Left = 358
        Top = 185
        Width = 25
        Height = 24
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
      object SolBuffSizeE: TEdit
        Left = 248
        Top = 72
        Width = 66
        Height = 21
        TabOrder = 6
        Text = '1000'
      end
      object NavSelectS: TComboBox
        Left = 248
        Top = 94
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
        Top = 116
        Width = 66
        Height = 21
        TabOrder = 9
        Text = '0'
      end
      object SavedSolE: TEdit
        Left = 316
        Top = 72
        Width = 66
        Height = 21
        TabOrder = 7
        Text = '100'
      end
      object NmeaCycleE: TEdit
        Left = 248
        Top = 49
        Width = 66
        Height = 21
        TabOrder = 3
        Text = '5000'
      end
      object TimeoutTimeE: TEdit
        Left = 248
        Top = 27
        Width = 66
        Height = 21
        TabOrder = 1
        Text = '10000'
      end
      object ReconTimeE: TEdit
        Left = 316
        Top = 27
        Width = 66
        Height = 21
        TabOrder = 2
        Text = '10000'
      end
      object MoniPortE: TEdit
        Left = 316
        Top = 116
        Width = 66
        Height = 21
        TabOrder = 10
        Text = '0'
      end
      object FileSwapMarginE: TEdit
        Left = 316
        Top = 49
        Width = 66
        Height = 21
        TabOrder = 4
        Text = '30'
      end
      object ProxyAddrE: TEdit
        Left = 148
        Top = 139
        Width = 234
        Height = 21
        TabOrder = 11
      end
      object PanelStackE: TComboBox
        Left = 248
        Top = 211
        Width = 134
        Height = 21
        Style = csDropDownList
        ItemIndex = 0
        TabOrder = 12
        Text = 'Horizontal'
        Items.Strings = (
          'Horizontal'
          'Vertical')
      end
      object BtnFont1: TButton
        Left = 358
        Top = 160
        Width = 25
        Height = 24
        Caption = '...'
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -9
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 14
        OnClick = BtnFont1Click
      end
    end
  end
  object BtnLoad: TButton
    Left = 0
    Top = 355
    Width = 100
    Height = 27
    Caption = '&Load...'
    TabOrder = 2
    OnClick = BtnLoadClick
  end
  object OpenDialog: TOpenDialog
    Filter = 
      'All (*.*)|*.*|PCV File (*.pcv,*.atx)|*.pcv;*.atx|Position File (' +
      '*.pos,*.snx)|*.pos;*.snx|Options File (*.conf)|*.conf'
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
