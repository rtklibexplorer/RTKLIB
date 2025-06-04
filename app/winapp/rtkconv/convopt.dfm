object ConvOptDialog: TConvOptDialog
  Left = 0
  Top = 0
  BorderStyle = bsDialog
  Caption = 'Options'
  ClientHeight = 389
  ClientWidth = 436
  Color = clWhite
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  Position = poMainFormCenter
  OnShow = FormShow
  TextHeight = 13
  object Label3: TLabel
    Left = 153
    Top = 357
    Width = 31
    Height = 13
    Caption = 'Debug'
  end
  object Label8: TLabel
    Left = 10
    Top = 330
    Width = 82
    Height = 13
    Caption = 'Receiver Options'
  end
  object Label11: TLabel
    Left = 343
    Top = 228
    Width = 89
    Height = 13
    Caption = 'Excluded Satellites'
  end
  object Label9: TLabel
    Left = 10
    Top = 6
    Width = 49
    Height = 13
    Caption = 'RINEX Ver'
  end
  object Label12: TLabel
    Left = 204
    Top = 6
    Width = 48
    Height = 13
    Caption = 'Station ID'
  end
  object Label13: TLabel
    Left = 10
    Top = 357
    Width = 88
    Height = 20
    Caption = 'Time Torelance (s)'
  end
  object BtnOk: TButton
    Left = 255
    Top = 350
    Width = 80
    Height = 36
    Caption = '&OK'
    ModalResult = 1
    TabOrder = 0
    OnClick = BtnOkClick
  end
  object BtnCancel: TButton
    Left = 342
    Top = 350
    Width = 80
    Height = 36
    Cancel = True
    Caption = '&Cancel'
    ModalResult = 2
    TabOrder = 1
  end
  object GroupBox1: TGroupBox
    Left = 1
    Top = 24
    Width = 433
    Height = 204
    TabOrder = 6
    object Label1: TLabel
      Left = 10
      Top = 72
      Width = 99
      Height = 13
      Caption = 'Maker Name/#/Type'
    end
    object Label2: TLabel
      Left = 10
      Top = 6
      Width = 104
      Height = 13
      Caption = 'RunBy/Obsrv/Agency'
    end
    object Label4: TLabel
      Left = 10
      Top = 94
      Width = 82
      Height = 13
      Caption = 'Rec #/Type/Vers'
    end
    object Label5: TLabel
      Left = 10
      Top = 116
      Width = 56
      Height = 13
      Caption = 'Ant #/Type'
    end
    object Label6: TLabel
      Left = 10
      Top = 138
      Width = 76
      Height = 13
      Caption = 'Approx Pos XYZ'
    end
    object Label7: TLabel
      Left = 10
      Top = 160
      Width = 100
      Height = 13
      Caption = 'Antenna Delta H/E/N'
    end
    object Label10: TLabel
      Left = 10
      Top = 37
      Width = 45
      Height = 13
      Caption = 'Comment'
    end
    object Marker: TEdit
      Left = 128
      Top = 69
      Width = 95
      Height = 21
      TabOrder = 5
    end
    object RunBy: TEdit
      Left = 128
      Top = 3
      Width = 95
      Height = 21
      TabOrder = 0
    end
    object Name0: TEdit
      Left = 224
      Top = 3
      Width = 95
      Height = 21
      TabOrder = 1
    end
    object Name1: TEdit
      Left = 320
      Top = 3
      Width = 95
      Height = 21
      TabOrder = 2
    end
    object Rec2: TEdit
      Left = 320
      Top = 91
      Width = 95
      Height = 21
      TabOrder = 10
    end
    object Rec1: TEdit
      Left = 224
      Top = 91
      Width = 95
      Height = 21
      TabOrder = 9
    end
    object Rec0: TEdit
      Left = 128
      Top = 91
      Width = 95
      Height = 21
      TabOrder = 8
    end
    object Ant0: TEdit
      Left = 128
      Top = 113
      Width = 95
      Height = 21
      TabOrder = 11
    end
    object Ant1: TEdit
      Left = 224
      Top = 113
      Width = 95
      Height = 21
      TabOrder = 12
    end
    object AppPos2: TEdit
      Left = 320
      Top = 135
      Width = 95
      Height = 21
      TabOrder = 15
      Text = '0.0000'
    end
    object AntDel2: TEdit
      Left = 320
      Top = 157
      Width = 95
      Height = 21
      TabOrder = 18
      Text = '0.0000'
    end
    object AppPos1: TEdit
      Left = 224
      Top = 135
      Width = 95
      Height = 21
      TabOrder = 14
      Text = '0.0000'
    end
    object AntDel1: TEdit
      Left = 224
      Top = 157
      Width = 95
      Height = 21
      TabOrder = 17
      Text = '0.0000'
    end
    object AppPos0: TEdit
      Left = 128
      Top = 135
      Width = 95
      Height = 21
      TabOrder = 13
      Text = '0.0000'
    end
    object AntDel0: TEdit
      Left = 128
      Top = 157
      Width = 95
      Height = 21
      TabOrder = 16
      Text = '0.0000'
    end
    object Comment0: TEdit
      Left = 128
      Top = 25
      Width = 287
      Height = 21
      TabOrder = 3
    end
    object Comment1: TEdit
      Left = 128
      Top = 47
      Width = 287
      Height = 21
      TabOrder = 4
    end
    object MarkerType: TEdit
      Left = 320
      Top = 69
      Width = 95
      Height = 21
      TabOrder = 7
    end
    object MarkerNo: TEdit
      Left = 224
      Top = 69
      Width = 95
      Height = 21
      TabOrder = 6
    end
    object AutoPos: TCheckBox
      Left = 109
      Top = 137
      Width = 18
      Height = 18
      TabOrder = 19
      OnClick = AutoPosClick
    end
    object PhaseShift: TCheckBox
      Left = 5
      Top = 182
      Width = 78
      Height = 18
      Caption = 'Phase Shift'
      TabOrder = 20
    end
    object HalfCyc: TCheckBox
      Left = 86
      Top = 182
      Width = 85
      Height = 18
      Caption = 'Half Cyc Corr'
      TabOrder = 21
    end
    object SortSats: TCheckBox
      Left = 175
      Top = 182
      Width = 44
      Height = 18
      Caption = 'Sort'
      TabOrder = 22
    end
    object OutIono: TCheckBox
      Left = 222
      Top = 182
      Width = 71
      Height = 18
      Caption = 'Iono Corr'
      TabOrder = 23
    end
    object OutTime: TCheckBox
      Left = 293
      Top = 182
      Width = 68
      Height = 18
      Caption = 'Time Corr'
      TabOrder = 24
    end
    object OutLeaps: TCheckBox
      Left = 364
      Top = 182
      Width = 72
      Height = 18
      Caption = 'Leap Sec'
      TabOrder = 25
    end
  end
  object GroupBox3: TGroupBox
    Left = -1
    Top = 263
    Width = 68
    Height = 60
    Caption = 'Obs Types'
    TabOrder = 8
    object Obs1: TCheckBox
      Left = 5
      Top = 14
      Width = 30
      Height = 15
      Caption = 'C'
      Checked = True
      State = cbChecked
      TabOrder = 0
    end
    object Obs2: TCheckBox
      Left = 32
      Top = 14
      Width = 30
      Height = 15
      Caption = 'L'
      Checked = True
      State = cbChecked
      TabOrder = 1
    end
    object Obs3: TCheckBox
      Left = 4
      Top = 32
      Width = 30
      Height = 15
      Caption = 'D'
      Checked = True
      State = cbChecked
      TabOrder = 2
    end
    object Obs4: TCheckBox
      Left = 32
      Top = 32
      Width = 30
      Height = 15
      Caption = 'S'
      Checked = True
      State = cbChecked
      TabOrder = 3
    end
  end
  object TraceLevel: TComboBox
    Left = 189
    Top = 354
    Width = 60
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 11
    Text = 'OFF'
    Items.Strings = (
      'OFF'
      'Level 1'
      'Level 2'
      'Level 3'
      'Level 4'
      'Level 5')
  end
  object RcvOption: TEdit
    Left = 98
    Top = 326
    Width = 315
    Height = 21
    TabOrder = 10
  end
  object GroupBox4: TGroupBox
    Left = 73
    Top = 263
    Width = 312
    Height = 60
    Caption = 'GNSS Signals       '
    TabOrder = 9
    object BtnFreq: TSpeedButton
      Left = 71
      Top = -2
      Width = 16
      Height = 16
      Caption = '?'
      Flat = True
      Spacing = 0
      OnClick = BtnFreqClick
    end
    object Freq1: TCheckBox
      Left = 10
      Top = 14
      Width = 81
      Height = 17
      Caption = 'F1'
      Checked = True
      State = cbChecked
      TabOrder = 0
    end
    object Freq2: TCheckBox
      Left = 101
      Top = 13
      Width = 98
      Height = 17
      Caption = 'F2'
      Checked = True
      State = cbChecked
      TabOrder = 1
    end
    object Freq3: TCheckBox
      Left = 205
      Top = 12
      Width = 104
      Height = 17
      Caption = 'F3'
      State = cbChecked
      TabOrder = 2
    end
    object Freq4: TCheckBox
      Left = 10
      Top = 32
      Width = 77
      Height = 17
      Caption = 'F4'
      State = cbChecked
      TabOrder = 4
    end
    object Freq5: TCheckBox
      Left = 101
      Top = 32
      Width = 70
      Height = 17
      Caption = 'F5'
      State = cbChecked
      TabOrder = 5
    end
    object Freq6: TCheckBox
      Left = 205
      Top = 32
      Width = 43
      Height = 17
      Caption = 'F6'
      State = cbChecked
      TabOrder = 3
    end
  end
  object ExSats: TEdit
    Left = 343
    Top = 241
    Width = 91
    Height = 21
    TabOrder = 12
  end
  object RnxVer: TComboBox
    Left = 67
    Top = 2
    Width = 60
    Height = 21
    Style = csDropDownList
    ItemIndex = 0
    TabOrder = 2
    Text = '3.04'
    OnChange = RnxVerChange
    Items.Strings = (
      '2.10'
      '2.11'
      '2.12'
      '3.00'
      '3.01'
      '3.02'
      '3.03'
      '3.04'
      '3.05'
      '4.00'
      '4.01'
      '4.02')
  end
  object RnxFile: TCheckBox
    Left = 327
    Top = 5
    Width = 84
    Height = 17
    Caption = 'RINEX2 Name'
    TabOrder = 5
    OnClick = RnxFileClick
  end
  object RnxCode: TEdit
    Left = 260
    Top = 3
    Width = 60
    Height = 21
    TabOrder = 4
    Text = '0000'
  end
  object GroupBox2: TGroupBox
    Left = 1
    Top = 228
    Width = 340
    Height = 35
    Caption = 'Satellite Systems'
    TabOrder = 7
    object Nav1: TCheckBox
      Left = 10
      Top = 14
      Width = 40
      Height = 15
      Caption = 'GPS'
      Checked = True
      State = cbChecked
      TabOrder = 0
    end
    object Nav2: TCheckBox
      Left = 58
      Top = 14
      Width = 40
      Height = 15
      Caption = 'GLO'
      Checked = True
      State = cbChecked
      TabOrder = 1
    end
    object Nav3: TCheckBox
      Left = 107
      Top = 14
      Width = 40
      Height = 15
      Caption = 'GAL'
      Checked = True
      State = cbChecked
      TabOrder = 2
    end
    object Nav4: TCheckBox
      Left = 152
      Top = 14
      Width = 40
      Height = 15
      Caption = 'QZS'
      State = cbChecked
      TabOrder = 3
    end
    object Nav5: TCheckBox
      Left = 298
      Top = 14
      Width = 48
      Height = 15
      Caption = 'SBS'
      State = cbChecked
      TabOrder = 6
    end
    object Nav6: TCheckBox
      Left = 197
      Top = 14
      Width = 40
      Height = 15
      Caption = 'BDS'
      TabOrder = 4
    end
    object Nav7: TCheckBox
      Left = 241
      Top = 14
      Width = 52
      Height = 15
      Caption = 'NavIC'
      TabOrder = 5
    end
  end
  object BtnMask: TButton
    Left = 388
    Top = 268
    Width = 43
    Height = 26
    Caption = 'Mask...'
    TabOrder = 13
    OnClick = BtnMaskClick
  end
  object ChkSepNav: TCheckBox
    Left = 137
    Top = 5
    Width = 66
    Height = 17
    Caption = 'Sep NAV'
    TabOrder = 3
  end
  object TimeTol: TEdit
    Left = 103
    Top = 354
    Width = 45
    Height = 28
    TabOrder = 14
    Text = '0.005'
  end
  object BtnFcn: TButton
    Left = 388
    Top = 295
    Width = 43
    Height = 26
    Caption = 'FCN...'
    TabOrder = 15
    OnClick = BtnFcnClick
  end
end
