object MaskOptDialog: TMaskOptDialog
  Left = 0
  Top = 0
  BorderStyle = bsDialog
  Caption = 'SNR Mask'
  ClientHeight = 245
  ClientWidth = 372
  Color = clWhite
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  Position = poOwnerFormCenter
  OnShow = FormShow
  TextHeight = 13
  object Label2: TLabel
    Left = 45
    Top = 27
    Width = 14
    Height = 13
    Caption = '<5'
  end
  object Label3: TLabel
    Left = 204
    Top = 8
    Width = 73
    Height = 13
    Caption = 'Elevation (deg)'
  end
  object Label6: TLabel
    Left = 326
    Top = 8
    Width = 32
    Height = 13
    Caption = '(dBHz)'
  end
  object Label4: TLabel
    Left = 83
    Top = 27
    Width = 12
    Height = 13
    Caption = '15'
  end
  object Label5: TLabel
    Left = 119
    Top = 27
    Width = 12
    Height = 13
    Caption = '25'
  end
  object Label7: TLabel
    Left = 156
    Top = 27
    Width = 12
    Height = 13
    Caption = '35'
  end
  object Label8: TLabel
    Left = 192
    Top = 27
    Width = 12
    Height = 13
    Caption = '45'
  end
  object Label9: TLabel
    Left = 228
    Top = 27
    Width = 12
    Height = 13
    Caption = '55'
  end
  object Label10: TLabel
    Left = 264
    Top = 27
    Width = 12
    Height = 13
    Caption = '65'
  end
  object Label11: TLabel
    Left = 300
    Top = 27
    Width = 12
    Height = 13
    Caption = '75'
  end
  object Label12: TLabel
    Left = 332
    Top = 27
    Width = 20
    Height = 13
    Caption = '>85'
  end
  object BtnOk: TButton
    Left = 210
    Top = 205
    Width = 73
    Height = 27
    Caption = '&OK'
    ModalResult = 1
    TabOrder = 0
    OnClick = BtnOkClick
  end
  object BtCcancel: TButton
    Left = 289
    Top = 205
    Width = 73
    Height = 27
    Caption = '&Cancel'
    ModalResult = 2
    TabOrder = 1
  end
  object MaskEna1: TCheckBox
    Left = 10
    Top = 7
    Width = 58
    Height = 17
    Caption = 'Rover'
    TabOrder = 2
    OnClick = MaskEna1Click
  end
  object Panel1: TPanel
    Left = 0
    Top = 41
    Width = 366
    Height = 22
    BevelOuter = bvNone
    TabOrder = 4
    object Label1: TLabel
      Left = 2
      Top = 2
      Width = 33
      Height = 13
      Caption = 'F1'
    end
    object Mask_1_1: TEdit
      Left = 35
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 0
      Text = '0'
    end
    object Mask_1_2: TEdit
      Left = 71
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 1
      Text = '0'
    end
    object Mask_1_3: TEdit
      Left = 107
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 2
      Text = '0'
    end
    object Mask_1_4: TEdit
      Left = 144
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 3
      Text = '0'
    end
    object Mask_1_5: TEdit
      Left = 180
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 4
      Text = '0'
    end
    object Mask_1_6: TEdit
      Left = 216
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 5
      Text = '0'
    end
    object Mask_1_7: TEdit
      Left = 253
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 6
      Text = '0'
    end
    object Mask_1_8: TEdit
      Left = 289
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 7
      Text = '0'
    end
    object Mask_1_9: TEdit
      Left = 325
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 8
      Text = '0'
    end
  end
  object Panel2: TPanel
    Left = 0
    Top = 67
    Width = 366
    Height = 22
    BevelOuter = bvNone
    TabOrder = 5
    object Label13: TLabel
      Left = 2
      Top = 2
      Width = 33
      Height = 13
      Caption = 'F2'
    end
    object Mask_2_1: TEdit
      Left = 35
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 0
      Text = '0'
    end
    object Mask_2_2: TEdit
      Left = 71
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 1
      Text = '0'
    end
    object Mask_2_3: TEdit
      Left = 107
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 2
      Text = '0'
    end
    object Mask_2_4: TEdit
      Left = 144
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 3
      Text = '0'
    end
    object Mask_2_5: TEdit
      Left = 180
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 4
      Text = '0'
    end
    object Mask_2_6: TEdit
      Left = 216
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 5
      Text = '0'
    end
    object Mask_2_7: TEdit
      Left = 253
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 6
      Text = '0'
    end
    object Mask_2_8: TEdit
      Left = 289
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 7
      Text = '0'
    end
    object Mask_2_9: TEdit
      Left = 325
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 8
      Text = '0'
    end
  end
  object Panel3: TPanel
    Left = 0
    Top = 92
    Width = 366
    Height = 22
    BevelOuter = bvNone
    TabOrder = 6
    object Label14: TLabel
      Left = 2
      Top = 2
      Width = 33
      Height = 13
      Caption = 'F3'
    end
    object Mask_3_1: TEdit
      Left = 35
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 0
      Text = '0'
    end
    object Mask_3_2: TEdit
      Left = 71
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 1
      Text = '0'
    end
    object Mask_3_3: TEdit
      Left = 107
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 2
      Text = '0'
    end
    object Mask_3_4: TEdit
      Left = 144
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 3
      Text = '0'
    end
    object Mask_3_5: TEdit
      Left = 180
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 4
      Text = '0'
    end
    object Mask_3_6: TEdit
      Left = 216
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 5
      Text = '0'
    end
    object Mask_3_7: TEdit
      Left = 253
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 6
      Text = '0'
    end
    object Mask_3_8: TEdit
      Left = 289
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 7
      Text = '0'
    end
    object Mask_3_9: TEdit
      Left = 325
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 8
      Text = '0'
    end
  end
  object MaskEna2: TCheckBox
    Left = 66
    Top = 7
    Width = 87
    Height = 17
    Caption = 'Base Station'
    TabOrder = 3
    OnClick = MaskEna1Click
  end
  object Panel4: TPanel
    Left = 0
    Top = 115
    Width = 366
    Height = 22
    BevelOuter = bvNone
    TabOrder = 7
    object Label15: TLabel
      Left = 2
      Top = 2
      Width = 33
      Height = 13
      Caption = 'F4'
    end
    object Mask_4_1: TEdit
      Left = 35
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 0
      Text = '0'
    end
    object Mask_4_2: TEdit
      Left = 71
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 1
      Text = '0'
    end
    object Mask_4_3: TEdit
      Left = 107
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 2
      Text = '0'
    end
    object Mask_4_4: TEdit
      Left = 144
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 3
      Text = '0'
    end
    object Mask_4_5: TEdit
      Left = 180
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 4
      Text = '0'
    end
    object Mask_4_6: TEdit
      Left = 216
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 5
      Text = '0'
    end
    object Mask_4_7: TEdit
      Left = 253
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 6
      Text = '0'
    end
    object Mask_4_8: TEdit
      Left = 289
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 7
      Text = '0'
    end
    object Mask_4_9: TEdit
      Left = 325
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 8
      Text = '0'
    end
  end
  object Panel5: TPanel
    Left = 0
    Top = 140
    Width = 366
    Height = 22
    BevelOuter = bvNone
    TabOrder = 8
    object Label16: TLabel
      Left = 2
      Top = 2
      Width = 33
      Height = 13
      Caption = 'F5'
    end
    object Mask_5_1: TEdit
      Left = 35
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 0
      Text = '0'
    end
    object Mask_5_2: TEdit
      Left = 71
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 1
      Text = '0'
    end
    object Mask_5_3: TEdit
      Left = 107
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 2
      Text = '0'
    end
    object Mask_5_4: TEdit
      Left = 144
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 3
      Text = '0'
    end
    object Mask_5_5: TEdit
      Left = 180
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 4
      Text = '0'
    end
    object Mask_5_6: TEdit
      Left = 216
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 5
      Text = '0'
    end
    object Mask_5_7: TEdit
      Left = 253
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 6
      Text = '0'
    end
    object Mask_5_8: TEdit
      Left = 289
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 7
      Text = '0'
    end
    object Mask_5_9: TEdit
      Left = 325
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 8
      Text = '0'
    end
  end
  object Panel6: TPanel
    Left = 0
    Top = 165
    Width = 366
    Height = 22
    BevelOuter = bvNone
    TabOrder = 9
    object Label17: TLabel
      Left = 2
      Top = 2
      Width = 33
      Height = 13
      Caption = 'F6'
    end
    object Mask_6_1: TEdit
      Left = 35
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 0
      Text = '0'
    end
    object Mask_6_2: TEdit
      Left = 71
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 1
      Text = '0'
    end
    object Mask_6_3: TEdit
      Left = 107
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 2
      Text = '0'
    end
    object Mask_6_4: TEdit
      Left = 144
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 3
      Text = '0'
    end
    object Mask_6_5: TEdit
      Left = 180
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 4
      Text = '0'
    end
    object Mask_6_6: TEdit
      Left = 216
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 5
      Text = '0'
    end
    object Mask_6_7: TEdit
      Left = 253
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 6
      Text = '0'
    end
    object Mask_6_8: TEdit
      Left = 289
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 7
      Text = '0'
    end
    object Mask_6_9: TEdit
      Left = 325
      Top = 0
      Width = 36
      Height = 21
      TabOrder = 8
      Text = '0'
    end
  end
end
