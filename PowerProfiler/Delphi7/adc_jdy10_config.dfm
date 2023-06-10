object FormAdcConfig: TFormAdcConfig
  Left = 535
  Top = 651
  Width = 487
  Height = 280
  Caption = 'ADC TLSR8266 Config'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  OnActivate = FormActivate
  PixelsPerInch = 96
  TextHeight = 13
  object LabelUz: TLabel
    Left = 232
    Top = 60
    Width = 37
    Height = 13
    Caption = 'U offset'
  end
  object LabelIz: TLabel
    Left = 234
    Top = 12
    Width = 32
    Height = 13
    Caption = 'I offset'
  end
  object LabelSPS: TLabel
    Left = 327
    Top = 156
    Width = 26
    Height = 13
    Caption = 'Smps'
  end
  object Label1: TLabel
    Left = 235
    Top = 84
    Width = 32
    Height = 13
    Caption = 'U coef'
  end
  object LabelIk: TLabel
    Left = 239
    Top = 36
    Width = 27
    Height = 13
    Caption = 'I coef'
  end
  object RadioGroupAdcChannels: TRadioGroup
    Left = 8
    Top = 8
    Width = 137
    Height = 225
    Caption = 'ADC Channel'
    ItemIndex = 3
    Items.Strings = (
      'PD4'
      'PD5'
      'PC2'
      'PC4'
      'PC6'
      'PC7'
      'GND'
      'DIFF PC2/C1, PGA'
      'PC1'
      'PC1, PGA'
      'PC2, PGA')
    TabOrder = 0
  end
  object ButtonOk: TButton
    Left = 296
    Top = 208
    Width = 75
    Height = 25
    Caption = 'Ok'
    TabOrder = 1
    OnClick = ButtonOkClick
  end
  object ButtonCancel: TButton
    Left = 384
    Top = 208
    Width = 75
    Height = 25
    Caption = 'Cancel'
    ModalResult = 2
    TabOrder = 2
  end
  object EditUz: TEdit
    Left = 280
    Top = 58
    Width = 89
    Height = 21
    Hint = #1057#1084#1077#1097#1077#1085#1080#1077' '#1076#1083#1103' U'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 3
    Text = '?.?'
  end
  object EditIz: TEdit
    Left = 280
    Top = 10
    Width = 89
    Height = 21
    Hint = #1057#1084#1077#1097#1077#1085#1080#1077' '#1076#1083#1103' U'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 4
    Text = '?.?'
  end
  object ButtonCopyUz: TButton
    Left = 376
    Top = 56
    Width = 89
    Height = 25
    Caption = 'Copy Us to Uz'
    TabOrder = 5
    OnClick = ButtonCopyUzClick
  end
  object ButtonCopyIz: TButton
    Left = 376
    Top = 8
    Width = 89
    Height = 25
    Caption = 'Copy Is to Iz'
    TabOrder = 6
    OnClick = ButtonCopyIzClick
  end
  object SpinEditAdcSmps: TSpinEdit
    Left = 364
    Top = 152
    Width = 62
    Height = 22
    Hint = #1047#1072#1084#1077#1088#1086#1074' '#1074' '#1089#1077#1082
    Increment = 50
    MaxValue = 50000
    MinValue = 250
    ParentShowHint = False
    ShowHint = True
    TabOrder = 7
    Value = 10000
  end
  object EditUk: TEdit
    Left = 280
    Top = 82
    Width = 89
    Height = 21
    Hint = #1050#1086#1101#1092'. '#1076#1083#1103' U'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 8
    Text = '?.?'
  end
  object EditIk: TEdit
    Left = 280
    Top = 34
    Width = 89
    Height = 21
    Hint = #1050#1086#1101#1092'. '#1076#1083#1103' I'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 9
    Text = '?.?'
  end
  object RadioGroupUIadc: TRadioGroup
    Left = 152
    Top = 8
    Width = 73
    Height = 65
    Hint = #1058#1086#1082' '#1080#1083#1080' '#1085#1072#1087#1088#1103#1078#1077#1085#1080#1077
    Caption = 'I or U'
    ItemIndex = 0
    Items.Strings = (
      'I'
      'U')
    ParentShowHint = False
    ShowHint = True
    TabOrder = 10
  end
  object GroupBox1: TGroupBox
    Left = 152
    Top = 120
    Width = 116
    Height = 113
    Caption = 'PGA'
    TabOrder = 11
    object Label2: TLabel
      Left = 48
      Top = 56
      Width = 59
      Height = 13
      Caption = 'PC1 x 2.5dB'
    end
    object Label3: TLabel
      Left = 48
      Top = 78
      Width = 59
      Height = 13
      Caption = 'PC2 x 2.5dB'
    end
    object CheckBoxPC1PGA20: TCheckBox
      Left = 9
      Top = 16
      Width = 81
      Height = 17
      Caption = 'PC1 +20dB'
      TabOrder = 0
    end
    object CheckBoxPC2PGA20: TCheckBox
      Left = 10
      Top = 32
      Width = 81
      Height = 17
      Caption = 'PC2 +20dB'
      TabOrder = 1
    end
    object SpinEditPC1PGA: TSpinEdit
      Left = 10
      Top = 51
      Width = 33
      Height = 22
      Hint = #1047#1072#1084#1077#1088#1086#1074' '#1074' '#1089#1077#1082
      MaxValue = 9
      MinValue = 0
      ParentShowHint = False
      ShowHint = True
      TabOrder = 2
      Value = 0
    end
    object SpinEditPC2PGA: TSpinEdit
      Left = 10
      Top = 75
      Width = 33
      Height = 22
      Hint = #1047#1072#1084#1077#1088#1086#1074' '#1074' '#1089#1077#1082
      MaxValue = 9
      MinValue = 0
      ParentShowHint = False
      ShowHint = True
      TabOrder = 3
      Value = 0
    end
  end
  object CheckBoxPd5Diff: TCheckBox
    Left = 305
    Top = 120
    Width = 80
    Height = 17
    Caption = 'PD5 - Diff'
    TabOrder = 12
  end
end
