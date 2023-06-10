object Form3221Config: TForm3221Config
  Left = 1028
  Top = 598
  BorderStyle = bsDialog
  Caption = 'INA3221: Registor Config'
  ClientHeight = 336
  ClientWidth = 479
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
  object Label1: TLabel
    Left = 96
    Top = 220
    Width = 61
    Height = 13
    Caption = 'SMBUS CLK'
  end
  object Label3: TLabel
    Left = 96
    Top = 252
    Width = 30
    Height = 13
    Caption = 'Config'
  end
  object LabelUz: TLabel
    Left = 240
    Top = 220
    Width = 37
    Height = 13
    Caption = 'U offset'
  end
  object LabelIz: TLabel
    Left = 240
    Top = 268
    Width = 32
    Height = 13
    Caption = 'I offset'
  end
  object LabelIk: TLabel
    Left = 240
    Top = 292
    Width = 27
    Height = 13
    Caption = 'I koef'
  end
  object Label4: TLabel
    Left = 240
    Top = 244
    Width = 32
    Height = 13
    Caption = 'U koef'
  end
  object ButtonOk: TButton
    Left = 32
    Top = 304
    Width = 75
    Height = 25
    Caption = 'Ok'
    TabOrder = 0
    OnClick = ButtonOkClick
  end
  object ButtonCancel: TButton
    Left = 128
    Top = 304
    Width = 75
    Height = 25
    Caption = 'Cancel'
    ModalResult = 2
    TabOrder = 1
  end
  object GroupBox1: TGroupBox
    Left = 8
    Top = 8
    Width = 185
    Height = 33
    Caption = 'Reset'
    TabOrder = 2
    object CheckBoxReset: TCheckBox
      Left = 8
      Top = 12
      Width = 65
      Height = 17
      Caption = 'Reset'
      TabOrder = 0
    end
  end
  object RadioGroupBusCTime: TRadioGroup
    Left = 8
    Top = 48
    Width = 89
    Height = 161
    Caption = 'Bus CTime'
    ItemIndex = 3
    Items.Strings = (
      '140 us'
      '204 us'
      '332 us'
      '588 us'
      '1.1 ms'
      '2.116 ms'
      '4.156 ms'
      '8.244 ms')
    TabOrder = 3
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object RadioGroupShuntCTime: TRadioGroup
    Left = 104
    Top = 48
    Width = 89
    Height = 161
    Caption = 'Shunt CTime'
    ItemIndex = 3
    Items.Strings = (
      '140 us'
      '204 us'
      '332 us'
      '588 us'
      '1.1 ms'
      '2.116 ms'
      '4.156 ms'
      '8.244 ms')
    TabOrder = 4
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object RadioGroupMode: TRadioGroup
    Left = 200
    Top = 8
    Width = 169
    Height = 201
    Caption = 'Mode '
    ItemIndex = 0
    Items.Strings = (
      'Power-down'
      'Shunt voltage, triggered'
      'Bus voltage, triggered'
      'Shunt and bus, triggered'
      'Power-Down (or Shutdown)'#13
      'Shunt voltage, continuous'
      'Bus voltage, continuous'
      'Shunt and bus, continuous')
    TabOrder = 5
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object RadioGroupAverageMode: TRadioGroup
    Left = 376
    Top = 8
    Width = 97
    Height = 201
    Caption = 'Averaging Mode'
    ItemIndex = 0
    Items.Strings = (
      '1'
      '4'
      '16'
      '64'
      '128'
      '256'
      '512'
      '1024')
    TabOrder = 6
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object EditRegConfig: TEdit
    Left = 160
    Top = 248
    Width = 65
    Height = 21
    Hint = 'Registor Config'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 7
    Text = '?.?'
  end
  object SpinEditCLkKHz: TSpinEdit
    Left = 160
    Top = 216
    Width = 65
    Height = 22
    Increment = 50
    MaxValue = 2400
    MinValue = 400
    TabOrder = 8
    Value = 1000
    OnChange = SpinEditCLkKHzChange
  end
  object EditUz: TEdit
    Left = 288
    Top = 218
    Width = 89
    Height = 21
    Hint = #1057#1084#1077#1097#1077#1085#1080#1077' '#1076#1083#1103' U'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 9
    Text = '?.?'
  end
  object ButtonCopyUz: TButton
    Left = 384
    Top = 216
    Width = 89
    Height = 25
    Caption = 'Copy Us to Uz'
    TabOrder = 10
    OnClick = ButtonCopyUzClick
  end
  object ButtonCopyIz: TButton
    Left = 384
    Top = 264
    Width = 89
    Height = 25
    Caption = 'Copy Is to Iz'
    TabOrder = 11
    OnClick = ButtonCopyIzClick
  end
  object EditIz: TEdit
    Left = 288
    Top = 266
    Width = 89
    Height = 21
    Hint = #1057#1084#1077#1097#1077#1085#1080#1077' '#1076#1083#1103' I'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 12
    Text = '?.?'
  end
  object RadioGroupChannels: TRadioGroup
    Left = 8
    Top = 208
    Width = 73
    Height = 81
    Caption = 'Channel'
    ItemIndex = 0
    Items.Strings = (
      '1'
      '2'
      '3')
    TabOrder = 13
    OnClick = RadioGroupChannelsClick
    OnEnter = RadioGroupChannelsClick
  end
  object EditIk: TEdit
    Left = 288
    Top = 290
    Width = 89
    Height = 21
    Hint = #1050#1086#1101#1092'. '#1076#1083#1103' I'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 14
    Text = '?.?'
  end
  object EditUk: TEdit
    Left = 288
    Top = 242
    Width = 89
    Height = 21
    Hint = #1050#1086#1101#1092'. '#1076#1083#1103' U'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 15
    Text = '?.?'
  end
end
