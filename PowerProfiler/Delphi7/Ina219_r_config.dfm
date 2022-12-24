object Form219Config: TForm219Config
  Left = 528
  Top = 274
  BorderStyle = bsDialog
  Caption = 'INA219: Registor Config'
  ClientHeight = 348
  ClientWidth = 555
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
  object LabelRegConfig: TLabel
    Left = 312
    Top = 188
    Width = 68
    Height = 13
    Caption = 'RegConfig = ?'
  end
  object Label1: TLabel
    Left = 432
    Top = 188
    Width = 20
    Height = 13
    Caption = 'CLK'
  end
  object Label2: TLabel
    Left = 528
    Top = 188
    Width = 20
    Height = 13
    Caption = 'KHz'
  end
  object LabelUz: TLabel
    Left = 312
    Top = 212
    Width = 37
    Height = 13
    Caption = 'U offset'
  end
  object Label7: TLabel
    Left = 312
    Top = 236
    Width = 32
    Height = 13
    Caption = 'U koef'
  end
  object LabelIz: TLabel
    Left = 312
    Top = 260
    Width = 32
    Height = 13
    Caption = 'I offset'
  end
  object LabelIk: TLabel
    Left = 320
    Top = 284
    Width = 27
    Height = 13
    Caption = 'I koef'
  end
  object RadioGroupBusRange: TRadioGroup
    Left = 8
    Top = 48
    Width = 145
    Height = 49
    Caption = 'Bus Voltage Range '
    ItemIndex = 0
    Items.Strings = (
      '0-16V Range'
      '0-32V Range')
    TabOrder = 0
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object RadioGroupGain: TRadioGroup
    Left = 160
    Top = 8
    Width = 145
    Height = 89
    Caption = 'Shunt Gain '
    ItemIndex = 3
    Items.Strings = (
      'Gain 1, 40mV Range'
      'Gain 2, 80mV Range'
      'Gain 4, 160mV Range'
      'Gain 8, 320mV Range')
    TabOrder = 1
    OnClick = RadioGroupGainClick
    OnEnter = RadioGroupGainClick
  end
  object RadioGroupShuntADCRes: TRadioGroup
    Left = 160
    Top = 104
    Width = 145
    Height = 225
    Caption = 'Shunt ADC Resolution'
    ItemIndex = 3
    Items.Strings = (
      '1 x 9-bit, 84 us'
      '1 x 10-bit, 148 us'
      '1 x 11-bit, 276 us'
      '1 x 12-bit, 532 us'
      '1 x 12-bit, 532 us'
      '1 x 12-bit, 1.06 ms'
      '4 x 12-bit, 2.13 ms'
      '8 x 12-bit, 4.26 ms'
      '16 x 12-bit, 8.51 ms'
      '32 x 12-bit, 17.02 ms'
      '64 x 12-bit, 34.05 ms'
      '128 x 12-bit, 68.10 ms')
    TabOrder = 2
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object RadioGroupMode: TRadioGroup
    Left = 312
    Top = 16
    Width = 161
    Height = 161
    Caption = 'Mode '
    ItemIndex = 0
    Items.Strings = (
      'Power-down'
      'Shunt voltage, triggered'
      'Bus voltage, triggered'
      'Shunt and bus, triggered'
      'ADC off (disabled)'
      'Shunt voltage, continuous'
      'Bus voltage, continuous'
      'Shunt and bus, continuous')
    TabOrder = 3
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object ButtonOk: TButton
    Left = 360
    Top = 312
    Width = 75
    Height = 25
    Caption = 'Ok'
    TabOrder = 4
    OnClick = ButtonOkClick
  end
  object ButtonCancel: TButton
    Left = 464
    Top = 312
    Width = 75
    Height = 25
    Caption = 'Cancel'
    ModalResult = 2
    TabOrder = 5
  end
  object RadioGroupBusADCRes: TRadioGroup
    Left = 8
    Top = 104
    Width = 145
    Height = 225
    Caption = 'Bus ADC Resolution'
    ItemIndex = 3
    Items.Strings = (
      '1 x 9-bit, 84 us'
      '1 x 10-bit, 148 us'
      '1 x 11-bit, 276 us'
      '1 x 12-bit, 532 us'
      '1 x 12-bit, 532 us'
      '1 x 12-bit, 1.06 ms'
      '4 x 12-bit, 2.13 ms'
      '8 x 12-bit, 4.26 ms'
      '16 x 12-bit, 8.51 ms'
      '32 x 12-bit, 17.02 ms'
      '64 x 12-bit, 34.05 ms'
      '128 x 12-bit, 68.10 ms')
    TabOrder = 6
    OnClick = ChargeReg
    OnEnter = ChargeReg
  end
  object GroupBox1: TGroupBox
    Left = 8
    Top = 8
    Width = 145
    Height = 33
    Caption = 'Reset'
    TabOrder = 7
    object CheckBoxReset: TCheckBox
      Left = 8
      Top = 12
      Width = 65
      Height = 17
      Caption = 'Reset'
      TabOrder = 0
      OnClick = ChargeReg
      OnEnter = ChargeReg
    end
  end
  object SpinEditCLkKHz: TSpinEdit
    Left = 456
    Top = 184
    Width = 65
    Height = 22
    Increment = 50
    MaxValue = 1200
    MinValue = 50
    TabOrder = 8
    Value = 1000
    OnChange = SpinEditCLkKHzChange
  end
  object EditIk: TEdit
    Left = 360
    Top = 282
    Width = 89
    Height = 21
    Hint = #1057#1084#1077#1097#1077#1085#1080#1077' '#1076#1083#1103' I'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 9
    Text = '?.?'
  end
  object EditIz: TEdit
    Left = 360
    Top = 258
    Width = 89
    Height = 21
    Hint = #1057#1084#1077#1097#1077#1085#1080#1077' '#1076#1083#1103' I'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 10
    Text = '?.?'
  end
  object EditUk: TEdit
    Left = 360
    Top = 234
    Width = 89
    Height = 21
    Hint = #1050#1086#1101#1092'. '#1076#1083#1103' U'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 11
    Text = '?.?'
  end
  object EditUz: TEdit
    Left = 360
    Top = 210
    Width = 89
    Height = 21
    Hint = #1057#1084#1077#1097#1077#1085#1080#1077' '#1076#1083#1103' U'
    ParentShowHint = False
    ShowHint = True
    TabOrder = 12
    Text = '?.?'
  end
  object ButtonCopyUz: TButton
    Left = 456
    Top = 208
    Width = 89
    Height = 25
    Caption = 'Copy Us to Uz'
    TabOrder = 13
    OnClick = ButtonCopyUzClick
  end
  object ButtonCopyIz: TButton
    Left = 456
    Top = 256
    Width = 89
    Height = 25
    Caption = 'Copy Is to Iz'
    TabOrder = 14
    OnClick = ButtonCopyIzClick
  end
end
