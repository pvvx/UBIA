unit adc_jdy10_config;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls, ExtCtrls, Spin;
const
  ADCshlcount = 11;

type
  TFormAdcConfig = class(TForm)
    RadioGroupAdcChannels: TRadioGroup;
    ButtonOk: TButton;
    ButtonCancel: TButton;
    LabelUz: TLabel;
    LabelIz: TLabel;
    EditUz: TEdit;
    EditIz: TEdit;
    ButtonCopyUz: TButton;
    ButtonCopyIz: TButton;
    SpinEditAdcSmps: TSpinEdit;
    LabelSPS: TLabel;
    EditUk: TEdit;
    Label1: TLabel;
    EditIk: TEdit;
    LabelIk: TLabel;
    RadioGroupUIadc: TRadioGroup;
    GroupBox1: TGroupBox;
    CheckBoxPC1PGA20: TCheckBox;
    CheckBoxPC2PGA20: TCheckBox;
    SpinEditPC1PGA: TSpinEdit;
    SpinEditPC2PGA: TSpinEdit;
    Label2: TLabel;
    Label3: TLabel;
    CheckBoxPd5Diff: TCheckBox;
    procedure FormActivate(Sender: TObject);
    procedure ButtonCopyUzClick(Sender: TObject);
    procedure ButtonCopyIzClick(Sender: TObject);
    procedure ButtonOkClick(Sender: TObject);
    procedure ShowScrParms;
    procedure GetScrParms;
    procedure GetParams;
//    procedure SetParamIU(i : double ; u : double);
  private
    { Private declarations }
    ChgEna : boolean;
  public
    { Public declarations }
  end;

var
  FormAdcConfig: TFormAdcConfig;

  ADC_channel : byte = 9;
  ADC_smps : dword = 500;
  PGA20db : byte;
  PGA2db5 : byte;

  UI_adc : integer = 0;

  Uk_adc : double = 0.00001983642578125;
  Uz_adc : double = -0.009799;
  Ik_adc : double = 0.00039673;
  Iz_adc : double = -0.0071562;

  Adc_channels_tab : array [0..ADCshlcount - 1] of byte = (
    $05,  // PD4
    $06,  // PD5
    $07,  // PC2
    $09,  // PC4
    $0b,  // PC6
    $0c,  // PC7
    $12,  // GND
    $6D,  // DIFF, PGA
    $8E,  // PC1
    $6E,  // PC1, PGA
    $8D   // PC2, PGA
  );

implementation

{$R *.dfm}
Uses MainFrm;


procedure TFormAdcConfig.GetScrParms;
begin
    DecimalSeparator := '.';
    Uk_adc := StrToFloat(EditUk.Text);
    Uz_adc := StrToFloat(EditUz.Text);
    Ik_adc := StrToFloat(EditIk.Text);
    Iz_adc := StrToFloat(EditIz.Text);

    PGA20db := 0;
    if CheckBoxPC1PGA20.Checked then
      PGA20db := 1 shl 5;
    if CheckBoxPC2PGA20.Checked then
      PGA20db := PGA20db or (1 shl 6);
    PGA2db5 := (SpinEditPC1PGA.Value and $0f) + ((SpinEditPC2PGA.Value and $0f) shl 4); 

    UI_adc := RadioGroupUIadc.ItemIndex;
    ADC_channel := Adc_channels_tab[RadioGroupAdcChannels.ItemIndex];
    if CheckBoxPd5Diff.Checked then begin
       ADC_channel := (ADC_channel and $9F) or $20;
    end;
    ADC_smps := SpinEditAdcSmps.Value;
end;

procedure TFormAdcConfig.FormActivate(Sender: TObject);
begin
     ChgEna := False;
     ShowScrParms;
     ChgEna := True;
end;

procedure TFormAdcConfig.GetParams;
begin
  if (UI_adc <> 0) then begin
    U_zero := Uz_adc;
    Uk := Uk_adc;
  end else begin
    I_zero := Iz_adc;
    Ik := Ik_adc;
  end;
end;

procedure TFormAdcConfig.ShowScrParms;
var
i : integer;
begin
    DecimalSeparator := '.';
    EditUz.Text:=FormatFloat('0.00000000', Uz_adc);
    EditUk.Text:=FormatFloat('0.00000000', Uk_adc);
    EditIz.Text:=FormatFloat('0.00000000', Iz_adc);
    EditIk.Text:=FormatFloat('0.00000000', Ik_adc);
    RadioGroupUIadc.ItemIndex := UI_adc;
//    EditIz.Text:=FormatFloat('0.00000000', Iz_adc);
//    ADC_channel := (ADC_channel and $9F) or $20;
    CheckBoxPd5Diff.Checked := (ADC_channel and $60) = $20;
    for i:=0 to ADCshlcount do begin
      if ADC_channel = Adc_channels_tab[i] then break;
    end;
    RadioGroupAdcChannels.ItemIndex := i;
    SpinEditAdcSmps.Value := ADC_smps;
    if (PGA20db and (1 shl 5)) <> 0 then
      CheckBoxPC1PGA20.Checked := True
    else
      CheckBoxPC1PGA20.Checked := False;
    if (PGA20db and (1 shl 6)) <> 0 then
      CheckBoxPC2PGA20.Checked := True
    else
      CheckBoxPC2PGA20.Checked := False;
    SpinEditPC1PGA.Value := PGA2db5 and $0f;
    SpinEditPC2PGA.Value := (PGA2db5 shr 4) and $0f;
end;

procedure TFormAdcConfig.ButtonCopyUzClick(Sender: TObject);
begin
    EditUz.Text:=FormatFloat('0.00000000', -OldsU);
end;

procedure TFormAdcConfig.ButtonCopyIzClick(Sender: TObject);
begin
    EditIz.Text:=FormatFloat('0.00000000', -OldsI);
end;

procedure TFormAdcConfig.ButtonOkClick(Sender: TObject);
begin
    GetScrParms;
    ModalResult := mrOk;
    Exit;
end;

end.
