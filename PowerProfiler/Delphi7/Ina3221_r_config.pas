unit Ina3221_r_config;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls, ExtCtrls, Spin;

const
  // Configuration Register (00h) (Read/Write)

  ConfigModeShl = 0;
  ConfigModeMsk = 7;
  ConfigModeCnt = 8;

  ShuntCTimeShl = 3;
  ShuntCTimeMsk = $0038;
  ShuntCTimeCnt = 8;

  BusCTimeShl = 6;
  BusCTimeMsk = $01c0;
  BusCTimeCnt = 8;

  AveragingModeShl = 9;
  AveragingModeMsk = $0e00;
  AveragingModeCnt = 8;

  ChannelEnableShl = 12;
  ChannelEnableMsk = $7000;
  ChannelEnableCnt = 3;


  ConfigResetMsk = $8000;

  // Mask/Enable Register (06h) (Read/Write)

  ShuntOverVoltageShl = 15;
  ShuntUnderVoltageShl = 14;
  BusOverVoltageShl = 13;
  BusUnderVoltageShl = 12;
  PowerOverLimitShl = 11;
  ConversionReadyShl = 10;
  AlertFunctionFlagShl = 4;
  ConversionReadyFlagShl = 3;
  MathOverflowFlagShl = 2;
  AlertPolarityShl = 1;
  AlertLatchEnableShl = 0;

  ShuntOverVoltageMsk = $8000;
  ShuntUnderVoltageMsk = $4000;
  BusOverVoltageMsk = $2000;
  BusUnderVoltageMsk = $1000;
  PowerOverLimitMsk = $0800;
  ConversionReadyMsk = $0400;
  AlertFunctionFlagMsk = $0010;
  ConversionReadyFlagMsk = $0008;
  MathOverflowFlagMsk = $0004;
  AlertPolarityMsk = $0002;
  AlertLatchEnableMsk = $0001;

  //
  SMBus_3221_Speed_Max_kHz = 2440; // Chip Max 2440 kHz
  SMBus_3221_Speed_kHz = 1000; // default
  SMBus_3221_Speed_Min_kHz = 100;

type
  TForm3221Config = class(TForm)
    ButtonOk: TButton;
    ButtonCancel: TButton;
    GroupBox1: TGroupBox;
    CheckBoxReset: TCheckBox;
    RadioGroupBusCTime: TRadioGroup;
    RadioGroupShuntCTime: TRadioGroup;
    RadioGroupMode: TRadioGroup;
    RadioGroupAverageMode: TRadioGroup;
    Label1: TLabel;
    Label3: TLabel;
    EditRegConfig: TEdit;
    SpinEditCLkKHz: TSpinEdit;
    EditUz: TEdit;
    ButtonCopyUz: TButton;
    ButtonCopyIz: TButton;
    EditIz: TEdit;
    LabelUz: TLabel;
    LabelIz: TLabel;
    RadioGroupChannels: TRadioGroup;
    EditIk: TEdit;
    LabelIk: TLabel;
    EditUk: TEdit;
    Label4: TLabel;
    procedure ShowAll;
    procedure GetParams;
    procedure GetScrParms;
    procedure ShowScrParms;
    function DevIniCfg(mode : integer) : byte;
    procedure ButtonCopyUzClick(Sender: TObject);
    procedure ButtonCopyIzClick(Sender: TObject);
    procedure ButtonOkClick(Sender: TObject);
    procedure FormActivate(Sender: TObject);
    procedure ChargeReg(Sender: TObject);
    procedure RadioGroupChannelsClick(Sender: TObject);
    procedure SpinEditCLkKHzChange(Sender: TObject);
  private
    { Private declarations }
    ChgEna : boolean;
    procedure CheckRegValue;
  public
    { Public declarations }
    reg_config : word;
//    reg_calibration : word;
//    reg_mask_enable : word;
//    reg_alert_limit : word;
  end;

var
  Form3221Config: TForm3221Config;

  TabBusClkTiming : array [0..ShuntCTimeCnt-1] of word =
   (1000, 800, 800, 800,
    800, 800, 800, 800);

  TabTimerCTime : array [0..BusCTimeCnt-1] of word =
   (140, 204, 332, 588,
    1100, 2116, 4156, 8244);

  // Voltage: Full-scale range = 32.76 V, (decimal = 7FF8); LSB: 1 мV, step 8 mV
  // Shunt: Full-scale range = 163.8 mV, (decimal = 7FF8); LSB: 5 uV, step 40 uV

  chl_3221 : integer = 2; // 0,1,2 -> channel 1,2,3
  Ik_3221_ch : array [0..2] of double =
    ( 0.005, 0.005, 0.005 ); // 5 uV/bit
  Uk_3221_ch : array [0..2] of  double =
    ( 0.001, 0.001, 0.001 ); // 1 mV/bit
  I_3221_zero_ch : array [0..2] of double =
    ( 0.0, 0.0, 0.0 );
  U_3221_zero_ch : array [0..2] of double =
    ( 0.0, 0.0, 0.0 );
  Clk_3221 : integer = SMBus_3221_Speed_kHz;

implementation

{$R *.dfm}
Uses MainFrm;

function Str2dword(const s: string): Dword;
var
 i: integer;
 o : string;
 hex : boolean;
begin
    i := 1;
    hex := False;
//  result := 0;
    while (i <= Length(s)) do begin
      if ((s[i] = '0') and (s[i+1] = 'x')) then begin
       hex := True;
       inc(i);
      end
      else if (s[i] = '$') then  hex := True
      else if (((s[i] >= 'a') and (s[i] <= 'f'))
      or ((s[i] >= 'A') and (s[i] <= 'F'))) then begin
       hex := True;
       o := o + s[i];
      end
      else if ((s[i] >= '0') and (s[i] <= '9')) then o := o + s[i];
      inc(i);
    end;
    if hex then result := StrToIntDef('$'+o,0)
    else result := StrToIntDef(o,0)
end;

procedure TForm3221Config.ButtonCopyUzClick(Sender: TObject);
begin
    EditUz.Text:=FormatFloat('0.00000000', -OldsU);
    U_zero := -OldsU;
end;

procedure TForm3221Config.ButtonCopyIzClick(Sender: TObject);
begin
    EditIz.Text:=FormatFloat('0.00000000', -OldsI);
    I_zero := -OldsI;
end;

procedure TForm3221Config.ButtonOkClick(Sender: TObject);
var
 min_clk_khz, clk_khz : integer;
begin
    CheckRegValue;

    if RadioGroupShuntCTime.ItemIndex < RadioGroupBusCTime.ItemIndex then
      min_clk_khz := TabBusClkTiming[RadioGroupShuntCTime.ItemIndex]
    else
      min_clk_khz := TabBusClkTiming[RadioGroupBusCTime.ItemIndex];
    clk_khz := SpinEditCLkKHz.Value;
    if clk_khz < min_clk_khz then begin
        ShowMessage('BUS CLK Low!' + #13#10 +'Min CLK ' + IntToStr(clk_khz) + ' kHz!');
        SpinEditCLkKHz.Value := min_clk_khz;
        ModalResult := mrNone;
        Exit;
    end else if clk_khz > SMBus_3221_Speed_Max_kHz then begin
        clk_khz := SMBus_3221_Speed_Max_kHz;
        SpinEditCLkKHz.Value := clk_khz;
    end;


    blk_cfg.clk_khz := clk_khz;

    ina_reg.config := reg_config;
//    ina_reg.ina2_calibration := reg_calibration;
//    ina_reg.ina2_mask_enable := reg_mask_enable;
//    ina_reg.ina2_alert_data := reg_alert_limit;

    GetScrParms;
    ShowScrParms;

    ModalResult := mrOk;
    Exit;
end;

procedure TForm3221Config.GetScrParms;
begin
    DecimalSeparator := '.';
    U_3221_zero_ch[chl_3221] := StrToFloat(EditUz.Text);
    I_3221_zero_ch[chl_3221] := StrToFloat(EditIz.Text);
    Uk_3221_ch[chl_3221] := StrToFloat(EditUk.Text);
    Ik_3221_ch[chl_3221] := StrToFloat(EditIk.Text);
end;

procedure TForm3221Config.ShowScrParms;
begin
    DecimalSeparator := '.';
    EditUz.Text := FormatFloat('0.00000000', U_3221_zero_ch[chl_3221]);
    EditIz.Text := FormatFloat('0.00000000', I_3221_zero_ch[chl_3221]);
    EditUk.Text := FormatFloat('0.00000000', Uk_3221_ch[chl_3221]);
    EditIk.Text := FormatFloat('0.00000000', Ik_3221_ch[chl_3221]);
end;


procedure TForm3221Config.ShowAll;
var
x : dword;
begin
     reg_config := ina_reg.config;
     ChgEna := False;
     if (reg_config and ConfigResetMsk) <> 0 then
        CheckBoxReset.Checked := True
     else
       CheckBoxReset.Checked := False;

     RadioGroupBusCTime.ItemIndex := (reg_config and BusCTimeMsk) shr BusCTimeShl;
     RadioGroupShuntCTime.ItemIndex := (reg_config and ShuntCTimeMsk) shr ShuntCTimeShl;
     RadioGroupAverageMode.ItemIndex := (reg_config and AveragingModeMsk) shr AveragingModeShl;
     x := (reg_config and ChannelEnableMsk) shr  ChannelEnableShl;
     if x > 3 then
       chl_3221 := 0
     else if x > 1 then
       chl_3221 := 1
     else
       chl_3221 := 2;
     RadioGroupChannels.ItemIndex := chl_3221;
     RadioGroupMode.ItemIndex := reg_config and ConfigModeMsk;
     ShowScrParms;
     EditRegConfig.Text := '0x' + IntToHex(reg_config, 4);
     if Clk_3221 > SpinEditCLkKHz.MaxValue then
       SpinEditCLkKHz.Value := SpinEditCLkKHz.MaxValue
     else
       SpinEditCLkKHz.Value := Clk_3221;
     ChgEna := True;
end;

procedure TForm3221Config.FormActivate(Sender: TObject);
begin
    if (dev_type = HI_DEVICE_TYPE) then begin
      SpinEditCLkKHz.MaxValue := SMBus_3221_Speed_Max_kHz;
    end
    else begin
       SpinEditCLkKHz.MaxValue := 1000;
    end;
    ShowAll;
end;

procedure TForm3221Config.CheckRegValue;
begin
     chl_3221 := RadioGroupChannels.ItemIndex;
     reg_config := (RadioGroupMode.ItemIndex shl ConfigModeShl) and ConfigModeMsk;
     reg_config := reg_config or ((RadioGroupBusCTime.ItemIndex shl BusCTimeShl) and BusCTimeMsk);
     reg_config := reg_config or ((RadioGroupShuntCTime.ItemIndex shl ShuntCTimeShl) and ShuntCTimeMsk);
     reg_config := reg_config or ((RadioGroupAverageMode.ItemIndex shl AveragingModeShl) and AveragingModeMsk);
     reg_config := reg_config or ((($04 shl ChannelEnableShl) shr RadioGroupChannels.ItemIndex) and ChannelEnableMsk);
     if CheckBoxReset.Checked then reg_config := reg_config or ConfigResetMsk;
     EditRegConfig.Text := '0x' + IntToHex(reg_config, 4);
     GetScrParms;
end;

procedure TForm3221Config.GetParams;
begin
  I_zero := I_3221_zero_ch[chl_3221];
  U_zero := U_3221_zero_ch[chl_3221];
  Uk := Uk_3221_ch[chl_3221];
  Ik := Ik_3221_ch[chl_3221];
end;

function TForm3221Config.DevIniCfg(mode : integer) : byte;
var
  mask : word;
  chlx2, u, i, x : dword;
begin
     // инициализировать в INA219 регистры:
     // записать config регистр:
     blk_cfg.init[0].dev_addr := INA2XX_I2C_ADDR;
     blk_cfg.init[0].reg_addr := 0;
     blk_cfg.init[0].data := ina_reg.config;
     // записать другие регистры ?:
     blk_cfg.init[1].dev_addr := INA2XX_I2C_ADDR;
     blk_cfg.init[1].reg_addr := $10; // Power-Valid Upper-Limit Register (address = 10h) [reset = 2710h]
     blk_cfg.init[1].data := 3250; // mV

     blk_cfg.init[2].dev_addr := INA2XX_I2C_ADDR;
     blk_cfg.init[2].reg_addr := $11; // Power-Valid Lower-Limit Register (address = 11h) [reset = 2328h]
     blk_cfg.init[2].data := 3200; // mV

     // записать другие регистры ?:
     blk_cfg.init[3].dev_addr := INA2XX_I2C_ADDR;
     blk_cfg.init[3].reg_addr := $0F; // Mask/Enable Register (address = 0Fh) [reset = 0002h]
     blk_cfg.init[3].data := $7000;

//     blk_cfg.init[3].dev_addr := 0;

     // min 140, max 8244*1024 = 8441856 us
     u := (ina_reg.config and BusCTimeMsk) shr BusCTimeShl;
     u := TabTimerCTime[u];
     i := (ina_reg.config and ShuntCTimeMsk) shr ShuntCTimeShl;
     i := TabTimerCTime[i];
     x := (ina_reg.config and AveragingModeMsk) shr AveragingModeShl;
     mask := ina_reg.config and ConfigModeMsk;
     chlx2 := (ina_reg.config and ChannelEnableMsk) shr ChannelEnableShl;
     if chlx2 > 3 then
        chlx2 := 0*2
     else if chlx2 > 1 then
        chlx2 := 1*2
     else
        chlx2 := 2*2;
     // задать адреса регистров чтения
     case mask of
         5 : begin // Shunt
             result := CHART_I_MASK;
             // задать адреса устройства чтения
             blk_cfg.data[0].dev_addr := INA2XX_I2C_ADDR;
             blk_cfg.data[0].reg_addr := 1 + chlx2;
             blk_cfg.data[1].dev_addr := 0; // stop rd
             blk_cfg.time_us := i;
             blk_cfg.multiplier := x;
            end;
         6 : begin // Bus
             result := CHART_U_MASK;
             // задать адреса устройства чтения
             blk_cfg.data[0].dev_addr := INA2XX_I2C_ADDR;
             blk_cfg.data[0].reg_addr := 2 + chlx2;
             blk_cfg.data[1].dev_addr := 0; // stop rd
             blk_cfg.time_us := u;
             blk_cfg.multiplier := x;
            end;
         7 : begin // Shunt + Bus
             result := CHART_UI_MASK;
             // задать адреса устройства чтения
             blk_cfg.data[0].dev_addr := INA2XX_I2C_ADDR;
             blk_cfg.data[0].reg_addr := 1 + chlx2;
             blk_cfg.data[1].dev_addr := INA2XX_I2C_ADDR;
             blk_cfg.data[1].reg_addr := 2 + chlx2;
             blk_cfg.data[2].dev_addr := 0;  // stop rd
             blk_cfg.time_us := (i + u) shr 1;
             blk_cfg.multiplier := x;
            end;
         else begin
            result := 0;
            blk_cfg.data[0].dev_addr := 0;
            blk_cfg.time_us := 10000;
            blk_cfg.multiplier := 0;
            end;
     end;
     if(mode <> 0) then begin
       // задать максимум регистров в пакете
       x := blk_cfg.time_us shl blk_cfg.multiplier;
       x := x div (30000 div 30);
       if(x > 29) then x := 29;
       if mask = 7 then x := x and $fffe;
       blk_cfg.pktcnt := 30 - x;
     end else
       blk_cfg.pktcnt := 0; // stop
     // задать частоту шины i2c в кГц
     blk_cfg.clk_khz := SpinEditCLkKHz.Value;
     if (dev_type = HI_DEVICE_TYPE) and (dev_ver > 8) then
        blk_cfg.clk_khz := blk_cfg.clk_khz or $8000; // Enable clock stretching
end;

procedure TForm3221Config.ChargeReg(Sender: TObject);
begin
    if ChgEna then CheckRegValue;
end;

procedure TForm3221Config.RadioGroupChannelsClick(Sender: TObject);
begin
   if ChgEna then begin
    GetScrParms;
    chl_3221 := RadioGroupChannels.ItemIndex;
    ShowScrParms;
    CheckRegValue;
   end;
end;

procedure TForm3221Config.SpinEditCLkKHzChange(Sender: TObject);
begin
     clk_3221 := SpinEditCLkKHz.Value;
end;

end.
