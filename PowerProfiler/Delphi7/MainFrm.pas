unit MainFrm;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls, ComCtrls, Spin, ExtCtrls, Buttons, TeEngine, Series,
  TeeProcs, Chart, ArrowCha, TeeOpenGL, DB, TeeData, TeeGalleryPanel,
  TeeDraw3D, TeeEdit, TeeComma, TeCanvas, TeeStore, TeeSurfa, TeePoin3,
  TeeSeriesStats,  IniFiles;

const
    INA2XX_I2C_ADDR = $80;
    I2C_DEVICE_ID = $16;
    ADC1_DEVICE_ID = $21;
    ADC2_DEVICE_ID = $22;
    HI_DEVICE_TYPE = $10;

    INAxxx_MID_REG = $fe;
    INAxxx_DID_REG = $ff;
    INAxxx_MID = $4954;

    NO_I2C_DID = $0000;
    INA219_DID = $0001;
    INA226_DID = $6022;
    INA3221_DID = $2032;

    INA219_MAX_R = 5;
    INA226_MAX_R = 7;
    INA3221_MAX_R = 18;

    SMBus_Speed_kHz = 1000; // default

    SMP_BUF_CNT = $3ffff;
    MAX_BLK_DEV1 = 30;
    MAX_BLK_DEV2 = 100;
    MAX_BLK_CNT = 116*2;
    COM_BUF_CNT = (MAX_BLK_CNT*2)+2;
    // DEV command id:
    CMD_DEV_VER = $00; // Get Ver
    // I2C/SBUS cfg
    CMD_DEV_CFG = $01; // Get/Set CFG/ini I2C & Start measure
    // Save cfg
    CMD_DEV_SCF = $02; // Store CFG/ini in Flash
    // Status
    CMD_DEV_STA = $03; // Status
    // BLE cfg
    CMD_DEV_CPU = $04; // Connect parameters Update (BLE)
    CMD_DEV_BLE = $05; // BLE parameters Update (BLE)
    // 0x06
    // I2C/SMBUS out regs
    CMD_DEV_I2C = $07; // blk out regs i2c data
    // ADC cfg
    CMD_DEV_CAD = $08; // Get/Set CFG/ini ADC & Start measure
    // DAC cfg
    CMD_DEV_DAC = $09; // DAC cfg
    // ADC out samples
    CMD_DEV_ADC = $0A; // blk out regs ADC data
    // TST device
    CMD_DEV_TST = $0B; // blk out X data, cfg TST device
    // I2C rd/wr
    CMD_DEV_UTR = $0C; // I2C read/write
    // Debug
    CMD_DEV_DBG = $0D; // Debug
    // Power, Sleep
    CMD_DEV_PWR = $0E; // Power On/Off, Sleep
    // Runtime Error
    CMD_DEV_ERR = $0F; // Runtime Error
    // I2C/SMBUS rd/wr regs
    CMD_DEV_GRG = $10; // Get reg I2C
    CMD_DEV_SRG = $11; // Set reg I2C
    // UART
    CMD_DEV_UAC = $12; // Set UART
    CMD_DEV_UAR = $13; // Send/Receive UART

    RES_OUT_REGS = $07; // Send blk regs

    CHART_I_MASK  = 2;
    CHART_U_MASK  = 1;
    CHART_UI_MASK  = 3;

    CHART_U_NUM  = 0;
    CHART_I_NUM  = 1;

    DEF_ADC_SPS = 10000;
    DEF_ADC_CHNL = 9;
type
  xName = (xByte, xWord, xData, xIna2Data, xIna3Data);

 Tcfg_ini = packed record
   none   : dword;
 end;


type
  ina2xx_regs_t = packed record
   case xName of
    xWord: ( w : array[0..255] of word);
    xData: (
      config      : word;		// Configuration Register
    );
    xIna2Data: (
      ina2_config      : word;		// Configuration Register
      ina2_shunt       : Smallint;		// Shunt Voltage Register
      ina2_bus         : Smallint;		// Bus Voltage Register
      ina2_power       : word; 	// Power Register
      ina2_current     : word; 	// Current Register
      ina2_calibration : word; 	// Calibration Register
      ina2_mask_enable : word; 	// INA226: Mask/Enable Register, INA219: =0000
      ina2_alert_data  : word; 	// INA226: Alert Limit Register, INA219: =7E0A
//      mid       : word;   // addr 0xfe: Manufacturer ID Register. INA226: =5449
//      did       : word;   // addr 0xff: Die ID Register. INA226: =2260

    );
    xIna3Data: (
      ina3_config      : word;		// Configuration Register
      ina3_shunt1      : Smallint;		// Shunt1 Voltage Register
      ina3_bus1        : Smallint;		// Bus1 Voltage Register
      ina3_shunt2      : Smallint;		// Shunt2 Voltage Register
      ina3_bus2        : Smallint;		// Bus2 Voltage Register
      ina3_shunt3      : Smallint;		// Shunt3 Voltage Register
      ina3_bus3        : Smallint;		// Bus3 Voltage Register
      ina3_cralrl1     : word; 	// Channel-1 Critical-Alert Limit
      ina3_wralrl1     : word; 	// Channel-1 Warning-Alert Limit
      ina3_cralrl2     : word; 	// Channel-2 Critical-Alert Limit
      ina3_wralrl2     : word; 	// Channel-2 Warning-Alert Limit
      ina3_cralrl3     : word; 	// Channel-3 Critical-Alert Limit
      ina3_wralrl3     : word; 	// Channel-3 Warning-Alert Limit
      ina3_shvsum      : word; 	// Shunt-Voltage Sum
      ina3_shvsuml     : word; 	// Shunt-Voltage Sum Limit
      ina3_mskena      : word;   // Mask/Enable
      ina3_pwrupl      : word;   // Power-Valid Upper Limit
      ina3_pwrlwl      : word;   // Power-Valid Lower Limit
//      ina3_mid       : word;   // addr 0xfe: Manufacturer ID Register. INA226: =5449
//      ina3_did       : word;   // addr 0xff: Die ID Register. INA226: =2260
    );
  end;

  ina32xx_regs_t = packed record
   case xName of
    xWord: ( w : array[0..9] of word);
    xData: (
      config      : word;		// Configuration Register
      shunt1      : Smallint;		// Shunt1 Voltage Register
      bus1        : Smallint;		// Bus1 Voltage Register
      shunt2      : Smallint;		// Shunt2 Voltage Register
      bus2        : Smallint;		// Bus2 Voltage Register
      shunt3      : Smallint;		// Shunt3 Voltage Register
      bus3        : Smallint;		// Bus3 Voltage Register
      cralrl1     : word; 	// Channel-1 Critical-Alert Limit
      wralrl1     : word; 	// Channel-1 Warning-Alert Limit
      cralrl2     : word; 	// Channel-2 Critical-Alert Limit
      wralrl2     : word; 	// Channel-2 Warning-Alert Limit
      cralrl3     : word; 	// Channel-3 Critical-Alert Limit
      wralrl3     : word; 	// Channel-3 Warning-Alert Limit
      shvsum      : word; 	// Shunt-Voltage Sum
      shvsuml     : word; 	// Shunt-Voltage Sum Limit
      mskena      : word;   // Mask/Enable
      pwrupl      : word;   // Power-Valid Upper Limit
      pwrlwl      : word;   // Power-Valid Lower Limit
      mid       : word;   // addr 0xfe: Manufacturer ID Register. INA226: =5449
      did       : word;   // addr 0xff: Die ID Register. INA226: =2260
    );
  end;

  ina2xx_rwr_t = packed record
      dev_addr    : byte;
      reg_addr    : byte;
      data        : word;
  end;
  ina2xx_rrd_t = packed record
      dev_addr    : byte;
      reg_addr    : byte;
  end;

  ina2xx_cfg_t = packed record
	    pktcnt      : byte; // кол-во передаваемых значений из регистров в одном пакете передачи
	    multiplier  : byte; // множитель периода опроса  time_us << multiplier
      time_us     : word; // период опроса регистров чтения в us
	    clk_khz     : word; 	// частота i2c шины кГц
      init        : array [0..3] of ina2xx_rwr_t;
      data        : array [0..3] of ina2xx_rrd_t;
      slp         : array [0..1] of ina2xx_rwr_t;
  end;

  TCommThread = class(TThread)
  private
    procedure QueryPort;
  protected
    procedure Execute; override;
  end;

  TfrmMain = class(TForm)
    StatusBar: TStatusBar;
    Panel1: TPanel;
    btnQueryDosDevice: TButton;
    ComboBox: TComboBox;
    ButtonOpen: TButton;
    ButtonConfigReg: TButton;
    ButtonStart: TButton;
    ButtonStore: TButton;
    Panel2: TPanel;
    Timer1: TTimer;
    ButtonRdAllRegs: TButton;
    ButtonClrGrf: TButton;
    ButtonStop: TButton;
    Chart: TChart;
    ButtonSaveGRF: TButton;
    ButtonPrtGrf: TButton;
    PrintDialog: TPrintDialog;
    SaveDialog: TSaveDialog;
    PrinterSetupDialog: TPrinterSetupDialog;
    SetColorI: TLabel;
    SetColorU: TLabel;
    ColorDialog: TColorDialog;
    ButtonAsize: TButton;
    ButtonScalP: TButton;
    ButtonScalM: TButton;
    EditRegs: TEdit;
    TeeOpenGL1: TTeeOpenGL;
    ChartPreviewer: TChartPreviewer;
    TeeCommander: TTeeCommander;
    ChartTool1: TSeriesStatsTool;
    CheckBoxOpenGL: TCheckBox;
    Series2: TFastLineSeries;
    Series1: TFastLineSeries;
    EditSizeGrf: TEdit;
    EditTriggerI: TEdit;
    CheckBoxTrigerRiseI: TCheckBox;
    LabelMXI: TLabel;
    LabelMXU: TLabel;
    CheckBoxTrigerRiseU: TCheckBox;
    EditTriggerU: TEdit;
    ButtonStartADC: TButton;
    ButtonADCcfg: TButton;
    ButtonSaveWav: TButton;
    Label1: TLabel;
    procedure btnReScanComDevices(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure ReScanComDevices;
    procedure ButtonOpenClick(Sender: TObject);
    function SetComName: boolean;
    procedure ChComName(Sender: TObject);
    procedure ButtonStartClick(Sender: TObject);
    procedure ButtonConfigRegClick(Sender: TObject);
    procedure ButtonStoreClick(Sender: TObject);
    function GetDevVersion: boolean;

    function  IndexOfComName(const S: String): Integer;
    procedure TimerTimer(Sender: TObject);
    procedure ButtonRdAllRegsClick(Sender: TObject);
    procedure ButtonClrGrfClick(Sender: TObject);
    procedure ButtonStopClick(Sender: TObject);
    procedure ButtonPrtGrfClick(Sender: TObject);
    procedure ButtonSaveGRFClick(Sender: TObject);

    procedure AddCrfSamples;
    function GrfSetColor(ChNum : integer): integer;
    procedure ButtonAsizeClick(Sender: TObject);
    procedure ButtonScalMClick(Sender: TObject);
    procedure ButtonScalPClick(Sender: TObject);

    procedure ReadIni;
    procedure WriteIni;
    procedure GetScrParms;
    procedure ShowScrParms;
    procedure ShowLabelsMX;
    procedure ShowSmps;
    procedure SetGrfMarging;
    procedure SetParStart;
    function SetAdcIniCfg(mode : integer) : boolean;
    function SetAdc2IniCfg(mode : integer) : boolean;
    function ReadBlk(id : byte) : boolean;
    function SendBlk(data_count : byte) : boolean;
    function ReadStatus : boolean;
    function ReadRegister(regnum : integer) : boolean;
    function WriteRegister(regnum : integer; val : word) : boolean;
    function ResetIna2xx : boolean;
    function RdAllRegs : boolean;
    function GetDevIniCfg : boolean;
    function SetDevIniCfg(mode : integer) : boolean;
    function StopReadDevice: boolean;
    procedure ShowAllRegs;
    procedure ClearGrf;
    procedure CheckBoxOpenGLClick(Sender: TObject);
    procedure CheckBoxTrigerClick(Sender: TObject);
    procedure FormClose(Sender: TObject; var Action: TCloseAction);
    procedure SetColorIMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure SetColorUMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure ButtonSaveWavClick(Sender: TObject);
    procedure ButtonStartADCClick(Sender: TObject);
    procedure ButtonADCcfgClick(Sender: TObject);
  private
    flgValueChg : boolean;
    Ini_Cfg : Tcfg_ini;
    dev_i2c_id : word;
    ina_max_regs : integer;
    DeviceTypeRecognized : boolean;

    TriggerI : Double;
    TriggerU : Double;
    TriggerUEnable : boolean;
    TriggerULow : boolean;
    TriggerIEnable : boolean;
    TriggerILow : boolean;

    SamplesCount : dword;
    MaxSamples: dword;
    SamplesAutoStop : boolean;
    ConnectStartTime : TDateTime;
    ConnectTime : TDateTime;
    ChartEnables : byte;
    OldCurI, OldCurU: double;
    CurI, CurU: double;
    SumI, SumU: double;
    FormConfigOk : integer;
//    procedure ReadNewAdcBlk(var Msg: TMessage); message WM_NEW_ADC_BLK;
//    procedure ReadCommand(var Msg: TMessage); message WM_RD_COMMAND;
  public
    { Public declarations }
  end;

var
  frmMain: TfrmMain;
  CommThread: TCommThread;
  ina_reg : ina2xx_regs_t;
  blk_cfg : ina2xx_cfg_t;

  dev_type : byte;
  dev_id : byte;
  dev_ver: word; // BCD 0x1234 - > 1.2.3.4

  BuildInfoString: string;

  Uk : double;
  Ik : double;
  I_zero : double;
  U_zero : double;
  OldsI, OldsU: double;

  current_smps : double;

   SamplesEna : boolean;
   bufsmp : array [0..SMP_BUF_CNT] of Smallint;
   ismprx, ismptx : dword;

   purge_com : byte;

   buftx : array [0..63] of byte;
    lentx : integer;
    errtx : byte;

   bufrx : array [0..63] of byte;
    lenrx : integer;
    wait_id : byte;
    wait_cnt : dword;

   bufcom : array [0..COM_BUF_CNT] of Byte;
    idx_bufcom : dword;

    dev_not_send_count : dword;
    dev_all_send_count : dword;
    dev_send_err : byte;

   work_adc : boolean;

  IniFile : TIniFile = nil;
  IniFileName : string = '.\PowerProfiler.ini';

implementation

uses
  StrUtils, ComPort, HexUtils, Ina219_r_config, Ina226_r_config, adc_jdy10_config, adc_bl702_config, WaveStorage,
  Ina3221_r_config;

{$R *.dfm}

procedure StartComThread;
begin
  if CommThread = nil then begin
    CommThread := TCommThread.Create(False);
    idx_bufcom := 0;
    wait_id := 0;
    wait_cnt := 0;
    lentx := 0;
    ismprx := 0;
    ismptx := 0;
    errtx := 0;
    bufcom[0] := 0;
  end;
  if CommThread = nil then begin
    SysErrorMessage(GetLastError);
//    Exit;
  end;
end;

procedure CloseComThread;
begin
  if CommThread <> nil then
    CommThread.Terminate;
  CommThread := nil;
end;

procedure TCommThread.Execute;
begin
  repeat
    QueryPort;
  until Terminated;
end;

{
function TCommThread.GetWaitCnt : dword;
begin
  result := wait_cnt;
end;

procedure TCommThread.SetWaitCnt(cnt : dword);
begin
  wait_cnt := cnt;
end;
}

procedure TCommThread.QueryPort;
var
  Buff: array[0..MAX_BLK_CNT+2] of Byte;
  ByteReaded, ByteWrited, itx: Dword;
  i, cnt : integer;
begin
  if purge_com <> 0 then begin
    purge_com := 0;
    PurgeComm(hCom,PURGE_TXCLEAR or PURGE_RXCLEAR);
    idx_bufcom := 0;
    ismptx := 0;
    ismprx := 0;
  end;
  if lentx <> 0 then begin
     if not WriteFile(hCom, buftx, lentx, ByteWrited, Nil) then begin
       errtx := 1;
//       ClearCommError(hCom,dErr,Nil);
//    SysErrorMessage(GetLastError);
     end
     else if dword(lentx) <> ByteWrited then begin
       errtx := 2;
     end
     else errtx := 0;
     lentx := 0;
  end;
  if not ReadFile(hCom, Buff, SizeOf(Buff), ByteReaded, nil) then begin
    idx_bufcom := 0;
//    ClearCommError(hCom,dErr,Nil);
//    SysErrorMessage(GetLastError);
    Exit;
  end;
  if ByteReaded > 0 then  begin
    if ByteReaded + idx_bufcom > sizeof(bufcom) then begin
      // error !
      idx_bufcom := 0;
    end;
    // буферизовать ввод...
    move(Buff, bufcom[idx_bufcom], ByteReaded);

    ByteReaded := ByteReaded + idx_bufcom;

    if bufcom[0] <= MAX_BLK_CNT then begin
      while (bufcom[0] + 2 <= ByteReaded) do begin
        // что-то есть
        if ((bufcom[1] and $7F) = RES_OUT_REGS)
          or ((work_adc) and ((bufcom[1] and $7F) = CMD_DEV_ADC)) then begin
          // блок данных для графиков
          if ((bufcom[1] and $80) = 0) then begin
            if SamplesEna then begin
              if (bufcom[0] > 1) then begin
                cnt := bufcom[0] div 2;
//                dev_send_smps := dev_send_smps + cnt;
                itx := ismptx and SMP_BUF_CNT;
                for i:=1 to cnt do begin
                  bufsmp[itx] := bufcom[i*2] or (bufcom[i*2 + 1] shl 8);
                  Inc(itx);
                  itx := itx and SMP_BUF_CNT;
                end;
                ismptx := itx;
              end;
            end
          end
          else begin
            // ошибки при чтении регистров по таймеру
            if (bufcom[0] = 9) then begin
                dev_all_send_count :=  bufcom[0+2] or (bufcom[1+2] shl 8) or (bufcom[2+2] shl 16) or (bufcom[3+2] shl 24);
                dev_not_send_count :=  bufcom[4+2] or (bufcom[5+2] shl 8) or (bufcom[6+2] shl 16) or (bufcom[7+2] shl 24);
                dev_send_err := bufcom[8+2]; // номер ошибки
            end
            else begin
                dev_all_send_count := 0;
                dev_not_send_count := 0;
                dev_send_err := 255; // общая ошибка
            end;
//            SendMessage(frmMain.Handle, WM_NEW_ADC_BLK, 1, 0);
          end;
        end else begin
        // не $07 (выборка по фильтру cmd: wait_id)
          if (wait_cnt > 0) then begin
            //  ожидание ответа команде
            if ((bufcom[1] and $7f) = wait_id) then begin
              // ответ пришел
              if ((bufcom[1] and $80) <> 0) then
                  wait_id := wait_id or $80; // timeout or error
              if bufcom[0] > 0 then
                  move(bufcom[2], bufrx, bufcom[0]);
              lenrx := bufcom[0];
              wait_cnt := 0;
//              SendMessage(frmMain.Handle, WM_RD_COMMAND, 1, 0);
            end else begin
              Inc(wait_cnt);
              if wait_cnt > 3 then begin
                // ответ не пришел
                lenrx := 0;
                wait_cnt := 0;
                wait_id := wait_id or $80; // timeout or error
//                SendMessage(frmMain.Handle, WM_RD_COMMAND, 1, 0);
//                StatusBar.Panels[2].Text:='Нет ответа от устройства в '+ sComNane+'!';
                exit;
              end;
            end;
          end;
        end;
        ByteReaded := ByteReaded - 2 - bufcom[0];
        if ByteReaded > 0 then begin
          move(bufcom[2 + bufcom[0]], bufcom, ByteReaded);
          if bufcom[0] > MAX_BLK_CNT then begin
             // сбой фреймов протокола
             ByteReaded := 0;
             bufcom[0] := 0;
             idx_bufcom := 0;
             break;
          end;
        end;
        idx_bufcom := ByteReaded;
      end;
    end else begin
        // сбой фреймов протокола
        idx_bufcom := 0;
        bufcom[0] := 0;
//        break;
    end;
  end;
end;

function TfrmMain.SendBlk( data_count : byte) : boolean;
var
x : integer;
begin
   if CommThread = nil then begin
     result := false;
     exit;
   end;
   x := 0;
   result := FALSE;
   while(lentx <> 0) do begin
    sleep(10);
    Inc(x);
    if x > 5 then begin
      lentx := 0;
      exit;
    end;
   end;
   buftx[0]:= data_count;
   wait_id := buftx[1];
   wait_cnt := 1;
   lentx := data_count + 2;
   while(lentx <> 0) do begin
    sleep(10);
    Inc(x);
    if x > 5 then begin
      lentx := 0;
      exit;
    end;
   end;
   if errtx <> 0 then exit;
   result := TRUE;
end;


function TfrmMain.ReadBlk(id : byte) : boolean;
var
x : integer;
begin
   if CommThread = nil then begin
     result := false;
     exit;
   end;
   result := true;
   x := 0;
   while(wait_cnt <> 0) do begin
      sleep(10);
      Inc(x);
      if x > 5 then begin
         wait_cnt := 0;
         result := false;
         exit;
      end;
   end;
   if (wait_id and $80) <> 0 then
     result := false;
end;

function TfrmMain.ReadStatus : boolean;
var
 ft : boolean;
begin
   ft := Timer1.Enabled;
   Timer1.Enabled := False;
   result := true;
   buftx[1] := CMD_DEV_STA; // Cmd: Get Status
   if SendBlk(0) then begin
     if ReadBlk(buftx[1]) and (lenrx = 8) then begin
        dev_all_send_count :=  bufrx[0] or (bufrx[1] shl 8) or (bufrx[2] shl 16) or (bufrx[3] shl 24);
        dev_not_send_count :=  bufrx[4] or (bufrx[5] shl 8) or (bufrx[6] shl 16) or (bufrx[7] shl 24);
     end
     else begin
      StatusBar.Panels[2].Text:='Ошибки при чении статуса устройства на '+ sComNane+'!';
      result := false;
     end;
   end
   else begin
      StatusBar.Panels[2].Text:='Ошибки при чении статуса устройства на '+ sComNane+'!';
      result := false;
   end;
   Timer1.Enabled := ft;
end;

function TfrmMain.RdAllRegs : boolean;
var
 i: integer;
 ft : boolean;
 fs : boolean;

begin
    ft := Timer1.Enabled;
    fs := SamplesEna;
    Timer1.Enabled := False;
    result := true;

    case dev_i2c_id of
      INA219_DID : begin
        ina_max_regs := INA219_MAX_R;
      end;
      INA226_DID : begin
        ina_max_regs := INA226_MAX_R;
      end;
      INA3221_DID : begin
        ina_max_regs := INA3221_MAX_R;
      end;
      else begin
        ina_max_regs := INA3221_MAX_R;
      end;
    end;
    buftx[1] := CMD_DEV_GRG; // Cmd: Get word
    buftx[2] := INA2XX_I2C_ADDR;
    for i := 0 to ina_max_regs do begin
      buftx[3] := i;
      if SendBlk(2) and ReadBlk(buftx[1]) and (lenrx = 4) then begin
        ina_reg.w[i] := bufrx[2] or (bufrx[3] shl 8);
      end
      else begin
       StatusBar.Panels[2].Text:='Ошибки при чении регистров INAXXX в устройстве на '+ sComNane+'!';
       result := false;
       break;
      end;
    end;
//   if result then begin
//     I_zero := I_219_zero_tab[(ina2xx_reg.config and ControlGainMsk) shr ControlGainShl];
//   end;
     Timer1.Enabled := ft;
     SamplesEna := fs;
end;

function TfrmMain.WriteRegister(regnum : integer; val : word) : boolean;
var
 ft : boolean;
 fs : boolean;
begin
    ft := Timer1.Enabled;
    fs := SamplesEna;
    Timer1.Enabled := False;
    result := true;
    buftx[1] := CMD_DEV_SRG; // Cmd: SMBUS Write reg
    buftx[2] := INA2XX_I2C_ADDR;
    buftx[3] := regnum;
    buftx[4] := val;
    buftx[5] := val shr 8;
    if SendBlk(4) then
       result := false;
    Timer1.Enabled := ft;
    SamplesEna := fs;
end;


procedure TfrmMain.ShowAllRegs;
var
i : integer;
s : string;
begin

   s := '';
   for i := 0 to ina_max_regs do begin
      if i < ina_max_regs then
        s := s + IntToHex(ina_reg.w[i],4) + ', '
     else
        s := s + IntToHex(ina_reg.w[i],4);
   end;
   EditRegs.Text := s;
//   StatusBar.Panels[2].Text:='REGS: ' + EditRegs.Text;
end;

// Чтение переменных из .ini
procedure TfrmMain.ReadIni;
begin
   if IniFile = nil then IniFile := TIniFile.Create(IniFileName);
   if IniFile.ReadString('System','Version','') = '' then begin
     IniFile.WriteString('System','Version', BuildInfoString);
     IniFile.WriteInteger('System','MaxSamples', MaxSamples);
     IniFile.WriteFloat('System','TriggerI', TriggerI);
     IniFile.WriteFloat('System','TriggerU', TriggerU);
     IniFile.WriteInteger('System', 'ChIColor', SetColorI.Color);
     IniFile.WriteInteger('System', 'ChUColor', SetColorU.Color);
     IniFile.WriteBool('System','TriggerUena', CheckBoxTrigerRiseU.Checked);
     IniFile.WriteBool('System','TriggerIena', CheckBoxTrigerRiseI.Checked);

     IniFile.WriteInteger('Setup','Top',Top);
     IniFile.WriteInteger('Setup','Left',Left);
     IniFile.WriteInteger('Setup','Width',Width);
     IniFile.WriteInteger('Setup','Height',Height);

     IniFile.WriteFloat('INA219','Iz40mV',I_219_zero_tab[0]);
     IniFile.WriteFloat('INA219','Iz80mV',I_219_zero_tab[1]);
     IniFile.WriteFloat('INA219','Iz160mV',I_219_zero_tab[2]);
     IniFile.WriteFloat('INA219','Iz320mV',I_219_zero_tab[3]);
     IniFile.WriteFloat('INA219','Ik40mV',Ik_219_tab[0]);
     IniFile.WriteFloat('INA219','Ik80mV',Ik_219_tab[1]);
     IniFile.WriteFloat('INA219','Ik160mV',Ik_219_tab[2]);
     IniFile.WriteFloat('INA219','Ik320mV',Ik_219_tab[3]);
     IniFile.WriteFloat('INA219','Uz',U_219_zero);
     IniFile.WriteFloat('INA219','Uk',Uk_219);
     IniFile.WriteFloat('INA219','I2C_CLK',clk_219);

     IniFile.WriteFloat('INA226','Iz',I_226_zero);
     IniFile.WriteFloat('INA226','Ik',Ik_226);
     IniFile.WriteFloat('INA226','Uz',U_226_zero);
     IniFile.WriteFloat('INA226','Uk',Uk_226);
     IniFile.WriteFloat('INA226','I2C_CLK',clk_226);
     IniFile.WriteInteger('INA226','reg_calibration',reg_226_calibration);
     IniFile.WriteInteger('INA226','reg_mask_enable',reg_226_mask_enable);
     IniFile.WriteInteger('INA226','reg_alert_limit',reg_226_alert_limit);

     IniFile.WriteFloat('INA3221','Iz1ch',I_3221_zero_ch[0]);
     IniFile.WriteFloat('INA3221','Iz2ch',I_3221_zero_ch[1]);
     IniFile.WriteFloat('INA3221','Iz3ch',I_3221_zero_ch[2]);
     IniFile.WriteFloat('INA3221','Ik1ch',Ik_3221_ch[0]);
     IniFile.WriteFloat('INA3221','Ik2ch',Ik_3221_ch[1]);
     IniFile.WriteFloat('INA3221','Ik3ch',Ik_3221_ch[2]);
     IniFile.WriteFloat('INA3221','Uz1ch',U_3221_zero_ch[0]);
     IniFile.WriteFloat('INA3221','Uz2ch',U_3221_zero_ch[1]);
     IniFile.WriteFloat('INA3221','Uz3ch',U_3221_zero_ch[2]);
     IniFile.WriteFloat('INA3221','Uk1ch',Uk_3221_ch[0]);
     IniFile.WriteFloat('INA3221','Uk2ch',Uk_3221_ch[1]);
     IniFile.WriteFloat('INA3221','Uk3ch',Uk_3221_ch[2]);
     IniFile.WriteFloat('INA3221','I2C_CLK',clk_3221);


     IniFile.WriteInteger('ADC','Smps', ADC_smps);
     IniFile.WriteInteger('ADC','Chnl', ADC_channel);
     IniFile.WriteInteger('ADC','UI', UI_adc);
     IniFile.WriteInteger('ADC','PGA20db',PGA20db);
     IniFile.WriteInteger('ADC','PGA2db5',PGA2db5);
     IniFile.WriteFloat('ADC','Uk', Uk_adc);
     IniFile.WriteFloat('ADC','Uz', Uz_adc);
     IniFile.WriteFloat('ADC','Ik', Ik_adc);
     IniFile.WriteFloat('ADC','Iz', Iz_adc);

     IniFile.WriteInteger('ADC2','Smps', ADC2_smps);
     IniFile.WriteInteger('ADC2','Chnl', ADC2_channel);
     IniFile.WriteInteger('ADC2','UI', UI_adc2);
     IniFile.WriteInteger('ADC2','PGA20db',PGA20dbA2);
     IniFile.WriteInteger('ADC2','PGA2db5',PGA2db5A2);
     IniFile.WriteFloat('ADC2','Uk', Uk_adc2);
     IniFile.WriteFloat('ADC2','Uz', Uz_adc2);
     IniFile.WriteFloat('ADC2','Ik', Ik_adc2);
     IniFile.WriteFloat('ADC2','Iz', Iz_adc2);
   end
   else begin
     MaxSamples := IniFile.ReadInteger('System','MaxSamples', MaxSamples);
     TriggerI := IniFile.ReadFloat('System','TriggerI', TriggerI);
     TriggerU := IniFile.ReadFloat('System','TriggerU', TriggerU);
     CheckBoxTrigerRiseU.Checked := IniFile.ReadBool('System','TriggerUena', CheckBoxTrigerRiseU.Checked);
     CheckBoxTrigerRiseI.Checked := IniFile.ReadBool('System','TriggerIena', CheckBoxTrigerRiseI.Checked);

     Top := IniFile.ReadInteger('Setup','Top',10);
     Left := IniFile.ReadInteger('Setup','Left',10);
     if Screen.DesktopHeight < Top then Top := 10;
     if Screen.DesktopWidth < Left then Left := 10;
     Height := IniFile.ReadInteger('Setup','Height',Height);
     Width := IniFile.ReadInteger('Setup','Width',Width);
     sComNane := IniFile.ReadString('Setup','ComPort', sComNane);

     SetColorI.Color := IniFile.ReadInteger('System', 'ChIColor', SetColorI.Color);
     SetColorU.Color := IniFile.ReadInteger('System', 'ChUColor', SetColorU.Color);

     I_219_zero_tab[0]:= IniFile.ReadFloat('INA219','Iz40mV',I_219_zero_tab[0]);
     I_219_zero_tab[1]:= IniFile.ReadFloat('INA219','Iz80mV',I_219_zero_tab[1]);
     I_219_zero_tab[2]:= IniFile.ReadFloat('INA219','Iz160mV',I_219_zero_tab[2]);
     I_219_zero_tab[3]:= IniFile.ReadFloat('INA219','Iz320mV',I_219_zero_tab[3]);
     Ik_219_tab[0]:= IniFile.ReadFloat('INA219','Ik40mV',I_219_zero_tab[0]);
     Ik_219_tab[1]:= IniFile.ReadFloat('INA219','Ik80mV',I_219_zero_tab[1]);
     Ik_219_tab[2]:= IniFile.ReadFloat('INA219','Ik160mV',I_219_zero_tab[2]);
     Ik_219_tab[3]:= IniFile.ReadFloat('INA219','Ik320mV',I_219_zero_tab[3]);
     U_219_zero := IniFile.ReadFloat('INA219','Uz',U_219_zero);
     Uk_219 := IniFile.ReadFloat('INA219','Uk',Uk_219);
     clk_219 := IniFile.ReadInteger('INA219','I2C_CLK',clk_219);

     I_226_zero := IniFile.ReadFloat('INA226','Iz',I_226_zero);
     U_226_zero := IniFile.ReadFloat('INA226','Uz',I_226_zero);
     Ik_226 := IniFile.ReadFloat('INA226','Ik',Ik_226);
     Uk_226 := IniFile.ReadFloat('INA226','Uk',Uk_226);
     clk_226 := IniFile.ReadInteger('INA226','I2C_CLK',clk_226);

     reg_226_calibration := IniFile.ReadInteger('INA226','reg_calibration',reg_226_calibration);
     reg_226_mask_enable := IniFile.ReadInteger('INA226','reg_mask_enable',reg_226_mask_enable);
     reg_226_alert_limit := IniFile.ReadInteger('INA226','reg_alert_limit',reg_226_alert_limit);


     I_3221_zero_ch[0] := IniFile.ReadFloat('INA3221','Iz1ch',I_3221_zero_ch[0]);
     I_3221_zero_ch[1] := IniFile.ReadFloat('INA3221','Iz2ch',I_3221_zero_ch[1]);
     I_3221_zero_ch[2] := IniFile.ReadFloat('INA3221','Iz3ch',I_3221_zero_ch[2]);
     Ik_3221_ch[0] := IniFile.ReadFloat('INA3221','Ik1ch',Ik_3221_ch[0]);
     Ik_3221_ch[1] := IniFile.ReadFloat('INA3221','Ik2ch',Ik_3221_ch[1]);
     Ik_3221_ch[2] := IniFile.ReadFloat('INA3221','Ik3ch',Ik_3221_ch[2]);
     U_3221_zero_ch[0] := IniFile.ReadFloat('INA3221','Uz1ch',I_3221_zero_ch[0]);
     U_3221_zero_ch[1] := IniFile.ReadFloat('INA3221','Uz2ch',I_3221_zero_ch[1]);
     U_3221_zero_ch[2] := IniFile.ReadFloat('INA3221','Uz3ch',I_3221_zero_ch[2]);
     Uk_3221_ch[0] := IniFile.ReadFloat('INA3221','Uk1ch',Uk_3221_ch[0]);
     Uk_3221_ch[1] := IniFile.ReadFloat('INA3221','Uk2ch',Uk_3221_ch[1]);
     Uk_3221_ch[2] := IniFile.ReadFloat('INA3221','Uk3ch',Uk_3221_ch[2]);
     clk_3221 := IniFile.ReadInteger('INA3221','I2C_CLK',clk_3221);


     ADC_smps := IniFile.ReadInteger('ADC','Smps', ADC_smps);
     ADC_channel := IniFile.ReadInteger('ADC','Chnl', ADC_channel);
     UI_adc := IniFile.ReadInteger('ADC','UI', UI_adc);
     PGA20db := IniFile.ReadInteger('ADC','PGA20db',PGA20db);
     PGA2db5 := IniFile.ReadInteger('ADC','PGA2db5',PGA2db5);

     Uk_adc := IniFile.ReadFloat('ADC','Uk', Uk_adc);
     Uz_adc := IniFile.ReadFloat('ADC','Uz', Uz_adc);

     Ik_adc := IniFile.ReadFloat('ADC','Ik', Ik_adc);
     Iz_adc := IniFile.ReadFloat('ADC','Iz', Iz_adc);

     ADC2_smps := FormAdc2Config.Checksmps(IniFile.ReadInteger('ADC2','Smps', ADC2_smps));
     ADC2_channel := IniFile.ReadInteger('ADC2','Chnl', ADC2_channel);
     UI_adc2 := IniFile.ReadInteger('ADC2','UI', UI_adc2);
     PGA20dbA2 := IniFile.ReadInteger('ADC2','PGA20db',PGA20dbA2);
     PGA2db5A2 := IniFile.ReadInteger('ADC2','PGA2db5',PGA2db5A2);

     Uk_adc2 := IniFile.ReadFloat('ADC2','Uk', Uk_adc2);
     Uz_adc2 := IniFile.ReadFloat('ADC2','Uz', Uz_adc2);

     Ik_adc2 := IniFile.ReadFloat('ADC2','Ik', Ik_adc2);
     Iz_adc2 := IniFile.ReadFloat('ADC2','Iz', Iz_adc2);


   end;
   IniFile.UpdateFile;
end;

// Запись переменных в *.ini
procedure TfrmMain.WriteIni;
begin
     IniFile.WriteString('System','Version', BuildInfoString);
     IniFile.WriteInteger('System','MaxSamples', MaxSamples);
     IniFile.WriteFloat('System','TriggerI', TriggerI);
     IniFile.WriteFloat('System','TriggerU', TriggerU);
     IniFile.WriteBool('System','TriggerUena', CheckBoxTrigerRiseU.Checked);
     IniFile.WriteBool('System','TriggerIena', CheckBoxTrigerRiseI.Checked);

     IniFile.WriteInteger('Setup','Top',Top);
     IniFile.WriteInteger('Setup','Left',Left);
     IniFile.WriteInteger('Setup','Width',Width);
     IniFile.WriteInteger('Setup','Height',Height);
     IniFile.WriteString('Setup','ComPort',sComNane);

     IniFile.WriteInteger('System', 'ChIColor', SetColorI.Color);
     IniFile.WriteInteger('System', 'ChUColor', SetColorU.Color);

     IniFile.WriteFloat('INA219','Iz40mV',I_219_zero_tab[0]);
     IniFile.WriteFloat('INA219','Iz80mV',I_219_zero_tab[1]);
     IniFile.WriteFloat('INA219','Iz160mV',I_219_zero_tab[2]);
     IniFile.WriteFloat('INA219','Iz320mV',I_219_zero_tab[3]);
     IniFile.WriteFloat('INA219','Ik40mV',Ik_219_tab[0]);
     IniFile.WriteFloat('INA219','Ik80mV',Ik_219_tab[1]);
     IniFile.WriteFloat('INA219','Ik160mV',Ik_219_tab[2]);
     IniFile.WriteFloat('INA219','Ik320mV',Ik_219_tab[3]);
     IniFile.WriteFloat('INA219','Uz',U_219_zero);
     IniFile.WriteFloat('INA219','Uk',Uk_219);
     IniFile.WriteFloat('INA219','I2C_CLK',clk_219);

     IniFile.WriteFloat('INA226','Iz',I_226_zero);
     IniFile.WriteFloat('INA226','Uz',U_226_zero);
     IniFile.WriteFloat('INA226','Ik',Ik_226);
     IniFile.WriteFloat('INA226','Uk',Uk_226);
     IniFile.WriteFloat('INA226','I2C_CLK',clk_226);
     IniFile.WriteInteger('INA226','reg_calibration',reg_226_calibration);
     IniFile.WriteInteger('INA226','reg_mask_enable',reg_226_mask_enable);
     IniFile.WriteInteger('INA226','reg_alert_limit',reg_226_alert_limit);

     IniFile.WriteFloat('INA3221','Iz1ch',I_3221_zero_ch[0]);
     IniFile.WriteFloat('INA3221','Iz2ch',I_3221_zero_ch[1]);
     IniFile.WriteFloat('INA3221','Iz3ch',I_3221_zero_ch[2]);
     IniFile.WriteFloat('INA3221','Uz1ch',U_3221_zero_ch[0]);
     IniFile.WriteFloat('INA3221','Uz2ch',U_3221_zero_ch[1]);
     IniFile.WriteFloat('INA3221','Uz3ch',U_3221_zero_ch[2]);
     IniFile.WriteFloat('INA3221','Ik1ch',Ik_3221_ch[0]);
     IniFile.WriteFloat('INA3221','Ik2ch',Ik_3221_ch[1]);
     IniFile.WriteFloat('INA3221','Ik3ch',Ik_3221_ch[2]);
     IniFile.WriteFloat('INA3221','Uk1ch',Uk_3221_ch[0]);
     IniFile.WriteFloat('INA3221','Uk2ch',Uk_3221_ch[1]);
     IniFile.WriteFloat('INA3221','Uk3ch',Uk_3221_ch[2]);
     IniFile.WriteFloat('INA3221','I2C_CLK',clk_3221);

     IniFile.WriteInteger('ADC','Smps', ADC_smps);
     IniFile.WriteInteger('ADC','Chnl', ADC_channel);
     IniFile.WriteInteger('ADC','UI', UI_adc);
     IniFile.WriteInteger('ADC','PGA20db',PGA20db);
     IniFile.WriteInteger('ADC','PGA2db5',PGA2db5);
     IniFile.WriteFloat('ADC','Uk', Uk_adc);
     IniFile.WriteFloat('ADC','Uz', Uz_adc);
     IniFile.WriteFloat('ADC','Ik', Ik_adc);
     IniFile.WriteFloat('ADC','Iz', Iz_adc);

     IniFile.WriteInteger('ADC2','Smps', ADC2_smps);
     IniFile.WriteInteger('ADC2','Chnl', ADC2_channel);
     IniFile.WriteInteger('ADC2','UI', UI_adc2);
     IniFile.WriteInteger('ADC2','PGA20db',PGA20dbA2);
     IniFile.WriteInteger('ADC2','PGA2db5',PGA2db5A2);
     IniFile.WriteFloat('ADC2','Uk', Uk_adc2);
     IniFile.WriteFloat('ADC2','Uz', Uz_adc2);
     IniFile.WriteFloat('ADC2','Ik', Ik_adc2);
     IniFile.WriteFloat('ADC2','Iz', Iz_adc2);

     IniFile.UpdateFile;
end;


function Str2dword(const s: string): dword;
var
 i: integer;
 o : string;
 hex : boolean;
begin
    i := 1;
    hex := False;
//    result := 0;
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

procedure GetBuildInfo(var V1, V2, V3, V4: word);
var
  VerInfoSize, VerValueSize, Dummy: DWORD;
  VerInfo: Pointer;
  VerValue: PVSFixedFileInfo;
begin
  VerInfoSize := GetFileVersionInfoSize(PChar(ParamStr(0)), Dummy);
  if VerInfoSize > 0 then
  begin
      GetMem(VerInfo, VerInfoSize);
      try
        if GetFileVersionInfo(PChar(ParamStr(0)), 0, VerInfoSize, VerInfo) then
        begin

          VerQueryValue(VerInfo, '\', Pointer(VerValue), VerValueSize);
          with VerValue^ do
          begin
            V1 := dwFileVersionMS shr 16;
            V2 := dwFileVersionMS and $FFFF;
            V3 := dwFileVersionLS shr 16;
            V4 := dwFileVersionLS and $FFFF;
          end;
        end;
      finally
        FreeMem(VerInfo, VerInfoSize);
      end;
  end;
end;

function GetBuildInfoAsString: string;
var
  V1, V2, V3, V4: word;
begin
  GetBuildInfo(V1, V2, V3, V4);
  Result := IntToStr(V1) + '.' + IntToStr(V2) + '.' +
    IntToStr(V3) + '.' + IntToStr(V4);
end;

//Достает из строки с нуль-терминированными подстроками следующую нуль-терминированную
//подстроку начиная с позиции aStartPos, потом устанавливает aStartPos на символ
//следующий за терминирующим #0.
function GetNextSubstring(aBuf: string; var aStartPos: integer): string;
var
  vLastPos: integer;
begin
  if (aStartPos < 1) then
    begin
      raise ERangeError.Create('aStartPos должен быть больше 0');
    end;

  if (aStartPos > Length(aBuf) ) then
    begin
      Result := '';
      Exit;
    end;

  vLastPos := PosEx(#0, aBuf, aStartPos);
  Result := Copy(aBuf, aStartPos, vLastPos - aStartPos);
  aStartPos := aStartPos + (vLastPos - aStartPos) + 1;
end;

// Заполняет список aList наденными в системе COM портами
procedure GetComPorts(aList: TStrings; aNameStart: string);
var
  vBuf: string;
  vRes: integer;
  vErr: Integer;
  vBufSize: Integer;
  vNameStartPos: Integer;
  vName: string;
begin
  vBufSize := 1024 * 5;
  vRes := 0;

  while vRes = 0 do
    begin
      setlength(vBuf, vBufSize) ;
      SetLastError(ERROR_SUCCESS);
      vRes := QueryDosDevice(nil, @vBuf[1], vBufSize) ;
      vErr := GetLastError();

      // Вариант для двухтонки
      if (vRes <> 0) and (vErr = ERROR_INSUFFICIENT_BUFFER) then
        begin
          vBufSize := vRes;
          vRes := 0;
        end;

      if (vRes = 0) and (vErr = ERROR_INSUFFICIENT_BUFFER) then
        begin
          vBufSize := vBufSize + 1024;
        end;

      if (vErr <> ERROR_SUCCESS) and (vErr <> ERROR_INSUFFICIENT_BUFFER) then
        begin
          raise Exception.Create(SysErrorMessage(vErr) );
        end
    end;
  setlength(vBuf, vRes) ;

  vNameStartPos := 1;
  vName := GetNextSubstring(vBuf, vNameStartPos);

  aList.BeginUpdate();
  try
    aList.Clear();
    while vName <> '' do
      begin
        if AnsiStartsStr(aNameStart, vName) then
          aList.Add(vName);
        vName := GetNextSubstring(vBuf, vNameStartPos);
      end;
  finally
    aList.EndUpdate();
  end;
end;


procedure TfrmMain.btnReScanComDevices(Sender: TObject);
begin
  ReScanComDevices;
end;

function TfrmMain.IndexOfComName(const S: String): Integer;
var
  I : Integer;
begin
  Result := -1;
  for I := 0 to ComboBox.Items.Count-1 do
  begin
    if ComboBox.Items.Strings[I] = S then
    begin
      Result := I;
      Exit;
    end;
  end;
end;

function TfrmMain.SetComName: boolean;
begin
  result:=False;
  sComNane:=ComboBox.Items.Strings[ComboBox.ItemIndex];
  if(sComNane='') then begin
    StatusBar.Panels[0].Text:='COM?';
    StatusBar.Panels[2].Text:='Не выбран COM порт!';
  end
  else begin
    StatusBar.Panels[0].Text:=sComNane;
    StatusBar.Panels[2].Text:='Выбран '+sComNane+'.';
    result:=True;
  end;
end;

procedure TfrmMain.ReScanComDevices;
begin
  ComboBox.Items.Clear;
  GetComPorts(ComboBox.Items, 'COM');
  if(sComNane <> '') then begin
    ComboBox.ItemIndex := IndexOfComName(sComNane);
  end
  else
    ComboBox.ItemIndex:=0;
  SetComName;
end;

procedure TfrmMain.GetScrParms;
begin
    DecimalSeparator := '.';
    MaxSamples := Str2dword(EditSizeGrf.Text);
    TriggerI := StrToFloat(EditTriggerI.Text);
    TriggerU := StrToFloat(EditTriggerU.Text);
end;

procedure TfrmMain.ShowScrParms;
begin
    DecimalSeparator := '.';
    EditSizeGrf.Text := IntToStr(MaxSamples);
    EditTriggerI.Text:=FormatFloat('0.0000', TriggerI);
    EditTriggerU.Text:=FormatFloat('0.0000', TriggerU);
end;

procedure TfrmMain.ShowLabelsMX;
begin
        if ((ChartEnables and 2) <> 0)  and (SamplesCount <> 0) then begin
            OldsI := SumI/SamplesCount;
            LabelMXI.Caption := 'I:' + FormatFloat('# ##0.000 000', OldsI);
        end
        else
            LabelMXI.Caption := 'I#' + FormatFloat('# ##0.000 000', OldCurI);
        if ((ChartEnables and 1) <> 0) and (SamplesCount <> 0) then begin
            OldsU := SumU/SamplesCount;
            LabelMXU.Caption := 'U:' + FormatFloat('# ##0.000 000', SumU/SamplesCount);
        end
        else
            LabelMXU.Caption := 'U#' + FormatFloat('# ##0.000 000', OldCurU);
end;

procedure TfrmMain.ShowSmps;
var
//k : double;
t : dword;
begin
  t := blk_cfg.time_us shl blk_cfg.multiplier;
  if t <> 0 then begin
//    if ChartEnables = CHART_UI_MASK then begin
//      k := 2000000.0/t;
//    end
//    else begin
      current_smps := 1000000.0/t;
//    end;
    StatusBar.Panels[1].Text := FormatFloat('# ##0.0', current_smps) + ' sps';
  end;
end;

procedure TfrmMain.FormCreate(Sender: TObject);
begin
  if Screen.DesktopHeight <= Top then Top := 10;
  if Screen.DesktopWidth <= Left then Left := 10;

  DecimalSeparator := '.';
  flgValueChg := False;
  work_adc := False;
  dev_i2c_id := NO_I2C_DID;

  DeviceTypeRecognized := False;

  BuildInfoString := GetBuildInfoAsString;
  Caption := Application.Title + ' ver ' + BuildInfoString;
  SetGrfMarging;

  SamplesAutoStop := False;

  Ini_Cfg.none := $00;
  dev_type := 0;
  dev_id := 0;
  dev_send_err := 0;
  ChartEnables := 0;
  blk_cfg.clk_khz := SMBus_Speed_kHz;
  blk_cfg.multiplier := 0;

  GetScrParms;

  SetColorU.Color := Chart.Series[CHART_U_NUM].Color;
  SetColorI.Color := Chart.Series[CHART_I_NUM].Color;

  ReadIni;

  Chart.Series[CHART_U_NUM].Color := SetColorU.Color;
  Chart.Series[CHART_I_NUM].Color := SetColorI.Color;

  ReScanComDevices;

  ShowScrParms;
  flgValueChg := True;
end;

procedure TfrmMain.ChComName(Sender: TObject);
begin
  SetComName;
end;

procedure TfrmMain.FormClose(Sender: TObject; var Action: TCloseAction);
begin
  GetScrParms;
  WriteIni;
end;

procedure TfrmMain.ButtonOpenClick(Sender: TObject);
begin
   Timer1.Enabled := False;
   SamplesEna := False;
   if not flgComOpen then begin
    if SetComName then begin
      if OpenCom(sComNane) then begin
        purge_com := 1;
        ClearGrf;
        ChartEnables := 0;
        StartComThread;
        dev_i2c_id := NO_I2C_DID;
        if not GetDevVersion and (dev_i2c_id <> NO_I2C_DID) then begin
          CloseComThread;
          CloseCom;
          if (dev_id <> I2C_DEVICE_ID) and
             (dev_id <> ADC1_DEVICE_ID) and
             (dev_id <> ADC2_DEVICE_ID) then begin
            ShowMessage('Error device ID!'+#13#10+'Com closed.');
            StatusBar.Panels[2].Text:='Не тот ID у устройства на '+ sComNane+'!';
          end else begin
            ShowMessage('Error read device config!'+#13#10+'Com closed.');
            StatusBar.Panels[2].Text:='Ошибки в чении конфигурации устройства на '+ sComNane+'!';
          end
        end else begin
          ComboBox.Enabled := False;
          btnQueryDosDevice.Enabled := False;
          ButtonOpen.Caption := 'Close';
          ButtonStop.Enabled := True;
          purge_com := 1;
          if dev_i2c_id <> NO_I2C_DID then begin
            if RdAllRegs then begin
              ShowAllRegs;
              ButtonRdAllRegs.Enabled := True;
              ButtonConfigReg.Enabled := True;
              ButtonStart.Enabled := True;
              ButtonStore.Enabled := True;
            end else begin
              StatusBar.Panels[2].Text:='Ошибки при чении регистров INA2XX в устройстве на '+ sComNane+'!';
              ShowMessage(StatusBar.Panels[2].Text);
              exit;
            end;
          end;
          if (dev_id = ADC1_DEVICE_ID) or (dev_id = ADC2_DEVICE_ID) then begin
              ButtonStartADC.Enabled := True;
              ButtonStartADC.Visible := True;
              ButtonADCcfg.Enabled := True;
              ButtonADCcfg.Visible := True;
          end;
//            StatusBar.Panels[2].Text:=sComNane+' открыт.';
          purge_com := 1;
          ClearGrf;
          ConnectStartTime := GetTime;
          if (dev_type = HI_DEVICE_TYPE) then begin
            Form219Config.SpinEditCLkKHz.MaxValue := 3000;
            Form226Config.SpinEditCLkKHz.MaxValue := 3000;
            Form3221Config.SpinEditCLkKHz.MaxValue := 3000;
          end
          else begin
            Form219Config.SpinEditCLkKHz.MaxValue := 1000;
            Form226Config.SpinEditCLkKHz.MaxValue := 1000;
            Form3221Config.SpinEditCLkKHz.MaxValue := 1000;
          end;
          case dev_i2c_id of
            INA219_DID : begin
              if (dev_id = ADC1_DEVICE_ID) or (dev_id = ADC2_DEVICE_ID) then
                Caption := Application.Title + ' ver ' + BuildInfoString + ' (INA219 + ADC)'
              else
                Caption := Application.Title + ' ver ' + BuildInfoString + ' (INA219)';
              Form219Config.ShowAll;
              Form219Config.GetParams;
            end;
            INA226_DID : begin
              if (dev_id = ADC1_DEVICE_ID) or (dev_id = ADC2_DEVICE_ID) then
                Caption := Application.Title + ' ver ' + BuildInfoString + ' (INA226 + ADC)'
              else
                Caption := Application.Title + ' ver ' + BuildInfoString + ' (INA226)';
              Form226Config.ShowAll;
              Form226Config.GetParams;
              Form226Config.SetRegs;
              if RdAllRegs then
                ShowAllRegs;
            end;
            INA3221_DID : begin
              if (dev_id = ADC1_DEVICE_ID) or (dev_id = ADC2_DEVICE_ID) then
                Caption := Application.Title + ' ver ' + BuildInfoString + ' (INA3221 + ADC)'
              else
                Caption := Application.Title + ' ver ' + BuildInfoString + ' (INA3221)';
              Form3221Config.ShowAll;
              Form3221Config.GetParams;
            end;
            else begin
                if (dev_id = ADC1_DEVICE_ID) then begin
                  Caption := Application.Title + ' ver ' + BuildInfoString + ' (ADC)';
                  FormAdcConfig.GetParams;
                end else
                if (dev_id = ADC2_DEVICE_ID) then begin
                  Caption := Application.Title + ' ver ' + BuildInfoString + ' (ADC BL702)';
                  FormAdc2Config.GetParams;
                end else
                 Caption := Application.Title + ' ver ' + BuildInfoString + ' (?)';
            end;
          end;
          DeviceTypeRecognized := True;
//            SamplesEna := True;
//            Timer1.Enabled := True;
        end;
      end
      else begin
        StatusBar.Panels[2].Text:='Ошибка открытия '+ sComNane+'!';
        ShowMessage(StatusBar.Panels[2].Text);
      end;
    end
    else begin
      StatusBar.Panels[2].Text:='Выберите COM порт!';
    end;
  end
  else begin
     CloseComThread;
     CloseCom;
     ButtonOpen.Caption := 'Open';
     ComboBox.Enabled := True;
     btnQueryDosDevice.Enabled := True;

     ButtonStart.Enabled := False;
     ButtonStop.Enabled := False;
     ButtonStartADC.Enabled := False;
     ButtonConfigReg.Enabled := False;
     ButtonStore.Enabled := False;
     ButtonRdAllRegs.Enabled := False;
     StatusBar.Panels[2].Text:=sComNane+' закрыт.';
  end;
end;

function TfrmMain.SetDevIniCfg(mode : integer) : boolean;
begin
   result := False;
   Timer1.Enabled := False;
   case dev_i2c_id of
     INA219_DID : begin
       ChartEnables := Form219Config.DevIniCfg(mode);
     end;
     INA226_DID : begin
       ChartEnables := Form226Config.DevIniCfg(mode);
     end;
     INA3221_DID : begin
       ChartEnables := Form3221Config.DevIniCfg(mode);
     end;
     else
       exit;
   end;
   buftx[1] := CMD_DEV_CFG;
   move(blk_cfg, buftx[2], SizeOf(blk_cfg));
   if SendBlk(SizeOf(blk_cfg)) then begin
     if ReadBlk(buftx[1]) and (lenrx >= SizeOf(blk_cfg)) then begin
       move(bufrx, blk_cfg, SizeOf(blk_cfg));
       StatusBar.Panels[2].Text:='Конфигурация передана в устройство на '+ sComNane+'.';
       result := True;
     end
     else begin
       StatusBar.Panels[2].Text:='Ошибка записи конфигурации в устройство на '+ sComNane+'!';
     end;
   end;
   ShowAllRegs;
   if SamplesEna then Timer1.Enabled := True;
end;

function TfrmMain.GetDevIniCfg : boolean;
begin
     result := False;
     Timer1.Enabled := False;
     buftx[1]:=CMD_DEV_SCF; // Set/Get CFG/ini & Start measure
     if SendBlk(0) then begin
       if ReadBlk(buftx[1]) and (lenrx = SizeOf(blk_cfg)) then begin
         move(bufrx, blk_cfg, SizeOf(blk_cfg));
         result := True;
       end;
     end;
     if SamplesEna then Timer1.Enabled := True;
end;


procedure TfrmMain.ButtonStartClick(Sender: TObject);
var
t, maxs : dword;
k : double;
begin
    SamplesEna := False;
    Timer1.Enabled := False;
    //if (dev_id = ADC1_DEVICE_ID) then SetAdcIniCfg(0);
    //if (dev_id = ADC2_DEVICE_ID) then SetAdc2IniCfg(0);
    if SetDevIniCfg(1) then begin
        ConnectStartTime := GetTime;
        StatusBar.Panels[2].Text:='Запущено непрерывное чтение INA2XX в устройстве на '+ sComNane+'.';
        work_adc := False;
        SetParStart;
        t := blk_cfg.time_us shl blk_cfg.multiplier; // t us
        if t <> 0 then begin
          maxs := 3000*t; // ms
          if MaxSamples > maxs then begin
            MaxSamples := maxs; // ms
            EditSizeGrf.Text := IntToStr(MaxSamples); // ms
          end;
        end;
        ShowSmps;
        if SamplesAutoStop then begin
           SamplesAutoStop := False;
           ClearGrf;
        end;
//        dev_send_smps := 0;
        ismptx := 0;
        ismprx := 0;
        SamplesEna := True;
        Timer1.Enabled := True;
   end;
end;

procedure TfrmMain.ButtonStopClick(Sender: TObject);
begin
   SamplesEna := False;
   Timer1.Enabled := False;
   if ReadStatus then begin
      if dev_i2c_id <> NO_I2C_DID then
        if not StopReadDevice then
          exit;
      if (dev_id = ADC1_DEVICE_ID) then begin
        if not SetAdcIniCfg(0)then
          exit;
      end;
      if (dev_id = ADC2_DEVICE_ID) then begin
        if not SetAdc2IniCfg(0)then
          exit;
      end;
      StatusBar.Panels[2].Text:='Непрерывное чтение устройства на '+ sComNane+' остановлено. Пропуск ' + IntToStr(dev_not_send_count) + ' блоков, счетчик: ' + IntToStr(dev_all_send_count) + '.'; // +    IntToStr(dev_send_smps);
      sleep(10);
      purge_com := 1;
      ismptx := 0;
      ismprx := 0;
      SetParStart;
   end;
end;

procedure TfrmMain.ButtonConfigRegClick(Sender: TObject);
begin
  if RdAllRegs then begin
    case dev_i2c_id of
      INA219_DID : begin
        Form219Config.Left := Left + (Width div 2) - Form219Config.Width div 2;
        Form219Config.Top := Top + (Height div 2) - Form219Config.Height div 2;
        FormConfigOk := Form219Config.ShowModal;
      end;
      INA226_DID : begin
        Form226Config.Left := Left + (Width div 2) - Form226Config.Width div 2;
        Form226Config.Top := Top + (Height div 2) - Form226Config.Height div 2;
        FormConfigOk := Form226Config.ShowModal;
      end;
      INA3221_DID : begin
        Form3221Config.Left := Left + (Width div 2) - Form3221Config.Width div 2;
        Form3221Config.Top := Top + (Height div 2) - Form3221Config.Height div 2;
        FormConfigOk := Form3221Config.ShowModal;
      end
      else
        exit;
    end;
    if FormConfigOk = mrOk then begin
      Timer1.Enabled := False;

      buftx[1]:=CMD_DEV_SRG; // Set Reg

      buftx[2]:=INA2XX_I2C_ADDR;
      buftx[3]:=0;
      buftx[4]:= ina_reg.config;
      buftx[5]:= ina_reg.config  shr 8;

      if SendBlk(4) then begin
        if not ReadBlk(buftx[1]) or (lenrx <> 4) then begin
          ShowMessage('Send config reg error!');
          FormConfigOk := mrIgnore;
        end
        else begin
          StatusBar.Panels[2].Text:='Регистр конфигурации записан в устройство.';
        end;
      end else begin
        ShowMessage('Send config reg error!');
        FormConfigOk := mrIgnore;
      end;
      ShowSmps;
      if SamplesEna then Timer1.Enabled := True;
    end;
  end;
end;

procedure TfrmMain.ButtonStoreClick(Sender: TObject);
begin
    ButtonStopClick(Sender);
    ButtonConfigRegClick(Sender);
    if FormConfigOk = mrOk then begin
      Timer1.Enabled := False;
      if SetDevIniCfg(0) then begin
        buftx[1]:=CMD_DEV_SCF; // Store CFG/ini in Flash
        buftx[2]:=$01;
        if SendBlk(1) then begin
          if ReadBlk(buftx[1]) and (lenrx >= 0) then begin
            StatusBar.Panels[2].Text:='Конфигурация записана во Flash в устройстве на '+ sComNane+'.';
          end;
        end;
     end;
     if SamplesEna then Timer1.Enabled := True;
   end;
end;

function TfrmMain.StopReadDevice: boolean;
var
 i : integer;
begin
  result := False;
  for i:=0 to 5 do begin
    purge_com := 1;

    buftx[1]:=CMD_DEV_CFG; // Set/Get CFG/ini & Start measure
    buftx[2]:=0; // read regs = 0

    if SendBlk(1) then begin
      if ReadBlk(buftx[1]) and
      (lenrx >= SizeOf(blk_cfg)) then begin
        move(bufrx, blk_cfg, SizeOf(blk_cfg));
        sleep(10);
        purge_com := 1;
        result := True;
        break;
      end;
    end;
  end;
end;

function TfrmMain.GetDevVersion: boolean;
var
 i : integer;
begin
  result := False;
  dev_i2c_id := NO_I2C_DID;
  for i:=0 to 5 do begin
    purge_com := 1;
    buftx[1]:= CMD_DEV_VER; // Get Version
    if SendBlk(0) then begin
      if ReadBlk(buftx[1]) and (lenrx = 4) then begin
        dev_type := bufrx[1];
        dev_id := bufrx[0];
        dev_ver := bufrx[2] or (bufrx[3] shl 8);
        StatusBar.Panels[2].Text:='Устройство ID:' + IntToHex(dev_type, 2) + '-' + IntToHex(dev_id, 2) +' версии '+IntToStr((dev_ver shr 12) and $0f) +'.'+IntToStr((dev_ver shr 8) and $0f)+'.'+IntToStr((dev_ver shr 4) and $0f)+'.'+IntToStr(dev_ver and $0f)+' подключено на '+ sComNane +'.';
        if (dev_id = I2C_DEVICE_ID) or (dev_id = ADC1_DEVICE_ID) or (dev_id = ADC2_DEVICE_ID)  then begin
          if not StopReadDevice then begin
              StatusBar.Panels[2].Text:=StatusBar.Panels[2].Text + ' Ошибка в команде останова!';
              ResetIna2xx;
              //dev_i2c_id := NO_I2C_DID;
              //exit;
          end;
          if not ReadRegister(0) then ResetIna2xx;
          if ReadRegister(INAxxx_MID_REG)
            and ReadRegister(INAxxx_DID_REG) then begin
              if (ina_reg.w[INAxxx_MID_REG] = INAxxx_MID) then begin
                case ina_reg.w[INAxxx_DID_REG] of
                  INA226_DID : begin
                    StatusBar.Panels[2].Text:=StatusBar.Panels[2].Text + ' Найдена INA226.';
                    dev_i2c_id := INA226_DID;
                  end;
                  INA3221_DID : begin
                    StatusBar.Panels[2].Text:=StatusBar.Panels[2].Text + ' Найдена INA3221.';
                    dev_i2c_id := INA3221_DID;
                  end
                  else
                    dev_i2c_id := INA219_DID;
                end;
              end
              else begin
                    dev_i2c_id := INA219_DID;
              end;
              result := True;
              exit;
          end else
            if (dev_id = ADC1_DEVICE_ID) or (dev_id = ADC2_DEVICE_ID) then begin
              dev_i2c_id := NO_I2C_DID;
              result := True;
              exit;
            end else begin
              StatusBar.Panels[2].Text:=StatusBar.Panels[2].Text + ' INAxxx не найдена!';
            end;
        end;
      end;
    end;
  end;
end;


function TfrmMain.ResetIna2xx : boolean;
var
 ft : boolean;
 fs : boolean;
begin
   result := true;
   ft := Timer1.Enabled;
   fs := SamplesEna;
   Timer1.Enabled := False;

   buftx[1]:=CMD_DEV_SRG; // Cmd: Set word

   buftx[2]:=INA2XX_I2C_ADDR;
   buftx[3] := 0;
   buftx[4] := $80;
   buftx[5] := 0;

   if SendBlk(4) and ReadBlk(buftx[1]) and (lenrx = 4) then begin
     if not RdAllRegs then begin
       result := false;
     end;
   end else begin
     result := false
   end;

   buftx[1]:=CMD_DEV_SRG; // Cmd: Set word

   buftx[2]:=$18;
   buftx[3] := $80;
   buftx[4] := 0;
   buftx[5] := 5;

   SendBlk(4);
   ReadBlk(buftx[1]);

   Timer1.Enabled := ft;
   SamplesEna := fs;
end;

function TfrmMain.ReadRegister(regnum : integer) : boolean;
var
 i: integer;
 ft : boolean;
 fs : boolean;
begin
   ft := Timer1.Enabled;
   fs := SamplesEna;
   Timer1.Enabled := False;
   i := regnum and $ff;
   result := true;
   buftx[1]:=CMD_DEV_GRG; // Cmd: Get word
//   buftx[3]:=2;
   buftx[2]:=INA2XX_I2C_ADDR;
   buftx[3]:= regnum;
   if SendBlk(2) then begin
     if ReadBlk(buftx[1]) and (lenrx = 4) then begin
        ina_reg.w[i] := bufrx[3] or (bufrx[2] shl 8);
     end else begin
      result := false;
     end;
   end else begin
      result := false;
   end;
   Timer1.Enabled := ft;
   SamplesEna := fs;
end;


procedure TfrmMain.ButtonRdAllRegsClick(Sender: TObject);
begin
   if RdAllRegs then begin
     ShowAllRegs;
     StatusBar.Panels[2].Text:='REGS: ' + EditRegs.Text;
   end;
end;

procedure TfrmMain.AddCrfSamples;
var
k, idx : double;
flg : boolean;
t, irx,itx : dword;
begin
  t := blk_cfg.time_us shl blk_cfg.multiplier;
  if ChartEnables = CHART_UI_MASK then begin
    k := t/500.0;
  end
  else begin
    k := t/1000.0;
  end;
  idx := SamplesCount*k;
  if idx >= MaxSamples then begin
    SamplesAutoStop := True;
    ShowLabelsMX;
    exit;
  end;
  irx := ismprx and SMP_BUF_CNT;
  itx := ismptx and SMP_BUF_CNT;
  while(irx <> itx) do begin
    if (ChartEnables and CHART_I_MASK) <> 0 then begin
      OldCurI := CurI;
      if work_adc then
        CurI := Word(bufsmp[irx])*Ik + I_zero
      else
        CurI := bufsmp[irx]*Ik + I_zero;
      Inc(irx);
      irx := irx and SMP_BUF_CNT;
    end;
    if (ChartEnables and CHART_U_MASK) <> 0 then begin
      OldCurU := CurU;
      CurU := Word(bufsmp[irx])*Uk + U_zero;
      Inc(irx);
      irx := irx and SMP_BUF_CNT;
    end;
    flg := True;
    if SamplesCount = 0 then begin
      if ((ChartEnables and CHART_I_MASK) <> 0) then begin
        if TriggerIEnable then begin
          // ждем триггера I
          if (CurI < TriggerI) then begin
            TriggerILow := True;
            flg := False;
          end
          else begin
            if not TriggerILow then
              flg := False;
          end;
        end;
      end;
      if ((ChartEnables and CHART_U_MASK) <> 0) then begin
        if TriggerUEnable then begin
          // ждем триггера U
          if (CurU < TriggerU) then begin
            TriggerULow := True;
            flg := False;
          end
          else begin
            if not TriggerULow then
              flg := False;
          end;
        end;
      end;
      if flg then begin
        if (TriggerIEnable or TriggerUEnable) then begin // поехали ?
          // Втсавить пред. точку
          if (ChartEnables and CHART_I_MASK) <> 0 then begin
            Chart.Series[CHART_I_NUM].Clear;
            Chart.Series[CHART_I_NUM].AddXY(idx, OldCurI,'',0);
            SumI := SumI + CurI;
          end;
          if (ChartEnables and CHART_U_MASK) <> 0 then begin
            Chart.Series[CHART_U_NUM].Clear;
            if (ChartEnables = CHART_UI_MASK) then
                Chart.Series[CHART_U_NUM].AddXY(idx+k/2, OldCurU,'',0)
            else
                Chart.Series[CHART_U_NUM].AddXY(idx, OldCurU,'',0);
            SumU := SumU + CurU;
          end;
          Inc(SamplesCount);
          idx := SamplesCount*k; //  idx := idx + k;
        end;
      end;
    end; // if SamplesCount = 0
    if (ChartEnables = CHART_UI_MASK) then begin
      Chart.Series[CHART_I_NUM].AddXY(idx, CurI,'',0);
      Chart.Series[CHART_U_NUM].AddXY(idx+k/2, CurU,'',0);
      if flg then begin
        SumU := SumU + CurU;
        SumI := SumI + CurI;
      end;
    end
    else if (ChartEnables and CHART_I_MASK) <> 0 then begin
      Chart.Series[CHART_I_NUM].AddXY(idx, CurI,'',0);
      if flg then begin
        SumI := SumI + CurI;
      end;
    end
    else if (ChartEnables and CHART_U_MASK) <> 0 then begin
      Chart.Series[CHART_U_NUM].AddXY(idx, CurU,'',0);
      if flg then begin
        SumU := SumU + CurU;
      end;
    end;
    if flg then begin
      Inc(SamplesCount);
      idx := SamplesCount*k; // idx := idx + k;
      if idx >= MaxSamples then begin
//        ShowSmps;
        ShowLabelsMX;
        if (not ReadStatus)
        or (not StopReadDevice) then
          StatusBar.Panels[2].Text:='Ошибка передачи в устройство на '+ sComNane+'!'
        else begin
          StatusBar.Panels[2].Text:='Непрерывное чтение остановлено по концу буфера. Пропуск ' + IntToStr(dev_not_send_count) + ' блоков, счетчик: ' + IntToStr(dev_all_send_count) + '.';
          SamplesEna := False;
          Timer1.Enabled := False;
          purge_com := 1;
        end;
        SamplesAutoStop := True;
        break;
      end; // if idx >= MaxSamples
    end; // if flg
  end; // while(i < count)
  ismprx := irx;
end;



procedure TfrmMain.TimerTimer(Sender: TObject);
var
 t : TDateTime;
begin
    if SamplesEna then begin
      if ismptx <> ismprx then begin
         AddCrfSamples();
      end;
      if dev_send_err <> 0 then begin
        if not StopReadDevice then
          StatusBar.Panels[2].Text:='Ошибка передачи команды останова в устройство на '+ sComNane+'!'
        else begin
          StatusBar.Panels[2].Text:='Сбой получения данных, ошибка N' + IntToStr(dev_send_err)
          +': пропуск ' + IntToStr(dev_not_send_count) + ' блоков, счетчик: ' + IntToStr(dev_all_send_count) + '!';
          SamplesEna := False;
          Timer1.Enabled := False;
          purge_com := 1;
        end;
        dev_send_err := 0;
        SamplesAutoStop := True;
      end;
      t := GetTime;
      if (t - ConnectStartTime) > 2/864000.0 then begin
        ConnectTime := ConnectTime + t - ConnectStartTime;
        ConnectStartTime := t;
//        StatusBar.Panels[1].Text := FormatFloat('# ##0.', (SamplesCount/86400.0)/ConnectTime ) + ' sps';
//        ShowSmps;
        ShowLabelsMX;
      end;
    end;
end;


procedure TfrmMain.ClearGrf;
var
i : integer;
begin
    for i:=0 to Chart.SeriesCount-1 do Chart.Series[i].Clear;
    ConnectTime := 0;
    ConnectStartTime := GetTime;
    SamplesCount := 0;
    SumI := 0.0;
    SumU := 0.0;
end;

procedure TfrmMain.ButtonClrGrfClick(Sender: TObject);
var
tmp1, tmp2 : boolean;
begin
    tmp1:=SamplesEna;
    tmp2:=Timer1.Enabled;
    if flgComOpen then begin
      SamplesEna := False;
      Timer1.Enabled := False;
      purge_com := 1;
      ismptx := 0;
      ismprx := 0;
      ClearGrf;
      SetParStart;
      ConnectStartTime := GetTime;
      SamplesEna:=tmp1;
      Timer1.Enabled:=tmp2;
    end
    else
      ClearGrf;
end;


procedure TfrmMain.ButtonPrtGrfClick(Sender: TObject);
begin
    ChartPreviewer.Execute;
end;

procedure TfrmMain.ButtonSaveGRFClick(Sender: TObject);
var
i, data_cont : dword;
bfile : THandle;
s: string;
x,y : double;
begin
    case ChartEnables of
     CHART_I_MASK : data_cont := Chart.Series[CHART_I_NUM].Count;
     CHART_U_MASK : data_cont := Chart.Series[CHART_U_NUM].Count;
     CHART_UI_MASK : data_cont := Chart.Series[CHART_U_NUM].Count;
    else data_cont := 0;
    end;
    if (data_cont <> 0) then begin
     with SaveDialog do begin
      FilterIndex:=0;
      FileName := 'data.csv';
      InitialDir := '.';
      DefaultExt := 'csv';
      Filter := 'csv files (*.csv)|*.csv';
      Options:=Options+[ofFileMustExist]-[ofHideReadOnly]
        +[ofNoChangeDir]-[ofNoLongNames]-[ofNoNetworkButton]-[ofHideReadOnly]
        -[ofOldStyleDialog]-[ofOverwritePrompt]+[ofPathMustExist]
        -[ofReadOnly]-[ofShareAware]-[ofShowHelp];
      Title:='Сохранить данные в файл';
     end;//with
     if SaveDialog.Execute then begin
      Repaint;
      bfile := FileCreate(SaveDialog.FileName);
      if bfile<>INVALID_HANDLE_VALUE then begin
        s:='X;Y;SPS '+FormatFloat('0.000',current_smps)+#13+#10;
        FileWrite(bfile, s[1], Length(s));
        for i:=0 to data_cont -1  do begin
          if (ChartEnables and 1) <> 0 then begin
            try
             x := Chart.Series[CHART_U_NUM].YValue[i];
            except
             x := 0;
            end;
          end else x := 0;
          if (ChartEnables and 2) <> 0 then begin
            try
             y:=Chart.Series[CHART_I_NUM].YValue[i];
            except
             y := -4096;
            end;
          end else y := 0;
          s:=FloatToStr(x)+';'
           +FloatToStr(y)+#13+#10;
          FileWrite(bfile, s[1],Length(s));
        end;
      end;
      FileClose(bfile);
     end;
    end;
end;

function TfrmMain.GrfSetColor(ChNum : integer): integer;
begin
  ColorDialog.Color := Chart.Series[ChNum].SeriesColor; //xShape[ChNum].Brush.Color;
  if ColorDialog.Execute then
   Chart.Series[ChNum].SeriesColor:=ColorDialog.Color;
  result := ColorDialog.Color
end;


procedure TfrmMain.ButtonAsizeClick(Sender: TObject);
begin
   if(Chart.LeftAxis.Automatic) then begin
    ButtonASize.Caption:='Auto';
    Chart.LeftAxis.Automatic:=False;
    Chart.RightAxis.Automatic:=False;
    ButtonScalM.Visible:=True;
    ButtonScalP.Visible:=True;
  end
  else begin
    ButtonASize.Caption:='Manual';
    Chart.LeftAxis.Automatic:=True;
    Chart.RightAxis.Automatic:=True;
    ButtonScalM.Visible:=False;
    ButtonScalP.Visible:=False;
  end;
end;

procedure TfrmMain.ButtonScalMClick(Sender: TObject);
var
d : double;
begin
  if(not Chart.LeftAxis.Automatic) then begin
    d:= (Chart.LeftAxis.Maximum - Chart.LeftAxis.Minimum)/25;
    Chart.LeftAxis.Minimum:=Chart.LeftAxis.Minimum - d;
    Chart.LeftAxis.Maximum:=Chart.LeftAxis.Maximum + d;
    d:= (Chart.RightAxis.Maximum - Chart.RightAxis.Minimum)/25;
    Chart.RightAxis.Minimum:=Chart.RightAxis.Minimum - d;
    Chart.RightAxis.Maximum:=Chart.RightAxis.Maximum + d;
  end;
end;


procedure TfrmMain.ButtonScalPClick(Sender: TObject);
var
d : double;
begin
  if(not Chart.LeftAxis.Automatic) then begin
    d:= (Chart.LeftAxis.Maximum - Chart.LeftAxis.Minimum)/25;
    Chart.LeftAxis.Minimum:=Chart.LeftAxis.Minimum + d;
    Chart.LeftAxis.Maximum:=Chart.LeftAxis.Maximum - d;
    d:= (Chart.RightAxis.Maximum - Chart.RightAxis.Minimum)/25;
    Chart.RightAxis.Minimum:=Chart.RightAxis.Minimum + d;
    Chart.RightAxis.Maximum:=Chart.RightAxis.Maximum - d;
  end;
end;


procedure TfrmMain.SetGrfMarging;
begin
  if CheckBoxOpenGL.Checked then begin
    Chart.MarginBottom := 5;
    Chart.MarginTop := 2;
    Chart.MarginRight := 2;
    Chart.MarginLeft := 2;
  end else begin
    Chart.MarginBottom := 2;
    Chart.MarginLeft := 2;
    Chart.MarginRight := 2;
    Chart.MarginTop := 2;
  end;
end;

procedure TfrmMain.CheckBoxOpenGLClick(Sender: TObject);
begin
  TeeOpenGL1.Active := CheckBoxOpenGL.Checked;
  SetGrfMarging;
end;


procedure TfrmMain.CheckBoxTrigerClick(Sender: TObject);
begin
  if flgValueChg then
    SetParStart;
end;

procedure TfrmMain.SetParStart;
begin
        GetScrParms;
        ShowScrParms;

        TriggerIEnable := CheckBoxTrigerRiseI.Checked;
        TriggerUEnable := CheckBoxTrigerRiseU.Checked;
        TriggerILow := False;
        TriggerULow := False;

        if not DeviceTypeRecognized then exit;
        if work_adc then begin
		      if (dev_id = ADC1_DEVICE_ID) then
            FormAdcConfig.GetParams;
    		  if (dev_id = ADC2_DEVICE_ID) then
            FormAdc2Config.GetParams;
          exit;
        end;

        case dev_i2c_id of
          INA226_DID : begin
            Form226Config.GetParams;
//            Form226Config.SetParamIU(Ik, Uk);
          end;
          INA3221_DID : begin
            Form3221Config.GetParams;
//            Form3221Config.SetParamIU(Ik, Uk);
          end;
          INA219_DID : begin
            Form219Config.GetParams;
//            Form219Config.SetParamIU(Ik, Uk);
          end;
          else
            exit;
        end;
end;

procedure TfrmMain.SetColorIMouseDown(Sender: TObject;
  Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
begin
  if Button = mbRight then // mbLeft
   SetColorI.Color := GrfSetColor(CHART_I_NUM)
  else
    if Chart.Series[CHART_I_NUM].Visible then
      Chart.Series[CHART_I_NUM].Visible := False
    else
      Chart.Series[CHART_I_NUM].Visible := True;

end;

procedure TfrmMain.SetColorUMouseDown(Sender: TObject;
  Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
begin
  if Button = mbRight then // mbLeft
    SetColorU.Color := GrfSetColor(CHART_U_NUM)
  else
   if Chart.Series[CHART_U_NUM].Visible then
    Chart.Series[CHART_U_NUM].Visible := False
   else
    Chart.Series[CHART_U_NUM].Visible := True;
end;

function TfrmMain.SetAdcIniCfg(mode : integer) : boolean;
const
  ADC_DFIFO_CLK = 16000000;
  SOFT_OUT_SCALE = 1;
  MAX_PERIOD_CH0 = 4095; // Max ADC auto channel 0 (Misc) period max (adc_period_chn0 * system clocks)
  MIN_PERIOD_CH0 = 77; 	// channel ADC period (adc_period_chn0 * system clocks)
  MAX_PERIOD_CH1 = 255; // ADC auto channel 1 (L)& 2 period max (adc_period_chn12 * 16 system clocks)
  MIN_PERIOD_CH1 = 15; // ADC auto channel 1 (L)& 2 period min (adc_period_chn12 * 16 system clocks)

  MAX_OUT_SPS_D1 = ((ADC_DFIFO_CLK div (1*SOFT_OUT_SCALE)) div (MIN_PERIOD_CH0 + MIN_PERIOD_CH1*16));
  MIN_OUT_SPS = ((ADC_DFIFO_CLK div (8*SOFT_OUT_SCALE)) div (MAX_PERIOD_CH0 + MAX_PERIOD_CH1*16));
var
  t_us : dword;
  smprate : dword;
  channel : byte;
begin
   result := False;
   Timer1.Enabled := False;
   if mode <> 0 then
     smprate := ADC_smps
   else
     smprate := 1000;
   channel := ADC_channel;

   if(smprate > MAX_OUT_SPS_D1) then // max 16000000/1/(77+16*15) = 50473.18612 sps
     smprate := MAX_OUT_SPS_D1
   else if(smprate < MIN_OUT_SPS) then // min 16000000/(8*1)/(4095+16*255) = 244.648318 sps
     smprate := MIN_OUT_SPS;

   if mode <> 0 then begin
     t_us := 1000000 div smprate;
     blk_cfg.multiplier := 0;
     while(t_us > $ffff) do begin
      t_us := t_us shr 1;
      Inc(blk_cfg.multiplier);
     end;
     blk_cfg.time_us := t_us;
   end;

   buftx[0] := 6;  // size
   buftx[1] := 8;  // CMD_DEV_CAD  Get/Set CFG/ini ADC & Start measure
   if mode = 0 then
     buftx[2] := 0 // pktcnt - кол-во передаваемых значений ADC в одном пакете передачи
   else begin
//     if dev_id = ADC1_DEVICE_ID then
//       buftx[2] := 30 //MAX_BLK_DEV2 // pktcnt - кол-во передаваемых значений ADC в одном пакете передачи
//     else
       buftx[2] := MAX_BLK_DEV1; // pktcnt - кол-во передаваемых значений ADC в одном пакете передачи
   end;
   buftx[3] := channel; // channel 9 - PC4
   buftx[4] := smprate;  // период adc chl0 lsb
   buftx[5] := smprate shr 8; // период adc chl0 msb
   buftx[6] := PGA20db;
   buftx[7] := PGA2db5;
   if SendBlk(6) then begin
     if ReadBlk(buftx[1]) and (lenrx >= 6) then begin
       StatusBar.Panels[2].Text:='Конфигурация ADC передана в устройство на '+ sComNane+'.';
       result := True;
     end
     else begin
       StatusBar.Panels[2].Text:='Ошибка записи ADC конфигурации в устройство на '+ sComNane+'!';
     end;
   end;
   if SamplesEna then Timer1.Enabled := True;
end;

function TfrmMain.SetAdc2IniCfg(mode : integer) : boolean;
var
  t_us : dword;
  smprate : dword;
  channel : byte;
begin
   result := False;
   Timer1.Enabled := False;
   if mode <> 0 then
     smprate := FormAdc2Config.Checksmps(ADC2_smps)
   else begin
     smprate := FormAdc2Config.Checksmps(1000);
     // ADC2_smps := smprate;
   end;
   channel := ADC2_channel;

   if mode <> 0 then begin
     t_us := 1000000 div smprate;
     blk_cfg.multiplier := 0;
     while(t_us > $ffff) do begin
      t_us := t_us shr 1;
      Inc(blk_cfg.multiplier);
     end;
     blk_cfg.time_us := t_us;
   end;

   buftx[0] := 6;  // size
   buftx[1] := 8;  // CMD_DEV_CAD  Get/Set CFG/ini ADC & Start measure
   if mode = 0 then
     buftx[2] := 0 // pktcnt - кол-во передаваемых значений ADC в одном пакете передачи
   else begin
     buftx[2] := 126; // pktcnt - кол-во передаваемых значений ADC в одном пакете передачи
   end;
   buftx[3] := channel; // channel
   buftx[4] := smprate;  // период adc
   buftx[5] := smprate shr 8; // период adc
   buftx[6] := PGA20dbA2;
   buftx[7] := PGA2db5A2;
   if SendBlk(6) then begin
     if ReadBlk(buftx[1]) and (lenrx >= 6) then begin
       StatusBar.Panels[2].Text:='Конфигурация ADC передана в устройство на '+ sComNane+'.';
       result := True;
     end
     else begin
       StatusBar.Panels[2].Text:='Ошибка записи ADC конфигурации в устройство на '+ sComNane+'!';
     end;
   end;
   if SamplesEna then Timer1.Enabled := True;
end;


procedure TfrmMain.ButtonStartADCClick(Sender: TObject);
var
t, maxs: dword;
begin
    SamplesEna := False;
    Timer1.Enabled := False;
    ButtonStartADC.Caption := 'Start ADC';
    if (dev_id = ADC1_DEVICE_ID) then begin
    	if not SetAdcIniCfg(1) then exit;
    end;
    if (dev_id = ADC2_DEVICE_ID) then begin
    	if not SetAdc2IniCfg(1) then exit;
    end;
    ConnectStartTime := GetTime;
    StatusBar.Panels[2].Text:='Запущено непрерывное чтение ADC в устройстве на '+ sComNane+'.';
    work_adc := True;
    SetParStart;
    if (dev_id = ADC1_DEVICE_ID) then begin
      if (UI_adc <> 0) then
          ChartEnables := CHART_U_MASK
      else
          ChartEnables := CHART_I_MASK;
    end else begin
      if (UI_adc2 <> 0) then
          ChartEnables := CHART_U_MASK
      else
          ChartEnables := CHART_I_MASK;
    end;
    t := blk_cfg.time_us shl blk_cfg.multiplier; // t us
    if t <> 0 then begin
      maxs := 3000*t; // ms
      if MaxSamples > maxs then begin
         MaxSamples := maxs; // ms
         EditSizeGrf.Text := IntToStr(MaxSamples); // ms
      end;
    end;
    ShowSmps;
    if SamplesAutoStop then begin
       SamplesAutoStop := False;
       ClearGrf;
    end;
//      dev_send_smps := 0;
    ismptx := 0;
    ismprx := 0;
    SamplesEna := True;
    Timer1.Enabled := True;
end;

procedure TfrmMain.ButtonADCcfgClick(Sender: TObject);
begin
  if (dev_id = ADC1_DEVICE_ID) then begin
    FormAdcConfig.Left := Left + (Width div 2) - FormAdcConfig.Width div 2;
    FormAdcConfig.Top := Top + (Height div 2) - FormAdcConfig.Height div 2;
    FormConfigOk := FormAdcConfig.ShowModal;
//    if FormConfigOk = mrOk then begin
  end else begin
    FormAdc2Config.Left := Left + (Width div 2) - FormAdc2Config.Width div 2;
    FormAdc2Config.Top := Top + (Height div 2) - FormAdc2Config.Height div 2;
    FormConfigOk := FormAdc2Config.ShowModal;
  end
end;

procedure TfrmMain.ButtonSaveWavClick(Sender: TObject);
var
i, data_cont : dword;
stereo : boolean;
bfile : THandle;
f,x,y : double;
val : array [0..1] of short;
resolution: integer;
begin
	resolution := 16; // 16 or 24 bits
    case ChartEnables of
     CHART_I_MASK : data_cont := Chart.Series[CHART_I_NUM].Count;
     CHART_U_MASK : data_cont := Chart.Series[CHART_U_NUM].Count;
     CHART_UI_MASK : data_cont := Chart.Series[CHART_U_NUM].Count;
    else data_cont := 0;
    end;
    if (data_cont <> 0) then begin
     with SaveDialog do begin
      FilterIndex:=0;
      FileName := 'data.wav';
      InitialDir := '.';
      DefaultExt := 'wav';
      Filter := 'wav files (*.wav)|*.wav';
      Options:=Options+[ofFileMustExist]-[ofHideReadOnly]
        +[ofNoChangeDir]-[ofNoLongNames]-[ofNoNetworkButton]-[ofHideReadOnly]
        -[ofOldStyleDialog]-[ofOverwritePrompt]+[ofPathMustExist]
        -[ofReadOnly]-[ofShareAware]-[ofShowHelp];
      Title:='Сохранить данные в файл';
     end;//with
     if SaveDialog.Execute then begin
      Repaint;
      bfile := FileCreate(SaveDialog.FileName);
      if bfile<>INVALID_HANDLE_VALUE then begin
        f := current_smps;
        if (not work_adc) and (ChartEnables = CHART_UI_MASK) then
          f := current_smps/2;
        if f = 0.0 then
          f := 8000.0
        else if f < 100.0 then
          f := f * 1000.0;
        if(ChartEnables = CHART_UI_MASK) then
          stereo := True
        else
          stereo := False;
        SetWavHeader(stereo, Round(f), data_cont, resolution);
        FileWrite(bfile, wav_header, sizeof(wav_header));

		    if resolution = 24 then begin
	        for i:=0 to data_cont -1  do begin
	          if (ChartEnables and CHART_U_MASK) <> 0 then begin
	            try
	             x := (Chart.Series[CHART_U_NUM].YValue[i]-U_zero)/Uk - 8388608.0;
	            except
	             x := 0;
	            end;
	            if(x > 8388607.0) then x := 8388607.0
            else if(x < -8388608.0) then x := -8388608.0;
            val[0] := Round(x); //Trunc(x);
	          end else val[0] := 0;

	          if (ChartEnables and CHART_I_MASK) <> 0 then begin
	            try
	             y := (Chart.Series[CHART_I_NUM].YValue[i]-I_zero)/Ik;
	            except
	             y := 0;
	            end;
                if work_adc then y := y - 8388608.0;
                if(y > 8388607.0) then y := 8388607.0
	            else if(y < -8388608.0) then y := -8388608.0;
	            val[1] := Round(y); // Trunc(y);
	          end else val[1] := 0;
	          case ChartEnables of
               CHART_U_MASK : FileWrite(bfile, val[0] , 3); //sizeof(val[0]));
	           CHART_I_MASK : FileWrite(bfile, val[1] , 3); //sizeof(val[1]));
	           CHART_UI_MASK : begin
	                 FileWrite(bfile, val[0] , 3); //sizeof(val[0]));
	                 FileWrite(bfile, val[1] , 3); //sizeof(val[1]));
	               end;
	          end;
	        end;
		    end else if resolution = 16 then begin
            for i:=0 to data_cont -1  do begin
	          if (ChartEnables and CHART_U_MASK) <> 0 then begin
	            try
	             x := (Chart.Series[CHART_U_NUM].YValue[i]-U_zero)/Uk - 32768.0;
	            except
	             x := 0;
	            end;
	            if(x > 32767.0) then x := 32767.0
	            else if(x < -32768.0) then x := -32768.0;
                val[0] := Round(x); //Trunc(x);
              end else val[0] := 0;

	          if (ChartEnables and CHART_I_MASK) <> 0 then begin
	            try
	             y := (Chart.Series[CHART_I_NUM].YValue[i]-I_zero)/Ik;
	            except
	             y := 0;
	            end;
	            if work_adc then y := y - 32768.0;
                if(y > 32767.0) then y := 32767.0
                else if(y < -32768.0) then y := -32768.0;
	            val[1] := Round(y); // Trunc(y);
	          end else val[1] := 0;
	          case ChartEnables of
	           CHART_U_MASK : FileWrite(bfile, val[0] , sizeof(val[0]));
	           CHART_I_MASK : FileWrite(bfile, val[1] , sizeof(val[1]));
	           CHART_UI_MASK : FileWrite(bfile, val , sizeof(val));
	          end;
	        end;
        end;
        FileClose(bfile);
      end;
     end;
    end;
end;

end.

