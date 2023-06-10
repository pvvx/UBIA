program PowerProfiler;

uses
  Forms,
  MainFrm in 'MainFrm.pas' {frmMain},
  ComPort in 'ComPort.pas',
  HexUtils in 'HexUtils.pas',
  Ina219_r_config in 'Ina219_r_config.pas' {Form219Config},
  Ina226_r_config in 'Ina226_r_config.pas' {Form226Config},
  Ina3221_r_config in 'Ina3221_r_config.pas' {Form3221Config},
  adc_jdy10_config in 'adc_jdy10_config.pas' {FormAdcConfig},
  adc_bl702_config in 'adc_bl702_config.pas' {FormAdc2Config},
  WaveStorage in 'WaveStorage.pas';

{$E exe}

{$R *.res}

begin
  Application.Initialize;
  Application.Title := 'PowerProfiler';
  Application.CreateForm(TfrmMain, frmMain);
  Application.CreateForm(TForm219Config, Form219Config);
  Application.CreateForm(TForm226Config, Form226Config);
  Application.CreateForm(TFormAdcConfig, FormAdcConfig);
  Application.CreateForm(TForm3221Config, Form3221Config);
  Application.CreateForm(TFormAdcConfig, FormAdcConfig);
  Application.Run;
end.
