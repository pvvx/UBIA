unit WaveStorage;

interface

uses
  Windows;

type

_WAV_HEADER = packed record // https://ccrma.stanford.edu/courses/422/projects/WaveFormat/
  RIFF  : dword; //* +00 'RIFF'           */
  size8 : dword; //* +04 file size - 8    */
  WAVE  : dword; //* +08 'WAVE'           */
  fmt   : dword; //* +12 'fmt '           */
  fsize : dword; //* +16 указатель до 'fact' или 'data' */
  ccod  :  word; //* +20 01 00  Compression code: 1 - PCM/uncompressed */
  mono  :  word; //* +22 00 01 или 00 02  */
  freq  : dword; //* +24 частота          */
  bps   : dword; //* +28                  */
  blka  :  word; //* +32 1/2/4  BlockAlign*/
  bits  :  word; //* +34 разрядность 8/16 */
  data  : dword; //* +36 'data'           */
  dsize : dword; //* +40 размер данных    */
end;

var 

wav_header : _WAV_HEADER = (
  RIFF  : $46464952; //* +00 'RIFF'           */
  size8 : $00000008; //* +04 file size - 8    */
  WAVE  : $45564157; //* +08 'WAVE'           */
  fmt   : $20746d66; //* +12 'fmt '           */
  fsize : $00000010; //* +16 указатель до 'fact' или 'data' */
  ccod  : $0001    ; //* +20 01 00  Compression code: 1 - PCM/uncompressed */
  mono  : $0001    ; //* +22 00 01 или 00 02  */
  freq  : $00001F40; //* +24 частота          */
  bps   : $00001F40; //* +28                  */
  blka  : $0002    ; //* +32 1/2/4  BlockAlign*/
  bits  : $0010    ; //* +34 разрядность 8/16 */
  data  : $61746164; //* +36 'data'           */
  dsize : $00000000; //* +40 размер данных    */
);

procedure SetWavHeader(stereo : boolean; smps : dword; samplecount: dword; bits : word);

implementation

procedure SetWavHeader(stereo : boolean; smps : dword; samplecount: dword; bits : word);
begin
  if stereo then wav_header.mono := 2
  else wav_header.mono := 1;
  wav_header.freq := smps;
  wav_header.bps := smps;
  wav_header.bits := bits;
  wav_header.blka :=  bits div 8;
  wav_header.dsize := samplecount * wav_header.blka * wav_header.mono;
  wav_header.size8 := wav_header.dsize + sizeof(wav_header) - 8;
end;

end.
