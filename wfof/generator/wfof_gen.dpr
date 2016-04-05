program wfof_gen;

{$APPTYPE CONSOLE}

{
 * Created by Martin Winkelhofer 02,03/2016
 * W-Dimension / wdim / wdim0 / maarty.w@gmail.com
 *    _____ __          ____         ______         __
 *   / __(_) /__ ___   / __ \___    / __/ /__ ____ / /
 *  / _// / / -_|_-<  / /_/ / _ \  / _// / _ `(_-</ _ \
 * /_/ /_/_/\__/___/  \____/_//_/ /_/ /_/\_,_/___/_//_/
 *
 * This file is part of WFOF - W-Dimension's Files On Flash (for ESP8266).
 *
 * WFOF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WFOF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WFOF. If not, see <http://www.gnu.org/licenses/>.
}

uses
  SysUtils, Classes;

const
  APP_NAME = 'wfof_gen v2.0';

type
  Fl = record
    Offs: integer;
    Size: integer;
    Name: string;
  end;

var
  CalledDir,InDir,OutFileDataH,OutFileIdxsH: string;
  SR: TSearchRec;
  TC1: integer = 0;
  TC2: integer;
  SL,SL2: TStringList;
  Buf: array of byte;
  OutBuf: array of byte;
  FilesCnt: integer = 0;
  Files: array of Fl;
  FilesStr: string;
  FilesIdxsStr: string;
  FilesSizesStr: string;
  FilesDataStr: string;
  FilesDataStrLen: integer;
  TmpS: string;
  FilesDataStrOffs: integer;
  Params: string;
  DoUppercase: boolean = false;
  AlignBytes: byte = 4;
  ItemsOnLine: byte;

procedure AddFileToBuf(FN: string; FullFN: string; Sz: integer);
var
  FH,LastOBLen: integer;
  SzMod, SzAdd: byte;
begin
  try
    LastOBLen:=Length(OutBuf);
    SetLength(Files,Length(Files)+1);
    Files[FilesCnt].Offs:=LastOBLen div AlignBytes; //is always divisible without remainder because of "SetLength(OutBuf,LastOBLen+Sz+SzAdd);" later
    Files[FilesCnt].Size:=Sz;
    Files[FilesCnt].Name:=FN;
    Inc(FilesCnt);
    //
    SetLength(Buf,Sz);
    FH:=FileOpen(FullFN,soFromBeginning);
    FileRead(FH,Buf[0],Sz);
    FileClose(FH);
    //
    SzMod:=Sz mod AlignBytes;
    if(SzMod=0) then SzAdd:=0
    else SzAdd := AlignBytes - SzMod;
    //
    SetLength(OutBuf,LastOBLen+Sz+SzAdd); //SzAdd is padding after last byte to fulfill the right alignment defined by AlignBytes
    Move(Buf[0],OutBuf[LastOBLen],Sz);
  except
    on E:Exception do begin
      Writeln(Output,'Error: '+E.Message);
    end;
  end;
end;

function UpperCaseAndReplaceCtrlChars(S: string): string;
var
  Idx: integer;
  Ch: char;
begin
  Result:='';
  S:=UpperCase(S);
  for Idx:=1 to Length(S) do begin
    Ch:=S[Idx];
    if( ((Ch>='0')and(Ch<='9')) or ((Ch>='A')and(Ch<='Z')) ) then Result:=Result+Ch
    else Result:=Result+'_';
  end;
end;

begin
  Writeln(Output,APP_NAME+', created by Martin Winkelhofer 02,03/2016');
  Writeln(Output,'W-Dimension / wdim / wdim0 / maarty.w@gmail.com');
  SetLength(OutBuf,0);
  SetLength(Files,0);
  if(ParamCount<4) then begin
    Writeln(Output,'----------------');
    Writeln(Output,'Usage:');
    Writeln(Output,'wfof_gen.exe <inputdir> <align_bytes> <outputfile_data_h> <outputfile_idxs_h> (<params>)');
    Writeln(Output,'params: -u uppercase');
    Exit;
  end;
  //
  CalledDir:=ExtractFilePath(ParamStr(0));
  InDir:=ParamStr(1);
  try
    AlignBytes:=StrToInt(ParamStr(2));
    if((AlignBytes<>1)and(AlignBytes<>2)and(AlignBytes<>4)) then begin
      Writeln(Output,'Error: <align_bytes> can be only 1, 2, or 4');
      Exit;
    end;
  except
    Writeln(Output,'Error: can''t convert 2nd parameter <align_bytes> to integer');
    Exit;
  end;
  OutFileDataH:=ParamStr(3);
  OutFileIdxsH:=ParamStr(4);
  //
  if((CalledDir<>'')and(CalledDir[Length(CalledDir)]<>'\')) then CalledDir:=CalledDir+'\'; //add last '\' if CalledDir is not empty and it's not ending with '\'
  if(InDir[Length(InDir)]='\') then InDir:=Copy(InDir,1,Length(InDir)); //remove last '\' if any
  //
  SL:=TStringList.Create();
  try
    SL.LoadFromFile(CalledDir+Format('template\wfof_data_align%d.tpl',[AlignBytes]));
  except
    on E:Exception do begin
      Writeln(Output,'Error: '+E.Message);
      Exit;
    end;
  end;
  //
  SL2:=TStringList.Create();
  try
    SL2.LoadFromFile(CalledDir+'template\wfof_idxs.tpl');
  except
    on E:Exception do begin
      Writeln(Output,'Error: '+E.Message);
      Exit;
    end;
  end;
  //
  if(ParamCount>4) then begin
    Params:=ParamStr(5);
    if((Length(Params)>1)and(Params[1]='-')) then begin
      Params:=Copy(Params,2,Length(Params)); //get rid of leading '-'
      DoUppercase:=(Pos('u',Params)<>0);
    end;
  end;
  //
  Writeln(Output,'----------------');
  if(FindFirst(InDir+'\*',faAnyFile,SR)=0) then begin
    repeat
      if((SR.Attr and faDirectory)<>faDirectory) then begin
        Writeln(Output,Format('%s (%d bytes)',[SR.Name,SR.Size]));
        AddFileToBuf(SR.Name,InDir+'\'+SR.Name,SR.Size);
      end;
      //
      Inc(TC1);
      if(TC1>255) then break; //max 255 files
      //
    until(FindNext(SR)<>0);
    FindClose(SR);
  end;
  Writeln(Output,'----------------');
  //
  FilesStr:='';
  FilesIdxsStr:='';
  FilesSizesStr:='';
  for TC2:=0 to FilesCnt-1 do begin //go through all Files[]
    if(DoUppercase) then TmpS:=UpperCase(Files[TC2].Name) else TmpS:=Files[TC2].Name;
    FilesStr:=FilesStr+Format('%s{.Offs=%d, .Size=%d, .Name="%s"},%s',[#9+#9,Files[TC2].Offs,Files[TC2].Size,TmpS,#13+#10]);
    FilesIdxsStr:=FilesIdxsStr+Format('#define WFOF_IDX_%s%s%s%d%s',[UpperCaseAndReplaceCtrlChars(Files[TC2].Name),#9,#9,TC2,#13+#10]);
    FilesSizesStr:=FilesSizesStr+Format('#define WFOF_SIZE_%s%s%s%d%s',[UpperCaseAndReplaceCtrlChars(Files[TC2].Name),#9,#9,Files[TC2].Size,#13+#10]);
  end;
  if(FilesStr<>'') then FilesStr:=Copy(FilesStr,1,Length(FilesStr)-3); //remove last ','+#13+#10
  if(FilesIdxsStr<>'') then FilesIdxsStr:=Copy(FilesIdxsStr,1,Length(FilesIdxsStr)-2); //remove last #13+#10
  if(FilesSizesStr<>'') then FilesSizesStr:=Copy(FilesSizesStr,1,Length(FilesSizesStr)-2); //remove last #13+#10
  //
  FilesDataStr:='';
  case(AlignBytes)of
    4: begin
      FilesDataStrLen:=Length(OutBuf)*(12 div 4); //each 4-byte is exported like this: '0x00000000, '
      FilesDataStrLen:=FilesDataStrLen+((Length(OutBuf) div (16*4))*2); //every 16 4-bytes a new line is added (#13+#10)
    end;
    2: begin
      FilesDataStrLen:=Length(OutBuf)*(8 div 2); //each 2-byte is exported like this: '0x0000, '
      FilesDataStrLen:=FilesDataStrLen+((Length(OutBuf) div (16*2))*2); //every 16 2-bytes a new line is added (#13+#10)
    end;
    else begin
      FilesDataStrLen:=Length(OutBuf)*(6 div 1); //each byte is exported like this: '0x00, '
      FilesDataStrLen:=FilesDataStrLen+((Length(OutBuf) div (16*1))*2); //every 16 bytes a new line is added (#13+#10)
    end;
  end;
  FilesDataStrLen:=FilesDataStrLen-2; //last ', ' is truncated
  if(FilesDataStrLen<0) then FilesDataStrLen:=0;
  SetLength(FilesDataStr,FilesDataStrLen);
  //
  FilesDataStrOffs:=1;
  TC2:=0;
  ItemsOnLine:=0;
  while(TC2<Length(OutBuf)) do begin //go through all bytes in OutBuf
    case(AlignBytes)of
      4: begin
        TmpS:=Format('0x%0.2X%0.2X%0.2X%0.2X, ',[OutBuf[TC2+3],OutBuf[TC2+2],OutBuf[TC2+1],OutBuf[TC2]]);
        TC2:=TC2+4;
      end;
      2: begin
        TmpS:=Format('0x%0.2X%0.2X, ',[OutBuf[TC2+1],OutBuf[TC2]]);
        TC2:=TC2+2;
      end;
      else begin
        TmpS:=Format('0x%0.2X, ',[OutBuf[TC2]]);
        Inc(TC2);
      end;
    end;
    if(TC2=Length(OutBuf)) then TmpS:=Copy(TmpS,1,Length(TmpS)-2); //last ', ' is truncated
    Inc(ItemsOnLine);
    if(ItemsOnLine=16) then begin
      TmpS:=TmpS+#13+#10;
      ItemsOnLine:=0;
    end;
    //
    Move(TmpS[1],FilesDataStr[FilesDataStrOffs],Length(TmpS));
    FilesDataStrOffs:=FilesDataStrOffs+Length(TmpS);
  end;
  //
  //--
  SL.Text:=StringReplace(SL.Text,'[[APPNAME]]',APP_NAME,[rfReplaceAll]);
  SL.Text:=StringReplace(SL.Text,'[[DATETIME]]',DateTimeToStr(Now()),[rfReplaceAll]);
  SL.Text:=StringReplace(SL.Text,'[[FILESCNT]]',IntToStr(FilesCnt),[rfReplaceAll]);
  SL.Text:=StringReplace(SL.Text,'[[ALIGNBYTES]]',IntToStr(AlignBytes),[rfReplaceAll]);
  //
  SL.Text:=StringReplace(SL.Text,'[[FILES]]',FilesStr,[rfReplaceAll]);
  SL.Text:=StringReplace(SL.Text,'[[FILESDATA]]',Trim(FilesDataStr),[rfReplaceAll]);
  //
  //--
  SL2.Text:=StringReplace(SL2.Text,'[[APPNAME]]',APP_NAME,[rfReplaceAll]);
  SL2.Text:=StringReplace(SL2.Text,'[[DATETIME]]',DateTimeToStr(Now()),[rfReplaceAll]);
  SL2.Text:=StringReplace(SL2.Text,'[[FILESCNT]]',IntToStr(FilesCnt),[rfReplaceAll]);
  SL2.Text:=StringReplace(SL2.Text,'[[ALIGNBYTES]]',IntToStr(AlignBytes),[rfReplaceAll]);
  //
  SL2.Text:=StringReplace(SL2.Text,'[[FILESIDXS]]',FilesIdxsStr,[rfReplaceAll]);
  SL2.Text:=StringReplace(SL2.Text,'[[FILESSIZES]]',FilesSizesStr,[rfReplaceAll]);
  //--
  //
  Writeln(Output,'Writing "'+OutFileDataH+'", "'+OutFileIdxsH+'", ...');
  try
    SL.SaveToFile(OutFileDataH);
  except
    on E:Exception do begin
      Writeln(Output,'Error: '+E.Message);
      Exit;
    end;
  end;
  try
    SL2.SaveToFile(OutFileIdxsH);
  except
    on E:Exception do begin
      Writeln(Output,'Error: '+E.Message);
      Exit;
    end;
  end;
  //
  SL.Free();
  SL2.Free();
  Writeln(Output,'DONE');
end.
