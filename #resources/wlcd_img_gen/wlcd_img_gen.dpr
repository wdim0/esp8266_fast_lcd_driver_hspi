program wlcd_img_gen;

{$APPTYPE CONSOLE}

{
 * Created by Martin Winkelhofer 03/2016
 * W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0
 *  _      ____   ________      _
 * | | /| / / /  / ___/ _ \    (_)_ _  ___ _    ___ ____ ___
 * | |/ |/ / /__/ /__/ // /   / /  ' \/ _ `/   / _ `/ -_) _ \
 * |__/|__/____/\___/____/   /_/_/_/_/\_, /    \_, /\__/_//_/
 *                                   /___/    /___/
 * Generator of binary files that are understood by WLCD system as compressed images.
 * (see wlcd.h / wlcd.c)
 * v1.2 (03/2016)
 *
 * This file is part of WLCD - W-Dimension's LCD driver for ESP8266.
 *
 * WLCD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WLCD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WLCD. If not, see <http://www.gnu.org/licenses/>.
}

uses
  SysUtils, Classes;

const
  APP_NAME = 'wlcd_img_gen v1.2';

var
  Params: string;
  ForceNoRLE: boolean = false;
  ForceRLE: boolean = false;
  FIn, FOut: integer;
  BMPHdr: array[0..33] of byte; //only part of header (all info we need)
  BMPWidth: LongInt; //signed 32-bit
  BMPHeight: LongInt; //signed 32-bit
  BMPBPP: Word; //unsigned 16-bit
  CompS: string;
  BMPComp: LongWord; //unsigned 32-bit
  BMPDataOffs: LongWord; //unsigned 32-bit
  BMPDataIn: array of byte;
  Row, Col, RowStartOffs, Offs: LongInt; //signed 32-bit
  PxR5G6B5, LastPxR5G6B5: word;
  RLECtr: byte; //we have max 255 pixels in one RLE chunk - this is wanted (for images with many different pixels provides better compression)
  CompSize: integer;
  Val16: word;
  Val8: byte;
  RowSize: word;

function GetR5G6B5FromR8G8B8(R,G,B: byte): word;
begin
  Result:=R shr 3;
  Result:=Result shl 6;
  Result:=Result or (G shr 2);
  Result:=Result shl 5;
  Result:=Result or (B shr 3);
end;

begin
  Writeln(Output,APP_NAME+', created by Martin Winkelhofer 03/2016');
  Writeln(Output,'W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0');
  if(ParamCount<2) then begin
    Writeln(Output,'----------------');
    Writeln(Output,'Usage:');
    Writeln(Output,'wlcd_img_gen.exe <inputfile> <outputfile> (<params>)');
    Writeln(Output,'params: -u force uncompressed WLCD image (no RLE)');
    Writeln(Output,'        -c force compressed WLCD image (RLE)');
    Exit;
  end;
  //
  if(ParamCount>2) then begin
    Params:=ParamStr(3);
    if((Length(Params)>1)and(Params[1]='-')) then begin
      Params:=Copy(Params,2,Length(Params)); //get rid of leading '-'
      ForceNoRLE:=(Pos('u',Params)<>0);
      ForceRLE:=(Pos('c',Params)<>0);
    end;
  end;
  //
  try
    //
    //open BMP file, read BMP header
    FIn:=FileOpen(ParamStr(1), fmOpenRead);
    FileRead(FIn, BMPHdr, 34);
    //
    //check if it's BMP
    if((BMPHdr[0]<>$42)or(BMPHdr[1]<>$4D)) then raise Exception.Create('input file is not BMP file');
    //
    //get BMPWidth, BMPHeight, BMPDataOffs
    BMPDataOffs:=BMPHdr[13];
    BMPDataOffs:=BMPDataOffs shl 8;
    BMPDataOffs:=BMPDataOffs or BMPHdr[12];
    BMPDataOffs:=BMPDataOffs shl 8;
    BMPDataOffs:=BMPDataOffs or BMPHdr[11];
    BMPDataOffs:=BMPDataOffs shl 8;
    BMPDataOffs:=BMPDataOffs or BMPHdr[10];
    //
    BMPWidth:=BMPHdr[21];
    BMPWidth:=BMPWidth shl 8;
    BMPWidth:=BMPWidth or BMPHdr[20];
    BMPWidth:=BMPWidth shl 8;
    BMPWidth:=BMPWidth or BMPHdr[19];
    BMPWidth:=BMPWidth shl 8;
    BMPWidth:=BMPWidth or BMPHdr[18];
    //
    BMPHeight:=BMPHdr[25];
    BMPHeight:=BMPHeight shl 8;
    BMPHeight:=BMPHeight or BMPHdr[24];
    BMPHeight:=BMPHeight shl 8;
    BMPHeight:=BMPHeight or BMPHdr[23];
    BMPHeight:=BMPHeight shl 8;
    BMPHeight:=BMPHeight or BMPHdr[22];
    //
    BMPBPP:=BMPHdr[29];
    BMPBPP:=BMPBPP shl 8;
    BMPBPP:=BMPBPP or BMPHdr[28];
    //
    BMPComp:=BMPHdr[33];
    BMPComp:=BMPComp shl 8;
    BMPComp:=BMPComp or BMPHdr[32];
    BMPComp:=BMPComp shl 8;
    BMPComp:=BMPComp or BMPHdr[31];
    BMPComp:=BMPComp shl 8;
    BMPComp:=BMPComp or BMPHdr[30];
    //
    case(BMPComp)of
      0: CompS:='none';
      1: CompS:='RLE 8-bit/pixel';
      2: CompS:='RLE 4-bit/pixel';
      3: CompS:='Huffman 1D';
      4: CompS:='RLE-24';
      5: CompS:='PNG';
      6: CompS:='RGBA bit field masks';
      else CompS:=Format('unknown (ID=%d)', [BMPComp]);
    end;
    //
    Writeln(Output,Format('Bitmap info: %d x %d, %d bpp, compression: %s', [BMPWidth, BMPHeight, BMPBPP, CompS]));
    Writeln(Output,Format('Bitmap data start offset: 0x%0.8X', [BMPDataOffs]));
    //
    if((BMPBPP<>24)and(BMPBPP<>16)) then raise Exception.Create('image must be 24 bpp or 16 bpp');
    //
    if((BMPComp<>0)and(BMPComp<>3)) then raise Exception.Create('supported compressions: none / Huffman 1D'); //yes, it's strange but Photoshop creates file with BMPComp=3 but it's not compressed => we treat it as uncompressed file
    //
    RowSize := (BMPWidth*(BMPBPP div 8));
    if(RowSize mod 4 <> 0) then RowSize := RowSize + (4 - (RowSize mod 4)); //padded to multiples of 4
    Writeln(Output,Format('RowSize = %d B', [RowSize]));
    //
    //read BMP data to buffer
    SetLength(BMPDataIn, BMPHeight*RowSize);
    FileSeek(FIn, BMPDataOffs, 0);
    FileRead(FIn, BMPDataIn[0], BMPHeight*RowSize);
    FileClose(FIn);
    //
    //write output file
    if(FileExists(ParamStr(2))) then DeleteFile(ParamStr(2));
    FOut:=FileCreate(ParamStr(2));
    if(FOut=-1) then raise Exception.Create('Can''t write file '+ParamStr(2));
    //
    if((ForceRLE)or((not ForceRLE)and(not ForceNoRLE))) then begin
      Writeln(Output,'Writing WLCD image - R5G6B5 RLE compressed data to output file ...');
      //
      Val16:=BMPWidth;
      FileWrite(FOut, Val16, 2);
      Val16:=BMPHeight or $8000; //bit15 is flag "RLE compression" - set
      FileWrite(FOut, Val16, 2);
      //
      RLECtr:=0;
      CompSize:=0;
      for Row:=BMPHeight-1 downto 0 do begin //BMP is upside down. Who came with this? :/
        RowStartOffs := Row*RowSize;
        for Col:=0 to BMPWidth-1 do begin
          Offs := RowStartOffs+(Col*(BMPBPP div 8));
          //
          if(BMPBPP=16) then begin //16 bpp
            PxR5G6B5:=BMPDataIn[Offs+1];           //swap bytes to get the right order (word 0xABCD is stored as 0xCD, 0xAB - low byte first)
            PxR5G6B5:=PxR5G6B5 shl 8;              // ...
            PxR5G6B5:=PxR5G6B5 or BMPDataIn[Offs]; // ...
          end
          else begin //24 bpp
            PxR5G6B5:=GetR5G6B5FromR8G8B8(BMPDataIn[Offs+2], BMPDataIn[Offs+1], BMPDataIn[Offs]);
          end;
          //
          if( (RLECtr=0) or ((LastPxR5G6B5=PxR5G6B5)and(RLECtr<255)) ) then Inc(RLECtr) //if first pixel OR last pixel is the same as this one and RLE counter still not at its max (255)
          else begin
            //
            FileWrite(FOut, RLECtr, 1);
            Val8:=LastPxR5G6B5 shr 8;
            FileWrite(FOut, Val8, 1);
            Val8:=LastPxR5G6B5;
            FileWrite(FOut, Val8, 1);
            CompSize:=CompSize+3;
            //
            RLECtr:=1;
          end;
          LastPxR5G6B5:=PxR5G6B5;
        end;
      end;
      if(RLECtr>0) then begin
        FileWrite(FOut, RLECtr, 1);
        Val8:=LastPxR5G6B5 shr 8;
        FileWrite(FOut, Val8, 1);
        Val8:=LastPxR5G6B5;
        FileWrite(FOut, Val8, 1);
        CompSize:=CompSize+3;
      end;
      //
      Writeln(Output,Format('Compression: %0.1f%% of original (%d B raw / %d B compressed)', [(CompSize/(BMPWidth*BMPHeight*2)) * 100, BMPWidth*BMPHeight*2, CompSize]));
      if((not ForceRLE)and(not ForceNoRLE)and(CompSize>=(BMPWidth*BMPHeight*2))) then begin
        Writeln(Output,'=> uncompressed WLCD image is smaller, forcing uncompressed version');
        FileClose(FOut);
        if(FileExists(ParamStr(2))) then DeleteFile(ParamStr(2));
        FOut:=FileCreate(ParamStr(2));
        ForceNoRLE:=true;
      end;
    end;
    //
    if(ForceNoRLE) then begin
      Writeln(Output,'Writing WLCD image - R5G6B5 uncompressed data to output file ...');
      //
      Val16:=BMPWidth;
      FileWrite(FOut, Val16, 2);
      Val16:=BMPHeight and $7FFF; //bit15 is flag "RLE compression" - clear
      FileWrite(FOut, Val16, 2);
      //
      for Row:=BMPHeight-1 downto 0 do begin //BMP is upside down. Who came with this? :/
        RowStartOffs := Row*RowSize;
        for Col:=0 to BMPWidth-1 do begin
          Offs := RowStartOffs+(Col*(BMPBPP div 8));
          //
          if(BMPBPP=16) then begin //16 bpp
            PxR5G6B5:=BMPDataIn[Offs+1];           //swap bytes to get the right order (word 0xABCD is stored as 0xCD, 0xAB - low byte first)
            PxR5G6B5:=PxR5G6B5 shl 8;              // ...
            PxR5G6B5:=PxR5G6B5 or BMPDataIn[Offs]; // ...
          end
          else begin //24 bpp
            PxR5G6B5:=GetR5G6B5FromR8G8B8(BMPDataIn[Offs+2], BMPDataIn[Offs+1], BMPDataIn[Offs]);
          end;
          //
          Val8:=PxR5G6B5 shr 8;
          FileWrite(FOut, Val8, 1);
          Val8:=PxR5G6B5;
          FileWrite(FOut, Val8, 1);
        end;
      end;
    end;
    //
    FileClose(FOut);
    //
    //clean-up
    SetLength(BMPDataIn,0);
    Writeln(Output,'DONE');
  except
    on E:Exception do begin
      Writeln(Output, 'Error: '+E.Message);
      try FileClose(FIn); except end;
      try FileClose(FOut); except end;
    end;
  end;
end.
