program wlcd_img_gen;

{$APPTYPE CONSOLE}

{
 * Created by Martin Winkelhofer 03,12/2016
 * W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0
 *  _      ____   ________      _
 * | | /| / / /  / ___/ _ \    (_)_ _  ___ _    ___ ____ ___
 * | |/ |/ / /__/ /__/ // /   / /  ' \/ _ `/   / _ `/ -_) _ \
 * |__/|__/____/\___/____/   /_/_/_/_/\_, /    \_, /\__/_//_/
 *                                   /___/    /___/
 * Generator of WLCD images from BMP files.
 * (see wlcd.h / wlcd.c)
 * v1.50 (12/2016)
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
  APP_NAME = 'wlcd_img_gen v1.50';

var
  Params: string;
  ForceNoRLE: boolean = false;
  ForceRLE: boolean = false;
  FIn: integer = -1;
  FOut: integer = -1;
  BMPHdr: array[0..33] of byte; //only part of header (all info we need)
  BMPWidth: LongInt; //signed 32-bit
  BMPHeight: LongInt; //signed 32-bit
  BMPBPP: word; //unsigned 16-bit
  CompS: string;
  BMPComp: LongWord; //unsigned 32-bit
  BMPDataOffs: LongWord; //unsigned 32-bit
  BMPDataIn: array of byte;
  Row, Col, RowStartOffs, Offs: LongInt; //signed 32-bit
  PxR8G8B8: LongWord; //unsigned 32-bit
  LastPxR8G8B8: LongWord = 0; //unsigned 32-bit
  PxR5G6B5: word;
  LastPxR5G6B5: word = 0;
  RLECtr: byte; //we have max 255 pixels in one RLE chunk - this is wanted (for images with many different pixels provides better compression)
  OutputRawSz, CmprsSz: integer;
  Val16: word;
  Val8: byte;
  RowSize: word;
  BPPMode: byte = 0; //0 = WLCD_IMG_16BPP = --16bpp; 1 = WLCD_IMG_24BPP = --24bpp

function GetR8G8B8From_R5G6B5H_R5G6B5L(R5G6B5H, R5G6B5L: byte): LongWord; //unsigned 32-bit
begin
  Result:=R5G6B5H and $F8; //mask R5G6B5H[7:3] => get R5[7:3]
  Result:=Result shl 8;
  Result:=Result or ((R5G6B5H and $07) shl 5) or ((R5G6B5L and $E0) shr 3); //mask R5G6B5H[2:0], shl by 5 = G6[7:5] | mask R5G6B5L[7:5], shr by 3 = G6[4:2] => get G6[7:2]
  Result:=Result shl 8;
  Result:=Result or ((R5G6B5L and $1F) shl 3); //mask R5G6B5L[4:0], shl 3 => get B5[7:3]
end;

function GetR8G8B8From_R_G_B(R, G, B: byte): LongWord; //unsigned 32-bit
begin
  Result:=R;
  Result:=Result shl 8;
  Result:=Result or G;
  Result:=Result shl 8;
  Result:=Result or B;
end;

function GetR5G6B5From_R8G8B8(R8G8B8: LongWord): word; //unsigned 16-bit
var
  R,G,B: byte;
begin
  R:=R8G8B8 shr 16;
  G:=R8G8B8 shr 8;
  B:=R8G8B8;
  //
  Result:=R shr 3;
  Result:=Result shl 6;
  Result:=Result or (G shr 2);
  Result:=Result shl 5;
  Result:=Result or (B shr 3);
end;

begin
  Writeln(Output,APP_NAME+', created by Martin Winkelhofer 12/2016');
  Writeln(Output,'W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0');
  Writeln(Output,'--------');
  if(ParamCount<3) then begin
    Writeln(Output,'Usage:');
    Writeln(Output,'wlcd_img_gen.exe <inputfile> <outputfile> <bpp> [<params>]');
    Writeln(Output,'bpp:    --16bpp - (WLCD image with R5G6B5 color depth)');
    Writeln(Output,'        --24bpp - (WLCD image with R8G8B8 color depth)');
    Writeln(Output,'params: -u      - force uncompressed WLCD image (no RLE)');
    Writeln(Output,'        -c      - force compressed WLCD image (RLE)');
    Exit;
  end;
  //
  if(ParamStr(3)='--24bpp') then BPPMode:=1;
  //
  if(ParamCount>3) then begin
    Params:=ParamStr(4);
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
    if(BPPMode=1) then OutputRawSz:=BMPWidth*BMPHeight*3 //24 bpp output WLCD
    else OutputRawSz:=BMPWidth*BMPHeight*2; //16 bpp output WLCD
    //
    RowSize := (BMPWidth*(BMPBPP div 8));
    if(RowSize mod 4 <> 0) then RowSize := RowSize + (4 - (RowSize mod 4)); //padded to multiples of 4
    //
    Writeln(Output,Format('Bitmap info: %d x %d px, %d bpp, compression: %s', [BMPWidth, BMPHeight, BMPBPP, CompS]));
    Writeln(Output,Format('Bitmap data start offset: 0x%0.8X, RowSize: %d B', [BMPDataOffs, RowSize]));
    //
    if((BMPBPP<>24)and(BMPBPP<>16)) then raise Exception.Create('input image must be 24 bpp or 16 bpp bitmap');
    //
    if((BMPComp<>0)and(BMPComp<>3)) then raise Exception.Create('supported compressions: none / Huffman 1D'); //yes, it's strange but Gimp creates file with BMPComp=3 but it's not compressed? => we treat it as uncompressed file
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
    if((BMPBPP=16)and(BPPMode=1)) then Writeln(Output,'WARNING: 24 bpp output from 16 bpp input, consider providing 24 bpp input');
    //
    if((ForceRLE)or((not ForceRLE)and(not ForceNoRLE))) then begin
      //---- generate RLE compressed WLCD image --------------------------------
      if(BPPMode=1) then Writeln(Output,'Writing 24 bpp (R8G8B8) RLE compressed WLCD image data to output file ...') //24 bpp output WLCD
      else Writeln(Output,'Writing 16 bpp (R5G6B5) RLE compressed WLCD image data to output file ...'); //16 bpp output WLCD
      //
      //header
      Val16:=BMPWidth;                          //bit[13:0] image width in pixels
      if(BPPMode=1) then Val16:=Val16 or $4000; //bit[15:14] = 01 (see wlcd_img_bpp_enum)
      //else Val16:=Val16 or $0000;             //bit[15:14] = 00
      FileWrite(FOut, Val16, 2);
      Val16:=BMPHeight;                         //bit[14:0] image height in pixels
      Val16:=Val16 or $8000;                    //bit[15] RLE compression flag
      FileWrite(FOut, Val16, 2);
      //
      //data / RLE words
      RLECtr:=0;
      CmprsSz:=0;
      for Row:=BMPHeight-1 downto 0 do begin //BMP is upside down. Who came with this? :/
        RowStartOffs := Row*RowSize;
        for Col:=0 to BMPWidth-1 do begin //for each pixel
          Offs := RowStartOffs+(Col*(BMPBPP div 8));
          //
          //get PxR8G8B8 in all cases
          if(BMPBPP=16) then PxR8G8B8:=GetR8G8B8From_R5G6B5H_R5G6B5L(BMPDataIn[Offs+1], BMPDataIn[Offs]) //input BMP is 16 bpp
          else PxR8G8B8:=GetR8G8B8From_R_G_B(BMPDataIn[Offs+2], BMPDataIn[Offs+1], BMPDataIn[Offs]); //input BMP is 24 bpp
          //
          //process RLE compression
          if(BPPMode=1) then begin //24 bpp output WLCD
            //
            if( (RLECtr=0) or ((LastPxR8G8B8=PxR8G8B8)and(RLECtr<255)) ) then Inc(RLECtr) //if first pixel OR last pixel is the same as this one and RLE counter still not at its max (255)
            else begin
              //
              FileWrite(FOut, RLECtr, 1);
              Val8:=LastPxR8G8B8 shr 16;
              FileWrite(FOut, Val8, 1);
              Val8:=LastPxR8G8B8 shr 8;
              FileWrite(FOut, Val8, 1);
              Val8:=LastPxR8G8B8;
              FileWrite(FOut, Val8, 1);
              CmprsSz:=CmprsSz+4;
              //
              RLECtr:=1;
            end;
            LastPxR8G8B8:=PxR8G8B8;
            //
          end
          else begin //16 bpp output WLCD
            PxR5G6B5:=GetR5G6B5From_R8G8B8(PxR8G8B8);
            //
            if( (RLECtr=0) or ((LastPxR5G6B5=PxR5G6B5)and(RLECtr<255)) ) then Inc(RLECtr) //if first pixel OR last pixel is the same as this one and RLE counter still not at its max (255)
            else begin
              //
              FileWrite(FOut, RLECtr, 1);
              Val8:=LastPxR5G6B5 shr 8;
              FileWrite(FOut, Val8, 1);
              Val8:=LastPxR5G6B5;
              FileWrite(FOut, Val8, 1);
              CmprsSz:=CmprsSz+3;
              //
              RLECtr:=1;
            end;
            LastPxR5G6B5:=PxR5G6B5;
            //
          end;
          //
        end;
      end;
      if(RLECtr>0) then begin
        if(BPPMode=1) then begin //24 bpp output WLCD
          FileWrite(FOut, RLECtr, 1);
          Val8:=LastPxR8G8B8 shr 16;
          FileWrite(FOut, Val8, 1);
          Val8:=LastPxR8G8B8 shr 8;
          FileWrite(FOut, Val8, 1);
          Val8:=LastPxR8G8B8;
          FileWrite(FOut, Val8, 1);
          CmprsSz:=CmprsSz+4;
        end
        else begin //16 bpp output WLCD
          FileWrite(FOut, RLECtr, 1);
          Val8:=LastPxR5G6B5 shr 8;
          FileWrite(FOut, Val8, 1);
          Val8:=LastPxR5G6B5;
          FileWrite(FOut, Val8, 1);
          CmprsSz:=CmprsSz+3;
        end;
      end;
      //------------------------------------------------------------------------
      //
      Writeln(Output,Format('Compression: %0.1f%% of original (%d B raw / %d B compressed)', [(CmprsSz/OutputRawSz)*100, OutputRawSz, CmprsSz]));
      if((not ForceRLE)and(not ForceNoRLE)and(CmprsSz>=OutputRawSz)) then begin
        Writeln(Output,'=> uncompressed WLCD image is smaller, forcing uncompressed version');
        FileClose(FOut);
        if(FileExists(ParamStr(2))) then DeleteFile(ParamStr(2));
        FOut:=FileCreate(ParamStr(2));
        ForceNoRLE:=true;
      end;
    end;
    //
    if(ForceNoRLE) then begin
      //---- generate uncompressed WLCD image ----------------------------------
      if(BPPMode=1) then Writeln(Output,'Writing 24 bpp (R8G8B8) uncompressed WLCD image data to output file ...') //24 bpp output WLCD
      else Writeln(Output,'Writing 16 bpp (R5G6B5) uncompressed WLCD image data to output file ...'); //16 bpp output WLCD
      //
      //header
      Val16:=BMPWidth;                          //bit[13:0] image width in pixels
      if(BPPMode=1) then Val16:=Val16 or $4000; //bit[15:14] = 01 (see wlcd_img_bpp_enum)
      //else Val16:=Val16 or $0000;             //bit[15:14] = 00
      FileWrite(FOut, Val16, 2);
      Val16:=BMPHeight;                         //bit[14:0] image height in pixels
      //Val16:=Val16 or $0000;                  //bit[15] RLE compression flag
      FileWrite(FOut, Val16, 2);
      //
      //data / raw pixels
      for Row:=BMPHeight-1 downto 0 do begin //BMP is upside down. Who came with this? :/
        RowStartOffs := Row*RowSize;
        for Col:=0 to BMPWidth-1 do begin
          Offs := RowStartOffs+(Col*(BMPBPP div 8));
          //
          //get PxR8G8B8 in all cases
          if(BMPBPP=16) then PxR8G8B8:=GetR8G8B8From_R5G6B5H_R5G6B5L(BMPDataIn[Offs+1], BMPDataIn[Offs]) //input BMP is 16 bpp
          else PxR8G8B8:=GetR8G8B8From_R_G_B(BMPDataIn[Offs+2], BMPDataIn[Offs+1], BMPDataIn[Offs]); //input BMP is 24 bpp
          //
          //process raw pixels
          if(BPPMode=1) then begin //24 bpp output WLCD
            Val8:=PxR8G8B8 shr 16;
            FileWrite(FOut, Val8, 1);
            Val8:=PxR8G8B8 shr 8;
            FileWrite(FOut, Val8, 1);
            Val8:=PxR8G8B8;
            FileWrite(FOut, Val8, 1);
          end
          else begin //16 bpp output WLCD
            PxR5G6B5:=GetR5G6B5From_R8G8B8(PxR8G8B8);
            //
            Val8:=PxR5G6B5 shr 8;
            FileWrite(FOut, Val8, 1);
            Val8:=PxR5G6B5;
            FileWrite(FOut, Val8, 1);
          end;
          //
        end;
      end;
      //------------------------------------------------------------------------
    end;
    //
    FileClose(FOut);
    //
    //clean-up
    SetLength(BMPDataIn,0);
    Writeln(Output,'--------');
    Writeln(Output,'DONE');
  except
    on E:Exception do begin
      Writeln(Output, 'Error: '+E.Message);
      try FileClose(FIn); except end;
      try FileClose(FOut); except end;
    end;
  end;
end.
