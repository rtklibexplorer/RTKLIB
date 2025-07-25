//---------------------------------------------------------------------------
#include <vcl.h>
#include <ctype.h>
#include <stdio.h>
#pragma hdrstop

#include "console.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"

#define MAXLEN		256
#define MAXLINE		2048
#define TOPMARGIN	2
#define LEFTMARGIN	3

TConsole *Console;
//---------------------------------------------------------------------------
__fastcall TConsole::TConsole(TComponent* Owner)
	: TForm(Owner)
{
	ConBuff=new TStringList;
	ConBuff->Add("");
	DoubleBuffered=true;
	Stop=0;
	ScrollPos=0;
}
//---------------------------------------------------------------------------
void __fastcall TConsole::ConsolePaint(TObject *Sender)
{
	TCanvas *c=Console->Canvas;
	TSize off=c->TextExtent(" ");
	int n,m,p,y=TOPMARGIN;
	
	c->Brush->Style=bsSolid;
	c->Brush->Color=clWhite;
	c->FillRect(Console->ClientRect);
	
	n=ConBuff->Count; if (ConBuff->Strings[n-1]=="") n--;
	m=(Console->Height-TOPMARGIN*2)/off.cy;
	p=m>=n?0:n-m-ScrollPos;
	
	for (int i=p<0?0:p;i<ConBuff->Count;i++,y+=off.cy) {
		if (y+off.cy>Console->Height-TOPMARGIN) break;
		c->Font->Color=i<n-1?clGray:clBlack;
		c->TextOut(LEFTMARGIN,y,ConBuff->Strings[i]);
	}
	Scroll->Max=n<=m?m-1:n-m;
	Scroll->Position=Scroll->Max-ScrollPos;
}
//---------------------------------------------------------------------------
void __fastcall TConsole::ScrollChange(TObject *Sender)
{
	ScrollPos=Scroll->Max-Scroll->Position;
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TConsole::FormResize(TObject *Sender)
{
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TConsole::BtnCloseClick(TObject *Sender)
{
	Close();
}
//---------------------------------------------------------------------------
void __fastcall TConsole::BtnAscClick(TObject *Sender)
{
	if (ConBuff->Strings[ConBuff->Count-1]!="") ConBuff->Add("");
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TConsole::BtnHexClick(TObject *Sender)
{
	if (ConBuff->Strings[ConBuff->Count-1]!="") ConBuff->Add("");
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TConsole::BtnClearClick(TObject *Sender)
{
	ConBuff->Clear();
	ConBuff->Add("");
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TConsole::BtnStopClick(TObject *Sender)
{
	Stop=!Stop;
	BtnStop->Down=Stop;
}
//---------------------------------------------------------------------------
void __fastcall TConsole::BtnDownClick(TObject *Sender)
{
	ScrollPos=0;
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TConsole::AddMsg(uint8_t *msg, int n)
{
	AnsiString str;
	char buff[MAXLEN+16],*p=buff,c;
	int mode=BtnAsc->Down;
	
	if (n<=0||Stop) return;
	
	str=ConBuff->Strings[ConBuff->Count-1];
	p+=sprintf(p,"%s",str.c_str());
	
	for (int i=0;i<n;i++) {
        int add = 0;
		if (mode) {
            if (msg[i] == '\r') continue;
            if (msg[i] == '\n') add = 1;
            else {
              p+=sprintf(p,"%c",isprint(msg[i])?msg[i]:'.');
              if (p-buff >= MAXLEN) add = 1;
            }
		}
		else {
            if (strlen(buff) % 17 == 16) strcat(p++, " ");
			p+=sprintf(p,"%02X",msg[i]);
			if (p-buff>=67) add = 1;
		}
		
		if (add) {
			ConBuff->Strings[ConBuff->Count-1]=buff;
			ConBuff->Add("");
			*(p=buff)=0;
			if (ConBuff->Count>=MAXLINE) ConBuff->Delete(0);
		}
	}
	ConBuff->Strings[ConBuff->Count-1]=buff;
	Console->Invalidate();
}
//---------------------------------------------------------------------------

