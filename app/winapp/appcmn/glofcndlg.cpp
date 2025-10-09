//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "rtklib.h"
#include "glofcndlg.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TGloFcnDialog *GloFcnDialog;
//---------------------------------------------------------------------------
__fastcall TGloFcnDialog::TGloFcnDialog(TComponent* Owner)
	: TForm(Owner)
{
	EnaGloFcn=0;

	for (int i=0;i<27;i++) {
		GloFcn[i]=0;
	}
}
//---------------------------------------------------------------------------
void __fastcall TGloFcnDialog::FormShow(TObject *Sender)
{
	AnsiString text;

	EnaFcn->Checked=EnaGloFcn;

	for (int i=0;i<27;i++) {
		if (GloFcn[i]) GetFcn(i+1)->Text=text.sprintf("%d",GloFcn[i]-8);
		else GetFcn(i+1)->Text="";
	}
    UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TGloFcnDialog::BtnOkClick(TObject *Sender)
{
	AnsiString text;
	int no;
	
	EnaGloFcn=EnaFcn->Checked;
	
	for (int i=0;i<27;i++) {
		text=GetFcn(i+1)->Text;
		if (sscanf(text.c_str(),"%d",&no)==1&&no>=-7&&no<=6) {
			GloFcn[i]=no+8;
		}
		else GloFcn[i]=0; // GLONASS FCN+8 (0:none)
	}
}
//---------------------------------------------------------------------------
void __fastcall TGloFcnDialog::BtnReadClick(TObject *Sender)
{
        if (!OpenDialog->Execute()) return;
	
        nav_t *nav = static_cast<nav_t *>(calloc(1, sizeof(nav_t)));
        if (nav == NULL) {
          trace(1, "TGloFcnDialog::BtnReadClick nav alloc failed\n");
          return;
        }

        AnsiString file=OpenDialog->FileName;
    
	if (!readrnx(file.c_str(),0,"",NULL,nav,NULL)) {
          free(nav);
          return;
        }
	
	for (int i=0;i<nav->ng;i++) {
          int prn;
          if (satsys(nav->geph[i].sat,&prn)!=SYS_GLO) continue;
          AnsiString text;
          GetFcn(prn)->Text=text.sprintf("%d",nav->geph[i].frq);
	}
	freenav(nav,0xFF);
        free(nav);
}
//---------------------------------------------------------------------------
void __fastcall TGloFcnDialog::BtnClearClick(TObject *Sender)
{
	for (int i=0;i<27;i++) {
		GetFcn(i+1)->Text="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TGloFcnDialog::EnaFcnClick(TObject *Sender)
{
    UpdateEnable();
}
//---------------------------------------------------------------------------
void __fastcall TGloFcnDialog::UpdateEnable(void)
{
	BtnClear->Enabled=EnaFcn->Checked;
	BtnRead ->Enabled=EnaFcn->Checked;
	Label21 ->Enabled=EnaFcn->Checked;
	Label22 ->Enabled=EnaFcn->Checked;
	
	for (int i=0;i<27;i++) {
		GetFcn(i+1)->Enabled=EnaFcn->Checked;
	}
}
//---------------------------------------------------------------------------
TEdit * __fastcall TGloFcnDialog::GetFcn(int prn)
{
	TEdit *fcn[]={
		Fcn01,Fcn02,Fcn03,Fcn04,Fcn05,Fcn06,Fcn07,Fcn08,Fcn09,Fcn10,
		Fcn11,Fcn12,Fcn13,Fcn14,Fcn15,Fcn16,Fcn17,Fcn18,Fcn19,Fcn20,
		Fcn21,Fcn22,Fcn23,Fcn24,Fcn25,Fcn26,Fcn27
	};
	return fcn[prn-1];
}
//---------------------------------------------------------------------------

