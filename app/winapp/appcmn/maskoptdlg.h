//---------------------------------------------------------------------------
#ifndef maskoptdlgH
#define maskoptdlgH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <Vcl.Controls.hpp>
#include <Vcl.StdCtrls.hpp>
#include <Vcl.Forms.hpp>
#include <Vcl.ExtCtrls.hpp>
#include "rtklib.h"

//---------------------------------------------------------------------------
class TMaskOptDialog : public TForm
{
__published:
	TButton *BtnOk;
	TCheckBox *MaskEna1;
	TLabel *Label3;
	TPanel *Panel1;
	TLabel *Label1;
	TEdit *Mask_1_1;
	TEdit *Mask_1_2;
	TEdit *Mask_1_3;
	TLabel *Label6;
	TEdit *Mask_1_4;
	TEdit *Mask_1_5;
	TEdit *Mask_1_6;
	TEdit *Mask_1_7;
	TEdit *Mask_1_8;
	TEdit *Mask_1_9;
	TLabel *Label4;
	TLabel *Label5;
	TLabel *Label7;
	TLabel *Label8;
	TLabel *Label9;
	TLabel *Label10;
	TLabel *Label11;
	TLabel *Label12;
	TPanel *Panel2;
	TLabel *Label13;
	TEdit *Mask_2_1;
	TEdit *Mask_2_2;
	TEdit *Mask_2_3;
	TEdit *Mask_2_4;
	TEdit *Mask_2_5;
	TEdit *Mask_2_6;
	TEdit *Mask_2_7;
	TEdit *Mask_2_8;
	TEdit *Mask_2_9;
	TPanel *Panel3;
	TLabel *Label14;
	TEdit *Mask_3_1;
	TEdit *Mask_3_2;
	TEdit *Mask_3_3;
	TEdit *Mask_3_4;
	TEdit *Mask_3_5;
	TEdit *Mask_3_6;
	TEdit *Mask_3_7;
	TEdit *Mask_3_8;
	TEdit *Mask_3_9;
	TCheckBox *MaskEna2;
    TPanel *Panel4;
    TLabel *Label15;
    TEdit *Mask_4_1;
    TEdit *Mask_4_2;
    TEdit *Mask_4_3;
    TEdit *Mask_4_4;
    TEdit *Mask_4_5;
    TEdit *Mask_4_6;
    TEdit *Mask_4_7;
    TEdit *Mask_4_8;
    TEdit *Mask_4_9;
    TPanel *Panel5;
    TLabel *Label16;
    TEdit *Mask_5_1;
    TEdit *Mask_5_2;
    TEdit *Mask_5_3;
    TEdit *Mask_5_4;
    TEdit *Mask_5_5;
    TEdit *Mask_5_6;
    TEdit *Mask_5_7;
    TEdit *Mask_5_8;
    TEdit *Mask_5_9;
    TPanel *Panel6;
    TLabel *Label17;
    TEdit *Mask_6_1;
    TEdit *Mask_6_2;
    TEdit *Mask_6_3;
    TEdit *Mask_6_4;
    TEdit *Mask_6_5;
    TEdit *Mask_6_6;
    TEdit *Mask_6_7;
    TEdit *Mask_6_8;
    TEdit *Mask_6_9;
	void __fastcall FormShow(TObject *Sender);
	void __fastcall BtnOkClick(TObject *Sender);
	void __fastcall MaskEna1Click(TObject *Sender);
private:
	void __fastcall UpdateEnable(void);
public:
	snrmask_t Mask;
	__fastcall TMaskOptDialog(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TMaskOptDialog *MaskOptDialog;
//---------------------------------------------------------------------------
#endif
