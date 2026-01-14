//---------------------------------------------------------------------------
#ifndef outstrdlgH
#define outstrdlgH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <Dialogs.hpp>
#include <Buttons.hpp>
//---------------------------------------------------------------------------
class TOutputStrDialog : public TForm
{
__published:
	TButton *BtnCancel;
	TButton *BtnOk;
	TLabel *Label5;
	TLabel *Label6;
	TComboBox *Stream1;
	TComboBox *Stream2;
	TComboBox *Stream3;
	TButton *BtnStr1;
	TButton *BtnStr2;
	TButton *BtnStr3;
	TLabel *Label10;
	TLabel *Label7;
	TComboBox *Format1;
	TComboBox *Format2;
	TComboBox *Format3;
	TButton *BtnFile1;
	TButton *BtnFile2;
	TButton *BtnFile3;
	TEdit *FilePath1;
	TEdit *FilePath2;
	TEdit *FilePath3;
	TLabel *LabelF1;
	TSaveDialog *SaveDialog;
	TCheckBox *TimeTagC;
	TCheckBox *Stream1C;
	TCheckBox *Stream2C;
	TCheckBox *Stream3C;
	TSpeedButton *BtnKey;
	TLabel *Label1;
	TComboBox *SwapIntv;
	TLabel *Label2;
	void __fastcall BtnOkClick(TObject *Sender);
	void __fastcall FormShow(TObject *Sender);
	void __fastcall BtnStr1Click(TObject *Sender);
	void __fastcall BtnStr2Click(TObject *Sender);
	void __fastcall BtnStr3Click(TObject *Sender);
	void __fastcall Stream1Change(TObject *Sender);
	void __fastcall Stream2Change(TObject *Sender);
	void __fastcall Stream3Change(TObject *Sender);
	void __fastcall BtnFile1Click(TObject *Sender);
	void __fastcall BtnFile2Click(TObject *Sender);
	void __fastcall BtnFile3Click(TObject *Sender);
	void __fastcall Stream1CClick(TObject *Sender);
	void __fastcall Stream2CClick(TObject *Sender);
	void __fastcall Stream3CClick(TObject *Sender);
	void __fastcall BtnKeyClick(TObject *Sender);
private:
	AnsiString __fastcall GetFilePath(AnsiString path);
	AnsiString __fastcall SetFilePath(AnsiString path);
	void __fastcall SerialOpt(int index, int opt);
	void __fastcall TcpOpt(int index, int opt);
	void __fastcall UpdateEnable(void);
public:
	int StreamC[RTKSVRNSOL],Stream[RTKSVRNSOL],Format[RTKSVRNSOL],OutTimeTag,OutAppend;
	AnsiString Paths[RTKSVRNSOL][4],SwapInterval;
	AnsiString History[10];
	__fastcall TOutputStrDialog(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TOutputStrDialog *OutputStrDialog;
//---------------------------------------------------------------------------
#endif
