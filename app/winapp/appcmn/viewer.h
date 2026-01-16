//---------------------------------------------------------------------------
#ifndef viewerH
#define viewerH
//---------------------------------------------------------------------------
#include <Buttons.hpp>
#include <Classes.hpp>
#include <ComCtrls.hpp>
#include <Controls.hpp>
#include <Dialogs.hpp>
#include <ExtCtrls.hpp>
#include <Forms.hpp>
#include <StdCtrls.hpp>

#define MAXLINE 1000000

//---------------------------------------------------------------------------
class TTextViewer : public TForm {
  __published : TPanel *Panel1;
  TButton *BtnClose;
  TPanel *Panel2;
  TButton *BtnRead;
  TOpenDialog *OpenDialog;
  TButton *BtnOpt;
  TSpeedButton *BtnReload;
  TRichEdit *Text;
  TSaveDialog *SaveDialog;
  TEdit *FindStr;
  TButton *BtnFind;
  void __fastcall BtnCloseClick(TObject *Sender);
  void __fastcall BtnReadClick(TObject *Sender);
  void __fastcall BtnOptClick(TObject *Sender);
  void __fastcall FormShow(TObject *Sender);
  void __fastcall BtnReloadClick(TObject *Sender);
  void __fastcall BtnFindClick(TObject *Sender);
  void __fastcall FindStrKeyPress(TObject *Sender, char &Key);

 private:
  AnsiString File;

  void __fastcall UpdateText(void);

 public:
  int Option;
  static TColor Color1, Color2;
  static TFont *FontD;
  __fastcall TTextViewer(TComponent *Owner);
  void __fastcall Read(AnsiString file);
  void __fastcall Save(AnsiString file);
};
//---------------------------------------------------------------------------
extern PACKAGE TTextViewer *TextViewer;
//---------------------------------------------------------------------------
#endif
