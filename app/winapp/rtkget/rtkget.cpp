//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop
//---------------------------------------------------------------------------

















USEFORM("staoptdlg.cpp", StaListDialog);
USEFORM("getoptdlg.cpp", DownOptDialog);
USEFORM("..\appcmn\timedlg.cpp", TimeDialog);
USEFORM("..\appcmn\keydlg.cpp", KeyDialog);
USEFORM("..\appcmn\aboutdlg.cpp", AboutDialog);
USEFORM("getmain.cpp", MainForm);
USEFORM("..\appcmn\vieweropt.cpp", ViewerOptDialog);
USEFORM("..\appcmn\viewer.cpp", TextViewer);
//---------------------------------------------------------------------------
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
	try
	{
		Application->Initialize();
		Application->CreateForm(__classid(TMainForm), &MainForm);
         Application->CreateForm(__classid(TDownOptDialog), &DownOptDialog);
         Application->CreateForm(__classid(TTimeDialog), &TimeDialog);
         Application->CreateForm(__classid(TKeyDialog), &KeyDialog);
         Application->CreateForm(__classid(TAboutDialog), &AboutDialog);
         Application->CreateForm(__classid(TTextViewer), &TextViewer);
         Application->CreateForm(__classid(TViewerOptDialog), &ViewerOptDialog);
         Application->CreateForm(__classid(TStaListDialog), &StaListDialog);
         Application->Run();
	}
	catch (Exception &exception)
	{
		Application->ShowException(&exception);
	}
	catch (...)
	{
		try
		{
			throw Exception("");
		}
		catch (Exception &exception)
		{
			Application->ShowException(&exception);
		}
	}
	return 0;
}
//---------------------------------------------------------------------------
