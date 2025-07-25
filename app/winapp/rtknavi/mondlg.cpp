//---------------------------------------------------------------------------
#include <vcl.h>
#pragma hdrstop

#include "rtklib.h"
#include "mondlg.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"

#define SQRT(x)     ((x)<0.0||(x)!=(x)?0.0:sqrt(x))
#define TOPMARGIN	2
#define LEFTMARGIN	3
#define MAXLINE		2048
#define MAXLEN		256
#define TYPE_WIDTH  90

#define NMONITEM	17

//---------------------------------------------------------------------------

extern rtksvr_t rtksvr;		// rtk server struct
extern stream_t monistr;	// monitor stream

static const int sys_tbl[]={
	SYS_ALL,SYS_GPS,SYS_GLO,SYS_GAL,SYS_QZS,SYS_CMP,SYS_IRN,SYS_SBS
};

//---------------------------------------------------------------------------
__fastcall TMonitorDialog::TMonitorDialog(TComponent* Owner)
	: TForm(Owner)
{
	int i;
	
	ScrollPos=0;
	ObsMode=0;
	ConFmt=-1;
	ConBuff=new TStringList;
	ConBuff->Add("");
	DoubleBuffered=true;
	Str1=Str2=0;
	
	for (i=0;i<=MAXRCVFMT;i++) {
		SelFmt->Items->Add(formatstrs[i]);
	}
	init_rtcm(&rtcm);
	init_raw(&raw,-1);
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::FormShow(TObject *Sender)
{
	TypeF=Type->ItemIndex;
	Label->Caption="";
	ClearTable();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::FormClose(TObject *Sender, TCloseAction &Action)
{
	free_rtcm(&rtcm);
	free_raw(&raw);
	Release();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::BtnCloseClick(TObject *Sender)
{
	Close();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::TypeChange(TObject *Sender)
{
	int index;
	
	TypeF=Type->ItemIndex;
	index=TypeF-NMONITEM;
	
	if (0<=index) {
		rtksvrlock(&rtksvr);
		if      (index<2) rtksvr.npb[index  ]=0;
		else if (index<4) rtksvr.nsb[index-2]=0;
		else              rtksvr.rtk.neb=0;
		rtksvrunlock(&rtksvr);
	}
	ClearTable();
	Label->Caption="";
	ConBuff->Clear();
	ConBuff->Add("");
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SelFmtChange(TObject *Sender)
{
	AddConsole((uint8_t *)"\n",1,1);
    
    if (ConFmt>=3&&ConFmt<17) {
        free_raw(&raw);
    }
    ConFmt=SelFmt->ItemIndex;
    
    if (ConFmt>=3&&ConFmt<17) {
        init_raw(&raw,ConFmt-2);
    }
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SelStrChange(TObject *Sender)
{
	Str1=SelStr->ItemIndex;
	ConBuff->Clear();
	ConBuff->Add("");
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SelStr2Change(TObject *Sender)
{
	Str2=SelStr2->ItemIndex;
	ConBuff->Clear();
	ConBuff->Add("");
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::Timer1Timer(TObject *Sender)
{
	if (!Visible) return;
	switch (TypeF) {
		case  0: ShowRtk();        break;
		case  1: ShowObs();        break;
		case  2: ShowNav();        break;
		case  3: ShowIonUtc();     break;
		case  4: ShowStr();        break;
		case  5: ShowSat();        break;
		case  6: ShowEst();        break;
		case  7: ShowCov();        break;
		case  8: ShowSbsMsg();     break;
		case  9: ShowSbsLong();    break;
		case 10: ShowSbsIono();    break;
		case 11: ShowSbsFast();    break;
		case 12: ShowRtcm();       break;
		case 13: ShowRtcmDgps();   break;
		case 14: ShowRtcmSsr();    break;
		case 15: ShowRefSta();     break;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ClearTable(void)
{
	int console=0;
	
	FontScale=Type->Width*96/TYPE_WIDTH;
	
	switch (TypeF) {
		case  0: SetRtk();      break;
		case  1: SetObs();      break;
		case  2: ;              break;
		case  3: SetIonUtc();   break;
		case  4: SetStr();      break;
		case  5: SetSat();      break;
		case  6: SetEst();      break;
		case  7: SetCov();      break;
		case  8: SetSbsMsg();   break;
		case  9: SetSbsLong();  break;
		case 10: SetSbsIono();  break;
		case 11: SetSbsFast();  break;
		case 12: SetRtcm();     break;
		case 13: SetRtcmDgps(); break;
		case 14: SetRtcmSsr();  break;
		case 15: SetRefSta();   break;
		default: console=1;     break;
	}
	Console ->Visible=console;
	Scroll  ->Visible=console;
	Panel2  ->Visible=console;
	Tbl     ->Visible=!console;
	SelObs  ->Visible=TypeF==1;
	SelSys  ->Visible=TypeF==1||TypeF==5;
	SelSys2 ->Visible=TypeF==2||TypeF==14;
	SelSat  ->Visible=TypeF==2||TypeF==5;
	SelStr  ->Visible=TypeF==12||TypeF==14||TypeF==15||TypeF==16;
	SelStr2 ->Visible=TypeF==17;
	SelFmt  ->Visible=TypeF==16;
	SelEph  ->Visible=TypeF==2;
	SelStr  ->Left=(TypeF==14)?SelSys->Left+SelSys->Width+1:Type->Left+Type->Width+1;
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::Timer2Timer(TObject *Sender)
{
	uint8_t *msg;
	char buff[256];
	int i,n,len;
	
	if (TypeF<16) return;
	
	rtksvrlock(&rtksvr);
	
	if (TypeF==16) { // input buffer
		len=rtksvr.npb[Str1];
		if (len>0&&(msg=(uint8_t *)malloc(len))) {
			memcpy(msg,rtksvr.pbuf[Str1],len);
			rtksvr.npb[Str1]=0;
		}
	}
	else if (TypeF==17) { // solution buffer
		len=rtksvr.nsb[Str2];
		if (len>0&&(msg=(uint8_t *)malloc(len))) {
			memcpy(msg,rtksvr.sbuf[Str2],len);
			rtksvr.nsb[Str2]=0;
		}
	}
	else { // error message buffer
		len=rtksvr.rtk.neb;
		if (len>0&&(msg=(uint8_t *)malloc(len))) {
			memcpy(msg,rtksvr.rtk.errbuf,len);
			rtksvr.rtk.neb=0;
		}
	}
	rtksvrunlock(&rtksvr);
	
	if (len<=0||!msg) return;
	
	rtcm.outtype=raw.outtype=1;
	
	if (TypeF>=17) {
		AddConsole(msg,len,1);
	}
	else if (ConFmt<2) {
		AddConsole(msg,len,ConFmt);
	}
	else if (ConFmt==2) {
		for (i=0;i<len;i++) {
			input_rtcm2(&rtcm,msg[i]);
			if (rtcm.msgtype[0]) {
				n=sprintf(buff,"%s\n",rtcm.msgtype);
				AddConsole((uint8_t *)buff,n,1);
				rtcm.msgtype[0]='\0';
			}
	    }
	}
	else if (ConFmt==3) {
		for (i=0;i<len;i++) {
			input_rtcm3(&rtcm,msg[i]);
			if (rtcm.msgtype[0]) {
				n=sprintf(buff,"%s\n",rtcm.msgtype);
				AddConsole((uint8_t *)buff,n,1);
				rtcm.msgtype[0]='\0';
			}
	    }
	}
	else if (ConFmt<17) {
		for (i=0;i<len;i++) {
			input_raw(&raw,ConFmt-2,msg[i]);
			if (raw.msgtype[0]) {
				n=sprintf(buff,"%s\n",raw.msgtype);
				AddConsole((uint8_t *)buff,n,1);
				raw.msgtype[0]='\0';
			}
	    }
	}
	free(msg);
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::AddConsole(uint8_t *msg, int len, int mode)
{
	AnsiString ConBuff_Str=ConBuff->Strings[ConBuff->Count-1];
	char buff[MAXLEN+16],*p=buff;
	
	if (BtnPause->Down) return;
	
	p+=sprintf(p,"%s",ConBuff_Str.c_str());
	
	for (int i=0;i<len;i++) {
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
	if (BtnDown->Down) ScrollPos=0;
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ConsolePaint(TObject *Sender)
{
	TCanvas *c=Console->Canvas;
	TSize off=c->TextExtent(" ");
	int n,m,p,y=TOPMARGIN;
	
	c->Brush->Style=bsSolid;
	c->Brush->Color=clWhite;
	c->FillRect(Console->ClientRect);
	
	n=ConBuff->Count;
	if (ConBuff->Strings[n-1]=="") n--;
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
void __fastcall TMonitorDialog::ScrollChange(TObject *Sender)
{
	ScrollPos=Scroll->Max-Scroll->Position;
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::BtnDownClick(TObject *Sender)
{
	if (BtnDown->Down) ScrollPos=0;
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::BtnClearClick(TObject *Sender)
{
	ConBuff->Clear();
	ConBuff->Add("");
	Console->Invalidate();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SelObsChange(TObject *Sender)
{
	ObsMode=SelObs->ItemIndex;
	SetObs();
	ShowObs();
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetRtk(void)
{
	AnsiString label[]={"Parameter","Value"};
	int width[]={220,380};
	
	Tbl->ColCount=2;
	Tbl->RowCount=2;
	for (int i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowRtk(void)
{
	AnsiString s,exsats,navsys="";
	AnsiString svrstate[]={"Stop","Run"};
	AnsiString sol[]={"-","Fix","Float","SBAS","DGPS","Single","PPP",""};
	AnsiString mode[]={"Single","DGPS","Kinematic","Static","Static-Start","Moving-Base",
					   "Fixed","PPP-Kinematic","PPP-Static",""};
	AnsiString freq[]={"-","L1","L1+L2","L1+L2+L3","L1+L2+L3+L4","L1+L2+L3+L4+L5",""};
	double *del,*off,runtime,rt[3]={0},dop[4]={0};
	double azel[MAXSAT*2],pos[3],vel[3],rr[3]={0},enu[3]={0};
	int i,j,k,thread,cycle,state,rtkstat,nsat0,nsat1,prcout,nave;
	int cputime,nb[3]={0},nmsg[3][10]={{0}},ne;
	char tstr[40],*ant,id[8],s1[40]="-",s2[40]="-",s3[40]="-";
	char file[1024]="";
	const char *ionoopt[]={"OFF","Broadcast","SBAS","Dual-Frequency","Estimate STEC","IONEX TEC","QZSS LEX",""};
	const char *tropopt[]={"OFF","Saastamoinen","SBAS","Estimate ZTD","Estimate ZTD+Grad",""};
	const char *ephopt []={"Broadcast","Precise","Broadcast+SBAS","Broadcat+SSR APC","Broadcast+SSR CoM","QZSS LEX",""};
	
        rtk_t *rtk = static_cast<rtk_t *>(malloc(sizeof(rtk_t)));
        if (rtk == NULL) return;

	rtksvrlock(&rtksvr); // lock
	
	*rtk=rtksvr.rtk;
	thread=(int)rtksvr.thread;
	cycle=rtksvr.cycle;
	state=rtksvr.state;
	rtkstat=rtksvr.rtk.sol.stat;
	nsat0=rtksvr.obs[0][0].n;
	nsat1=rtksvr.obs[1][0].n;
	cputime=rtksvr.cputime;
	prcout =rtksvr.prcout;
	nave=rtksvr.nave;
	for (i=0;i<3;i++) nb[i]=rtksvr.nb[i];
	for (i=0;i<3;i++) for (j=0;j<10;j++) {
		nmsg[i][j]=rtksvr.nmsg[i][j];
	}
	if (rtksvr.state) {
		runtime=(double)(tickget()-rtksvr.tick)/1000.0;
		rt[0]=floor(runtime/3600.0); runtime-=rt[0]*3600.0;
		rt[1]=floor(runtime/60.0); rt[2]=runtime-rt[1]*60.0;
	}
	if ((ne=rtksvr.nav.ne)>0) {
		time2str(rtksvr.nav.peph[   0].time,s1,0);
		time2str(rtksvr.nav.peph[ne-1].time,s2,0);
		time2str(rtksvr.ftime[2],s3,0);
	}
	strcpy(file,rtksvr.files[2]);

	rtksvrunlock(&rtksvr); // unlock
	
	for (j=k=0;j<MAXSAT;j++) {
		if (rtk->opt.mode==PMODE_SINGLE&&!rtk->ssat[j].vs) continue;
		if (rtk->opt.mode!=PMODE_SINGLE&&!rtk->ssat[j].vsat[0]) continue;
		azel[  k*2]=rtk->ssat[j].azel[0];
		azel[1+k*2]=rtk->ssat[j].azel[1];
		k++;
	}
	dops(k,azel,0.0,dop);
	
	if (rtk->opt.navsys&SYS_GPS) navsys=navsys+"GPS ";
	if (rtk->opt.navsys&SYS_GLO) navsys=navsys+"GLONASS ";
	if (rtk->opt.navsys&SYS_GAL) navsys=navsys+"Galileo ";
	if (rtk->opt.navsys&SYS_QZS) navsys=navsys+"QZSS ";
	if (rtk->opt.navsys&SYS_CMP) navsys=navsys+"BDS ";
	if (rtk->opt.navsys&SYS_IRN) navsys=navsys+"NavIC ";
	if (rtk->opt.navsys&SYS_SBS) navsys=navsys+"SBAS ";
	
	Label->Caption="";
	Tbl->RowCount=55+NFREQ*2;
	
	i=1;
	Tbl->Cells[0][i  ]="RTKLIB Version";
	Tbl->Cells[1][i++]=s.sprintf("%s %s",VER_RTKLIB,PATCH_LEVEL);
	
	Tbl->Cells[0][i  ]="RTK Server Thread";
	Tbl->Cells[1][i++]=s.sprintf("%d",thread);
	
	Tbl->Cells[0][i  ]="RTK Server State";
	Tbl->Cells[1][i++]=svrstate[state];
	
	Tbl->Cells[0][i  ]="Processing Cycle (ms)";
	Tbl->Cells[1][i++]=s.sprintf("%d",cycle);
	
	Tbl->Cells[0][i  ]="Positioning Mode";
	Tbl->Cells[1][i++]=mode[rtk->opt.mode];
	
	Tbl->Cells[0][i  ]="Frequencies";
	Tbl->Cells[1][i++]=freq[rtk->opt.nf];
	
	Tbl->Cells[0][i  ]="Elevation Mask (deg)";
	Tbl->Cells[1][i++]=s.sprintf("%.0f",rtk->opt.elmin*R2D);
	
	Tbl->Cells[0][i  ]="SNR Mask L1 (dBHz)";
	Tbl->Cells[1][i++]=!rtk->opt.snrmask.ena[0]?s.sprintf(""):
		s.sprintf("%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f",
				  rtk->opt.snrmask.mask[0][0],rtk->opt.snrmask.mask[0][1],rtk->opt.snrmask.mask[0][2],
				  rtk->opt.snrmask.mask[0][3],rtk->opt.snrmask.mask[0][4],rtk->opt.snrmask.mask[0][5],
				  rtk->opt.snrmask.mask[0][6],rtk->opt.snrmask.mask[0][7],rtk->opt.snrmask.mask[0][8]);
	
	Tbl->Cells[0][i  ]="SNR Mask L2 (dBHz)";
	Tbl->Cells[1][i++]=!rtk->opt.snrmask.ena[0]?s.sprintf(""):
		s.sprintf("%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f",
				  rtk->opt.snrmask.mask[1][0],rtk->opt.snrmask.mask[1][1],rtk->opt.snrmask.mask[1][2],
				  rtk->opt.snrmask.mask[1][3],rtk->opt.snrmask.mask[1][4],rtk->opt.snrmask.mask[1][5],
				  rtk->opt.snrmask.mask[1][6],rtk->opt.snrmask.mask[1][7],rtk->opt.snrmask.mask[1][8]);

	Tbl->Cells[0][i  ]="SNR Mask L5 (dBHz)";
	Tbl->Cells[1][i++]=!rtk->opt.snrmask.ena[0]?s.sprintf(""):
		s.sprintf("%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f",
				  rtk->opt.snrmask.mask[2][0],rtk->opt.snrmask.mask[2][1],rtk->opt.snrmask.mask[2][2],
				  rtk->opt.snrmask.mask[2][3],rtk->opt.snrmask.mask[2][4],rtk->opt.snrmask.mask[2][5],
				  rtk->opt.snrmask.mask[2][6],rtk->opt.snrmask.mask[2][7],rtk->opt.snrmask.mask[2][8]);

	Tbl->Cells[0][i  ]="SNR Mask L6 (dBHz)";
	Tbl->Cells[1][i++]=!rtk->opt.snrmask.ena[0]?s.sprintf(""):
		s.sprintf("%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f",
				  rtk->opt.snrmask.mask[3][0],rtk->opt.snrmask.mask[3][1],rtk->opt.snrmask.mask[3][2],
				  rtk->opt.snrmask.mask[3][3],rtk->opt.snrmask.mask[3][4],rtk->opt.snrmask.mask[3][5],
				  rtk->opt.snrmask.mask[3][6],rtk->opt.snrmask.mask[3][7],rtk->opt.snrmask.mask[3][8]);
	Tbl->Cells[0][i  ]="Rec Dynamic/Earth Tides Correction";
	Tbl->Cells[1][i++]=s.sprintf("%s, %s",rtk->opt.dynamics?"ON":"OFF",rtk->opt.tidecorr?"ON":"OFF");
	
	Tbl->Cells[0][i  ]="Ionosphere/Troposphere Model";
	Tbl->Cells[1][i++]=s.sprintf("%s, %s",ionoopt[rtk->opt.ionoopt],tropopt[rtk->opt.tropopt]);
	
	Tbl->Cells[0][i  ]="Satellite Ephemeris";
	Tbl->Cells[1][i++]=ephopt[rtk->opt.sateph];
	
	for (j=1;j<=MAXSAT;j++) {
		if (!rtk->opt.exsats[j-1]) continue;
		satno2id(j,id);
		if (rtk->opt.exsats[j-1]==2) exsats=exsats+"+";
		exsats=exsats+id+" ";
	}
	Tbl->Cells[0][i  ]="Excluded Satellites";
	Tbl->Cells[1][i++]=exsats;
	
	Tbl->Cells[0][i  ]="Navi Systems";
	Tbl->Cells[1][i++]=navsys;
	
	Tbl->Cells[0][i  ]="Accumulated Time to Run";
	Tbl->Cells[1][i++]=s.sprintf("%02.0f:%02.0f:%04.1f",rt[0],rt[1],rt[2]);
	
	Tbl->Cells[0][i  ]="CPU Time for a Processing Cycle (ms)";
	Tbl->Cells[1][i++]=s.sprintf("%d",cputime);
	
	Tbl->Cells[0][i  ]="Missing Obs Data Count";
	Tbl->Cells[1][i++]=s.sprintf("%d",prcout);
	
	Tbl->Cells[0][i  ]="Bytes in Input Buffer";
	Tbl->Cells[1][i++]=s.sprintf("%d, %d, %d",nb[0],nb[1],nb[2]);
	
	Tbl->Cells[0][i  ]="# of Input Data Rover";
	Tbl->Cells[1][i++]=s.sprintf("Obs(%d), Nav(%d), Ion(%d), Sbs(%d), Pos(%d), Dgps(%d), Ssr(%d), Err(%d)",
								 nmsg[0][0],nmsg[0][1]+nmsg[0][6],nmsg[0][2],nmsg[0][3],
								 nmsg[0][4],nmsg[0][5],nmsg[0][7],nmsg[0][9]);
	
	Tbl->Cells[0][i  ]="# of Input Data Base Station";
	Tbl->Cells[1][i++]=s.sprintf("Obs(%d), Nav(%d), Ion(%d), Sbs(%d), Pos(%d), Dgps(%d), Ssr(%d), Err(%d)",
								 nmsg[1][0],nmsg[1][1]+nmsg[1][6],nmsg[1][2],nmsg[1][3],
								 nmsg[1][4],nmsg[1][5],nmsg[1][7],nmsg[1][9]);
	
	Tbl->Cells[0][i  ]="# of Input Data Ephemeris";
	Tbl->Cells[1][i++]=s.sprintf("Obs(%d), Nav(%d), Ion(%d), Sbs(%d), Pos(%d), Dgps(%d), Ssr(%d), Err(%d)",
								 nmsg[2][0],nmsg[2][1]+nmsg[2][6],nmsg[2][2],nmsg[2][3],
								 nmsg[2][4],nmsg[2][5],nmsg[2][7],nmsg[2][9]);
	
	Tbl->Cells[0][i  ]="Solution Status";
	Tbl->Cells[1][i++]=sol[rtkstat];
	
	time2str(rtk->sol.time,tstr,9);
	Tbl->Cells[0][i  ] ="Time of Receiver Clock Rover";
	Tbl->Cells[1][i++]=rtk->sol.time.time?tstr:"-";
	
	Tbl->Cells[0][i  ] ="Time Sytem Offset/Receiver Bias (GLO-GPS,GAL-GPS,BDS-GPS,IRN-GPS) (ns)";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f, %.3f",rtk->sol.dtr[1]*1E9,rtk->sol.dtr[2]*1E9,
                                 rtk->sol.dtr[3]*1E9,rtk->sol.dtr[4]*1E9);
	
	Tbl->Cells[0][i  ]="Solution Interval (s)";
	Tbl->Cells[1][i++]=s.sprintf("%.3f",rtk->tt);
	
	Tbl->Cells[0][i  ]="Age of Differential (s)";
	Tbl->Cells[1][i++] =s.sprintf("%.3f",rtk->sol.age);
	
	Tbl->Cells[0][i  ]="Ratio for AR Validation";
	Tbl->Cells[1][i++]=s.sprintf("%.3f",rtk->sol.ratio);
	
	Tbl->Cells[0][i  ]="# of Satellites Rover";
	Tbl->Cells[1][i++]=s.sprintf("%d",nsat0);
	
	Tbl->Cells[0][i  ]="# of Satellites Base Station";
	Tbl->Cells[1][i++]=s.sprintf("%d",nsat1);
	
	Tbl->Cells[0][i  ]="# of Valid Satellites";
	Tbl->Cells[1][i++]=s.sprintf("%d",rtk->sol.ns);
	
	Tbl->Cells[0][i  ]="GDOP/PDOP/HDOP/VDOP";
	Tbl->Cells[1][i++]=s.sprintf("%.1f, %.1f, %.1f, %.1f",dop[0],dop[1],dop[2],dop[3]);
	
	Tbl->Cells[0][i  ]="# of Real Estimated States";
	Tbl->Cells[1][i++]=s.sprintf("%d",rtk->na);
	
	Tbl->Cells[0][i  ]="# of All Estimated States";
	Tbl->Cells[1][i++]=s.sprintf("%d",rtk->nx);
	
	Tbl->Cells[0][i  ]="Pos X/Y/Z Single (m) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",rtk->sol.rr[0],rtk->sol.rr[1],rtk->sol.rr[2]);
	
	if (norm(rtk->sol.rr,3)>0.0) ecef2pos(rtk->sol.rr,pos); else pos[0]=pos[1]=pos[2]=0.0;
	Tbl->Cells[0][i  ]="Lat/Lon/Height Single (deg,m) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.8f, %.8f, %.3f",pos[0]*R2D,pos[1]*R2D,pos[2]);
	
	ecef2enu(pos,rtk->sol.rr+3,vel);
	Tbl->Cells[0][i  ]="Vel E/N/U (m/s) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",vel[0],vel[1],vel[2]);
	
	Tbl->Cells[0][i  ]="Pos X/Y/Z Float (m) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",
		rtk->x?rtk->x[0]:0,rtk->x?rtk->x[1]:0,rtk->x?rtk->x[2]:0);
	
	Tbl->Cells[0][i  ]="Pos X/Y/Z Float Std (m) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",
		rtk->P?SQRT(rtk->P[0]):0,rtk->P?SQRT(rtk->P[1+1*rtk->nx]):0,rtk->P?SQRT(rtk->P[2+2*rtk->nx]):0);
	
	Tbl->Cells[0][i  ]="Pos X/Y/Z Fixed (m) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",
		rtk->xa?rtk->xa[0]:0,rtk->xa?rtk->xa[1]:0,rtk->xa?rtk->xa[2]:0);
	
	Tbl->Cells[0][i  ]="Pos X/Y/Z Fixed Std (m) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",
		rtk->Pa?SQRT(rtk->Pa[0]):0,rtk->Pa?SQRT(rtk->Pa[1+1*rtk->na]):0,rtk->Pa?SQRT(rtk->Pa[2+2*rtk->na]):0);
	
	Tbl->Cells[0][i  ]="Pos X/Y/Z (m) Base Station";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",rtk->rb[0],rtk->rb[1],rtk->rb[2]);
	
	if (norm(rtk->rb,3)>0.0) ecef2pos(rtk->rb,pos); else pos[0]=pos[1]=pos[2]=0.0;
	Tbl->Cells[0][i  ]="Lat/Lon/Height (deg,m) Base Station";
	Tbl->Cells[1][i++]=s.sprintf("%.8f, %.8f, %.3f",pos[0]*R2D,pos[1]*R2D,pos[2]);
	
	ecef2enu(pos,rtk->rb+3,vel);
	Tbl->Cells[0][i  ]="Vel E/N/U (m/s) Base Station";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",vel[0],vel[1],vel[2]);
	
	if (norm(rtk->rb,3)>0.0) {
	    for (k=0;k<3;k++) rr[k]=rtk->sol.rr[k]-rtk->rb[k];
	    ecef2enu(pos,rr,enu);
	}
	Tbl->Cells[0][i  ]="Baseline Length/E/N/U (m) Rover-Base Station";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f, %.3f",norm(rr,3),enu[0],enu[1],enu[2]);
	
	Tbl->Cells[0][i  ]="# of Averaging Single Pos Base Station";
	Tbl->Cells[1][i++]=s.sprintf("%d",nave);
	
	Tbl->Cells[0][i  ]="Antenna Type Rover";
	Tbl->Cells[1][i++]=rtk->opt.pcvr[0].type;
	
	for (j=0;j<NFREQ;j++) {
	    off=rtk->opt.pcvr[0].off[j];
	    Tbl->Cells[0][i  ]=s.sprintf("Ant Phase Center L%d E/N/U (m) Rover",j+1);
	    Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",off[0],off[1],off[2]);
	}
	del=rtk->opt.antdel[0];
	Tbl->Cells[0][i  ]="Ant Delta E/N/U (m) Rover";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",del[0],del[1],del[2]);
	
	Tbl->Cells[0][i  ]="Antenna Type Base Station";
	Tbl->Cells[1][i++]=rtk->opt.pcvr[1].type;
	
	for (j=0;j<NFREQ;j++) {
	    off=rtk->opt.pcvr[1].off[0];
    	Tbl->Cells[0][i  ]=s.sprintf("Ant Phase Center L%d E/N/U (m) Base Station",j+1);
	    Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",off[0],off[1],off[2]);
	}
	del=rtk->opt.antdel[1];
	Tbl->Cells[0][i  ]="Ant Delta E/N/U (m) Base Station";
	Tbl->Cells[1][i++]=s.sprintf("%.3f, %.3f, %.3f",del[0],del[1],del[2]);
	
	Tbl->Cells[0][i  ]="Precise Ephemeris Time/# of Epoch";
	Tbl->Cells[1][i++]=s.sprintf("%s-%s (%d)",s1,s2,ne);
	
	Tbl->Cells[0][i  ]="Precise Ephemeris Download Time";
	Tbl->Cells[1][i++]=s3;
	
	Tbl->Cells[0][i  ]="Precise Ephemeris Download File";
	Tbl->Cells[1][i++]=file;
        free(rtk);
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetSat(void)
{
	int i,j=0;
	AnsiString s,label[]={
		"SAT","Status","Azimuth (deg)","Elevation (deg)","LG (m)","PHW(cyc)",
		"P1-P2(m)","P1-C1(m)","P2-C2(m)"
	};
	int width[]={25,30,45,45,60,40,40,40,40},nfreq;
	
	rtksvrlock(&rtksvr);
	nfreq=rtksvr.rtk.opt.nf;
	rtksvrunlock(&rtksvr);
	
	Tbl->ColCount=9+nfreq*8;
	Tbl->RowCount=2;
	for (i=0;i<4;i++) {
		Tbl->ColWidths [j]=width[i]*FontScale/96;
		Tbl->Cells[j  ][0]=label[i];
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=30*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("L%d",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=40*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("Fix%d",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=45*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("P%d Residual(m)",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=45*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("L%d Residual(m)",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=45*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("Slip%d",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=45*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("Lock%d",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=45*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("Outage%d",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<nfreq;i++) {
		Tbl->ColWidths [j]=45*FontScale/96;
		Tbl->Cells[j  ][0]=s.sprintf("Reject%d",i+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=4;i<9;i++) {
		Tbl->ColWidths [j]=width[i]*FontScale/96;
		Tbl->Cells[j  ][0]=label[i];
		Tbl->Cells[j++][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowSat(void)
{
	ssat_t *ssat;
	AnsiString s;
	int i,j,k,n,fix,prn,pmode,nfreq,sys=sys_tbl[SelSys->ItemIndex];
	int vsat[MAXSAT]={0};
	char id[8];
	double az,el,cbias[MAXSAT][2];
	
        rtk_t *rtk = static_cast<rtk_t *>(malloc(sizeof(rtk_t)));
        if (rtk == NULL) return;

	SetSat();

	rtksvrlock(&rtksvr);
	*rtk=rtksvr.rtk;
	for (i=0;i<MAXSAT;i++) for (j=0;j<2;j++) {
		cbias[i][j]=rtksvr.nav.cbias[i][j][0];
	}
	pmode=rtksvr.rtk.opt.mode;
	nfreq=rtksvr.rtk.opt.nf;
	rtksvrunlock(&rtksvr);
	
	Label->Caption="";
	
	for (i=0;i<MAXSAT;i++) {
		ssat=rtk->ssat+i;
		vsat[i]=ssat->vs;
	}
	for (i=0,n=1;i<MAXSAT;i++) {
		if (!(satsys(i+1,NULL)&sys)) continue;
		ssat=rtk->ssat+i;
		if (SelSat->ItemIndex==1&&!vsat[i]) continue;
		n++;
	}
	if (n<2) {
		Tbl->RowCount=2;
		for (i=0;i<Tbl->ColCount;i++) Tbl->Cells[i][1]="";
                free(rtk);
		return;
	}
	Tbl->RowCount=n;
	
	for (i=0,n=1;i<MAXSAT;i++) {
		if (!(satsys(i+1,NULL)&sys)) continue;
		j=0;
		ssat=rtk->ssat+i;
		if (SelSat->ItemIndex==1&&!vsat[i]) continue;
		satno2id(i+1,id);
		Tbl->Cells[j++][n]=id;
		Tbl->Cells[j++][n]=ssat->vs?"OK":"-";
		az=ssat->azel[0]*R2D; if (az<0.0) az+=360.0;
		el=ssat->azel[1]*R2D;
		Tbl->Cells[j++][n]=s.sprintf("%.1f",az);
		Tbl->Cells[j++][n]=s.sprintf("%.1f",el);
		for (k=0;k<nfreq;k++) {
			Tbl->Cells[j++][n]=ssat->vsat[k]?"OK":"-";
		}
		for (k=0;k<nfreq;k++) {
			fix=ssat->fix[k];
			Tbl->Cells[j++][n]=fix==1?"FLOAT":(fix==2?"FIX":(fix==3?"HOLD":"-"));
		}
		for (k=0;k<nfreq;k++) {
			Tbl->Cells[j++][n]=s.sprintf("%.2f",ssat->resp[k]);
		}
		for (k=0;k<nfreq;k++) {
			Tbl->Cells[j++][n]=s.sprintf("%.4f",ssat->resc[k]);
		}
		for (k=0;k<nfreq;k++) {
			Tbl->Cells[j++][n]=s.sprintf("%d",ssat->slipc[k]);
		}
		for (k=0;k<nfreq;k++) {
			Tbl->Cells[j++][n]=s.sprintf("%d",ssat->lock[k]);
		}
		for (k=0;k<nfreq;k++) {
			Tbl->Cells[j++][n]=s.sprintf("%d",ssat->outc[k]);
		}
		for (k=0;k<nfreq;k++) {
			Tbl->Cells[j++][n]=s.sprintf("%d",ssat->rejc[k]);
		}
		Tbl->Cells[j++][n]=s.sprintf("%.3f",ssat->gf[0]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",ssat->phw);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",cbias[i][0]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",cbias[i][1]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",0);
		n++;
	}
        free(rtk);
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetEst(void)
{
	AnsiString label[]={
		"State","Estimate Float","Std Float","Estimate Fixed","Std Fixed"
	};
	int i,width[]={40,100,100,100,100};
	
	Tbl->ColCount=5;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowEst(void)
{
	gtime_t time;
	int i,nx,na,n;
	double *x,*P=NULL,*xa=NULL,*Pa=NULL;
	AnsiString s,s0="-";
	char tstr[40];

	rtksvrlock(&rtksvr);
	
	time=rtksvr.rtk.sol.time;
	nx=rtksvr.rtk.nx;
	na=rtksvr.rtk.na;
	if ((x =(double *)malloc(sizeof(double)*nx))&&
	    (P =(double *)malloc(sizeof(double)*nx*nx))&&
		(xa=(double *)malloc(sizeof(double)*na))&&
	    (Pa=(double *)malloc(sizeof(double)*na*na))) {
		memcpy(x ,rtksvr.rtk.x ,sizeof(double)*nx);
		memcpy(P ,rtksvr.rtk.P ,sizeof(double)*nx*nx);
		memcpy(xa,rtksvr.rtk.xa,sizeof(double)*na);
		memcpy(Pa,rtksvr.rtk.Pa,sizeof(double)*na*na);
	}
	else {
		rtksvrunlock(&rtksvr);
		free(x); free(P); free(xa); free(Pa);
		return;
	}
	rtksvrunlock(&rtksvr);
	
	for (i=0,n=1;i<nx;i++) {
		if (SelSat->ItemIndex==1&&x[i]==0.0) continue;
		n++;
	}
	if (n<2) {
		Tbl->RowCount=2;
		for (i=0;i<Tbl->ColCount;i++) Tbl->Cells[i][1]="";
		return;
	}
	Tbl->RowCount=n;
	
	time2str(time,tstr,9);
	Label->Caption=time.time?s.sprintf("Time: %s",tstr):s0;
	for (i=0,n=1;i<nx;i++) {
		int j=0;
		if (SelSat->ItemIndex==1&&x[i]==0.0) continue;
		Tbl->Cells[j++][n]=s.sprintf("X_%d",i+1);
		Tbl->Cells[j++][n]=x[i]==0.0?s0:s.sprintf("%.3f",x[i]);
		Tbl->Cells[j++][n]=P[i+i*nx]==0.0?s0:s.sprintf("%.3f",SQRT(P[i+i*nx]));
		Tbl->Cells[j++][n]=i>=na||xa[i]==0?s0:s.sprintf("%.3f",xa[i]);
		Tbl->Cells[j++][n]=i>=na||Pa[i+i*na]==0.0?s0:s.sprintf("%.3f",SQRT(Pa[i+i*na]));
		n++;
	}
	free(x); free(P); free(xa); free(Pa);
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetCov(void)
{
	int i;
	
	Tbl->ColCount=2;
	Tbl->RowCount=2;
	for (i=0;i<2;i++) {
		Tbl->ColWidths[i]=(i==0?35:45)*FontScale/96;
		Tbl->Cells[i][0]="";
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowCov(void)
{
	gtime_t time;
	int i,j,nx,n,m;
	double *x,*P=NULL;
	AnsiString s,s0="-";
	char tstr[40];

	rtksvrlock(&rtksvr);
	
	time=rtksvr.rtk.sol.time;
	nx=rtksvr.rtk.nx;
	if ((x =(double *)malloc(sizeof(double)*nx))&&
	    (P =(double *)malloc(sizeof(double)*nx*nx))) {
		memcpy(x ,rtksvr.rtk.x ,sizeof(double)*nx);
		memcpy(P ,rtksvr.rtk.P ,sizeof(double)*nx*nx);
	}
	else {
		rtksvrunlock(&rtksvr);
		free(x); free(P);
		return;
	}
	rtksvrunlock(&rtksvr);
	
	for (i=0,n=1;i<nx;i++) {
		if (SelSat->ItemIndex==1&&(x[i]==0.0||P[i+i*nx]==0.0)) continue;
		n++;
	}
	if (n<2) {
		Tbl->ColCount=2;
		Tbl->RowCount=2;
		Tbl->Cells[1][1]="";
		return;
	}
	Tbl->ColCount=n;
	Tbl->RowCount=n;
	
	time2str(time,tstr,9);
	Label->Caption=time.time?s.sprintf("Time: %s",tstr):s0;
	for (i=0,n=1;i<nx;i++) {
		if (SelSat->ItemIndex==1&&(x[i]==0.0||P[i+i*nx]==0.0)) continue;
		Tbl->ColWidths[n]=45*FontScale/96;
		Tbl->Cells[0][n]=s.sprintf("X_%d",i+1);
		Tbl->Cells[n][0]=s.sprintf("X_%d",i+1);
		for (j=0,m=1;j<nx;j++) {
			if (SelSat->ItemIndex==1&&(x[j]==0.0||P[j+j*nx]==0.0)) continue;
			Tbl->Cells[m][n]=
				P[i+j*nx]==0.0?s0:s.sprintf("%.5f",SQRT(P[i+j*nx]));
			m++;
		}
		n++;
	}
	free(x); free(P);
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetObs(void)
{
	AnsiString s,label[]={"Trcv (GPST)","SAT","STR"};
	int i,j=0,width[]={135,25,25};
	int nex=ObsMode?NEXOBS:0;
	
	Tbl->ColCount=3+(NFREQ+nex)*6;
	Tbl->RowCount=2;
	for (i=0;i<3;i++) {
		Tbl->ColWidths [j]=width[i]*FontScale/96;
		Tbl->Cells[j  ][0]=label[i];
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<NFREQ+nex;i++) {
		Tbl->ColWidths [j]=22*FontScale/96;
		Tbl->Cells[j  ][0]=i<NFREQ?s.sprintf("C%d",i+1):s.sprintf("CX%d",i-NFREQ+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<NFREQ+nex;i++) {
		Tbl->ColWidths [j]=30*FontScale/96;
		Tbl->Cells[j  ][0]=i<NFREQ?s.sprintf("S%d",i+1):s.sprintf("SX%d",i-NFREQ+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<NFREQ+nex;i++) {
		Tbl->ColWidths [j]=80*FontScale/96;
		Tbl->Cells[j  ][0]=i<NFREQ?s.sprintf("P%d (m)",i+1):s.sprintf("PX%d (m)",i-NFREQ+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<NFREQ+nex;i++) {
		Tbl->ColWidths [j]=85*FontScale/96;
		Tbl->Cells[j  ][0]=i<NFREQ?s.sprintf("L%d (cycle)",i+1):s.sprintf("LX%d (cycle)",i-NFREQ+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<NFREQ+nex;i++) {
		Tbl->ColWidths [j]=60*FontScale/96;
		Tbl->Cells[j  ][0]=i<NFREQ?s.sprintf("D%d (Hz)",i+1):s.sprintf("DX%d (Hz)",i-NFREQ+1);
		Tbl->Cells[j++][1]="";
	}
	for (i=0;i<NFREQ+nex;i++) {
		Tbl->ColWidths [j]=15*FontScale/96;
		Tbl->Cells[j  ][0]="I";
		Tbl->Cells[j++][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowObs(void)
{
	AnsiString s;
	char tstr[40],id[8],*code;
	int i,j,k,n=0,nex=ObsMode?NEXOBS:0,sys=sys_tbl[SelSys->ItemIndex];
	
        obsd_t *obs = static_cast<obsd_t *>(calloc(MAXOBS * 2, sizeof(obsd_t)));
        if (obs == NULL) {
          trace(1, "TMonitorDialog::ShowObs obsd_t alloc failed\n");
          return;
        }

	rtksvrlock(&rtksvr);
	for (i=0;i<rtksvr.obs[0][0].n&&n<MAXOBS*2;i++) {
        if (!(satsys(rtksvr.obs[0][0].data[i].sat,NULL)&sys)) continue;
		obs[n++]=rtksvr.obs[0][0].data[i];
	}
	for (i=0;i<rtksvr.obs[1][0].n&&n<MAXOBS*2;i++) {
        if (!(satsys(rtksvr.obs[1][0].data[i].sat,NULL)&sys)) continue;
		obs[n++]=rtksvr.obs[1][0].data[i];
	}
	rtksvrunlock(&rtksvr);
	
	Tbl->RowCount=n+1<2?2:n+1;
	Label->Caption="";
	
	for (i=0;i<Tbl->ColCount;i++) Tbl->Cells[i][1]="";
	for (i=0;i<n;i++) {
		j=0;
		time2str(obs[i].time,tstr,3);
		Tbl->Cells[j++][i+1]=tstr;
		satno2id(obs[i].sat,id);
		Tbl->Cells[j++][i+1]=id;
		Tbl->Cells[j++][i+1]=s.sprintf("(%d)",obs[i].rcv);
		for (k=0;k<NFREQ+nex;k++) {
			code=code2obs(obs[i].code[k]);
			if (*code) Tbl->Cells[j++][i+1]=s.sprintf("%s",code);
			else       Tbl->Cells[j++][i+1]="-";
		}
		for (k=0;k<NFREQ+nex;k++) {
			if (obs[i].SNR[k]) Tbl->Cells[j++][i+1]=s.sprintf("%.1f",obs[i].SNR[k]);
			else               Tbl->Cells[j++][i+1]=s.sprintf("-");
		}
		for (k=0;k<NFREQ+nex;k++) {
			Tbl->Cells[j++][i+1]=s.sprintf("%.3f",obs[i].P[k]);
		}
		for (k=0;k<NFREQ+nex;k++) {
			Tbl->Cells[j++][i+1]=s.sprintf("%.3f",obs[i].L[k]);
		}
		for (k=0;k<NFREQ+nex;k++) {
			Tbl->Cells[j++][i+1]=s.sprintf("%.3f",obs[i].D[k]);
		}
		for (k=0;k<NFREQ+nex;k++) {
			Tbl->Cells[j++][i+1]=s.sprintf("%d",obs[i].LLI[k]);
		}
	}
        free(obs);
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetNav(void)
{
	UTF8String label[]={
		"SAT","PRN","Status","IODE","IODC","URA","SVH","Toe","Toc","Ttrans",
		"A (m)","e","i0 (\302\260)","\316\2510 (\302\260)","\317\211 (\302\260)",
		"M0 (\302\260)","\316\224n (\302\260/s)","\316\251dot (\302\260/s)",
		"IDOT (\302\260/s)","Af0 (ns)","Af1 (ns/s)","Af2 (ns/s2)","TGD1 (ns)",
		"TGD2 (ns)","Cuc (rad)","Cus (rad)","Crc (m)","Crs (m)","Cic (rad)",
		"Cis (rad)","Code","Flag",
	};
	int i,width[]={
		25,25,30,30,30,25,25,115,115,115, 80,80,60,60,60,60,70,70,70,70,
		50,50,50,50,70,70,50,50,70, 70,30,30
	};
	Tbl->ColCount=32;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowNav(void)
{
	eph_t eph[MAXSAT];
	gtime_t time;
	AnsiString s;
	char tstr[40],id[8];
	int i,j,k,n,valid,prn,off=SelEph->ItemIndex*MAXSAT;
	int sys=sys_tbl[SelSys2->ItemIndex+1];
	
	if (sys==SYS_GLO) {
		SetGnav();
		ShowGnav();
		return;
	}
	if (sys==SYS_SBS) {
		SetSbsNav();
		ShowSbsNav();
		return;
	}
	SetNav();
	
	rtksvrlock(&rtksvr);
	time=rtksvr.rtk.sol.time;
	for (i=0;i<MAXSAT;i++) eph[i]=rtksvr.nav.eph[i+off];
	rtksvrunlock(&rtksvr);

	if (sys==SYS_GAL) {
	    Label->Caption=(SelEph->ItemIndex%2)?"F/NAV":"I/NAV";
	}
	else {
	    Label->Caption="";
	}
	for (k=0,n=1;k<MAXSAT;k++) {
		int ssys = satsys(k+1,&prn);
		if (!(ssys&sys)) continue;
		// Mask QZS LEX health.
		valid = eph[k].toe.time != 0 && fabs(timediff(time, eph[k].toe)) <= MAXDTOE &&
			(ssys == SYS_QZS ? (eph[k].svh & 0xfe) == 0 : eph[k].svh == 0);
		if (SelSat->ItemIndex==1&&!valid) continue;
		n++;
	}
	if (n<2) {
		Tbl->RowCount=2;
		for (i=0;i<Tbl->ColCount;i++) Tbl->Cells[i][1]="";
		return;
	}
	Tbl->RowCount=n;
	
	for (k=0,n=1;k<MAXSAT;k++) {
		j=0;
		int ssys = satsys(k+1,&prn);
		if (!(ssys&sys)) continue;
		// Mask QZS LEX health.
		valid = eph[k].toe.time != 0 && fabs(timediff(time, eph[k].toe)) <= MAXDTOE &&
			(ssys == SYS_QZS ? (eph[k].svh & 0xfe) == 0 : eph[k].svh == 0);
		if (SelSat->ItemIndex==1&&!valid) continue;
		satno2id(k+1,id);
		Tbl->Cells[j++][n]=id;
		Tbl->Cells[j++][n]=s.sprintf("%d",prn);
		Tbl->Cells[j++][n]=valid?"OK":"-";
		if (eph[k].iode<0) s="-"; else s.sprintf("%d",eph[k].iode);
		Tbl->Cells[j++][n]=s;
		if (eph[k].iodc<0) s="-"; else s.sprintf("%d",eph[k].iodc);
		Tbl->Cells[j++][n]=s;
		Tbl->Cells[j++][n]=s.sprintf("%d",eph[k].sva);
		Tbl->Cells[j++][n]=s.sprintf("%03x",eph[k].svh);
		if (eph[k].toe.time!=0) time2str(eph[k].toe,tstr,0); else strcpy(tstr,"-");
		Tbl->Cells[j++][n]=tstr;
		if (eph[k].toc.time!=0) time2str(eph[k].toc,tstr,0); else strcpy(tstr,"-");
		Tbl->Cells[j++][n]=tstr;
		if (eph[k].ttr.time!=0) time2str(eph[k].ttr,tstr,0); else strcpy(tstr,"-");
		Tbl->Cells[j++][n]=tstr;
		Tbl->Cells[j++][n]=s.sprintf("%.3f",eph[k].A);
		Tbl->Cells[j++][n]=s.sprintf("%.8f",eph[k].e);
		Tbl->Cells[j++][n]=s.sprintf("%.5f",eph[k].i0  *R2D);
		Tbl->Cells[j++][n]=s.sprintf("%.5f",eph[k].OMG0*R2D);
		Tbl->Cells[j++][n]=s.sprintf("%.5f",eph[k].omg *R2D);
		Tbl->Cells[j++][n]=s.sprintf("%.5f",eph[k].M0  *R2D);
		Tbl->Cells[j++][n]=s.sprintf("%.4E",eph[k].deln*R2D);
		Tbl->Cells[j++][n]=s.sprintf("%.4E",eph[k].OMGd*R2D);
		Tbl->Cells[j++][n]=s.sprintf("%.4E",eph[k].idot*R2D);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",eph[k].f0*1E9);
		Tbl->Cells[j++][n]=s.sprintf("%.4f",eph[k].f1*1E9);
		Tbl->Cells[j++][n]=s.sprintf("%.4f",eph[k].f2*1E9);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",eph[k].tgd[0]*1E9);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",eph[k].tgd[1]*1E9);
		
		Tbl->Cells[j++][n]=s.sprintf("%.4E",eph[k].cuc);
		Tbl->Cells[j++][n]=s.sprintf("%.4E",eph[k].cus);
		Tbl->Cells[j++][n]=s.sprintf("%.3f",eph[k].crc);
		Tbl->Cells[j++][n]=s.sprintf("%.3f",eph[k].crs);
		Tbl->Cells[j++][n]=s.sprintf("%.4E",eph[k].cic);
		Tbl->Cells[j++][n]=s.sprintf("%.4E",eph[k].cis);
		Tbl->Cells[j++][n]=s.sprintf("%03x",eph[k].code);
		Tbl->Cells[j++][n]=s.sprintf("%02x",eph[k].flag);
		n++;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetGnav(void)
{
	UTF8String label[]={
		"SAT","PRN","Status","IOD","FCN","SVH","Age (days)","Toe","Tof",
		"X (m)","Y (m)","Z (m)","Vx (m/s)","Vy (m/s)","Vz (m/s)",
		"Ax (m/s\302\262)","Ay (m/s\302\262)","Az (m/s\302\262)",
		"\317\204 (ns)","\316\263 (ns/s)","\316\224\317\204 (ns)"
	};
	int i,width[]={
		25,25,30,30,30,25,25,115,115,75,75,75,70,70,70,65,65,65,70,60,50
	};
	Tbl->ColCount=21;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowGnav(void)
{
	geph_t geph[NSATGLO];
	gtime_t time;
	AnsiString s;
	char tstr[40],id[8];
	int i,j,n,valid,prn,off=SelEph->ItemIndex?NSATGLO:0;
	
	rtksvrlock(&rtksvr);
	time=rtksvr.rtk.sol.time;
	for (i=0;i<NSATGLO;i++) geph[i]=rtksvr.nav.geph[i+off];
	rtksvrunlock(&rtksvr);
	
	Label->Caption="";
	
	for (i=0,n=1;i<NSATGLO;i++) {
		valid = geph[i].toe.time != 0 && fabs(timediff(time, geph[i].toe)) <= MAXDTOE_GLO &&
			(geph[i].svh & 9) == 0 && (geph[i].svh & 6) != 4;
		if (SelSat->ItemIndex==1&&!valid) continue;
		n++;
	}
	if (n<2) {
		Tbl->RowCount=2;
		for (i=0;i<Tbl->ColCount;i++) Tbl->Cells[i][1]="";
		return;
	}
	Tbl->RowCount=n;
	
	for (i=0,n=1;i<NSATGLO;i++) {
		j=0;
		valid = geph[i].toe.time != 0 && fabs(timediff(time, geph[i].toe)) <= MAXDTOE_GLO &&
			(geph[i].svh & 9) == 0 && (geph[i].svh & 6) != 4;
		if (SelSat->ItemIndex==1&&!valid) continue;
		prn=MINPRNGLO+i;
		satno2id(satno(SYS_GLO,prn),id);
		Tbl->Cells[j++][n]=id;
		Tbl->Cells[j++][n]=s.sprintf("%d",prn);
		Tbl->Cells[j++][n]=valid?"OK":"-";
		if (geph[i].iode<0) s="-"; else s.sprintf("%d",geph[i].iode);
		Tbl->Cells[j++][n]=s;
		Tbl->Cells[j++][n]=s.sprintf("%d",geph[i].frq);
		Tbl->Cells[j++][n]=s.sprintf("%02x",geph[i].svh);
		Tbl->Cells[j++][n]=s.sprintf("%d",geph[i].age);
		if (geph[i].toe.time!=0) time2str(geph[i].toe,tstr,0); else strcpy(tstr,"-");
		Tbl->Cells[j++][n]=tstr;
		if (geph[i].tof.time!=0) time2str(geph[i].tof,tstr,0); else strcpy(tstr,"-");
		Tbl->Cells[j++][n]=tstr;
		Tbl->Cells[j++][n]=s.sprintf("%.2f",geph[i].pos[0]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",geph[i].pos[1]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",geph[i].pos[2]);
		Tbl->Cells[j++][n]=s.sprintf("%.5f",geph[i].vel[0]);
		Tbl->Cells[j++][n]=s.sprintf("%.5f",geph[i].vel[1]);
		Tbl->Cells[j++][n]=s.sprintf("%.5f",geph[i].vel[2]);
		Tbl->Cells[j++][n]=s.sprintf("%.7f",geph[i].acc[0]);
		Tbl->Cells[j++][n]=s.sprintf("%.7f",geph[i].acc[1]);
		Tbl->Cells[j++][n]=s.sprintf("%.7f",geph[i].acc[2]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",geph[i].taun*1E9);
		Tbl->Cells[j++][n]=s.sprintf("%.4f",geph[i].gamn*1E9);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",geph[i].dtaun*1E9);
		n++;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetSbsNav(void)
{
	UTF8String label[]={
		"SAT","PRN","Status","T0","Tof","SVH","URA","X (m)","Y (m)","Z (m)",
		"Vx (m/s)","Vy (m/s)","Vz (m/s)","Ax (m/s\302\262)","Ay (m/s\302\262)",
		"Az (m/s\302\262)","Af0 (ns)","Af1 (ns/s)"
	};
	int i,width[]={25,25,30,115,115,30,30,75,75,75,70,70,70,65,65,65,60,60};
	
	Tbl->ColCount=18;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowSbsNav(void)
{
	AnsiString s,s0="-";
	seph_t seph[MAXPRNSBS-MINPRNSBS+1]={0};
	gtime_t time;
	int i,j,n,valid,prn,off=SelEph->ItemIndex?NSATSBS:0;
	char tstr[40],id[8];
	
	rtksvrlock(&rtksvr); // lock
	time=rtksvr.rtk.sol.time;
	for (int i=0;i<NSATSBS;i++) {
		seph[i]=rtksvr.nav.seph[i+off];
	}
	rtksvrunlock(&rtksvr); // unlock
	
	Label->Caption="";
	
	for (i=0,n=1;i<NSATSBS;i++) {
		valid=fabs(timediff(time,seph[i].t0))<=MAXDTOE_SBS&&
			  seph[i].t0.time&&seph[i].svh==0;
		if (SelSat->ItemIndex==1&&!valid) continue;
		n++;
	}
	if (n<2) {
		Tbl->RowCount=2;
		for (i=0;i<Tbl->ColCount;i++) Tbl->Cells[i][1]="";
		return;
	}
	Tbl->RowCount=n;
	
	for (i=0,n=1;i<NSATSBS;i++) {
		j=0;
		valid=fabs(timediff(time,seph[i].t0))<=MAXDTOE_SBS&&
			  seph[i].t0.time&&seph[i].svh==0;
		if (SelSat->ItemIndex==1&&!valid) continue;
		prn=MINPRNSBS+i;
		satno2id(satno(SYS_SBS,prn),id);
		Tbl->Cells[j++][n]=id;
		Tbl->Cells[j++][n]=s.sprintf("%d",prn);
		Tbl->Cells[j++][n]=valid?"OK":"-";
		if (seph[i].t0.time) time2str(seph[i].t0,tstr,0);
		else strcpy(tstr,"-");
		Tbl->Cells[j++][n]=tstr;
		if (seph[i].tof.time) time2str(seph[i].tof,tstr,0);
		else strcpy(tstr,"-");
		Tbl->Cells[j++][n]=tstr;
		Tbl->Cells[j++][n]=s.sprintf("%02x",seph[i].svh);
		Tbl->Cells[j++][n]=s.sprintf("%d",  seph[i].sva);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",seph[i].pos[0]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",seph[i].pos[1]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",seph[i].pos[2]);
		Tbl->Cells[j++][n]=s.sprintf("%.6f",seph[i].vel[0]);
		Tbl->Cells[j++][n]=s.sprintf("%.6f",seph[i].vel[1]);
		Tbl->Cells[j++][n]=s.sprintf("%.6f",seph[i].vel[2]);
		Tbl->Cells[j++][n]=s.sprintf("%.7f",seph[i].acc[0]);
		Tbl->Cells[j++][n]=s.sprintf("%.7f",seph[i].acc[1]);
		Tbl->Cells[j++][n]=s.sprintf("%.7f",seph[i].acc[2]);
		Tbl->Cells[j++][n]=s.sprintf("%.2f",seph[i].af0*1E9);
		Tbl->Cells[j++][n]=s.sprintf("%.4f",seph[i].af1*1E9);
		n++;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetIonUtc(void)
{
	AnsiString label[]={"Parameter","Value"};
	int i,width[]={270,330};
	
	Tbl->ColCount=2;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowIonUtc(void)
{
	double utc_gps[8],utc_glo[8],utc_gal[8],utc_qzs[8],utc_cmp[8],utc_irn[9];
	double ion_gps[8],ion_gal[4],ion_qzs[8],ion_cmp[8],ion_irn[8];
	gtime_t time;
	AnsiString s;
	double tow=0.0;
	char tstr[40];
	int i,j,k,leaps,week=0;
	
	rtksvrlock(&rtksvr);
	time=rtksvr.rtk.sol.time;
	for (i=0;i<8;i++) utc_gps[i]=rtksvr.nav.utc_gps[i];
	for (i=0;i<8;i++) utc_glo[i]=rtksvr.nav.utc_glo[i];
	for (i=0;i<8;i++) utc_gal[i]=rtksvr.nav.utc_gal[i];
	for (i=0;i<8;i++) utc_qzs[i]=rtksvr.nav.utc_qzs[i];
	for (i=0;i<8;i++) utc_cmp[i]=rtksvr.nav.utc_cmp[i];
	for (i=0;i<9;i++) utc_irn[i]=rtksvr.nav.utc_irn[i];
	for (i=0;i<8;i++) ion_gps[i]=rtksvr.nav.ion_gps[i];
	for (i=0;i<4;i++) ion_gal[i]=rtksvr.nav.ion_gal[i];
	for (i=0;i<8;i++) ion_qzs[i]=rtksvr.nav.ion_qzs[i];
	for (i=0;i<8;i++) ion_cmp[i]=rtksvr.nav.ion_cmp[i];
	for (i=0;i<8;i++) ion_irn[i]=rtksvr.nav.ion_irn[i];
	rtksvrunlock(&rtksvr);
	
	Label->Caption="";
	
	Tbl->RowCount=21;
	i=1;
	
	time2str(timeget(),tstr,3);
	Tbl->Cells[0][i  ]="CPU Time (UTC)";
	Tbl->Cells[1][i++]=s.sprintf("%s",tstr);
	
	if (time.time!=0) time2str(gpst2utc(time),tstr,3); else strcpy(tstr,"-");
	Tbl->Cells[0][i  ]="Receiver Time (UTC)";
	Tbl->Cells[1][i++]=s.sprintf("%s",tstr);
	
	if (time.time!=0) time2str(time,tstr,3); else strcpy(tstr,"-");
	Tbl->Cells[0][i  ]="Receiver Time (GPST)";
	Tbl->Cells[1][i++]=s.sprintf("%s",tstr);
	
	if (time.time!=0) tow=time2gpst(time,&week);
	Tbl->Cells[0][i  ]="GPS Week/Time (s)";
	Tbl->Cells[1][i++]=s.sprintf("%d, %.3f",week,tow);
	
	Tbl->Cells[0][i  ]="Leap Seconds dt_LS(s), WN_LSF,DN, dt_LSF(s)";
	Tbl->Cells[1][i++]=s.sprintf("%.0f, %.0f, %.0f, %.0f",utc_gps[4],utc_gps[5],
	                             utc_gps[6],utc_gps[7]);
	
	Tbl->Cells[0][i  ]="GPST-UTC Ref Week, Time(s), A0(ns), A1(ns/s)";
	Tbl->Cells[1][i++]=s.sprintf("%.0f, %.0f, %.3f, %.5E",utc_gps[3],utc_gps[2],
	                             utc_gps[0]*1E9,utc_gps[1]*1E9);
	
	Tbl->Cells[0][i  ]="GLOT-UTC Tau_C, Tau_GPS(ns)";
	Tbl->Cells[1][i++]=s.sprintf("%.9f, %.3f",utc_glo[0],utc_glo[1]*1E9);
	
	Tbl->Cells[0][i  ]="GST-UTC Ref Week, Time(s), A0(ns), A1(ns/s)";
	Tbl->Cells[1][i++]=s.sprintf("%.0f, %.0f, %.3f, %.5E",utc_gal[3],utc_gal[2],
	                             utc_gal[0]*1E9,utc_gal[1]*1E9);
	
	Tbl->Cells[0][i  ]="QZSST-UTC Ref Week, Time(s), A0(ns), A1(ns/s)";
	Tbl->Cells[1][i++]=s.sprintf("%.0f, %.0f, %.3f, %.5E",utc_qzs[3],utc_qzs[2],
	                             utc_qzs[0]*1E9,utc_qzs[1]*1E9);
	
	Tbl->Cells[0][i  ]="BDST-UTC Ref Week, Time(s), A0(ns), A1(ns/s)";
	Tbl->Cells[1][i++]=s.sprintf("%.0f, %.0f, %.3f, %.5E",utc_cmp[3],utc_cmp[2],
	                             utc_cmp[0]*1E9,utc_cmp[1]*1E9);
	
	Tbl->Cells[0][i  ]="IRNT-UTC Ref Week,Time(s), A0(ns), A1(ns/s), A2(ns/s2)";
	Tbl->Cells[1][i++]=s.sprintf("%.0f, %.0f, %.3f, %.5E, %.5E",utc_irn[3],
	                             utc_irn[2],utc_irn[0]*1E9,utc_irn[1]*1E9,
								 utc_irn[8]*1E9);
	
	Tbl->Cells[0][i  ]="GPS Iono Parameters Alpha0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_gps[0],ion_gps[1],ion_gps[2],ion_gps[3]);
	
	Tbl->Cells[0][i  ]="GPS Iono Parameters Beta0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_gps[4],ion_gps[5],ion_gps[6],ion_gps[7]);
	
	Tbl->Cells[0][i  ]="Galileo Iono Parameters 0-2";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E",ion_gal[0],ion_gal[1],
	                             ion_gal[2]);
	
	Tbl->Cells[0][i  ]="QZSS Iono Parameters Alpha0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_qzs[0],ion_qzs[1],ion_qzs[2],ion_qzs[3]);
	
	Tbl->Cells[0][i  ]="QZSS Iono Parameters Beta0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_qzs[4],ion_qzs[5],ion_qzs[6],ion_qzs[7]);
	
	Tbl->Cells[0][i  ]="BDS Iono Parameters Alpha0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_cmp[0],ion_cmp[1],ion_cmp[2],ion_cmp[3]);
	
	Tbl->Cells[0][i  ]="BDS Iono Parameters Beta0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_cmp[4],ion_cmp[5],ion_cmp[6],ion_cmp[7]);
	
	Tbl->Cells[0][i  ]="NavIC Iono Parameters Alpha0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_irn[0],ion_irn[1],ion_irn[2],ion_irn[3]);
	
	Tbl->Cells[0][i  ]="NavIC Iono Parameters Beta0-3";
	Tbl->Cells[1][i++]=s.sprintf("%.5E, %.5E, %.5E, %.5E",
	                             ion_irn[4],ion_irn[5],ion_irn[6],ion_irn[7]);
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetStr(void)
{
	AnsiString label[]={
		"STR","Stream","Type","Format","Mode","State","Input(bytes)","Input(bps)",
		"Output(bytes)","Output(bps)","Path","Message"
	};
	int i,width[]={25,95,70,80,35,35,70,70,70,70,220,220};
	
	Tbl->ColCount=12;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowStr(void)
{
	AnsiString ch[]={
		"Input Rover","Input Base","Input Correction","Output Solution 1",
		"Output Solution 2","Log Rover","Log Base","Log Correction",
		"Monitor"
	};
	AnsiString type[]={
		"-","Serial","File","TCP Server","TCP Client","NTRIP Server",
		"NTRIP Client","FTP","HTTP","NTRIP Caster","UDP Server",
		"UDP Client",""
	};
	AnsiString outformat[]={
		"Lat/Lon/Height","X/Y/Z-ECEF","E/N/U-Baseline","NMEA-0183",
		"Solution stats","GSI F1/F2"
	};
	AnsiString state[]={"Error","-","OK"};
	AnsiString s,mode,form;
	stream_t stream[9];
	int i,j,format[9]={0};
	char path[MAXSTRPATH]="",*p,*q,*pp;
	
	rtksvrlock(&rtksvr); // lock
	for (i=0;i<8;i++) stream[i]=rtksvr.stream[i];
	for (i=0;i<3;i++) format[i]=rtksvr.format[i];
	for (i=3;i<5;i++) format[i]=rtksvr.solopt[i-3].posf;
	stream[8]=monistr;
	format[8]=SOLF_LLH;
	rtksvrunlock(&rtksvr); // unlock
	
	Tbl->RowCount=10;
	Label->Caption="";
	for (i=0;i<9;i++) {
		j=0;
		Tbl->Cells[j++][i+1]=s.sprintf("(%d)",i+1);
		Tbl->Cells[j++][i+1]=ch[i];
		Tbl->Cells[j++][i+1]=type[stream[i].type];
		if (!stream[i].type) form="-";
		else if (i<3) form=formatstrs[format[i]];
		else if (i<5||i==8) form=outformat[format[i]];
		else form="-";
		Tbl->Cells[j++][i+1]=form;
		if (stream[i].mode&STR_MODE_R) mode="R"; else mode="";
		if (stream[i].mode&STR_MODE_W) mode=mode+(mode==""?"":"/")+"W";
		Tbl->Cells[j++][i+1]=mode;
		Tbl->Cells[j++][i+1]=state[stream[i].state+1];
		Tbl->Cells[j++][i+1]=s.sprintf("%d",stream[i].inb);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",stream[i].inr);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",stream[i].outb);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",stream[i].outr);
		strcpy(path,stream[i].path);
		pp=path;
		if ((p=strchr(path,'@'))) {
			for (q=p-1;q>=path;q--) if (*q==':') break;
			if (q>=path) for (q++;q<p;q++) *q='*';
		}
		if (stream[i].type==STR_TCPCLI||stream[i].type==STR_TCPSVR) {
			if ((p=strchr(path,'/'))) *p='\0';
			if ((p=strchr(path,'@'))) pp=p+1;
			if (stream[i].type==STR_TCPSVR) {
				if ((p=strchr(pp,':'))) pp=p+1; else pp=(char *)"";
			}
		}
		Tbl->Cells[j++][i+1]=pp;
		Tbl->Cells[j++][i+1]=stream[i].msg;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetSbsMsg(void)
{
	AnsiString label[]={
		"Trcv","PRN","STR","Type","Message","Contents"
	};
	int i,width[]={115,25,25,25,420,200};
	
	Tbl->ColCount=6;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowSbsMsg(void)
{
	AnsiString s;
	sbsmsg_t msg[MAXSBSMSG];
	const char *content[]={
		"For Testing","PRN Mask","Fast Corrections","Fast Corrections",
		"Fast Corrections","Fast Corrections","Integrity Information",
		"Fast Correction Degradation Factor","GEO Navigation Message",
		"Degradation Parameters","WAAS Network Time/UTC Offset Parameters",
		"GEO Satellite Almanacs","Ionospheric Grid Point Masks",
		"Mixed Fast Corrections/Long Term Satellite Error Corrections",
		"Long Term Satellite Error Corrections","Ionospheric Delay Corrections",
		"WAAS Service Messages","Clock-Ephemeris Covariance Matrix Message",
		"Internal Test Message","Null Message",
		"QZSS: DC Report (JMA)","QZSS: DC Report (Other)",
		"QZSS: Monitoring Station Info","QZSS: PRN Mask",
		"QZSS: Data Issue Number","QZSS: DGPS Correction",
		"QZSS: Satellite Health",
		""
	};
	const int id[]={0,1,2,3,4,5,6,7,9,10,12,17,18,24,25,26,27,28,62,63,
					43,44,47,48,49,50,51,-1};
	char str[256],*p;
	int i,j,k,n,type,prn;
	
	
	rtksvrlock(&rtksvr); // lock
	for (i=n=0;i<rtksvr.nsbs;i++) {
		msg[n++]=rtksvr.sbsmsg[i];
	}
	rtksvrunlock(&rtksvr); // unlock
	
	Tbl->RowCount=n<=0?2:n+1;
	Label->Caption="";
	for (i=0;i<n;i++) {
		j=0;
		prn=msg[i].prn;
		time2str(gpst2time(msg[i].week,msg[i].tow),str,0);
		Tbl->Cells[j++][i+1]=str;
		Tbl->Cells[j++][i+1]=s.sprintf("%d",prn);
		Tbl->Cells[j++][i+1]=s.sprintf("(%d)",msg[i].rcv);
		type=msg[i].msg[1]>>2;
		Tbl->Cells[j++][i+1]=s.sprintf("%d",type);
		p=str;
		for (k=0;k<29;k++) p+=sprintf(p,"%02X",msg[i].msg[k]);
		Tbl->Cells[j++][i+1]=str;
		for (k=0;id[k]>=0;k++) if (type==id[k]) break;
		Tbl->Cells[j++][i+1]=id[k]<0?"?":content[k];
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetSbsLong(void)
{
	AnsiString label[]={
		"SAT","Status","IODE","dX (m)","dY (m)","dZ (m)","dVX (m/s)",
		"dVY (m/s)","dVZ (m/s)","daf0 (ns)","daf1 (ns/s)","T0"
	};
	int i,width[]={25,30,30,55,55,55,55,55,55,55,55,115};
	
	Tbl->ColCount=12;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowSbsLong(void)
{
	AnsiString s;
	sbssat_t sbssat;
	sbssatp_t *satp;
	gtime_t time;
	int i,j,n,valid;
	char tstr[40],id[8];
	
	rtksvrlock(&rtksvr); // lock
	time=rtksvr.rtk.sol.time;
	sbssat=rtksvr.nav.sbssat;
	rtksvrunlock(&rtksvr); // unlock
	
	Label->Caption="";
	Tbl->RowCount=sbssat.nsat<=0?1:sbssat.nsat+1;
	Label->Caption=s.sprintf("IODP:%2d  System Latency:%2d s",
							 sbssat.iodp,sbssat.tlat);
	for (i=0;i<sbssat.nsat;i++) {
		j=0;
		satp=sbssat.sat+i;
		valid=timediff(time,satp->lcorr.t0)<=MAXSBSAGEL&&satp->lcorr.t0.time;
		satno2id(satp->sat,id);
		Tbl->Cells[j++][i+1]=id;
		Tbl->Cells[j++][i+1]=valid?"OK":"-";
		Tbl->Cells[j++][i+1]=s.sprintf("%d",satp->lcorr.iode);
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",satp->lcorr.dpos[0]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",satp->lcorr.dpos[1]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",satp->lcorr.dpos[2]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.4f",satp->lcorr.dvel[0]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.4f",satp->lcorr.dvel[1]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.4f",satp->lcorr.dvel[2]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",satp->lcorr.daf0*1E9);
		Tbl->Cells[j++][i+1]=s.sprintf("%.4f",satp->lcorr.daf1*1E9);
		if (satp->lcorr.t0.time) time2str(satp->lcorr.t0,tstr,0);
		else strcpy(tstr,"-");
		Tbl->Cells[j++][i+1]=tstr;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetSbsIono(void)
{
	AnsiString label[]={
		"IODI","Lat (deg)","Lon (deg)","GIVEI","Delay (m)","T0"
	};
	int i,width[]={30,50,50,30,60,115};
	
	Tbl->ColCount=6;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowSbsIono(void)
{
	AnsiString s,s0="-";
	sbsion_t sbsion[MAXBAND+1],*ion;
	char tstr[40];
	int i,j,k,n=0;
	
	rtksvrlock(&rtksvr); // lock
	for (i=0;i<=MAXBAND;i++) sbsion[i]=rtksvr.nav.sbsion[i];
	rtksvrunlock(&rtksvr); // unlock
	
	Label->Caption="";
	for (i=0;i<MAXBAND+1;i++) {
		ion=sbsion+i;
		for (j=0;j<ion->nigp;j++) {
			k=0;
			Tbl->Cells[k++][n+1]=s.sprintf("%d",ion->iodi);
			Tbl->Cells[k++][n+1]=s.sprintf("%d",ion->igp[j].lat);
			Tbl->Cells[k++][n+1]=s.sprintf("%d",ion->igp[j].lon);
			Tbl->Cells[k++][n+1]=ion->igp[j].give?s.sprintf("%d",ion->igp[j].give-1):s0;
			Tbl->Cells[k++][n+1]=s.sprintf("%.3f",ion->igp[j].delay);
			if (ion->igp[j].t0.time) time2str(ion->igp[j].t0,tstr,0);
			else strcpy(tstr,"-");
			Tbl->Cells[k++][n+1]=tstr;
			n++;
		}
	}
	Tbl->RowCount=n<=0?2:n+1;
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetSbsFast(void)
{
	AnsiString label[]={
		"SAT","Status","PRC (m)","RRC (m)","IODF","UDREI","AI","Tof"
	};
	int i,width[]={25,30,60,60,30,30,30,115};
	
	Tbl->ColCount=8;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowSbsFast(void)
{
	AnsiString s,s0="-";
	sbssat_t sbssat;
	sbssatp_t *satp;
	gtime_t time;
	int i,j,n,valid;
	char tstr[40],id[8];
	
	rtksvrlock(&rtksvr); // lock
	time=rtksvr.rtk.sol.time;
	sbssat=rtksvr.nav.sbssat;
	rtksvrunlock(&rtksvr); // unlock
	
	Label->Caption="";
	Tbl->RowCount=sbssat.nsat<=0?1:sbssat.nsat+1;
	//Label->Caption=s.sprintf("IODP:%2d  System Latency:%2d s",sbssat.iodp,sbssat.tlat);
	for (i=0;i<sbssat.nsat;i++) {
		j=0;
		satp=sbssat.sat+i;
		valid=fabs(timediff(time,satp->fcorr.t0))<=MAXSBSAGEF&&satp->fcorr.t0.time&&
			  0<=satp->fcorr.udre-1&&satp->fcorr.udre-1<14;
		satno2id(satp->sat,id);
		Tbl->Cells[j++][i+1]=id;
		Tbl->Cells[j++][i+1]=valid?"OK":"-";
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",satp->fcorr.prc);
		Tbl->Cells[j++][i+1]=s.sprintf("%.4f",satp->fcorr.rrc);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",satp->fcorr.iodf);
		Tbl->Cells[j++][i+1]=satp->fcorr.udre?s.sprintf("%d",satp->fcorr.udre-1):s0;
		Tbl->Cells[j++][i+1]=s.sprintf("%d",satp->fcorr.ai);
		if (satp->fcorr.t0.time) time2str(satp->fcorr.t0,tstr,0);
		else strcpy(tstr,"-");
		Tbl->Cells[j++][i+1]=tstr;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetRtcm(void)
{
	AnsiString label[]={"Parameter","Value"};
	int i,width[]={220,520};

	Tbl->ColCount=2;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowRtcm(void)
{
	AnsiString s;
	static rtcm_t rtcm;
	double pos[3]={0};
	int i=1,j,format;
	char tstr[40]="-",mstr1[1024]="",mstr2[1024]="",*p1=mstr1,*p2=mstr2;
	
	rtksvrlock(&rtksvr);
	format=rtksvr.format[Str1];
	rtcm=rtksvr.rtcm[Str1];
	rtksvrunlock(&rtksvr);
	
	if (rtcm.time.time) time2str(rtcm.time,tstr,3);
	
	for (j=1;j<100;j++) {
		if (rtcm.nmsg2[j]==0) continue;
        p1+=sprintf(p1,"%s%d (%d)",p1>mstr1?",":"",j,rtcm.nmsg2[j]);
	}
	if (rtcm.nmsg2[0]>0) {
		sprintf(p1,"%sother (%d)",p1>mstr1?",":"",rtcm.nmsg2[0]);
	}
	for (j=1;j<300;j++) {
		if (rtcm.nmsg3[j]==0) continue;
        p2+=sprintf(p2,"%s%d(%d)",p2>mstr2?",":"",j+1000,rtcm.nmsg3[j]);
	}
	for (j=300;j<399;j++) {
		if (rtcm.nmsg3[j]==0) continue;
        p2+=sprintf(p2,"%s%d(%d)",p2>mstr2?",":"",j+3770,rtcm.nmsg3[j]);
	}
	if (rtcm.nmsg3[0]>0) {
		sprintf(p2,"%sother(%d)",p2>mstr2?",":"",rtcm.nmsg3[0]);
	}
	Label->Caption="";
	
	Tbl->RowCount=16;
	
	Tbl->Cells[0][i  ]="Format";
	Tbl->Cells[1][i++]=format==STRFMT_RTCM2?"RTCM 2":"RTCM 3";
	
	Tbl->Cells[0][i  ]="Message Time";
	Tbl->Cells[1][i++]=tstr;
	
	Tbl->Cells[0][i  ]="Station ID";
	Tbl->Cells[1][i++]=s.sprintf("%d",rtcm.staid);
	
	Tbl->Cells[0][i  ]="Station Health";
	Tbl->Cells[1][i++]=s.sprintf("%d",rtcm.stah);
	
	Tbl->Cells[0][i  ]="Sequence No";
	Tbl->Cells[1][i++]=s.sprintf("%d",rtcm.seqno);
	
	Tbl->Cells[0][i  ]="RTCM Special Message";
	Tbl->Cells[1][i++]=rtcm.msg;
	
	Tbl->Cells[0][i  ]="Last Message";
	Tbl->Cells[1][i++]=rtcm.msgtype;
	
	Tbl->Cells[0][i  ]="# of RTCM Messages";
	Tbl->Cells[1][i++]=format==STRFMT_RTCM2?mstr1:mstr2;
	
	Tbl->Cells[0][i  ]="MSM Signals for GPS";
	Tbl->Cells[1][i++]=rtcm.msmtype[0];
	
	Tbl->Cells[0][i  ]="MSM Signals for GLONASS";
	Tbl->Cells[1][i++]=rtcm.msmtype[1];
	
	Tbl->Cells[0][i  ]="MSM Signals for Galileo";
	Tbl->Cells[1][i++]=rtcm.msmtype[2];
	
	Tbl->Cells[0][i  ]="MSM Signals for QZSS";
	Tbl->Cells[1][i++]=rtcm.msmtype[3];
	
	Tbl->Cells[0][i  ]="MSM Signals for SBAS";
	Tbl->Cells[1][i++]=rtcm.msmtype[4];
	
	Tbl->Cells[0][i  ]="MSM Signals for BDS";
	Tbl->Cells[1][i++]=rtcm.msmtype[5];
	
	Tbl->Cells[0][i  ]="MSM Signals for NavIC";
	Tbl->Cells[1][i++]=rtcm.msmtype[6];
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetRtcmDgps(void)
{
	AnsiString label[]={
		"SAT","Status","PRC (m)","RRC (m)","IOD","UDRE","T0"
	};
	int i,width[]={25,30,60,60,30,30,115};
	
	Tbl->ColCount=7;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowRtcmDgps(void)
{
	AnsiString s;
	gtime_t time;
	dgps_t dgps[MAXSAT];
	int i,j,valid;
	char tstr[40],id[8];
	
	rtksvrlock(&rtksvr);
	time=rtksvr.rtk.sol.time;
	for (i=0;i<MAXSAT;i++) dgps[i]=rtksvr.nav.dgps[i];
	rtksvrunlock(&rtksvr);
	
	Label->Caption="";
	Tbl->RowCount=MAXSAT+1;
	
	for (i=0;i<Tbl->RowCount;i++) {
		j=0;
		satno2id(i+1,id);
		valid=dgps[i].t0.time&&fabs(timediff(time,dgps[i].t0))<=1800.0;
		Tbl->Cells[j++][i+1]=id;
		Tbl->Cells[j++][i+1]=valid?"OK":"-";
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",dgps[i].prc);
		Tbl->Cells[j++][i+1]=s.sprintf("%.4f",dgps[i].rrc);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",dgps[i].iod);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",dgps[i].udre);
		if (dgps[i].t0.time) time2str(dgps[i].t0,tstr,0); else strcpy(tstr,"-");
		Tbl->Cells[j++][i+1]=tstr;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetRtcmSsr(void)
{
	AnsiString s,label[]={
		"SAT","Status","UDI(s)","UDHR(s)","IOD","URA","Datum","T0",
		"D0-A(m)","D0-C(m)","D0-R(m)","D1-A(mm/s)","D1-C(mm/s)","D1-R(mm/s)",
		"C0(m)","C1(mm/s)","C2(mm/s2)","C-HR(m)","Code Bias(m)",
		"Phase Bias(m)"
	};
	int i,width[]={
		25,30,30,30,30,25,15,115,50,50,50,50,50,50,50,50,50,50,180,180
	};
	char *code;

	Tbl->ColCount=20;
	Tbl->RowCount=2;
	for (i=0;i<20;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowRtcmSsr(void)
{
	AnsiString s;
	gtime_t time;
	ssr_t ssr[MAXSAT];
	int i,j,k,n,valid,sat[MAXSAT],sys=sys_tbl[SelSys2->ItemIndex+1];
	char tstr[40],id[8],buff[256]="",*p;

	rtksvrlock(&rtksvr);
	time=rtksvr.rtk.sol.time;
	for (i=n=0;i<MAXSAT;i++) {
		if (!(satsys(i+1,NULL)&sys)) continue;
		ssr[n]=rtksvr.rtcm[Str1].ssr[i];
		sat[n++]=i+1;
	}
	rtksvrunlock(&rtksvr);

	Label->Caption="";
	Tbl->RowCount=n+1;
	
	for (i=0;i<n;i++) {
		j=0;
		satno2id(sat[i],id);
		Tbl->Cells[j++][i+1]=id;
		valid=ssr[i].t0[0].time&&fabs(timediff(time,ssr[i].t0[0]))<=1800.0;
		Tbl->Cells[j++][i+1]=valid?"OK":"-";
		Tbl->Cells[j++][i+1]=s.sprintf("%.0f",ssr[i].udi[0]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.0f",ssr[i].udi[2]);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",ssr[i].iode);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",ssr[i].ura);
		Tbl->Cells[j++][i+1]=s.sprintf("%d",ssr[i].refd);
		if (ssr[i].t0[0].time) time2str(ssr[i].t0[0],tstr,0); else strcpy(tstr,"-");
		Tbl->Cells[j++][i+1]=tstr;
		for (k=0;k<3;k++) {
			Tbl->Cells[j++][i+1]=s.sprintf("%.3f",ssr[i].deph[k]);
		}
		for (k=0;k<3;k++) {
			Tbl->Cells[j++][i+1]=s.sprintf("%.3f",ssr[i].ddeph[k]*1E3);
		}
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",ssr[i].dclk[0]);
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",ssr[i].dclk[1]*1E3);
		Tbl->Cells[j++][i+1]=s.sprintf("%.5f",ssr[i].dclk[2]*1E3);
		Tbl->Cells[j++][i+1]=s.sprintf("%.3f",ssr[i].hrclk);
		buff[0]='\0';
		for (p=buff,k=0;k<MAXCODE;k++) {
			if (ssr[i].cbias[k]==0.0) continue;
			p+=sprintf(p,"%s:%.3f ",code2obs(k+1),ssr[i].cbias[k]);
		}
		Tbl->Cells[j++][i+1]=buff;
		buff[0]='\0';
		for (p=buff,k=0;k<MAXCODE;k++) {
			if (ssr[i].pbias[k]==0.0) continue;
			p+=sprintf(p,"%s:%.3f ",code2obs(k+1),ssr[i].pbias[k]);
		}
		Tbl->Cells[j++][i+1]=buff;
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::SetRefSta(void)
{
	AnsiString label[]={"Parameter","Value"};
	int i,width[]={220,520};

	Tbl->ColCount=2;
	Tbl->RowCount=2;
	for (i=0;i<Tbl->ColCount;i++) {
		Tbl->ColWidths[i]=width[i]*FontScale/96;
		Tbl->Cells[i][0]=label[i];
		Tbl->Cells[i][1]="";
	}
}
//---------------------------------------------------------------------------
void __fastcall TMonitorDialog::ShowRefSta(void)
{
	AnsiString s;
	gtime_t time;
	sta_t sta;
	double pos[3]={0};
	int i=1,format;
	char tstr[40]="-";
	
	rtksvrlock(&rtksvr);
	format=rtksvr.format[Str1];
	if (format==STRFMT_RTCM2||format==STRFMT_RTCM3) {
		time=rtksvr.rtcm[Str1].time;
		sta=rtksvr.rtcm[Str1].sta;
	}
	else {
		time=rtksvr.raw[Str1].time;
		sta=rtksvr.raw[Str1].sta;
	}
	rtksvrunlock(&rtksvr);
	
	Label->Caption="";
	
	Tbl->RowCount=17;
	
	Tbl->Cells[0][i  ]="Format";
	Tbl->Cells[1][i++]=formatstrs[format];
	
	if (time.time) time2str(time,tstr,3);
	Tbl->Cells[0][i  ]="Message Time";
	Tbl->Cells[1][i++]=tstr;
	
	Tbl->Cells[0][i  ]="Station Pos X/Y/Z (m)";
	Tbl->Cells[1][i++]=s.sprintf("%.3f,%.3f,%.3f",sta.pos[0],sta.pos[1],sta.pos[2]);
	
	if (norm(sta.pos,3)>0.0) ecef2pos(sta.pos,pos);
	Tbl->Cells[0][i  ]="Station Lat/Lon/Height (deg,m)";
	Tbl->Cells[1][i++]=s.sprintf("%.8f,%.8f,%.3f",pos[0]*R2D,pos[1]*R2D,pos[2]);
	
	Tbl->Cells[0][i  ]="ITRF Realization Year";
	Tbl->Cells[1][i++]=s.sprintf("%d",sta.itrf);
	
	Tbl->Cells[0][i  ]="Antenna Delta Type";
	Tbl->Cells[1][i++]=sta.deltype?"X/Y/Z":"E/N/U";
	
	Tbl->Cells[0][i  ]="Antenna Delta (m)";
	Tbl->Cells[1][i++]=s.sprintf("%.3f,%.3f,%.3f",sta.del[0],sta.del[1],sta.del[2]);
	
	Tbl->Cells[0][i  ]="Antenna Height (m)";
	Tbl->Cells[1][i++]=s.sprintf("%.3f",sta.hgt);
	
	Tbl->Cells[0][i  ]="Antenna Descriptor";
	Tbl->Cells[1][i++]=sta.antdes;
	
	Tbl->Cells[0][i  ]="Antenna Setup Id";
	Tbl->Cells[1][i++]=s.sprintf("%d",sta.antsetup);
	
	Tbl->Cells[0][i  ]="Antenna Serial No";
	Tbl->Cells[1][i++]=sta.antsno;
	
	Tbl->Cells[0][i  ]="Receiver Type Descriptor";
	Tbl->Cells[1][i++]=sta.rectype;
	
	Tbl->Cells[0][i  ]="Receiver Firmware Version";
	Tbl->Cells[1][i++]=sta.recver;
	
	Tbl->Cells[0][i  ]="Receiver Serial No";
	Tbl->Cells[1][i++]=sta.recsno;
	
	Tbl->Cells[0][i  ]="GLONASS Code-Phase Alignment";
	Tbl->Cells[1][i++]=sta.glo_cp_align?"Aligned":"Not aligned";
	
	Tbl->Cells[0][i  ]="GLONASS Code-Phase Bias C1/P1/C2/P2 (m)";
	Tbl->Cells[1][i++]=s.sprintf("%.2f, %.2f, %.2f, %.2f",sta.glo_cp_bias[0],
	                             sta.glo_cp_bias[1],sta.glo_cp_bias[2],
								 sta.glo_cp_bias[3]);
}
//---------------------------------------------------------------------------


