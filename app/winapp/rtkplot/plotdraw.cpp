//---------------------------------------------------------------------------
// plotdraw.c: rtkplot draw functions
//---------------------------------------------------------------------------
#include <vcl.h>
#include "rtklib.h"
#include "plotmain.h"
#include "graph.h"
#include "refdlg.h"
#include "mapview.h"

#define MS_FONT     "Consolas"      // monospace font name
#define CHARGTE     "\342\211\245"  // grater equal charactor (UTF-8)
#define CHARMUL     "\303\227"      // multiply charactor (UTF-8)
#define COL_ELMASK  clRed
#define ATAN2(x,y)  ((x)*(x)+(y)*(y)>1E-12?atan2(x,y):0.0)

// update plot --------------------------------------------------------------
void __fastcall TPlot::UpdatePlot(void)
{
    trace(3,"UpdatePlot\n");
    
    UpdateInfo();
    Refresh();
}
// refresh plot -------------------------------------------------------------
void __fastcall TPlot::Refresh(void)
{
    trace(3,"Refresh\n");
    
    Flush=1;
    Disp->Invalidate();
}
// draw plot ----------------------------------------------------------------
void __fastcall TPlot::UpdateDisp(void)
{
    TCanvas *c=Disp->Canvas;
    TRect r=Disp->ClientRect;
    int level=Drag?0:1;
    
    trace(3,"UpdateDisp\n");
    
    if (Flush) {
        c->Pen  ->Color=CColor[0];
        c->Brush->Color=CColor[0];
        c->Pen  ->Style=psSolid;
        c->Brush->Style=bsSolid;
        c->FillRect(r);
        
        switch (PlotType) {
            case  PLOT_TRK : DrawTrk (level);   break;
            case  PLOT_SOLP: DrawSol (level,0); break;
            case  PLOT_SOLV: DrawSol (level,1); break;
            case  PLOT_SOLA: DrawSol (level,2); break;
            case  PLOT_NSAT: DrawNsat(level);   break;
            case  PLOT_OBS : DrawObs (level);   break;
            case  PLOT_SKY : DrawSky (level);   break;
            case  PLOT_SSKY : DrawSolSky (level);   break;
            case  PLOT_DOP : DrawDop (level);   break;
            case  PLOT_SDOP : DrawSolDop (level);   break;
            case  PLOT_RES : DrawRes (level);   break;
            case  PLOT_RESE: DrawResE(level);   break;
            case  PLOT_SNR : DrawSnr (level);   break;
            case  PLOT_SNRE: DrawSnrE(level);   break;
            case  PLOT_MPS : DrawMpS (level);   break;
        }
        Buff->SetSize(Disp->ClientWidth,Disp->ClientHeight);
        Buff->Canvas->CopyRect(r,c,r);
    }
    else {
        c->CopyRect(r,Buff->Canvas,r);
    }
    Flush=0;
}
// draw track-plot ----------------------------------------------------------
void __fastcall TPlot::DrawTrk(int level)
{
    UTF8String header,s;
    TIMEPOS *pos,*pos1,*pos2,*vel;
    gtime_t time1={0},time2={0};
    sol_t *sol;
    TPoint p1,p2;
    TColor color;
    double xt,yt,sx,sy,opos[3],pnt[3],rr[3],enu[3]={0},cent[3];
    int i,j,index,sel=!BtnSol1->Down&&BtnSol2->Down?1:0,p=0;
    
    trace(3,"DrawTrk: level=%d\n",level);
    
    if (BtnShowTrack->Down&&BtnFixCent->Down) {
        if (!BtnSol12->Down) {
            pos=SolToPos(SolData+sel,SolIndex[sel],0,0);
            if (pos->n>0) GraphT->SetCent(pos->x[0],pos->y[0]);
            delete pos;
        }
        else {
            pos1=SolToPos(SolData  ,SolIndex[0],0,0);
            pos2=SolToPos(SolData+1,SolIndex[1],0,0);
            pos=pos1->diff(pos2,0);
            if (pos->n>0) GraphT->SetCent(pos->x[0],pos->y[0]);
            delete pos;
            delete pos1;
            delete pos2;
        }
    }
    if (!BtnSol12->Down&&BtnShowImg->Down) { // image
        DrawTrkImage(level);
    }
    if (BtnShowMap->Down) { // map
        DrawTrkMap(level);
    }
    if (BtnShowGrid->Down) { // grid
        if (level) { // center +
            GraphT->GetPos(p1,p2);
            p1.x=(p1.x+p2.x)/2;
            p1.y=(p1.y+p2.y)/2;
            DrawMark(GraphT,p1,5,CColor[1],20,0);
        }
        if (ShowGLabel>=3) { // circles
            GraphT->XLPos=7; GraphT->YLPos=7;
            GraphT->DrawCircles(ShowGLabel==4);
        }
        else if (ShowGLabel>=1) { // grid
            GraphT->XLPos=2; GraphT->YLPos=4;
            GraphT->DrawAxis(ShowLabel,ShowGLabel==2);
        }
    }
    if (norm(OPos,3)>0.0) {
        ecef2pos(OPos,opos);
        header="ORI="+LatLonStr(opos,9);
        header+=s.sprintf(" %.4fm",opos[2]);
    }
    if (BtnSol1->Down) {
        pos=SolToPos(SolData,-1,QFlag->ItemIndex,0);
        DrawTrkPnt(pos,level,0);
        if (BtnShowMap->Down&&norm(SolData[0].rb,3)>1E-3) {
            DrawTrkPos(SolData[0].rb,0,8,CColor[2],"Base Station 1");
        }
        DrawTrkStat(pos,header,p++);
        header="";
        delete pos;
    }
    if (BtnSol2->Down) {
        pos=SolToPos(SolData+1,-1,QFlag->ItemIndex,0);
        DrawTrkPnt(pos,level,1);
        if (BtnShowMap->Down&&norm(SolData[1].rb,3)>1E-3) {
            DrawTrkPos(SolData[1].rb,0,8,CColor[2],"Base Station 2");
        }
        DrawTrkStat(pos,header,p++);
        delete pos;
    }
    if (BtnSol12->Down) {
        pos1=SolToPos(SolData  ,-1,0,0);
        pos2=SolToPos(SolData+1,-1,0,0);
        pos=pos1->diff(pos2,QFlag->ItemIndex);
        DrawTrkPnt(pos,level,0);
        DrawTrkStat(pos,"",p++);
        delete pos;
        delete pos1;
        delete pos2;
    }
    if (BtnShowTrack->Down&&BtnSol1->Down) {
        
        pos=SolToPos(SolData,SolIndex[0],0,0);
        
        if ((sol=getsol(SolData,SolIndex[0]))) time1=sol->time;
        
        if (pos->n) {
            pos->n=1;
            DrawTrkError(pos,0);
            GraphT->ToPoint(pos->x[0],pos->y[0],p1);
            GraphT->DrawMark(p1,0,CColor[0],MarkSize*2+12,0);
            GraphT->DrawMark(p1,1,CColor[2],MarkSize*2+10,0);
            GraphT->DrawMark(p1,5,CColor[2],MarkSize*2+14,0);
            GraphT->DrawMark(p1,0,CColor[2],MarkSize*2+6,0);
            GraphT->DrawMark(p1,0,MColor[0][pos->q[0]],MarkSize*2+4,0);
            if (BtnSol2->Down) {
                p1.x+=MarkSize+8;
                DrawLabel(GraphT,p1,"1",1,0);
            }
        }
        delete pos;
    }
    if (BtnShowTrack->Down&&BtnSol2->Down) {
        
        pos=SolToPos(SolData+1,SolIndex[1],0,0);
        
        if ((sol=getsol(SolData+1,SolIndex[1]))) time2=sol->time;
        
        if (pos->n>0&&(time1.time==0||fabs(timediff(time1,time2))<DTTOL*2.0)) {
            pos->n=1;
            DrawTrkError(pos,1);
            GraphT->ToPoint(pos->x[0],pos->y[0],p1);
            GraphT->DrawMark(p1,0,CColor[0],MarkSize*2+12,0);
            GraphT->DrawMark(p1,1,CColor[1],MarkSize*2+10,0);
            GraphT->DrawMark(p1,5,CColor[1],MarkSize*2+14,0);
            GraphT->DrawMark(p1,0,CColor[2],MarkSize*2+6,0);
            GraphT->DrawMark(p1,0,MColor[1][pos->q[0]],MarkSize*2+4,0);
            if (BtnSol1->Down) {
                p1.x+=MarkSize+8;
                DrawLabel(GraphT,p1,"2",1,0);
            }
        }
        delete pos;
    }
    if (BtnShowTrack->Down&&BtnSol12->Down) {
        
        pos1=SolToPos(SolData  ,SolIndex[0],0,0);
        pos2=SolToPos(SolData+1,SolIndex[1],0,0);
        pos=pos1->diff(pos2,0);
        
        if (pos->n>0) {
            pos->n=1;
            DrawTrkError(pos,1);
            GraphT->ToPoint(pos->x[0],pos->y[0],p1);
            GraphT->DrawMark(p1,0,CColor[0],MarkSize*2+12,0);
            GraphT->DrawMark(p1,1,CColor[2],MarkSize*2+10,0);
            GraphT->DrawMark(p1,5,CColor[2],MarkSize*2+14,0);
            GraphT->DrawMark(p1,0,CColor[2],MarkSize*2+6,0);
            GraphT->DrawMark(p1,0,MColor[0][pos->q[0]],MarkSize*2+4,0);
        }
        delete pos;
        delete pos1;
        delete pos2;
    }
    if (level&&BtnShowMap->Down) {
        for (i=0;i<NWayPnt;i++) {
            if (i==SelWayPnt) continue;
            pnt[0]=PntPos[i][0]*D2R;
            pnt[1]=PntPos[i][1]*D2R;
            pnt[2]=PntPos[i][2];
            pos2ecef(pnt,rr);
            AnsiString str=PntName[i];
            DrawTrkPos(rr,0,8,CColor[2],str);
        }
        if (SelWayPnt>=0) {
            pnt[0]=PntPos[SelWayPnt][0]*D2R;
            pnt[1]=PntPos[SelWayPnt][1]*D2R;
            pnt[2]=PntPos[SelWayPnt][2];
            pos2ecef(pnt,rr);
            AnsiString str=PntName[SelWayPnt];
            DrawTrkPos(rr,0,16,clRed,str);
        }
    }
    if (ShowCompass) {
        GraphT->GetPos(p1,p2);
        p1.x+=SIZE_COMP/2+25;
        p1.y+=SIZE_COMP/2+35;
        DrawMark(GraphT,p1,13,CColor[2],SIZE_COMP,0);
    }
    if (ShowArrow&&BtnShowTrack->Down) {
        vel=SolToPos(SolData+sel,SolIndex[sel],0,1);
        DrawTrkVel(vel);
        delete vel;
    }
    if (ShowScale) {
        UTF8String label;
        
        GraphT->GetPos(p1,p2);
        GraphT->GetTick(xt,yt);
        GraphT->GetScale(sx,sy);
        p2.x-=70;
        p2.y-=25;
        DrawMark(GraphT,p2,11,CColor[2],(int)(xt/sx+0.5),0);
        p2.y-=3;
        if      (xt<0.0099) label.sprintf("%.0f mm",xt*1000.0);
        else if (xt<0.999 ) label.sprintf("%.0f cm",xt*100.0);
        else if (xt<999.0 ) label.sprintf("%.0f m" ,xt);
        else                label.sprintf("%.0f km",xt/1000.0);
        DrawLabel(GraphT,p2,label,0,1);
    }
    if (!level) { // center +
        GraphT->GetCent(xt,yt);
        GraphT->ToPoint(xt,yt,p1);
        DrawMark(GraphT,p1,5,CColor[2],20,0);
    }
    // update map view center
    if (level) {
        if (norm(OPos,3)>0.0) {
            GraphT->GetCent(xt,yt);
            GraphT->ToPoint(xt,yt,p1);
            GraphT->ToPos(p1,enu[0],enu[1]);
            ecef2pos(OPos,opos);
            enu2ecef(opos,enu,rr);
            for (i=0;i<3;i++) rr[i]+=OPos[i];
            ecef2pos(rr,cent);
            MapView->SetCent(cent[0]*R2D,cent[1]*R2D);
        }
        Refresh_MapView();
    }
}
// draw map-image on track-plot ---------------------------------------------
void __fastcall TPlot::DrawTrkImage(int level)
{
    gtime_t time={0};
    TCanvas *c=Disp->Canvas;
    TPoint p1,p2;
    double pos[3]={0},rr[3],xyz[3]={0},x1,x2,y1,y2;
    
    trace(3,"DrawTrkImage: level=%d\n",level);
    
    pos[0]=MapLat*D2R;
    pos[1]=MapLon*D2R;
    pos2ecef(pos,rr);
    if (norm(OPos,3)>0.0) {
        PosToXyz(time,rr,0,xyz);
    }
    x1=xyz[0]-MapSize[0]*0.5*MapScaleX;
    x2=xyz[0]+MapSize[0]*0.5*MapScaleX;
    y1=xyz[1]-MapSize[1]*0.5*(MapScaleEq?MapScaleX:MapScaleY);
    y2=xyz[1]+MapSize[1]*0.5*(MapScaleEq?MapScaleX:MapScaleY);
    
    GraphT->ToPoint(x1,y2,p1);
    GraphT->ToPoint(x2,y1,p2);
    TRect r(p1,p2);
    c->StretchDraw(r,MapImage);
}
// check in boundary --------------------------------------------------------
#define P_IN_B(pos,bound) \
    (pos[0]>=bound[0]&&pos[0]<=bound[1]&&pos[1]>=bound[2]&&pos[1]<=bound[3])

#define B_IN_B(bound1,bound2) \
    (bound1[0]<=bound2[1]&&bound1[1]>=bound2[0]&&bound1[2]<=bound2[3]&&bound1[3]>=bound2[2])

// draw gis-map on track-plot -----------------------------------------------
void __fastcall TPlot::DrawTrkMap(int level)
{
    gisd_t *data;
    gis_pnt_t *pnt;
    gis_poly_t *poly;
    gis_polygon_t *polygon;
    gtime_t time={0};
    TColor color;
    TPoint *p,p1;
    double xyz[3],S,xl[2],yl[2],enu[8][3]={{0}},opos[3],pos[3],rr[3];
    double bound[4]={PI/2.0,-PI/2.0,PI,-PI};
    int i,j,n,m;
    
    trace(3,"DrawTrkMap: level=%d\n",level);
    
    // get map boundary
    GraphT->GetLim(xl,yl);
    enu[0][0]=xl[0]; enu[0][1]=yl[0];
    enu[1][0]=xl[1]; enu[1][1]=yl[0];
    enu[2][0]=xl[0]; enu[2][1]=yl[1];
    enu[3][0]=xl[1]; enu[3][1]=yl[1];
    enu[4][0]=(xl[0]+xl[1])/2.0; enu[4][1]=yl[0];
    enu[5][0]=(xl[0]+xl[1])/2.0; enu[5][1]=yl[1];
    enu[6][0]=xl[0]; enu[6][1]=(yl[0]+yl[1])/2.0;
    enu[7][0]=xl[1]; enu[7][1]=(yl[0]+yl[1])/2.0;
    ecef2pos(OPos,opos);
    for (i=0;i<8;i++) {
        if (norm(enu[i],2)>=1000000.0) {
            bound[0]=-PI/2.0;
            bound[1]= PI/2.0;
            bound[2]=-PI;
            bound[3]= PI;
            break;
        }
        enu2ecef(opos,enu[i],rr);
        for (j=0;j<3;j++) rr[j]+=OPos[j];
        ecef2pos(rr,pos);
        if (pos[0]<bound[0]) bound[0]=pos[0]; // min lat
        if (pos[0]>bound[1]) bound[1]=pos[0]; // max lat
        if (pos[1]<bound[2]) bound[2]=pos[1]; // min lon
        if (pos[1]>bound[3]) bound[3]=pos[1]; // max lon
    }
    for (i=MAXMAPLAYER-1;i>=0;i--) {
        if (!Gis.flag[i]) continue;
        
        for (data=Gis.data[i];data;data=data->next) {
            if (data->type==1) { // point
                pnt=(gis_pnt_t *)data->data;
                if (!P_IN_B(pnt->pos,bound)) continue;
                PosToXyz(time,pnt->pos,0,xyz);
                if (xyz[2]<-RE_WGS84) continue;
                GraphT->ToPoint(xyz[0],xyz[1],p1);
                DrawMark(GraphT,p1,1,CColor[2],6,0);
                DrawMark(GraphT,p1,0,CColor[2],2,0);
            }
            else if (level&&data->type==2) { // polyline
                poly=(gis_poly_t *)data->data;
                if ((n=poly->npnt)<=0||!B_IN_B(poly->bound,bound)) {
                    continue;
                }
                p=new TPoint [n];
                for (j=m=0;j<n;j++) {
                    PosToXyz(time,poly->pos+j*3,0,xyz);
                    if (xyz[2]<-RE_WGS84) {
                        if (m>1) {
                            GraphT->DrawPoly(p,m,MapColor[i],0);
                            m=0;
                        }
                        continue;
                    }
                    GraphT->ToPoint(xyz[0],xyz[1],p1);
                    if (m==0||p1.x!=p[m-1].x||p1.y!=p[m-1].y) {
                        p[m++]=p1;
                    }
                }
                GraphT->DrawPoly(p,m,MapColor[i],0);
                delete [] p;
            }
            else if (level&&data->type==3) { // polygon
                polygon=(gis_polygon_t *)data->data;
                if ((n=polygon->npnt)<=0||!B_IN_B(polygon->bound,bound)) {
                    continue;
                }
                p=new TPoint [n];
                for (j=m=0;j<n;j++) {
                    PosToXyz(time,polygon->pos+j*3,0,xyz);
                    if (xyz[2]<-RE_WGS84) {
                        continue;
                    }
                    GraphT->ToPoint(xyz[0],xyz[1],p1);
                    if (m==0||p1.x!=p[m-1].x||p1.y!=p[m-1].y) {
                        p[m++]=p1;
                    }
                }
                // judge hole
                for (j=0,S=0.0;j<m-1;j++) {
                    S+=(double)p[j].x*p[j+1].y-(double)p[j+1].x*p[j].y;
                }
                color=S<0.0?CColor[0]:MapColorF[i];
                GraphT->DrawPatch(p,m,MapColor[i],color,0);
                delete [] p;
            }
        }
    }
}
// draw track-points on track-plot ------------------------------------------
void __fastcall TPlot::DrawTrkPnt(const TIMEPOS *pos, int level, int style)
{
    TColor *color;
    int i;
    
    trace(3,"DrawTrkPnt: level=%d style=%d\n",level,style);
    
    if (level) DrawTrkArrow(pos);
    
    if (level&&PlotStyle<=1&&!BtnShowTrack->Down) { // error circle
        DrawTrkError(pos,style);
    }
    if (!(PlotStyle%2)) {
        GraphT->DrawPoly(pos->x,pos->y,pos->n,CColor[3],style);
    }
    if (level&&PlotStyle<2) {
        color=new TColor [pos->n];
        if (BtnShowImg->Down) {
            for (i=0;i<pos->n;i++) color[i]=CColor[0];
            GraphT->DrawMarks(pos->x,pos->y,color,pos->n,0,MarkSize+2,0);
        }
        for (i=0;i<pos->n;i++) color[i]=MColor[style][pos->q[i]];
        GraphT->DrawMarks(pos->x,pos->y,color,pos->n,0,MarkSize,0);
        delete [] color;
    }
}
// draw point with label on track-plot --------------------------------------
void __fastcall TPlot::DrawTrkPos(const double *rr, int type, int siz,
                                  TColor color, UTF8String label)
{
    gtime_t time={0};
    TPoint p1;
    double xyz[3],xs,ys;
    
    trace(3,"DrawTrkPos: type=%d rr=%.3f %.3f %.3f\n",type,rr[0],rr[1],rr[2]);
    
    if (norm(rr,3)>0.0) {
        GraphT->GetScale(xs,ys);
        PosToXyz(time,rr,type,xyz);
        GraphT->ToPoint(xyz[0],xyz[1],p1);
        DrawMark(GraphT,p1,5,color,siz+6,0);
        DrawMark(GraphT,p1,1,color,siz  ,0);
        DrawMark(GraphT,p1,1,color,siz-6,0);
        p1.y+=10;
        DrawLabel(GraphT,p1,label,0,2);
    }
}
// draw statistics on track-plot --------------------------------------------
void __fastcall TPlot::DrawTrkStat(const TIMEPOS *pos, UTF8String header, int p)
{
    UTF8String s[6];
    TPoint p1,p2;
    double *d,ave[4],std[4],rms[4],opos[3];
    int i,n=0,fonth=(int)(Disp->Font->Size*1.5);
    
    trace(3,"DrawTrkStat: p=%d\n",p);
    
    if (!ShowStats) return;
    
    if (p==0&&header!="") s[n++]=header;
    
    if (pos->n>0) {
        d=new double[pos->n];
        for (i=0;i<pos->n;i++) {
            d[i]=SQRT(SQR(pos->x[i])+SQR(pos->y[i]));
        }
        CalcStats(pos->x,pos->n,0.0,ave[0],std[0],rms[0]);
        CalcStats(pos->y,pos->n,0.0,ave[1],std[1],rms[1]);
        CalcStats(pos->z,pos->n,0.0,ave[2],std[2],rms[2]);
        CalcStats(d     ,pos->n,0.0,ave[3],std[3],rms[3]);
        s[n++].sprintf("AVE=E:%7.4fm N:%7.4fm U:%7.4fm",ave[0],ave[1],ave[2]);
        s[n++].sprintf("STD=E:%7.4fm N:%7.4fm U:%7.4fm",std[0],std[1],std[2]);
        s[n++].sprintf("RMS=E:%7.4fm N:%7.4fm U:%7.4fm 2D:%7.4fm",
                       rms[0],rms[1],rms[2],2.0*rms[3]);
        delete [] d;
    }
    GraphT->GetPos(p1,p2);
    p1.x=p2.x-10;
    p1.y+=8+fonth*4*p;
    for (i=0;i<n;i++,p1.y+=fonth) {
        DrawLabel(GraphT,p1,s[i],2,2);
    }
}
// draw error-circle on track-plot ------------------------------------------
void __fastcall TPlot::DrawTrkError(const TIMEPOS *pos, int style)
{
    const double sint[36]={
         0.0000, 0.1736, 0.3420, 0.5000, 0.6428, 0.7660, 0.8660, 0.9397, 0.9848,
         1.0000, 0.9848, 0.9397, 0.8660, 0.7660, 0.6428, 0.5000, 0.3420, 0.1736,
         0.0000,-0.1736,-0.3420,-0.5000,-0.6428,-0.7660,-0.8660,-0.9397,-0.9848,
        -1.0000,-0.9848,-0.9397,-0.8660,-0.7660,-0.6428,-0.5000,-0.3420,-0.1736
    };
    double xc[37],yc[37],a,b,s,c;
    int i,j;
    
    trace(3,"DrawTrkError: style=%d\n",style);
    
    if (!ShowErr) return;
    
    for (i=0;i<pos->n;i++) {
        if (pos->xs[i]<=0.0||pos->ys[i]<=0.0) continue;
        
        a=pos->xys[i]/SQRT(pos->xs[i]);
        
        if ((b=pos->ys[i]-a*a)>=0.0) b=SQRT(b); else continue;
        
        for (j=0;j<37;j++) {
            s=sint[j%36];
            c=sint[(45-j)%36];
            xc[j]=pos->x[i]+SQRT(pos->xs[i])*c;
            yc[j]=pos->y[i]+a*c+b*s;
        }
        GraphT->DrawPoly(xc,yc,37,CColor[1],ShowErr==1?0:1);
    }
}
// draw direction-arrow on track-plot ---------------------------------------
void __fastcall TPlot::DrawTrkArrow(const TIMEPOS *pos)
{
    TPoint p;
    double tt,d[2],dist,dt,vel;
    int i,off=8;
    
    trace(3,"DrawTrkArrow\n");
    
    if (!ShowArrow) return;
    
    for (i=1;i<pos->n-1;i++) {
        tt=time2gpst(pos->t[i],NULL);
        d[0]=pos->x[i+1]-pos->x[i-1];
        d[1]=pos->y[i+1]-pos->y[i-1];
        dist=norm(d,2);
        dt=timediff(pos->t[i+1],pos->t[i-1]);
        vel=dt==0.0?0.0:dist/dt;
        
        if (vel<0.5||fmod(tt+0.005,INTARROW)>=0.01) continue;
        
        GraphT->ToPoint(pos->x[i],pos->y[i],p);
        p.x-=(int)(off*d[1]/dist);
        p.y-=(int)(off*d[0]/dist);
        DrawMark(GraphT,p,10,CColor[3],15,(int)(ATAN2(d[1],d[0])*R2D));
    }
}
// draw velocity-indicator on track-plot ------------------------------------
void __fastcall TPlot::DrawTrkVel(const TIMEPOS *vel)
{
    UTF8String label;
    TPoint p1,p2;
    double v=0.0,dir=0.0;
    
    trace(3,"DrawTrkVel\n");
    
    if (vel&&vel->n>0) {
        if ((v=SQRT(SQR(vel->x[0])+SQR(vel->y[0])))>1.0) {
            dir=ATAN2(vel->x[0],vel->y[0])*R2D;
        }
    }
    GraphT->GetPos(p1,p2);
    p1.x+=SIZE_VELC/2+30;
    p1.y=p2.y-SIZE_VELC/2-30;
    DrawMark(GraphT,p1,1,CColor[2],SIZE_VELC,0);
    p1.y+=SIZE_VELC/2;
    label.sprintf("%.0f km/h",v*3600.0/1000.0);
    DrawLabel(GraphT,p1,label,0,2);
    p1.y-=SIZE_VELC/2;
    if (v>=1.0) DrawMark(GraphT,p1,10,CColor[2],SIZE_VELC,90-(int)dir);
    DrawMark(GraphT,p1,0,CColor[0],8,0);
    DrawMark(GraphT,p1,1,CColor[2],8,0);
}
// draw solution-plot -------------------------------------------------------
void __fastcall TPlot::DrawSol(int level, int type)
{
    UTF8String label[]={"E-W","N-S","U-D"},unit[]={"m","m/s","m/s" CHARUP2};
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    TIMEPOS *pos,*pos1,*pos2;
    gtime_t time1={0},time2={0};
    TPoint p1,p2;
    double xc,yc,xl[2],yl[2],off,y;
    int i,j,k,sel=!BtnSol1->Down&&BtnSol2->Down?1:0,p=0;
    
    trace(3,"DrawSol: level=%d\n",level);
    
    if (BtnShowTrack->Down&&(BtnFixHoriz->Down||BtnFixVert->Down)) {
        
        pos=SolToPos(SolData+sel,SolIndex[sel],0,type);
        
        for (i=0;i<3&&pos->n>0;i++) {
            GraphG[i]->GetCent(xc,yc);
            if (BtnFixVert->Down) {
                yc=i==0?pos->x[0]:(i==1?pos->y[0]:pos->z[0]);
            }
            if (BtnFixHoriz->Down) {
                GraphG[i]->GetLim(xl,yl);
                off=Xcent*(xl[1]-xl[0])/2.0;
                GraphG[i]->SetCent(TimePos(pos->t[0])-off,yc);
            }
            else {
                GraphG[i]->SetCent(xc,yc);
            }
        }
        delete pos;
    }
    j=-1;
    for (i=0;i<3;i++) if (btn[i]->Down) j=i;
    for (i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        GraphG[i]->XLPos=TimeLabel?(i==j?6:5):(i==j?1:0);
        GraphG[i]->Week=Week;
        GraphG[i]->DrawAxis(ShowLabel,ShowLabel);
    }
    if (BtnSol1->Down) {
        pos=SolToPos(SolData,-1,QFlag->ItemIndex,type);
        DrawSolPnt(pos,level,0);
        DrawSolStat(pos,unit[type],p++);
        delete pos;
    }
    if (BtnSol2->Down) {
        pos=SolToPos(SolData+1,-1,QFlag->ItemIndex,type);
        DrawSolPnt(pos,level,1);
        DrawSolStat(pos,unit[type],p++);
        delete pos;
    }
    if (BtnSol12->Down) {
        pos1=SolToPos(SolData  ,-1,0,type);
        pos2=SolToPos(SolData+1,-1,0,type);
        pos=pos1->diff(pos2,QFlag->ItemIndex);
        DrawSolPnt(pos,level,0);
        DrawSolStat(pos,unit[type],p++);
        delete pos;
        delete pos1;
        delete pos2;
    }
    if (BtnShowTrack->Down&&(BtnSol1->Down||BtnSol2->Down||BtnSol12->Down)) {
        
        pos =SolToPos(SolData+sel,SolIndex[sel],0,type);
        pos1=SolToPos(SolData    ,SolIndex[0]  ,0,type);
        pos2=SolToPos(SolData+1  ,SolIndex[1]  ,0,type);
        if (pos1->n>0) time1=pos1->t[0];
        if (pos2->n>0) time2=pos2->t[0];
        
        for (j=k=0;j<3&&pos->n>0;j++) {
            
            if (!btn[j]->Down) continue;
            
            GraphG[j]->GetLim(xl,yl);
            xl[0]=xl[1]=TimePos(pos->t[0]);
            GraphG[j]->DrawPoly(xl,yl,2,CColor[2],0);
            
            if (BtnSol2->Down&&pos2->n>0&&(time1.time==0||fabs(timediff(time1,time2))<DTTOL*2.0)) {
                xl[0]=xl[1]=TimePos(time2);
                y=j==0?pos2->x[0]:(j==1?pos2->y[0]:pos2->z[0]);
                GraphG[j]->DrawMark(xl[0],y,0,CColor[0],MarkSize*2+6,0);
                GraphG[j]->DrawMark(xl[0],y,1,CColor[1],MarkSize*2+6,0);
                GraphG[j]->DrawMark(xl[0],y,1,CColor[2],MarkSize*2+2,0);
                GraphG[j]->DrawMark(xl[0],y,0,MColor[1][pos->q[0]],MarkSize*2,0);
                if (BtnSol1->Down&&pos1->n>0&&GraphG[j]->ToPoint(xl[0],y,p1)) {
                    p1.x+=MarkSize+4;
                    DrawLabel(GraphG[j],p1,"2",1,0);
                }
            }
            if (BtnSol1->Down&&pos1->n>0) {
                xl[0]=xl[1]=TimePos(time1);
                y=j==0?pos1->x[0]:(j==1?pos1->y[0]:pos1->z[0]);
                GraphG[j]->DrawMark(xl[0],y,0,CColor[0],MarkSize*2+6,0);
                GraphG[j]->DrawMark(xl[0],y,1,CColor[2],MarkSize*2+6,0);
                GraphG[j]->DrawMark(xl[0],y,1,CColor[2],MarkSize*2+2,0);
                GraphG[j]->DrawMark(xl[0],y,0,MColor[0][pos->q[0]],MarkSize*2,0);
                if (BtnSol2->Down&&pos2->n>0&&GraphG[j]->ToPoint(xl[0],y,p1)) {
                    p1.x+=MarkSize+4;
                    DrawLabel(GraphG[j],p1,"1",1,0);
                }
            }
            xl[0]=xl[1]=TimePos(pos->t[0]);
            if (k++==0) {
                GraphG[j]->DrawMark(xl[0],yl[1]-1E-6,0,CColor[2],5,0);
                
                if (!BtnFixHoriz->Down) {
                    GraphG[j]->DrawMark(xl[0],yl[1]-1E-6,1,CColor[2],9,0);
                }
            }
        }
        delete pos;
        delete pos1;
        delete pos2;
    }
    for (i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        GraphG[i]->GetPos(p1,p2);
        p1.x+=5; p1.y+=3;
        DrawLabel(GraphG[i],p1,label[i]+" ("+unit[type]+")",1,2);
    }
}
// draw points and line on solution-plot ------------------------------------
void __fastcall TPlot::DrawSolPnt(const TIMEPOS *pos, int level, int style)
{
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    double *x,*y,*s,xs,ys,*yy;
    int i,j;
    
    trace(3,"DrawSolPnt: level=%d style=%d\n",level,style);
    
    x=new double [pos->n];
    
    for (i=0;i<pos->n;i++) {
        x[i]=TimePos(pos->t[i]);
    }
    for (i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        
        y=i==0?pos->x :(i==1?pos->y :pos->z );
        s=i==0?pos->xs:(i==1?pos->ys:pos->zs);
        
        if (!level||!(PlotStyle%2)) {
            DrawPolyS(GraphG[i],x,y,pos->n,CColor[3],style);
        }
        if (level&&ShowErr&&PlotType<=PLOT_SOLA&&PlotStyle<2) {
            
            GraphG[i]->GetScale(xs,ys);
            
            if (ShowErr==1) {
                for (j=0;j<pos->n;j++) {
                    GraphG[i]->DrawMark(x[j],y[j],12,CColor[1],(int)(SQRT(s[j])*2.0/ys),0);
                }
            }
            else {
                yy=new double [pos->n];
                
                for (j=0;j<pos->n;j++) yy[j]=y[j]-SQRT(s[j]);
                DrawPolyS(GraphG[i],x,yy,pos->n,CColor[1],1);
                
                for (j=0;j<pos->n;j++) yy[j]=y[j]+SQRT(s[j]);
                DrawPolyS(GraphG[i],x,yy,pos->n,CColor[1],1);
                
                delete [] yy;
            }
        }
        if (level&&PlotStyle<2) {
            TColor *color=new TColor[pos->n];
            for (j=0;j<pos->n;j++) color[j]=MColor[style][pos->q[j]];
            GraphG[i]->DrawMarks(x,y,color,pos->n,0,MarkSize,0);
            delete [] color;
        }
    }
    delete [] x;
}
// draw statistics on solution-plot -----------------------------------------
void __fastcall TPlot::DrawSolStat(const TIMEPOS *pos, UTF8String unit, int p)
{
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    TPoint p1,p2;
    double ave,std,rms,*y,opos[3];
    int i,j=0,k=0,fonth=(int)(Disp->Font->Size*1.5);
    char *u;
    UTF8String label,s;
    
    trace(3,"DrawSolStat: p=%d\n",p);
    
    if (!ShowStats||pos->n<=0) return;
    
    for (i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        
        y=i==0?pos->x:(i==1?pos->y:pos->z);
        CalcStats(y,pos->n,0.0,ave,std,rms);
        GraphG[i]->GetPos(p1,p2);
        p1.x=p2.x-5;
        p1.y+=3+fonth*(p+(!k++&&p>0?1:0));
        
        if (j==0&&p==0) {
            
            if (norm(OPos,3)>0.0) {
                ecef2pos(OPos,opos);
                label="ORI="+LatLonStr(opos,9)+s.sprintf(" %.4fm",opos[2]);
                DrawLabel(GraphG[j],p1,label,2,2);
                j++; p1.y+=fonth;
            }
        }
        u=unit.c_str();
        s.sprintf("AVE=%.4f%s STD=%.4f%s RMS=%.4f%s",ave,u,std,u,rms,u);
        DrawLabel(GraphG[i],p1,s,2,2);
    }
}
// draw number-of-satellite plot --------------------------------------------
void __fastcall TPlot::DrawNsat(int level)
{
    UTF8String label[]={
        "# of Valid Satellites",
        "Age of Differential (s)",
        "Ratio Factor for AR Validation"
    };
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    TIMEPOS *ns;
    TPoint p1,p2;
    double xc,yc,y,xl[2],yl[2],off;
    int i,j,k,sel=!BtnSol1->Down&&BtnSol2->Down?1:0;
    
    trace(3,"DrawNsat: level=%d\n",level);
    
    if (BtnShowTrack->Down&&BtnFixHoriz->Down) {
        
        ns=SolToNsat(SolData+sel,SolIndex[sel],0);
        
        for (i=0;i<3;i++) {
            if (BtnFixHoriz->Down) {
                GraphG[i]->GetLim(xl,yl);
                off=Xcent*(xl[1]-xl[0])/2.0;
                GraphG[i]->GetCent(xc,yc);
                GraphG[i]->SetCent(TimePos(ns->t[0])-off,yc);
            }
            else {
                GraphG[i]->GetRight(xc,yc);
                GraphG[i]->SetRight(TimePos(ns->t[0]),yc);
            }
        }
        delete ns;
    }
    j=-1;
    for (i=0;i<3;i++) if (btn[i]->Down) j=i;
    for (i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        GraphG[i]->XLPos=TimeLabel?(i==j?6:5):(i==j?1:0);
        GraphG[i]->Week=Week;
        GraphG[i]->DrawAxis(ShowLabel,ShowLabel);
    }
    if (BtnSol1->Down) {
        ns=SolToNsat(SolData,-1,QFlag->ItemIndex);
        DrawSolPnt(ns,level,0);
        delete ns;
    }
    if (BtnSol2->Down) {
        ns=SolToNsat(SolData+1,-1,QFlag->ItemIndex);
        DrawSolPnt(ns,level,1);
        delete ns;
    }
    if (BtnShowTrack->Down&&(BtnSol1->Down||BtnSol2->Down)) {
        
        ns=SolToNsat(SolData+sel,SolIndex[sel],0);
        
        for (j=k=0;j<3&&ns->n>0;j++) {
            
            if (!btn[j]->Down) continue;
            
            y=j==0?ns->x[0]:(j==1?ns->y[0]:ns->z[0]);
            GraphG[j]->GetLim(xl,yl);
            xl[0]=xl[1]=TimePos(ns->t[0]);
            
            GraphG[j]->DrawPoly(xl,yl,2,CColor[2],0);
            GraphG[j]->DrawMark(xl[0],y,0,CColor[0],MarkSize*2+6,0);
            GraphG[j]->DrawMark(xl[0],y,1,CColor[2],MarkSize*2+6,0);
            GraphG[j]->DrawMark(xl[0],y,1,CColor[2],MarkSize*2+2,0);
            GraphG[j]->DrawMark(xl[0],y,0,MColor[sel][ns->q[0]],MarkSize*2,0);
            
            if (k++==0) {
                GraphG[j]->DrawMark(xl[0],yl[1]-1E-6,0,CColor[2],5,0);
                
                if (!BtnFixHoriz->Down) {
                    GraphG[j]->DrawMark(xl[0],yl[1]-1E-6,1,CColor[2],9,0);
                }
            }
        }
        delete ns;
    }
    for (i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        GraphG[i]->GetPos(p1,p2);
        p1.x+=5; p1.y+=3;
        DrawLabel(GraphG[i],p1,label[i],1,2);
    }
}
// draw observation-data-plot -----------------------------------------------
void __fastcall TPlot::DrawObs(int level)
{
    TPoint p1,p2,p;
    gtime_t time;
    obsd_t *obs;
    double xs,ys,xt,xl[2],yl[2],tt[MAXSAT]={0},xp,xc,yc,yp[MAXSAT]={0};
    int i,j,m=0,sats[MAXSAT]={0},ind=ObsIndex,prn;
    char id[8];
    
    trace(3,"DrawObs: level=%d\n",level);
    
    for (i=0;i<Obs.n;i++) {
        if (SatMask[Obs.data[i].sat-1]) continue;
        sats[Obs.data[i].sat-1]=1;
    }
    for (i=0;i<MAXSAT;i++) if (sats[i]) m++;
    
    GraphR->XLPos=TimeLabel?6:1; 
    GraphR->YLPos=0;
    GraphR->Week=Week;
    GraphR->GetLim(xl,yl);
    yl[0]=0.5;
    yl[1]=m>0?m+0.5:m+10.5;
    GraphR->SetLim(xl,yl);
    GraphR->SetTick(0.0,1.0);
    
    if (0<=ind&&ind<NObs&&BtnShowTrack->Down&&BtnFixHoriz->Down) {
        xp=TimePos(Obs.data[IndexObs[ind]].time);
        if (BtnFixHoriz->Down) {
            double xl[2],yl[2],off;
            GraphR->GetLim(xl,yl);
            off=Xcent*(xl[1]-xl[0])/2.0;
            GraphR->GetCent(xc,yc);
            GraphR->SetCent(xp-off,yc);
        }
        else {
            GraphR->GetRight(xc,yc);
            GraphR->SetRight(xp,yc);
        }
    }
    GraphR->DrawAxis(1,1);
    GraphR->GetPos(p1,p2);
    
    for (i=0,j=0;i<MAXSAT;i++) {
        if (!sats[i]) continue;
        UTF8String label;
        p.x=p1.x;
        p.y=p1.y+(int)((p2.y-p1.y)*(j+0.5)/m);
        yp[i]=m-(j++);
        satno2id(i+1,id);
        label=id;
        GraphR->DrawText(p,label,CColor[2],2,0,0);
    }
    p1.x=Disp->Font->Size;
    p1.y=(p1.y+p2.y)/2;
    GraphR->DrawText(p1,"SATELLITE NO",CColor[2],0,0,90);
    
    if (!BtnSol1->Down) return;
    
    if (level&&PlotStyle<=2) {
        DrawObsEphem(yp);
    }
    if (level&&PlotStyle<=2) {
        GraphR->GetScale(xs,ys);
        for (i=0;i<Obs.n;i++) {
            obs=&Obs.data[i];
            TColor col=ObsColor(obs,Az[i],El[i]);
            if (col==clBlack) continue;
            
            xt=TimePos(obs->time);
            if (fabs(xt-tt[obs->sat-1])/xs>0.9) {
                GraphR->DrawMark(xt,yp[obs->sat-1],0,PlotStyle<2?col:CColor[3],
                                 PlotStyle<2?MarkSize:0,0);
                tt[obs->sat-1]=xt;
            }
        }
    }
    if (level&&PlotStyle<=2) {
        DrawObsSlip(yp);
    }
    if (BtnShowTrack->Down&&0<=ind&&ind<NObs) {
        i=IndexObs[ind];
        time=Obs.data[i].time;
        
        GraphR->GetLim(xl,yl);
        xl[0]=xl[1]=TimePos(Obs.data[i].time);
        GraphR->DrawPoly(xl,yl,2,CColor[2],0);
        
        for (;i<Obs.n&&timediff(Obs.data[i].time,time)==0.0;i++) {
            obs=&Obs.data[i];
            TColor col=ObsColor(obs,Az[i],El[i]);
            if (col==clBlack) continue;
            GraphR->DrawMark(xl[0],yp[obs->sat-1],0,col,MarkSize*2+2,0);
        }
        GraphR->DrawMark(xl[0],yl[1]-1E-6,0,CColor[2],5,0);
        if (!BtnFixHoriz->Down) {
            GraphR->DrawMark(xl[0],yl[1]-1E-6,1,CColor[2],9,0);
        }
    }
}
// generate observation-data-slips ------------------------------------------
void __fastcall TPlot::GenObsSlip(int *LLI)
{
    AnsiString text=ObsType->Text;
    const char *obstype=text.c_str();
    double gfp[MAXSAT][NFREQ+NEXOBS]={{0}};

    for (int i=0;i<Obs.n;i++) {
        obsd_t *obs=Obs.data+i;
        int j,k;
        
        LLI[i]=0;
        if (El[i]<ElMask*D2R||!SatSel[obs->sat-1]) continue;
        if (ElMaskP&&El[i]<ElMaskData[(int)(Az[i]*R2D+0.5)]) continue;
        
        if (ShowSlip==1) { // LG jump
            double freq1,freq2,gf;
            
            if (!strcmp(obstype,"ALL")) {
                if ((freq1=sat2freq(obs->sat,obs->code[0],&Nav))==0.0) continue;
                LLI[i]=obs->LLI[0]&2;
                for (j=1;j<NFREQ+NEXOBS;j++) {
                    LLI[i]|=obs->LLI[j]&2;
                    if ((freq2=sat2freq(obs->sat,obs->code[j],&Nav))==0.0) continue;
                    gf=CLIGHT*(obs->L[0]/freq1-obs->L[j]/freq2);
                    if (fabs(gfp[obs->sat-1][j]-gf)>THRESLIP) LLI[i]|=1;
                    gfp[obs->sat-1][j]=gf;
                }
            }
            else {
                if (sscanf(obstype,"L%1d",&k)==1) {
                    j=k>2?k-3:k-1;  /* L1,L2,L5,L6 ... */
                }
                else {
                    for (j=0;j<NFREQ+NEXOBS;j++) {
                        if (!strcmp(code2obs(obs->code[j]),obstype)) break;
                    }
                    if (j>=NFREQ+NEXOBS) continue;
                }    
                LLI[i]=obs->LLI[j]&2;
                k=(j==0)?1:0;
                if ((freq1=sat2freq(obs->sat,obs->code[k],&Nav))==0.0) continue;
                if ((freq2=sat2freq(obs->sat,obs->code[j],&Nav))==0.0) continue;
                gf=CLIGHT*(obs->L[k]/freq1-obs->L[j]/freq2);
                if (fabs(gfp[obs->sat-1][j]-gf)>THRESLIP) LLI[i]|=1;
                gfp[obs->sat-1][j]=gf;
            }
        }
        else {
            if (!strcmp(obstype,"ALL")) {
                for (j=0;j<NFREQ+NEXOBS;j++) {
                    LLI[i]|=obs->LLI[j];
                }
            }
            else {
                if (sscanf(obstype,"L%1d",&k)==1) {
                    j=k>2?k-3:k-1;  /* L1,L2,L5,L6 ... */
                }
                else {
                    for (j=0;j<NFREQ+NEXOBS;j++) {
                        if (!strcmp(code2obs(obs->code[j]),obstype)) break;
                    }
                    if (j>=NFREQ+NEXOBS) continue;
                }
                LLI[i]=obs->LLI[j];
            }
        }
    }
}
// draw slip on observation-data-plot ---------------------------------------
void __fastcall TPlot::DrawObsSlip(double *yp)
{
    trace(3,"DrawObsSlip\n");

    if (Obs.n<=0||(!ShowSlip&&!ShowHalfC)) return;

    int *LLI=new int [Obs.n];
    
    GenObsSlip(LLI);

    for (int i=0;i<Obs.n;i++) {
        if (!LLI[i]) continue;
        TPoint ps[2];
        obsd_t *obs=Obs.data+i;
        if (GraphR->ToPoint(TimePos(obs->time),yp[obs->sat-1],ps[0])) {
            ps[1].x=ps[0].x;
            ps[1].y=ps[0].y+MarkSize*3/2+1;
            ps[0].y=ps[0].y-MarkSize*3/2;
            if (ShowHalfC&&(LLI[i]&2)) GraphR->DrawPoly(ps,2,MColor[0][0],0);
            if (ShowSlip &&(LLI[i]&1)) GraphR->DrawPoly(ps,2,MColor[0][5],0);
        }
    }
    delete [] LLI;
}
// draw ephemeris on observation-data-plot ----------------------------------
void __fastcall TPlot::DrawObsEphem(double *yp)
{
    TPoint ps[3];
    int i,j,k,in,svh,off[MAXSAT]={0};
    
    trace(3,"DrawObsEphem\n");
    
    if (!ShowEph) return;
    
    for (i=0;i<MAXSAT;i++) {
        if (!SatSel[i]) continue;
        for (j=0;j<Nav.n;j++) {
            if (Nav.eph[j].sat!=i+1) continue;
            GraphR->ToPoint(TimePos(Nav.eph[j].ttr),yp[i],ps[0]);
            in=GraphR->ToPoint(TimePos(Nav.eph[j].toe),yp[i],ps[2]);
            ps[1]=ps[0];
            off[Nav.eph[j].sat-1]=off[Nav.eph[j].sat-1]?0:3;
            
            for (k=0;k<3;k++) ps[k].y+=MarkSize+2+off[Nav.eph[j].sat-1];
            ps[0].y-=2;
            
            svh=Nav.eph[j].svh;
            // Mask QZS LEX health.
            int health = satsys(i + 1, NULL) == SYS_QZS ? (svh & 0xfe) != 0 : svh != 0;
            GraphR->DrawPoly(ps,3,health?MColor[0][5]:CColor[1],0);
            
            if (in) GraphR->DrawMark(ps[2],0,health?MColor[0][5]:CColor[1],health?4:3,0);
        }
        for (j=0;j<Nav.ng;j++) {
            if (Nav.geph[j].sat!=i+1) continue;
            GraphR->ToPoint(TimePos(Nav.geph[j].tof),yp[i],ps[0]);
            in=GraphR->ToPoint(TimePos(Nav.geph[j].toe),yp[i],ps[2]);
            ps[1]=ps[0];
            off[Nav.geph[j].sat-1]=off[Nav.geph[j].sat-1]?0:3;
            for (k=0;k<3;k++) ps[k].y+=MarkSize+2+off[Nav.geph[j].sat-1];
            ps[0].y-=2;
            
            svh=Nav.geph[j].svh;
            int health = (svh & 9) != 0 || (svh & 6) == 4;
            GraphR->DrawPoly(ps,3,health?MColor[0][5]:CColor[1],0);
            
            if (in) GraphR->DrawMark(ps[2],0,Nav.geph[j].svh?MColor[0][5]:CColor[1],
                                     Nav.geph[j].svh?4:3,0);
        }
        for (j=0;j<Nav.ns;j++) {
            if (Nav.seph[j].sat!=i+1) continue;
            GraphR->ToPoint(TimePos(Nav.seph[j].tof),yp[i],ps[0]);
            in=GraphR->ToPoint(TimePos(Nav.seph[j].t0),yp[i],ps[2]);
            ps[1]=ps[0];
            off[Nav.seph[j].sat-1]=off[Nav.seph[j].sat-1]?0:3;
            for (k=0;k<3;k++) ps[k].y+=MarkSize+2+off[Nav.seph[j].sat-1];
            ps[0].y-=2;
            
            svh=Nav.seph[j].svh;
            GraphR->DrawPoly(ps,3,svh?MColor[0][5]:CColor[1],0);
            
            if (in) GraphR->DrawMark(ps[2],0,Nav.seph[j].svh?MColor[0][5]:CColor[1],
                                     Nav.seph[j].svh?4:3,0);
        }
    }
}
// draw sky-image on sky-plot -----------------------------------------------
void __fastcall TPlot::DrawSkyImage(int level)
{
    TCanvas *c=Disp->Canvas;
    TPoint p1,p2;
    double xl[2],yl[2],r,s,mx[190],my[190];
    
    trace(3,"DrawSkyImage: level=%d\n",level);
    
    if (SkySize[0]<=0||SkySize[1]<=0) return;
    
    GraphS->GetLim(xl,yl);
    r=(xl[1]-xl[0]<yl[1]-yl[0]?xl[1]-xl[0]:yl[1]-yl[0])*0.45;
    s=r*SkyImageR->Width/2.0/SkyScaleR;
    GraphS->ToPoint(-s,s,p1);
    GraphS->ToPoint(s,-s,p2);
    TRect rect(p1,p2);
    c->StretchDraw(rect,SkyImageR);
    
    if (SkyElMask) { // elevation mask
        int n=0;
        
        mx[n]=0.0;   my[n++]=yl[1];
        for (int i=0;i<=180;i++) {
            mx[n  ]=r*sin(i*2.0*D2R);
            my[n++]=r*cos(i*2.0*D2R);
        }
        mx[n]=0.0;   my[n++]=yl[1];
        mx[n]=xl[0]; my[n++]=yl[1];
        mx[n]=xl[0]; my[n++]=yl[0];
        mx[n]=xl[1]; my[n++]=yl[0];
        mx[n]=xl[1]; my[n++]=yl[1];
        GraphS->DrawPatch(mx,my,n,CColor[0],CColor[0],0);
    }
}
// draw sky-plot ------------------------------------------------------------
void __fastcall TPlot::DrawSky(int level)
{
    TPoint p1,p2;
    AnsiString text=ObsType->Text;
    const char *obstype=text.c_str();
    obsd_t *obs;
    gtime_t t[MAXSAT]={{0}};
    double p[MAXSAT][2]={{0}},gfp[MAXSAT][NFREQ+NEXOBS]={{0}},p0[MAXSAT][2]={{0}};
    double x,y,xp,yp,xs,ys,dt,dx,dy,xl[2],yl[2],r,gf,freq1,freq2;
    int i,j,ind=ObsIndex,freq;
    int hh=(int)(Disp->Font->Size*1.5),prn,color,slip;
    
    trace(3,"DrawSky: level=%d\n",level);
    
    GraphS->GetLim(xl,yl);
    r=(xl[1]-xl[0]<yl[1]-yl[0]?xl[1]-xl[0]:yl[1]-yl[0])*0.45;
    
    if (BtnShowImg->Down) {
        DrawSkyImage(level);
    }
    if (BtnShowSkyplot->Down) {
        GraphS->DrawSkyPlot(0.0,0.0,CColor[1],CColor[2],CColor[0],r*2.0);
    }
    if (!BtnSol1->Down) return;
    
    GraphS->GetScale(xs,ys);
    
    if (PlotStyle<=2) {
        for (i=0;i<Obs.n;i++) {
            obs=&Obs.data[i];
            if (SatMask[obs->sat-1]||!SatSel[obs->sat-1]||El[i]<=0.0) continue;
            TColor col=ObsColor(obs,Az[i],El[i]);
            if (col==clBlack) continue;
            
            x =r*sin(Az[i])*(1.0-2.0*El[i]/PI);
            y =r*cos(Az[i])*(1.0-2.0*El[i]/PI);
            xp=p[obs->sat-1][0];
            yp=p[obs->sat-1][1];
            
            if ((x-xp)*(x-xp)+(y-yp)*(y-yp)>=xs*xs) {
                int siz=PlotStyle<2?MarkSize:1;
                GraphS->DrawMark(x,y,0,PlotStyle<2?col:CColor[3],siz,0);
                p[obs->sat-1][0]=x;
                p[obs->sat-1][1]=y;
            }
            if (xp==0.0&&yp==0.0) {
                p0[obs->sat-1][0]=x;
                p0[obs->sat-1][1]=y;
            }
        }
    }
    if ((PlotStyle==0||PlotStyle==2)&&!BtnShowTrack->Down) {
        
        for (i=0;i<MAXSAT;i++) {
            if (p0[i][0]!=0.0||p0[i][1]!=0.0) {
                TPoint pnt;
                if (GraphS->ToPoint(p0[i][0],p0[i][1],pnt)) {
                    char id[8];
                    satno2id(i+1,id);
                    UTF8String ustr=id;
                    DrawLabel(GraphS,pnt,ustr,1,0);
                }
            }
        }
    }
    if (!level) return;
    
    if (ShowSlip&&PlotStyle<=2) {
        int *LLI=new int [Obs.n];
         
        GenObsSlip(LLI);
        
        for (i=0;i<Obs.n;i++) {
            if (!(LLI[i]&1)) continue;
            obs=Obs.data+i;
            x=r*sin(Az[i])*(1.0-2.0*El[i]/PI);
            y=r*cos(Az[i])*(1.0-2.0*El[i]/PI);
            dt=timediff(obs->time,t[obs->sat-1]);
            dx=x-p[obs->sat-1][0];
            dy=y-p[obs->sat-1][1];
            t[obs->sat-1]=obs->time;
            p[obs->sat-1][0]=x;
            p[obs->sat-1][1]=y;
            if (fabs(dt)>300.0) continue;
            GraphS->DrawMark(x,y,4,MColor[0][5],MarkSize*3+2,ATAN2(dy,dx)*R2D+90);
        }
        delete [] LLI;
    }
    if (ElMaskP) {
        double *x=new double [361];
        double *y=new double [361];
        for (i=0;i<=360;i++) {
            x[i]=r*sin(i*D2R)*(1.0-2.0*ElMaskData[i]/PI);
            y[i]=r*cos(i*D2R)*(1.0-2.0*ElMaskData[i]/PI);
        }
        Disp->Canvas->Pen->Width=2;
        GraphS->DrawPoly(x,y,361,COL_ELMASK,0);
        Disp->Canvas->Pen->Width=1;
        delete [] x;
        delete [] y;
    }
    if (BtnShowTrack->Down&&0<=ind&&ind<NObs) {
        
        for (i=IndexObs[ind];i<Obs.n&&i<IndexObs[ind+1];i++) {
            obs=&Obs.data[i];
            if (SatMask[obs->sat-1]||!SatSel[obs->sat-1]||El[i]<=0.0) continue;
            TColor col=ObsColor(obs,Az[i],El[i]);
            if (col==clBlack) continue;
            
            x=r*sin(Az[i])*(1.0-2.0*El[i]/PI);
            y=r*cos(Az[i])*(1.0-2.0*El[i]/PI);
            
            char id[8];
            satno2id(obs->sat,id);
            UTF8String ustr=id;
            GraphS->DrawMark(x,y,0,col,Disp->Font->Size*2+5,0);
            GraphS->DrawMark(x,y,1,col==clBlack?MColor[0][0]:CColor[2],Disp->Font->Size*2+5,0);
            GraphS->DrawText(x,y,ustr,CColor[0],0,0,0);
        }
    }
    GraphS->GetPos(p1,p2);
    p1.x+=10; p1.y+=8; p2.x-=10; p2.y=p1.y;
    
    if (ShowStats&&!SimObs) {
        UTF8String ustr;
        ustr.sprintf("MARKER: %s %s %s",Sta.name,Sta.markerno,Sta.markertype);
        DrawLabel(GraphS,p1,ustr,1,2); p1.y+=hh;
        ustr.sprintf("REC: %s %s %s",Sta.rectype,Sta.recver,Sta.recsno);
        DrawLabel(GraphS,p1,ustr,1,2); p1.y+=hh;
        ustr.sprintf("ANT: %s %s",Sta.antdes,Sta.antsno);
        DrawLabel(GraphS,p1,ustr,1,2); p1.y+=hh;
    }
    // show statistics
    if (ShowStats&&BtnShowTrack->Down&&0<=ind&&ind<NObs&&!SimObs) {
        UTF8String ustr,ss;
        char id[8];
        
        if (!strcmp(obstype,"ALL")) {
            ustr.sprintf("%3s: %*s %*s%*s %*s","SAT",NFREQ,"PR",NFREQ,"CP",
                         NFREQ*3,"CN0",NFREQ,"LLI");
        }
        else {
            ustr.sprintf("%3s: %3s  %4s   %3s %3s","SAT","SIG","OBS","CN0","LLI");
        }
        GraphS->DrawText(p2,ustr,clBlack,2,2,0,MS_FONT);
        p2.y+=3;
        
        for (i=IndexObs[ind];i<Obs.n&&i<IndexObs[ind+1];i++) {
            obs=&Obs.data[i];
            if (SatMask[obs->sat-1]||!SatSel[obs->sat-1]) continue;
            if (HideLowSat&&El[i]<ElMask*D2R) continue;
            if (HideLowSat&&ElMaskP&&El[i]<ElMaskData[(int)(Az[i]*R2D+0.5)]) continue;
            
            satno2id(obs->sat,id);
            ustr.sprintf("%-3s: ",id);
            
            if (!strcmp(obstype,"ALL")) {
                for (j=0;j<NFREQ;j++) ustr+=obs->P[j]==0.0?"-":"C";
                ustr+=" ";
                for (j=0;j<NFREQ;j++) ustr+=obs->L[j]==0.0?"-":"L";
                ustr+=" ";
                for (j=0;j<NFREQ;j++) {
                    if (obs->P[j]==0.0&&obs->L[j]==0.0) ustr+="-- ";
                    else ustr+=ss.sprintf("%02.0f ",obs->SNR[j]);
                }
                for (j=0;j<NFREQ;j++) {
                    if (obs->L[j]==0.0) ustr+="-";
                    else ustr+=ss.sprintf("%d",obs->LLI[j]);
                }
            }
            else if (sscanf(obstype,"L%1d",&freq)==1) {
                freq-=freq>2?2:0;  /* L1,L2,L5,L6 ... */
                if (!obs->code[freq-1]) continue;
                ustr+=ss.sprintf("%3s  %s %s %s  ",code2obs(obs->code[freq-1]),
                              obs->P[freq-1]==0.0?"-":"C",obs->L[freq-1]==0.0?"-":"L",
                              obs->D[freq-1]==0.0?"-":"D");
                if (obs->P[freq-1]==0.0&&obs->L[freq-1]==0.0) ustr+="---- ";
                else ustr+=ss.sprintf("%4.1f ",obs->SNR[freq-1]);
                if (obs->L[freq-1]==0.0) ustr+=" -";
                else ustr+=ss.sprintf("%2d",obs->LLI[freq-1]);
            }
            else {
                for (j=0;j<NFREQ+NEXOBS;j++) {
                    if (!strcmp(code2obs(obs->code[j]),obstype)) break;
                }
                if (j>=NFREQ+NEXOBS) continue;
                ustr+=ss.sprintf("%3s  %s %s %s  ",code2obs(obs->code[j]),
                                 obs->P[j]==0.0?"-":"C",obs->L[j]==0.0?"-":"L",
                                 obs->D[j]==0.0?"-":"D");
                if (obs->P[j]==0.0&&obs->L[j]==0.0) ustr+="---- ";
                else ustr+=ss.sprintf("%4.1f ",obs->SNR[j]);
                if (obs->L[j]==0.0) ustr+=" -";
                else ustr+=ss.sprintf("%2d",obs->LLI[j]);
            }
            TColor col=ObsColor(obs,Az[i],El[i]);
            p2.y+=hh;
            GraphS->DrawText(p2,ustr,col==clBlack?MColor[0][0]:col,2,2,0,MS_FONT);
        }
    }
    if (Nav.n<=0&&Nav.ng<=0&&!SimObs) {
        GraphS->GetPos(p1,p2);
        p2.x-=10;
        p2.y-=3;
        DrawLabel(GraphS,p2,"No Navigation Data",2,1);
    }
}
// Draw sky-plot ------------------------------------------------------------
void __fastcall TPlot::DrawSolSky(int level) {
  trace(3, "DrawSolSky: level=%d\n", level);

  int sel = !BtnSol1->Down && BtnSol2->Down ? 1 : 0, ind = SolIndex[sel];
  int frq = FrqType->ItemIndex + 1, sn = SolStat[sel].n;

  double xl[2], yl[2];
  GraphS->GetLim(xl, yl);
  double r = (xl[1] - xl[0] < yl[1] - yl[0] ? xl[1] - xl[0] : yl[1] - yl[0]) * 0.45;

  if (BtnShowImg->Down) DrawSkyImage(level);

  if (BtnShowSkyplot->Down) GraphS->DrawSkyPlot(0.0, 0.0, CColor[1], CColor[2], CColor[0], r * 2.0);

  if (!BtnSol1->Down && !BtnSol2->Down) return;

  double xs, ys;
  GraphS->GetScale(xs, ys);

  double p0[MAXSAT][2] = {{0}};
  if (PlotStyle <= 2) {
    double p[MAXSAT][2]={{0}};
    for (int i = 0; i < sn; i++) {
      solstat_t *solstat = SolStat[sel].data + i;
      if (SatMask[solstat->sat - 1] || !SatSel[solstat->sat - 1]) continue;
      if (solstat->frq != frq || solstat->el <= 0.0) continue;

      TColor col = SnrColor(solstat->snr);
      // Include satellites with invalid data but note this in the color.
      if ((solstat->flag & 0x20) == 0) col = MColor[0][7];
      double azel[2];
      azel[0] = solstat->az;
      azel[1] = solstat->el;
      if (solstat->el < ElMask * D2R ||
          (ElMaskP && solstat->el < ElMaskData[(int)(solstat->az * R2D + 0.5)])) {
        if (HideLowSat) continue;
        col = MColor[0][0];
      }

      double x = r * sin(solstat->az) * (1.0 - 2.0 * solstat->el / PI);
      double y = r * cos(solstat->az) * (1.0 - 2.0 * solstat->el / PI);
      double xp = p[solstat->sat - 1][0];
      double yp = p[solstat->sat - 1][1];

      if ((x - xp) * (x - xp) + (y - yp) * (y - yp) >= xs * xs) {
        int siz = PlotStyle < 2 ? MarkSize : 1;
        if (PlotStyle >= 2) col = CColor[3];
        GraphS->DrawMark(x, y, 0, col, siz, 0);
        p[solstat->sat - 1][0] = x;
        p[solstat->sat - 1][1] = y;
      }
      if (xp == 0.0 && yp == 0.0) {
        // Save first point.
        p0[solstat->sat - 1][0] = x;
        p0[solstat->sat - 1][1] = y;
      }
    }
  }
  if ((PlotStyle == 0 || PlotStyle == 2) && !BtnShowTrack->Down) {
    // Plot style with lines.
    for (int i = 0; i < MAXSAT; i++) {
      if (p0[i][0] != 0.0 || p0[i][1] != 0.0) {
        TPoint pnt;
        if (GraphS->ToPoint(p0[i][0], p0[i][1], pnt)) {
          char id[8];
          satno2id(i + 1, id);
          UTF8String ustr = id;
          DrawLabel(GraphS, pnt, ustr, 1, 0);
        }
      }
    }
  }
  if (!level) return;

  if (ShowSlip && PlotStyle <= 2) {
    // Plot style not "none".
    gtime_t t[MAXSAT] = {{0}};
    double p[MAXSAT][2] = {{0}};

    for (int i = 0; i < sn; i++) {
      solstat_t *solstat = SolStat[sel].data + i;
      if (SatMask[solstat->sat - 1] || !SatSel[solstat->sat - 1]) continue;
      if (solstat->frq != frq || solstat->el <= 0.0) continue;

      int slip = (solstat->flag >> 3) & 3;
      if (slip == 0) continue;
      double x = r * sin(solstat->az) * (1.0 - 2.0 * solstat->el / PI);
      double y = r * cos(solstat->az) * (1.0 - 2.0 * solstat->el / PI);
      double dt = timediff(solstat->time, t[solstat->sat - 1]);
      double dx = x - p[solstat->sat - 1][0];
      double dy = y - p[solstat->sat - 1][1];
      t[solstat->sat - 1] = solstat->time;
      p[solstat->sat - 1][0] = x;
      p[solstat->sat - 1][1] = y;
      if (fabs(dt) > 300.0) continue;
      GraphS->DrawMark(x, y, 4, MColor[0][5], MarkSize * 3 + 2, ATAN2(dy, dx) * R2D + 90);
    }
  }
  // Draw elevation mask.
  if (ElMaskP) {
    double *x = new double[361];
    double *y = new double[361];
    for (int i = 0; i <= 360; i++) {
      x[i] = r * sin(i * D2R) * (1.0 - 2.0 * ElMaskData[i] / PI);
      y[i] = r * cos(i * D2R) * (1.0 - 2.0 * ElMaskData[i] / PI);
    }
    Disp->Canvas->Pen->Width = 2;
    GraphS->DrawPoly(x, y, 361, COL_ELMASK, 0);
    Disp->Canvas->Pen->Width = 1;
    delete[] x;
    delete[] y;
  }
  // Draw current solution.
  if (BtnShowTrack->Down && 0 <= ind && ind < SolData[sel].n) {
    gtime_t t = SolData[sel].data[ind].time;

    for (int i = 0; i < sn; i++) {
      solstat_t *solstat = SolStat[sel].data + i;
      if (timediff(solstat->time, t) < -DTTOL) continue;
      if (timediff(solstat->time, t) > DTTOL) break;
      if (SatMask[solstat->sat - 1] || !SatSel[solstat->sat - 1]) continue;
      if (solstat->frq != frq || solstat->el <= 0.0) continue;

      TColor col = SnrColor(solstat->snr);
      // Include satellites with invalid data but note this in the color.
      if ((solstat->flag & 0x20) == 0) col = MColor[0][7];
      double azel[2];
      azel[0] = solstat->az;
      azel[1] = solstat->el;
      if (solstat->el < ElMask * D2R ||
          (ElMaskP && solstat->el < ElMaskData[(int)(solstat->az * R2D + 0.5)])) {
        if (HideLowSat) continue;
        col = MColor[0][0];
      }

      double x = r * sin(solstat->az) * (1.0 - 2.0 * solstat->el / PI);
      double y = r * cos(solstat->az) * (1.0 - 2.0 * solstat->el / PI);

      char id[8];
      satno2id(solstat->sat, id);
      UTF8String ustr = id;
      GraphS->DrawMark(x, y, 0, col, Disp->Font->Size * 2 + 5, 0);
      GraphS->DrawMark(x, y, 1, col == clBlack ? MColor[0][0] : CColor[2], Disp->Font->Size * 2 + 5,
                       0);
      GraphS->DrawText(x, y, ustr, CColor[0], 0, 0, 0);
    }
  }
}
// draw DOP and number-of-satellite plot ------------------------------------
void __fastcall TPlot::DrawDop(int level)
{
    UTF8String label;
    TPoint p1,p2;
    gtime_t time;
    double xp,xc,yc,xl[2],yl[2],azel[MAXSAT*2],*dop,*x,*y;
    int i,j,*ns,prn,n=0;
    int ind=ObsIndex,doptype=DopType->ItemIndex;
    
    trace(3,"DrawDop: level=%d\n",level);
    
    GraphR->XLPos=TimeLabel?6:1; 
    GraphR->YLPos=1;
    GraphR->Week=Week;
    GraphR->GetLim(xl,yl);
    yl[0]=0.0;
    yl[1]=MaxDop;
    GraphR->SetLim(xl,yl);
    GraphR->SetTick(0.0,0.0);
    
    if (0<=ind&&ind<NObs&&BtnShowTrack->Down&&BtnFixHoriz->Down) {
        double xl[2],yl[2],off;
        GraphR->GetLim(xl,yl);
        off=Xcent*(xl[1]-xl[0])/2.0;
        xp=TimePos(Obs.data[IndexObs[ind]].time);
        GraphR->GetCent(xc,yc);
        GraphR->SetCent(xp-off,yc);
    }
    GraphR->DrawAxis(1,1);
    GraphR->GetPos(p1,p2);
    p1.x=Disp->Font->Size;
    p1.y=(p1.y+p2.y)/2;
    if (doptype==0) {
        label.sprintf("# OF SATELLITES / DOP %s 10 (EL %s %.0f%s)",CHARMUL,
                      CHARGTE,ElMask,CHARDEG);
    }
    else if (doptype==1) {
        label.sprintf("# OF SATELLITES (EL %s %.0f%s)",CHARGTE,ElMask,CHARDEG);
    }
    else {
        label.sprintf("DOP %s 10 (EL %s %.0f%s)",CHARMUL,CHARGTE,ElMask,CHARDEG);
    }
    GraphR->DrawText(p1,label,CColor[2],0,0,90);
    
    if (!BtnSol1->Down) return;
    
    x  =new double[NObs];
    y  =new double[NObs];
    dop=new double[NObs*4];
    ns =new int   [NObs];
    
    for (i=0;i<NObs;i++) {
        ns[n]=0;
        for (j=IndexObs[i];j<Obs.n&&j<IndexObs[i+1];j++) {
            if (SatMask[Obs.data[j].sat-1]||!SatSel[Obs.data[j].sat-1]) continue;
            if (El[j]<ElMask*D2R) continue;
            if (ElMaskP&&El[j]<ElMaskData[(int)(Az[j]*R2D+0.5)]) continue;
            azel[  ns[n]*2]=Az[j];
            azel[1+ns[n]*2]=El[j];
            ns[n]++;
        }
        dops(ns[n],azel,ElMask*D2R,dop+n*4);
        x[n++]=TimePos(Obs.data[IndexObs[i]].time);
    }
    for (i=0;i<4;i++) {
        if (doptype!=0&&doptype!=i+2) continue;
        
        for (j=0;j<n;j++) y[j]=dop[i+j*4]*10.0;
        
        if (!(PlotStyle%2)) {
            DrawPolyS(GraphR,x,y,n,CColor[3],0);
        }
        if (level&&PlotStyle<2) {
            for (j=0;j<n;j++) {
                if (y[j]==0.0) continue;
                GraphR->DrawMark(x[j],y[j],0,MColor[0][i+2],MarkSize,0);
            }
        }
    }
    if (doptype==0||doptype==1) {
        for (i=0;i<n;i++) y[i]=ns[i];
        
        if (!(PlotStyle%2)) {
            DrawPolyS(GraphR,x,y,n,CColor[3],1);
        }
        if (level&&PlotStyle<2) {
            for (i=0;i<n;i++) {
                GraphR->DrawMark(x[i],y[i],0,MColor[0][1],MarkSize,0);
            }
        }
    }
    if (BtnShowTrack->Down&&0<=ind&&ind<NObs) {
        GraphR->GetLim(xl,yl);
        xl[0]=xl[1]=TimePos(Obs.data[IndexObs[ind]].time);
        
        GraphR->DrawPoly(xl,yl,2,CColor[2],0);
        
        ns[0]=0;
        for (i=IndexObs[ind];i<Obs.n&&i<IndexObs[ind+1];i++) {
            if (SatMask[Obs.data[i].sat-1]||!SatSel[Obs.data[i].sat-1]) continue;
            if (El[i]<ElMask*D2R) continue;
            if (ElMaskP&&El[i]<ElMaskData[(int)(Az[i]*R2D+0.5)]) continue;
            azel[  ns[0]*2]=Az[i];
            azel[1+ns[0]*2]=El[i];
            ns[0]++;
        }
        dops(ns[0],azel,ElMask*D2R,dop);
        
        for (i=0;i<4;i++) {
            if ((doptype!=0&&doptype!=i+2)||dop[i]<=0.0) continue;
            GraphR->DrawMark(xl[0],dop[i]*10.0,0,MColor[0][i+2],MarkSize*2+2,0);
        }
        if (doptype==0||doptype==1) {
            GraphR->DrawMark(xl[0],ns[0],0,MColor[0][1],MarkSize*2+2,0);
        }
        GraphR->DrawMark(xl[0],yl[1]-1E-6,0,CColor[2],5,0);
        if (!BtnFixHoriz->Down) {
            GraphR->DrawMark(xl[0],yl[1]-1E-6,1,CColor[2],9,0);
        }
    }
    else {
        DrawDopStat(dop,ns,n);
    }
    if (Nav.n<=0&&Nav.ng<=0&&(doptype==0||doptype>=2)&&!SimObs) {
        GraphR->GetPos(p1,p2);
        p2.x-=10;
        p2.y-=3;
        DrawLabel(GraphR,p2,"No Navigation Data",2,1);
    }
    delete [] x;
    delete [] y;
    delete [] dop;
    delete [] ns;
}
// Draw DOP and number-of-satellite plot ------------------------------------
void __fastcall TPlot::DrawSolDop(int level) {
  trace(3, "DrawSolDop: level=%d\n", level);

  int sel = !BtnSol1->Down && BtnSol2->Down ? 1 : 0, ind = SolIndex[sel];
  int sn = SolStat[sel].n;

  GraphR->XLPos = TimeLabel ? 6 : 1;
  GraphR->YLPos = 1;
  GraphR->Week = Week;

  // Adjust plot limits.
  double xl[2], yl[2];
  GraphR->GetLim(xl, yl);
  yl[0] = 0.0;
  yl[1] = MaxDop;
  GraphR->SetLim(xl, yl);
  GraphR->SetTick(0.0, 0.0);

  // Update plot center to current position.
  if (0 <= ind && ind < SolData[sel].n && BtnShowTrack->Down && BtnFixHoriz->Down) {
    double xl[2], yl[2], off;
    GraphR->GetLim(xl, yl);
    off = Xcent * (xl[1] - xl[0]) / 2.0;
    gtime_t t = SolData[sel].data[ind].time;
    double xp = TimePos(t);
    double xc, yc;
    GraphR->GetCent(xc, yc);
    GraphR->SetCent(xp - off, yc);
  }
  GraphR->DrawAxis(1, 1);

  // Draw title.
  TPoint p1, p2;
  GraphR->GetPos(p1, p2);
  p1.x = Disp->Font->Size;
  p1.y = (p1.y + p2.y) / 2;
  UTF8String label;
  int doptype = DopType->ItemIndex;
  if (doptype == 0) {  // All
    label.sprintf("# OF SATELLITES / DOP %s 10 (EL %s %.0f%s)", CHARMUL, CHARGTE, ElMask, CHARDEG);
  } else if (doptype == 1) {  // NSAT
    label.sprintf("# OF SATELLITES (EL %s %.0f%s)", CHARGTE, ElMask, CHARDEG);
  } else {
    label.sprintf("DOP %s 10 (EL %s %.0f%s)", CHARMUL, CHARGTE, ElMask, CHARDEG);
  }
  GraphR->DrawText(p1, label, CColor[2], 0, 0, 90);

  if (!BtnSol1->Down && !BtnSol2->Down) return;

  double *x = new double[sn];
  double *y = new double[sn];
  double *dop = new double[sn * 4];
  int *ns = new int[sn];

  // Calculate DOP. Collecting the azimuth and elevation for all the
  // satellites, and counting the number of satellites.
  gtime_t time = {0, 0};
  int nsat = 0, n = 0, prev_sat = 0;
  double azel[MAXSAT * 2];
  for (int i = 0; i < sn; i++) {
    solstat_t *solstat = SolStat[sel].data + i;
    if (timediff(solstat->time, time) > DTTOL) {
      if (nsat > 0) {
        dops(nsat, azel, ElMask * D2R, dop + n * 4);
        ns[n] = nsat;
        x[n] = TimePos(time);
        n++;
      }
      time = solstat->time;
      nsat = 0;
      prev_sat = 0;
    }
    if (SatMask[solstat->sat - 1] || !SatSel[solstat->sat - 1]) continue;
    if (solstat->el < ElMask * D2R) continue;
    if (ElMaskP && solstat->el < ElMaskData[(int)(solstat->az * R2D + 0.5)]) continue;
    if (solstat->sat != prev_sat && (solstat->flag & 0x20) != 0) {
      azel[nsat * 2] = solstat->az;
      azel[nsat * 2 + 1] = solstat->el;
      prev_sat = solstat->sat;
      nsat++;
    }
  }
  for (int i = 0; i < 4; i++) {  // For all DOP type (GDOP, PDOP, HDOP, VDOP).
    if (doptype != 0 && doptype != i + 2) continue;

    for (int j = 0; j < n; j++) y[j] = dop[i + j * 4] * 10.0;

    if (!(PlotStyle % 2)) {  // Line style.
      DrawPolyS(GraphR, x, y, n, CColor[3], 0);
    }
    if (level && PlotStyle < 2) {  // Marker style.
      for (int j = 0; j < n; j++) {
        if (y[j] == 0.0) continue;
        GraphR->DrawMark(x[j], y[j], 0, MColor[0][i + 2], MarkSize, 0);
      }
    }
  }
  // Draw number of satellites.
  if (doptype == 0 || doptype == 1) {  // ALL or NSAT.
    for (int i = 0; i < n; i++) y[i] = ns[i];

    if (!(PlotStyle % 2)) {  // Line style.
      DrawPolyS(GraphR, x, y, n, CColor[3], 1);
    }
    if (level && PlotStyle < 2) {  // Marker style.
      for (int i = 0; i < n; i++) {
        GraphR->DrawMark(x[i], y[i], 0, MColor[0][1], MarkSize, 0);
      }
    }
  }

  // Draw currently selected data.
  if (BtnShowTrack->Down && 0 <= ind && ind < SolData[sel].n && n > 0) {
    GraphR->GetLim(xl, yl);
    double xp = TimePos(SolData[sel].data[ind].time);
    xl[0] = xl[1] = xp;

    // Vertical line at current position.
    GraphR->DrawPoly(xl, yl, 2, CColor[2], 0);

    int xi = 0;
    for (; xi < n; xi++)
      if (fabs(x[xi] - xp) < DTTOL) break;
    if (xi >= n) xi = n - 1;

    for (int i = 0; i < 4; i++) {  // For all DOP type (GDOP, PDOP, HDOP, VDOP).
      if ((doptype != 0 && doptype != i + 2) || dop[i + xi * 4] <= 0.0) continue;
      GraphR->DrawMark(xl[0], dop[i + xi * 4] * 10.0, 0, MColor[0][i + 2], MarkSize * 2 + 2, 0);
    }
    if (doptype == 0 || doptype == 1) {  // ALL or NSAT.
      GraphR->DrawMark(xl[0], ns[xi], 0, MColor[0][1], MarkSize * 2 + 2, 0);
    }
    GraphR->DrawMark(xl[0], yl[1] - 1E-6, 0, CColor[2], 5, 0);
    if (!BtnFixHoriz->Down) {
      GraphR->DrawMark(xl[0], yl[1] - 1E-6, 1, CColor[2], 9, 0);
    }

    if (ShowStats) {
      AnsiString s0;
      s0.sprintf("NSAT:%d, GDOP:%4.1f PDOP:%4.1f HDOP:%4.1f VDOP:%4.1f", ns[xi], dop[0 + xi * 4],
                 dop[1 + xi * 4], dop[2 + xi * 4], dop[3 + xi * 4]);
      TPoint p1, p2, p3, p4;
      int fonth = (int)(Disp->Font->Size * 1.5);
      GraphR->GetPos(p1, p2);
      p1.x = p2.x - 10;
      p1.y += 8;
      DrawLabel(GraphR, p1, s0, 2, 2);
    }
  } else {
    DrawDopStat(dop, ns, n);
  }
  delete[] x;
  delete[] y;
  delete[] dop;
  delete[] ns;
}
// draw statistics on DOP and number-of-satellite plot ----------------------
void __fastcall TPlot::DrawDopStat(double *dop, int *ns, int n)
{
    AnsiString s0[MAXOBS+2],s1[MAXOBS+2],s2[MAXOBS+2];
    double ave[4]={0};
    int nsat[MAXOBS]={0},ndop[4]={0},m=0;
    
    trace(3,"DrawDopStat: n=%d\n",n);
    
    if (!ShowStats) return;
    
    for (int i=0;i<n;i++) {
        nsat[ns[i]]++;
    }
    for (int i=0;i<4;i++) {
        for (int j=0;j<n;j++) {
            if (dop[i+j*4]<=0.0||dop[i+j*4]>MaxDop) continue;
            ave[i]+=dop[i+j*4];
            ndop[i]++;
        }
        if (ndop[i]>0) ave[i]/=ndop[i];
    }
    if (DopType->ItemIndex==0||DopType->ItemIndex>=2) {
        s2[m++].sprintf("AVE= GDOP:%4.1f PDOP:%4.1f HDOP:%4.1f VDOP:%4.1f",
                        ave[0],ave[1],ave[2],ave[3]);
        s2[m++].sprintf("NDOP=%d(%4.1f%%) %d(%4.1f%%) %d(%4.1f%%) %d(%4.1f%%)",
                        ndop[0],n>0?ndop[0]*100.0/n:0.0,
                        ndop[1],n>0?ndop[1]*100.0/n:0.0,
                        ndop[2],n>0?ndop[2]*100.0/n:0.0,
                        ndop[3],n>0?ndop[3]*100.0/n:0.0);
    }
    if (DopType->ItemIndex<=1) {
        for (int i=0,j=0;i<MAXOBS;i++) {
            if (nsat[i]<=0) continue;
            s0[m].sprintf("%s%2d:",j++==0?"NSAT= ":"",i);
            s1[m].sprintf("%7d",nsat[i]);
            s2[m++].sprintf("(%4.1f%%)",nsat[i]*100.0/n);
        }
    }
    TPoint p1,p2,p3,p4;
    int fonth=(int)(Disp->Font->Size*1.5);
    
    GraphR->GetPos(p1,p2);
    p1.x=p2.x-10;
    p1.y+=8;
    p2=p1; p2.x-=fonth*4;
    p3=p2; p3.x-=fonth*8;
    
    for (int i=0;i<m;i++) {
        DrawLabel(GraphR,p3,s0[i],2,2);
        DrawLabel(GraphR,p2,s1[i],2,2);
        DrawLabel(GraphR,p1,s2[i],2,2);
        p1.y+=fonth;
        p2.y+=fonth;
        p3.y+=fonth;
    }
}
// draw SNR, MP and elevation-plot ---------------------------------------------
void __fastcall TPlot::DrawSnr(int level)
{
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    UTF8String label[]={"SNR","Multipath","Elevation"};
    UTF8String unit[]={"dBHz","m",CHARDEG};
    AnsiString s;
    gtime_t time={0};
    
    trace(3,"DrawSnr: level=%d\n",level);
    
    if (0<=ObsIndex&&ObsIndex<NObs&&BtnShowTrack->Down) {
        time=Obs.data[IndexObs[ObsIndex]].time;
    }
    if (0<=ObsIndex&&ObsIndex<NObs&&BtnShowTrack->Down&&BtnFixHoriz->Down) {
        double xc,yc,xp=TimePos(time);
        double xl[2],yl[2];

        GraphG[0]->GetLim(xl,yl);
        xp-=Xcent*(xl[1]-xl[0])/2.0;
        for (int i=0;i<3;i++) {
            GraphG[i]->GetCent(xc,yc);
            GraphG[i]->SetCent(xp,yc);
        }
    }
    int j=0;
    for (int i=0;i<3;i++) if (btn[i]->Down) j=i;
    for (int i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        GraphG[i]->XLPos=TimeLabel?(i==j?6:5):(i==j?1:0);
        GraphG[i]->Week=Week;
        GraphG[i]->DrawAxis(ShowLabel,ShowLabel);
    }
    if (NObs>0&&BtnSol1->Down) {
        AnsiString text=ObsType2->Text;
        const char *obstype=text.c_str();
        double *x=new double[NObs];
        double *y=new double[NObs];
        TColor *col=new TColor[NObs];
        
        for (int i=0,l=0;i<3;i++) {
            TColor colp[MAXSAT];
            double yp[MAXSAT],ave=0.0,rms=0.0;
            int np=0,nrms=0;
            
            if (!btn[i]->Down) continue;
            
            for (int sat=1,np=0;sat<=MAXSAT;sat++) {
                if (SatMask[sat-1]||!SatSel[sat-1]) continue;
                int n=0;
                for (int j=0;j<Obs.n;j++) {
                    obsd_t *obs=Obs.data+j;
                    int k,freq;
                    if (obs->sat!=sat) continue;
                    if (sscanf(obstype,"L%1d",&freq)==1) {
                        k=freq>2?freq-3:freq-1;
                    }
                    else {
                        for (k=0;k<NFREQ+NEXOBS;k++) {
                            if (!strcmp(code2obs(obs->code[k]),obstype)) break;
                        }
                        if (k>=NFREQ+NEXOBS) continue;
                    }
                    if (obs->SNR[k]<=0.0) continue;
                    
                    x[n]=TimePos(obs->time);
                    if (i==0) {
                        y[n]=obs->SNR[k];
                        col[n]=MColor[0][4];
                    }
                    else if (i==1) {
                        if (!Mp[k]||Mp[k][j]==0.0) continue;
                        y[n]=Mp[k][j];
                        col[n]=MColor[0][4];
                    }
                    else {
                        y[n]=El[j]*R2D;
                        if (SimObs) col[n]=SysColor(obs->sat);
                        else col[n]=SnrColor(obs->SNR[k]);
                        if (El[j]>0.0&&El[j]<ElMask*D2R) col[n]=MColor[0][0];
                    }
                    if (timediff(time,obs->time)==0.0&&np<MAXSAT) {
                        yp[np]=y[n];
                        colp[np++]=col[n];
                    }
                    if (n<NObs) n++;
                }
                if (!level||!(PlotStyle%2)) {
                    for (int j=0,k=0;j<n;j=k) {
                        for (k=j+1;k<n;k++) if (fabs(y[k-1]-y[k])>30.0) break;
                        DrawPolyS(GraphG[i],x+j,y+j,k-j,CColor[3],0);
                    }
                }
                if (level&&PlotStyle<2) {
                    for (int j=0;j<n;j++) {
                        if (i!=1&&y[j]<=0.0) continue;
                        GraphG[i]->DrawMark(x[j],y[j],0,col[j],MarkSize,0);
                    }
                }
                for (int j=0;j<n;j++) {
                    if (y[j]==0.0) continue;
                    ave+=y[j];
                    rms+=SQR(y[j]);
                    nrms++;
                }
            }
            if (level&&i==1&&nrms>0&&ShowStats&&!BtnShowTrack->Down) {
                TPoint p1,p2;
                ave=ave/nrms;
                rms=SQRT(rms/nrms);
                GraphG[i]->GetPos(p1,p2);
                p1.x=p2.x-8; p1.y+=3;
                DrawLabel(GraphG[i],p1,s.sprintf("AVE=%.4fm RMS=%.4fm",ave,rms),2,2);
            }
            if (BtnShowTrack->Down&&0<=ObsIndex&&ObsIndex<NObs&&BtnSol1->Down) {
                if (!btn[i]->Down) continue;
                TPoint p1,p2;
                double xl[2],yl[2];
                GraphG[i]->GetLim(xl,yl);
                xl[0]=xl[1]=TimePos(time);
                GraphG[i]->DrawPoly(xl,yl,2,CColor[2],0);
                
                if (l++==0) {
                    GraphG[i]->DrawMark(xl[0],yl[1]-1E-6,0,CColor[2],5,0);
                    
                    if (!BtnFixHoriz->Down) {
                        GraphG[i]->DrawMark(xl[0],yl[1]-1E-6,1,CColor[2],9,0);
                    }
                }
                for (int k=0;k<np;k++) {
                    if (i!=1&&yp[k]<=0.0) continue;
                    GraphG[i]->DrawMark(xl[0],yp[k],0,CColor[0],MarkSize*2+4,0);
                    GraphG[i]->DrawMark(xl[0],yp[k],0,colp[k],MarkSize*2+2,0);
                }
                if (np<=0||np>1||(i!=1&&yp[0]<=0.0)) continue;
                GraphG[i]->GetPos(p1,p2);
                p1.x=p2.x-8; p1.y+=3;
                DrawLabel(GraphG[i],p1,s.sprintf("%.*f %s",i==1?4:1,yp[0],unit[i].c_str()),2,2);
            }
        }
        delete [] x;
        delete [] y;
        delete [] col;
    }
    for (int i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        TPoint p1,p2;
        GraphG[i]->GetPos(p1,p2);
        p1.x+=5; p1.y+=3;
        DrawLabel(GraphG[i],p1,s.sprintf("%s (%s)",label[i].c_str(),unit[i].c_str()),1,2);
    }
}
// draw SNR/MP to evation plot ----------------------------------------------
void __fastcall TPlot::DrawSnrE(int level)
{
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    UTF8String label[]={"SNR (dBHz)","Multipath (m)"};
    AnsiString s;
    gtime_t time={0};
    double ave=0.0,rms=0.0;
    int nrms=0;
    
    trace(3,"DrawSnrE: level=%d\n",level);
    
    int j=0;
    for (int i=0;i<2;i++) if (btn[i]->Down) j=i;
    for (int i=0;i<2;i++) {
        TPoint p1,p2;
        double xl[2]={-0.001,90.0},yl[2][2]={{10.0,65.0},{-MaxMP,MaxMP}};
        
        if (!btn[i]->Down) continue;
        GraphE[i]->XLPos=i==j?1:0;
        GraphE[i]->YLPos=1;
        GraphE[i]->SetLim(xl,yl[i]);
        GraphE[i]->SetTick(0.0,0.0);
        GraphE[i]->DrawAxis(1,1);
        GraphE[i]->GetPos(p1,p2);
        p1.x=Disp->Font->Size;
        p1.y=(p1.y+p2.y)/2;
        GraphE[i]->DrawText(p1,label[i],CColor[2],0,0,90);
        if (i==j) {
            UTF8String ustr="Elevation (" CHARDEG ")";
            p2.x-=8; p2.y-=6;
            GraphE[i]->DrawText(p2,ustr,CColor[2],2,1,0);
        }
    }
    if (0<=ObsIndex&&ObsIndex<NObs&&BtnShowTrack->Down) {
        time=Obs.data[IndexObs[ObsIndex]].time;
    }
    if (NObs>0&&BtnSol1->Down) {
        TColor *col[2],colp[2][MAXSAT];
        AnsiString text=ObsType2->Text;
        const char *obstype=text.c_str();
        double *x[2],*y[2],xp[2][MAXSAT],yp[2][MAXSAT];
        int n[2],np[2]={0};

        for (int i=0;i<2;i++) {
            x[i]=new double[NObs],
            y[i]=new double[NObs];
            col[i]=new TColor[NObs];
        }
        for (int sat=1;sat<=MAXSAT;sat++) {
            if (SatMask[sat-1]||!SatSel[sat-1]) continue;
            n[0]=n[1]=0;
            
            for (int j=0;j<Obs.n;j++) {
                obsd_t *obs=Obs.data+j;
                int k,freq;
                
                if (obs->sat!=sat||El[j]<=0.0) continue;
                
                if (sscanf(obstype,"L%1d",&freq)==1) {
                    k=freq>2?freq-3:freq-1;  /* L1,L2,L5,L6 ... */
                }
                else {
                    for (k=0;k<NFREQ+NEXOBS;k++) {
                        if (!strcmp(code2obs(obs->code[k]),obstype)) break;
                    }
                    if (k>=NFREQ+NEXOBS) continue;
                }
                if (obs->SNR[k]<=0.0) continue;

                x[0][n[0]]=x[1][n[1]]=El[j]*R2D;
                y[0][n[0]]=obs->SNR[k];
                y[1][n[1]]=!Mp[k]?0.0:Mp[k][j];
                
                col[0][n[0]]=col[1][n[1]]=
                    El[j]>0.0&&El[j]<ElMask*D2R?MColor[0][0]:MColor[0][4];
                
                if (y[0][n[0]]>0.0) {
                    if (timediff(time,Obs.data[j].time)==0.0) {
                        xp[0][np[0]]=x[0][n[0]];
                        yp[0][np[0]]=y[0][n[0]];
                        colp[0][np[0]]=ObsColor(Obs.data+j,Az[j],El[j]);
                        if (np[0]<MAXSAT&&colp[0][np[0]]!=clBlack) np[0]++;
                    }
                    if (n[0]<NObs) n[0]++;
                }
                if (y[1][n[1]]!=0.0) {
                    if (El[j]>=ElMask*D2R) {
                        ave+=y[1][n[1]];
                        rms+=SQR(y[1][n[1]]);
                        nrms++;
                    }
                    if (timediff(time,Obs.data[j].time)==0.0) {
                        xp[1][np[1]]=x[1][n[1]];
                        yp[1][np[1]]=y[1][n[1]];
                        colp[1][np[1]]=ObsColor(Obs.data+j,Az[j],El[j]);
                        if (np[1]<MAXSAT&&colp[1][np[1]]!=clBlack) np[1]++;
                    }
                    if (n[1]<NObs) n[1]++;
                }
            }
            if (!level||!(PlotStyle%2)) {
                for (int i=0;i<2;i++) {
                    if (!btn[i]->Down) continue;
                    DrawPolyS(GraphE[i],x[i],y[i],n[i],CColor[3],0);
                }
            }
            if (level&&PlotStyle<2) {
                for (int i=0;i<2;i++) {
                    if (!btn[i]->Down) continue;
                    for (int j=0;j<n[i];j++) {
                        GraphE[i]->DrawMark(x[i][j],y[i][j],0,col[i][j],MarkSize,0);
                    }
                }
            }
        }
        for (int i=0;i<2;i++) {
            delete [] x[i];
            delete [] y[i];
            delete [] col[i];
        }
        if (BtnShowTrack->Down&&0<=ObsIndex&&ObsIndex<NObs&&BtnSol1->Down) {
            for (int i=0;i<2;i++) {
                if (!btn[i]->Down) continue;
                for (int j=0;j<np[i];j++) {
                    GraphE[i]->DrawMark(xp[i][j],yp[i][j],0,CColor[0],MarkSize*2+8,0);
                    GraphE[i]->DrawMark(xp[i][j],yp[i][j],1,CColor[2],MarkSize*2+6,0);
                    GraphE[i]->DrawMark(xp[i][j],yp[i][j],0,colp[i][j],MarkSize*2+2,0);
                }
            }
        }
    }
    if (ShowStats) {
        int i;
        for (i=0;i<2;i++) if (btn[i]->Down) break;
        if (i<2) {
            TPoint p1,p2;
            int hh=(int)(Disp->Font->Size*1.5);
            
            GraphE[i]->GetPos(p1,p2);
            p1.x+=8; p1.y+=6;
            s.sprintf("MARKER: %s %s %s",Sta.name,Sta.markerno,Sta.markertype);
            DrawLabel(GraphE[i],p1,s,1,2); p1.y+=hh;
            s.sprintf("REC: %s %s %s",Sta.rectype,Sta.recver,Sta.recsno);
            DrawLabel(GraphE[i],p1,s,1,2); p1.y+=hh;
            s.sprintf("ANT: %s %s",Sta.antdes,Sta.antsno);
            DrawLabel(GraphE[i],p1,s,1,2); p1.y+=hh;
        }
        if (btn[1]->Down&&nrms>0&&!BtnShowTrack->Down) {
            TPoint p1,p2;
            ave=ave/nrms;
            rms=SQRT(rms/nrms);
            GraphE[1]->GetPos(p1,p2);
            p1.x=p2.x-8; p1.y+=6;
            DrawLabel(GraphE[1],p1,s.sprintf("AVE=%.4fm RMS=%.4fm",ave,rms),2,2);
        }
    }
}
// draw MP-skyplot ----------------------------------------------------------
void __fastcall TPlot::DrawMpS(int level)
{
    AnsiString text=ObsType2->Text;
    const char *obstype=text.c_str();
    double r,xl[2],yl[2],xs,ys;
    
    trace(3,"DrawSnrS: level=%d\n",level);
    
    GraphS->GetLim(xl,yl);
    r=(xl[1]-xl[0]<yl[1]-yl[0]?xl[1]-xl[0]:yl[1]-yl[0])*0.45;
    
    if (BtnShowImg->Down) {
        DrawSkyImage(level);
    }
    if (BtnShowSkyplot->Down) {
        GraphS->DrawSkyPlot(0.0,0.0,CColor[1],CColor[2],CColor[0],r*2.0);
    }
    if (!BtnSol1->Down||NObs<=0||SimObs) return;
    
    GraphS->GetScale(xs,ys);
    
    for (int sat=1;sat<=MAXSAT;sat++) {
        double p[MAXSAT][2]={{0}};
        
        if (SatMask[sat-1]||!SatSel[sat-1]) continue;
        
        for (int i=0;i<Obs.n;i++) {
            obsd_t *obs=Obs.data+i;
            int j,freq;

            if (obs->sat!=sat||El[i]<=0.0) continue;
            
            if (sscanf(obstype,"L%1d",&freq)==1) {
                j=freq>2?freq-3:freq-1;  /* L1,L2,L5,L6 ... */
            }
            else {
                for (j=0;j<NFREQ+NEXOBS;j++) {
                    if (!strcmp(code2obs(obs->code[j]),obstype)) break;
                }
                if (j>=NFREQ+NEXOBS) continue;
            }
            double x=r*sin(Az[i])*(1.0-2.0*El[i]/PI);
            double y=r*cos(Az[i])*(1.0-2.0*El[i]/PI);
            double xp=p[sat-1][0];
            double yp=p[sat-1][1];
            TColor col=MpColor(!Mp[j]?0.0:Mp[j][i]);
            
            if ((x-xp)*(x-xp)+(y-yp)*(y-yp)>=xs*xs) {
                int siz=PlotStyle<2?MarkSize:1;
                GraphS->DrawMark(x,y,0,col,siz,0);
                GraphS->DrawMark(x,y,0,PlotStyle<2?col:CColor[3],siz,0);
                p[sat-1][0]=x;
                p[sat-1][1]=y;
            }
        }
    }
    if (BtnShowTrack->Down&&0<=ObsIndex&&ObsIndex<NObs) {
        for (int i=IndexObs[ObsIndex];i<Obs.n&&i<IndexObs[ObsIndex+1];i++) {
            obsd_t *obs=Obs.data+i;
            int j,freq;
            
            if (SatMask[obs->sat-1]||!SatSel[obs->sat-1]||El[i]<=0.0) continue;
            
            if (sscanf(obstype,"L%1d",&freq)==1) {
                j=freq>2?freq-3:freq-1;  /* L1,L2,L5,L6 ... */
            }
            else {
                for (j=0;j<NFREQ+NEXOBS;j++) {
                    if (!strcmp(code2obs(obs->code[j]),obstype)) break;
                }
                if (j>=NFREQ+NEXOBS) continue;
            }
            TColor col=MpColor(!Mp[j]?0.0:Mp[j][i]);
            double x=r*sin(Az[i])*(1.0-2.0*El[i]/PI);
            double y=r*cos(Az[i])*(1.0-2.0*El[i]/PI);
            UTF8String ustr;
            char id[8];
            satno2id(obs->sat,id);
            ustr=id;
            GraphS->DrawMark(x,y,0,col,Disp->Font->Size*2+5,0);
            GraphS->DrawMark(x,y,1,CColor[2],Disp->Font->Size*2+5,0);
            GraphS->DrawText(x,y,ustr,CColor[0],0,0,0);
        }
    }
}
// draw residuals and SNR/elevation plot ------------------------------------
void __fastcall TPlot::DrawRes(int level)
{
    UTF8String label[]={
        "Pseudorange Residuals (m)", "Carrier-Phase Residuals (m)",
        "Elevation Angle (" CHARDEG ") / SNR (dBHz)"
    };
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    int sel=!BtnSol1->Down&&BtnSol2->Down?1:0,ind=SolIndex[sel];
    int frq=FrqType->ItemIndex+1,n=SolStat[sel].n;
    
    trace(3,"DrawRes: level=%d\n",level);
    
    if (0<=ind&&ind<SolData[sel].n&&BtnShowTrack->Down&&BtnFixHoriz->Down) {
        gtime_t t=SolData[sel].data[ind].time;
        
        for (int i=0;i<3;i++) {
            double off,xc,yc,xl[2],yl[2];
            
            if (BtnFixHoriz->Down) {
                GraphG[i]->GetLim(xl,yl);
                off=Xcent*(xl[1]-xl[0])/2.0;
                GraphG[i]->GetCent(xc,yc);
                GraphG[i]->GetCent(xc,yc);
                GraphG[i]->SetCent(TimePos(t)-off,yc);
            }
            else {
                GraphG[i]->GetRight(xc,yc);
                GraphG[i]->SetRight(TimePos(t),yc);
            }
        }
    }
    int j=-1;
    for (int i=0;i<3;i++) if (btn[i]->Down) j=i;
    for (int i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        GraphG[i]->XLPos=TimeLabel?(i==j?6:5):(i==j?1:0);
        GraphG[i]->Week=Week;
        GraphG[i]->DrawAxis(ShowLabel,ShowLabel);
    }
    if (n>0&&((sel==0&&BtnSol1->Down)||(sel==1&&BtnSol2->Down))) {
        TColor *col[4];
        double *x[4],*y[4],res[2],sum[2]={0},sum2[2]={0};
        int ns[2]={0};
        
        for (int i=0;i<4;i++) {
            x[i]=new double[n],
            y[i]=new double[n];
            col[i]=new TColor[n];
        } 
        for (int sat=1;sat<=MAXSAT;sat++) {
            if (SatMask[sat-1]||!SatSel[sat-1]) continue;
            
            int m[4]={0};
            
            for (int i=0;i<n;i++) {
                solstat_t *p=SolStat[sel].data+i;
                int q;
                
                if (p->sat!=sat||p->frq!=frq) continue;
                
                x[0][m[0]]=x[1][m[1]]=x[2][m[2]]=x[3][m[3]]=TimePos(p->time);
                y[0][m[0]]=p->resp;
                y[1][m[1]]=p->resc;
                y[2][m[2]]=p->el*R2D;
                y[3][m[3]]=p->snr;
                if      (!(p->flag>>5))  q=0; // invalid
                else if ((p->flag&7)<=1) q=2; // float
                else if ((p->flag&7)<=3) q=1; // fixed
                else                     q=6; // ppp
                
                col[0][m[0]]=MColor[0][q];
                col[1][m[1]]=((p->flag>>3)&1)?clRed:MColor[0][q];
                col[2][m[2]]=MColor[0][1];
                col[3][m[3]]=MColor[0][4];
                
                if (p->resp!=0.0) {
                    sum [0]+=p->resp;
                    sum2[0]+=p->resp*p->resp;
                    ns[0]++;
                }
                if (p->resc!=0.0) {
                    sum [1]+=p->resc;
                    sum2[1]+=p->resc*p->resc;
                    ns[1]++;
                }
                m[0]++; m[1]++; m[2]++; m[3]++;
            }
            for (int i=0;i<3;i++) {
                if (!btn[i]->Down) continue;
                if (!level||!(PlotStyle%2)) {
                    DrawPolyS(GraphG[i],x[i],y[i],m[i],CColor[3],0);
                    if (i==2) DrawPolyS(GraphG[i],x[3],y[3],m[3],CColor[3],0);
                }
                if (level&&PlotStyle<2) {
                    for (int j=0;j<m[i];j++) {
                        GraphG[i]->DrawMark(x[i][j],y[i][j],0,col[i][j],MarkSize,0);
                    }
                    if (i==2) {
                        for (int j=0;j<m[3];j++) {
                            GraphG[i]->DrawMark(x[3][j],y[3][j],0,col[3][j],MarkSize,0);
                        }
                    }
                }
            }
        }
        for (int i=0;i<4;i++) {
            delete [] x[i];
            delete [] y[i];
            delete [] col[i];
        }
        if (ShowStats) {
            for (int i=0;i<2;i++) {
                if (!btn[i]->Down) continue;
                
                AnsiString str;
                TPoint p1,p2;
                double ave,std,rms;
                
                ave=ns[i]<=0?0.0:sum[i]/ns[i];
                std=ns[i]<=1?0.0:SQRT((sum2[i]-2.0*sum[i]*ave+ns[i]*ave*ave)/(ns[i]-1));
                rms=ns[i]<=0?0.0:SQRT(sum2[i]/ns[i]);
                GraphG[i]->GetPos(p1,p2);
                p1.x=p2.x-5;
                p1.y+=3;
                str.sprintf("AVE=%.3fm STD=%.3fm RMS=%.3fm",ave,std,rms);
                DrawLabel(GraphG[i],p1,str,2,2);
            }
        }
        if (BtnShowTrack->Down&&0<=ind&&ind<SolData[sel].n&&(BtnSol1->Down||BtnSol2->Down)) {
            for (int i=0,j=0;i<3;i++) {
                if (!btn[i]->Down) continue;
                
                gtime_t t=SolData[sel].data[ind].time;
                double xl[2],yl[2];
                
                GraphG[i]->GetLim(xl,yl);
                xl[0]=xl[1]=TimePos(t);
                GraphG[i]->DrawPoly(xl,yl,2,ind==0?CColor[1]:CColor[2],0);
                if (j++==0) {
                    GraphG[i]->DrawMark(xl[0],yl[1]-1E-6,0,CColor[2],5,0);
                    GraphG[i]->DrawMark(xl[0],yl[1]-1E-6,1,CColor[2],9,0);
                }
            }
        }
    }
    for (int i=0;i<3;i++) {
        if (!btn[i]->Down) continue;
        TPoint p1,p2;
        
        GraphG[i]->GetPos(p1,p2);
        p1.x+=5; p1.y+=3;
        DrawLabel(GraphG[i],p1,label[i],1,2);
    }
}
// draw residuals - elevation plot ------------------------------------------
void __fastcall TPlot::DrawResE(int level)
{
    TSpeedButton *btn[]={BtnOn1,BtnOn2,BtnOn3};
    UTF8String label[]={"Pseudorange Residuals (m)","Carrier-Phase Residuals (m)"};
    int j,sel=!BtnSol1->Down&&BtnSol2->Down?1:0,ind=SolIndex[sel];
    int frq=FrqType->ItemIndex+1,n=SolStat[sel].n;
    
    trace(3,"DrawResE: level=%d\n",level);
    
    j=0;
    for (int i=0;i<2;i++) if (btn[i]->Down) j=i;
    for (int i=0;i<2;i++) {
        if (!btn[i]->Down) continue;

        TPoint p1,p2;
        double xl[2]={-0.001,90.0};
        double yl[2][2]={{-MaxMP,MaxMP},{-MaxMP/100.0,MaxMP/100.0}};
        
        GraphE[i]->XLPos=i==j?1:0;
        GraphE[i]->YLPos=1;
        GraphE[i]->SetLim(xl,yl[i]);
        GraphE[i]->SetTick(0.0,0.0);
        GraphE[i]->DrawAxis(1,1);
        GraphE[i]->GetPos(p1,p2);
        p1.x=Disp->Font->Size;
        p1.y=(p1.y+p2.y)/2;
        GraphE[i]->DrawText(p1,label[i],CColor[2],0,0,90);
        if (i==j) {
            UTF8String ustr="Elevation (" CHARDEG ")";
            p2.x-=8; p2.y-=6;
            GraphE[i]->DrawText(p2,ustr,CColor[2],2,1,0);
        }                
    }
    if (n>0&&((sel==0&&BtnSol1->Down)||(sel==1&&BtnSol2->Down))) {
        TColor *col[2];
        double *x[2],*y[2],sum[2]={0},sum2[2]={0};
        int ns[2]={0};

        for (int i=0;i<2;i++) {
            x  [i]=new double[n],
            y  [i]=new double[n];
            col[i]=new TColor[n];
        }
        for (int sat=1;sat<=MAXSAT;sat++) {
            if (SatMask[sat-1]||!SatSel[sat-1]) continue;
            int q,m[2]={0};

            for (int i=0;i<n;i++) {
                solstat_t *p=SolStat[sel].data+i;
                if (p->sat!=sat||p->frq!=frq) continue;
                 
                x[0][m[0]]=x[1][m[1]]=p->el*R2D;
                y[0][m[0]]=p->resp;
                y[1][m[1]]=p->resc;
                if      (!(p->flag>>5))  q=0; // invalid
                else if ((p->flag&7)<=1) q=2; // float
                else if ((p->flag&7)<=3) q=1; // fixed
                else                     q=6; // ppp
                
                col[0][m[0]]=MColor[0][q];
                col[1][m[1]]=((p->flag>>3)&1)?clRed:MColor[0][q];
                
                if (p->resp!=0.0) {
                    sum [0]+=p->resp;
                    sum2[0]+=p->resp*p->resp;
                    ns[0]++;
                }
                if (p->resc!=0.0) {
                    sum [1]+=p->resc;
                    sum2[1]+=p->resc*p->resc;
                    ns[1]++;
                }
                m[0]++; m[1]++;
            }
            for (int i=0;i<2;i++) {
                if (!btn[i]->Down) continue;
                if (!level||!(PlotStyle%2)) {
                    DrawPolyS(GraphE[i],x[i],y[i],m[i],CColor[3],0);
                }
                if (level&&PlotStyle<2) {
                    for (int j=0;j<m[i];j++) {
                        GraphE[i]->DrawMark(x[i][j],y[i][j],0,col[i][j],MarkSize,0);
                    }
                }
            }
        }
        for (int i=0;i<2;i++) {
            delete [] x[i];
            delete [] y[i];
            delete [] col[i];
        }
        if (ShowStats) {
            for (int i=0;i<2;i++) {
                if (!btn[i]->Down) continue;
                
                AnsiString str;
                TPoint p1,p2;
                double ave,std,rms;

                ave=ns[i]<=0?0.0:sum[i]/ns[i];
                std=ns[i]<=1?0.0:SQRT((sum2[i]-2.0*sum[i]*ave+ns[i]*ave*ave)/(ns[i]-1));
                rms=ns[i]<=0?0.0:SQRT(sum2[i]/ns[i]);
                GraphE[i]->GetPos(p1,p2);
                p1.x=p2.x-5;
                p1.y+=3;
                str.sprintf("AVE=%.3fm STD=%.3fm RMS=%.3fm",ave,std,rms);
                DrawLabel(GraphG[i],p1,str,2,2);
            }
        }
    }
}
// draw polyline without time-gaps ------------------------------------------
void __fastcall TPlot::DrawPolyS(TGraph *graph, double *x, double *y, int n,
    TColor color, int style)
{
    int i,j;
    
    for (i=0;i<n;i=j) {
        for (j=i+1;j<n;j++) if (fabs(x[j]-x[j-1])>TBRK) break;
        graph->DrawPoly(x+i,y+i,j-i,color,style);
    }
}
// draw label with hemming --------------------------------------------------
void __fastcall TPlot::DrawLabel(TGraph *g, TPoint p, UTF8String label, int ha,
    int va)
{
    g->DrawText(p,label,CColor[2],CColor[0],ha,va,0);
}
// draw mark with hemming ---------------------------------------------------
void __fastcall TPlot::DrawMark(TGraph *g, TPoint p, int mark, TColor color,
    int size, int rot)
{
    g->DrawMark(p,mark,color,CColor[0],size,rot);
}
// refresh map view ---------------------------------------------------------
void __fastcall TPlot::Refresh_MapView(void)
{
    sol_t *sol;
    double pos[3]={0};
    
    if (BtnShowTrack->Down) {
        
        if (BtnSol2->Down&&SolData[1].n>0&&
            (sol=getsol(SolData+1,SolIndex[1]))) {
            ecef2pos(sol->rr,pos);
            MapView->SetMark(2,pos[0]*R2D,pos[1]*R2D);
            MapView->ShowMark(2);
        }
        else {
            MapView->HideMark(2);
        }
        if (BtnSol1->Down&&SolData[0].n>0&&
            (sol=getsol(SolData,SolIndex[0]))) {
            ecef2pos(sol->rr,pos);
            MapView->SetMark(1,pos[0]*R2D,pos[1]*R2D);
            MapView->ShowMark(1);
        }
        else {
            MapView->HideMark(1);
        }
    }
    else {
        MapView->HideMark(1);
        MapView->HideMark(2);
    }
}
