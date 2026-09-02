/*-----------------------------------------------------------------------------
* merge log and tag files
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include "rtklib.h"

#define HEADLEN			76

/* main ----------------------------------------------------------------------*/
int main(int argc, char **argv) {
  FILE *ifp,*itagfp,*ofp,*otagfp;
  gtime_t time1;
  char ifiles[32]={},*ofile="";
  char itagfile[1024],otagfile[1024];
  int i,n=0;
  uint32_t tick0,tick1,tick,fpos;
  uint8_t buff[4096],tagbuff[64];
  char time_str[40];

  for (i=0;i<argc;i++) {
    if (!strcmp(argv[i],"-o")&&i+1<argc) ofile=argv[++i];
    else ifiles[n++]=argv[i];
  }
  sprintf(otagfile,"%s.tag",ofile);

  if (!(ofp   =fopen(ofile   ,"wb"))||
      !(otagfp=fopen(otagfile,"wb"))) {
    fprintf(stderr,"out file open error: %s\n",ofile);
    return -1;
  }
  for (i=0;i<n;i++) {
    sprintf(itagfile,"%s.tag",ifiles[i]);

    if (!(ifp   =fopen(ifiles[i],"rb"))||
        !(itagfp=fopen(itagfile ,"rb"))) {
      fprintf(stderr,"in file open error: %s\n",ifiles[i]);
      return -1;
    }
    if (fread(tagbuff,HEADLEN,1,itagfp)) {
      fprintf(stderr,"in tag file read error\n");
      return -1;
    }
    tick1=*(uint32_t *)(tagbuff+60);
    time1=*(gtime_t  *)(tagbuff+64);
    time2str(time1,time_str,0);
    fprintf(stderr,"tick=%8u: t=%s %s\n",tick1,time_str,ifiles[i]);

    if (i==0) {
      if (fwrite(tagbuff,HEADLEN,1,otagfp)) {
        fprintf(stderr,"out tag file write error\n");
        return -1;
      }
      tick0=tick1;
    }
    for (fpos=0;fread(tagbuff,8,1,itagfp)==1;) {
      tick=*(uint32_t *)tagbuff+tick1;
      fpos=*(uint32_t *)(tagbuff+4);

      fprintf(stderr,"tick=%8u: fpos=%8u\n",tick,fpos);

      fread (buff,sizeof(buff),1,ifp);
      fwrite(buff,sizeof(buff),1,ofp);

      /*fwrite(buff,len,ofp);*/
    }
    fclose(ifp); fclose(itagfp);
  }
  fclose(ofp); fclose(otagfp);

  return 0;
}
