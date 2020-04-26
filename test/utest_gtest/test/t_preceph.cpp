/*------------------------------------------------------------------------------
* rtklib unit test driver : precise ephemeris function
*-----------------------------------------------------------------------------*/
#include "gtest/gtest.h"
#include "rtklib.h"
#include "datadir.h"

/* readsp3() */
TEST(TEST_precise_ephemeris,test_readsp3)
{
#ifdef WIN32
   string file1 = TestData::ins().full_path_filename("..\\data\\sp3\\igs15904.sp4");
   string file2 = TestData::ins().full_path_filename("..\\data\\sp3\\igs15904.sp3");
   string file3 = TestData::ins().full_path_filename("..\\data\\sp3\\igs1590*.sp3");
#else
    string file1 = TestData::ins().full_path_filename("../data/sp3/igs15904.sp4");
    string file2 = TestData::ins().full_path_filename("../data/sp3/igs15904.sp3");
    string file3 = TestData::ins().full_path_filename("../data/sp3/igs1590*.sp3");
#endif
    nav_t nav={0};
    
    readsp3(file1.c_str(),&nav,0);
        EXPECT_TRUE(nav.ne==0);
    
    readsp3(file2.c_str(),&nav,0);
        EXPECT_TRUE(nav.ne==96);
    
    readsp3(file3.c_str(),&nav,0);
        EXPECT_TRUE(nav.ne==192);
}
/* readsap() */
TEST(TEST_precise_ephemeris, test_readsap)
{
    double ep1[]={2008,3,1,0,0,0};
    double ep2[]={2006,11,4,23,59,59};
#ifdef WIN32
    string file1 = TestData::ins().full_path_filename("..\\data\\sp3\\igs06.atx");
    string file2 = TestData::ins().full_path_filename("..\\..\\data\\igs05.atx");
#else
    string file1 = TestData::ins().full_path_filename("../data/sp3/igs06.atx");
    string file2 = TestData::ins().full_path_filename("../../data/igs05.atx");
#endif
    pcvs_t pcvs={0};
    pcv_t *pcv;
    gtime_t time;
    int i,stat;
    
    stat=readpcv(file1.c_str(),&pcvs);
        EXPECT_TRUE(!stat);
    stat=readpcv(file2.c_str(),&pcvs);
        EXPECT_TRUE(stat);
    
    time=epoch2time(ep1);
    for (i=0;i<MAXSAT;i++) {
        if (!(pcv=searchpcv(i+1,"",time,&pcvs))) continue;
    }
    time=epoch2time(ep2);
    for (i=0;i<MAXSAT;i++) {
        if (!(pcv=searchpcv(i+1,"",time,&pcvs))) continue;
    }
}
/* readrnxc() */
TEST(TEST_precise_ephemeris, test_rnxc)
{
#ifdef WIN32
   string file1 = TestData::ins().full_path_filename("..\\data\\sp3\\igs15904.cls");
   string file2 = TestData::ins().full_path_filename("..\\data\\sp3\\igs15904.clk");
   string file3 = TestData::ins().full_path_filename("..\\data\\sp3\\igs1590*.clk");
#else
    string file1 = TestData::ins().full_path_filename("../data/sp3/igs15904.cls");
    string file2 = TestData::ins().full_path_filename("../data/sp3/igs15904.clk");
    string file3 = TestData::ins().full_path_filename("../data/sp3/igs1590*.clk");
#endif
    nav_t nav={0};
    
    readrnxc(file1.c_str(),&nav);
        EXPECT_TRUE(nav.nc==0);
    
    readrnxc(file2.c_str(),&nav);
        EXPECT_TRUE(nav.nc>0);
    free(nav.pclk); nav.pclk=NULL; nav.nc=nav.ncmax=0;
    
    readrnxc(file3.c_str(),&nav);
        EXPECT_TRUE(nav.nc>0);
    free(nav.pclk); nav.pclk=NULL; nav.nc=nav.ncmax=0;
    
}
/* peph2pos() */
TEST(TEST_precise_ephemeris, test_peph2pos)
{
    FILE *fp;
#ifdef WIN32
    string file1 = TestData::ins().full_path_filename("..\\data\\sp3\\igs1590*.sp3"); /* 2010/7/1 */
    string file2 = TestData::ins().full_path_filename("..\\data\\sp3\\igs1590*.clk"); /* 2010/7/1 */
#else
    string file1 = TestData::ins().full_path_filename("../data/sp3/igs1590*.sp3"); /* 2010/7/1 */
    string file2 = TestData::ins().full_path_filename("../data/sp3/igs1590*.clk"); /* 2010/7/1 */
#endif
    nav_t nav={0};
    int i,j,stat,sat;
    double ep[]={2010,7,1,0,0,0};
    double rs[6]={0},dts[2]={0};
    double var;
    gtime_t t,time;
    
    time=epoch2time(ep);
    
    readsp3(file1.c_str(),&nav,0);
        EXPECT_TRUE(nav.ne>0);
    readrnxc(file2.c_str(),&nav);
        EXPECT_TRUE(nav.nc>0);
    stat=peph2pos(time,0,&nav,0,rs,dts,&var);
        EXPECT_TRUE(!stat);
    stat=peph2pos(time,160,&nav,0,rs,dts,&var);
        EXPECT_TRUE(!stat);
    
    fp=fopen("testpeph1.out","w");
    
    sat=4;
    
    for (i=0;i<86400*2;i+=30) {
        t=timeadd(time,(double)i);
        for (j=0;j<6;j++) rs [j]=0.0;
        for (j=0;j<2;j++) dts[j]=0.0;
        peph2pos(t,sat,&nav,0,rs,dts,&var);
    }
    fclose(fp);
}
/* satpos() */
TEST(TEST_precise_ephemeris, test_satpos)
{
    FILE *fp;
#ifdef WIN32
    string file1 = TestData::ins().full_path_filename("..\\data\\sp3\\igs1590*.sp3"); /* 2010/7/1 */
    string file2 = TestData::ins().full_path_filename("..\\data\\sp3\\igs1590*.clk"); /* 2010/7/1 */
    string file3 = TestData::ins().full_path_filename("..\\..\\data\\igs05.atx");
    string file4 = TestData::ins().full_path_filename("..\\data\\rinex\\brdc*.10n");
#else
    string file1 = TestData::ins().full_path_filename("../data/sp3/igs1590*.sp3"); /* 2010/7/1 */
    string file2 = TestData::ins().full_path_filename("../data/sp3/igs1590*.clk"); /* 2010/7/1 */
    string file3 = TestData::ins().full_path_filename("../../data/igs05.atx");
    string file4 = TestData::ins().full_path_filename("../data/rinex/brdc*.10n");
#endif
    pcvs_t pcvs={0};
    pcv_t *pcv;
    nav_t nav={0};
    int i,stat,sat,svh;
    double ep[]={2010,7,1,0,0,0};
    double rs1[6]={0},dts1[2]={0},rs2[6]={0},dts2[2]={0};
    double var;
    gtime_t t,time;
    
    time=epoch2time(ep);
    
    readsp3(file1.c_str(),&nav,0);
        EXPECT_TRUE(nav.ne>0);
    readrnxc(file2.c_str(),&nav);
        EXPECT_TRUE(nav.nc>0);
    stat=readpcv(file3.c_str(),&pcvs);
        EXPECT_TRUE(stat);
    readrnx(file4.c_str(),1,"",NULL,&nav,NULL);
        EXPECT_TRUE(nav.n>0);
    for (i=0;i<MAXSAT;i++) {
        if (!(pcv=searchpcv(i+1,"",time,&pcvs))) continue;
        nav.pcvs[i]=*pcv;
    }
    fp=fopen("testpeph2.out","w");
    
    sat=3;
    
    for (i=0;i<86400*2;i+=30) {
        t=timeadd(time,(double)i);
        satpos(t,t,sat,EPHOPT_BRDC,&nav,rs1,dts1,&var,&svh);
        satpos(t,t,sat,EPHOPT_PREC,&nav,rs2,dts2,&var,&svh);
    }
    fclose(fp);
}
