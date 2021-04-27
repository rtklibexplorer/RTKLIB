/*------------------------------------------------------------------------------
* rtklib unit test driver : glonass ephemeris function
*-----------------------------------------------------------------------------*/
#include "gtest/gtest.h"
#include "rtklib.h"
#include "datadir.h"

/* readrnx() */
TEST(TEST_GLO_EPH, test_readrnx)
{
#ifdef WIN32
   string file1 = TestData::ins().full_path_filename("..\\data\\rinex\\brdd0910.09g");
   string file2 = TestData::ins().full_path_filename("..\\data\\rinex\\brdc0910.09g");
#else
   string file1 = TestData::ins().full_path_filename("../data/rinex/brdd0910.09g");
   string file2 = TestData::ins().full_path_filename("../data/rinex/brdc0910.09g");
#endif
    nav_t nav={0};
    
    readrnx(file1.c_str(),1,"",NULL,&nav,NULL);
        EXPECT_TRUE(nav.ng==0);
    readrnx(file2.c_str(),1,"",NULL,&nav,NULL);
        EXPECT_TRUE(nav.ng>0)<<"couldn't get glonass eph";
}
/* readsp3() */
TEST(TEST_GLO_EPH, test_readsp3)
{
#ifdef WIN32
   string file1 = TestData::ins().full_path_filename("..\\data\\sp3\\igl15253.sp4");
   string file2 = TestData::ins().full_path_filename("..\\data\\sp3\\igl15253.sp3");
#else
   string file1 = TestData::ins().full_path_filename("../data/sp3/igl15253.sp4");
   string file2 = TestData::ins().full_path_filename("../data/sp3/igl15253.sp3");
#endif
    nav_t nav={0};
    double tow,*pos;
    int i,week,sat;
    
    sat=satno(SYS_GLO,13);
    
    readsp3(file1.c_str(),&nav,0);
        EXPECT_TRUE(nav.ne<=0);
    readsp3(file2.c_str(),&nav,0);
        EXPECT_TRUE(nav.ne>0);
    
    for (i=0;i<nav.ne;i++) {
        tow=time2gpst(nav.peph[i].time,&week);
        pos=nav.peph[i].pos[sat-1];
     
        EXPECT_TRUE(norm(pos,4)>0.0);
    }
}
/* broadcast ephemeris */
TEST(TEST_GLO_EPH, test_broadcast_ephemeris)
{
    gtime_t time;
#ifdef WIN32
    string file1 = TestData::ins().full_path_filename("..\\data\\rinex\\brdc0910.09g");
#else
    string file1 = TestData::ins().full_path_filename("../data/rinex/brdc0910.09g");
#endif
    nav_t nav={0};
    double ep[]={2009,4,1,0,0,0};
    double tspan=86400.0,tint=30.0,tow;
    double rs[6],dts[2];
    double var;
    int i,sat,week,svh;
    
    sat=satno(SYS_GLO,7);
    
    readrnx(file1.c_str(),1,"",NULL,&nav,NULL);
    
    for (i=0;i<tspan/tint;i++) {
        time=timeadd(epoch2time(ep),tint*i);
        satpos(time,time,sat,EPHOPT_BRDC,&nav,rs,dts,&var,&svh);
        tow=time2gpst(time,&week);
  
        EXPECT_TRUE(norm(rs,3)>0.0);
        EXPECT_TRUE(norm(rs+3,3)>0.0);
        EXPECT_TRUE(dts[0]!=0.0);
        EXPECT_TRUE(dts[1]!=0.0);
    }
}
/* precise ephemeris */
TEST(TEST_GLO_EPH, test_precise_ephemeris)
{
    gtime_t time;
#ifdef WIN32
    string file = TestData::ins().full_path_filename("..\\data\\sp3\\igl15253.sp3");
#else
    string file = TestData::ins().full_path_filename("../data/sp3/igl15253.sp3");
#endif
    nav_t nav={0};
    double ep[]={2009,4,1,0,0,0};
    double tspan=86400.0,tint=30.0,tow;
    double rs[6],dts[2];
    double var;
    int i,sat,week,svh;
    
    sat=satno(SYS_GLO,7);
    
    readsp3(file.c_str(),&nav,0);
    
    for (i=0;i<tspan/tint;i++) {
        time=timeadd(epoch2time(ep),tint*i);
        satpos(time,time,sat,EPHOPT_PREC,&nav,rs,dts,&var,&svh);
        tow=time2gpst(time,&week);
   
        EXPECT_TRUE(norm(rs,3)>0.0);
        EXPECT_TRUE(norm(rs+3,3)>0.0);
        EXPECT_TRUE(dts[0]!=0.0);
    }
}
/* readsap() */
TEST(TEST_GLO_EPH, test_readsap)
{
    char id[32];
#ifdef WIN32
    string file1 = TestData::ins().full_path_filename("..\\..\\data\\igs05.atx");
#else
    string file1 = TestData::ins().full_path_filename("../../data/igs05.atx");
#endif
    double ep[]={2009,4,1,0,0,0};
    gtime_t time=epoch2time(ep);
    nav_t nav={0};
    int i,stat;
    
    stat=readsap(file1.c_str(),time,&nav);
    EXPECT_EQ(stat, 1);
}
/* satpos() */
TEST(TEST_GLO_EPH, test_satpos)
{
    FILE *fp;
#ifdef WIN32
    string file1 = TestData::ins().full_path_filename("..\\data\\rinex\\brdc0910.09g");
    string file2 = TestData::ins().full_path_filename("..\\data\\sp3\\igl15253.sp3");
    string file3 = TestData::ins().full_path_filename("..\\..\\data\\igs05.atx");
#else
    string file1 = TestData::ins().full_path_filename("../data/rinex/brdc0910.09g");
    string file2 = TestData::ins().full_path_filename("../data/sp3/igl15253.sp3");
    string file3 = TestData::ins().full_path_filename("../../data/igs05.atx");
#endif
/*
    char *file4="../data/esa15253.sp3";
    char *file5="../data/esa15253.clk";
*/
    char *outfile="testgloeph.out";
    double ep[]={2009,4,1,0,0,0};
    double tspan=86400.0,tint=30.0,tow;
    double rs1[6],dts1[2],rs2[6],dts2[2],dr[3],ddts;
    double var1,var2;
    gtime_t time=epoch2time(ep);
    nav_t nav={0};
    int i,j,sat,week,svh1,svh2;
    
    readrnx(file1.c_str(),1,"",NULL,&nav,NULL);
    readsp3(file2.c_str(),&nav,0);
/*
    readsp3(file4,&nav,0);
    readrnxc(file5,&nav);
*/
    readsap(file3.c_str(),time,&nav);
    
/*
    sat=satno(SYS_GLO,21);
*/
    sat=satno(SYS_GLO,22);
    
    fp=fopen(outfile,"w");
    
    for (i=0;i<tspan/tint;i++) {
        time=timeadd(epoch2time(ep),tint*i);
        tow=time2gpst(time,&week);
        satpos(time,time,sat,EPHOPT_BRDC,&nav,rs1,dts1,&var1,&svh1);
        satpos(time,time,sat,EPHOPT_PREC,&nav,rs2,dts2,&var2,&svh2);
        
        if (norm(rs1,3)<=0.0||norm(rs2,3)<=0.0) continue;
        
        for (j=0;j<3;j++) dr[j]=rs1[j]-rs2[j];
        ddts=dts1[0]-dts2[0];
        
        EXPECT_TRUE(norm(dr,3)<10.0);
    }
    fclose(fp);
}
