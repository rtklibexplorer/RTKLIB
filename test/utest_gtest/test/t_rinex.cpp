/*------------------------------------------------------------------------------
* rtklib unit test driver : rinex function
*-----------------------------------------------------------------------------*/
#include "gtest/gtest.h"
#include "rtklib.h"
#include "datadir.h"

static void dumpobs(obs_t *obs)
{
    gtime_t time={0};
    int i;
    char str[64];
    for (i=0;i<obs->n;i++) {
        time2str(obs->data[i].time,str,3);
        
        EXPECT_TRUE(1<=obs->data[i].sat&&obs->data[i].sat<=32);
        EXPECT_TRUE(timediff(obs->data[i].time,time)>=-DTTOL);
        
        time=obs->data[i].time;
    }
}
static void dumpnav(nav_t *nav)
{
    int i;
    char str[64],s1[64],s2[64];
    for (i=0;i<nav->n;i++) {
        time2str(nav->eph[i].toe,str,3);
        time2str(nav->eph[i].toc,s1,0);
        time2str(nav->eph[i].ttr,s2,0);
        
        EXPECT_TRUE(nav->eph[i].iode==(nav->eph[i].iodc&0xFF));
    }
}

/* readrnx(), sortobs(), uniqnav()  */
TEST(TEST_RINEX,test_read_sort_uniqnav)
{
    char file1[]="abc.00o";
    char file2[]="bcd.00n";
#ifdef WIN32
    string file3 = TestData::ins().full_path_filename("..\\data\\rinex\\07590920.05o");
    string file4 = TestData::ins().full_path_filename("..\\data\\rinex\\07590920.05n");
    string file5 = TestData::ins().full_path_filename("..\\data\\rinex\\30400920.05o");
    string file6 = TestData::ins().full_path_filename("..\\data\\rinex\\30400920.05n");
#else
    string file3 = TestData::ins().full_path_filename("../data/rinex/07590920.05o");
    string file4 = TestData::ins().full_path_filename("../data/rinex/07590920.05n");
    string file5 = TestData::ins().full_path_filename("../data/rinex/30400920.05o");
    string file6 = TestData::ins().full_path_filename("../data/rinex/30400920.05n");
#endif
    obs_t obs={0};
    nav_t nav={0};
    sta_t sta={""};
    int n,stat;
    
    stat=readrnx(file1,1,"",&obs,&nav,&sta);
        EXPECT_TRUE(stat==0&&obs.n==0&&nav.n==0&&nav.ng==0&&nav.ns==0);
    stat=readrnx(file2,1,"",&obs,&nav,&sta);
        EXPECT_TRUE(stat==0&&obs.n==0&&nav.n==0&&nav.ng==0&&nav.ns==0);
    stat=readrnx(file3.c_str(),1,"",&obs,&nav,&sta);
        EXPECT_TRUE(stat==1);
    stat=readrnx(file4.c_str(),1,"",&obs,&nav,&sta);
        EXPECT_TRUE(stat==1);
    stat=readrnx(file5.c_str(),2,"",&obs,&nav,&sta);
        EXPECT_TRUE(stat==1);
    stat=readrnx(file6.c_str(),2,"",&obs,&nav,&sta);
        EXPECT_TRUE(stat==1);
    n=sortobs(&obs);
        EXPECT_TRUE(n>100);
    uniqnav(&nav);
        EXPECT_TRUE(nav.n==167);
    dumpobs(&obs); dumpnav(&nav);
        EXPECT_TRUE(obs.data&&obs.n>0&&nav.eph&&nav.n>0);
    free(obs.data);
    free(nav.eph);
    free(nav.geph);
    free(nav.seph);
}
/* readrnxt() */
TEST(TEST_RINEX, test_readrnxt)
{
    gtime_t t0={0},ts,te;
    double ep1[]={2005,4,2,1,0,0},ep2[]={2005,4,2,2,0,0};
#ifdef WIN32
    string file1 = TestData::ins().full_path_filename("..\\data\\rinex\\07590920.05o");
    string file2 = TestData::ins().full_path_filename("..\\data\\rinex\\07590920.05n");
#else
    string file1 = TestData::ins().full_path_filename("../data/rinex/07590920.05o");
    string file2 = TestData::ins().full_path_filename("../data/rinex/07590920.05n");
#endif
    int n;
    obs_t obs={0};
    nav_t nav={0};
    sta_t sta={""};
    
    ts=epoch2time(ep1);
    te=epoch2time(ep2);
    n=readrnxt(file1.c_str(),1,ts,te,0.0,"",&obs,&nav,&sta);
    n=readrnxt(file2.c_str(),1,ts,te,0.0,"",&obs,&nav,&sta);
    dumpobs(&obs);
    free(obs.data); obs.data=NULL; obs.n=obs.nmax=0;
    n=readrnxt(file1.c_str(),1,t0,t0,240.0,"",&obs,&nav,&sta);
    dumpobs(&obs);
    free(obs.data);
}
