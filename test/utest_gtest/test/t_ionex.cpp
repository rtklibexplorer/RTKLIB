/*------------------------------------------------------------------------------
* rtklib unit test driver : ionex function
*-----------------------------------------------------------------------------*/
#include "gtest/gtest.h"
#include "rtklib.h"
#include "datadir.h"

/* readtec() */
TEST(TEST_IONEX, test_readtec)
{
#ifdef WIN32
   string file1 = TestData::ins().full_path_filename("..\\data\\sp3\\igrg3380.10j");
   string file2 = TestData::ins().full_path_filename("..\\data\\sp3\\igrg3380.10i");
   string file3 = TestData::ins().full_path_filename("..\\data\\sp3\\igrg33*0.10i");
#else
    string file1 = TestData::ins().full_path_filename("../data/sp3/igrg3380.10j");
    string file2 = TestData::ins().full_path_filename("../data/sp3/igrg3380.10i");
    string file3 = TestData::ins().full_path_filename("../data/sp3/igrg33*0.10i");
#endif
    nav_t nav={0};
    
    readtec(file1.c_str(),&nav,0);
        EXPECT_TRUE(nav.nt==0);
    
    readtec(file2.c_str(),&nav,0);
        EXPECT_TRUE(nav.nt==13);
    
    readtec(file3.c_str(),&nav,0);
        EXPECT_TRUE(nav.nt==25);
}
/* iontec() 1 */
TEST(TEST_IONEX, test_iontec)
{
#ifdef WIN32
   string file3 = TestData::ins().full_path_filename("..\\data\\sp3\\igrg33*0.10i");
#else
   string file3 = TestData::ins().full_path_filename("../data/sp3/igrg33*0.10i");
#endif
    
    nav_t nav={0};
    gtime_t time1,time2,time3,time4;
    double ep1  []={2010,12, 4, 0, 0, 0};
    double ep2  []={2010,12, 5,23,59,59};
    double ep3  []={2010,12, 3,23,59,59}; /* error */
    double ep4  []={2010,12, 6, 0, 0, 0}; /* error */
    double pos1 []={ 45.1*D2R, 135.7*D2R,0.0};
    double pos2 []={-45.1*D2R,-170.7*D2R,0.0};
    double pos3 []={-45.1*D2R, 189.3*D2R,0.0};
    double pos4 []={ 87.6*D2R,   0.0*D2R,0.0}; /* out of grid */
    double pos5 []={-87.6*D2R,   0.0*D2R,0.0}; /* out of grid */
    double azel1[]={  0.0,90.0*D2R};
    double azel2[]={120.0,30.0*D2R};
    double azel3[]={  0.0,-0.1*D2R}; /* error */
    double delay1,var1,delay2=0,var2;
    int stat;
    
    time1=epoch2time(ep1);
    time2=epoch2time(ep2);
    time3=epoch2time(ep3);
    time4=epoch2time(ep4);
    
    readtec(file3.c_str(),&nav,0);
    stat=iontec(time1,&nav,pos1,azel1,1,&delay1,&var1);
        EXPECT_TRUE(stat==1);
    stat=iontec(time2,&nav,pos1,azel1,1,&delay1,&var1);
        EXPECT_TRUE(stat==1);
    stat=iontec(time3,&nav,pos1,azel1,1,&delay1,&var1);
        EXPECT_TRUE(stat==0);
    stat=iontec(time4,&nav,pos1,azel1,1,&delay1,&var1);
        EXPECT_TRUE(stat==0);
    stat=iontec(time1,&nav,pos2,azel1,1,&delay1,&var1);
        EXPECT_TRUE(stat==1);
    stat=iontec(time1,&nav,pos3,azel1,1,&delay2,&var2);
        EXPECT_TRUE(stat==1);
        EXPECT_TRUE(fabs(delay1-delay2)<1E-4);
        EXPECT_TRUE(fabs(var1-var2)<1E-8);
    stat=iontec(time1,&nav,pos4,azel1,1,&delay1,&var1);
        EXPECT_TRUE(stat==1);
    stat=iontec(time1,&nav,pos5,azel1,1,&delay1,&var1);
        EXPECT_TRUE(stat==1);
    stat=iontec(time1,&nav,pos1,azel2,1,&delay1,&var1);
        EXPECT_TRUE(stat==1);
    stat=iontec(time1,&nav,pos1,azel3,1,&delay1,&var1);
        EXPECT_TRUE(stat==1&&delay1==0.0);
}
/* iontec() 2 */
TEST(TEST_IONEX, test_iontec2)
{
    FILE *fp;
#ifdef WIN32
    string file3 = TestData::ins().full_path_filename("..\\data\\sp3\\igrg33*0.10i");
#else
    string file3 = TestData::ins().full_path_filename("../data/sp3/igrg33*0.10i");
#endif
    nav_t nav={0};
    gtime_t time1;
    double ep1[]={2010,12, 5, 0, 0, 0};
    double delay,var,pos[3]={0},azel[]={0.0,PI/2};
    int i,j;
    
    time1=epoch2time(ep1);
    readtec(file3.c_str(),&nav,0);
    
    fp=fopen("testionex3.m","w");
        EXPECT_TRUE(fp);
    
    for (i=90;i>=-90;i-=2) {
        for (j=0;j<=360;j+=2) {
            pos[0]=i*D2R;
            pos[1]=j*D2R;
            if (iontec(time1,&nav,pos,azel,1,&delay,&var)) {
            }
            else {
            }
        }
    }
    fclose(fp);
    
    fp=fopen("testionex3.m","a");
        EXPECT_TRUE(fp);
    
    for (i=90;i>=-90;i-=2) {
        for (j=0;j<=360;j+=2) {
            pos[0]=i*D2R;
            pos[1]=j*D2R;
            if (iontec(time1,&nav,pos,azel,1,&delay,&var)) {
            }
            else {
            }
        }
    }
    fclose(fp);
    
}
/* iontec() 3 */
TEST(TEST_IONEX, test_iontec3)
{
    FILE *fp;
#ifdef WIN32
    string file3 = TestData::ins().full_path_filename("..\\data\\sp3\\igrg33*0.10i");
#else
    string file3 = TestData::ins().full_path_filename("../data/sp3/igrg33*0.10i");
#endif
    nav_t nav={0};
    gtime_t time1;
    double ep1[]={2010,12, 3,12, 0, 0};
    double delay,var,pos[3]={25*D2R,135*D2R,0};
    double azel[]={75*D2R,90*D2R};
    int i;
    
    time1=epoch2time(ep1);
    readtec(file3.c_str(),&nav,0);
    
    fp=fopen("testionex4.m","w");
        EXPECT_TRUE(fp);
    
    for (i=0;i<=86400*3;i+=30) {
        if (iontec(timeadd(time1,i),&nav,pos,azel,1,&delay,&var)) {
        }
        else {
        }
    }
    fclose(fp);
}
