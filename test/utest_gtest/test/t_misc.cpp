/*------------------------------------------------------------------------------
* rtklib unit test driver : misc functions
*-----------------------------------------------------------------------------*/
#include "gtest/gtest.h"
#include "rtklib.h"

/* reppath() */
TEST(TEST_misc, test_repath)
{
    gtime_t t0={0},t1,t2;
    double ep1[]={1990, 1, 1, 0, 0, 0.00000};
    double ep2[]={2010,12,31,23,59,59.99999};
    char path0[]="01234567890123456789";
    char path1[]="abcde_%Y/%m/%d_%h:%M:%S_%Y%m%d%h%M%S";
    char path2[]="abcde_%y%n_%W%D%H_%ha%hb%hc";
    char path3[]="rover %r %r base %b %b";
    char path4[]="%a %b %c";
    char rpath[1024];
    char rov[]="RRRRRRRR",base[]="BBBBBBBB";
    int stat;
    
    t1=epoch2time(ep1);
    t2=epoch2time(ep2);
    
    stat=reppath(path0,rpath,t1,"","");
        EXPECT_TRUE(stat==0);
    stat=strcmp(rpath,path0);
        EXPECT_TRUE(stat==0);
    stat=reppath(path0,rpath,t0,rov,base);
        EXPECT_TRUE(stat==0);
    stat=strcmp(rpath,path0);
        EXPECT_TRUE(stat==0);
    stat=reppath(path0,rpath,t1,rov,base);
        EXPECT_TRUE(stat==0);
    stat=strcmp(rpath,path0);
        EXPECT_TRUE(stat==0);
    stat=reppath(path1,rpath,t1,"","");
        EXPECT_TRUE(stat==1);
    stat=strcmp(rpath,"abcde_1990/01/01_00:00:00_19900101000000");
        EXPECT_TRUE(stat==0);
    stat=reppath(path2,rpath,t2,rov,base);
        EXPECT_TRUE(stat==1);
    stat=strcmp(rpath,"abcde_10365_16165x_211812");
        EXPECT_TRUE(stat==0);
    stat=reppath(path3,rpath,t0,rov,base);
        EXPECT_TRUE(stat==1);
    stat=strcmp(rpath,"rover RRRRRRRR RRRRRRRR base BBBBBBBB BBBBBBBB");
        EXPECT_TRUE(stat==0);
    stat=reppath(path4,rpath,t1,rov,"");
        EXPECT_TRUE(stat==0);
    stat=strcmp(rpath,"%a %b %c");
        EXPECT_TRUE(stat==0);
}
/* reppaths() */
TEST(TEST_misc, test_repaths)
{
    gtime_t t0={0},t1,t2,t3,t4;
    double ep1[]={2010, 7,31,21,36,50.00000};
    double ep2[]={2010, 8, 1, 4, 0, 0.00000};
    double ep3[]={2010, 8,31, 0, 0, 0.00000};
    double ep4[]={2012, 1,31, 0, 0, 0.00000};
    char path0[]="01234567890123456789";
    char path1[]="abcde_%Y/%m/%d_%h:%M:%S_%Y%m%d%h%M%S";
    char path2[]="%r_%b_%r_%b_%y%n_%W%D%H_%ha%hb%hc";
    char path4[]="YEAR=%Y GPSWEEK=%W";
    char *paths[100];
    int i,n,stat;
    
    t1=epoch2time(ep1);
    t2=epoch2time(ep2);
    t3=epoch2time(ep3);
    t4=epoch2time(ep4);
    
    for (i=0;i<100;i++) paths[i]=(char *)malloc(1024);
    
    n=reppaths(path1,paths,10,t0,t1,"ROV","BASE");
        EXPECT_TRUE(n==0);
    n=reppaths(path1,paths,10,t1,t0,"ROV","BASE");
        EXPECT_TRUE(n==0);
    n=reppaths(path1,paths, 0,t1,t2,"ROV","BASE");
        EXPECT_TRUE(n==0);
    n=reppaths(path1,paths,10,t2,t1,"ROV","BASE");
        EXPECT_TRUE(n==0);
    n=reppaths(path0,paths,10,t1,t2,"ROV","BASE");
        EXPECT_TRUE(n==1);
    stat=strcmp(paths[0],path0);
        EXPECT_TRUE(stat==0);
    n=reppaths(path1,paths,100,t1,t2,"ROV","BASE");
        
        EXPECT_TRUE(n==27);
    stat=strcmp(paths[ 0],"abcde_2010/07/31_21:30:00_20100731213000");
        EXPECT_TRUE(stat==0);
    stat=strcmp(paths[26],"abcde_2010/08/01_04:00:00_20100801040000");
        EXPECT_TRUE(stat==0);
    n=reppaths(path2,paths,100,t1,t3,"ROV","BASE");
        
        EXPECT_TRUE(n==100);
    stat=strcmp(paths[ 0],"ROV_BASE_ROV_BASE_10212_15946v_211812");
        EXPECT_TRUE(stat==0);
    stat=strcmp(paths[99],"ROV_BASE_ROV_BASE_10217_15954a_000000");
        EXPECT_TRUE(stat==0);
    n=reppaths(path4,paths,100,t1,t4,"ROV","BASE");
        
        EXPECT_TRUE(n==81);
    stat=strcmp(paths[ 0],"YEAR=2010 GPSWEEK=1594");
        EXPECT_TRUE(stat==0);
    stat=strcmp(paths[80],"YEAR=2012 GPSWEEK=1673");
        EXPECT_TRUE(stat==0);
    
    for (i=0;i<100;i++) free(paths[i]);
    
}
/* getbitu(),getbits(),setbitu(),setbits() */
TEST(TEST_misc, test_getbitu)
{
    unsigned char buff[1024]={0};
    unsigned int vu;
    int vs;
    
    setbitu(buff,  0, 8,     1); vu=getbitu(buff,  0, 8); EXPECT_TRUE(vu==     1);
    setbitu(buff,  4, 8,   255); vu=getbitu(buff,  4, 8); EXPECT_TRUE(vu==   255);
    setbitu(buff, 13, 8,     1); vu=getbitu(buff, 13, 8); EXPECT_TRUE(vu==     1);
    setbitu(buff, 29, 8,   255); vu=getbitu(buff, 29, 8); EXPECT_TRUE(vu==   255);
    setbitu(buff, 99,10,  1023); vu=getbitu(buff, 99,10); EXPECT_TRUE(vu==  1023);
    setbitu(buff,666,31,123456); vu=getbitu(buff,666,31); EXPECT_TRUE(vu==123456);
    setbitu(buff,777,32,789012); vu=getbitu(buff,777,32); EXPECT_TRUE(vu==789012);
    
    setbits(buff,100, 8,     1); vs=getbitu(buff,100, 8); EXPECT_TRUE(vs==     1);
    setbits(buff,104, 8,   127); vs=getbitu(buff,104, 8); EXPECT_TRUE(vs==   127);
    setbits(buff,113, 8,     1); vs=getbitu(buff,113, 8); EXPECT_TRUE(vs==     1);
    setbits(buff,129, 8,   127); vs=getbitu(buff,129, 8); EXPECT_TRUE(vs==   127);
    setbits(buff,199,10,   511); vs=getbitu(buff,199,10); EXPECT_TRUE(vs==   511);
    setbits(buff,766,31,123456); vs=getbitu(buff,766,31); EXPECT_TRUE(vs==123456);
    setbits(buff,877,32,789012); vs=getbitu(buff,877,32); EXPECT_TRUE(vs==789012);
    
    setbits(buff,200, 8,    -1); vs=getbits(buff,200, 8); EXPECT_TRUE(vs==    -1);
    setbits(buff,204, 8,  -127); vs=getbits(buff,204, 8); EXPECT_TRUE(vs==  -127);
    setbits(buff,213, 8,    -3); vu=getbits(buff,213, 8); EXPECT_TRUE(vu==    -3);
    setbits(buff,229, 8,  -126); vu=getbits(buff,229, 8); EXPECT_TRUE(vu==  -126);
    setbits(buff,299,24,-99999); vu=getbits(buff,299,24); EXPECT_TRUE(vu==-99999);
    setbits(buff,866,31,-12345); vs=getbits(buff,866,31); EXPECT_TRUE(vs==-12345);
    setbits(buff,977,32,-67890); vs=getbits(buff,977,32); EXPECT_TRUE(vs==-67890);
}
