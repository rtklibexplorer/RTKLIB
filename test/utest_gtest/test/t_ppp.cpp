/*------------------------------------------------------------------------------
* rtklib unit test driver : ppp functions
*-----------------------------------------------------------------------------*/
#include "gtest/gtest.h"
#include "rtklib.h"

/* eci2ecef() */
TEST(TEST_PPP, test_eci2ecef)
{
    double ep1[]={1999,3,4,0,0,0};
    double gmst,U[9];
    double U1[]={
        -0.947378027425279,        0.320116956820115,   -8.43090456427539e-005,
         -0.32011695222455,       -0.947378030590727,   -6.36592598714651e-005,
      -0.00010025094616549,   -3.33206293083182e-005,        0.999999994419742
    };
    double erpv[5]={0.06740*D2R/3600,0.24713*D2R/3600,0.649232};
    int i,j;
    
    eci2ecef(epoch2time(ep1),erpv,U,&gmst);
    
    for (i=0;i<3;i++) for (j=0;j<3;j++) {
        EXPECT_TRUE(fabs(U[i+j*3]-U1[j+i*3])<1E-11);
    }
}
/* sunmoonpos() */
TEST(TEST_PPP, test_sunmmonpos)
{
    double ep1[]={2010,12,31,8,9,10}; /* utc */
    double rs[]={70842740307.0837,115293403265.153,-57704700666.9715}; /* de405 */
    double rm[]={350588081.147922,29854134.6432052,-136870369.169738};
    double rsun[3],rmoon[3],erpv[5]={0};
    int i;
    
    sunmoonpos(epoch2time(ep1),erpv,rsun,rmoon,NULL);
    
    for (i=0;i<3;i++) {
        EXPECT_TRUE(fabs((rsun [i]-rs[i])/rsun [i])<0.03);
        EXPECT_TRUE(fabs((rmoon[i]-rm[i])/rmoon[i])<0.03);
    }
}
/* tidedisp() */
TEST(TEST_PPP, test_tidedisp)
{
    double ep1[]={2010,6,7,1,2,3};
    double rr[]={-3957198.431,3310198.621,3737713.474}; /* TSKB */
    double dp[]={-0.05294,0.075607,0.03644};
    double dr[3]={0};
    int i;
    
    tidedisp(epoch2time(ep1),rr,1,NULL,NULL,dr);
    
    
    for (i=0;i<3;i++) {
        EXPECT_TRUE(fabs(dr[i]-dp[i])<0.001);
    }
}
