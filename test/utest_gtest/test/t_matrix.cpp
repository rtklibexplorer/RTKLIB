/*------------------------------------------------------------------------------
* rtklib unit test driver : matrix and vector functions
*-----------------------------------------------------------------------------*/
#include "gtest/gtest.h"
#include "rtklib.h"

/* mat(),imat(),zeros(),eye()  */
TEST(Test_Matrix, mat_imat_zeros_eye)
{
    int i,j,n=100,m=200,*b;
    double *a;
    a=mat(0,0); EXPECT_EQ(a,nullptr);
    a=mat(0,1); EXPECT_TRUE(a==nullptr);
    a=mat(1,0); EXPECT_TRUE(a==nullptr);
    a=mat(1,1); EXPECT_TRUE(a!=nullptr);
    free(a);
/*  a=mat(1000000,1000000); EXPECT_TRUE(a==nullptr); */
    a=zeros(0,m); EXPECT_TRUE(a==nullptr);
    a=zeros(n,0); EXPECT_TRUE(a==nullptr);
    a=zeros(n,m); EXPECT_TRUE(a!=nullptr);
    for (i=0;i<n;i++) for (j=0;j<m;j++) EXPECT_TRUE(a[i+j*n]==0.0);
    free(a);
/*  a=zeros(10000000,1000000); EXPECT_TRUE(a==nullptr); */
    a=eye(0); EXPECT_TRUE(a==nullptr);
    a=eye(n); EXPECT_TRUE(a!=nullptr);
    for (i=0;i<n;i++) for (j=0;j<n;j++) EXPECT_TRUE(a[i+j*n]==(i==j?1.0:0.0));
    free(a);
/*  a=eye(1000000); EXPECT_TRUE(a==nullptr); */
    b=imat(0,m); EXPECT_TRUE(b==nullptr);
    b=imat(n,0); EXPECT_TRUE(b==nullptr);
    b=imat(n,m); EXPECT_TRUE(b!=nullptr);
    free(b);
/*
    a=imat(1000000,1000000); EXPECT_TRUE(a==NULL);
*/
}
/* dot() */
TEST(Test_Matrix, test_dot)
{
    int i;
    double a[]={1.0,2.0,3.0,4.0,5.0,6.0};
    double b[]={7.0,8.0,9.0,1.4,1.6,7.8};
    double c,d;
    for (i=0,d=0.0;i<6;i++) d+=a[i]*b[i];
    c=dot(a,b,0); EXPECT_TRUE(c==0.0);
    c=dot(a,b,6); EXPECT_TRUE(fabs(c-d)<1E-14);
    for (i=0,d=0.0;i<6;i++) d+=a[i]*a[i]; d=sqrt(d);
    c=norm(a,6);  EXPECT_TRUE(fabs(c-d)<1E-14);
    for (i=0,d=0.0;i<6;i++) d+=b[i]*b[i]; d=sqrt(d);
    c=norm(b,6);  EXPECT_TRUE(fabs(c-d)<1E-14);
}
static double A[]={
         0.935469699107605,        0.893649530913534,
         0.916904439913408,       0.0578913047842686,
         0.410270206990945,           0.352868132217
};
static double B[]={
         0.813166497303758,         0.13889088195695,
       0.00986130066092356,        0.202765218560273
};
static double AB[]={
         0.769505165266963,        0.311129254005026,
          0.74616685532878,        0.139088009397138,
         0.337097725912365,        0.128532174841568
};
static double ABT[]={
         0.884812390066127,        0.190425990414052,
         0.753636546145776,       0.0207802134266434,
         0.382628153265035,       0.0755951818152924
};
static double invB[]={
          1.24006142464565,       -0.849421938204008,
       -0.0603092514252338,         4.97312316323555
};
/* matmul() */
TEST(Test_Matrix, test_matmul)
{
    int i,j;
    double C[6];
    matmul("TT",3,2,2,1.0,A,B,0.0,C);
    for (i=0;i<3;i++) for (j=0;j<2;j++) EXPECT_TRUE(fabs(C[i+j*3]-AB[j+i*2])<1E-9);
    
    matmul("NN",2,3,2,1.0,B,A,0.0,C);
    for (i=0;i<2;i++) for (j=0;j<3;j++) EXPECT_TRUE(fabs(C[i+j*2]-AB[i+j*2])<1E-9);
    
    matmul("TN",3,2,2,1.0,A,B,0.0,C);
    for (i=0;i<3;i++) for (j=0;j<2;j++) EXPECT_TRUE(fabs(C[i+j*3]-ABT[j+i*2])<1E-9);
    
    matmul("TN",2,3,2,1.0,B,A,0.0,C);
    for (i=0;i<2;i++) for (j=0;j<3;j++) EXPECT_TRUE(fabs(C[i+j*2]-ABT[i+j*2])<1E-9);
}
/* matinv() */
TEST(Test_Matrix, test_matinv)
{
    int i,j;
    double C[4];
    memcpy(C,B,sizeof(double)*4);
    matinv(B,2);
    for (i=0;i<2;i++) for (j=0;j<2;j++) EXPECT_TRUE(fabs(B[i+j*2]-invB[i+j*2])<1E-9);
    matinv(B,2);
    for (i=0;i<2;i++) for (j=0;j<2;j++) EXPECT_TRUE(fabs(B[i+j*2]-C[i+j*2])<1E-9);
}
static double H[]={
    0.123, 0.345, 0.567,
    0.890,-0.135, 0.791,
    1.020, 2.489, 0.111,
    0.321,-1.002, 5.678
};
static double y[]={
    0.3456,
    1.5678,
    0.1047,
    0.1047
};
static double xs[]={
    1.77586016656388,
   -0.64484683008484,
   -0.18684144028875
};
static double Qs[]={
    1.41343666098904, -0.56122396983116, -0.20536436422033,
   -0.56122396983117,  0.37710272892140,  0.10628198948746,
   -0.20536436422033,  0.10628198948746,  0.06392702788446
};
/* lsq() */
TEST(Test_Matrix, test_lsq)
{
    int i;
    double x[3],Q[9];
    lsq(H,y,3,4,x,Q);
    for (i=0;i<3;i++) EXPECT_TRUE(fabs(x[i]-xs[i])<1E-9);
    for (i=0;i<9;i++) EXPECT_TRUE(fabs(Q[i]-Qs[i])<1E-9);
}
/* matcpy() */
TEST(Test_Matrix, test_matcpy)
{
    double *a=mat(100,200),*b=zeros(100,200),c=0.0;
    int i,j;
    for (i=0;i<100;i++) {
        for (j=0;j<200;j++) {
            a[i+j*100]=(c+=1.0);
        }
    }
    matcpy(b,a,100,200);
    for (i=0,c=0.0;i<100;i++) {
        for (j=0;j<200;j++) {
            EXPECT_TRUE(a[i+j*100]==b[i+j*100]);
        }
    }
    free(a); free(b);
}
