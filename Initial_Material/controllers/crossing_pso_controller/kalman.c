#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include "kalman.h"

//-----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 	0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter
#define GPS_TRUST_ACC  1.0                    // factors for increase of covariance of gps over time
#define GPS_TRUST_ENC  4

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC false     			// Print position values computed with kalman filter
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

void kalman_compute_acc(gsl_matrix*mu_old, gsl_matrix*lambda_old, const double acc_x, const double acc_y, const double gps_x, const double gps_y, const double delay)
{
  if(!isnan(acc_x) && !isnan(acc_y))
  {//accelerations values in world frame
  gsl_matrix*u= gsl_matrix_alloc(2,1);
  gsl_matrix_set(u,0,0,acc_x);
  gsl_matrix_set(u,1,0,acc_y);
  
  //noise matrix
  gsl_matrix*R= gsl_matrix_calloc(4,4);
  gsl_matrix_set(R,0,0,0.05);
  gsl_matrix_set(R,1,1,0.05);
  gsl_matrix_set(R,2,2,0.01);
  gsl_matrix_set(R,2,2,0.01);
  gsl_matrix_scale(R,_T);
  
  //A matrix
  gsl_matrix*A= gsl_matrix_calloc(4,4);
  gsl_matrix_set(A,0,0,1);
  gsl_matrix_set(A,0,2,_T);
  gsl_matrix_set(A,1,1,1);
  gsl_matrix_set(A,1,3,_T);
  gsl_matrix_set(A,2,2,1);
  gsl_matrix_set(A,3,3,1);
  
  //B matrix
  gsl_matrix*B= gsl_matrix_calloc(4,2);
  gsl_matrix_set(B,2,0,_T);
  gsl_matrix_set(B,3,1,_T);
  
  //kalman filter algorithm
  gsl_matrix*temp1= gsl_matrix_alloc(4,1);
  gsl_matrix*temp2= gsl_matrix_alloc(4,4);
  gsl_matrix*mu_new= gsl_matrix_alloc(4,1);
  gsl_matrix*lambda_new= gsl_matrix_alloc(4,4);

  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, mu_old, 0.0, mu_new); 
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, B, u, 0.0, temp1); 
  gsl_matrix_add(mu_new,temp1);
  
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, lambda_old, A, 0.0, temp2); 
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, temp2, 0.0, lambda_new); 
  gsl_matrix_add(lambda_new,R);
  //printf("%g\n",delay);
//---------------------------------------------------------------------------  
  //choose an option between
  /*
  // 1 use gps once per second only
  if (delay == 0.0f) 
  {
    //printf("on est la");
    //C matrix
    gsl_matrix*C= gsl_matrix_calloc(2,4);
    gsl_matrix_set(C,0,0,1);
    gsl_matrix_set(C,1,1,1);

    //Q matrix
    gsl_matrix*Q= gsl_matrix_calloc(2,2);
    gsl_matrix_set(Q,0,0,1);
    gsl_matrix_set(Q,1,1,1);

    //gps observations (initial position deduced)
    gsl_matrix*z= gsl_matrix_alloc(2,1);
    gsl_matrix_set(z,0,0,gps_x);
    gsl_matrix_set(z,1,0,gps_y);

    gsl_matrix*temp3= gsl_matrix_alloc(4,2);
    gsl_matrix*temp4= gsl_matrix_alloc(2,2);

    
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, lambda_new, C, 0.0, temp3);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, C, temp3, 0.0, temp4);
    gsl_matrix_add(temp4,Q);
    gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasNonUnit, 1, temp4, temp3);

    //K matrix
    gsl_matrix*K= gsl_matrix_alloc(4,2);
    gsl_matrix_memcpy(K,temp3);

    gsl_matrix*temp5= gsl_matrix_alloc(2,1);
    gsl_matrix_memcpy(temp5,z);
    gsl_matrix*temp6= gsl_matrix_alloc(4,4);
    gsl_matrix_set_identity(temp6);
    gsl_matrix*temp7= gsl_matrix_alloc(4,4);

    //mu_new
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, C, mu_new, 1.0, temp5);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, temp5, 1.0, mu_new);
    
    //lambda_new
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, K, C, 1.0, temp6);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, temp6, lambda_new, 0.0, temp7);
    gsl_matrix_memcpy(lambda_new,temp7);
  }
  */
  // 2 always use gps but error increases between gps updates
  
  //C matrix
  gsl_matrix*C= gsl_matrix_calloc(2,4);
  gsl_matrix_set(C,0,0,1);
  gsl_matrix_set(C,1,1,1);

  //Q matrix
  gsl_matrix*Q= gsl_matrix_calloc(2,2);
  gsl_matrix_set(Q,0,0,1+delay*GPS_TRUST_ACC);
  gsl_matrix_set(Q,1,1,1+delay*GPS_TRUST_ACC);

  //gps observations (initial position deduced)
  gsl_matrix*z= gsl_matrix_alloc(2,1);
  gsl_matrix_set(z,0,0,gps_x);
  gsl_matrix_set(z,1,0,gps_y);

  gsl_matrix*temp3= gsl_matrix_alloc(4,2);
  gsl_matrix*temp4= gsl_matrix_alloc(2,2);

    
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, lambda_new, C, 0.0, temp3);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, C, temp3, 0.0, temp4);
  gsl_matrix_add(temp4,Q);
  gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasNonUnit, 1, temp4, temp3);

  //K matrix
  gsl_matrix*K= gsl_matrix_alloc(4,2);
  gsl_matrix_memcpy(K,temp3);

  gsl_matrix*temp5= gsl_matrix_alloc(2,1);
  gsl_matrix_memcpy(temp5,z);
  gsl_matrix*temp6= gsl_matrix_alloc(4,4);
  gsl_matrix_set_identity(temp6);
  gsl_matrix*temp7= gsl_matrix_alloc(4,4);

  //mu_new
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, C, mu_new, 1.0, temp5);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, temp5, 1.0, mu_new);
    
  //lambda_new
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, K, C, 1.0, temp6);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, temp6, lambda_new, 0.0, temp7);
  gsl_matrix_memcpy(lambda_new,temp7);
  
//---------------------------------------------------------------------------   
  
  //update values
  gsl_matrix_memcpy(mu_old, mu_new);
  gsl_matrix_memcpy(lambda_old, lambda_new);
  
  //memset(&kalman_state, X , sizeof(state_t));
  //printf("kalman function %g\n", gsl_matrix_get(mu_old, 0, 0));
  
  // free memory
  gsl_matrix_free(u);
  gsl_matrix_free(R);
  gsl_matrix_free(A);
  gsl_matrix_free(B);
  gsl_matrix_free(temp1);
  gsl_matrix_free(temp2);
  gsl_matrix_free(temp3);
  gsl_matrix_free(temp4);
  gsl_matrix_free(temp5);
  gsl_matrix_free(temp6);
  gsl_matrix_free(temp7);
  gsl_matrix_free(mu_new);
  gsl_matrix_free(lambda_new);
  gsl_matrix_free(C);
  gsl_matrix_free(Q);
  gsl_matrix_free(z);
  gsl_matrix_free(K);
  }
}

void kalman_compute_enc(gsl_matrix*mu_old, gsl_matrix*lambda_old, const double speed_x,const double speed_y, const double gps_x, const double gps_y,const double delay)
{
  //speed values in world frame
  gsl_matrix*u= gsl_matrix_alloc(2,1);
  gsl_matrix_set(u,0,0,speed_x);
  gsl_matrix_set(u,1,0,speed_y);
  
  //noise matrix
  gsl_matrix*R= gsl_matrix_calloc(2,2);
  gsl_matrix_set(R,0,0,0.0001);
  gsl_matrix_set(R,1,1,0.0001);
  gsl_matrix_scale(R,_T);
  
  //A matrix
  gsl_matrix*A= gsl_matrix_calloc(2,2);
  gsl_matrix_set(A,0,0,1);
  gsl_matrix_set(A,1,1,1);
  
  //B matrix
  gsl_matrix*B= gsl_matrix_calloc(2,2);
  gsl_matrix_set(B,0,0,_T);
  gsl_matrix_set(B,1,1,_T);
  
  //kalman filter algorithm
  gsl_matrix*temp1= gsl_matrix_alloc(2,1);
  gsl_matrix*temp2= gsl_matrix_alloc(2,2);
  gsl_matrix*mu_new= gsl_matrix_alloc(2,1);
  gsl_matrix*lambda_new= gsl_matrix_alloc(2,2);

  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, mu_old, 0.0, mu_new); 
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, B, u, 0.0, temp1); 
  gsl_matrix_add(mu_new,temp1);
  
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, lambda_old, A, 0.0, temp2); 
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, temp2, 0.0, lambda_new); 
  gsl_matrix_add(lambda_new,R);
  
   //C matrix
  gsl_matrix*C= gsl_matrix_calloc(2,2);
  gsl_matrix_set(C,0,0,1);
  gsl_matrix_set(C,1,1,1);

  //Q matrix
  gsl_matrix*Q= gsl_matrix_calloc(2,2);
  gsl_matrix_set(Q,0,0,1+delay*GPS_TRUST_ENC);
  gsl_matrix_set(Q,1,1,1+delay*GPS_TRUST_ENC);

  //gps observations (initial position deduced)
  gsl_matrix*z= gsl_matrix_alloc(2,1);
  gsl_matrix_set(z,0,0,gps_x);
  gsl_matrix_set(z,1,0,gps_y);

  gsl_matrix*temp3= gsl_matrix_alloc(2,2);
  gsl_matrix*temp4= gsl_matrix_alloc(2,2);

    
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, lambda_new, C, 0.0, temp3);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, C, temp3, 0.0, temp4);
  gsl_matrix_add(temp4,Q);
  gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasNonUnit, 1, temp4, temp3);

  //K matrix
  gsl_matrix*K= gsl_matrix_alloc(2,2);
  gsl_matrix_memcpy(K,temp3);

  gsl_matrix*temp5= gsl_matrix_alloc(2,1);
  gsl_matrix_memcpy(temp5,z);
  gsl_matrix*temp6= gsl_matrix_alloc(2,2);
  gsl_matrix_set_identity(temp6);
  gsl_matrix*temp7= gsl_matrix_alloc(2,2);

  //mu_new
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, C, mu_new, 1.0, temp5);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, temp5, 1.0, mu_new);
    
  //lambda_new
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, -1.0, K, C, 1.0, temp6);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, temp6, lambda_new, 0.0, temp7);
  gsl_matrix_memcpy(lambda_new,temp7);
  
  gsl_matrix_memcpy(mu_old, mu_new);
  gsl_matrix_memcpy(lambda_old, lambda_new);
  
  // free memory
  gsl_matrix_free(u);
  gsl_matrix_free(R);
  gsl_matrix_free(A);
  gsl_matrix_free(B);
  gsl_matrix_free(temp1);
  gsl_matrix_free(temp2);
  gsl_matrix_free(temp3);
  gsl_matrix_free(temp4);
  gsl_matrix_free(temp5);
  gsl_matrix_free(temp6);
  gsl_matrix_free(temp7);
  gsl_matrix_free(mu_new);
  gsl_matrix_free(lambda_new);
  gsl_matrix_free(C);
  gsl_matrix_free(Q);
  gsl_matrix_free(z);
  gsl_matrix_free(K);
}


void kalman_reset(int time_step)
{
	_T = time_step / 1000.0;
}