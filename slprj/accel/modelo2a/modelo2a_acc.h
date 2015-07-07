#include "__cf_modelo2a.h"
#ifndef RTW_HEADER_modelo2a_acc_h_
#define RTW_HEADER_modelo2a_acc_h_
#ifndef modelo2a_acc_COMMON_INCLUDES_
#define modelo2a_acc_COMMON_INCLUDES_
#include <stdlib.h>
#include <stddef.h>
#include <float.h>
#define S_FUNCTION_NAME simulink_only_sfcn 
#define S_FUNCTION_LEVEL 2
#define RTW_GENERATED_S_FUNCTION
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#endif
#include "modelo2a_acc_types.h"
typedef struct { real_T B_0_0_0 ; real_T B_0_1_0 ; real_T B_0_2_0 [ 4 ] ;
real_T B_0_3_0 ; real_T B_0_4_0 ; real_T B_0_5_0 [ 12 ] ; real_T B_0_6_0 [ 9
] ; real_T B_0_6_1 [ 3 ] ; real_T B_0_7_0 [ 4 ] ; real_T B_0_7_1 [ 2 ] ;
real_T B_0_8_0 [ 12 ] ; real_T B_0_8_1 [ 3 ] ; real_T B_0_8_2 [ 3 ] ; real_T
B_0_8_3 [ 3 ] ; real_T B_0_8_4 [ 3 ] ; real_T B_0_11_0 [ 6 ] ; real_T
B_0_18_0 [ 12 ] ; } B_modelo2a_T ; typedef struct { struct { real_T
TimeStampA ; real_T LastUAtTimeA ; real_T TimeStampB ; real_T LastUAtTimeB ;
} Derivative2_RWORK ; struct { real_T TimeStampA ; real_T LastUAtTimeA ;
real_T TimeStampB ; real_T LastUAtTimeB ; } Derivative1_RWORK ; struct {
real_T TimeStampA ; real_T LastUAtTimeA ; real_T TimeStampB ; real_T
LastUAtTimeB ; } Derivative_RWORK ; struct { real_T modelTStart ; }
TransportDelay_RWORK ; void * FromWorkspace_PWORK [ 3 ] ; void * FromWs_PWORK
[ 3 ] ; struct { void * TUbufferPtrs [ 24 ] ; } TransportDelay_PWORK ; struct
{ void * LoggedData ; } omega_PWORK ; struct { void * LoggedData ; }
pos_PWORK ; struct { void * LoggedData ; } rollpitchyaw_PWORK ; struct { void
* LoggedData ; } vel_PWORK ; struct { void * LoggedData ; } control_PWORK ;
struct { void * LoggedData ; } Scope_PWORK ; void * ToWorkspace_PWORK ; void
* ToWorkspace1_PWORK ; int_T FromWorkspace_IWORK ; int_T FromWs_IWORK ;
struct { int_T Tail [ 12 ] ; int_T Head [ 12 ] ; int_T Last [ 12 ] ; int_T
CircularBufSize [ 12 ] ; int_T MaxNewBufSize ; } TransportDelay_IWORK ; char
pad_TransportDelay_IWORK [ 4 ] ; } DW_modelo2a_T ; typedef struct { real_T
QuadrotorNavigation_CSTATE [ 12 ] ; real_T QuadrotorController_CSTATE [ 12 ]
; real_T Quadrotor_CSTATE [ 12 ] ; } X_modelo2a_T ; typedef struct { real_T
QuadrotorNavigation_CSTATE [ 12 ] ; real_T QuadrotorController_CSTATE [ 12 ]
; real_T Quadrotor_CSTATE [ 12 ] ; } XDot_modelo2a_T ; typedef struct {
boolean_T QuadrotorNavigation_CSTATE [ 12 ] ; boolean_T
QuadrotorController_CSTATE [ 12 ] ; boolean_T Quadrotor_CSTATE [ 12 ] ; }
XDis_modelo2a_T ; struct P_modelo2a_T_ { real_T P_0 ; real_T P_1 ; real_T P_2
; real_T P_3 ; } ; extern P_modelo2a_T modelo2a_rtDefaultP ;
#endif
