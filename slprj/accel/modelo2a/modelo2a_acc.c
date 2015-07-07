#include "__cf_modelo2a.h"
#include <math.h>
#include "modelo2a_acc.h"
#include "modelo2a_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
#ifndef __RTW_UTFREE__  
extern void * utMalloc ( size_t ) ; extern void utFree ( void * ) ;
#endif
boolean_T modelo2a_acc_rt_TDelayUpdateTailOrGrowBuf ( int_T * bufSzPtr ,
int_T * tailPtr , int_T * headPtr , int_T * lastPtr , real_T tMinusDelay ,
real_T * * tBufPtr , real_T * * uBufPtr , real_T * * xBufPtr , boolean_T
isfixedbuf , boolean_T istransportdelay , int_T * maxNewBufSzPtr ) { int_T
testIdx ; int_T tail = * tailPtr ; int_T bufSz = * bufSzPtr ; real_T * tBuf =
* tBufPtr ; real_T * xBuf = ( NULL ) ; int_T numBuffer = 2 ; if (
istransportdelay ) { numBuffer = 3 ; xBuf = * xBufPtr ; } testIdx = ( tail <
( bufSz - 1 ) ) ? ( tail + 1 ) : 0 ; if ( ( tMinusDelay <= tBuf [ testIdx ] )
&& ! isfixedbuf ) { int_T j ; real_T * tempT ; real_T * tempU ; real_T *
tempX = ( NULL ) ; real_T * uBuf = * uBufPtr ; int_T newBufSz = bufSz + 1024
; if ( newBufSz > * maxNewBufSzPtr ) { * maxNewBufSzPtr = newBufSz ; } tempU
= ( real_T * ) utMalloc ( numBuffer * newBufSz * sizeof ( real_T ) ) ; if (
tempU == ( NULL ) ) { return ( false ) ; } tempT = tempU + newBufSz ; if (
istransportdelay ) tempX = tempT + newBufSz ; for ( j = tail ; j < bufSz ; j
++ ) { tempT [ j - tail ] = tBuf [ j ] ; tempU [ j - tail ] = uBuf [ j ] ; if
( istransportdelay ) tempX [ j - tail ] = xBuf [ j ] ; } for ( j = 0 ; j <
tail ; j ++ ) { tempT [ j + bufSz - tail ] = tBuf [ j ] ; tempU [ j + bufSz -
tail ] = uBuf [ j ] ; if ( istransportdelay ) tempX [ j + bufSz - tail ] =
xBuf [ j ] ; } if ( * lastPtr > tail ) { * lastPtr -= tail ; } else { *
lastPtr += ( bufSz - tail ) ; } * tailPtr = 0 ; * headPtr = bufSz ; utFree (
uBuf ) ; * bufSzPtr = newBufSz ; * tBufPtr = tempT ; * uBufPtr = tempU ; if (
istransportdelay ) * xBufPtr = tempX ; } else { * tailPtr = testIdx ; }
return ( true ) ; } real_T modelo2a_acc_rt_TDelayInterpolate ( real_T
tMinusDelay , real_T tStart , real_T * tBuf , real_T * uBuf , int_T bufSz ,
int_T * lastIdx , int_T oldestIdx , int_T newIdx , real_T initOutput ,
boolean_T discrete , boolean_T minorStepAndTAtLastMajorOutput ) { int_T i ;
real_T yout , t1 , t2 , u1 , u2 ; if ( ( newIdx == 0 ) && ( oldestIdx == 0 )
&& ( tMinusDelay > tStart ) ) return initOutput ; if ( tMinusDelay <= tStart
) return initOutput ; if ( ( tMinusDelay <= tBuf [ oldestIdx ] ) ) { if (
discrete ) { return ( uBuf [ oldestIdx ] ) ; } else { int_T tempIdx =
oldestIdx + 1 ; if ( oldestIdx == bufSz - 1 ) tempIdx = 0 ; t1 = tBuf [
oldestIdx ] ; t2 = tBuf [ tempIdx ] ; u1 = uBuf [ oldestIdx ] ; u2 = uBuf [
tempIdx ] ; if ( t2 == t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; } else {
yout = u1 ; } } else { real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; yout = f1 * u1 + f2 * u2 ; } return yout ; } } if (
minorStepAndTAtLastMajorOutput ) { if ( newIdx != 0 ) { if ( * lastIdx ==
newIdx ) { ( * lastIdx ) -- ; } newIdx -- ; } else { if ( * lastIdx == newIdx
) { * lastIdx = bufSz - 1 ; } newIdx = bufSz - 1 ; } } i = * lastIdx ; if (
tBuf [ i ] < tMinusDelay ) { while ( tBuf [ i ] < tMinusDelay ) { if ( i ==
newIdx ) break ; i = ( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0 ; } } else { while
( tBuf [ i ] >= tMinusDelay ) { i = ( i > 0 ) ? i - 1 : ( bufSz - 1 ) ; } i =
( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0 ; } * lastIdx = i ; if ( discrete ) {
double tempEps = ( DBL_EPSILON ) * 128.0 ; double localEps = tempEps *
muDoubleScalarAbs ( tBuf [ i ] ) ; if ( tempEps > localEps ) { localEps =
tempEps ; } localEps = localEps / 2.0 ; if ( tMinusDelay >= ( tBuf [ i ] -
localEps ) ) { yout = uBuf [ i ] ; } else { if ( i == 0 ) { yout = uBuf [
bufSz - 1 ] ; } else { yout = uBuf [ i - 1 ] ; } } } else { if ( i == 0 ) {
t1 = tBuf [ bufSz - 1 ] ; u1 = uBuf [ bufSz - 1 ] ; } else { t1 = tBuf [ i -
1 ] ; u1 = uBuf [ i - 1 ] ; } t2 = tBuf [ i ] ; u2 = uBuf [ i ] ; if ( t2 ==
t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; } else { yout = u1 ; } } else {
real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; yout
= f1 * u1 + f2 * u2 ; } } return ( yout ) ; } static void mdlOutputs (
SimStruct * S , int_T tid ) { int32_T i ; B_modelo2a_T * _rtB ; P_modelo2a_T
* _rtP ; DW_modelo2a_T * _rtDW ; _rtDW = ( ( DW_modelo2a_T * ) ssGetRootDWork
( S ) ) ; _rtP = ( ( P_modelo2a_T * ) ssGetDefaultParam ( S ) ) ; _rtB = ( (
B_modelo2a_T * ) _ssGetBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 0 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; { real_T t = ssGetTaskTime ( S , 0 ) ; real_T
timeStampA = _rtDW -> Derivative2_RWORK . TimeStampA ; real_T timeStampB =
_rtDW -> Derivative2_RWORK . TimeStampB ; real_T * lastU = & _rtDW ->
Derivative2_RWORK . LastUAtTimeA ; if ( timeStampA >= t && timeStampB >= t )
{ ( ( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) -> B_0_1_0 = 0.0 ; } else {
real_T deltaT ; real_T lastTime = timeStampA ; if ( timeStampA < timeStampB )
{ if ( timeStampB < t ) { lastTime = timeStampB ; lastU = & _rtDW ->
Derivative2_RWORK . LastUAtTimeB ; } } else if ( timeStampA >= t ) { lastTime
= timeStampB ; lastU = & _rtDW -> Derivative2_RWORK . LastUAtTimeB ; } deltaT
= t - lastTime ; ( ( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) -> B_0_1_0 = ( (
( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) -> B_0_0_0 - * lastU ++ ) / deltaT ;
} } ssCallAccelRunBlock ( S , 0 , 2 , SS_CALL_MDL_OUTPUTS ) ; { real_T t =
ssGetTaskTime ( S , 0 ) ; real_T timeStampA = _rtDW -> Derivative1_RWORK .
TimeStampA ; real_T timeStampB = _rtDW -> Derivative1_RWORK . TimeStampB ;
real_T * lastU = & _rtDW -> Derivative1_RWORK . LastUAtTimeA ; if (
timeStampA >= t && timeStampB >= t ) { ( ( B_modelo2a_T * ) _ssGetBlockIO ( S
) ) -> B_0_3_0 = 0.0 ; } else { real_T deltaT ; real_T lastTime = timeStampA
; if ( timeStampA < timeStampB ) { if ( timeStampB < t ) { lastTime =
timeStampB ; lastU = & _rtDW -> Derivative1_RWORK . LastUAtTimeB ; } } else
if ( timeStampA >= t ) { lastTime = timeStampB ; lastU = & _rtDW ->
Derivative1_RWORK . LastUAtTimeB ; } deltaT = t - lastTime ; ( ( B_modelo2a_T
* ) _ssGetBlockIO ( S ) ) -> B_0_3_0 = ( ( ( B_modelo2a_T * ) _ssGetBlockIO (
S ) ) -> B_0_2_0 [ 1 ] - * lastU ++ ) / deltaT ; } } { real_T t =
ssGetTaskTime ( S , 0 ) ; real_T timeStampA = _rtDW -> Derivative_RWORK .
TimeStampA ; real_T timeStampB = _rtDW -> Derivative_RWORK . TimeStampB ;
real_T * lastU = & _rtDW -> Derivative_RWORK . LastUAtTimeA ; if ( timeStampA
>= t && timeStampB >= t ) { ( ( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) ->
B_0_4_0 = 0.0 ; } else { real_T deltaT ; real_T lastTime = timeStampA ; if (
timeStampA < timeStampB ) { if ( timeStampB < t ) { lastTime = timeStampB ;
lastU = & _rtDW -> Derivative_RWORK . LastUAtTimeB ; } } else if ( timeStampA
>= t ) { lastTime = timeStampB ; lastU = & _rtDW -> Derivative_RWORK .
LastUAtTimeB ; } deltaT = t - lastTime ; ( ( B_modelo2a_T * ) _ssGetBlockIO (
S ) ) -> B_0_4_0 = ( ( ( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) -> B_0_2_0 [
1 ] - * lastU ++ ) / deltaT ; } } { real_T * * uBuffer = ( real_T * * ) &
_rtDW -> TransportDelay_PWORK . TUbufferPtrs [ 0 ] ; real_T * * tBuffer = (
real_T * * ) & _rtDW -> TransportDelay_PWORK . TUbufferPtrs [ 12 ] ; real_T
simTime = ssGetT ( S ) ; real_T tMinusDelay ; { int_T i1 ; real_T * y0 = ( (
B_modelo2a_T * ) _ssGetBlockIO ( S ) ) -> B_0_5_0 ; int_T * iw_Tail = & _rtDW
-> TransportDelay_IWORK . Tail [ 0 ] ; int_T * iw_Head = & _rtDW ->
TransportDelay_IWORK . Head [ 0 ] ; int_T * iw_Last = & _rtDW ->
TransportDelay_IWORK . Last [ 0 ] ; int_T * iw_CircularBufSize = & _rtDW ->
TransportDelay_IWORK . CircularBufSize [ 0 ] ; for ( i1 = 0 ; i1 < 12 ; i1 ++
) { tMinusDelay = ( ( _rtP -> P_0 > 0.0 ) ? _rtP -> P_0 : 0.0 ) ; tMinusDelay
= simTime - tMinusDelay ; y0 [ i1 ] = modelo2a_acc_rt_TDelayInterpolate (
tMinusDelay , 0.0 , * tBuffer , * uBuffer , iw_CircularBufSize [ i1 ] , &
iw_Last [ i1 ] , iw_Tail [ i1 ] , iw_Head [ i1 ] , _rtP -> P_1 , 0 , (
boolean_T ) ( ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) ==
ssGetT ( S ) ) ) ) ; tBuffer ++ ; uBuffer ++ ; } } } ssCallAccelRunBlock ( S
, 0 , 6 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 7 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 8 , SS_CALL_MDL_OUTPUTS
) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { ssCallAccelRunBlock ( S , 0 , 9 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 10 ,
SS_CALL_MDL_OUTPUTS ) ; } _rtB -> B_0_11_0 [ 0 ] = _rtP -> P_2 * _rtB ->
B_0_0_0 ; _rtB -> B_0_11_0 [ 1 ] = _rtP -> P_2 * _rtB -> B_0_2_0 [ 1 ] ; _rtB
-> B_0_11_0 [ 2 ] = _rtP -> P_2 * _rtB -> B_0_2_0 [ 1 ] ; _rtB -> B_0_11_0 [
3 ] = _rtP -> P_2 * _rtB -> B_0_8_0 [ 6 ] ; _rtB -> B_0_11_0 [ 4 ] = _rtP ->
P_2 * _rtB -> B_0_8_0 [ 7 ] ; _rtB -> B_0_11_0 [ 5 ] = _rtP -> P_2 * _rtB ->
B_0_8_0 [ 8 ] ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { ssCallAccelRunBlock ( S
, 0 , 12 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 13 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 14 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 15 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 16 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 17 ,
SS_CALL_MDL_OUTPUTS ) ; } for ( i = 0 ; i < 12 ; i ++ ) { _rtB -> B_0_18_0 [
i ] = _rtP -> P_3 * _rtB -> B_0_8_0 [ i ] ; } UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { B_modelo2a_T * _rtB ;
P_modelo2a_T * _rtP ; DW_modelo2a_T * _rtDW ; _rtDW = ( ( DW_modelo2a_T * )
ssGetRootDWork ( S ) ) ; _rtP = ( ( P_modelo2a_T * ) ssGetDefaultParam ( S )
) ; _rtB = ( ( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) ; { real_T timeStampA =
_rtDW -> Derivative2_RWORK . TimeStampA ; real_T timeStampB = _rtDW ->
Derivative2_RWORK . TimeStampB ; real_T * lastTime = & _rtDW ->
Derivative2_RWORK . TimeStampA ; real_T * lastU = & _rtDW ->
Derivative2_RWORK . LastUAtTimeA ; if ( timeStampA != rtInf ) { if (
timeStampB == rtInf ) { lastTime = & _rtDW -> Derivative2_RWORK . TimeStampB
; lastU = & _rtDW -> Derivative2_RWORK . LastUAtTimeB ; } else if (
timeStampA >= timeStampB ) { lastTime = & _rtDW -> Derivative2_RWORK .
TimeStampB ; lastU = & _rtDW -> Derivative2_RWORK . LastUAtTimeB ; } } *
lastTime = ssGetTaskTime ( S , 0 ) ; * lastU ++ = ( ( B_modelo2a_T * )
_ssGetBlockIO ( S ) ) -> B_0_0_0 ; } { real_T timeStampA = _rtDW ->
Derivative1_RWORK . TimeStampA ; real_T timeStampB = _rtDW ->
Derivative1_RWORK . TimeStampB ; real_T * lastTime = & _rtDW ->
Derivative1_RWORK . TimeStampA ; real_T * lastU = & _rtDW ->
Derivative1_RWORK . LastUAtTimeA ; if ( timeStampA != rtInf ) { if (
timeStampB == rtInf ) { lastTime = & _rtDW -> Derivative1_RWORK . TimeStampB
; lastU = & _rtDW -> Derivative1_RWORK . LastUAtTimeB ; } else if (
timeStampA >= timeStampB ) { lastTime = & _rtDW -> Derivative1_RWORK .
TimeStampB ; lastU = & _rtDW -> Derivative1_RWORK . LastUAtTimeB ; } } *
lastTime = ssGetTaskTime ( S , 0 ) ; * lastU ++ = ( ( B_modelo2a_T * )
_ssGetBlockIO ( S ) ) -> B_0_2_0 [ 1 ] ; } { real_T timeStampA = _rtDW ->
Derivative_RWORK . TimeStampA ; real_T timeStampB = _rtDW -> Derivative_RWORK
. TimeStampB ; real_T * lastTime = & _rtDW -> Derivative_RWORK . TimeStampA ;
real_T * lastU = & _rtDW -> Derivative_RWORK . LastUAtTimeA ; if ( timeStampA
!= rtInf ) { if ( timeStampB == rtInf ) { lastTime = & _rtDW ->
Derivative_RWORK . TimeStampB ; lastU = & _rtDW -> Derivative_RWORK .
LastUAtTimeB ; } else if ( timeStampA >= timeStampB ) { lastTime = & _rtDW ->
Derivative_RWORK . TimeStampB ; lastU = & _rtDW -> Derivative_RWORK .
LastUAtTimeB ; } } * lastTime = ssGetTaskTime ( S , 0 ) ; * lastU ++ = ( (
B_modelo2a_T * ) _ssGetBlockIO ( S ) ) -> B_0_2_0 [ 1 ] ; } { real_T * *
uBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK . TUbufferPtrs [ 0 ]
; real_T * * tBuffer = ( real_T * * ) & _rtDW -> TransportDelay_PWORK .
TUbufferPtrs [ 12 ] ; real_T simTime = ssGetT ( S ) ; { int_T i1 ; const
real_T * u0 = ( ( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) -> B_0_18_0 ; int_T
* iw_Tail = & _rtDW -> TransportDelay_IWORK . Tail [ 0 ] ; int_T * iw_Head =
& _rtDW -> TransportDelay_IWORK . Head [ 0 ] ; int_T * iw_Last = & _rtDW ->
TransportDelay_IWORK . Last [ 0 ] ; int_T * iw_CircularBufSize = & _rtDW ->
TransportDelay_IWORK . CircularBufSize [ 0 ] ; for ( i1 = 0 ; i1 < 12 ; i1 ++
) { iw_Head [ i1 ] = ( ( iw_Head [ i1 ] < ( iw_CircularBufSize [ i1 ] - 1 ) )
? ( iw_Head [ i1 ] + 1 ) : 0 ) ; if ( iw_Head [ i1 ] == iw_Tail [ i1 ] ) { if
( ! modelo2a_acc_rt_TDelayUpdateTailOrGrowBuf ( & iw_CircularBufSize [ i1 ] ,
& iw_Tail [ i1 ] , & iw_Head [ i1 ] , & iw_Last [ i1 ] , simTime - _rtP ->
P_0 , tBuffer , uBuffer , ( NULL ) , ( boolean_T ) 0 , FALSE , & _rtDW ->
TransportDelay_IWORK . MaxNewBufSize ) ) { ssSetErrorStatus ( S ,
"tdelay memory allocation error" ) ; return ; } } ( * tBuffer ++ ) [ iw_Head
[ i1 ] ] = simTime ; ( * uBuffer ++ ) [ iw_Head [ i1 ] ] = u0 [ i1 ] ; } } }
ssCallAccelRunBlock ( S , 0 , 6 , SS_CALL_MDL_UPDATE ) ; ssCallAccelRunBlock
( S , 0 , 7 , SS_CALL_MDL_UPDATE ) ; ssCallAccelRunBlock ( S , 0 , 8 ,
SS_CALL_MDL_UPDATE ) ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { B_modelo2a_T * _rtB ; _rtB = (
( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 0 , 6 ,
SS_CALL_MDL_DERIVATIVES ) ; ssCallAccelRunBlock ( S , 0 , 7 ,
SS_CALL_MDL_DERIVATIVES ) ; ssCallAccelRunBlock ( S , 0 , 8 ,
SS_CALL_MDL_DERIVATIVES ) ; }
#define MDL_PROJECTION
static void mdlProjection ( SimStruct * S ) { B_modelo2a_T * _rtB ; _rtB = (
( B_modelo2a_T * ) _ssGetBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 0 , 6 ,
SS_CALL_MDL_PROJECTION ) ; ssCallAccelRunBlock ( S , 0 , 7 ,
SS_CALL_MDL_PROJECTION ) ; ssCallAccelRunBlock ( S , 0 , 8 ,
SS_CALL_MDL_PROJECTION ) ; } static void mdlInitializeSizes ( SimStruct * S )
{ ssSetChecksumVal ( S , 0 , 597651231U ) ; ssSetChecksumVal ( S , 1 ,
121915398U ) ; ssSetChecksumVal ( S , 2 , 518877062U ) ; ssSetChecksumVal ( S
, 3 , 2728310348U ) ; { mxArray * slVerStructMat = NULL ; mxArray * slStrMat
= mxCreateString ( "simulink" ) ; char slVerChar [ 10 ] ; int status =
mexCallMATLAB ( 1 , & slVerStructMat , 1 , & slStrMat , "ver" ) ; if ( status
== 0 ) { mxArray * slVerMat = mxGetField ( slVerStructMat , 0 , "Version" ) ;
if ( slVerMat == NULL ) { status = 1 ; } else { status = mxGetString (
slVerMat , slVerChar , 10 ) ; } } mxDestroyArray ( slStrMat ) ;
mxDestroyArray ( slVerStructMat ) ; if ( ( status == 1 ) || ( strcmp (
slVerChar , "8.1" ) != 0 ) ) { return ; } } ssSetOptions ( S ,
SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork ( S ) != sizeof (
DW_modelo2a_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( B_modelo2a_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
P_modelo2a_T ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetDefaultParam ( S , (
real_T * ) & modelo2a_rtDefaultP ) ; rt_InitInfAndNaN ( sizeof ( real_T ) ) ;
} static void mdlInitializeSampleTimes ( SimStruct * S ) { } static void
mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
