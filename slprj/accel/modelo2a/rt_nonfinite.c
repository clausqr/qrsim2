#include "__cf_modelo2a.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "rtGetInf.h"
real_T rtInf ; real_T rtMinusInf ; real_T rtNaN ; real32_T rtInfF ; real32_T
rtMinusInfF ; real32_T rtNaNF ; void rt_InitInfAndNaN ( size_t realSize ) { (
void ) ( realSize ) ; rtNaN = rtGetNaN ( ) ; rtNaNF = rtGetNaNF ( ) ; rtInf =
rtGetInf ( ) ; rtInfF = rtGetInfF ( ) ; rtMinusInf = rtGetMinusInf ( ) ;
rtMinusInfF = rtGetMinusInfF ( ) ; } boolean_T rtIsInf ( real_T value ) {
return ( boolean_T ) ( ( value == rtInf || value == rtMinusInf ) ? 1U : 0U )
; } boolean_T rtIsInfF ( real32_T value ) { return ( boolean_T ) ( ( ( value
) == rtInfF || ( value ) == rtMinusInfF ) ? 1U : 0U ) ; } boolean_T rtIsNaN (
real_T value ) { return ( boolean_T ) ( ( value != value ) ? 1U : 0U ) ; }
boolean_T rtIsNaNF ( real32_T value ) { return ( boolean_T ) ( ( ( value !=
value ) ? 1U : 0U ) ) ; }
