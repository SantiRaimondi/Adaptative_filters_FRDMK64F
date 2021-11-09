/*  Autor: Santiago Raimondi.
    Fecha: 08/11/2021.

	Consigna:

	Realizar un programa en C utilizando el ambiente como simulador, que muestre el proceso adaptativo
	de un filtro transversal de 30 coeficientes, utilizando el algoritmo LMS y estudiando el mismo con
	diferentes valores de la constante de velocidad de adaptación (μ), cuando se utiliza en el esquema
	“identificación de planta” (figura 1).

	A tal fin será necesario:

	1) Simular en el mismo programa una planta a identificar de entrada única, con una respuesta al
	impulso de 30 muestras de largo, colocando valores arbitrarios (fijos) en sus coeficientes.

	2) Escribir en el mismo programa una rutina que calcule el MSE promediando cada 100 muestras a fin
	de ver el proceso de convergencia. Mostrar en ese momento (cada 100 muestras) la diferencia entre
	los valores fijos de los coeficientes de la planta a identificar y los coeficientes del filtro adaptativo.
	
	3) Utilizar como entrada una señal pseudorandom, por ejemplo, con una función random del ambiente.
	Esta señal brinda suficiente variedad en la entrada para analizar la convergencia.

	Analizar el resultado con diferentes valores de μ y de la potencia de la señal random de entrada.


    Notas: 

 */

#include "arm_math.h"
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* ----------------------------------------------------------------------
** Global defines for the simulation
* ------------------------------------------------------------------- */
#define NUMTAPS  (uint32_t)	 30
#define BLOCKSIZE            100
#define MU       (q15_t)	1
#define NUMFRAMES 			10000	/* Cantidad de iteraciones para estimar la planta */
#define POST_SHIFT	(uint8_t )  0 	/* Coef. de escaleo para que los coeficientes del filtro puedan superar los valores de [-1, 1) */
/* Threashold values to check the converging error */
#define DELTA_ERROR         0.0009f
#define DELTA_COEFF         0.001f

/* ----------------------------------------------------------------------
* Declare FIR state buffers and structure
* ------------------------------------------------------------------- */
q15_t firStateQ15[NUMTAPS + BLOCKSIZE];
arm_fir_instance_q15 LPF_instance;

/* ----------------------------------------------------------------------
* Declare LMSNorm state buffers and structure
* ------------------------------------------------------------------- */
q15_t lmsStateQ15[NUMTAPS + BLOCKSIZE];
arm_lms_norm_instance_q15 lmsNorm_instance;

/* ----------------------------------------------------------------------
* Auxiliar Declarations for FIR Q15 module Test
* ------------------------------------------------------------------- */

q15_t lmsNormCoeff_q15[NUMTAPS] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};

const q15_t FIRCoeff_q15[NUMTAPS] = {5,		10,		20,		40,		80,	  160,	320,	640,	1320,	2640,
		5280, 10560,	21120, 21120, 21120, 21120, 21120, 21120, 10560, 	5280,
		2640, 	1320, 	640,	320, 	160, 	80, 	40, 	20, 	10, 	5};

/* ----------------------------------------------------------------------
* Declare I/O buffers
* ------------------------------------------------------------------- */
q15_t wire1[BLOCKSIZE];	/*src*/
q15_t wire2[BLOCKSIZE];	/*out*/
q15_t wire3[BLOCKSIZE]; /*ref*/
q15_t err_signal[BLOCKSIZE];	/*err*/

/* ----------------------------------------------------------------------
* Signal converge test
* ------------------------------------------------------------------- */
int32_t main(void)
{
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    #ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
        /* Init FSL debug console. */
        BOARD_InitDebugConsole();
    #endif

    arm_status status;
    uint32_t index;
    q15_t minValue;
    /* Initialize the LMSNorm data structure */
    arm_lms_norm_init_q15(&lmsNorm_instance, NUMTAPS, lmsNormCoeff_q15, lmsStateQ15, MU, BLOCKSIZE, POST_SHIFT);
    /* Initialize the FIR data structure */
    arm_fir_init_q15(&LPF_instance, NUMTAPS, (q15_t *)FIRCoeff_q15, firStateQ15, BLOCKSIZE);
    /* ----------------------------------------------------------------------
    * Loop over the frames of data and execute each of the processing
    * functions in the system.
    * ------------------------------------------------------------------- */
    for(uint32_t i=0; i < NUMFRAMES; i++)
    {
        /* Se crea la input data usando la funcion rand() */
        for (int j = 0; j < BLOCKSIZE; j++)
        {
            wire1[j] = (q15_t)(rand()/(q15_t)(RAND_MAX));
        }

        /* Execute the FIR processing function.  Input wire1 and output wire2 */
        arm_fir_q15(&LPF_instance, wire1, wire2, BLOCKSIZE);
        /* Execute the LMS Norm processing function*/
        arm_lms_norm_q15(&lmsNorm_instance, /* LMSNorm instance */
            wire1,                         /* Input signal */
            wire2,                         /* Reference Signal */
            wire3,                         /* Converged Signal */
            err_signal,                    /* Error Signal, this will become small as the signal converges */
            BLOCKSIZE);                    /* BlockSize */
        /* apply overall gain */
//        arm_scale_f32(wire3, 5, wire3, BLOCKSIZE);   /* in-place buffer */
    }
    status = ARM_MATH_SUCCESS;
    /* -------------------------------------------------------------------------------
    * Test whether the error signal has reached towards 0.
    * ----------------------------------------------------------------------------- */
    arm_abs_q15(err_signal, err_signal, BLOCKSIZE);
    arm_min_q15(err_signal, BLOCKSIZE, &minValue, &index);
    if (minValue > DELTA_ERROR)
    {
        status = ARM_MATH_TEST_FAILURE;
    }

    PRINTF("MinValue of err_signal: %f\r\n", minValue);

    /* ----------------------------------------------------------------------
    * Test whether the filter coefficients have converged.
    * ------------------------------------------------------------------- */
    arm_sub_q15(FIRCoeff_q15, lmsNormCoeff_q15, lmsNormCoeff_q15, NUMTAPS);
    arm_abs_q15(lmsNormCoeff_q15, lmsNormCoeff_q15, NUMTAPS);
    arm_min_q15(lmsNormCoeff_q15, NUMTAPS, &minValue, &index);
    status = (minValue > DELTA_COEFF) ? ARM_MATH_TEST_FAILURE : ARM_MATH_SUCCESS;

    PRINTF("MinValue of coef_err: %f\r\n", minValue);

    if (status != ARM_MATH_SUCCESS)
        PRINTF("FAILURE\r\n");
    else
        PRINTF("SUCCESS\r\n");

    PRINTF("\r\n");

    while (1) {}

}

