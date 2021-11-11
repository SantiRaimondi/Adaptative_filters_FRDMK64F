/*  Autor: Santiago Raimondi.
    Fecha: 09/11/2021.

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

    	Con datos de tipo f32 funciona bien, con q15 no funciona.

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
#define NUMFRAMES 			1000	/* Cantidad de iteraciones para estimar la planta */
#define POST_SHIFT	(uint8_t )  0 	/* Coef. de escaleo para que los coeficientes del filtro puedan superar los valores de [-1, 1) */

//#define __DEBUGG 	/* Variable de debugeo */

/* Threashold values to check the converging error */
/* Modificar a valores q15 */
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
    /* -------------------------------------------------------------------------------
    * Test whether the error signal has reached towards 0.
    * ----------------------------------------------------------------------------- */
    arm_abs_q15(err_signal, err_signal, BLOCKSIZE);
    arm_min_q15(err_signal, BLOCKSIZE, &minValue, &index);

	#ifdef __DEBUGG
    	arm_status status;
        status = ARM_MATH_SUCCESS;

        if (minValue > DELTA_ERROR)
        {
            status = ARM_MATH_TEST_FAILURE;
        }

        PRINTF("MinValue of err_signal: %f\r\n", minValue);
    #endif

    /* ----------------------------------------------------------------------
    * Test whether the filter coefficients have converged.
    * ------------------------------------------------------------------- */
    arm_sub_q15(FIRCoeff_q15, lmsNormCoeff_q15, lmsNormCoeff_q15, NUMTAPS);
    arm_abs_q15(lmsNormCoeff_q15, lmsNormCoeff_q15, NUMTAPS);
    arm_min_q15(lmsNormCoeff_q15, NUMTAPS, &minValue, &index);

    #ifdef __DEBUGG
    	status = (minValue > DELTA_COEFF) ? ARM_MATH_TEST_FAILURE : ARM_MATH_SUCCESS;

    	PRINTF("MinValue of coef_err: %f\r\n", minValue);

        if (status != ARM_MATH_SUCCESS)
            PRINTF("FAILURE\r\n");
        else
            PRINTF("SUCCESS\r\n");

        PRINTF("\r\n");
	#endif



    /* Se crea la trama de salida
     * El tx_buffer es de tamaño x4 porque el tamaño de dato que se puede
     * transmitir es de 8bits y cada dato q15_t ocupa 2 bytes y se meten los
     * coeficientes de ambos filtros en un solo arreglo
     *
     * La trama es:
     * 	Byte N°     |       Data
			0       |   MSE LowerByte
			1       |       "
			2       |       "
			3       |   MSE HigherByte
			4       |   lmsNormCoeff_q15[0] LowByte
			5       |   lmsNormCoeff_q15[0] HighByte
		   ...      |       "
			66      |   lmsNormCoeff_q15[29] LowByte
			67      |   lmsNormCoeff_q15[29] HighByte
			68      |   FIRCoeff_q15[0] LowByte
			69      |   FIRCoeff_q15[0] HighByte
		   ...      |       "
		   126      |   FIRCoeff_q15[29] LowByte
		   127      |   FIRCoeff_q15[29] HighByte
     */
    uint8_t tx_buffer[NUMTAPS*4];
    uint8_t* tx_buffer_ptr = tx_buffer;

    for(uint8_t j = 0; j < 2; j++)
    {

    	for(uint8_t i = 0; i < NUMTAPS; i++)
    	{
			/* Se guardan 8 bits menos significativos, se incrementa el puntero
			* y luego se guardan los 8 bits mas significativos.
			* Primero se llena con los valores de los coeficientes del filtro LMS y
			* luego con los del filtro FIR (planta). Se hace asi para respetar la
			* trama propuesta mas arriba.
			*/
			if(j == 0)
			{
			    *tx_buffer_ptr = (uint8_t) lmsNormCoeff_q15[i] & 0x0FF;
			    tx_buffer_ptr++;
			    *tx_buffer_ptr = (uint8_t) (lmsNormCoeff_q15[i] >> 8) ;
			    tx_buffer_ptr++;
			}
			else
			{
				*tx_buffer_ptr  = (uint8_t) FIRCoeff_q15[i] & 0x0FF;
				tx_buffer_ptr++;
				*tx_buffer_ptr = (uint8_t) (FIRCoeff_q15[i] >> 8) & 0x0FF;
				tx_buffer_ptr++;
			}
        }
    }

    /* Se hace la transmision de los datos */
	#ifndef	__DEBUGG
    	UART_WriteBlocking(UART0, tx_buffer, NUMTAPS*4);
    #endif

    while (1) {}

}

