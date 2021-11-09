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

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
//#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "arm_math.h"

#define NUM_TAPS 	(uint16_t ) 30	/* Cantidad de coeficientes del filtro FIR que simula ser la planta y del filtro LMS */
#define BLOCK_SIZE 	(uint32_t ) 100	/* Cantidad de muestras que se procesan por llamada del filtro LMS */
#define POST_SHIFT	(uint8_t )  0 	/* Coef. de escaleo para que los coeficientes del filtro puedan superar los valores de [-1, 1) */
//#define _DEBUGG						/* Variable de debugg */
/* Variables globales */

// QUE MU Y LA POTENCIA DE LA ENTRADA SEAN PARAMETROS MODIFICABLES CON LOS SW DE LA PLACA
q15_t mu = 100;	// INICIALIZAR A ALGUN VALOR RAZONABLE
q15_t input_signal_power = 1; // IDEM ANTERIOR

q63_t mse;	/* El valor devuelto por la funcion arm_power_q15 es de tipo q63_t */

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Muestra del TP4 de DSP.\r\n");

    /* Variables locales.
     * Se definen las variables necesarias para las funciones de filtrado y
     * se las inicializa.
     */

    arm_fir_instance_q15 plant;	/* Planta que el sistema adaptativo debería identificar */
//    arm_lms_norm_instance_q15 lms_filter;
    arm_lms_instance_q15 lms_filter;

    /* Punto de inicio del filtro adaptativo */
    q15_t lms_coeficients[NUM_TAPS] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};

    /* Arreglo aleatorio con 30 coeficientes que simulan la planta */
    const q15_t plant_coeficients[NUM_TAPS] = {5,		10,		20,		40,		80,	  160,	320,	640,	1320,	2640,
    											5280, 10560,	21120, 21120, 21120, 21120, 21120, 21120, 10560, 	5280,
												2640, 	1320, 	640,	320, 	160, 	80, 	40, 	20, 	10, 	5};

    q15_t plant_state[NUM_TAPS + BLOCK_SIZE];	/* Es de este tamaño porque se ejecuta en un Cortex-M4 */
    q15_t lms_state[NUM_TAPS + BLOCK_SIZE - 1];

    q15_t src[BLOCK_SIZE];	/* Entrada a la planta y al sistema adaptativo (por ser un problema de deteccion de planta) */
    q15_t out[BLOCK_SIZE];	/* Salida del sistema adaptativo */
    q15_t ref[BLOCK_SIZE];	/* Salida de la planta (señal de referencia para el sistema adaptativo) */
    q15_t err[BLOCK_SIZE];	/* Diferencia entre ref - out. Es la señal que se realimenta al sistema adaptativo */

//    arm_lms_norm_init_q15(&lms_filter, NUM_TAPS, lms_coeficients, lms_state, mu, BLOCK_SIZE, POST_SHIFT);
    arm_lms_init_q15(&lms_filter, NUM_TAPS, lms_coeficients, lms_state, mu, BLOCK_SIZE, POST_SHIFT);

    arm_fir_init_q15(&plant, NUM_TAPS, plant_coeficients, plant_state, BLOCK_SIZE);

	#ifdef _DEBUGG
    	uint16_t contador =0;
	#endif

    while(1) {

    	/* Se llena el buffer de muestras aleatorias */
    	for(int i = 0; i < BLOCK_SIZE; i++)
    	{
//    		src[i] = (q15_t) (rand()>>16);
    		src[i] = (q15_t) rand();
    	}

    	arm_fir_q15(&plant, src, ref, BLOCK_SIZE);	/* Se computa la planta */

    	// arm_lms_norm_q15(&lms_filter, src, ref, out, err, BLOCK_SIZE);	/* Se computa el filtro adaptativo */
    	arm_lms_q15(&lms_filter, src, ref, out, err, BLOCK_SIZE);	/* Se computa el filtro adaptativo */

		/* Se computa el error medio cuadratico (MSE) */
		arm_power_q15(err, BLOCK_SIZE, &mse);
		mse = mse / BLOCK_SIZE;

		#ifdef _DEBUGG
			contador ++;
			if(contador>1000)
			{
				contador = 0;
			    for(int i=0; i<NUM_TAPS; i++)
			    	PRINTF("%d/r/n", lms_coeficients[i]);
			}
		#endif
    }



    return 0 ;
}
