/*  Autor: Santiago Raimondi.
    Fecha: 11/11/2021.
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
    @brief:
    El programa primero inicializa las funciones de los filtros.
	Luego, crea un arreglo de 100 muestras aleatorias utilizando la función de C rand() la cual se acota
	a valores entre [-1, 1]. Al finalizar esta operación se lo castea a punto fijo al resultado.
	Finalmente, se computa la planta (filtro FIR) con las entradas aleatorias y luego con la salida de la
	planta, las entradas aleatorias y el error (diferencia entre la referencia y la salida anterior) se computa
	la salida actual del filtro adaptativo. Este proceso se repite NUMFRAMES cantidad de veces, en este caso se
	llegó a la conclusión de que con 1000 veces se llega a una aproximación bastante cercana al valor real de la
	planta.
	Al terminar todas las iteraciones, se envían los valores de los coeficientes utilizando el puerto serie para
	que sean procesados y analizados en forma gráfica por un script desarrollado en Python.

	ATENCION: No usar filtros normalizados (lms_norm_q15) porque normalizan la salida y no se puede observar
	los cambios de mu o de amplitud de señal.
 */

#include "arm_math.h"
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#define NUMTAPS (uint16_t) 30
#define BLOCKSIZE (uint32_t) 100
#define POSTSHIFT (uint32_t) 0
#define NUMFRAMES (uint16_t) 5000

volatile q15_t mu = 1;
volatile q15_t signal_power = 1;	/* Amplitud de la señal de entrada */
volatile bool restart = false;

/* SW2 Interr.: Se actualiza el valor de la potencia de señal */
void GPIOC_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOC);

  /* Place your interrupt code here */
  restart = true;
  if(signal_power >= 10000)
  {
	  signal_power = signal_power + 3000;	/* Para signal_power alto, se usa un incremento chico */
  }
  else
  {
	  signal_power = signal_power * 10;	/* Para signal_power bajo, se usa un incremento alto */
  }

  if(signal_power >= 28000)
  {
	  signal_power = 28000;	/* Se satura el signal_power a 28000 */
  }

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOC, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


/* SW3 Interr.: Se aumenta el valor de mu */
void GPIOA_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOA);

  /* Place your interrupt code here */
  restart = true;
  if(mu >= 10000)
  {
	  mu = mu + 3000;	/* Para mu alto, se usa un incremento chico */
  }
  else
  {
	  mu = mu * 10;	/* Para mu bajo, se usa un incremento alto */
  }

  if(mu >= 28000)
  {
	  mu = 28000;	/* Se satura el mu a 28000 */
  }

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOA, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}



int main()
{
	BOARD_InitBootPeripherals();
	BOARD_InitBootClocks();
	BOARD_InitBootPins();
	BOARD_InitDebugConsole();

	/****************************************************************
	 * Se crean las instancias de cada filtro de acuerdo a la
	 * documentacion oficial CMSIS.
	 ****************************************************************
	 */

	/* Filtro FIR (planta) */
	arm_fir_instance_q15 fir_struct;
	q15_t fir_coeficients[NUMTAPS] = {5,		10,		20,		40,		80,	  160,	320,	640,	1320,	2640,
									5280, 10560,	21120, 21120, 21120, 21120, 21120, 21120, 10560, 	5280,
									2640, 	1320, 	640,	320, 	160, 	80, 	40, 	20, 	10, 	5};
	q15_t fir_state[NUMTAPS + BLOCKSIZE - 1];
	arm_fir_init_q15(&fir_struct, NUMTAPS, fir_coeficients, fir_state, BLOCKSIZE);

	/* Filtro LMS */
	arm_lms_instance_q15 lms_struct;
	q15_t lms_coeficients[NUMTAPS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	q15_t lms_state[NUMTAPS + BLOCKSIZE - 1];

	/* Buffers auxiliares para computar algoritmo LMS */
	q15_t src[BLOCKSIZE];
	q15_t ref[BLOCKSIZE];
	q15_t out[BLOCKSIZE];
	q15_t err[BLOCKSIZE];

    /* Variables para guardar la evolucion del error */
    q31_t mse[NUMFRAMES];

	while(1)
	{

		restart = false;	/* Para que se ejecuta una vez la deteccion de planta*/

		/* Se resetea el valor de los coficientes del filtro LMS*/
		for(uint8_t i = 0; i < NUMTAPS; i++)
			lms_coeficients[i] = 0;

		/* Se inicializa el filtro LMS para una nueva deteccion de planta */
		arm_lms_init_q15(&lms_struct, NUMTAPS, lms_coeficients, lms_state, mu, BLOCKSIZE, POSTSHIFT);

		for(uint16_t i = 0; i < NUMFRAMES; i++)
		{
			/* Se construye la señal aleatoria. Se shiftea el valor devuelto
			 * para que sea una señal pequeña y no sature el calculo del filtro.
			 * Recordar que la amplitud de señal de entrada y mu tienen una
			 * relacion de compromiso para la velocidad de convergencia del
			 * algoritmo. Si mu es muy grande o la señal de entrada es muy
			 * grande, el algoritmo puede diverger.
			 */
			for(uint16_t j = 0; j < BLOCKSIZE; j++)
			{
				src[j] = (q15_t)(rand()>>20) * signal_power;
			}

			arm_fir_q15(&fir_struct, src, ref, BLOCKSIZE);

			arm_lms_q15(&lms_struct, src, ref, out, err, BLOCKSIZE);

			/* Se computa el MSE para cada iteracion */
	        for(uint16_t k = 0; k < BLOCKSIZE; k++)
	        {
	        	mse[i] += err[k] * err [k];
	        }

	        mse[i] = mse[i] / BLOCKSIZE;

	        /* Se satura el error para poder enviar los bits menos significativos.
	         * No interesa que el error sea grande al principio, pero si es importante
	         * saber que tan pequeño es al final.
	         * Se satura al valor de 2^17.
	         */
	        if(mse[i] > 262143)
	        	mse[i] = 262143;
		}

		/* Se crea la trama de salida
		 * El tx_buffer es de tamaño x4 porque el tamaño de dato que se puede
		 * transmitir es de 8bits y cada dato q15_t ocupa 2 bytes y se meten los
		 * coeficientes de ambos filtros en un solo arreglo
		 *
		 * La trama es:
		 * 	Byte N°     |       Data
				0       |   lms_coeficients[0] LowByte
				1       |   lms_coeficients[0] HighByte
			   ...      |       "
				60      |   lms_coeficients[29] LowByte
				61      |   lms_coeficients[29] HighByte
				62      |   fir_coeficients[0] LowByte
				63      |   fir_coeficients [0] HighByte
			   ...      |       "
			   118      |   fir_coeficients[29] LowByte
			   119      |   fir_coeficients[29] HighByte
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
					*tx_buffer_ptr = (uint8_t) lms_coeficients[i] & 0x0FF;
					tx_buffer_ptr++;
					*tx_buffer_ptr = (uint8_t) (lms_coeficients[i] >> 8) & 0x0FF ;
					tx_buffer_ptr++;
				}
				else
				{
					*tx_buffer_ptr  = (uint8_t) fir_coeficients[i] & 0x0FF;
					tx_buffer_ptr++;
					*tx_buffer_ptr = (uint8_t) (fir_coeficients[i] >> 8) & 0x0FF;
					tx_buffer_ptr++;
				}
			}
		}

		/* Se hace la transmision de los datos */
		UART_WriteBlocking(UART0, tx_buffer, NUMTAPS*4);

		/* Se transmite la informacion del error */

		uint8_t err_tx_buffer[NUMFRAMES*2];
		uint8_t* err_tx_buffer_ptr = err_tx_buffer;

		for(uint16_t i = 0; i < NUMFRAMES; i++)
		{
			/* Se descartan los dos bits LSB. No importan los bits mayores a 17 ya que
			 * esta saturado. El rango de importancia es entre 2 a 17 bits.
			 */
			*err_tx_buffer_ptr = (uint8_t) (mse[i] >> 2) & 0x0FF;
			err_tx_buffer_ptr++;
			*err_tx_buffer_ptr = (uint8_t) (mse[i] >> 10) & 0x0FF ;
			err_tx_buffer_ptr++;
		}

		UART_WriteBlocking(UART0, err_tx_buffer, NUMFRAMES*2);

		/* Se espera hasta que el usuario haga cambie el mu o la potencia de entrada */
		while(!restart){}
	}
}
