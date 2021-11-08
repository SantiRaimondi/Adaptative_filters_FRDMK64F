/*  Autor: Santiago Raimondi.
    Fecha: 08/11/2021.

    Notas: 

 */

/**
 * @file    Adaptative_filters_FRDMK64F.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

/* TODO: insert other include files here. */
#define NUM_TAPS (uint8_t) 30   /* Cantidad de coeficientes del filtro FIR que simula ser la planta */
#define BLOCK_SIZE              /* Cantidad de muestras que se procesan por llamada del filtro */

/* TODO: insert other definitions and declarations here. */

/* Arreglo aleatorio con 30 coeficientes que simulan la planta */
q15_t coeficients[NUM_TAPS] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};

/*
 * @brief   Application entry point.
 */
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
//    rand(); // Para crear valores pseudo-random
    while(1) {

    }
    return 0 ;
}
