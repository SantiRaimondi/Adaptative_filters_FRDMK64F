# Adaptative_filters_FRDMK64F
---
## Índice / Index

1. [Español](#español).
	1.  [Descripción general del programa](#descripción-general-del-programa).
	2.  [Resultados](#resultados).
2. 
   
### Español

Se utiliza un filtro adaptativo para identificar una planta cuya función de transferencia es desconocida, representada
con un filtro FIR.

Para ello se utilizan las funciones de CMSIS para filtros [FIR](https://arm-software.github.io/CMSIS_5/DSP/html/group__FIR.html) y para filtros [LMS](https://arm-software.github.io/CMSIS_5/DSP/html/group__LMS.html).

### Descripcion general del programa

Primero inicializa los elementos de cada filtro de acuerdo a la documentació de CMSIS.

Luego, crea un arreglo de 100 muestras aleatorias utilizando la función de C *rand()*. Se castea el valor devuelto por la función, que es tipo entero signado, a formato de punto fijo. También se utilizan los bits más significativos de dicho entero (en particular los 12 MSB se utilizan, haciendo un shift hacia la derecha de 20 posiciones). Esto sirve para luego poder jugar con la amplitud de dicha señal, al multiplicarla por una variable (*signal_power*), que el usuario puede modificar con uno de los botones de la placa.

Finalmente, se computa la planta (filtro FIR) con las entradas aleatorias y luego con la salida de la planta, las entradas aleatorias y el error (diferencia entre la referencia y la salida anterior) se computa la salida actual del filtro adaptativo. Este proceso se repite *NUMFRAMES* cantidad de veces. 

Para cada repetición, se computa el MSE (medium square error) de la siguiente forma:
```
for(uint16_t k = 0; k < BLOCKSIZE; k++)
{
    mse[i] += err[k] * err [k];
}
mse[i] = mse[i] / BLOCKSIZE;
if(mse[i] > 262143)
    mse[i] = 262143;
```
Al ser el arreglo *mse* de tipo q32_t (se decidió una variable del doble de tamaño que del err, que es de tipo q15_t, debido a que en la operación de multiplicación se duplica el número de bits de la variable involucrada). En un principio se pensó en transmitir sólo los 16 MSB de mse, para evitar transmitir demasiados datos. Pero ocurrió que el error “más grande” al principio, era tan solo de 2^22, por lo que si se utilizan los bits 16 a 32 solo los 6 LSB tendrían valores distintos de 0. Este error no era representativo y no se podía visualizar correctamente. Por lo tanto, se aplicó una saturación al valor 262143 (que es igual a 2^17-1) para utilizar luego los bits entre 2 a 17 del mse. Esto se justifica en que no es relevante para el análisis que el error sea “grande” al principio, pero si es de interés no perder la información de los bits menos significativos ya que van a mostrar hasta qué valor realmente disminuye el error.

Al terminar todas las iteraciones, se envían los valores de los coeficientes y del MSE utilizando el puerto serie para que sean procesados y analizados en forma gráfica por un script desarrollado en Python ([plot_serial.ipynb](./plot_serial.ipynb)) para analizar como afecta el valor de μ (*mu*) del filtro LMS a la precisión de la detección de la planta. También es de interés analizar la relación de 
compromiso entre los valores que puede adoptar μ (*mu*) y la amplitud de la señal de entrada (*signal_power*).

La trama que se utilizó para enviar los datos es la siguiente:

        Byte N°     |       Data
    ----------------------------------------------
            0       |   lms_coeficients[0] LowByte
            1       |   lms_coeficients[0] HighByte
           ...      |           “
            60      |   lms_coeficients[29] LowByte
            61      |   lms_coeficients[29] HighByte
            62      |   fir_coeficients[0] LowByte
            63      |   fir_coeficients[0] HighByte
            ...     |           “
            118     |   FIRCoeff_q15[29] LowByte
            119     |   FIRCoeff_q15[29] HighByte
            120     |   (mse[0] >> 2) & 0x0FF
            121     |   (mse[0] >> 10) & 0x0FF
            ...     |           “
            10118   |   (mse[9999] >> 2) & 0x0FF
            10119   |   (mse[9999] >> 10) & 0x0FF

	
También se implementó un sistema para modificar el μ y la potencia de señal de entrada con los pulsadores de la placa, donde luego de haber incrementado la variable se reinicia el sistema y se corre nuevamente la detección de planta.


### Resultados

A continuación se muestran algunos gráficos con la evolución de los coeficientes y el error para la variación de μ a amplitud de entrada constante:

![variacion_mu](./img/variacion_mu.gif)

A continuación se muestran algunos gráficos con la evolución de los coeficientes y el error para la variación de la amplitud de entrada a μ constante:

![variacion_pwr](./img/variacion_pwr.gif)
