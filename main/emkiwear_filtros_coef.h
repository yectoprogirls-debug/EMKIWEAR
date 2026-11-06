#ifndef EMKIWEAR_FILTROS_COEF_H
#define EMKIWEAR_FILTROS_COEF_H

/*
 * emkiwear_filtros_coef.h
 * ---------------------------------------------------------------------------
 * Coeficientes de los filtros del analizador de temblor, diseñados y
 * validados en MATLAB/SciPy con datos reales (4 actividades cotidianas y
 * 17 pruebas con un robot a 3-7 Hz):
 *   - EMK_BP_SOS: Butterworth pasa-banda de orden 3, 2.5-8 Hz (3 secciones
 *     de segundo orden, ganancia incluida en la primera seccion).
 *   - EMK_HP_SOS: Butterworth pasa-altos de orden 2, 0.3 Hz (1 seccion).
 *
 * VALIDOS SOLO PARA fs = EMK_FS_HZ (50 Hz). Si la frecuencia de muestreo del
 * proyecto cambia, estos coeficientes YA NO SIRVEN (los cortes se desplazan):
 * hay que regenerarlos con emkiwear_filtros.m (MATLAB) para la nueva fs y
 * volver a pegarlos aqui. Como fs_hz llega a tremor_create() como parametro
 * en tiempo de ejecucion (no es una constante de compilacion), el chequeo
 * de coherencia NO puede hacerse con un _Static_assert: tremor_create()
 * compara fs_hz contra EMK_FS_HZ y avisa por ESP_LOGE si no coinciden.
 *
 * Formato de cada fila: {b0, b1, b2, a1, a2} con a0 = 1 (ya normalizado).
 * ---------------------------------------------------------------------------
 */

#define EMK_FS_HZ 50

#define EMK_BP_N_BIQUADS 3
static const float EMK_BP_SOS[3][5] = {
    { 2.303355220e-02f, 4.606710439e-02f, 2.303355220e-02f, -1.234985108e+00f, 4.705642812e-01f },
    { 1.000000000e+00f, 0.000000000e+00f, -1.000000000e+00f, -9.299606635e-01f, 6.219943598e-01f },
    { 1.000000000e+00f, -2.000000000e+00f, 1.000000000e+00f, -1.731027913e+00f, 8.306046148e-01f },
};

#define EMK_HP_N_BIQUADS 1
static const float EMK_HP_SOS[1][5] = {
    { 9.736948117e-01f, -1.947389623e+00f, 9.736948117e-01f, -1.946697541e+00f, 9.480817061e-01f },
};

/*
 * Detector V2: caracteristicas por eje sobre la ventana de 5 s (250
 * muestras a 50 Hz), validado con datos reales (48/48 ventanas de robot
 * detectadas, 0/25 falsos positivos en actividades cotidianas). Ver
 * tremor_process() en tremor.c para el detalle del calculo y la votacion.
 */
#define EMK_SIGMA_BANDA_LO_HZ   2.5f
#define EMK_SIGMA_BANDA_HI_HZ   7.5f
#define EMK_SIGMA_PASO_HZ       0.125f   /* 41 bins */
#define EMK_WELCH_SEG           100      /* 2 s a 50 Hz */
#define EMK_WELCH_SALTO         50       /* 50% de solape -> 4 segmentos en 250 muestras */
#define EMK_UMBRAL_SIGMA_HZ     0.40f
#define EMK_UMBRAL_PREL_PCT     60.0f
#define EMK_UMBRAL_RMS_MIN_DPS  3.0f
#define EMK_VOTOS_MIN           2
#define EMK_K_CAL               0.938f   /* calibracion de severidad con el robot */

/*
 * Graduacion de severidad LEVE/MODERADO/GRAVE por theta_cal_deg (angulo
 * calibrado, ver tremor.c). Escala unica calibrada con el banco robotico
 * (matriz 3-7 Hz, ±0.5° a ±6°) y con temblor imitado por personas (9
 * registros). Aprox. MDS-UPDRS amplitud 1/2/3. Solo aplican si hay temblor
 * detectado; si no, la clase es SIN_TEMBLOR o VOLUNTARIO (ver tremor.c).
 */
#define EMK_SEV_LEVE_MAX_DEG      1.5f  /* < 1.5°  -> LEVE */
#define EMK_SEV_MODERADO_MAX_DEG  4.5f  /* < 4.5°  -> MODERADO; >= 4.5° -> GRAVE */
#define EMK_PALANCA_CM           15.0f  /* solo para mostrar el desplazamiento (desplaz_cm) */

/*
 * Compuerta de energia voluntaria (teorema de Parseval: la varianza de una
 * señal ya filtrada en una sub-banda ES la energia de esa sub-banda, sin
 * necesidad de FFT). Un temblor parkinsoniano (3-8 Hz) casi no tiene energia
 * por debajo de 2.5 Hz; un movimiento voluntario rapido (agitar la mano,
 * gesticular, ~1-2 Hz) si, y eso se ve como una fraccion alta de la energia
 * de 1-10 Hz concentrada en la sub-banda 1-2.5 Hz. Ambos filtros se aplican
 * sobre la MISMA señal que alimenta a EMK_BP_SOS: el giro tras el pasa-altos
 * de 0.3 Hz (hp_val en tremor.c), no sobre el giro crudo.
 *   - EMK_VOL_SOS:  Butterworth pasa-banda orden 2, 1-2.5 Hz (2 secciones).
 *   - EMK_B110_SOS: Butterworth pasa-banda orden 2, 1-10 Hz (2 secciones).
 * Solo validos para fs = EMK_FS_HZ, igual que el resto de esta cabecera.
 */
#define EMK_VOL_N_BIQUADS 2
static const float EMK_VOL_SOS[2][5] = {
    { 7.820208033e-03f, 1.564041607e-02f, 7.820208033e-03f, -1.765303228e+00f, 8.375337583e-01f },
    { 1.000000000e+00f, -2.000000000e+00f, 1.000000000e+00f, -1.895726029e+00f, 9.145978814e-01f },
};

#define EMK_B110_N_BIQUADS 2
static const float EMK_B110_SOS[2][5] = {
    { 1.750876437e-01f, 3.501752873e-01f, 1.750876437e-01f, -4.733854564e-01f, 2.607010831e-01f },
    { 1.000000000e+00f, -2.000000000e+00f, 1.000000000e+00f, -1.825669900e+00f, 8.425510982e-01f },
};

#define EMK_UMBRAL_VOL110_PCT 30.0f   /* vol110 >= esto -> energia dominada por
                                       * movimiento voluntario de baja frecuencia */

/*
 * RUTA2: temblor de gran amplitud concentrado en UN SOLO eje de giro.
 * Complementa a la ruta por votacion (ruta1), que exige 2 ejes activos
 * cuando hay mas de uno por encima del piso de ruido (EMK_VOTOS_MIN): un
 * temblor muy fuerte en un solo eje, con los otros dos casi quietos, puede
 * fallar esa votacion. RUTA2 mide directamente que tan "unidireccional" es
 * el giro en banda con la covarianza 3x3 de bp[0..2] (giro en banda 2.5-8 Hz,
 * los 3 ejes): si el mayor autovalor domina la traza (lin cerca de 1), el
 * movimiento es casi un vector fijo que oscila en una sola direccion -> eso
 * es exactamente un temblor de un solo eje, no ruido ni vibracion repartida
 * en varios ejes. Ver tremor.c (bloque RUTA2) para el detalle del calculo
 * (autovalor mayor por iteracion de potencia).
 */
#define EMK_R2_LIN_MIN        0.95f   /* autovalor mayor / traza de la covarianza 3x3 */
#define EMK_R2_PREL_MIN_PCT   65.0f   /* potencia relativa en banda, 3 ejes juntos, % */
#define EMK_R2_RMS_MIN_DPS    45.0f   /* RMS sobre el eje principal (autovector), dps */

#endif
