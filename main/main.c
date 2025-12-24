#include "freertos/FreeRTOS.h" // Núcleo de FreeRTOS
#include "freertos/task.h" // Tareas y delays
#include "driver/adc.h" // Manejo de ADC (MAX4466)
#include "driver/dac.h" // Manejo de DAC (LM386)
#include "driver/gpio.h" // Control de GPIOs (botón)
#include "rom/ets_sys.h" // Funciones de bajo nivel

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← MAX4466: G36 (ADC1_CH0), 3V3, GND
//                   ← LM386: G25 (DAC1), 5V, GND ← Bocina (3W 4Ω): 5V, GND
//                   ← RFM98: G5 (CS), G18 (SCK), G19 (MISO), G23 (MOSI), G26 (RST), 3V3, GND
//                   ← Micro switch (2 pin): G34 (entrada), GND

// PINES DE LA PLACA
#define PIN_MIC_ADC ADC1_CHANNEL_0 // G36 entrada analógica (MAX4466 OUT)
#define PIN_SPEAKER_DAC DAC_CHANNEL_1 // G25 salida analógica (LM386 IN)
#define PIN_BOTON_TRANSMITIR 34 // G34 para botón de transmisión
#define PIN_RFM98_CS 5 // G5 para selección de chip RFM98
#define PIN_RFM98_RST 26 // G26 para reset RFM98

// PARÁMETROS DE AUDIO Y RADIO
#define FRECUENCIA_RADIO 915000000 // Frecuencia de operación en Hz (915 MHz)
#define TAMANO_PAQUETE_AUDIO 64 // Tamaño de paquete de audio en bytes
#define CLAVE_CIFRADO 0xA5 // Clave simple para cifrado XOR
#define RETARDO_MUESTREO_US 125 // Retardo para 8kHz (1,000,000 / 8000 = 125μs)
#define UMBRAL_SILENCIO 50 // Umbral para detección de silencio

// CONSTANTES DEL PROGRAMA
#define ESTADO_RECEPCION 0 // Modo recepción
#define ESTADO_TRANSMISION 1 // Modo transmisión

// VARIABLES GLOBALES
volatile uint8_t estado_actual = ESTADO_RECEPCION; // Estado actual del dispositivo
volatile uint32_t tiempo_ultimo_boton = 0; // Tiempo del último cambio de botón

// FUNCIONES DE CIFRADO/DESCIFRADO
// Cifra datos usando XOR con clave y desplazamiento
void cifrar_datos(uint8_t* datos_original, uint8_t* datos_cifrados, uint16_t longitud)
{
    for (uint16_t i = 0; i < longitud; i++)
    {
        uint8_t clave_modificada = CLAVE_CIFRADO ^ (i & 0xFF);
        datos_cifrados[i] = datos_original[i] ^ clave_modificada;
        datos_cifrados[i] = (datos_cifrados[i] << 1) | (datos_cifrados[i] >> 7);
    }
}

// Descifra datos usando XOR con clave y desplazamiento inverso
void descifrar_datos(uint8_t* datos_cifrados, uint8_t* datos_descifrados, uint16_t longitud)
{
    for (uint16_t i = 0; i < longitud; i++)
    {
        uint8_t dato_restaurado = (datos_cifrados[i] >> 1) | (datos_cifrados[i] << 7);
        uint8_t clave_modificada = CLAVE_CIFRADO ^ (i & 0xFF);
        datos_descifrados[i] = dato_restaurado ^ clave_modificada;
    }
}

// FUNCIONES DE RADIO RFM98
// Escribe un registro del RFM98
void escribir_registro_rfm98(uint8_t direccion, uint8_t valor)
{
    uint8_t datos_escritura[2] = {direccion | 0x80, valor};
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.flags = 0;
    transaccion_spi.length = 16;
    transaccion_spi.tx_buffer = datos_escritura;
    
    gpio_set_level((gpio_num_t)PIN_RFM98_CS, 0);
    spi_device_polling_transmit(dispositivo_spi_rfm98, &transaccion_spi);
    gpio_set_level((gpio_num_t)PIN_RFM98_CS, 1);
}

// Lee un registro del RFM98
uint8_t leer_registro_rfm98(uint8_t direccion)
{
    uint8_t datos_lectura[2] = {direccion & 0x7F, 0x00};
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.flags = 0;
    transaccion_spi.length = 16;
    transaccion_spi.tx_buffer = datos_lectura;
    transaccion_spi.rx_buffer = datos_lectura;
    
    gpio_set_level((gpio_num_t)PIN_RFM98_CS, 0);
    spi_device_polling_transmit(dispositivo_spi_rfm98, &transaccion_spi);
    gpio_set_level((gpio_num_t)PIN_RFM98_CS, 1);
    
    return datos_lectura[1];
}

// Configura el módulo RFM98 para LoRa
void configurar_modulo_rfm98()
{
    // Resetear módulo RFM98
    gpio_set_level((gpio_num_t)PIN_RFM98_RST, 0);
    ets_delay_us(10000);
    gpio_set_level((gpio_num_t)PIN_RFM98_RST, 1);
    ets_delay_us(10000);
    
    // Entrar en modo sleep
    escribir_registro_rfm98(0x01, 0x00);
    
    // Entrar en modo LoRa
    escribir_registro_rfm98(0x01, 0x80);
    
    // Configurar frecuencia (915 MHz)
    uint64_t valor_frecuencia = ((uint64_t)FRECUENCIA_RADIO << 19) / 32000000;
    
    escribir_registro_rfm98(0x06, (valor_frecuencia >> 16) & 0xFF);
    escribir_registro_rfm98(0x07, (valor_frecuencia >> 8) & 0xFF);
    escribir_registro_rfm98(0x08, valor_frecuencia & 0xFF);
    
    // Configurar potencia de transmisión
    escribir_registro_rfm98(0x09, 0xFF);
    escribir_registro_rfm98(0x4D, 17);
    
    // Configurar parámetros LoRa
    escribir_registro_rfm98(0x1D, 0x72);
    escribir_registro_rfm98(0x1E, 0x74);
    
    // Configurar FIFO
    escribir_registro_rfm98(0x0E, 0x00);
    escribir_registro_rfm98(0x0F, 0x00);
    
    // Salir de sleep mode
    escribir_registro_rfm98(0x01, 0x01);
}

// Transmite un paquete de audio cifrado
void transmitir_paquete_audio(uint8_t* datos_audio, uint16_t longitud)
{
    uint8_t datos_cifrados[TAMANO_PAQUETE_AUDIO];
    cifrar_datos(datos_audio, datos_cifrados, longitud);
    
    escribir_registro_rfm98(0x01, 0x81);
    escribir_registro_rfm98(0x0D, 0x00);
    escribir_registro_rfm98(0x22, longitud);
    
    for (uint16_t i = 0; i < longitud; i++)
    {
        escribir_registro_rfm98(0x00, datos_cifrados[i]);
    }
    
    escribir_registro_rfm98(0x01, 0x83);
    
    while ((leer_registro_rfm98(0x12) & 0x08) == 0)
    {
        ets_delay_us(1000);
    }
    
    escribir_registro_rfm98(0x12, 0x08);
    escribir_registro_rfm98(0x01, 0x01);
}

// Recibe un paquete de audio cifrado
int recibir_paquete_audio(uint8_t* buffer_audio, uint16_t* longitud_recibida)
{
    if ((leer_registro_rfm98(0x12) & 0x40) == 0)
    {
        return 0;
    }
    
    *longitud_recibida = leer_registro_rfm98(0x13);
    escribir_registro_rfm98(0x0D, 0x00);
    
    for (uint16_t i = 0; i < *longitud_recibida; i++)
    {
        buffer_audio[i] = leer_registro_rfm98(0x00);
    }
    
    escribir_registro_rfm98(0x12, 0x40);
    escribir_registro_rfm98(0x01, 0x05);
    
    return 1;
}

// FUNCIONES DE AUDIO
// Tarea para captura y transmisión de audio
void tarea_captura_transmision_audio(void* parametro)
{
    uint8_t buffer_audio[TAMANO_PAQUETE_AUDIO];
    uint16_t indice_buffer = 0;
    uint32_t contador_silencio = 0;
    
    while (1)
    {
        int lectura_adc = adc1_get_raw(PIN_MIC_ADC);
        uint8_t muestra_audio = lectura_adc / 16;
        
        if (estado_actual == ESTADO_TRANSMISION)
        {
            buffer_audio[indice_buffer] = muestra_audio;
            indice_buffer++;
            
            if (muestra_audio < UMBRAL_SILENCIO)
            {
                contador_silencio++;
            }
            else
            {
                contador_silencio = 0;
            }
            
            if (indice_buffer >= TAMANO_PAQUETE_AUDIO || contador_silencio > 100)
            {
                if (indice_buffer > 0)
                {
                    transmitir_paquete_audio(buffer_audio, indice_buffer);
                    indice_buffer = 0;
                }
            }
        }
        else
        {
            indice_buffer = 0;
            contador_silencio = 0;
        }
        
        ets_delay_us(RETARDO_MUESTREO_US);
    }
}

// Tarea para recepción y reproducción de audio
void tarea_recepcion_reproduccion_audio(void* parametro)
{
    uint8_t buffer_cifrado[TAMANO_PAQUETE_AUDIO];
    uint8_t buffer_descifrado[TAMANO_PAQUETE_AUDIO];
    uint16_t longitud_recibida = 0;
    
    escribir_registro_rfm98(0x01, 0x05);
    
    while (1)
    {
        if (recibir_paquete_audio(buffer_cifrado, &longitud_recibida))
        {
            if (longitud_recibida > 0 && longitud_recibida <= TAMANO_PAQUETE_AUDIO)
            {
                descifrar_datos(buffer_cifrado, buffer_descifrado, longitud_recibida);
                
                for (uint16_t i = 0; i < longitud_recibida; i++)
                {
                    dac_output_voltage(PIN_SPEAKER_DAC, buffer_descifrado[i]);
                    ets_delay_us(RETARDO_MUESTREO_US);
                }
            }
        }
        
        ets_delay_us(1000);
    }
}

// Tarea para manejo del botón
void tarea_manejo_boton(void* parametro)
{
    uint8_t estado_boton_anterior = 1;
    
    while (1)
    {
        uint8_t estado_boton_actual = gpio_get_level((gpio_num_t)PIN_BOTON_TRANSMITIR);
        
        if (estado_boton_actual == 0 && estado_boton_anterior == 1)
        {
            estado_actual = ESTADO_TRANSMISION;
        }
        else if (estado_boton_actual == 1 && estado_boton_anterior == 0)
        {
            estado_actual = ESTADO_RECEPCION;
        }
        
        estado_boton_anterior = estado_boton_actual;
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// PUNTO DE PARTIDA
void app_main()
{
    // CONFIGURACIÓN DE ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PIN_MIC_ADC, ADC_ATTEN_DB_11);
    
    // CONFIGURACIÓN DE DAC
    dac_output_enable(PIN_SPEAKER_DAC);
    
    // CONFIGURACIÓN DE BOTÓN
    gpio_config_t configuracion_boton;
    configuracion_boton.pin_bit_mask = (1ULL << PIN_BOTON_TRANSMITIR);
    configuracion_boton.mode = GPIO_MODE_INPUT;
    configuracion_boton.pull_up_en = GPIO_PULLUP_ENABLE;
    configuracion_boton.pull_down_en = GPIO_PULLDOWN_DISABLE;
    configuracion_boton.intr_type = GPIO_INTR_DISABLE;
    
    gpio_config(&configuracion_boton);
    
    // CONFIGURACIÓN DE RFM98 PINES
    gpio_config_t configuracion_rfm98;
    configuracion_rfm98.pin_bit_mask = (1ULL << PIN_RFM98_CS) | (1ULL << PIN_RFM98_RST);
    configuracion_rfm98.mode = GPIO_MODE_OUTPUT;
    configuracion_rfm98.pull_up_en = GPIO_PULLUP_DISABLE;
    configuracion_rfm98.pull_down_en = GPIO_PULLDOWN_DISABLE;
    configuracion_rfm98.intr_type = GPIO_INTR_DISABLE;
    
    gpio_config(&configuracion_rfm98);
    
    gpio_set_level((gpio_num_t)PIN_RFM98_CS, 1);
    gpio_set_level((gpio_num_t)PIN_RFM98_RST, 1);
    
    // CONFIGURACIÓN SPI PARA RFM98
    spi_bus_config_t configuracion_bus_spi;
    configuracion_bus_spi.mosi_io_num = 23;
    configuracion_bus_spi.miso_io_num = 19;
    configuracion_bus_spi.sclk_io_num = 18;
    configuracion_bus_spi.quadwp_io_num = -1;
    configuracion_bus_spi.quadhd_io_num = -1;
    configuracion_bus_spi.max_transfer_sz = TAMANO_PAQUETE_AUDIO;
    
    spi_bus_initialize(SPI2_HOST, &configuracion_bus_spi, SPI_DMA_CH_AUTO);
    
    spi_device_interface_config_t configuracion_dispositivo_spi;
    configuracion_dispositivo_spi.clock_speed_hz = 1000000;
    configuracion_dispositivo_spi.mode = 0;
    configuracion_dispositivo_spi.spics_io_num = PIN_RFM98_CS;
    configuracion_dispositivo_spi.queue_size = 7;
    configuracion_dispositivo_spi.pre_cb = NULL;
    configuracion_dispositivo_spi.post_cb = NULL;
    
    spi_bus_add_device(SPI2_HOST, &configuracion_dispositivo_spi, &dispositivo_spi_rfm98);
    
    // CONFIGURAR MÓDULO RFM98
    configurar_modulo_rfm98();
    
    // CREACIÓN DE TAREAS
    xTaskCreate(tarea_captura_transmision_audio, "captura_tx", 4096, NULL, 5, NULL);
    xTaskCreate(tarea_recepcion_reproduccion_audio, "recepcion_rx", 4096, NULL, 5, NULL);
    xTaskCreate(tarea_manejo_boton, "boton", 2048, NULL, 3, NULL);
}
