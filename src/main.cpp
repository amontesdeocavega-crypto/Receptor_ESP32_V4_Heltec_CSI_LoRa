#include <Arduino.h>
#include "freertos/FreeRTOS.h" // Cabecera para el RTOS
#include "freertos/task.h"     // Cabecera para vTaskDelay
#include "freertos/semphr.h"   // --- AÑADIDO: Cabecera para el Mutex ---
#include <RadioLib.h>

// --- LIBRERÍAS CSI (ESP-IDF) ---
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

// ==========================================
// CONFIGURACIÓN GLOBAL Y MUTEX
// ==========================================
SemaphoreHandle_t serialMutex; 

// ==========================================
// CONFIGURACIÓN LORA
// ==========================================
// Pines para el ESP32-S3 Heltec V4
SX1262 radio = new Module(8, 14, 12, 13);

// Flag para la interrupción
volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

// ==========================================
// CONFIGURACIÓN CSI
// ==========================================
// --- CONFIGURACIÓN ---
#define CONFIG_LESS_INTERFERENCE_CHANNEL   11
#define CONFIG_WIFI_BANDWIDTH              WIFI_BW_HT20

// ESCALA VISUAL: Imprime 1 de cada 50 paquetes para no saturar la pantalla.
// Ponerlo a 1 cuando vayas a conectar la Raspberry para extraer todos los datos.
#define CONFIG_PRINT_SCALE_FACTOR         25  

// MAC Base: Aceptará 1A:00:00:00:00:01, 02, 03 y 04
static const uint8_t TX_MAC_PREFIX[] = {0x1a, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";

static void wifi_init()
{
    // 1. Crea el "Sistema Nervioso" del ESP32. 
    // Permite que el chip pueda gestionar eventos en segundo plano (como avisar cuando llega un paquete o cuando el Wi-Fi se desconecta).
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 2. Inicializa la "pila de red" (LwIP).
    // Prepara las estructuras de memoria para manejar redes. 
    ESP_ERROR_CHECK(esp_netif_init());

    // WIFI_INIT_CONFIG_DEFAULT() es una macro que rellena una estructura con 
    // decenas de parámetros ocultos optimizados por los ingenieros de Espressif.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    // 4. Enciende el motor de la antena.
    // Le pasa el "molde" del paso anterior al hardware físico. A partir de esta línea,
    // el módulo de radiofrecuencia (el hardware) recibe energía y despierta.
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. Define el "Rol" del dispositivo (Station).
    // WIFI_MODE_STA significa que actuará como un "cliente" 
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // 6. Protege la memoria del chip (Guarda en RAM).
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Configuración de ancho de banda y arranque
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // IMPORTANTE: Desactivar ahorro de energía para no perder paquetes
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); 
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// CALLBACK DE RECEPCIÓN CSI 
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;

    // 1. Filtrar por la MAC
    if (memcmp(info->mac, TX_MAC_PREFIX, 5) != 0) {
        return; 
    }

    // Extraer ID del transmisor (Último byte: 01, 02, 03 o 04)
    uint8_t tx_id = info->mac[5];

    static int s_count = 0;

    // 2. Filtro visual (Imprime 1 de cada 50 paquetes)
    if (s_count % CONFIG_PRINT_SCALE_FACTOR  == 0) {

        // --- PROTECCIÓN MUTEX PARA EL CSI ---
        if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
            // Cabecera CSV la primera vez
            if (s_count == 0) {
                Serial.println("================ CSI RECV INICIADO ================");
                // Cabecera: Tipo, Tx_ID, RSSI, Ruido, Timestap y matriz CSI
                Serial.println("TYPE,Tx_ID,RSSI,NOISE,TIMESTAMP,DATA");
            }
            // 1. Imprimimos la cabecera directamente
            Serial.printf("\nCSI,TX_%d, RSSI: %d, Ruido: %d, Timestamp: %u, Matriz CSI [" , 
                          tx_id, info->rx_ctrl.rssi, info->rx_ctrl.noise_floor, info->rx_ctrl.timestamp);

            // 2. Imprimimos los números directamente
            int8_t *csi_data = (int8_t *)info->buf;
            for (int i = 0; i < info->len; i++) {
                Serial.print(csi_data[i]);
                if (i < info->len - 1) Serial.print(",");
            }

            // 3. Impresión de la matriz cargada anteriormente con los datos
            Serial.println("]");
            xSemaphoreGive(serialMutex);
        } 
    }
    s_count++;
}

static void wifi_csi_init()
{
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

// ==========================================
// TAREA CSI (ASIGNADA AL CORE 0)
// ==========================================
void CsiTask(void *pvParameters)
{
    // 1. Inicializar Memoria NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Iniciar Wi-Fi y ESP-NOW
    wifi_init();

    esp_now_peer_info_t peer = {
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, // Broadcast ESP-NOW
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
    };
    wifi_esp_now_init(peer);

    // 3. Iniciar Captura CSI
    wifi_csi_init();

    // Pausa para estabilizar USB antes de imprimir por consola
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        Serial.printf("[CSI_RX] Receptor Listo. Escuchando en Canal %d...\n", CONFIG_LESS_INTERFERENCE_CHANNEL);
        xSemaphoreGive(serialMutex);
    }

    // Bucle principal: Solo mantiene el sistema vivo sin gastar recursos extra
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==========================================
// TAREA LORA (ASIGNADA AL CORE 1)
// ==========================================
void LoRaTask(void *pvParameters) {
  for (;;) {
    // Si la antena atrapó algo en el aire...
    if(receivedFlag) {
      // 1. Reseteamos el flag para poder recibir el siguiente
      receivedFlag = false;

      // 2. Extraemos el texto del paquete
      String str;
      int state = radio.readData(str);

      if (state == RADIOLIB_ERR_NONE) {
        
        // --- 3. EL FILTRO DE IDs ---
        // Comprobamos si el mensaje empieza por alguno de nuestros 4 IDs válidos
        if (str.startsWith("TX_1:") || str.startsWith("TX_2:") || 
            str.startsWith("TX_3:") || str.startsWith("TX_4:")) {
          
          // Es nuestro transmisor. Extraemos la "firma física" (RSSI y SNR).
          float rssi = radio.getRSSI();
          float snr = radio.getSNR();
          String id_tx = str.substring(0, str.indexOf(':')); // Extraemos el ID del transmisor 
          String timestamp = str.substring(str.indexOf(':') + 1, str.indexOf(':', str.indexOf(':')  + 1)); //Extraemos el timestamp

          // --- PROTECCIÓN MUTEX PARA LORA VÁLIDO ---
          if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
              Serial.print(F("[LoRa_RX] VÁLIDO -> "));
              Serial.print(F("ID_Tx: "));
              Serial.print(id_tx);
              Serial.print(F(" | Timestamp: "));
              Serial.print(timestamp);
              Serial.print(F(" | RSSI: "));
              Serial.print(rssi);
              Serial.print(F(" dBm | SNR: "));
              Serial.print(snr);
              Serial.println(F(" dB"));
              xSemaphoreGive(serialMutex);
          }
          
        } else {
          // La antena recibió un paquete LoRa ajeno
          if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
              Serial.print(F("[LoRa_RX] IGNORADO (ID Desconocido): "));
              Serial.println(str);
              xSemaphoreGive(serialMutex);
          }
        }

      } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
            Serial.println(F("[LoRa_RX] Error de CRC (Paquete corrupto)"));
            xSemaphoreGive(serialMutex);
        }
      }
      // Obligamos al chip a volver a escuchar el aire
      radio.startReceive();
    }
    // Ceder 1 milisegundo al RTOS en cada vuelta del loop (Vital para el rendimiento)
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ==========================================
// SETUP PRINCIPAL
// ==========================================
void setup() {
  
  // Le damos un buffer grande al USB para que cargue la matriz entera de golpe
  Serial.setTxBufferSize(4096);
  // Velocidad para volcar datos a la Raspberry Pi sin cuellos de botella
  Serial.begin(921600);
  
  // --- INICIALIZAR EL MUTEX ---
  serialMutex = xSemaphoreCreateMutex();

  // Obligamos al procesador a esperar infinitamente hasta que Windows abra el puerto
  vTaskDelay(pdMS_TO_TICKS(2000)); 

  // Da electricidad al interruptor de la antena RF
  pinMode(36, OUTPUT);
  digitalWrite(36, LOW);

  // Enciende el amplificador de la antena
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  
  if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
      Serial.println(F("\n\n--- TERMINAL CONECTADA. ARRANCANDO SISTEMA ---"));
      xSemaphoreGive(serialMutex);
  }

  // Pausa para que la terminal se estabilice
  vTaskDelay(pdMS_TO_TICKS(2000)); 

  if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
      Serial.print(F("[SX1262] Inicializando Receptor ... "));
      xSemaphoreGive(serialMutex);
  }
  
  // MISMOS PARÁMETROS FÍSICOS QUE EL TRANSMISOR
  // (Frecuencia 868.0, BW 125, SF 7, CR 4/7, SyncWord Privada)
  // Nota: Quitamos el parámetro de potencia (14) porque un receptor no emite
  int state = radio.begin(868.0, 125.0, 7, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE);
  
  if (state == RADIOLIB_ERR_NONE) {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        Serial.println(F("¡Éxito!"));
        xSemaphoreGive(serialMutex);
    }
    radio.setDio2AsRfSwitch(true); // enciende un switch de la placa que permite alimentar la antena
    radio.setTCXO(1.8); 
  } else {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        Serial.printf("Fallo, código %d\n", state);
        xSemaphoreGive(serialMutex);
    }
    // Bucle infinito  
    while (true) { vTaskDelay(pdMS_TO_TICKS(10)); } 
  }

  // Enganchar la interrupción
  radio.setPacketReceivedAction(setFlag);

  // Empezar a escuchar continuamente
  if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
      Serial.print(F("[SX1262] Escuchando el aire ... "));
      xSemaphoreGive(serialMutex);
  }
  
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        Serial.println(F("¡Éxito!"));
        xSemaphoreGive(serialMutex);
    }
  } else {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        Serial.printf("Fallo al escuchar, código %d\n", state);
        xSemaphoreGive(serialMutex);
    }
    while (true) { vTaskDelay(pdMS_TO_TICKS(10)); }
  }

  // --- ANCLANDO LA TAREA LORA AL CORE 1 ---
  xTaskCreatePinnedToCore(
    LoRaTask,       // Función que contiene el código
    "LoRa_RX_Task", // Nombre descriptivo
    8192,           // Memoria RAM asignada
    NULL,           // Parámetros
    1,              // Prioridad
    NULL,           // Puntero de control
    1               // NÚCLEO ASIGNADO: 1 (Core 1)
  );

  // --- ANCLANDO LA TAREA CSI AL CORE 0 ---
  xTaskCreatePinnedToCore(
    CsiTask,        // Función que contiene el código CSI
    "CSI_RX_Task",  // Nombre descriptivo
    16384,          // Memoria RAM asignada (Asignamos más RAM para las matrices)
    NULL,           // Parámetros
    2,              // Prioridad (Un punto más alta por el altísimo tráfico de red)
    NULL,           // Puntero de control
    0               // NÚCLEO ASIGNADO: 0 (Core 0)
  );
}

void loop() {
  // Eliminamos el loop() de Arduino de la memoria.
  vTaskDelete(NULL);
}