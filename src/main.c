#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#include "hardware/irq.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "lwip/udp.h"
#include "lwip/sockets.h"

// GPS & PPS
#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define PPS_PIN 2

// NTP
#define NTP_PORT 123
#define NTP_MSG_LEN 48
#define JAN_1970 0x83aa7e80UL

// Flash
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define WIFI_CONFIG_MAGIC 0x57494649

typedef struct {
    uint32_t magic;
    char ssid[33];
    char password[64];
    uint32_t auth_mode;
} wifi_config_t;

// NTP Sync State
typedef struct {
    uint64_t last_pps_pico_us;
    int32_t pps_pico_us_diff;
    uint32_t utc_seconds;
    struct tm utc_tm;
    int fix_quality;
    int num_satellites;
    char raw_nmea[4][128];
    int sentence_count;
    bool ntp_active;
} ntp_sync_t;

static ntp_sync_t sync_data = {0, 0, 0, {0, 0, 0, 0}, 0, 0, {"", "", "", ""}, 0, false};
static SemaphoreHandle_t sync_mutex;
static TaskHandle_t led_task_handle;
static QueueHandle_t uart_rx_queue;
#define EVENT_PPS 0x01
#define EVENT_NMEA 0x02
#define EVENT_NTP_RECV 0x04

// UART RX Interrupt Handler
void on_uart_rx() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        xQueueSendFromISR(uart_rx_queue, &ch, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// PPS Handler
void pps_callback(uint gpio, uint32_t events) {
    uint64_t now = time_us_64();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(led_task_handle, EVENT_PPS, eSetBits, &xHigherPriorityTaskWoken);
    if (xSemaphoreTakeFromISR(sync_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        sync_data.pps_pico_us_diff = (int32_t)(now - sync_data.last_pps_pico_us);
        sync_data.last_pps_pico_us = now;
        sync_data.utc_seconds++;
        xSemaphoreGiveFromISR(sync_mutex, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// NTP Raw UDP Receive Callback
static void ntp_recv_raw(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        if (p->tot_len >= NTP_MSG_LEN) {
            uint8_t *req = (uint8_t *)p->payload;
            xTaskNotify(led_task_handle, EVENT_NTP_RECV, eSetBits); // LEDを点灯させる
            
            xSemaphoreTake(sync_mutex, portMAX_DELAY);
            int fix = sync_data.fix_quality;
            uint32_t sec = sync_data.utc_seconds;
            uint64_t pps_us = sync_data.last_pps_pico_us;
            int32_t pps_us_diff = sync_data.pps_pico_us_diff;
            xSemaphoreGive(sync_mutex);
            if (fix == 0) {
                datetime_t t;
                if (rtc_get_datetime(&t)) {
                    time_t temp_time;
                    datetime_to_time(&t, &temp_time);
                    sec = (uint32_t)temp_time;
                } else {
                    sec = 0; // 壊れてない限りこれはないはずなのだけど
                } 
            }

            struct pbuf *rp = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
            if (rp != NULL) {
                uint8_t *resp = (uint8_t *)rp->payload;
                memset(resp, 0, NTP_MSG_LEN);

                uint64_t now_us = time_us_64();
                uint64_t diff_us = (fix > 0) ? (now_us - pps_us) : (now_us - pps_us) % 1000000; // GPSが固定されていない場合はPPSのタイミングを考慮する
                
                // NTP Header
                // LI=0 (no warning), VN=from client, Mode=4 (server)
                uint8_t vn = (req[0] >> 3) & 7;
                resp[0] = (vn << 3) | 4; // vnはバージョン番号 閏秒は無視している
                resp[1] = (fix > 0) ? 1 : 16;    // Stratum 1
                resp[2] = 4;    // Poll
                resp[3] = -18;  // Precision
                
                // Root Delay & Dispersion (Fixed values for Stratum 1)
                // Set to 0.0001s (approx)
                resp[9] = 1;    // Root Delay
                resp[13] = 1;   // Root Dispersion

                // Reference Identifier
                memcpy(&resp[12], "GPS ", 4);

                uint32_t ntp_sec = sec + JAN_1970;
                uint32_t ntp_frac = (uint32_t)((diff_us * 4294967296ULL) / 1000000ULL);
                uint32_t n_sec = htonl(ntp_sec);
                uint32_t n_frac = htonl(ntp_frac);

                // Reference Timestamp (Last PPS time)
                memcpy(&resp[16], &n_sec, 4);
                resp[20] = 0; resp[21] = 0; resp[22] = 0; resp[23] = 0;

                // Originate Timestamp (from Client Transmit Timestamp)
                memcpy(&resp[24], &req[40], 8);

                // Receive & Transmit Timestamp
                memcpy(&resp[32], &n_sec, 4);
                memcpy(&resp[36], &n_frac, 4);
                memcpy(&resp[40], &n_sec, 4);
                memcpy(&resp[44], &n_frac, 4);

                udp_sendto(pcb, rp, addr, port);
                pbuf_free(rp);
            }
        }
        pbuf_free(p);
    }
}

void start_ntp_server_raw() {
    cyw43_arch_lwip_begin();
    struct udp_pcb *pcb = udp_new();
    if (pcb != NULL) {
        err_t err = udp_bind(pcb, IP_ADDR_ANY, NTP_PORT);
        if (err == ERR_OK) {
            udp_recv(pcb, ntp_recv_raw, NULL);
            sync_data.ntp_active = true;
            printf("NTP: Server Active\n");
        } else {
            udp_remove(pcb);
        }
    }
    cyw43_arch_lwip_end();
}

// Telemetry Web Server (Socket API)
static char http_response[3500];
const char *HTTP_BODY_TPL = 
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta http-equiv='refresh' content='2'>"
    "<style>body{font-family:sans-serif;margin:20px;background:#eee;} .box{background:white;padding:15px;margin-bottom:10px;border-radius:5px;}</style>"
    "<title>Pico WH NTP Debug</title></head><body>"
    "<h1>Pico WH NTP Server</h1>"
    "<div class='box'><b>NTP Server:</b> %s</div>"
    "<div class='box'><b>GPS Fix:</b> %d | Sats: %d | Total NMEA: %d</div>"
    "<div class='box'><b>Current Pico Time:</b> %llu | diff = %d</div>"
    "<div class='box'><b>Last GPS Time:</b> %s</div>"
    "<div class='box'><b>Network:</b> %s | %s</div>"
    "<div class='box'><pre style='font-size:10px;'>%s\n%s\n%s\n%s</pre></div>"
    "</body></html>";

void http_server_task(void *pvParameters) {
    char ssid_copy[33]; strncpy(ssid_copy, (char*)pvParameters, 32); ssid_copy[32] = '\0'; // SSIDをコピー
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP); // ソケットを作成
    struct sockaddr_in addr; memset(&addr, 0, sizeof(addr)); // アドレスを初期化
    addr.sin_family = AF_INET; addr.sin_port = htons(80); addr.sin_addr.s_addr = htonl(INADDR_ANY); // アドレスを設定
    bind(server_sock, (struct sockaddr *)&addr, sizeof(addr)); listen(server_sock, 4); // バインドしてリッスン
    while (true) {
        struct sockaddr_in client_addr; socklen_t client_addr_len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_addr_len); // クライアントからの接続を受け入れる
        if (client_sock >= 0) {
            char req[512]; recv(client_sock, req, sizeof(req), 0); // クライアントからのリクエストを受信
            char last_time_str[64]; // 最終取得のGPS時刻を文字列にして格納するバッファ

            xSemaphoreTake(sync_mutex, portMAX_DELAY);
            strftime(last_time_str, sizeof(last_time_str), "%Y-%m-%d %H:%M:%S UTC", &sync_data.utc_tm);                
            int body_len = snprintf(http_response + 512, 2500, HTTP_BODY_TPL, 
                     (sync_data.ntp_active ? (sync_data.fix_quality > 0 ? "Stratum 1 Active" : "Waiting for GPS Fix") : "NTP Init Failed"),
                     sync_data.fix_quality, sync_data.num_satellites, sync_data.sentence_count,
                     sync_data.last_pps_pico_us, sync_data.pps_pico_us_diff, last_time_str,
                     ssid_copy, ip4addr_ntoa(netif_ip4_addr(netif_default)),
                     sync_data.raw_nmea[(sync_data.sentence_count - 1) % 4], sync_data.raw_nmea[(sync_data.sentence_count - 2) % 4], 
                     sync_data.raw_nmea[(sync_data.sentence_count - 3) % 4], sync_data.raw_nmea[(sync_data.sentence_count) % 4]);
            int head_len = snprintf(http_response, 512, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nConnection: close\r\n\r\n", body_len);
            memmove(http_response + head_len, http_response + 512, body_len); // HTTPヘッダーとボディを結合
            send(client_sock, http_response, head_len + body_len, 0); // クライアントに送信
            xSemaphoreGive(sync_mutex);
            vTaskDelay(pdMS_TO_TICKS(20)); close(client_sock);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void parse_nmea_sentence(char *sentence) {
    xSemaphoreTake(sync_mutex, portMAX_DELAY);
    strncpy(sync_data.raw_nmea[sync_data.sentence_count % 4], sentence, 127); // 0-3のいずれかに書き込まれる
    sync_data.sentence_count++;
    xSemaphoreGive(sync_mutex);
    // $GxRMCのフォーマットは $GPRMC,hhmmss.sss,A,ddmm.mmmm,N,dddmm.mmmm,E,v,ss,ddmmyy,ddd.d,E,A*hh
    //                        00000000001111111111222222222233333333334444444444555555555566666666
    //                        01234567890123456789001234567890012345678900123456789001234567890012
    if ((strncmp(sentence, "$GPRMC", 6) == 0) || (strncmp(sentence, "$GNRMC", 6) == 0)) {
        int valid = 0; int field = 0;
        char *time_str_p = NULL; char *date_str_p = NULL; char *p = sentence;
        printf("%s\n", sentence); // debug
        while ((p = strchr(p, ',')) != NULL) {
            field++;
            p++;
            if (field == 1) {
                time_str_p = p;         // hhmmss.sss
            }
            if (field == 2) {
                valid = (p[0] == 'A');  // Aなら有効 Vなら無効
            }
            if (field == 9) {
                date_str_p = p;         // ddmmyy
            }
        }
        if (time_str_p != NULL && date_str_p != NULL) {
            struct tm t; 
            memset(&t, 0, sizeof(t));  char b[3] = {0};
            xSemaphoreTake(sync_mutex, portMAX_DELAY);
            b[0] = date_str_p[0]; b[1] = date_str_p[1]; t.tm_mday = atoi(b);
            b[0] = date_str_p[2]; b[1] = date_str_p[3]; t.tm_mon = atoi(b) - 1;
            b[0] = date_str_p[4]; b[1] = date_str_p[5]; t.tm_year = atoi(b) + 100;
            b[0] = time_str_p[0]; b[1] = time_str_p[1]; t.tm_hour = atoi(b);
            b[0] = time_str_p[2]; b[1] = time_str_p[3]; t.tm_min = atoi(b);
            b[0] = time_str_p[4]; b[1] = time_str_p[5]; t.tm_sec = atoi(b);
            sync_data.utc_seconds = (uint32_t)mktime(&t);
            sync_data.utc_tm = t;
            xSemaphoreGive(sync_mutex);
        }
    }
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char tmp[128]; strncpy(tmp, sentence, 127); char *p = tmp; char *token; int field = 0;
        xSemaphoreTake(sync_mutex, portMAX_DELAY);
        while ((token = strsep(&p, ",")) != NULL) {
            if (field == 6) sync_data.fix_quality = atoi(token);
            if (field == 7) sync_data.num_satellites = atoi(token);
            field++;
        }
        xSemaphoreGive(sync_mutex);
    }
}

void gps_task(void *pvParameters) {
    static char line_buf[128]; int pos = 0;
    while (true) {
        uint8_t c;
        if (xQueueReceive(uart_rx_queue, &c, portMAX_DELAY) == pdTRUE) {
            if (c == '$') pos = 0;
            if (pos < sizeof(line_buf) - 1) {
                line_buf[pos++] = c;
                if (c == '\n' || c == '\r') {
                    line_buf[pos] = '\0';
                    if (pos > 5) {
                        parse_nmea_sentence(line_buf);
                        xTaskNotify(led_task_handle, EVENT_NMEA, eSetBits);
                    }
                    pos = 0;
                }
            }
        }
    }
}

void save_wifi_config(const char *ssid, const char *password, uint32_t auth) {
    wifi_config_t config; memset(&config, 0, sizeof(config));
    config.magic = WIFI_CONFIG_MAGIC; strncpy(config.ssid, ssid, 32); strncpy(config.password, password, 63); config.auth_mode = auth;
    uint8_t buffer[FLASH_PAGE_SIZE]; memset(buffer, 0, sizeof(buffer)); memcpy(buffer, &config, sizeof(config));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE); flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}
bool load_wifi_config(wifi_config_t *config) {
    const wifi_config_t *saved = (const wifi_config_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    if (saved->magic == WIFI_CONFIG_MAGIC) { memcpy(config, saved, sizeof(wifi_config_t)); return true; }
    return false;
}
void get_line_with_timeout(char *buffer, int max_len, uint32_t timeout_ms) {
    int i = 0; absolute_time_t end_time = make_timeout_time_ms(timeout_ms);
    while (i < max_len - 1) {
        if (time_reached(end_time)) { buffer[0] = '\0'; return; }
        int c = getchar_timeout_us(100000); if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\r' || c == '\n') { if (i == 0) continue; buffer[i] = '\0'; printf("\r\n"); break; }
        else if (c == 8 || c == 127) { if (i > 0) { i--; printf("\b \b"); } }
        else if (c >= 32 && c <= 126) { buffer[i++] = c; putchar(c); end_time = at_the_end_of_time; }
    }
}

void wifi_task(void *pvParameters) {
    if (cyw43_arch_init()) { vTaskDelete(NULL); return; }
    cyw43_arch_enable_sta_mode();
    static wifi_config_t saved_config; bool has_saved = load_wifi_config(&saved_config);
    while (true) {
        printf("\nSSID (wait 10s): ");
        char target_ssid[33] = {0}; char target_pass[64] = {0}; uint32_t target_auth = CYW43_AUTH_WPA2_MIXED_PSK;
        get_line_with_timeout(target_ssid, sizeof(target_ssid), 10000);
        if (target_ssid[0] == '\0' && has_saved) { strncpy(target_ssid, saved_config.ssid, 32); strncpy(target_pass, saved_config.password, 63); target_auth = saved_config.auth_mode; }
        else if (target_ssid[0] != '\0') { printf("Pass: "); get_line_with_timeout(target_pass, sizeof(target_pass), 60000); }
        else continue;
        printf("Connecting to %s...\n", target_ssid);
        if (!cyw43_arch_wifi_connect_timeout_ms(target_ssid, target_pass, target_auth, 30000)) {
            while (ip4_addr_isany_val(*netif_ip4_addr(netif_default))) vTaskDelay(pdMS_TO_TICKS(100));
            save_wifi_config(target_ssid, target_pass, target_auth);
            static char active_ssid[33]; strncpy(active_ssid, target_ssid, 32);
            xTaskCreate(http_server_task, "HTTP", 4096, (void*)active_ssid, 2, NULL);
            start_ntp_server_raw();
            break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    datetime_t t;
    xSemaphoreTake(sync_mutex, portMAX_DELAY);
    t.year  = sync_data.utc_tm.tm_year + 1900;
    t.month = sync_data.utc_tm.tm_mon + 1;
    t.day   = sync_data.utc_tm.tm_mday;
    t.dotw  = sync_data.utc_tm.tm_wday; // 0=Sunday
    t.hour  = sync_data.utc_tm.tm_hour;
    t.min   = sync_data.utc_tm.tm_min;
    t.sec   = sync_data.utc_tm.tm_sec;
    rtc_set_datetime(&t); // この頃までには1度くらいGPSで時刻が取得できていると思う
    xSemaphoreGive(sync_mutex);
    vTaskDelete(NULL);
}

void led_task(void *pvParameters) {
    uint32_t notifiedValue;
    while (true) { 
        if (xTaskNotifyWait(0, ULONG_MAX, &notifiedValue, portMAX_DELAY) == pdTRUE) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); 
            if (notifiedValue & EVENT_PPS) {
                vTaskDelay(pdMS_TO_TICKS(50)); 
            } else if (notifiedValue & EVENT_NTP_RECV) {
                vTaskDelay(pdMS_TO_TICKS(100)); 
            } else if (notifiedValue & EVENT_NMEA) {
                vTaskDelay(pdMS_TO_TICKS(20)); 
            }
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); 
        }
    }
}

int main() {
    stdio_init_all(); 
    uart_init(UART_ID, BAUD_RATE); gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Setup UART RX Interrupt
    uart_rx_queue = xQueueCreate(1024, sizeof(uint8_t));
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    rtc_init(); // RTCを初期化

    gpio_init(PPS_PIN); gpio_set_dir(PPS_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PPS_PIN, GPIO_IRQ_EDGE_RISE, true, &pps_callback);
    sync_mutex = xSemaphoreCreateMutex();
    xTaskCreate(led_task, "LED", 256, NULL, 1, &led_task_handle);
    xTaskCreate(wifi_task, "Wi-Fi", 4096, NULL, 2, NULL);
    xTaskCreate(gps_task, "GPS", 2048, NULL, 2, NULL);
    vTaskStartScheduler();
    while (true); // ここには来ないはず
}
