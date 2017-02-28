#ifndef __ESP_HEADER__
#define __ESP_HEADER__

#include <stdint.h>

void esp_init();

void esp_task();

void esp_send_cmd(char data[]);
void esp_write(char data[], uint16_t size);

void esp_rst();

// wifi layer
typedef enum
{
    ESP_MODE_STA = 0,
    ESP_MODE_AP,
    ESP_MODE_STA_AP
} ESP_MODE;
void esp_set_mode(ESP_MODE mode);
uint8_t esp_connect_ap(char *ssid, char *pw);       // warning, be careful to special car, protect with backslash
uint8_t esp_disconnect_ap();

// tcp/ip layer
uint8_t esp_open_tcp_socket(char *ip_domain, uint16_t port);
uint8_t esp_open_udp_socket(char *ip_domain, uint16_t port, uint16_t localPort, uint8_t mode);
void esp_close_socket(uint8_t sock);
void esp_write_socket(uint8_t sock, char *data, uint16_t size);
void esp_write_socket_string(uint8_t sock, char *str);

void esp_server_create(uint16_t port);
void esp_server_destroy();

typedef enum
{
    WIFI_STATE_NONE = 0,
    WIFI_STATE_READY,
    WIFI_STATE_OK,
    WIFI_STATE_ERROR,
    WIFI_STATE_RECEIVE_DATA,
    WIFI_STATE_SEND_DATA,
    WIFI_STATE_SEND_OK

} WIFI_STATE;

void wait_ok();
uint8_t getRec();
WIFI_STATE get_state();
uint8_t getRecSocket();

char *getRecData();
uint16_t getRecSize();

#endif   //__ESP_HEADER__