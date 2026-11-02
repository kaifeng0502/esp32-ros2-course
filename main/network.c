// Local Wi-Fi transport. The pairing key is provisioned over USB, never broadcast.
#include "network.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/usb_serial_jtag.h"
#include "mbedtls/md.h"

#define UDP_PORT 43821
static uint8_t pairing_key[32];
static QueueHandle_t mailbox;
static int udp_socket = -1;
static esp_netif_t *station;

static void serial_reply(const char *message)
{
    usb_serial_jtag_write_bytes(message, strlen(message), pdMS_TO_TICKS(100));
}

static bool signature(const uint8_t *data, size_t len, uint8_t output[32])
{
    return mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                          pairing_key, sizeof(pairing_key), data, len, output) == 0;
}

static bool verified(const uint8_t *data, size_t len)
{
    if (len < 32) return false;
    uint8_t expected[32];
    if (!signature(data, len - 32, expected)) return false;
    unsigned diff = 0;
    for (int i = 0; i < 32; ++i) diff |= expected[i] ^ data[len - 32 + i];
    return diff == 0;
}

static void udp_task(void *unused)
{
    struct sockaddr_in listen_addr = {.sin_family=AF_INET, .sin_port=htons(UDP_PORT),
                                     .sin_addr.s_addr=htonl(INADDR_ANY)};
    while (true) {
        udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (udp_socket < 0) { vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
        if (bind(udp_socket, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
            close(udp_socket); udp_socket=-1; vTaskDelay(pdMS_TO_TICKS(2000)); continue;
        }
        // Bigger than a valid packet so truncated oversized datagrams cannot validate.
        uint8_t data[1200];
        while (true) {
            struct sockaddr_in from;
            socklen_t fromlen=sizeof(from);
            int n=recvfrom(udp_socket,data,sizeof(data),0,(struct sockaddr *)&from,&fromlen);
            if (n < 0) break;
            if (!verified(data,n)) continue;
            if (n==52 && memcmp(data,"COD1",4)==0) {
                memcpy(data,"COR1",4);
                signature(data,20,data+20);
                sendto(udp_socket,data,52,0,(struct sockaddr *)&from,fromlen);
            } else if (n==1064 && memcmp(data,"COF1",4)==0) {
                network_frame_t pending;
                pending.sequence=(uint32_t)data[4] | ((uint32_t)data[5]<<8)
                    | ((uint32_t)data[6]<<16) | ((uint32_t)data[7]<<24);
                memcpy(pending.pixels,data+8,1024);
                pending.sender=from;
                xQueueOverwrite(mailbox,&pending);
            }
        }
        close(udp_socket); udp_socket=-1;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *event)
{
    if (base==WIFI_EVENT && (id==WIFI_EVENT_STA_START || id==WIFI_EVENT_STA_DISCONNECTED)) {
        esp_wifi_connect();
    }
}

static int nibble(char c)
{
    if (c>='0' && c<='9') return c-'0';
    if (c>='a' && c<='f') return c-'a'+10;
    if (c>='A' && c<='F') return c-'A'+10;
    return -1;
}

static int unhex(const char *source, uint8_t *dest, size_t limit)
{
    size_t len=strlen(source);
    if (len%2 || len/2>limit) return -1;
    for (size_t i=0;i<len/2;i++) {
        int a=nibble(source[2*i]), b=nibble(source[2*i+1]);
        if (a<0 || b<0) return -1;
        dest[i]=(a<<4)|b;
    }
    return len/2;
}

bool network_command(char *line)
{
    if (strcmp(line,"NET?")==0) {
        esp_netif_ip_info_t info={0};
        if (station && esp_netif_get_ip_info(station,&info)==ESP_OK && info.ip.addr) {
            char response[48];
            snprintf(response,sizeof(response),"NET IP " IPSTR "\n",IP2STR(&info.ip));
            serial_reply(response);
        } else serial_reply("NET WAIT\n");
        return true;
    }
    if (strncmp(line,"WIFI1 ",6)!=0) return false;
    char ssid_hex[65]={0}, pass_hex[127]={0}, key_hex[65]={0}, extra;
    char ssid[33]={0}, password[64]={0}; uint8_t key[32];
    int fields=sscanf(line+6,"%64s %126s %64s %c",ssid_hex,pass_hex,key_hex,&extra);
    int ssid_len=unhex(ssid_hex,(uint8_t *)ssid,32);
    int pass_len=strcmp(pass_hex,"-")==0 ? 0 : unhex(pass_hex,(uint8_t *)password,63);
    if (fields!=3 || ssid_len<=0 || (pass_len!=0 && pass_len<8)
        || unhex(key_hex,key,32)!=32 || (int)strlen(ssid)!=ssid_len
        || (int)strlen(password)!=pass_len) {
        serial_reply("NET ERROR CONFIG\n"); return true;
    }
    nvs_handle_t store;
    esp_err_t err=nvs_open("oledwifi",NVS_READWRITE,&store);
    if (err==ESP_OK) {
        err=nvs_set_str(store,"ssid",ssid);
        if (err==ESP_OK) err=nvs_set_str(store,"password",password);
        if (err==ESP_OK) err=nvs_set_blob(store,"key",key,32);
        if (err==ESP_OK) err=nvs_commit(store);
        nvs_close(store);
    }
    memset(password,0,sizeof(password)); memset(pass_hex,0,sizeof(pass_hex));
    memset(key,0,sizeof(key)); memset(line,0,strlen(line));
    if (err!=ESP_OK) { serial_reply("NET ERROR STORAGE\n"); return true; }
    serial_reply("NET SAVED\n");
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(500));
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_restart();
    return true;
}

void network_init(void)
{
    mailbox=xQueueCreate(1,sizeof(network_frame_t));
    assert(mailbox);
    // Never erase unrelated NVS data as a recovery shortcut.
    esp_err_t err=nvs_flash_init();
    if (err!=ESP_OK) { serial_reply("NET ERROR NVS\n"); return; }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    station=esp_netif_create_default_wifi_sta();
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event,NULL));
    // Wi-Fi credentials are only loaded locally; never written to log output.
    wifi_config_t config={0};
    nvs_handle_t store;
    char ssid[33]={0},password[64]={0}; size_t slen=sizeof(ssid),plen=sizeof(password),klen=32;
    err=nvs_open("oledwifi",NVS_READONLY,&store);
    if (err!=ESP_OK) return;
    err=nvs_get_str(store,"ssid",ssid,&slen);
    if (err==ESP_OK) err=nvs_get_str(store,"password",password,&plen);
    if (err==ESP_OK) err=nvs_get_blob(store,"key",pairing_key,&klen);
    nvs_close(store);
    if (err!=ESP_OK || klen!=32 || !ssid[0]) return;
    memcpy(config.sta.ssid,ssid,strlen(ssid));
    memcpy(config.sta.password,password,strlen(password));
    config.sta.threshold.authmode=password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    config.sta.sae_pwe_h2e=WPA3_SAE_PWE_BOTH;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&config));
    memset(password,0,sizeof(password)); memset(&config,0,sizeof(config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    xTaskCreate(udp_task,"oled_udp",6144,NULL,4,NULL);
}

bool network_take_frame(network_frame_t *frame)
{
    return mailbox && xQueueReceive(mailbox,frame,0)==pdTRUE;
}

void network_ack(const network_frame_t *frame)
{
    uint8_t reply[40]={'C','O','A','1'};
    for (int i=0;i<4;i++) reply[4+i]=(frame->sequence>>(i*8))&0xff;
    if (signature(reply,8,reply+8) && udp_socket>=0)
        sendto(udp_socket,reply,sizeof(reply),0,(struct sockaddr *)&frame->sender,sizeof(frame->sender));
}
