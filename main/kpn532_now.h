#ifndef _KPN532_NOW_H_INCLUDED
#define _KPN532_NOW_H_INCLUDED
#include <string.h>
#include <rom/ets_sys.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"

typedef struct _MYTARGET {
    uint8_t cbUID;
    uint8_t UID[10];
    uint8_t SENS_RES[2];
    uint8_t SEL_RES;
} MYTARGET, *PMYTARGET;

class PN532_NOW {

public:
    PN532_NOW(const uint8_t PeerAddr[ESP_NOW_ETH_ALEN], wifi_phy_rate_t rate = WIFI_PHY_RATE_1M_L);
    ~PN532_NOW();

    void begin();

    // Initiator
    uint8_t InListPassiveTarget(uint8_t *pbUID, uint8_t cbUID, uint8_t *pcbUID, uint8_t SENS_RES[2], uint8_t *pSEL_RES, uint8_t *pbATS = NULL, uint8_t cbATS = 0, uint8_t *pcbATS = NULL);  // NFC-A, 106kbps, 1 target, ATS not here
    uint8_t InDataExchange(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t **ppReceived = NULL, uint8_t *pcbReceived = NULL, uint8_t *pErrorCode = NULL);
    uint8_t InCommunicateThru(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t **ppReceived = NULL, uint8_t *pcbReceived = NULL, uint8_t *pErrorCode = NULL);
    uint8_t InRelease(uint8_t *pErrorCode = NULL);
    // Target
    uint8_t TgInitAsTarget(uint8_t *pbUID, uint8_t cbUID, uint8_t SENS_RES[2], uint8_t SEL_RES);
    uint8_t TgGetData(uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode = NULL);
    uint8_t TgSetData(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t *pErrorCode = NULL);
    uint8_t TgGetInitiatorCommand(uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode = NULL);
    uint8_t TgResponseToInitiator(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t *pErrorCode = NULL);

private:
    esp_now_peer_info_t Peer;

public:
    static uint8_t InitGlobalNOW(const uint8_t Channel = 1);
    static void DeinitGlobalNOW();

private:
    static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    static uint8_t Buffer[ESP_NOW_MAX_DATA_LEN], cbData;
    static uint8_t _recvWanted;
    static uint8_t _sendWanted;
    static esp_now_send_status_t _status;
    static SemaphoreHandle_t xSemaphore;
};

#endif
