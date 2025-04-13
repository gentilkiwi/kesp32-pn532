#include "kpn532_now.h"
#include "kpn532.h"

#define TAG "KPN532_NOW"

SemaphoreHandle_t PN532_NOW::xSemaphore;
uint8_t PN532_NOW::_recvWanted;
uint8_t PN532_NOW::_sendWanted;
esp_now_send_status_t PN532_NOW::_status;

uint8_t PN532_NOW::Buffer[ESP_NOW_MAX_DATA_LEN] = {0}, PN532_NOW::cbData = 0;

PN532_NOW::PN532_NOW(const uint8_t PeerAddr[ESP_NOW_ETH_ALEN], wifi_phy_rate_t rate) {
    this->Peer = {
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        .lmk = {0x00},
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
        .priv = NULL,
    };
    
    memcpy(this->Peer.peer_addr, PeerAddr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&this->Peer));
    //esp_now_set_peer_rate_config // WIFI_PHY_RATE_2M_S/*WIFI_PHY_RATE_1M_L*/
}

PN532_NOW::~PN532_NOW() {
    ESP_ERROR_CHECK(esp_now_del_peer(this->Peer.peer_addr));
}

void PN532_NOW::begin() {
    printf("ESP-NOW @ " MACSTR "\n", MAC2STR(this->Peer.peer_addr));
}

// Initiator

uint8_t PN532_NOW::InListPassiveTarget(uint8_t *pbUID, uint8_t cbUID, uint8_t *pcbUID, uint8_t SENS_RES[2], uint8_t *pSEL_RES, uint8_t *pbATS, uint8_t cbATS, uint8_t *pcbATS)  // NFC-A, 106kbps, 1 target, ATS not here
{
  uint8_t ret = 0;
  MYTARGET *pMyTg = (MYTARGET *) PN532_NOW::Buffer;

  PN532_NOW::_recvWanted = 1;
  if(xSemaphoreTake(this->xSemaphore, portMAX_DELAY))
  {
      if(PN532_NOW::cbData == sizeof(MYTARGET))
      {
        if (SENS_RES) {
          memcpy(SENS_RES, pMyTg->SENS_RES, 2);
        }

        if (pSEL_RES) {
          *pSEL_RES = pMyTg->SEL_RES;
        }

        if (pbUID && pcbUID) {
          if (cbUID >= pMyTg->cbUID) {
            memcpy(pbUID, pMyTg->UID, pMyTg->cbUID);
            *pcbUID = pMyTg->cbUID;
          }
          else {
            *pcbUID = 0;
          }
        }

        if (pbATS && pcbATS) {
          *pcbATS = 0;
        }  
          
        ret = 1;  
      }
  }

  return ret;
}

uint8_t PN532_NOW::InDataExchange(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode) {
    uint8_t ret = 0;

    PN532_NOW::_sendWanted = 1;
    ESP_ERROR_CHECK(esp_now_send(this->Peer.peer_addr, pcbInData, cbInData));
    if(xSemaphoreTake(this->xSemaphore, portMAX_DELAY))
    {
        if(PN532_NOW::_status == ESP_NOW_SEND_SUCCESS)
        {
            PN532_NOW::_recvWanted = 1;
            if(xSemaphoreTake(this->xSemaphore, portMAX_DELAY))
            {
                *ppReceived = PN532_NOW::Buffer;
                *pcbReceived = PN532_NOW::cbData;
                ret = 1;
            }
            else
            {
                *ppReceived = NULL;
                *pcbReceived = 0;
            }
        }
    }

    return ret;
}

uint8_t PN532_NOW::InCommunicateThru(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode) {
    uint8_t ret = 0;
    // TODO
    return ret;
}

const uint32_t PN532_NOW_SPECIAL_RELEASE = 0xc0febabe;

uint8_t PN532_NOW::InRelease(uint8_t *pErrorCode) {
    uint8_t ret = 0;

    PN532_NOW::_sendWanted = 1;
    ESP_ERROR_CHECK(esp_now_send(this->Peer.peer_addr, (const uint8_t*) &PN532_NOW_SPECIAL_RELEASE, sizeof(PN532_NOW_SPECIAL_RELEASE)));
    if(xSemaphoreTake(this->xSemaphore, portMAX_DELAY))
    {
        if(PN532_NOW::_status == ESP_NOW_SEND_SUCCESS)
        {
            ret = 1;
        }
    }

    return ret;
}

// Target

uint8_t PN532_NOW::TgInitAsTarget(uint8_t *pbUID, uint8_t cbUID, uint8_t SENS_RES[2], uint8_t SEL_RES) {
    uint8_t ret = 0;

    MYTARGET MyTg;
    MyTg.cbUID = cbUID;
    memcpy(MyTg.UID, pbUID, cbUID);
    memcpy(MyTg.SENS_RES, SENS_RES, 2);
    MyTg.SEL_RES = SEL_RES;

    PN532_NOW::_sendWanted = 1;
    ESP_ERROR_CHECK(esp_now_send(this->Peer.peer_addr, (const uint8_t*) &MyTg, sizeof(MyTg)));
    if(xSemaphoreTake(this->xSemaphore, portMAX_DELAY))
    {
        if(PN532_NOW::_status == ESP_NOW_SEND_SUCCESS)
        {
            ret = 1;
        }
    }

    return ret;
}

uint8_t PN532_NOW::TgGetData(uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode)
{
    uint8_t ret = 0;

    PN532_NOW::_recvWanted = 1;
    if(xSemaphoreTake(this->xSemaphore, portMAX_DELAY))
    {
        if((PN532_NOW::cbData == sizeof(PN532_NOW_SPECIAL_RELEASE)) && ((*(uint32_t *) PN532_NOW::Buffer) == PN532_NOW_SPECIAL_RELEASE))
        {
            ret = 0;
        }
        else
        {
            *ppReceived = PN532_NOW::Buffer;
            *pcbReceived = PN532_NOW::cbData;
            ret = 1;
        }
    }
    else
    {
        *ppReceived = NULL;
        *pcbReceived = 0;
    }

    return ret;
}

uint8_t PN532_NOW::TgSetData(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t *pErrorCode) {
    uint8_t ret = 0;

    PN532_NOW::_sendWanted = 1;
    ESP_ERROR_CHECK(esp_now_send(this->Peer.peer_addr, pcbInData, cbInData));
    if(xSemaphoreTake(this->xSemaphore, portMAX_DELAY))
    {
        if(PN532_NOW::_status == ESP_NOW_SEND_SUCCESS)
        {
            ret = 1;
        }
    }

    return ret;
}

uint8_t PN532_NOW::TgGetInitiatorCommand(uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode) {
    uint8_t ret = 0;
    // TODO
    return ret;
}

uint8_t PN532_NOW::TgResponseToInitiator(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t *pErrorCode) {
    uint8_t ret = 0;
    // TODO
    return ret;
}

uint8_t PN532_NOW::InitGlobalNOW(const uint8_t Channel)
{
    esp_err_t ret;
    
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(Channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, /*WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | */WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));
    
    PN532_NOW::_recvWanted = 0;
    PN532_NOW::_sendWanted = 0;
    PN532_NOW::xSemaphore = xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(PN532_NOW::espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(PN532_NOW::espnow_recv_cb));
    
    return 1;
}

void PN532_NOW::DeinitGlobalNOW()
{
    ESP_ERROR_CHECK(esp_now_unregister_recv_cb());
    ESP_ERROR_CHECK(esp_now_unregister_send_cb());
    ESP_ERROR_CHECK(esp_now_deinit());

    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    /*ESP_ERROR_CHECK*/(esp_netif_deinit()); // ESP_ERR_NOT_SUPPORTED
}

void PN532_NOW::espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if(PN532_NOW::_sendWanted)
    {
        PN532_NOW::_sendWanted = 0;
        PN532_NOW::_status = status;
        xSemaphoreGive(PN532_NOW::xSemaphore);
    }
}

void PN532_NOW::espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if(PN532_NOW::_recvWanted)
    {
        PN532_NOW::_recvWanted = 0;
        PN532_NOW::cbData = 0;
        if((len > 0) && len <= sizeof(PN532_NOW::Buffer))
        {
            memcpy(PN532_NOW::Buffer, data, len);
            PN532_NOW::cbData = (uint8_t) len;
        }
        else
        {
            ESP_LOGE(TAG, "espnow_recv_cb - len problem (%i, max is %hu)", len, sizeof(PN532_NOW::Buffer));
        }
        xSemaphoreGive(PN532_NOW::xSemaphore);
    }
}