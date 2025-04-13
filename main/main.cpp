#include "kpn532.h"
#include "kpn532_now.h"

#define KPN532_RELAY_VERBOSE 0  // or 1, be careful, verbose slow down the relay
#define KPN532_RELAY_RAW 0      // 0 = TgGetData, InDataExchange, TgSetData (WTX, slower, overall faster because ISO/DEP and no real deselect/hlt)
                                // 1 = TgGetInitiatorCommand, InCommunicateThru, TgResponseToInitiator (no WTX, faster, overall slower because of real deselect/hlt),

#if KPN532_RELAY_RAW == 0
#define KPN532_RELAY_GET TgGetData
#define KPN532_RELAX_EXC InDataExchange
#define KPN532_RELAY_SET TgSetData
#else
#define KPN532_RELAY_GET TgGetInitiatorCommand
#define KPN532_RELAX_EXC InCommunicateThru
#define KPN532_RELAY_SET TgResponseToInitiator
#endif

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define SPI             SPI2_HOST
#define GPIO_CLK        GPIO_NUM_12
#define GPIO_MOSI       GPIO_NUM_11
#define GPIO_MISO       GPIO_NUM_13

#define KPN532_0_CS     GPIO_NUM_10
#define KPN532_0_IRQ    GPIO_NUM_4
#if CONFIG_IDF_TARGET_ESP32S2
#define KPN532_1_CS     GPIO_NUM_34
#else // bad choice for S3 & kimonoboard, no 34
#define KPN532_1_CS     GPIO_NUM_48
#endif
#define KPN532_1_IRQ    GPIO_NUM_1
#else
#error Target not supported	
#endif

//#define KPN532_ESP_NOW_SENDER
#define KPN532_ESP_NOW_RECEIVER

// const uint8_t ESP32S2_MAC_ADDR_0[ESP_NOW_ETH_ALEN] = {0x7c, 0xdf, 0xa1, 0x9e, 0x5c, 0x3c}; //S2
const uint8_t ESP32S2_MAC_ADDR_0[ESP_NOW_ETH_ALEN] = {0x48, 0x27, 0xe2, 0xfc, 0x2d, 0xec}; // S3
//const uint8_t ESP32S2_MAC_ADDR_1[ESP_NOW_ETH_ALEN] = {0x7c, 0xdf, 0xa1, 0x9e, 0x5b, 0xba}; // S2
const uint8_t ESP32S2_MAC_ADDR_1[ESP_NOW_ETH_ALEN] = {0x48, 0x27, 0xe2, 0xfc, 0x2e, 0x98}; // S3

#if defined(KPN532_ESP_NOW_SENDER)
PN532 *pNFCReader;
PN532_NOW *pNFCEmulator;
#elif defined(KPN532_ESP_NOW_RECEIVER)
PN532_NOW *pNFCReader;
PN532 *pNFCEmulator;
#else
PN532 *pNFCReader, *pNFCEmulator;
#endif

extern "C" void app_main(void)
{
    uint8_t *pResult, cbResult, bSuccess;
    uint8_t UID[10], cbUID = 0, SENS_RES[2], SEL_RES;
    uint8_t ATS[0x40], cbATS;

    printf("- kpn532 relay -\n");
    printf("V:%s M:%s\n", KPN532_RELAY_VERBOSE ? "full" : "min", KPN532_RELAY_RAW ? "RAW" : "ISO/DEP");

    if(PN532::InitGlobalSPI(GPIO_CLK, GPIO_MOSI, GPIO_MISO, 3000000, SPI))
    {
        gpio_install_isr_service(0);

        #if defined(KPN532_ESP_NOW_SENDER) || defined(KPN532_ESP_NOW_RECEIVER)
        PN532_NOW::InitGlobalNOW(1);
        uint8_t mac[ESP_NOW_ETH_ALEN];
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
        printf("ESP-NOW @ " MACSTR " (local)\n", MAC2STR(mac));
        #endif

        #if defined(KPN532_ESP_NOW_SENDER)
        printf("** Real READER side ** (remote emulator)\n");
        pNFCReader = new PN532(KPN532_0_CS, KPN532_0_IRQ);
        pNFCEmulator = new PN532_NOW(ESP32S2_MAC_ADDR_1);
        #elif defined(KPN532_ESP_NOW_RECEIVER)
        printf("** Real EMULATOR side ** (remote reader)\n");
        pNFCReader = new PN532_NOW(ESP32S2_MAC_ADDR_0);
        pNFCEmulator = new PN532(KPN532_1_CS, KPN532_1_IRQ);
        #else
        pNFCReader = new PN532(KPN532_0_CS, KPN532_0_IRQ);
        pNFCEmulator = new PN532(KPN532_1_CS, KPN532_1_IRQ);
        #endif

        printf("0|");
        pNFCReader->begin();
        printf("1|");
        pNFCEmulator->begin();
    
        while(1)
        {
            printf("~Waiting target~\n");
            if (pNFCReader->InListPassiveTarget(UID, sizeof(UID), &cbUID, SENS_RES, &SEL_RES, ATS, sizeof(ATS), &cbATS))
            {
                printf("SENS_RES: ");
                PN532::PrintHex(SENS_RES, sizeof(SENS_RES));
                printf("SEL_RES : ");
                PN532::PrintHex(&SEL_RES, sizeof(SEL_RES));
                printf("UID (%2hhu): ", cbUID);
                PN532::PrintHex(UID, cbUID);
                printf("ATS(%3hhu): ", cbATS);
                PN532::PrintHex(ATS, cbATS);

                printf("~Waiting reader~\n");
                if (pNFCEmulator->TgInitAsTarget(UID, cbUID, SENS_RES, SEL_RES))
                {
                    printf("|Reader detected\n");
                    do
                    {
                        bSuccess = 0;

                        if (pNFCEmulator->KPN532_RELAY_GET(&pResult, &cbResult))
                        {
                        #if KPN532_RELAY_VERBOSE
                            printf("< ");
                            PN532::PrintHex(pResult, cbResult);
                        #endif
                            if (pNFCReader->KPN532_RELAX_EXC(pResult, cbResult, &pResult, &cbResult))
                            {
                            #if KPN532_RELAY_VERBOSE
                                printf("> ");
                                PN532::PrintHex(pResult, cbResult);
                            #endif
                                bSuccess = pNFCEmulator->KPN532_RELAY_SET(pResult, cbResult);
                            }
                        }
                    } while (bSuccess);
                }
                
                if (pNFCReader->InRelease())
                {
                    printf("|Target released\n");
                }
            }
        }
        delete pNFCReader;
        delete pNFCEmulator;

        gpio_uninstall_isr_service();
        
        PN532::DeinitGlobalSPI();
    }
}
