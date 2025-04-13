#include "kpn532.h"

//uint8_t PN532::_bUseGlobalSPI = 0x00;
spi_host_device_t PN532::_spi_host_device;
spi_device_handle_t PN532::_spi_device;

const uint8_t PN532_ACK[] = { PN532_Data_Writing, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00 };
const uint8_t PN532_NACK[] = { PN532_Data_Writing, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00 };

#define PACKET_DATA_IN (this->Buffer + 7)
#define PACKET_DATA_OUT (this->Buffer + 8)

PN532::PN532(const gpio_num_t ss_pin, const gpio_num_t irq_pin, spi_device_handle_t hDevice)
  : hPN532(hDevice ? hDevice : PN532::_spi_device),  _ss(ss_pin), _irq(irq_pin) {
  gpio_config_t _ss_config = {
    .pin_bit_mask = BIT64(this->_ss),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  
  gpio_config_t _irq_config = {
    .pin_bit_mask = BIT64(this->_irq),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE,
  };
  
  this->xSemaphore = xSemaphoreCreateBinary();
  
  gpio_config(&_ss_config);
  gpio_config(&_irq_config);
  gpio_set_level(this->_ss, 1);
  gpio_isr_handler_add(this->_irq, PN532::ISR_NFC, this->xSemaphore);
}

PN532::~PN532() {
  gpio_isr_handler_remove(this->_irq);
  vSemaphoreDelete(this->xSemaphore);
  releaseSPI();
}

void PN532::acquireSPI() {
  gpio_set_level(this->_ss, 0);
}

void PN532::releaseSPI() {
  gpio_set_level(this->_ss, 1);
}

void PN532::begin() {
  uint8_t status;
  
  spi_transaction_t transaction = {
    .flags = 0,
    .cmd = 0,
    .addr = 0,
    .length = 0,
    .rxlength = 0,
    .user = NULL,
    .tx_buffer = NULL,
    .rx_buffer = NULL,
  };
  
#if (KPN532_OUTPUT_LEVEL >= KPN532_OUTPUT_LEVEL_INFO)
  uint8_t IC, Ver, Rev, Support;
#endif

  while (1) {
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = 2 * 8;
    transaction.rxlength = 0;
    transaction.tx_data[0] = PN532_Status_Reading;
    
    acquireSPI();
    vTaskDelay(pdMS_TO_TICKS(PN532_T_osc_start));
    spi_device_polling_transmit(this->hPN532, &transaction);
    status = transaction.rx_data[1];
    releaseSPI();

    if ((status != 0xff) && (status != 0xaa)) {  // seen, 0xff (init ?), 0xaa (default char ?), 0x08 (?), last bit - & 0x01 (ready or not ready)
      if (status & 0x01) {
        transaction.flags = 0;
        transaction.length = sizeof(PN532_ACK) * 8;
        transaction.tx_buffer = PN532_ACK;
        transaction.rxlength = 0;
        transaction.rx_buffer = NULL;
          
        acquireSPI();
        spi_device_polling_transmit(this->hPN532, &transaction);
        releaseSPI();
      } else {
        break;
      }
    }
    // TODO add delay interframe?
  }
  
    if (!SAMConfiguration(0x01)) {
#if (KPN532_OUTPUT_LEVEL >= KPN532_OUTPUT_LEVEL_ERROR)
    printf("Bad SAMConfiguration :(\n");
#endif
    while (1)
      ;
  }
  
#if (KPN532_OUTPUT_LEVEL >= KPN532_OUTPUT_LEVEL_INFO)
  if (GetFirmwareVersion(&IC, &Ver, &Rev, &Support)) {
      printf("PN5%02X %hhu.%hhu.%hhu", IC, Ver, Rev, Support);
    
    if (ReadRegister(PN53X_REG_CIU_Version, &Ver)) {
        printf(" %02X", Ver);
    }
    printf("\n");
  }
#endif

}

// Miscellaneous

uint8_t PN532::GetFirmwareVersion(uint8_t *pIC, uint8_t *pVer, uint8_t *pRev, uint8_t *pSupport) {
  uint8_t ret = 0;

  PACKET_DATA_IN[0] = PN532_CMD_GetFirmwareVersion;
  this->cbData = 1;

  if (Information_Frame_Exchange() && (this->cbData == 4)) {

    if (pIC) {
      *pIC = PACKET_DATA_OUT[0];
    }

    if (pVer) {
      *pVer = PACKET_DATA_OUT[1];
    }

    if (pRev) {
      *pRev = PACKET_DATA_OUT[2];
    }

    if (pSupport) {
      *pSupport = PACKET_DATA_OUT[3];
    }

    ret = 1;
  }

  return ret;
}

uint8_t PN532::ReadRegister(uint16_t Register, uint8_t *pValue) {
  uint8_t ret = 0;

  PACKET_DATA_IN[0] = PN532_CMD_ReadRegister;
  *(uint16_t *)(PACKET_DATA_IN + 1) = __builtin_bswap16(Register);
  this->cbData = 3;

  if (Information_Frame_Exchange() && (this->cbData == 1)) {
    *pValue = PACKET_DATA_OUT[0];
    ret = 1;
  }

  return ret;
}

uint8_t PN532::WriteRegister(uint16_t Register, uint8_t Value) {
  PACKET_DATA_IN[0] = PN532_CMD_WriteRegister;
  *(uint16_t *)(PACKET_DATA_IN + 1) = __builtin_bswap16(Register);
  PACKET_DATA_IN[3] = Value;
  this->cbData = 4;

  return Information_Frame_Exchange();
}

uint8_t PN532::WriteRegister(const PN53X_REGISTER_VALUE *pRV, uint8_t szData) {
  PACKET_DATA_IN[0] = PN532_CMD_WriteRegister;
  memcpy(PACKET_DATA_IN + 1, pRV, szData);
  this->cbData = 1 + szData;

  return Information_Frame_Exchange();
}

uint8_t PN532::SAMConfiguration(uint8_t IRQ) {
  PACKET_DATA_IN[0] = PN532_CMD_SAMConfiguration;
  PACKET_DATA_IN[1] = 0x01;
  PACKET_DATA_IN[2] = 0x00;
  PACKET_DATA_IN[3] = IRQ;
  this->cbData = 4;

  return Information_Frame_Exchange();
}

// RFcommunication

uint8_t PN532::RfConfiguration__RF_field(uint8_t ConfigurationData) {
  PACKET_DATA_IN[0] = PN532_CMD_RFConfiguration;
  PACKET_DATA_IN[1] = 0x01;
  PACKET_DATA_IN[2] = ConfigurationData;
  this->cbData = 3;

  return Information_Frame_Exchange();
}

void PN532::RfConfiguration__RF_field_fast(uint8_t ConfigurationData, unsigned int microseconds) {
  PACKET_DATA_IN[0] = PN532_CMD_RFConfiguration;
  PACKET_DATA_IN[1] = 0x01;
  PACKET_DATA_IN[2] = ConfigurationData;
  this->cbData = 3;

  Information_Frame_Host_To_PN532();
  if(microseconds) {
    ets_delay_us(microseconds);
  }
}

uint8_t PN532::RfConfiguration__Various_timings(uint8_t fATR_RES_Timeout, uint8_t fRetryTimeout) {
  PACKET_DATA_IN[0] = PN532_CMD_RFConfiguration;
  PACKET_DATA_IN[1] = 0x02;
  PACKET_DATA_IN[2] = 0x00;  // RFU
  PACKET_DATA_IN[3] = fATR_RES_Timeout;
  PACKET_DATA_IN[4] = fRetryTimeout;
  this->cbData = 5;

  return Information_Frame_Exchange();
}

uint8_t PN532::RfConfiguration__MaxRtyCOM(uint8_t MaxRtyCOM) {
  PACKET_DATA_IN[0] = PN532_CMD_RFConfiguration;
  PACKET_DATA_IN[1] = 0x04;
  PACKET_DATA_IN[2] = MaxRtyCOM;
  this->cbData = 3;

  return Information_Frame_Exchange();
}

uint8_t PN532::RfConfiguration__MaxRetries(uint8_t MxRtyPassiveActivation) {
  PACKET_DATA_IN[0] = PN532_CMD_RFConfiguration;
  PACKET_DATA_IN[1] = 0x05;
  PACKET_DATA_IN[2] = 0xff;
  PACKET_DATA_IN[3] = 0x01;
  PACKET_DATA_IN[4] = MxRtyPassiveActivation;
  this->cbData = 5;

  return Information_Frame_Exchange();
}

// Initiator

uint8_t PN532::InListPassiveTarget(uint8_t *pbUID, uint8_t cbUID, uint8_t *pcbUID, uint8_t SENS_RES[2], uint8_t *pSEL_RES, uint8_t *pbATS, uint8_t cbATS, uint8_t *pcbATS)  // NFC-A, 106kbps, 1 target, ATS not here
{
  uint8_t ret = 0, idx;

  PACKET_DATA_IN[0] = PN532_CMD_InListPassiveTarget;
  PACKET_DATA_IN[1] = 0x01;
  PACKET_DATA_IN[2] = 0x00;
  this->cbData = 3;

  if (Information_Frame_Exchange() && (this->cbData > 1) && (PACKET_DATA_OUT[0] == 0x01)) {

    if (SENS_RES) {
      *(uint16_t *)SENS_RES = *(uint16_t *)(PACKET_DATA_OUT + 2);
    }

    if (pSEL_RES) {
      *pSEL_RES = PACKET_DATA_OUT[4];
    }

    if (pbUID && pcbUID) {
      if (cbUID >= PACKET_DATA_OUT[5]) {
        memcpy(pbUID, PACKET_DATA_OUT + 6, PACKET_DATA_OUT[5]);
        *pcbUID = PACKET_DATA_OUT[5];
      }
      else {
        *pcbUID = 0;
      }
    }

    if (pbATS && pcbATS) {
      idx = 6 + PACKET_DATA_OUT[5];
      if((idx < this->cbData) && ((this->cbData - idx) == PACKET_DATA_OUT[idx]) && (cbATS >= PACKET_DATA_OUT[idx])) {
        memcpy(pbATS, PACKET_DATA_OUT + idx, PACKET_DATA_OUT[idx]);
        *pcbATS = PACKET_DATA_OUT[idx]; 
      }
      else {
        *pcbATS = 0;
      }
    }

    ret = 1;
  }

  return ret;
}

uint8_t PN532::InDataExchange(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode) {
  uint8_t ret = 0, errorCode = 0;

  PACKET_DATA_IN[0] = PN532_CMD_InDataExchange;
  PACKET_DATA_IN[1] = 0x01;
  memcpy(PACKET_DATA_IN + 2, pcbInData, cbInData);
  this->cbData = 2 + cbInData;

  if (pErrorCode || (ppReceived && pcbReceived)) {
    if (Information_Frame_Exchange() && this->cbData) {
      errorCode = PACKET_DATA_OUT[0] & 0x3f;

      if (ppReceived && pcbReceived) {
        if (!errorCode) {
          *ppReceived = PACKET_DATA_OUT + 1;
          *pcbReceived = this->cbData - 1;
          ret = 1;
        } else {
          *ppReceived = NULL;
          *pcbReceived = 0;
        }
      }
    }

    if (pErrorCode) {
      *pErrorCode = errorCode;
    }

  } else {
    ret = Information_Frame_Exchange(0x01);
  }

  return ret;
}

uint8_t PN532::InCommunicateThru(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode) {
  uint8_t ret = 0, errorCode = 0;

  PACKET_DATA_IN[0] = PN532_CMD_InCommunicateThru;
  memcpy(PACKET_DATA_IN + 1, pcbInData, cbInData);
  this->cbData = 1 + cbInData;

  if (pErrorCode || (ppReceived && pcbReceived)) {
    if (Information_Frame_Exchange() && this->cbData) {
      errorCode = PACKET_DATA_OUT[0] & 0x3f;

      if (ppReceived && pcbReceived) {
        if (!errorCode) {
          *ppReceived = PACKET_DATA_OUT + 1;
          *pcbReceived = this->cbData - 1;
          ret = 1;
        } else {
          *ppReceived = NULL;
          *pcbReceived = 0;
        }
      }
    }

    if (pErrorCode) {
      *pErrorCode = errorCode;
    }

  } else {
    ret = Information_Frame_Exchange(0x01);
  }

  return ret;
}

uint8_t PN532::InRelease(uint8_t *pErrorCode) {
  uint8_t ret = 0, errorCode;

  PACKET_DATA_IN[0] = PN532_CMD_InRelease;
  PACKET_DATA_IN[1] = 0x00;
  this->cbData = 2;

  if (Information_Frame_Exchange() && this->cbData) {
    errorCode = PACKET_DATA_OUT[0] & 0x3f;
    if (!errorCode) {
      ret = 1;
    } else if (pErrorCode) {
      *pErrorCode = errorCode;
    }
  }

  return ret;
}

// Target

uint8_t PN532::TgInitAsTarget(uint8_t *pbUID, uint8_t cbUID, uint8_t SENS_RES[2], uint8_t SEL_RES) {
  uint8_t ret = 0, mode;

  if (cbUID >= 4) {
    PACKET_DATA_IN[0] = PN532_CMD_TgInitAsTarget;
    PACKET_DATA_IN[1] = 0x05;  // PICC only & passive

    PACKET_DATA_IN[2] = SENS_RES[1];
    PACKET_DATA_IN[3] = SENS_RES[0];
    PACKET_DATA_IN[4] = pbUID[1];
    PACKET_DATA_IN[5] = pbUID[2];
    PACKET_DATA_IN[6] = pbUID[3];
    PACKET_DATA_IN[7] = SEL_RES;
    memset(PACKET_DATA_IN + 8, 0, 29); // FelicaParams[] + NFCID3t - ATR_RES + LenGT
    PACKET_DATA_IN[37] = 0x01; // LenTk
    PACKET_DATA_IN[38] = 0x80;
    this->cbData = 39;

    if (Information_Frame_Exchange() && this->cbData) {
      mode = PACKET_DATA_OUT[0] & 0x7f;
      if ((mode & 0x78) == 0x08) {
        ret = 1;
      }
    }
  }

  return ret;
}

uint8_t PN532::TgGetData(uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode) {
  uint8_t ret = 0, errorCode;

  PACKET_DATA_IN[0] = PN532_CMD_TgGetData;
  this->cbData = 1;

  if (Information_Frame_Exchange() && this->cbData) {
    errorCode = PACKET_DATA_OUT[0] & 0x3f;
    if (!errorCode) {
      *ppReceived = PACKET_DATA_OUT + 1;
      *pcbReceived = this->cbData - 1;
      ret = 1;
    } else if (pErrorCode) {
      *pErrorCode = errorCode;
    }
  }

  if (!ret) {
    *ppReceived = NULL;
    *pcbReceived = 0;
  }

  return ret;
}

uint8_t PN532::TgSetData(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t *pErrorCode) {
  uint8_t ret = 0, errorCode;

  PACKET_DATA_IN[0] = PN532_CMD_TgSetData;
  memcpy(PACKET_DATA_IN + 1, pcbInData, cbInData);
  this->cbData = 1 + cbInData;

  if (Information_Frame_Exchange() && this->cbData) {
    errorCode = PACKET_DATA_OUT[0] & 0x3f;
    if (!errorCode) {
      ret = 1;
    } else if (pErrorCode) {
      *pErrorCode = errorCode;
    }
  }

  return ret;
}

uint8_t PN532::TgGetInitiatorCommand(uint8_t **ppReceived, uint8_t *pcbReceived, uint8_t *pErrorCode) {
  uint8_t ret = 0, errorCode;

  PACKET_DATA_IN[0] = PN532_CMD_TgGetInitiatorCommand;
  this->cbData = 1;

  if (Information_Frame_Exchange() && this->cbData) {
    errorCode = PACKET_DATA_OUT[0] & 0x3f;
    if (!errorCode) {
      *ppReceived = PACKET_DATA_OUT + 1;
      *pcbReceived = this->cbData - 1;
      ret = 1;
    } else if (pErrorCode) {
      *pErrorCode = errorCode;
    }
  }

  if (!ret) {
    *ppReceived = NULL;
    *pcbReceived = 0;
  }

  return ret;
}

uint8_t PN532::TgResponseToInitiator(const uint8_t *pcbInData, const uint8_t cbInData, uint8_t *pErrorCode) {
  uint8_t ret = 0, errorCode;

  PACKET_DATA_IN[0] = PN532_CMD_TgRespondToInitiator;
  memcpy(PACKET_DATA_IN + 1, pcbInData, cbInData);
  this->cbData = 1 + cbInData;

  if (Information_Frame_Exchange() && this->cbData) {
    errorCode = PACKET_DATA_OUT[0] & 0x3f;
    if (!errorCode) {
      ret = 1;
    } else if (pErrorCode) {
      *pErrorCode = errorCode;
    }
  }

  return ret;
}

uint8_t PN532::Information_Frame_Exchange(uint8_t bNoAnswer) {
  uint8_t ret = 0, cmd = PACKET_DATA_IN[0] + 1;

  Information_Frame_Host_To_PN532();
  if (Wait_Ready_IRQ()) {
    if (Generic_Frame_PN532_To_Host() == PN532_ACK_FRAME) {
      if (bNoAnswer) {
        // Was testing to send ACK to avoid waiting NFC response
        // but sending ACK (or consecutive command) is to abord the current command :(
        // Strategy is now to use smaller timeout (100µs) and to handle it in protocol
        // using bNoAnswer = 1 is now only for particular usage
        //
        // memcpy(this->Buffer, PN532_ACK, sizeof(PN532_ACK));
        // digitalWrite(this->_ss, LOW);
        // SPI.transfer(this->Buffer, sizeof(PN532_ACK));
        // digitalWrite(this->_ss, HIGH);
        ret = 1;
      } else {
        if (Wait_Ready_IRQ()) {
          if (Generic_Frame_PN532_To_Host() == PN532_NORMAL_INFORMATION_FRAME) {
            if (cmd == PACKET_DATA_OUT[-1]) {
              this->cbData = PACKET_DATA_OUT[-4] - 2;
              ret = 1;
            }
          }
        }
      }
    }
  }
  return ret;
}

uint8_t PN532::Wait_Ready_IRQ() {
  return (xSemaphoreTake(this->xSemaphore, portMAX_DELAY) == pdTRUE);
}

void PN532::Information_Frame_Host_To_PN532() {
  uint8_t DCS = 0, i;
  spi_transaction_t transaction = {
    .flags = 0,
    .cmd = 0,
    .addr = 0,
    .length = 0,
    .rxlength = 0,
    .user = NULL,
    .tx_buffer = NULL,
    .rx_buffer = NULL,
  };

  if (this->cbData) {
    this->Buffer[0] = PN532_Data_Writing;

    this->Buffer[1] = 0x00;                     // PREAMBLE - Preamble
    this->Buffer[2] = 0x00;                     // START CODE - Start of Packet Code
    this->Buffer[3] = 0xff;                     // ...
    this->Buffer[4] = this->cbData + 1;         // LEN - Packet Length
    this->Buffer[5] = ~Buffer[4] + 1;           // LCS - Packet Length Checksum
    this->Buffer[6] = PN532_TFI_Host_to_PN532;  // TFI - Specific PN532 Frame Identifier
    for (i = 0; i < (this->cbData + 1); i++) {
      DCS += this->Buffer[6 + i];
    }
    this->Buffer[7 + this->cbData] = ~DCS + 1;
    this->Buffer[7 + this->cbData + 1] = 0x00;

    transaction.length = (7 + this->cbData + 1 + 1) * 8;
    transaction.tx_buffer = this->Buffer;

    acquireSPI();
    spi_device_polling_transmit(this->hPN532, &transaction);
    releaseSPI();
  }
}

PN532_FRAME_TYPE PN532::Generic_Frame_PN532_To_Host() {
  PN532_FRAME_TYPE ret = PN532_UNKNOWN;
  uint8_t DCS = 0, i, cbDataIn;
  spi_transaction_t transaction = {
    .flags = 0,
    .cmd = 0,
    .addr = 0,
    .length = 0,
    .rxlength = 0,
    .user = NULL,
    .tx_buffer = NULL,
    .rx_buffer = NULL,
  };

  this->Buffer[0] = PN532_Data_Reading;
  this->cbData = 7;

  transaction.length = this->cbData * 8;
  transaction.tx_buffer = this->Buffer;
  transaction.rxlength = 0;
  transaction.rx_buffer = this->Buffer; // we want the return value here :)

  acquireSPI();
  spi_device_polling_transmit(this->hPN532, &transaction);
  cbDataIn = this->Buffer[4];                // often used

  if ((this->Buffer[0] == 0x01) || (this->Buffer[0] == 0xff) || (this->Buffer[0] == 0xaa))  // ? :')
  {
    if ((this->Buffer[1] == 0x00) && (this->Buffer[2] == 0x00) && (this->Buffer[3] == 0xff)) {
      if ((cbDataIn == 0x00) && (this->Buffer[5] == 0xff) && (this->Buffer[6] == 0x00)) {
        ret = PN532_ACK_FRAME;
      } else if ((cbDataIn == 0xff) && (this->Buffer[5] == 0x00) && (this->Buffer[6] == 0x00)) {
        ret = PN532_NACK_FRAME;
      } else if (this->Buffer[5] == (uint8_t)(~cbDataIn + 1)) {
        
        transaction.length = (cbDataIn - 1 + 2) * 8;
        transaction.tx_buffer = this->Buffer + 7;
        transaction.rxlength = 0;
        transaction.rx_buffer = this->Buffer + 7; // we want the return value here :)
        
        spi_device_polling_transmit(this->hPN532, &transaction); 
        
        this->cbData += cbDataIn - 1 + 2;
        for (i = 0; i < cbDataIn; i++) {
          DCS += this->Buffer[6 + i];
        }

        if (this->Buffer[6 + cbDataIn] == (uint8_t)(~DCS + 1)) {
          if (this->Buffer[6] == PN532_TFI_PN532_to_Host) {
            ret = PN532_NORMAL_INFORMATION_FRAME;
          } else if (cbDataIn == 0x01) {
            ret = PN532_ERROR_FRAME;
          }
        }
      }
    }
  }

  releaseSPI();

  return ret;
}

void PN532::ISR_NFC(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t) arg, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

uint8_t PN532::InitGlobalSPI(const gpio_num_t clk_pin, const gpio_num_t mosi_pin, const gpio_num_t miso_pin, const int clock_speed_hz, const spi_host_device_t spi_host_device) {
  uint8_t ret = 0;
  esp_err_t esp_ret;

  spi_bus_config_t spi_bus_cfg = {
    .mosi_io_num = mosi_pin,
    .miso_io_num = miso_pin,
    .sclk_io_num = clk_pin,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .data4_io_num = -1,
    .data5_io_num = -1,
    .data6_io_num = -1,
    .data7_io_num = -1,
    .max_transfer_sz = 1024,//SOC_SPI_MAXIMUM_BUFFER_SIZE,
    .flags = 0,
    .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
    .intr_flags = 0,
  };

  spi_device_interface_config_t spi_dev_cfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,
    .clock_source = SPI_CLK_SRC_DEFAULT,
    .duty_cycle_pos = 128,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = clock_speed_hz,
    .input_delay_ns = 0,
    .spics_io_num = -1,
    .flags = SPI_DEVICE_BIT_LSBFIRST,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL,
  };

    _spi_host_device = spi_host_device;

	esp_ret = spi_bus_initialize(_spi_host_device, &spi_bus_cfg, SPI_DMA_CH_AUTO/*SPI_DMA_DISABLED*/);
    if(esp_ret == ESP_OK)
	{
		esp_ret = spi_bus_add_device(_spi_host_device, &spi_dev_cfg, &_spi_device);
		if(esp_ret == ESP_OK)
		{
            esp_ret = spi_device_acquire_bus(_spi_device, portMAX_DELAY);
            if(esp_ret == ESP_OK)
            {
                ret = 1;
            }
            
            if(!ret)
            {
                spi_bus_remove_device(_spi_device);
            }
        }
        
        if(!ret)
        {
            spi_bus_free(_spi_host_device);
        }
    }
    
    return ret;
}

void PN532::DeinitGlobalSPI() {
    spi_device_release_bus(_spi_device);
    spi_bus_remove_device(_spi_device);
    spi_bus_free(_spi_host_device);
}

void PN532::PrintHex(const uint8_t *pcbData, const size_t cbData, const uint8_t flags) {
  size_t i, idx;

  for (i = 0; i < cbData; i++) {
    idx = (flags & PN532_PRINTHEX_REV) ? cbData - 1 - i : i;
    printf("%02x", pcbData[idx]);
  }
  
  if(!(flags & PN532_PRINTHEX_NOLN)) {
    printf("\n");
  }
}
