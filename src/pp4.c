/*
 *  Copyright (C) 2021  Skip Hansen
 * 
 *  Code derived from Public Domain morse_code example from the espidf.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms and conditions of the GNU General Public License,
 *  version 2, as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

#define TAG __FUNCTION__

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#ifndef RMT_TX_GPIO
#define RMT_TX_GPIO     23
#endif

#define PP4_MAX_FRAME_LEN     100
#define PP4_CODES_PER_BYTE    4

#define SYNC_BYTE             0xaa


// rmt clock: 80 Mhz
// t=1/32768: 30.52us
// 00: 61us  (2t=61.03)    zero for 4883 clocks, actual = 61.0375
// 01: 244us (8t=244.14)   zero for 19531 clocks, actual = 244.1375
// 10: 122us (4t=122.07)   zero for 9766 clocks, actual = 122.075
// 11: 183us (6t=183.10)   zero for 14648 clocks, actual = 183.1
// Burst: 40us, one for 3200 clocks, error 0

#define DURATION_BURST     3200
#define DURATION_CODE00    4883
#define DURATION_CODE01    19531
#define DURATION_CODE10    9776
#define DURATION_CODE11    14648
#define DURATION_250_US    20000

static const rmt_item32_t PP4_code_00 = {
   {{ DURATION_BURST, 1, DURATION_CODE00, 0 }}
};

static const rmt_item32_t PP4_code_01 = {
   {{ DURATION_BURST, 1, DURATION_CODE01, 0 }}
};

static const rmt_item32_t PP4_code_10 = {
   {{ DURATION_BURST, 1, DURATION_CODE10, 0 }}
};

static const rmt_item32_t PP4_code_11 = {
   {{ DURATION_BURST, 1, DURATION_CODE11, 0 }}
};

static const rmt_item32_t PP4_final_burst = {
   {{ DURATION_BURST, 1, DURATION_BURST, 0 }}
};

static const rmt_item32_t Delay_500_us = {
   {{ DURATION_250_US, 0, DURATION_250_US, 0 }}
};



static uint8_t gDataBuf[PP4_MAX_FRAME_LEN];
static rmt_item32_t gTxBuf[(PP4_MAX_FRAME_LEN * PP4_CODES_PER_BYTE) + 5];

void SendStartupTestData(void);

/*
 * Initialize the RMT Tx channel
 */
void rmt_tx_init(void)
{
   ESP_LOGI(TAG, "Configuring transmitter");
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO, RMT_TX_CHANNEL);
    config.tx_config.carrier_en = true;
    config.tx_config.carrier_duty_percent = 50;
    config.tx_config.carrier_freq_hz = 1250000; // 1.25 Mhz
    config.clk_div = 1;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
#if 1
// for easy of hardware testing ... pressing reset sends flash LED command
    SendStartupTestData();
#endif
}

static int PP4Send(uint8_t *Data,int DataLen,int RepeatCount)
{
   int i;
   int j;
   int k = 0;
   uint8_t Byte;
   int Ret = '0';    // Assume the best
   esp_err_t Err;

#ifdef LED_BUILTIN
   gpio_set_level(LED_BUILTIN,1);
#endif

   if(DataLen < PP4_MAX_FRAME_LEN) {
      ESP_LOGI(TAG,"Sending %d byte frame %d times",DataLen,RepeatCount);

   // Delay 2 ms before starting the transmission (for repeat spacing)
      gTxBuf[k++] = Delay_500_us;
      gTxBuf[k++] = Delay_500_us;
      gTxBuf[k++] = Delay_500_us;
      gTxBuf[k++] = Delay_500_us;

      for(i = 0; i < DataLen; i++) {
         Byte = *Data++;
         for(j = 0; j < PP4_CODES_PER_BYTE; j++) {
            switch(Byte & 0x3) {
               case 0:
                  gTxBuf[k++] = PP4_code_00;
                  break;
                  
               case 1:
                  gTxBuf[k++] = PP4_code_01;
                  break;

               case 2:
                  gTxBuf[k++] = PP4_code_10;
                  break;

               case 3:
                  gTxBuf[k++] = PP4_code_11;
                  break;
            }
            Byte >>= 2;
         }
      }
      gTxBuf[k++] = PP4_final_burst;

      for(i = 0; i < RepeatCount; i++) {
         Err = rmt_write_items(RMT_TX_CHANNEL,gTxBuf, k, true);
         ESP_ERROR_CHECK(Err);
         if(Err != ESP_OK) {
            ESP_LOGE(TAG,"rmt_write_items failed: %d",Err);
            Ret = '1';  // error
            break;
         }
      }
      ESP_LOGI(TAG, "%d transmissions complete",RepeatCount);

#ifdef LED_BUILTIN
      gpio_set_level(LED_BUILTIN,0);
#endif
   }
   else {
      ESP_LOGE(TAG,"Error: frame is too big (%d bytes)",DataLen);
      Ret = '1';  // error
   }

   return Ret;
}

uint16_t CRCCalc(uint8_t *data,int datalength) 
{
    uint16_t result = 0x8408;
    uint16_t poly = 0x8408;
    
    for (int i = 0; i < datalength-2; i++) {
        result ^= data[i];
        
        for (int j = 0; j < 8; j++) {
            if (result & 1) {
                result >>= 1;
                result ^= poly;
            }
            else {
                result >>= 1;
            }
        }
    }

    return result;
}

typedef enum {
   STATE_WAIT_4_SYNC,
   STATE_REPEAT_MSB,
   STATE_REPEAT_LSB,
   STATE_DATA_LEN,
   STATE_DATA
} ParseState;

static int ProcessData(uint8_t *Data,int DataLen,int RepeatCount)
{
   uint16_t CalculatedCrc = CRCCalc(Data,DataLen);
   uint16_t Crc = Data[DataLen - 2] + (Data[DataLen - 1] << 8);
   int Ret;

   if(CalculatedCrc == Crc) {
      ESP_LOGI(TAG,"CRC is correct");
      Ret = PP4Send(Data,DataLen,RepeatCount);
   }
   else {
      ESP_LOGE(TAG,"CRC Error: Crc 0x%x, calculated 0x%x",Crc,CalculatedCrc);
      Ret = '1';  // error
   }

   return Ret;
}

// The serial protocol is:
// (byte) data size        (STATE_DATA_LEN)
// (byte) repeat count     (STATE_REPEAT_LSB)
// Data bytes...           (STATE_DATA)
// Transmission begins as soon as the last data byte is received
// Once done, the MCU replies with character 'A'
//
// Bluetooth protocol
// Sync Byte 170/0xaa      (STATE_WAIT_4_SYNC)
// (byte) Repeat count MSB (STATE_REPEAT_MSB)
// (byte) Repeat count MSB (STATE_REPEAT_LSB)
// (byte) Data length      (STATE_DATA_LEN)
// Data bytes...           (STATE_DATA)
static int ParseData(uint8_t Data,bool bSerialPort)
{
   static ParseState BT_State = STATE_WAIT_4_SYNC;
   static ParseState bSerialPort_State = STATE_DATA_LEN;
   ParseState State = bSerialPort ? bSerialPort_State : BT_State;
   static int RepeatCount;
   static uint8_t DataLen = 0;
   static uint8_t DataCnt = 0;
   int Ret = -1;

   switch(State) {
      case STATE_WAIT_4_SYNC:
         if(Data == SYNC_BYTE) {
            ESP_LOGI(TAG, "Got sync");
            State = STATE_REPEAT_MSB;
         }
         break;

      case STATE_REPEAT_MSB:
         RepeatCount = Data << 8;
         State = STATE_REPEAT_LSB;
         break;

      case STATE_REPEAT_LSB:
         if(bSerialPort) {
            RepeatCount = Data;
            State = STATE_DATA;
         }
         else {
            RepeatCount += Data;
            State = STATE_DATA_LEN;
         }
         ESP_LOGI(TAG, "RepeatCount: %d",RepeatCount);
         break;

      case STATE_DATA_LEN:
         DataLen = Data;
         if(DataLen == 0 || DataLen > PP4_MAX_FRAME_LEN) {
            if(!bSerialPort) {
               State = STATE_WAIT_4_SYNC;
            }
         }
         else {
            DataCnt = 0;
            State = bSerialPort ? STATE_REPEAT_LSB : STATE_DATA;
            ESP_LOGI(TAG, "DataLen: %d",DataLen);
         }
         break;

      case STATE_DATA:
         gDataBuf[DataCnt++] = Data;
         if(DataCnt == DataLen) {
            State = bSerialPort ? STATE_DATA_LEN : STATE_WAIT_4_SYNC;
            ESP_LOGI(TAG, "Calling ProcessData");
            Ret = ProcessData(gDataBuf,DataLen,RepeatCount);
         }
         break;
   }

   if(bSerialPort) {
      bSerialPort_State = State;
   }
   else {
      BT_State = State;
   }

   return Ret;
}

int ParsePP4Data(uint8_t *Data,int DataLen,bool bSerialPort)
{
   int i;
   int Ret = -1;
   for(i = 0; i < DataLen; i++) {
      if((Ret = ParseData(*Data++,bSerialPort)) != -1) {
         break;
      }
   }

   return Ret;
}

uint8_t StartupTestData[] = {
// Command to do a "SmartTag LED low flash" twice on an DotMatrix Tag
   0xaa,0x00,0x96,0x0d,0x85,0x00,0x00,0x00,0x00,0x06,0x49,0x00,0x00,0x00,0x01,0x2a,0xae
};

void SendStartupTestData()
{
   ParsePP4Data(StartupTestData,sizeof(StartupTestData),false);
}

