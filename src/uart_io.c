/*
 *  Copyright (C) 2021  Skip Hansen
 * 
 *  Code derived from Public Domain UART events example from the espidf.
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
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#define TAG __FUNCTION__

#define UART_NUM UART_NUM_0
#define RD_BUF_SIZE  32
// NB: BUF_SIZE must be > 128 or uart_driver_install() will fail!
#define BUF_SIZE     200

static uint8_t TempBuf[RD_BUF_SIZE];
static QueueHandle_t uart0_queue;
int ParsePP4Data(uint8_t *Data,int DataLen,bool bSerialPort);

static void uart_event_task(void *pvParameters)
{
   uart_event_t event;
   int BytesRead;
   int Resp;

   for(;;) {
      //Waiting for UART event.
      if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
         ESP_LOGI(TAG, "uart[%d] event:",UART_NUM);
         switch(event.type) {
            case UART_DATA:
               ESP_LOGI(TAG,"[UART DATA]: %d",event.size);
               BytesRead = uart_read_bytes(UART_NUM,TempBuf,event.size,
                                           portMAX_DELAY);
               if(BytesRead > 0) {
                  Resp = ParsePP4Data(TempBuf,BytesRead,true);
                  if(Resp == '0') {
                  // Transmssion complete, no error send ACK
                     char Ack = 'A';
                     uart_write_bytes(UART_NUM,&Ack,sizeof(Ack));
                  }
               }
               break;

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
               if(event.type == UART_FIFO_OVF) {
                  ESP_LOGI(TAG,"hw fifo overflow");
               }
               else {
                  ESP_LOGI(TAG,"ring buffer full");
               }
               uart_flush_input(UART_NUM);
               xQueueReset(uart0_queue);
               break;

            default:
               ESP_LOGI(TAG, "uart event type: %d", event.type);
               break;
         }
      }
   }
   vTaskDelete(NULL);
}

esp_err_t uart_io_init()
{
   esp_err_t Err;
   int Line = 0;
   /* Configure parameters of an UART driver,
    * communication pins and install the driver */
   uart_config_t uart_config = {
      .baud_rate = 57600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
   };
   //Install UART driver, and get the queue.

   do {
      if((Err = uart_driver_delete(UART_NUM)) != ESP_OK) {
         Line = __LINE__;
         break;
      }

      Err = uart_driver_install(UART_NUM,BUF_SIZE,0,20,&uart0_queue,0);
      if(Err != ESP_OK) {
         Line = __LINE__;
         break;
      }

      if((Err = uart_param_config(UART_NUM,&uart_config)) != ESP_OK) {
         Line = __LINE__;
         break;
      }

      Err = uart_set_pin(UART_NUM,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
      if(Err != ESP_OK) {
         Line = __LINE__;
      }
   } while(false);

   if(Err != ESP_OK) {
      ESP_LOGE(TAG,"Err %d on line %d",Err,Line);
   }
   else {
      //Create a task to handler UART event from ISR
      xTaskCreate(uart_event_task,"uart_event_task",2048,NULL,12,NULL);
   }

   return Err;
}

