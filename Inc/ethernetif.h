/**
 ******************************************************************************
  * File Name          : ethernetif.h
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  ******************************************************************************
  */
  

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"
#include "cmsis_os.h"

/* Exported types ------------------------------------------------------------*/
/* Structure that include link thread parameters */
struct link_str {
  struct netif *netif;
  osSemaphoreId semaphore;
};

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

struct ptptime_t {
  s32_t tv_sec;
  s32_t tv_nsec;
};
/* USER CODE END 0 */

/* Exported functions ------------------------------------------------------- */
err_t ethernetif_init(struct netif *netif);

void ethernetif_input( void const * argument ); 
 

void ethernetif_set_link(void const *argument);
void ethernetif_update_config(struct netif *netif);
void ethernetif_notify_conn_changed(struct netif *netif);

/* USER CODE BEGIN 1 */
void ETH_PTPTime_SetTime(ETH_HandleTypeDef * heth, struct ptptime_t * timestamp);
void ETH_PTPTime_GetTime(ETH_HandleTypeDef * heth, struct ptptime_t * timestamp);

void ETH_PTPTime_UpdateOffset(ETH_HandleTypeDef * heth, struct ptptime_t * timeoffset);
void ETH_PTPTime_AdjFreq(ETH_HandleTypeDef * heth, int32_t Adj);

  /* Examples of subsecond increment and addend values using SysClk = 72 MHz

   Addend * Increment = 2^63 / SysClk

    ptp_tick = Increment * 10^9 / 2^31

   +-----------+-----------+------------+
   | ptp tick  | Increment | Addend     |
   +-----------+-----------+------------+
   |  119 ns   |   255     | 0x1DF170C7 |
   |  100 ns   |   215     | 0x238391AA |
   |   50 ns   |   107     | 0x475C1B20 |
   |   20 ns   |    43     | 0xB191D856 |
   |   14 ns   |    30     | 0xFE843E9E |
   +-----------+-----------+------------+
  */

#define ADJ_FREQ_BASE_ADDEND      0xFE843E9E
#define ADJ_FREQ_BASE_INCREMENT   30

/* USER CODE END 1 */
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
