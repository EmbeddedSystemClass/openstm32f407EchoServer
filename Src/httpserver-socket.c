/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Socket_RTOS/Src/httpserver-socket.c
  * @author  MCD Application Team
  * @version V1.3.1
  * @date    09-October-2015  
  * @brief   Basic http server implementation using LwIP socket API   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "fs.h"
#include "fsdata.h"
#include "string.h"
#include "httpserver-socket.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Format of dynamic web page: the page header */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief serve tcp connection  
  * @param conn: connection socket 
  * @retval None
  */
void voltage_server(int conn)
{
  int buflen = 1500;
  unsigned char recv_buffer[1500];
  uint16_t voltage = getVoltage();

				
  /* Read in the request */
//  read(conn, recv_buffer, buflen);

  int goodWrite = 1;
  while(goodWrite)
  {
	  voltage = getVoltage();
	  goodWrite = send(conn, (uint16_t *)&voltage, (size_t)sizeof(uint16_t), 1);
	  osDelay(100);
  }

  /* Close connection socket */
  close(conn);
}

/**
  * @brief  http server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
void voltage_server_socket()
{
  int sock, newconn, size;
  struct sockaddr_in address, remotehost;

 /* create a TCP socket */
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
  {
    return;
  }
  
  /* bind to port 80 at any interface */
  address.sin_family = AF_INET;
  address.sin_port = htons(80);
  address.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0)
  {
    return;
  }
  
  /* listen for incoming connections (TCP listen backlog = 5) */
  listen(sock, 5);
  
  size = sizeof(remotehost);
  
  while (1) 
  {
    newconn = accept(sock, (struct sockaddr *)&remotehost, (socklen_t *)&size);
    voltage_server(newconn);
  }
}

/**
/**
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
