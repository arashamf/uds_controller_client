/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "lwip.h"

#include "iwdg.h"
#include "tim.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "gpio.h"
#include "myWeb.h"
#include "st7735.h"
#include "rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t IP_ADDRESS[4]; //установленный ip-адрес в виде четырёх uint8_t (lwip.c)  
extern char UART3_msg_TX [RS232_BUFFER_SIZE];
char lcd_buffer [50]; //буффер с конвертированными данными времени в строковом отображении

const unsigned char httpHeader[] = "HTTP/1.1" ;  // HTTP header
const unsigned char httpMethod[] = "GET";
const unsigned char state[] = "/?command=port00_state";
const unsigned char httpMimeTypeHTML[] = "text/html" ;              // HTML MIME type
const unsigned char httpMimeTypeScript[] = "text/plain" ;           // TEXT MIME type
const unsigned char httpTypeConnection[] = "Connection: keep-alive" ; 
const unsigned char serverIP [] = { 192, 168, 7, 13};
	
char http_send_buffer [100]; //буффер, в который записывается сформированный http-запрос

osMutexId mutex_RS485_Handle; //мьютекс блокировки передачи команд ячейкам

osMessageQId RS485_msg_Queue;

osTimerId osProgTimerIWDG;  //программный таймер перезагружающий сторожевик
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void send_thread (void * argument);
void recv_thread (void * argument);
void ProgTimerIWDGCallback(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  osMutexDef (mutex_RS485); 
	mutex_RS485_Handle = osMutexCreate(osMutex (mutex_RS485));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
   osTimerDef (TimerIWDG, ProgTimerIWDGCallback);
	osProgTimerIWDG = osTimerCreate(osTimer (TimerIWDG), osTimerPeriodic, NULL);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQDef (RS485_msg_Queuename, 2, uint8_t *);
	RS485_msg_Queue = osMessageCreate (osMessageQ (RS485_msg_Queuename), NULL); //очередь передачи полученного по RS-485 сообщения
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
	osTimerStart(osProgTimerIWDG, 2000); //запуск циклического таймера
	struct netconn *conn; //указатель на структуру соединения
  err_t err; //переменная ошибки
  ip_addr_t ServerIPaddr;
  IP4_ADDR(&ServerIPaddr, 192, 168, 7, 247); //запись ip-адреса http-сервера
	u16_t buflen = 0;
	
  if((conn = netconn_new(NETCONN_TCP)) != NULL)
  {
    if ((err = netconn_bind(conn, NULL, 23006)) == ERR_OK) //привязка нового netconn-соединения к IP-адресу и порту
    {
      if ((err = netconn_connect(conn, &ServerIPaddr, TCP_PORT)) == ERR_OK) //установка соединения с сервером
      {
				ClearLcdMemory();
				sprintf (lcd_buffer, "get_connect");
				LCD_ShowString(2, 2 , lcd_buffer);
				LCD_Refresh();
 //       sys_thread_new("send_thread", send_thread, (void*)&conn, 512, osPriorityNormal );
				buflen = sprintf (http_send_buffer, "%s %s %s\r\nHost: %u.%u.%u.%u:%u\r\nContent-Type:%s\r\n", httpMethod, state, httpHeader, 
				serverIP[0], serverIP[1], serverIP[2], serverIP[3], TCP_PORT, httpMimeTypeHTML);
				if((err = netconn_write(conn, (void *)http_send_buffer, buflen, NETCONN_NOCOPY)) == ERR_OK)
				{
					ClearLcdMemory();
					sprintf (lcd_buffer, "%s %u", http_send_buffer, buflen);
					LCD_ShowString(2, 2 , lcd_buffer);
					LCD_Refresh();
				}
        sys_thread_new("recv_thread", recv_thread, (void*)&conn, DEFAULT_THREAD_STACKSIZE, osPriorityNormal );
      }
    }
    else
    {
      netconn_delete(conn);
    }
  }
  /* Infinite loop */
  for(;;)
  {
		LED_RED(1)
    osDelay(500);
		LED_RED(0);
		osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/************************************************************************************************/
void send_thread(void *arg)
{
	err_t send_err;
	struct netconn *conn; //указатель на структуру соединения
	u16_t buflen;

	buflen = sprintf (http_send_buffer, "%s %s %s\r\nHost: %u.%u.%u.%u:%u\r\n", httpMethod, state, httpHeader, serverIP[0], serverIP[1], serverIP[2], serverIP[3], TCP_PORT);
  for(;;)
  {
		ClearLcdMemory();
  	if((send_err = netconn_write(conn, (void *)http_send_buffer, buflen, NETCONN_NOCOPY)) == ERR_OK)
  	{
			sprintf (lcd_buffer, "%s %u", http_send_buffer, buflen);
			LCD_ShowString(2, 2 , lcd_buffer);
  	}
		else
		{
			sprintf (lcd_buffer, "error=%u", send_err);
			LCD_ShowString(2, 2 , lcd_buffer);
		}
		LCD_Refresh();
    osDelay(1000);
  }
}

/************************************************************************************************/
void recv_thread(void *arg)
{
	err_t recv_err;
  struct netconn *conn;
  struct netbuf *inbuf;
  uint8_t* buf;
  u16_t buflen;
  for(;;)
  {
  	
  	if ((recv_err = netconn_recv(conn, &inbuf)) == ERR_OK)
  	{
  	  if (netconn_err(conn) == ERR_OK)
  	  {
  	    netbuf_data(inbuf, (void**)&buf, &buflen);
				sprintf (UART3_msg_TX,"%s\r\n", buf);
				UART3_SendString ((char*)UART3_msg_TX);
  	    netbuf_delete(inbuf);
  	  }
  	}
		osDelay(1000);
  }
}

/************************************************************************************************/
void ProgTimerIWDGCallback(void const *argument)
{
	HAL_IWDG_Refresh(&hiwdg); //перезагрузка iwdg
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
