/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <stdbool.h>

uint16_t next_user_id = 1;
#define MAX_IDS 100
bool id_used[MAX_IDS + 1] = { false }; // ID 1 à 100 (index 0 inutilisé)
volatile bool system_busy = false;


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Reset_Fingerprint_Sensor(void);
void UART_SendCommand(uint8_t *cmd, uint8_t len);
int UART_ReadResponse(uint8_t *buffer, uint8_t len);
void PrintToPuTTY(char *msg);
bool Add_Fingerprint_3Steps(uint16_t user_id);
uint16_t Compare_Fingerprint(void);
bool Read_Valid_Frame(uint8_t *buffer, uint32_t timeout);
uint16_t Get_First_Free_ID(void);
void Delete_All_Users(void);
uint16_t Compare_Fingerprint_With_Timeout(uint32_t timeout_ms);
bool Is_Fingerprint_Registered(uint16_t id);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

GPIO_PinState last_ajout_btn = GPIO_PIN_SET;
GPIO_PinState last_suppr_btn = GPIO_PIN_SET;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  Reset_Fingerprint_Sensor();
 HAL_Delay(1000);

Delete_All_Users();
HAL_Delay(1000);

 if (!Is_Fingerprint_Registered(1))  // si admin non encore enregistré
 {
     PrintToPuTTY("🛡️ Enregistrement de l’admin (ID=1)...\r\n");

     if (Add_Fingerprint_3Steps(1))
     {
         id_used[1] = true;
         PrintToPuTTY("✅ Admin enregistré.\r\n");
         LED_Green_On();
         HAL_Delay(2000);
         LED_Green_Off();
     }
     else
     {
         PrintToPuTTY("❌ Échec .\r\n");
         LED_Red_On();
         HAL_Delay(2000);
         LED_Red_Off();
     }
 }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (system_busy) continue; // Ne fait rien si une opération est en cours

        // --- Bouton Ajout (PA7) ---
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET)
        {
            HAL_Delay(50); // anti-rebond
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET)
            {
                system_busy = true;

                PrintToPuTTY(">> Mode ajout\r\n");
                PrintToPuTTY(">> Authentification admin requise...\r\n");

                uint16_t admin_id = Compare_Fingerprint();
                HAL_Delay(500);
                if (admin_id == 1)
                {
                    PrintToPuTTY("✅ Admin reconnu\r\n");

                    uint16_t new_id = Get_First_Free_ID();
                    if (new_id == 0xFFFF)
                    {
                        PrintToPuTTY("❌ Aucun ID disponible.\r\n");
                    }
                    else
                    {
                        Add_Fingerprint_3Steps(new_id); // ⚠️ Fonction qui demande à poser le doigt 3 fois
                    }
                }
                else
                {
                    PrintToPuTTY("❌ Accès refusé : seul l'admin peut ajouter une empreinte\r\n");
                }

                while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET); // attendre relâchement
                system_busy = false;
            }
        }

        // --- Bouton Suppression (PA6) ---
        else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
        {
            HAL_Delay(50); // anti-rebond
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET)
            {
                system_busy = true;

                PrintToPuTTY(">> Mode suppression\r\n");
                PrintToPuTTY(">> Authentification admin requise...\r\n");

                uint16_t admin_id = Compare_Fingerprint();
                HAL_Delay(500);
                if (admin_id == 1)
                {
                    PrintToPuTTY("✅ Admin reconnu\r\n");
                    PrintToPuTTY(">> Posez l'empreinte à supprimer...\r\n");

                    uint16_t id = Compare_Fingerprint();
                    if (id != 0xFFFF)
                    {
                        Delete_Fingerprint_By_ID(id);
                    }
                    else
                    {
                        PrintToPuTTY("❌ Empreinte inconnue, suppression échouée\r\n");
                    }
                }
                else
                {
                    PrintToPuTTY("❌ Accès refusé : seul l'admin peut supprimer une empreinte\r\n");
                }

                while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET); // attendre relâchement
                system_busy = false;
            }
        }

        // --- Mode comparaison automatique ---
        else
        {
            uint16_t id = Compare_Fingerprint_With_Timeout(2000);
           if (id != 0xFFFF)
            {
                PrintToPuTTY("✅ Empreinte reconnue !\r\n");
                char msg[32];
                sprintf(msg, "ID = %d\r\n", id);
                PrintToPuTTY(msg);
                LED_Green_On();
                HAL_Delay(1000);
                LED_Green_Off();
            }
        }
    }

}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LED_Green_On(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); }
void LED_Green_Off(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); }
void LED_Red_On(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); }
void LED_Red_Off(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); }

 void PrintToPuTTY(char *msg)
 {
 while (CDC_Transmit_FS((uint8_t *)msg, strlen(msg)) == USBD_BUSY)
 {
 HAL_Delay(1);
 }
 }

 void Reset_Fingerprint_Sensor(void)
 {
	  PrintToPuTTY("🔄 Reset du capteur en cours...\r\n");
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
     HAL_Delay(50);

     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
     HAL_Delay(200);
     PrintToPuTTY("✅ Reset du capteur terminé.\r\n");
 }

 bool Add_Fingerprint_3Steps(uint16_t user_id)
 {
     char msg[100];

     for (int step = 1; step <= 3; step++)
     {
         sprintf(msg, "📤 Éssai %d.\r\n", step);
         PrintToPuTTY(msg);
         HAL_Delay(800);

         // Vide le buffer UART
         uint8_t dummy;
         while (HAL_UART_Receive(&huart1, &dummy, 1, 10) == HAL_OK);

         // Construction de la trame pour cette étape
         uint8_t cmd[8] = {
             0xF5,
             0x01 + step - 1,
             (user_id >> 8) & 0xFF,
             user_id & 0xFF,
             0x01, 0x00, 0x00, 0xF5
         };
         cmd[6] = cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5];

         // Envoi de la commande d'enregistrement
         HAL_UART_Transmit(&huart1, cmd, 8, 1000);
         HAL_Delay(100);

         // Réception
         uint8_t response[8] = {0};
         if (HAL_UART_Receive(&huart1, response, 8, 5000) == HAL_OK)
         {
             sprintf(msg, "📥 Réponse (%d): %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                     step, response[0], response[1], response[2], response[3],
                     response[4], response[5], response[6], response[7]);

             uint8_t checksum = response[1] ^ response[2] ^ response[3] ^ response[4] ^ response[5];

             if (response[0] == 0xF5 &&
                 response[1] == cmd[1] &&
                 response[6] == checksum &&
                 response[7] == 0xF5 &&
                 response[4] == 0x00)
             {
                 PrintToPuTTY("✅ Étape réussie\r\n");

                 // ✅ Allume LED VERTE STM32
                 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                 HAL_Delay(500);
                 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

                 HAL_Delay(1000); // Attente avant prochaine étape
             }
             else
             {
                 PrintToPuTTY("❌ Échec, recommencez\r\n");
                 HAL_Delay(1500);
                 step--; // Répéter cette étape
             }
         }
         else
         {
             HAL_Delay(1000);
             step--; // Répéter cette étape
         }

         HAL_Delay(800); // Pause entre étapes
     }

     PrintToPuTTY("🎉 Empreinte enregistrée avec succès !\r\n");
     return true;
 }



 uint16_t Get_First_Free_ID(void)
 {
     for (uint16_t i = 1; i <= MAX_IDS; i++)
     {
         if (!id_used[i])
             return i;
     }
     return 0xFFFF; // Aucun ID libre
 }

 uint16_t Compare_Fingerprint(void)
 {
     uint8_t cmd[8] = {0xF5, 0x0C, 0, 0, 0, 0, 0, 0xF5};
     cmd[6] = cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5];

     HAL_UART_Transmit(&huart1, cmd, 8, 1000);
     HAL_Delay(100);

     PrintToPuTTY("🔍 Démarrage de la comparaison d’empreinte...\r\n");

     uint8_t frame[8] = {0};

     if (Read_Valid_Frame(frame, 3000))
     {
         char msg[100];
         sprintf(msg, "📥 Trame reçue : %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 frame[0], frame[1], frame[2], frame[3],
                 frame[4], frame[5], frame[6], frame[7]);


         switch (frame[5])
         {
             case 0x00:
             {
                 uint16_t uid = (frame[2] << 8) | frame[3];
                 if (uid == 0)
                 {
                     PrintToPuTTY("❌ Empreinte détectée mais non enregistrée.\r\n");
                     LED_Red_On();
                     HAL_Delay(2000);
                     LED_Red_Off();
                     return 0;
                 }
                 else
                 {
                     sprintf(msg, "✅ Empreinte reconnue : ID=%d, Priv=%d\r\n", uid, frame[4]);
                     PrintToPuTTY(msg);
	                 LED_Green_On();
	                 HAL_Delay(2000);
	                 LED_Green_Off();
                     return uid;
                 }
             }
             case 0x01:
                 PrintToPuTTY("❌ Empreinte non reconnue.\r\n");
                 return 0;
             case 0x02:
                 PrintToPuTTY("⚠️ Aucun utilisateur trouvé.\r\n");
                 return 0;
             case 0x03:
                 PrintToPuTTY("⚠️ Temps d’attente dépassé.\r\n");
                 return 0;
             default:
                 PrintToPuTTY("⚠️ Code retour inconnu.\r\n");
                 return 0;
         }
     }
     else
     {
         PrintToPuTTY("⏱️ Timeout ou trame invalide.\r\n");
         return 0;
     }
 }


 void Delete_All_Users(void)
 {
     uint8_t cmd[8] = {0xF5, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF5};

     // Calcul du checksum (XOR de cmd[1] à cmd[5])
     cmd[6] = cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5];

     HAL_UART_Transmit(&huart1, cmd, 8, 1000);

     uint8_t resp[8] = {0};
     PrintToPuTTY("🗑️ Suppression de toutes les empreintes...\r\n");

     if (HAL_UART_Receive(&huart1, resp, 8, 3000) == HAL_OK)
     {
         char msg[128];
         sprintf(msg, "📥 Réponse : %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 resp[0], resp[1], resp[2], resp[3],
                 resp[4], resp[5], resp[6], resp[7]);


         uint8_t checksum = resp[1] ^ resp[2] ^ resp[3] ^ resp[4] ^ resp[5];

         if (resp[0] == 0xF5 && resp[1] == 0x05 && resp[7] == 0xF5 && resp[6] == checksum)
         {
             if (resp[4] == 0x00) // ACK_SUCCESS
             {
                 PrintToPuTTY("✅ Toutes les empreintes ont été supprimées !\r\n");
             }
             else
             {
                 PrintToPuTTY("❌ Échec de la suppression.\r\n");
             }
         }
         else
         {
             PrintToPuTTY("❌ Réponse invalide (format ou checksum).\r\n");
         }
     }
     else
     {
         PrintToPuTTY("⏱️ Timeout - pas de réponse du capteur.\r\n");
     }
 }

 bool Read_Valid_Frame(uint8_t *buffer, uint32_t timeout_ms)
 {
     uint32_t start_tick = HAL_GetTick();
     uint8_t idx = 0;

     while (idx < 8)
     {
         if (HAL_UART_Receive(&huart1, &buffer[idx], 1, 10) == HAL_OK)
         {
             idx++;
         }
         else
         {
             if ((HAL_GetTick() - start_tick) > timeout_ms)
                 return false; // timeout global dépassé
         }
     }

     // Vérifications comme avant
     if (buffer[0] != 0xF5 || buffer[7] != 0xF5)
         return false;

     uint8_t checksum = buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
     if (buffer[6] != checksum)
         return false;

     return true;
 }



 void Delete_Fingerprint_By_ID(uint16_t id)
 {
     char msg[100];

     // ⚠️ Commande 0x04 pour suppression spécifique (PAS 0x05)
     uint8_t cmd[8] = {0xF5, 0x04, id >> 8, id & 0xFF, 0x00, 0x00, 0x00, 0xF5};
     cmd[6] = cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5];

     sprintf(msg, "📤 Suppression de l’empreinte ID=%d...\r\n", id);


     HAL_UART_Transmit(&huart1, cmd, 8, 1000);

     uint8_t frame[8] = {0};

     if (Read_Valid_Frame(frame, 1000))
     {
    	 if (frame[1] == 0x04 && frame[4] == 0x00) {
    	     PrintToPuTTY("✅ Suppression confirmée : empreinte effacée ✅\r\n");
    	     id_used[id] = false;  // <-- AJOUT ICI
    	     return;
         }
     }

     // 🔍 Vérification de suppression par comparaison
     PrintToPuTTY("🔍 Vérification de suppression par comparaison...\r\n");

     uint8_t compare_cmd[8] = {0xF5, 0x0C, 0, 0, 0, 0, 0, 0xF5};
     compare_cmd[6] = compare_cmd[1] ^ compare_cmd[2] ^ compare_cmd[3] ^ compare_cmd[4] ^ compare_cmd[5];
     HAL_UART_Transmit(&huart1, compare_cmd, 8, 1000);
     HAL_Delay(100);

     if (Read_Valid_Frame(frame, 1000))
     {
         sprintf(msg, "📥 Vérification : %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 frame[0], frame[1], frame[2], frame[3],
                 frame[4], frame[5], frame[6], frame[7]);


         if (frame[1] == 0x0C && frame[4] == 0x00 && ((frame[2] << 8) | frame[3]) == id)
         {
             PrintToPuTTY("❌ Suppression échouée : empreinte toujours présente.\r\n");
         }
         else
         {
             PrintToPuTTY("✅ Aucune empreinte reconnue : suppression réussie ✅\r\n");
         }
     }
     else
     {
         PrintToPuTTY("⏱️ Timeout lors de la suppression. Aucune confirmation reçue.\r\n");
     }
 }

 uint16_t Compare_Fingerprint_With_Timeout(uint32_t timeout_ms)
 {
     uint8_t cmd[8] = {0xF5, 0x0C, 0, 0, 0, 0, 0, 0xF5};
     cmd[6] = cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4] ^ cmd[5];

     HAL_UART_Transmit(&huart1, cmd, 8, 100);

     uint8_t frame[8] = {0};
     uint32_t start_time = HAL_GetTick();

     while ((HAL_GetTick() - start_time) < timeout_ms)
     {
         if (HAL_UART_Receive(&huart1, frame, 8, 100) == HAL_OK)
         {
             if (frame[0] == 0xF5 && frame[7] == 0xF5)
             {
                 uint8_t checksum = frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5];
                 if (frame[6] == checksum && frame[1] == 0x0C)
                 {
                     if (frame[5] == 0x00) // ACK_SUCCESS
                     {
                         uint16_t uid = (frame[2] << 8) | frame[3];

                         if (uid == 0)
                         {
                             // 🟥 Empreinte détectée mais non enregistrée
                             LED_Red_On();
                             HAL_Delay(1000);
                             LED_Red_Off();
                             return 0xFFFF;
                         }
                         else
                         {
                             // 🟩 Empreinte reconnue
                             LED_Green_On();
                             HAL_Delay(1000);
                             LED_Green_Off();
                             return uid;
                         }
                     }
                     else
                     {
                         // Code retour ≠ 0x00 (non reconnu ou autre)
                         return 0xFFFF;
                     }
                 }
             }
         }

         // Si un bouton est appuyé, annule la comparaison
         if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET ||
             HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET)
         {
             return 0xFFFF;
         }
     }

     // Timeout : aucune empreinte détectée ⇒ pas de LED
     return 0xFFFF;
 }

 bool Is_Fingerprint_Registered(uint16_t id)
 {
     uint8_t tx_buffer[8] = {
         0xF5, 0x0F,
         (uint8_t)(id >> 8), (uint8_t)(id & 0xFF),
         0x00, 0x00, 0x00, 0xF5
     };
     tx_buffer[6] = tx_buffer[1] ^ tx_buffer[2] ^ tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5];

     HAL_UART_Transmit(&huart1, tx_buffer, 8, HAL_MAX_DELAY);

     uint8_t rx_buffer[8] = {0};
     if (HAL_UART_Receive(&huart1, rx_buffer, 8, 1000) == HAL_OK)
     {
         return (rx_buffer[4] == 0x00); // 0x00 = ID existe
     }

     return false;
 }


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}
  /* USER CODE END Error_Handler_Debug */


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
