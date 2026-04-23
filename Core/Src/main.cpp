#include "main.hpp"
#include "cmsis_os.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "string.h"
#include "message.h"
#include "consol_logger.hpp"
extern "C"
{
#include "app_fatfs.h"
}

//#define SIMU_ENABLED
#define SENDER_ENABLED

osThreadId_t readerTaskHandle;
const osThreadAttr_t readerTask_attributes = {
  .name = "ReaderTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal
};
osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes = {
  .name = "LoggerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal
};
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1
};
osThreadId_t IRQsimuTaskHandle;
const osThreadAttr_t IRQsimuTask_attributes = {
  .name = "IRQsimuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh
};
osMessageQueueId_t messageQueueHandle;
const osMessageQueueAttr_t messageQueue_attributes = {
  .name = "messageQueue"
};
osTimerId_t timer10msHandle;
const osTimerAttr_t timer10ms_attributes = {
  .name = "timer10ms"
};

const uint32_t FDCAN1_FIFO0{1U << 0};
const uint32_t FDCAN1_FIFO1{1U << 1};
const uint32_t FDCAN2_FIFO0{1U << 2};
const uint32_t FDCAN2_FIFO1{1U << 3};

void SystemClock_Config();
void StartLoggerTask(void *argument);
void StartReaderTask(void *argument);
void StartMonitorTask(void *argument);
void StartIRQsimuTask(void *argument);
void Timer10msCallback(void *argument);

const char* getString(const FRESULT fr);
const char* getString(const osThreadState_t fr);
uint32_t getField(const uint16_t LSB, const uint16_t sizeInBits, const uint8_t *buf);
void switchLEDs();
void createLog();
void renameLog();
void updateDateAndTime(const uint32_t ID, const uint8_t *data);
void saveData(const uint32_t timestamp, const uint32_t ID, const bool isExtended, const char *dir, const uint8_t channelID, const uint8_t filterID, const uint8_t length, const uint8_t *payload);
void diagnosticController(const uint32_t timestamp, uint32_t ID, bool isExtended, const char *dir, const uint8_t channelID, const uint8_t filterID, uint8_t length, uint8_t *payload);
void processMessage(CMessage& message);
bool readOutMessages(FDCAN_HandleTypeDef *hfdcan, const uint32_t RxLocation);
template <typename... Ts>
void trace(FIL& logFile, const char *value, Ts... values);
template <typename... Ts>
void log(const char *value, Ts... values);

const char defaultLogName[] = {"LOG.CSV"};
const char systemLogName[] = {"DEBUG.TXT"};
FIL systemLogFile;
volatile bool toFlushData{false};
volatile bool isDateReady{false};
volatile bool isReadyToSend{false};
volatile bool isLogRenamed{false};
volatile uint16_t hour{0}, minute{0}, second{0}, day{1}, month{1}, year{26};

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    if (MX_FATFS_Init() != APP_OK)
    {
        Error_Handler();
    }
    MX_LPUART1_UART_Init();
    HAL_Delay(100);
    MX_FDCAN1_Init();
    HAL_Delay(100);
    MX_FDCAN2_Init();
    HAL_Delay(100);

#ifdef DEBUG
    printf("\r\n\r\n\r\n  DEBUG  \r\n\r\n\r\n");
#else
    printf("\r\n\r\n\r\n RELEASE \r\n\r\n\r\n");
#endif

    HAL_GPIO_WritePin(GPIOB, (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7), GPIO_PIN_SET);

	osKernelInitialize();

	readerTaskHandle = osThreadNew(StartReaderTask, NULL, &readerTask_attributes);
    loggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &loggerTask_attributes);
#ifdef SIMU_ENABLED
	//monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);
	IRQsimuTaskHandle = osThreadNew(StartIRQsimuTask, NULL, &IRQsimuTask_attributes);
#endif
    messageQueueHandle = osMessageQueueNew(8, sizeof(CMessage), &messageQueue_attributes);
	timer10msHandle = osTimerNew(Timer10msCallback, osTimerPeriodic, NULL, &timer10ms_attributes);
    osTimerStart(timer10msHandle, 10);

	osKernelStart();

	while (true)
	{
	    printf("\r\nYOU ARE IN THE VERY MIDDLE OF THE ASS!!!\r\n");
	}
}

bool readOutMessages(FDCAN_HandleTypeDef *hfdcan, const uint32_t RxLocation)
{
    const uint8_t channelID{static_cast<uint8_t>((hfdcan->Instance == FDCAN1) ? 1 : 2)};
    bool returnResult{false};
#ifdef SIMU_ENABLED
    uint32_t rxFIFOfillLevel{3};
#else
    uint32_t rxFIFOfillLevel{HAL_FDCAN_GetRxFifoFillLevel(hfdcan, RxLocation)};
#endif
    uint32_t queueFreeSpace{osMessageQueueGetSpace(messageQueueHandle)};
    while (
           (0 < rxFIFOfillLevel)                                                        // if there is something to read out from CANs FIFOs
           &&                                                                           // AND
           (0 < queueFreeSpace)                                                         // there is free space in message queue to store data
          )
    {
        log("readOutMessages(): RX FIFO fill Level = %d, queue free space = %d\r\n", rxFIFOfillLevel, queueFreeSpace);
        static FDCAN_RxHeaderTypeDef rxHeader;
        static uint8_t rxData[payloadMaxSize];
#ifdef SIMU_ENABLED
        bool result{true};
        --rxFIFOfillLevel;
        rxHeader.Identifier = 0x00CE;
        rxHeader.DataLength = 8;
        rxHeader.IdType = FDCAN_STANDARD_ID;
        rxHeader.FilterIndex = 123;
#else
        bool result{HAL_FDCAN_GetRxMessage(hfdcan, RxLocation, &rxHeader, rxData) == HAL_OK};
#endif
        if (result)
        {
            const uint32_t messageID{rxHeader.Identifier};
            const uint8_t dataLength{static_cast<uint8_t>(rxHeader.DataLength)};
            result = ((messageID != 0) && (dataLength != 0));
            if (result)
            {
                const uint8_t filterID{static_cast<uint8_t>(rxHeader.FilterIndex)};
                CMessage message = {
                    .timestamp = HAL_GetTick(),
                    .ID = messageID,
                    .isExtended = (rxHeader.IdType == FDCAN_EXTENDED_ID),
                    .channelID = channelID,
                    .filterID = filterID,
                    .length = dataLength
                };
                memcpy(message.payload, rxData, dataLength);
                osMessageQueuePut(messageQueueHandle, &message, 0, 0);
                queueFreeSpace = osMessageQueueGetSpace(messageQueueHandle);
#ifndef SIMU_ENABLED
                rxFIFOfillLevel = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, RxLocation);
#endif
                returnResult = true;
                log("readOutMessages(FDCAN%01d, 0x%02X): ID = 0x%04lX, size = 0x%02X:", channelID, filterID, messageID, dataLength);
                for (uint32_t i = 0; i < dataLength; ++i)
                {
                    log("%s0x%02X", ((i == 0) ? " [" : ", "), rxData[i]);
                }
                log("]\r\n");
            }
            else
            {
                log("readOutMessages(FDCAN%01d, FIFO%01d): wrong ID or data length\r\n", channelID, ((RxLocation == FDCAN_RX_FIFO0) ? 0 : 1));
            }
        }
        else
        {
            log("readOutMessages(FDCAN%01d, FIFO%01d): message read failure\r\n", channelID, ((RxLocation == FDCAN_RX_FIFO0) ? 0 : 1));
        }
    }
    log("readOutMessages(): exit, RX FIFO fill Level = %d, queue free space = %d\r\n", rxFIFOfillLevel, queueFreeSpace);
    return returnResult;
}

void StartReaderTask(void *argument)
{
    log("ReaderTask: start\r\n");
    osDelay(100);

    while (true)
    {
        // wait for any event from CAN1/CAN2 stored in FIFO0/FIFO1
        log("ReaderTask: wait for event\r\n");
        const auto eventID{osThreadFlagsWait((FDCAN2_FIFO1 | FDCAN2_FIFO0 | FDCAN1_FIFO1 | FDCAN1_FIFO0), osFlagsWaitAny, osWaitForever)};
        log("ReaderTask: eventID received = %08X\r\n", eventID);
        if ((eventID & osFlagsError) == osFlagsError)
        {
            log("ReaderTask: ERROR RECEIVED\r\n");
            osDelay(10);
        }
        else
        {
            // read out messages accordingly and put them into message queue
            bool toYield{0 == osMessageQueueGetSpace(messageQueueHandle)};
            if (((eventID & FDCAN1_FIFO0) == FDCAN1_FIFO0) && !toYield)
            {
                toYield = (readOutMessages(&hfdcan1, FDCAN_RX_FIFO0) && (0 == osMessageQueueGetSpace(messageQueueHandle)));
                log("ReaderTask: event received: FDCAN1_FIFO0, to store = %d\r\n", toYield);
            }
            if (((eventID & FDCAN1_FIFO1) == FDCAN1_FIFO1) && !toYield)
            {
                toYield = (readOutMessages(&hfdcan1, FDCAN_RX_FIFO1) && (0 == osMessageQueueGetSpace(messageQueueHandle)));
                log("ReaderTask: event received: FDCAN1_FIFO1, to store = %d\r\n", toYield);
            }
            if (((eventID & FDCAN2_FIFO0) == FDCAN2_FIFO0) && !toYield)
            {
                toYield = (readOutMessages(&hfdcan2, FDCAN_RX_FIFO0) && (0 == osMessageQueueGetSpace(messageQueueHandle)));
                log("ReaderTask: event received: FDCAN2_FIFO0, to store = %d\r\n", toYield);
            }
            if (((eventID & FDCAN2_FIFO1) == FDCAN2_FIFO1) && !toYield)
            {
                toYield = (readOutMessages(&hfdcan2, FDCAN_RX_FIFO1) && (0 == osMessageQueueGetSpace(messageQueueHandle)));
                log("ReaderTask: event received: FDCAN2_FIFO1, to store = %d\r\n", toYield);
            }
            // yield if nothing read out from FIFOs or message queue is full
            if (toYield)
            {
                log("ReaderTask: yield\r\n");
                osDelay(10);
            }
        }
    }
}

void mountSD()
{
    const FRESULT mountResult{f_mount(&USERFatFs, "/", 1)};
    if (mountResult == FR_OK)
    {
        log("uSD card mounted OK\r\n");
        HAL_GPIO_WritePin(GPIOB, (GPIO_PIN_6 | GPIO_PIN_7), GPIO_PIN_RESET);
        createLog();
    }
    else
    {
        log("uSD card mount error = %s\r\n", getString(mountResult));
    }
}

void StartLoggerTask(void *argument)
{
    log("LoggerTask: start\r\n");
    mountSD();
    osDelay(200);

    CMessage message;
    while (true)
    {
        uint32_t messagesInQueue{osMessageQueueGetCount(messageQueueHandle)};
        if (0 < messagesInQueue)
        {
            //       Time Stamp,      ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8
            // 1763113953920924,000000AA,   false, Rx,  0,  8,D4,9B,04,00,48,0D,80,B4
            log("LoggerTask: messages in queue = %d\r\n", messagesInQueue);
            if (osMessageQueueGet(messageQueueHandle, &message, 0, osWaitForever) == osOK)
            {
                log("LoggerTask: message received\r\n");
                processMessage(message);
                switchLEDs();
                if (isDateReady && !isLogRenamed)
                {
                    isLogRenamed = true;
                    renameLog();
                }
                if (toFlushData)
                {
                    toFlushData = false;
                    f_sync(&USERFile);
                }
            }
            else
            {
                log("LoggerTask: empty queue\r\n");
            }
        }
        else
        {
            log("LoggerTask: exit, messages in queue = %d\r\n", messagesInQueue);
            log("LoggerTask: yield\r\n");
            osDelay(2);
        }
    }
}

void StartMonitorTask(void *argument)
{
    log("MonitorTask: start\r\n");
    osDelay(200);

    while (true)
    {
        log("\r\n"
            "   MonitorTask: IRQ simu task status = %s\r\n"
            "   MonitorTask:  monitor task status = %s\r\n"
            "   MonitorTask:   reader task status = %s\r\n"
            "   MonitorTask:   logger task status = %s\r\n"
            "\r\n",
                getString(osThreadGetState(IRQsimuTaskHandle)),
                getString(osThreadGetState(      monitorTaskHandle)),
                getString(osThreadGetState(       readerTaskHandle)),
                getString(osThreadGetState(       loggerTaskHandle)));
        osDelay(100);
        static uint8_t counter{0};
        if (++counter == 10)
        {
            counter = 0;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        }
    }
}

void StartIRQsimuTask(void *argument)
{
    log("IRQsimuTask: start\r\n");
    osDelay(700);

    while (true)
    {
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            osThreadFlagsSet(readerTaskHandle, FDCAN1_FIFO0);
            //log("                                                       IRQsimuTask:  EVENT SENT\r\n");
            osDelay(1);
        }
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            osThreadFlagsSet(readerTaskHandle, FDCAN1_FIFO1);
            //log("                                                       IRQsimuTask:  EVENT SENT\r\n");
            osDelay(1);
        }
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            osThreadFlagsSet(readerTaskHandle, FDCAN2_FIFO0);
            //log("                                                       IRQsimuTask:  EVENT SENT\r\n");
            osDelay(1);
        }
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            osThreadFlagsSet(readerTaskHandle, FDCAN2_FIFO1);
            //log("                                                       IRQsimuTask:  EVENT SENT\r\n");
            osDelay(1);
        }
        osDelay(1);
    }
}

void Timer10msCallback(void *argument)
{
    static const uint16_t msCounterToSendThreshold{1};                        // period in ms to send requests
    static const uint16_t msCounterToFlushThreshold{100};                     // period in ms to flush data
    static uint16_t msCounterToSend{0};
    static uint16_t msCounterToFlush{0};
    if (msCounterToSendThreshold <= ++msCounterToSend)
    {
        msCounterToSend = 0;
        isReadyToSend = true;
    }
    if (msCounterToFlushThreshold <= ++msCounterToFlush)
    {
        msCounterToFlush = 0;
        toFlushData = true;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        log("HAL_TIM_PeriodElapsedCallback: toFlushData\r\n");
    }
}

void SystemClock_Config(void)
{
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

const char* getString(const FRESULT fr)
{
    switch (fr)
    {
        case FR_OK: return "(0) Succeeded";
        case FR_DISK_ERR: return "(1) A hard error occurred in the low level disk I/O layer";
        case FR_INT_ERR: return "(2) Assertion failed";
        case FR_NOT_READY: return "(3) The physical drive cannot work";
        case FR_NO_FILE: return "(4) Could not find the file";
        case FR_NO_PATH: return "(5) Could not find the path";
        case FR_INVALID_NAME: return "(6) The path name format is invalid";
        case FR_DENIED: return "(7) Access denied due to prohibited access or directory full";
        case FR_EXIST: return "(8) Access denied due to prohibited access";
        case FR_INVALID_OBJECT: return "(9) The file/directory object is invalid";
        case FR_WRITE_PROTECTED: return "(10) The physical drive is write protected";
        case FR_INVALID_DRIVE: return "(11) The logical drive number is invalid";
        case FR_NOT_ENABLED: return "(12) The volume has no work area";
        case FR_NO_FILESYSTEM: return "There is no valid FAT volume";
        case FR_MKFS_ABORTED: return "(14) The f_mkfs() aborted due to any problem";
        case FR_TIMEOUT: return "(15) Could not get a grant to access the volume within defined period";
        case FR_LOCKED: return "(16) The operation is rejected according to the file sharing policy";
        case FR_NOT_ENOUGH_CORE: return "(17) LFN working buffer could not be allocated";
        case FR_TOO_MANY_OPEN_FILES: return "(18) Number of open files > _FS_LOCK";
        case FR_INVALID_PARAMETER: return "(19) Given parameter is invalid";
        default: return "";
    }
}

const char* getString(const osThreadState_t ts)
{
    switch (ts)
    {
        case osThreadError: return "(-1) Error";
        case osThreadInactive: return "(0) Inactive";
        case osThreadReady: return "(1) Ready";
        case osThreadRunning: return "(2) Running";
        case osThreadBlocked: return "(3) Blocked";
        case osThreadTerminated: return "(4) Terminated";
        default: return "";
    }
}

uint32_t getField(const uint16_t LSB, const uint16_t sizeInBits, const uint8_t *buf)
{
    const uint16_t bytesToShift{static_cast<uint16_t>(LSB >> 3)};
    const uint8_t bitsToShift{static_cast<uint8_t>(LSB & 0x07)};
    const uint16_t sizeInBytes{static_cast<uint16_t>((sizeInBits >> 3) + (((sizeInBits & 0x07) != 0) ? 1 : 0))};
    uint32_t result{0};
    for (uint16_t i = 0; i < sizeInBytes; ++i)
    {
        result |= (buf[bytesToShift + i] << (8 * i));
    }
    result >>= bitsToShift;
    const uint32_t mask{static_cast<uint32_t>((1 << sizeInBits) - 1)};
    result &= mask;
    return result;
}

void switchLEDs()
{
    static uint8_t messageCounter{0};
    HAL_GPIO_WritePin(GPIOB,                    (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5), GPIO_PIN_SET  );
    HAL_GPIO_WritePin(GPIOB, ++messageCounter & (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5), GPIO_PIN_RESET);
}

void createLog()
{
    FRESULT result;

    result = f_open(&systemLogFile, systemLogName, (FA_READ | FA_WRITE | FA_OPEN_APPEND));
    if (result == FR_OK)
    {
        log("%s created OK\r\n", systemLogName);
        trace(systemLogFile, "%08d: %s created OK\n", HAL_GetTick(), systemLogName);
    }
    else
    {
        log("%s creation error = %s\r\n", systemLogName, getString(result));
        trace(systemLogFile, "%08d: %s creation error = %s\n", HAL_GetTick(), systemLogName, getString(result));
    }

    result = f_open(&USERFile, defaultLogName, (FA_READ | FA_WRITE | FA_OPEN_APPEND));            // FA_CREATE_ALWAYS to recreate LOG.CSV
    if (result == FR_OK)
    {
        log("%s created OK\r\n", defaultLogName);
        trace(systemLogFile, "%08d: %s created OK\n", HAL_GetTick(), defaultLogName);
        f_printf(&USERFile, "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n");
        f_sync(&USERFile);
    }
    else
    {
        log("%s creation error = %s\r\n", defaultLogName, getString(result));
        trace(systemLogFile, "%08d: %s creation error = %s\n", HAL_GetTick(), defaultLogName, getString(result));
    }
}

void renameLog()
{
    char FileName[] = {"DD_MM_YY/hh_mm_ss.CSV"};
    FileName[ 0] = '0' + (day / 10);
    FileName[ 1] = '0' + (day % 10);
    FileName[ 3] = '0' + (month / 10);
    FileName[ 4] = '0' + (month % 10);
    FileName[ 6] = '0' + (year / 10);
    FileName[ 7] = '0' + (year % 10);
    FileName[ 9] = '0' + (hour / 10);
    FileName[10] = '0' + (hour % 10);
    FileName[12] = '0' + (minute / 10);
    FileName[13] = '0' + (minute % 10);
    FileName[15] = '0' + (second / 10);
    FileName[16] = '0' + (second % 10);
    FileName[8] = 0;
    DIR targetDirectory;
    FRESULT result{f_opendir(&targetDirectory, FileName)};
    if (result == FR_OK)
    {
        log("%s folder opened OK\r\n", FileName);
        trace(systemLogFile, "%08d: %s folder opened OK\n", HAL_GetTick(), FileName);
    }
    else
    {
        log("%s folder open error = %s\r\n", FileName, getString(result));
        trace(systemLogFile, "%08d: %s folder open error = %s\n", HAL_GetTick(), FileName, getString(result));
        result = f_mkdir(FileName);
        if (result == FR_OK)
        {
            log("%s folder created\r\n", FileName);
            trace(systemLogFile, "%08d: %s folder created\n", HAL_GetTick(), FileName);
        }
        else
        {
            log("%s folder creation error = %s\r\n", FileName, getString(result));
            trace(systemLogFile, "%08d: %s folder creation error = %s\n", HAL_GetTick(), FileName, getString(result));
        }
    }
    FileName[8] = '/';

    result = f_unlink(FileName);
    if (result == FR_OK)
    {
        log("%s erased OK\r\n", FileName);
        trace(systemLogFile, "%08d: %s erased OK\n", HAL_GetTick(), FileName);
    }
    else
    {
        log("%s erase error = %s\r\n", FileName, getString(result));
        trace(systemLogFile, "%08d: %s erase error = %s\n", HAL_GetTick(), FileName, getString(result));
    }

    f_close(&USERFile);
    result = f_rename(defaultLogName, FileName);
    if (result == FR_OK)
    {
        log("%s renamed to %s\r\n", defaultLogName, FileName);
        trace(systemLogFile, "%08d: %s renamed to %s\n", HAL_GetTick(), defaultLogName, FileName);
        result = f_open(&USERFile, FileName, (FA_READ | FA_WRITE | FA_OPEN_APPEND));
        if (result == FR_OK)
        {
            log("%s opened\r\n", FileName);
            trace(systemLogFile, "%08d: %s opened\n", HAL_GetTick(), FileName);
        }
        else
        {
            log("%s open error = %s\r\n", FileName, getString(result));
            trace(systemLogFile, "%08d: %s open error = %s\n", HAL_GetTick(), FileName, getString(result));
        }
    }
    else
    {
        log("%s rename error = %s\r\n", defaultLogName, getString(result));
        trace(systemLogFile, "%08d: %s rename error = %s\n", HAL_GetTick(), defaultLogName, getString(result));
    }
}

void updateDateAndTime(const uint32_t ID, const uint8_t *data)
{
    if (!isDateReady)
    {
        if (ID == 0x0310)
        {
            const auto isWinterTime{false};
            const uint32_t timeOffset{isWinterTime ? 5639478 : 5635878 };
            uint32_t dayCounter{getField(40, 16, data)};
            uint32_t timeInSeconds{(getField(8, 24, data) - timeOffset) % 86400};
            year = 0;

            static const auto isLeapYear = [](const uint16_t _year)
            {
                return ((_year % 4) == 0);
            };
            while ((isLeapYear(year) ? 366 : 365) <= dayCounter)
            {
                dayCounter -= (((year % 4) == 0) ? 366 : 365);
                ++year;
            }

            const uint8_t daysInMonth[] = { 31, static_cast<uint8_t>(isLeapYear(year) ? 29 : 28), 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
            uint8_t monthIndex{0};
            while (daysInMonth[monthIndex] <= dayCounter)
            {
                dayCounter -= daysInMonth[monthIndex];
                ++monthIndex;
            }
            month = (monthIndex + 1);
            day = (dayCounter + 1);

            hour = (timeInSeconds / 3600);
            timeInSeconds = (timeInSeconds - hour * 3600);
            minute = (timeInSeconds / 60);
            second = (timeInSeconds - minute * 60);

            isDateReady = true;
            log("Current time: %02d:%02d:%02d\r\n", hour, minute, second);
            log("Current date: %02d.%02d.%02d\r\n",  day,  month,   year);
            trace(systemLogFile, "%08d: Current time: %02d:%02d:%02d\n", hour, minute, second);
            trace(systemLogFile, "%08d: Current date: %02d.%02d.%02d\n",  day,  month,   year);
        }
    }
}

void saveData(const uint32_t timestamp, const uint32_t ID, const bool isExtended, const char *dir, const uint8_t channelID, const uint8_t filterID, const uint8_t length, const uint8_t *payload)
{
    f_printf(&USERFile, "%08d,%08X,%s,%s,%d%02d,%lu",
            timestamp,
            ID,
          ((isExtended) ? "true" : "false"),
            dir,
            channelID,
            filterID,
            length);
    for (uint32_t i = 0; i < length; ++i)
    {
        f_printf(&USERFile, ",%02X", payload[i]);
    }
    f_printf(&USERFile, "\n");
}

void diagnosticController(const uint32_t timestamp, uint32_t ID, bool isExtended, const char *dir, const uint8_t channelID, const uint8_t filterID, uint8_t length, uint8_t *payload)
{
#ifdef SENDER_ENABLED
    // diagnostic response controller
    static bool isRequesterEnabled{false};
    static bool gotResponse{false};
    static uint8_t senderID{0};
    if (ID == 0x00AA)
    {
        isRequesterEnabled = (300 < getField(34, 14, payload));    // RPM is non-zero
    }
    else if (ID == 0x0612)
    {
        if (      // EGT response
            (payload[0] == 0xF1) &&
            (payload[1] == 0x04) &&
            (payload[2] == 0x41) &&
            (payload[3] == 0x3C)
           )
        {
            ID = 0x7F0;
            length = 8;
            payload[7] = senderID;
            gotResponse = true;
        }
        else if ( // IAT response
                 (payload[0] == 0xF1) &&
                 (payload[1] == 0x03) &&
                 (payload[2] == 0x41) &&
                 (payload[3] == 0x0F)
                )
        {
            ID = 0x7F3;
            length = 8;
            payload[7] = senderID;
            gotResponse = true;
        }
        else if ( // CAP response
                 (payload[0] == 0xF1) &&
                 (payload[1] == 0x04) &&
                 (payload[2] == 0x6C) &&
                 (payload[3] == 0x10) &&
                 (senderID == 1)
                )
        {
            ID = 0x7F1;
            length = 8;
            payload[7] = senderID;
            gotResponse = true;
        }
        else if ( // CAT response
                 (payload[0] == 0xF1) &&
                 (payload[1] == 0x04) &&
                 (payload[2] == 0x6C) &&
                 (payload[3] == 0x10) &&
                 (senderID == 3)
                )
        {
            ID = 0x7F4;
            length = 8;
            payload[7] = senderID;
            gotResponse = true;
        }
    }

    // 1744555446303341,000007xx,false,Rx,0,8,7F,0E,64,00,02,00,00,10
    if (gotResponse)
    {
        saveData(timestamp, ID, isExtended, "Rx", channelID, filterID, length, payload);
    }

    // diagnostic request controller
    if (isRequesterEnabled && isReadyToSend)
    {
        struct SizeAndData
        {
            uint8_t size;
            uint8_t data[8];
        };
        static const SizeAndData dataToSend[] =
        {
            {4, {0x12, 0x02, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00}},                   // OBD2_EGT: F1 04 41 3C AA BB
            {6, {0x12, 0x04, 0x2C, 0x10, 0x07, 0x6D, 0x00, 0x00}},                   //      CAP: F1 04 6C 10 ZZ zz 55 55
            {4, {0x12, 0x02, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x00}},                   // OBD2_IAT: F1 03 41 0F AA
            {6, {0x12, 0x04, 0x2C, 0x10, 0x07, 0x6F, 0x00, 0x00}},                   //      CAT: F1 04 6C 10 YY yy 55 55
        };
        static const uint8_t senderIDmax{sizeof(dataToSend) / sizeof(dataToSend[0])};

        if (gotResponse)
        {
            ++senderID;
            if (senderIDmax <= senderID)
            {
                senderID = 0;
            }
            gotResponse = false;
        }

        FDCAN_TxHeaderTypeDef pTxHeader;
        pTxHeader.Identifier = 0x06F1;
        pTxHeader.IdType = FDCAN_STANDARD_ID;
        pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
        pTxHeader.DataLength = dataToSend[senderID].size;
        pTxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
        pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
        pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        pTxHeader.MessageMarker = 0;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, dataToSend[senderID].data);
        isReadyToSend = false;

        ID = pTxHeader.Identifier;
        isExtended = pTxHeader.IdType;
        length = pTxHeader.DataLength;
        saveData(timestamp, ID, isExtended, "Tx", 1, filterID, length, dataToSend[senderID].data);
    }
#endif
}

void processMessage(CMessage& message)
{
    const auto timestamp{message.timestamp};
    const auto ID{message.ID};
    const auto isExtended{message.isExtended};
    const auto channelID{message.channelID};
    const auto filterID{message.filterID};
    const auto length{message.length};
    const auto payload{message.payload};
    updateDateAndTime(ID, payload);
    saveData(timestamp, ID, isExtended, "Rx", channelID, filterID, length, payload);
    diagnosticController(timestamp, ID, isExtended, "Rx", channelID, filterID, length, payload);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

void HAL_FDCAN_HighPriorityMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (0 != osMessageQueueGetSpace(messageQueueHandle))
    {
        osThreadFlagsSet(readerTaskHandle, (hfdcan->Instance == FDCAN1) ? FDCAN1_FIFO0 : FDCAN2_FIFO0);
        //log("HAL_FDCAN_HighPriorityMessageCallback\r\n");
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (
        ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) &&
        (0 != osMessageQueueGetSpace(messageQueueHandle))
       )
    {
        osThreadFlagsSet(readerTaskHandle, (hfdcan->Instance == FDCAN1) ? FDCAN1_FIFO0 : FDCAN2_FIFO0);
        //log("HAL_FDCAN_RxFifo0Callback: RxFifo0ITs = %04lX\r\n", RxFifo0ITs);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if (
        ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == FDCAN_IT_RX_FIFO1_NEW_MESSAGE) &&
        (0 != osMessageQueueGetSpace(messageQueueHandle))
       )
    {
        osThreadFlagsSet(readerTaskHandle, (hfdcan->Instance == FDCAN1) ? FDCAN1_FIFO1 : FDCAN2_FIFO1);
        //log("HAL_FDCAN_RxFifo1Callback: RxFifo1ITs = %04lX\r\n", RxFifo1ITs);
    }
}

extern "C"
{
    int __io_putchar(int ch)
    {
        HAL_UART_Transmit(&hlpuart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
        return ch;
    }
}

template <typename... Ts>
void trace(FIL& logFile, const char *value, Ts... values)
{
    f_printf(&logFile, value, values...);
    f_sync(&logFile);
}

void Error_Handler(void)
{
    __disable_irq();
    while (true);
}
