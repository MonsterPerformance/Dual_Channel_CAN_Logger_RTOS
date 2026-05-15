#include "main.h"
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
//#define MONITORING_ENABLED
//#define STATISTICS_ENABLED
#define SENDER_ENABLED

osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes = {
  .name = "LoggerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal
};
osThreadId_t IRQsimuTaskHandle;
const osThreadAttr_t IRQsimuTask_attributes = {
  .name = "IRQsimuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal
};
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh
};
osMessageQueueId_t messageQueueHandle;
uint8_t messageQueueBuffer[ 64 * sizeof( CMessage ) ];
StaticQueue_t messageQueueControlBlock;
const osMessageQueueAttr_t messageQueue_attributes = {
  .name = "messageQueue",
  .cb_mem = &messageQueueControlBlock,
  .cb_size = sizeof(messageQueueControlBlock),
  .mq_mem = &messageQueueBuffer,
  .mq_size = sizeof(messageQueueBuffer)
};
osTimerId_t timer10msHandle;
const osTimerAttr_t timer10ms_attributes = {
  .name = "timer10ms"
};

void SystemClock_Config();
void StartLoggerTask(void *argument);
void StartIRQsimuTask(void *argument);
void StartMonitorTask(void *argument);
void Timer10msCallback(void *argument);

const char defaultLogName[] = {"LOG_RT.CSV"};
const char systemLogName[] = {"DEBUG_RT.TXT"};
FIL systemLogFile;
volatile bool toFlushData{false};
volatile bool isDateReady{false};
volatile bool isLogRenamed{false};
volatile bool isSenderEnabled{false};
volatile bool gotResponse{false};
volatile uint16_t hour{0}, minute{0}, second{0}, day{1}, month{1}, year{26};

#ifdef STATISTICS_ENABLED
volatile uint32_t rxFIFO0IRQHPcounter{0}, rxFIFO0IRQcounter{0}, rxFIFO1IRQcounter{0}, rxFIFO0messageCounter{0}, rxFIFO1messageCounter{0};
#endif

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
    printf("\r\n\r\n\r\n  DEBUG  (RTOS) \r\n\r\n\r\n");
#else
    printf("\r\n\r\n\r\n RELEASE (RTOS) \r\n\r\n\r\n");
#endif

    HAL_GPIO_WritePin(GPIOB, (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7), GPIO_PIN_SET);

    uwTick = 0;

	osKernelInitialize();

	loggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &loggerTask_attributes);
#ifdef SIMU_ENABLED
	IRQsimuTaskHandle = osThreadNew(StartIRQsimuTask, NULL, &IRQsimuTask_attributes);
#ifdef MONITORING_ENABLED
    monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);
#endif
#endif
    messageQueueHandle = osMessageQueueNew(64, sizeof(CMessage), &messageQueue_attributes);
	timer10msHandle = osTimerNew(Timer10msCallback, osTimerPeriodic, NULL, &timer10ms_attributes);
    osTimerStart(timer10msHandle, 10);

	osKernelStart();

	while (true)
	{
	    printf("\r\nYOU ARE IN THE VERY MIDDLE OF THE ASS!!!\r\n");
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

inline const char* getString(const FRESULT fr)
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

inline const char* getString(const osThreadState_t ts)
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

inline uint32_t getField(const uint16_t LSB, const uint16_t sizeInBits, const uint8_t *buf)
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

template <typename... Ts>
inline void trace(FIL& logFile, const char *value, Ts... values)
{
    f_printf(&logFile, value, values...);
    f_sync(&logFile);
}

inline void createLog()
{
    FRESULT result;

    result = f_open(&systemLogFile, systemLogName, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (result == FR_OK)
    {
        log("%08d: %s created OK\r\n", HAL_GetTick(), systemLogName);
        trace(systemLogFile, "%08d: %s created OK\n", HAL_GetTick(), systemLogName);
    }
    else
    {
        log("%08d: %s creation error = %s\r\n", HAL_GetTick(), systemLogName, getString(result));
        trace(systemLogFile, "%08d: %s creation error = %s\n", HAL_GetTick(), systemLogName, getString(result));
    }

    result = f_open(&USERFile, defaultLogName, FA_READ | FA_WRITE | FA_OPEN_APPEND);            // FA_CREATE_ALWAYS to recreate LOG.CSV
    if (result == FR_OK)
    {
        log("%08d: %s created OK\r\n", HAL_GetTick(), defaultLogName);
        trace(systemLogFile, "%08d: %s created OK\n", HAL_GetTick(), defaultLogName);
        f_printf(&USERFile, "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n");
        f_sync(&USERFile);
    }
    else
    {
        log("%08d: %s creation error = %s\r\n", HAL_GetTick(), defaultLogName, getString(result));
        trace(systemLogFile, "%08d: %s creation error = %s\n", HAL_GetTick(), defaultLogName, getString(result));
    }
}

inline void renameLog()
{
    char FileName[] = {"DDMMYYRT/HHMMSSRT.CSV"};
    FileName[ 0] = '0' + (day / 10);
    FileName[ 1] = '0' + (day % 10);
    FileName[ 2] = '0' + (month / 10);
    FileName[ 3] = '0' + (month % 10);
    FileName[ 4] = '0' + (year / 10);
    FileName[ 5] = '0' + (year % 10);
    FileName[ 9] = '0' + (hour / 10);
    FileName[10] = '0' + (hour % 10);
    FileName[11] = '0' + (minute / 10);
    FileName[12] = '0' + (minute % 10);
    FileName[13] = '0' + (second / 10);
    FileName[14] = '0' + (second % 10);
    FileName[8] = 0;
    DIR targetDirectory;
    FRESULT result{f_opendir(&targetDirectory, FileName)};
    if (result == FR_OK)
    {
        log("%08d: %s folder opened OK\r\n", HAL_GetTick(), FileName);
        trace(systemLogFile, "%08d: %s folder opened OK\n", HAL_GetTick(), FileName);
    }
    else
    {
        log("%08d: %s folder open error = %s\r\n", HAL_GetTick(), FileName, getString(result));
        trace(systemLogFile, "%08d: %s folder open error = %s\n", HAL_GetTick(), FileName, getString(result));
        result = f_mkdir(FileName);
        if (result == FR_OK)
        {
            log("%08d: %s folder created\r\n", HAL_GetTick(), FileName);
            trace(systemLogFile, "%08d: %s folder created\n", HAL_GetTick(), FileName);
        }
        else
        {
            log("%08d: %s folder creation error = %s\r\n", HAL_GetTick(), FileName, getString(result));
            trace(systemLogFile, "%08d: %s folder creation error = %s\n", HAL_GetTick(), FileName, getString(result));
        }
    }
    FileName[8] = '/';

    result = f_unlink(FileName);
    if (result == FR_OK)
    {
        log("%08d: %s erased OK\r\n", HAL_GetTick(), FileName);
        trace(systemLogFile, "%08d: %s erased OK\n", HAL_GetTick(), FileName);
    }
    else
    {
        log("%08d: %s erase error = %s\r\n", HAL_GetTick(), FileName, getString(result));
        trace(systemLogFile, "%08d: %s erase error = %s\n", HAL_GetTick(), FileName, getString(result));
    }

    f_close(&USERFile);
    result = f_rename(defaultLogName, FileName);
    if (result == FR_OK)
    {
        log("%08d: %s renamed to %s\r\n", HAL_GetTick(), defaultLogName, FileName);
        trace(systemLogFile, "%08d: %s renamed to %s\n", HAL_GetTick(), defaultLogName, FileName);
        result = f_open(&USERFile, FileName, FA_READ | FA_WRITE | FA_OPEN_APPEND);
        if (result == FR_OK)
        {
            log("%08d: %s opened\r\n", HAL_GetTick(), FileName);
            trace(systemLogFile, "%08d: %s opened\n", HAL_GetTick(), FileName);
        }
        else
        {
            log("%08d: %s open error = %s\r\n", HAL_GetTick(), FileName, getString(result));
            trace(systemLogFile, "%08d: %s open error = %s\n", HAL_GetTick(), FileName, getString(result));
        }
    }
    else
    {
        log("%08d: %s rename error = %s\r\n", HAL_GetTick(), defaultLogName, getString(result));
        trace(systemLogFile, "%08d: %s rename error = %s\n", HAL_GetTick(), defaultLogName, getString(result));
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

inline void switchLEDs()
{
    static uint16_t messageCounter{0};
    HAL_GPIO_WritePin(GPIOB,                           (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5), GPIO_PIN_SET  );
    HAL_GPIO_WritePin(GPIOB, (++messageCounter >> 3) & (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5), GPIO_PIN_RESET);
}

inline void updateDateAndTime(const uint32_t ID, const uint8_t *data)
{
    if (!isDateReady && (ID == 0x0310))
    {
        const auto isWinterTime{false};
        const uint32_t timeOffset{isWinterTime ? 5647183 : 5643583 };
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
        log("%08d: Current time: %02d:%02d:%02d\r\n", HAL_GetTick(), hour, minute, second);
        log("%08d: Current date: %02d.%02d.%02d\r\n", HAL_GetTick(),  day,  month,   year);
        trace(systemLogFile, "%08d: Current time: %02d:%02d:%02d\n", HAL_GetTick(), hour, minute, second);
        trace(systemLogFile, "%08d: Current date: %02d.%02d.%02d\n", HAL_GetTick(),  day,  month,   year);
    }
}

inline void saveData(const uint32_t timestamp, const uint32_t ID, const bool isExtended, const char *dir, const uint8_t channelID, const uint8_t filterID, const uint8_t length, const uint8_t *payload)
{
    static uint16_t counter{0};
    static uint32_t timestampPreviousMessage{0};
    counter = ((timestampPreviousMessage == timestamp) ? (counter + 1) : 0);
    f_printf(&USERFile, "%08d%02d0,%08X,%s,%s,%d%02d,%lu",
            timestamp,
            counter,
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
    timestampPreviousMessage = timestamp;

#ifdef STATISTICS_ENABLED
    static const uint32_t periodToSaveStatistics{1000};         // in micros
    static uint32_t timestampPreviousStatistics{0};
    if (periodToSaveStatistics <= (timestamp - timestampPreviousStatistics))
    {
        timestampPreviousStatistics = timestamp;
        static uint32_t _rxFIFO0IRQHPcounter{0}, _rxFIFO0IRQcounter{0}, _rxFIFO1IRQcounter{0}, _rxFIFO0messageCounter{0}, _rxFIFO1messageCounter{0};
        __disable_irq();
            _rxFIFO0IRQHPcounter = rxFIFO0IRQHPcounter;
            _rxFIFO0IRQcounter = rxFIFO0IRQcounter;
            _rxFIFO1IRQcounter = rxFIFO1IRQcounter;
            _rxFIFO0messageCounter = rxFIFO0messageCounter;
            _rxFIFO1messageCounter = rxFIFO1messageCounter;
        __enable_irq();
        f_printf(&USERFile, "%08d%02d0,00000000,false,Tx,000,4,%02X,%02X,%02X,%02X\n",
            timestamp,
            counter,
            ((_rxFIFO0IRQHPcounter >> 0x00) & 0xFF),
            ((_rxFIFO0IRQHPcounter >> 0x08) & 0xFF),
            ((_rxFIFO0IRQHPcounter >> 0x10) & 0xFF),
            ((_rxFIFO0IRQHPcounter >> 0x18) & 0xFF));
        f_printf(&USERFile, "%08d%02d0,00000000,false,Tx,001,4,%02X,%02X,%02X,%02X\n",
            timestamp,
            counter,
            ((_rxFIFO0IRQcounter >> 0x00) & 0xFF),
            ((_rxFIFO0IRQcounter >> 0x08) & 0xFF),
            ((_rxFIFO0IRQcounter >> 0x10) & 0xFF),
            ((_rxFIFO0IRQcounter >> 0x18) & 0xFF));
        f_printf(&USERFile, "%08d%02d0,00000000,false,Tx,002,4,%02X,%02X,%02X,%02X\n",
            timestamp,
            counter,
            ((_rxFIFO1IRQcounter >> 0x00) & 0xFF),
            ((_rxFIFO1IRQcounter >> 0x08) & 0xFF),
            ((_rxFIFO1IRQcounter >> 0x10) & 0xFF),
            ((_rxFIFO1IRQcounter >> 0x18) & 0xFF));
        f_printf(&USERFile, "%08d%02d0,00000000,false,Tx,011,4,%02X,%02X,%02X,%02X\n",
            timestamp,
            counter,
            ((_rxFIFO0messageCounter >> 0x00) & 0xFF),
            ((_rxFIFO0messageCounter >> 0x08) & 0xFF),
            ((_rxFIFO0messageCounter >> 0x10) & 0xFF),
            ((_rxFIFO0messageCounter >> 0x18) & 0xFF));
        f_printf(&USERFile, "%08d%02d0,00000000,false,Tx,012,4,%02X,%02X,%02X,%02X\n",
            timestamp,
            counter,
            ((_rxFIFO1messageCounter >> 0x00) & 0xFF),
            ((_rxFIFO1messageCounter >> 0x08) & 0xFF),
            ((_rxFIFO1messageCounter >> 0x10) & 0xFF),
            ((_rxFIFO1messageCounter >> 0x18) & 0xFF));
    }
#endif
}

inline void RQcontroller()
{
    if (isSenderEnabled)
    {
        struct SizeAndData
        {
            uint8_t size;
            uint8_t data[8];
        };
        static const SizeAndData dataToSend[] =
        {
            {5, {0x12, 0x03, 0x01, 0x3C, 0x0F, 0x00, 0x00, 0x00}}                                                               // EGT&IAT: 12 03 01 3C 0F
            ,
            {8, {0x12, 0x06, 0x2C, 0x10, 0x07, 0x6D, 0x07, 0x6F}}                                                               // CAP&CAT: 12 06 2C 10 07 6D 07 6F
        };
        static const uint8_t senderIDmax{sizeof(dataToSend) / sizeof(dataToSend[0])};
        static uint8_t senderID{0};

        if (gotResponse)
        {
            gotResponse = false;
            if (senderIDmax == ++senderID)
            {
                senderID = 0;
            }
        }

        static FDCAN_TxHeaderTypeDef pTxHeader = {
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0
        };
        pTxHeader.Identifier = static_cast<uint32_t>(0x06F1 + senderID);
        pTxHeader.DataLength = dataToSend[senderID].size;
        __disable_irq();
            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, dataToSend[senderID].data);
        __enable_irq();
    }
}

inline void RPcontroller(const uint32_t timestamp, uint32_t ID, bool isExtended, const char *dir, const uint8_t channelID, const uint8_t filterID, uint8_t length, uint8_t *payload)
{
    if (
        (ID == 0x0612) &&
        (
         ((payload[0] == 0xF1) && (payload[1] == 0x06) && (payload[2] == 0x41) && (payload[3] == 0x3C) && (payload[6] == 0x0F)) // EGT&IAT: F1 06 41 3C AA aa 0F BB
         ||
         ((payload[0] == 0xF2) && (payload[1] == 0x06) && (payload[2] == 0x6C) && (payload[3] == 0x10))                         // CAP&CAT: F2 06 6C 10 AA aa BB bb
        )
       )
    {
        saveData(timestamp, (0x0700 | payload[0]), isExtended, "Rx", channelID, filterID, length, payload);
        gotResponse = true;
    }
}

inline void processMessage(CMessage& message)
{
    const auto timestamp{message.timestamp};
    const auto ID{message.ID};
    const auto isExtended{message.isExtended};
    const auto channelID{message.channelID};
    const auto filterID{message.filterID};
    const auto length{message.length};
    const auto payload{message.payload};
    if (!isSenderEnabled && (ID == 0x00AA) && (300 < getField(34, 14, payload)))
    {
        isSenderEnabled = true;    // RPM is non-zero
    }
    updateDateAndTime(ID, payload);
	if (ID != 0x0612)
    {
        const bool isDiagnosticRequest{(0x06F1 <= ID) && (ID <= 0x06FF)};
        saveData(timestamp, ID, isExtended, (isDiagnosticRequest ? "Tx" : "Rx"), channelID, filterID, length, payload);
    }
#ifdef SENDER_ENABLED
    RPcontroller(timestamp, ID, isExtended, "Rx", channelID, filterID, length, payload);
#endif
}

inline bool readOutMessages(FDCAN_HandleTypeDef *hfdcan, const uint32_t RxLocation)
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
        static uint8_t rxData[BUFFERSIZE];
#ifdef SIMU_ENABLED
        bool result{true};
        --rxFIFOfillLevel;
        rxHeader.Identifier = 0x00CE;
        rxHeader.DataLength = 8;
        rxHeader.IdType = FDCAN_STANDARD_ID;
        rxHeader.FilterIndex = 99;
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
#ifdef STATISTICS_ENABLED
                if (channelID == 1)
                {
                    ++rxFIFO0messageCounter;
                }
                else
                {
                    ++rxFIFO1messageCounter;
                }
#endif
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

void StartLoggerTask(void *argument)
{
    log("LoggerTask: start\r\n");
    mountSD();

    CMessage message;
    while (true)
    {
        const uint32_t messagesInQueue{osMessageQueueGetCount(messageQueueHandle)};
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
        }
    }
}

void StartIRQsimuTask(void *argument)
{
    log("IRQsimuTask: start\r\n");

    while (true)
    {
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            readOutMessages(&hfdcan1, FDCAN_RX_FIFO0);
        }
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            readOutMessages(&hfdcan1, FDCAN_RX_FIFO1);
        }
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            readOutMessages(&hfdcan2, FDCAN_RX_FIFO0);
        }
        if (0 != osMessageQueueGetSpace(messageQueueHandle))
        {
            readOutMessages(&hfdcan2, FDCAN_RX_FIFO1);
        }
        osDelay(1);
    }
}

void StartMonitorTask(void *argument)
{
    log("MonitorTask: start\r\n");

    while (true)
    {
        log("\r\n"
            "   MonitorTask: IRQ simu task status = %s\r\n"
            "   MonitorTask:  monitor task status = %s\r\n"
            "   MonitorTask:   logger task status = %s\r\n"
            "\r\n",
                getString(osThreadGetState(IRQsimuTaskHandle)),
                getString(osThreadGetState(monitorTaskHandle)),
                getString(osThreadGetState( loggerTaskHandle)));
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        osDelay(1000);
    }
}

void Timer10msCallback(void *argument)
{
#ifdef SENDER_ENABLED
    static const uint16_t msCounterToSendThreshold{1};                            // period in x10 ms to send requests
    static uint16_t msCounterToSend{0};
    if (msCounterToSendThreshold == ++msCounterToSend)
    {
        msCounterToSend = 0;
        static uint8_t counter{0};
        if (10 == ++counter)
        {
            counter = 0;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        }
        RQcontroller();
    }
#endif
    static const uint16_t msCounterToFlushThreshold{1000};                        // period in x10 ms to flush data
    static uint16_t msCounterToFlush{0};
    if (msCounterToFlushThreshold == ++msCounterToFlush)
    {
        msCounterToFlush = 0;
        toFlushData = true;
        log("HAL_TIM_PeriodElapsedCallback: toFlushData\r\n");
    }
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
#ifdef STATISTICS_ENABLED
    ++rxFIFO0IRQHPcounter;
#endif
    readOutMessages(hfdcan, FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
#ifdef STATISTICS_ENABLED
        ++rxFIFO0IRQcounter;
#endif
        readOutMessages(hfdcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
    {
#ifdef STATISTICS_ENABLED
        ++rxFIFO1IRQcounter;
#endif
        readOutMessages(hfdcan, FDCAN_RX_FIFO1);
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

void Error_Handler(void)
{
    __disable_irq();
    while (true);
}
