#include <stdint.h>


#define ACK 0x15
#define NAK 0x17

uint8_t ackResponse = ACK;
uint8_t nakResponse = NAK;


#define ECG_LEN 8192//3200
#define FrameSize 128





/// @brief Пакет ответа ошибкой МП 
struct errortPacksStructure {
    uint8_t res;
    uint8_t packType;
    uint8_t requestPackType;
    uint8_t errorCode;
    char errorData[128];
};

/// @brief Структура выдачи информации в прилождение. Периодическая. 
struct mainPackStructure { 
    uint8_t res;
    uint8_t packType;
    uint8_t batteryPercent; 
    uint8_t ecgPartsCount; // 0 - нет измерений
    uint8_t loP;
    uint8_t loN;
};

/// @brief Пакет ответа на запрос МП получения фрейма ECG
struct ecgResponsePacksStructure {
    uint8_t res;
    uint16_t* ecgvals;
};

void getEcgADC();

uint16_t readAvgAdc(uint8_t pin, uint8_t avgAdcSampling);

void returnAckResponse();

void calcBattValues();
