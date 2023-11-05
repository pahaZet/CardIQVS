#include <stdint.h>


/// @brief Структура выдачи информации в прилождение. Периодическая. 
struct mainPackStructure { 
    uint8_t batteryPercent; 
    uint8_t ecgPartsCount; // 0 - нет измерений
};


void getEcgADC();