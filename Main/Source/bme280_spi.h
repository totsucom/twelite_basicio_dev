
extern void bme280_init(uint8_t u8SlaveNo);
extern bool_t bme280_sleeping();
extern void bme280_readData(signed long int *pTemp, unsigned long int *pPres, unsigned long int *pHumi);
