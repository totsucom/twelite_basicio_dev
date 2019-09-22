#include "basicio.h"
#include "bme280_i2c.h"

static uint8_t bme280_address; // SDO:GND=0x76 / SDO:VCC=0x77

static signed long int t_fine;
static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;
static int8_t  dig_H1;
static int16_t dig_H2;
static int8_t  dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t  dig_H6;

static bool_t readTrim()
{
    uint8_t data[32];
    if (!i2c_read(bme280_address, 0x88, data, 24)) return FALSE;
    //8e 6e a1 66 32 00 3d 93 22 d6 d0 0b
    //b8 22 1e ff f9 ff 0c 30 20 d1 88 13

    if (!i2c_read(bme280_address, 0xA1, &data[24], 1)) return FALSE;
    //4b

    if (!i2c_read(bme280_address, 0xE1, &data[25], 7)) return FALSE;
    //65 01 00 14 2c 03 1e 
    
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];
    return TRUE;
}

//センサーの初期化。最初に１回だけ呼び出す
//アドレス SDO:GND=0x76 / SDO:VCC=0x77
bool_t bme280_init(uint8_t u8Address) {
    bme280_address = u8Address;

    //int16_t r = i2c_readByte(bme280_address, 0xd0);
    //if (r == -1 || r != 0x60) return FALSE;    //Not BME280

    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 1;               //1:Forced mode (3:Normal mode) フォースモードで１回測定
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;

    if (!i2c_writeByte(bme280_address, 0xF2, ctrl_hum_reg)) return FALSE;
    if (!i2c_writeByte(bme280_address, 0xF4, ctrl_meas_reg)) return FALSE;
    if (!i2c_writeByte(bme280_address, 0xF5, config_reg)) return FALSE;
    if (!readTrim()) return FALSE;
    return TRUE;
}

static signed long int calibration_T(signed long int adc_T)
{
    signed long int var1, var2;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

static unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0) {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if (P<0x80000000) {
       P = (P << 1) / ((unsigned long int) var1);   
    } else {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    return (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
}

static unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;
    unsigned long int w;
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    w = v_x1 >> 12;
    return (w * 100) >> 10;
}

//フォースモードが完了したらスリープに入るので
//この関数がTRUEならbme280_readData()で値を取り出せる
bool_t bme280_sleeping() {
    int16_t r = i2c_readByte(bme280_address, 0xf4);
    if (r == -1) return FALSE;
    return ((r & 3) == 0);
}

//温度、気圧、湿度はx100の値が返される
bool_t bme280_readData(signed long int *pTemp, unsigned long int *pPres, unsigned long int *pHumi)
{
    unsigned long int humi_raw,temp_raw,pres_raw;
    uint32_t data[8];
    uint8_t i;

    if (!i2c_write(bme280_address, 0xF7, NULL, 0)) return FALSE;
    for (i=0; i<8; i++) {
        data[i] = i2c_readByteOnly(bme280_address);
    }

    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    humi_raw  = (data[6] << 8) | data[7];

    *pTemp = calibration_T(temp_raw);
    *pPres = calibration_P(pres_raw);
    *pHumi = calibration_H(humi_raw);
    return TRUE;
}

