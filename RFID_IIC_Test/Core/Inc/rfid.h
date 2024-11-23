#ifndef __RFID_H__
#define __RFID_H__

#include "main.h"
#include "usart.h"
#include  "string.h"
#include "oled.h"
#include  "aes.h"

//typedef struct
//{
//    uint8_t material_type[9];
//    uint8_t rgb[3];
//    uint8_t produced_date[4];
//    uint8_t batch_number[8];
//    uint8_t hot_bed_temp[4];
//    uint8_t exthead_temp[4];
//    uint8_t init_weight[2];
//    uint8_t empty_tray_weight[2];
//    uint8_t residual_weight[2];
//    uint8_t supplier[2];
//    float material_diameter;
//    float material_density;
//}Material_Data;

#define MACHINE_TYPE_NUM  	1 // S 400D (600D 1000D) PRO
#define RGB_LEN 			3
#define MATERIAL_TYPE_LEN 	16
#define BATCH_NUMBER_LEN 	16
#define MACHINE_TYPE_LEN	11
#define SUPPLIER_LEN		10

typedef struct
{
	uint8_t machine_type[MACHINE_TYPE_LEN];
	uint8_t fan_speed;
	float flow_ratio;
	uint16_t nozzle_temp;
	uint16_t nozzle_temp_l;
	uint16_t nozzle_temp_h;
	uint16_t hot_bed_temp;
	uint16_t hot_bed_temp_l;
	uint16_t hot_bed_temp_h;
	float advance_pressure;
}machine_Data;

typedef struct
{
    uint8_t material_type[MATERIAL_TYPE_LEN];
    uint8_t batch_number[BATCH_NUMBER_LEN];
    uint8_t cavity_temp;
    uint8_t rgb[RGB_LEN];
    uint32_t produced_date;
    float material_diameter;
    float material_density;
    uint16_t init_weight;
    uint16_t empty_tray_weight;
    uint16_t residual_weight;
    uint8_t supplier[SUPPLIER_LEN];
    machine_Data machines[MACHINE_TYPE_NUM];
}Material_Data;

//typedef struct
//{
//    uint8_t material_type[16];
//    //uint8_t machine_type[16];
//    uint8_t batch_number[16];
//    uint8_t cavity_temp;
//    uint8_t rgb[3];
//    uint32_t produced_date;
//    float material_diameter;
//    float material_density;
////    uint16_t hot_bed_temp_L;
////    uint16_t hot_bed_temp_H;
////    uint16_t exthead_temp_L;
////    uint16_t exthead_temp_H;
//    uint16_t init_weight;
//    uint16_t empty_tray_weight;
//    uint16_t residual_weight;
//    uint16_t supplier;
//    machine_Data machines[5];
//}Material_Data;

uint8_t RFID_Rc523_Config(void);

uint8_t RFID_Rc523_Wirte_Data(uint8_t* Data);
uint8_t RFID_Rc523_Read_Data(uint8_t* Data);

void RFID_Rc523_Write_Block(uint8_t Channel, uint8_t  blockaddr, uint8_t* Data);
void RFID_Rc523_Read_Block(uint8_t Channel, uint8_t blockaddr, uint8_t* Data);

void rfid_write_channel_data(uint8_t channel, Material_Data* data);
void rfid_read_channel_data(uint8_t channel, Material_Data* data);

#endif
