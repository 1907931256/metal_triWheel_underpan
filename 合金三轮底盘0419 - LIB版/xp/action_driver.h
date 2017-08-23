#ifndef __ACTION_DRIVER
#define __ACTION_DRIVER

#include "can.h"

#define SDO_LOAD_1 0x4f
#define SDO_LOAD_2 0x4b
#define SDO_LOAD_3 0x47 
#define SDO_LOAD_4 0x43

#define BOOTUP     0x00
#define COMMAND_SPECIFIER_NOT_VALID 0x0504
#define POSITION_ACTUAL_VALUE 0x6064
#define ACTUAL_POSITION_INTERNAL_UNIT 0x6063
#define VELOCITY_SENSOR_ACTUAL_VALUE 0x6069
#define CURRENT_ACTUAL_VALUE 0x6078

#define SDOTX 0x600
#define SDORX 0x580

#define DATA_LEN 8

#define ACTION_ENABLE  1
#define ACTION_DISABLE 0

#define RELATIVE_POSITION_MODE 0
#define ABSOLUATE_POSITION_MODE 1

#define CURRENT 1
#define VELOCITY 2
#define RELATIVE_POSITION 3
#define ABSOLUATE_POSITION 4

typedef struct
{
  uint8_t command_specifier_not_valid;

}SDO_ERROR;


#define RPDO1 0x1400
#define RPDO2 0x1401
#define RPDO3 0x1402
#define RPDO4 0x1403

#define TPDO1 0x1800
#define TPDO2 0x1801
#define TPDO3 0x1802
#define TPDO4 0x1803

void ActionSetTPdo(uint8_t drivernum,uint16_t pdo,uint8_t enable);
void ActionSetRPdo(uint8_t drivernum,uint16_t pdo,uint8_t enable);
void ActionSetTPdoComType(uint8_t drivernum,uint16_t pdo,uint8_t type);
void ActionSetRPdoComType(uint8_t drivernum,uint16_t pdo,uint8_t type);
void ActionSetTPdoBanTime(uint8_t drivernum,uint16_t pdo,uint8_t time);
void ActionSetTPdoEventCnt(uint8_t drivernum,uint16_t pdo,uint8_t event_cnt);
void ActionSetTPdoMap(uint8_t drivernum,uint16_t pdo);
void ActionSetRPdoMap(uint8_t drivernum,uint16_t pdo);
void ActionPDOVelConfig(uint8_t drivernum,uint16_t pdo,uint32_t acc,uint32_t dec);
uint8_t ActionPDOReData(CanRxMsg* RxMessage,uint8_t drivernum,uint16_t pdo);


void ActionSetHeartbeatInterval(uint8_t nodeId,uint16_t interval);
//fix me for pre-operational
void ActionSetOperationalMode(uint8_t nodeId);
void ActionEnable(uint8_t nodeId, int8_t status);

void ActionConfigTorque(uint8_t nodeId, int16_t maxTorque);   	
void ActionSetTorque( uint8_t nodeId, int16_t torque);  

void ActionConfigVelocity(uint8_t nodeId, uint32_t acc, uint32_t dec);           
void ActionSetVelocity( uint8_t nodeId, int32_t vel);  

void ActionConfigPosition(uint8_t nodeId, int32_t vel, uint32_t acc, uint32_t dec);   	
void ActionSetPosition( uint8_t nodeId, uint8_t mode, int32_t pos);   

void ActionSetHome(uint8_t nodeId, int32_t pos);

void ActionRequestMotorInfo(uint8_t nodeId, uint16_t infoType);
uint16_t ActionUpdateMotorInfo( CanRxMsg* RxMessage);

uint8_t ActionCheckError(uint8_t nodeId, CanRxMsg* RxMessage);

#endif











#define MOTOR_NUM   3






void Set_Speed_Value(int id,int tmp);
void Set_AbsolutePosition_Value(int tmp);
void Set_RelativePosition_Value(int tmp);
void Set_Current_Value(int tmp);

int32_t Get_Speed_Value(int id);
int32_t Get_AbsolutePosition_Value(void);
int32_t Get_RelativePosition_Value(void);
int32_t Get_Current_Value(void);



