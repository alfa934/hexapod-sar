#include <Dynamixel2Arduino.h>

/*
 * Dynamixel AX18 servos (currently),
 * but other models can be used
 *
 */
 
#define RXD2 16
#define TXD2 17
HardwareSerial dxl_serial(2);

#define DXL_SERIAL dxl_serial
const int DXL_DIR_PIN = 4;

////---- Hexapod Legs
//const uint8_t DXL_ID_CNT = 20;
//const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 67, 69};

//--- Test Servos
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {4, 14};


const float DXL_PROTOCOL_VERSION = 1.0;

const uint16_t GOAL_POSITION_ADDR = 30;
const uint16_t GOAL_POSITION_LEN = 2;

typedef struct sw_data
{
  int16_t goal_position;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

//---- Rx from STM32
char rx_buff[40];
uint16_t rx_dxl_pos[DXL_ID_CNT] = {0};
unsigned long int rx_interval = 0;


void setup()
{
    Serial.begin(115200);
    
    dxl_serial.begin(1000000, SERIAL_8N1, RXD2, TXD2);
    
    dxl.begin(1000000);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
    for(uint8_t i = 0; i < DXL_ID_CNT; i++)
    {
        dxl.torqueOff(DXL_ID_LIST[i]);
        dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
        dxl.torqueOn(DXL_ID_LIST[i]);
    }
    
    sw_infos.packet.p_buf = nullptr;
    sw_infos.packet.is_completed = false;
    sw_infos.addr = GOAL_POSITION_ADDR;
    sw_infos.addr_length = GOAL_POSITION_LEN;
    sw_infos.p_xels = info_xels_sw;
    sw_infos.xel_count = 0;

    
    for(uint8_t i = 0; i < DXL_ID_CNT; i++)
    {
        info_xels_sw[i].id = DXL_ID_LIST[i];
        info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
        sw_infos.xel_count++;

        rx_dxl_pos[i] = 512; //--- middle position
    }
    
    sw_infos.is_info_changed = true;
  
}


void loop()
{
    if(millis() - rx_interval >= 5) //--- every x ms
    {
        rx_interval = millis();
        
        receiveData();

        for(int i = 0; i < DXL_ID_CNT; i++)
        {
            sw_data[i].goal_position = rx_dxl_pos[i];
        }
        
        sw_infos.is_info_changed = true;
        
        if(dxl.syncWrite(&sw_infos)){}
    }
}


void receiveData()
{
    if(Serial.available())
    {      
        static int rx_status;
        static int header_status;
        char temp;
        
        if(header_status == 0)
        { 
            if(Serial.available() >= 3 + sizeof(rx_buff))
            { 
                for(int i = 0; i < 3 + sizeof(rx_buff); i++)
                { 
                    temp = Serial.read();
                    
                    if(rx_status == 0 && temp == 'A')      
                    { 
                        rx_status++; 
                    }
                    else if(rx_status == 1 && temp == 'B') 
                    { 
                        rx_status++; 
                    }
                    else if(rx_status == 2 && temp == 'C')
                    {
                        rx_status = 0;
                        header_status = 1;
                        break;
                    }
                }
            }
        }
        
        else if(header_status == 1)
        { 
            char rx_data[sizeof(rx_buff)];
            
            if(Serial.available() >= sizeof(rx_buff))
            {
                for(int i = 0; i < sizeof(rx_buff); i++)
                {
                    rx_data[i] = Serial.read();
                }
                
                memcpy(&rx_buff, rx_data, sizeof(rx_buff));

                for(int i = 0; i < DXL_ID_CNT; i++)
                {
                    memcpy(&rx_dxl_pos[i], rx_buff + (i * 2), 2);
                }
                
                header_status = 0;
            }
        }
    
    }
}
