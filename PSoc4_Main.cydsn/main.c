/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
//#include <project.h>

#include <device.h>
#include <stdio.h>            //definitions for all standard io functions

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define PSOC5_MSG_MAX 60


enum RUN_MODE { RECORDING, EXPORT_BLUE } runMode;
//Bluetooth port
void fetchBlueMsg();
void initBluetooth();

//Buffer for PSOC5 Communication
const int MAX_STR = 150;
char psoc5_msg[PSOC5_MSG_MAX];

struct _ADC {
    uint16 X;
    uint16 Y;
    uint16 Z;
}accStatus;

//Functions declaration
void send2PSoC5( char *msg );
void initPSoC5();
void exportData();

void initAccelerometer(){
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    ADC_ACCELEROMETER_Start();
    ADC_ACCELEROMETER_StartConvert();
    ADC_ACCELEROMETER_IsEndConversion( ADC_ACCELEROMETER_WAIT_FOR_RESULT); 
    
}

void getAccelerometer( struct _ADC *out )
{
    ADC_ACCELEROMETER_IsEndConversion( ADC_ACCELEROMETER_WAIT_FOR_RESULT); 
    out->X = ADC_ACCELEROMETER_GetResult16(AXIS_X);
    out->Y = ADC_ACCELEROMETER_GetResult16(AXIS_Y);
    out->Z = ADC_ACCELEROMETER_GetResult16(AXIS_Z);
}

int buildMsg(struct _ADC *accelData, char outMsg[] ){    
    memset(outMsg, 0, sizeof(char)*PSOC5_MSG_MAX  );
    sprintf( outMsg, ":%d,%d,%d", accelData->X, accelData->Y, accelData->Z );
    return strlen(outMsg);
}

int main()
{    
    CyGlobalIntEnable;  /* Uncomment this line to enable global interrupts. */
    
    //Initialize Accelerometer
    initAccelerometer();
    //Initialize PSoC5 communication channel
    initPSoC5();
    //Initialize Bluetooth
    initBluetooth();
    
    //Simple delay just for the sake of it
    // allow other code on PSoC5 to start up, no special time
    
    
    
	CyDelay(500);    
    for(;;)
    {            
        //Check if there is something from bluetooth
        fetchBlueMsg();

        if( runMode == EXPORT_BLUE )
        {
            // Send Message to PSoC5 and request flush!
            exportData();
        }else if( runMode == RECORDING )
        {
            getAccelerometer( &accStatus );
            buildMsg(&accStatus, psoc5_msg);
            send2PSoC5( psoc5_msg );
            CyDelay(500);
        }        
    }
    
    return 0;
}

/*
    Initialize PSOC5 communication
*/
void initPSoC5()
{    
    PSOC5_UartInit();
    PSOC5_Start();
    
}
/*
    Sends buffer to PSoC5 to log the data
*/
void send2PSoC5( char *msg )
{       
    PSOC5_UartPutString(msg);    
    PSOC5_UartPutChar(13);    
}

char inBlueChar;
void initBluetooth()
{    
    BLUE_Init();
    BLUE_Start();
}

void fetchBlueMsg()
{    
    //handle all PSoc4Msg parsing of mesg
    inBlueChar = BLUE_GetChar();

    if( inBlueChar == 'B' ){       
        runMode = EXPORT_BLUE;    
        exportData();
    }else if( inBlueChar == 0 ){        
        runMode = RECORDING;
    }        
}

void exportData()
{
    //Send PSoC5 Data Request
    send2PSoC5("B");
    //Wait for Data from PSoC5 and flush to bluetooth
    
    char inChar=0;
    //Read from PSoc5 and flush to Bluetooth
    while( inChar != '>' )
    {
        inChar = PSOC5_UartGetChar();
        if( inChar == '$' ){
            BLUE_PutChar(13);
            //BLUE_PutChar(10);
        }
        BLUE_PutChar(inChar);
        
        
    }
    
    
}
/* [] END OF FILE */

