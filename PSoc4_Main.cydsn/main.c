/* ========================================
 *
 * The following firmware was developed by Soynerdito
 * 
 * 
 * You are free to:
 * -To Share — to copy, distribute and transmit the work 
 * -To Remix — to adapt the work 
 *
 * ========================================
 */
 
/* GeoCar Project source
 * By Soynerdito
 * Version 1
 * Updated Use Github dates
 *
 * Refer to element14 Cypress Development Kit GeoCar blog for project descriptions
 *
 */

#include <device.h>
#include <stdio.h>            //definitions for all standard io functions

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define PSOC5_MSG_MAX 60


enum RUN_MODE { RECORDING, EXPORT_BLUE, ALERT } runMode;
//Bluetooth port
void fetchBlueMsg();
void initBluetooth();

void setRunMode(enum RUN_MODE mode );

//Buffer for PSOC5 Communication
const int MAX_STR = 150;
char psoc5_msg[PSOC5_MSG_MAX];

//Connection GREEN BLUE RED
// Color    B2 B1 B0
// RED       1  1  0    = 6
// BLUE      1  0  1    = 5
// GREEN     0  1  1    = 3
// NONE      1  1  1    = 7
const int LED_RED   = 6;
const int LED_BLUE  = 5;
const int LED_GREEN = 3;
const int LED_NONE  = 7;

struct _ADC {
    uint16 X;
    uint16 Y;
    uint16 Z;
}accStatus;

//Functions declaration
void send2PSoC5( char *msg );
void initPSoC5();
void exportData();

/* Initialize Accelerometer ADC control */
void initAccelerometer(){
    ADC_ACCELEROMETER_Start();
    ADC_ACCELEROMETER_StartConvert();
    ADC_ACCELEROMETER_IsEndConversion( ADC_ACCELEROMETER_WAIT_FOR_RESULT); 
    
}

/* Reads ADC data from Accelerometer */
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
    /* Enables interrupts */
    CyGlobalIntEnable;  
    
    /* Initialize Accelerometer */
    initAccelerometer();
    /*  Initialize PSoC5 communication channel */
    initPSoC5();
    /*  Initialize Bluetooth    */
    initBluetooth();
    /*  Delay just because - Allow PSoC5 to initialize as well    */
	CyDelay(500);    
    
    for(;;)
    {        
        /*  Check if there is something from bluetooth */
        fetchBlueMsg();

        switch( runMode ){
            case EXPORT_BLUE:
            {
                //TODO this code should not be needed here. Check on when message received.
                //this Export Blue might be dead code
                //Send Message to PSoC5 and request flush!
                exportData();
            }
            break;
            case RECORDING:
            {
                getAccelerometer( &accStatus );
                buildMsg(&accStatus, psoc5_msg);
                send2PSoC5( psoc5_msg );
                CyDelay(500);
            }
            break;
            default:
            break;
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
    switch( inBlueChar ){
        case 'B':
        {
            setRunMode( EXPORT_BLUE );
            exportData();                     
            setRunMode( RECORDING );  
        }
        break;
        case 'A':
        {
            setRunMode( ALERT );           
        }
        break;
        case 0:
        {
            setRunMode( RECORDING );
        }
        break;
        default:
        break;
        
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

/* Configures the Running mode of the PSoC4 */
void setRunMode(enum RUN_MODE mode )
{
    runMode = mode;
    switch( mode ){
        case EXPORT_BLUE:
        {
            OutStatus_Write(LED_BLUE);        
        }
        break;
        case RECORDING:        
        {
            OutStatus_Write(LED_GREEN);
        }
        break;
        case ALERT:
        {
            OutStatus_Write(LED_RED);
        }
        break;
        default:
            OutStatus_Write(LED_NONE);
        break;
    }
}
/* [] END OF FILE */

