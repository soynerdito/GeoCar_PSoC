/* ========================================
 *
 * Copyright Soynerdito, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * Open source
 *
 * ========================================
*/
#include <device.h>
#include <FS.h>
#include <string.h>
#include <Global.h>
#include <stdlib.h>

int main();
int logTime(char input[] );
void clearBuffer(char *buff, int len);
void clearPSoC4Buffer();
void clearPSoC4Msg();
void fetchPSoC4Msg();

enum RUN_MODE { RECORDING, EXPORT_BLUE } runMode;

/****************************************
    Bluetooth Communication
*****************************************/
//void initBluetooth();
//void clearBlueBuffer();
/*enum RUN_MODE fetchPBlueMsg();*/
void flushGPSData();

const int MAX_STR = 150;
char sdFile[10] = "GPSDAT.txt";
char CRLF[2] = { 10, 13 };
//Start character for GPS messages
char START = '$';

char gpsSeconds[2] = {'0','0'};


/****************************************
    PSoC4 Data Buffers
*****************************************/
#define PSOC4_MSG_MAX  80
char psoc4_msg[PSOC4_MSG_MAX];
char psoc4_buffer[PSOC4_MSG_MAX];

int psoc4MsgPos = 0;
int psoc4MsgStarted = 0;
char inPSoc4Char;

void clearBuffer(char *buff, int len)
{
    memset(buff, 0, (sizeof(char)*len));
}

void clearPSoC4Buffer()
{
    clearBuffer(psoc4_buffer, PSOC4_MSG_MAX );
}
void clearPSoC4Msg()
{
    clearBuffer(psoc4_msg, PSOC4_MSG_MAX );
}

void movePSoC4Buffer2Msg()
{
    memcpy(psoc4_msg, psoc4_buffer, strlen(psoc4_buffer)+1 );
    clearPSoC4Buffer();
}


int main()
{
    //Set default status as Running
    runMode = RECORDING;
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
	PWM_1_Start();
    char sdVolName[10];     /* Buffer that will hold SD card Volume name */
    //U16 step = 1u;
    FS_FILE * pFile;    
    char inChar;    
    GPS_Init();
    GPS_Start();
    
    PSOC4_Init();
    PSOC4_Start();
    
    /* Initialize file system */
    FS_Init();
        
    char inputLine[MAX_STR];
    int pos=0;
    //memset(inputLine, sizeof(char)*MAX_STR,0);
    memset(inputLine, 0, sizeof(char)*MAX_STR );
    
    /* Get volume name of SD card #0 */
    if(0 != FS_GetVolumeName(0u, &sdVolName[0], 9u))
    {
    }

    CyGlobalIntEnable; /* Uncomment this line to enable global interrupts. */
    /* Create specific file for modification */
    pFile = FS_FOpen(sdFile, "a");
    int started = 0;
    //char inPSOC4;
    for(;;)
    {
        //If  Flush to bluetooth then flush via PSOC4 all file
        if( runMode ==EXPORT_BLUE )
        {
            //Close existing file
            FS_FClose( pFile );    
            //Open file for Reading
            pFile = FS_FOpen(sdFile, "r");
            //Move to the beginning of the file
            FS_FSeek(pFile, 0, 0 );
            //Flush SDCard to Bluetooth
            flushGPSData( pFile );
            FS_FClose( pFile ); 
            CyDelay(5);
            pFile = FS_FOpen(sdFile, "a");
             runMode = RECORDING;
        }

        //Fetch data from PSoC4
        fetchPSoC4Msg();
        
        inChar = GPS_GetChar();
        if( inChar == START ){
            started = 1;
            pos=0;
        }

        /* Place your application code here. */
        if( pos >=MAX_STR ){
            //Avoid overflow - clear buffer
            pos=0;
            memset(inputLine, sizeof(char)*MAX_STR,0);
        }else if( inChar == 13 ){
            //Line Feed - Time to break into new line!!
            //Flush to disk!!            
            if( pos >5 ){
                if( strncmp(inputLine, "$GPRMC", 6 ) == 0 ){                    
                    char input[2];
                    input[0] = inputLine[11];
                    input[1] = inputLine[12];
                    if( logTime(input ) )
                    {
                        pos--;
                        FS_Write(pFile, inputLine, pos);
                        //If Msg is not empty
                        if( psoc4_msg[0] != 0 )
                        {                                 
                            FS_Write(pFile, psoc4_msg, strlen(psoc4_msg));                        
                        }else{
                            FS_Write(pFile, psoc4_buffer,(psoc4MsgPos-1));                             
                        }
                        //CyDelay(2);
                        FS_Write(pFile, "\n", 1);
                        //CyDelay(1);
                    }                    
                }                
            }
            started = 0;
            pos=0;
            memset(inputLine, sizeof(char)*MAX_STR,0);
        }else if( started>0 && inChar != 0 && inChar != 10 && inChar != 13 ){
            inputLine[pos] = inChar;
            pos++;
        }
    }
    
    return 0;
}


//$GPRMC,020036
int iLastSec = 0;
int logTime(char input[] )
{
  int logMe = 0;
  int value = (input[0]-48)*10 + (input[1]-48);  
  if( (iLastSec - value) >= 5 || (iLastSec - value) <= -5 )
  {
    iLastSec = value;
    logMe = 1;
  }

  return logMe;
}


void readPSoC4AgregateData()
{
    //Check for any overflow
    if( psoc4MsgPos >=PSOC4_MSG_MAX ){
        //Avoid overflow - clear buffer
        psoc4MsgPos=0;            
        clearPSoC4Buffer();
    }else if( inPSoc4Char == 10 || inPSoc4Char == 13 ){
        //Line Feed - Time to break into new line!!
        //store in mesg for latter storage
        if( strncmp(psoc4_buffer, ":", 1 ) == 0 ){                    
            movePSoC4Buffer2Msg();                    
        }          
        psoc4MsgStarted = 0;
        psoc4MsgPos=0;            
    }else if( psoc4MsgStarted>0 && inPSoc4Char != 0 
        && inPSoc4Char != 10 && inPSoc4Char != 13 ){
        psoc4_buffer[psoc4MsgPos] = inPSoc4Char;
        psoc4MsgPos++;
    }
}

void fetchPSoC4Msg()
{
    //handle all PSoc4Msg parsing of mesg
    inPSoc4Char = PSOC4_GetChar();
 
    if( inPSoc4Char == ':' ){        
        psoc4MsgStarted = 1;
        psoc4MsgPos=0;
        runMode = RECORDING;
    }else if( inPSoc4Char == 'B' ){        
        psoc4MsgStarted = 1;
        psoc4MsgPos=0;
        runMode = EXPORT_BLUE;
    }
    if( runMode == RECORDING )
    {
        readPSoC4AgregateData();
    }else if( runMode == EXPORT_BLUE )
    {
        //Export data to Bluetooth via PSoC4
    }
    
}


/*
void initBluetooth()
{    
    BLUE_Init();
    BLUE_Start();
}

char inBlueChar;
int blueMsgStarted = 0;
#define BLUE_MSG_MAX 10
char blue_buffer[BLUE_MSG_MAX];
int blueMsgPos = 0;

void clearBlueBuffer()
{
    clearBuffer(blue_buffer, BLUE_MSG_MAX );
}
*/
/*enum RUN_MODE fetchPBlueMsg()
{
    //handle all PSoc4Msg parsing of mesg
    inBlueChar = BLUE_GetChar();

    if( inBlueChar == 'B' ){        
        blueMsgStarted = 1;
        blueMsgPos=0;
    }
    //Check for any overflow
    if( blueMsgPos >= BLUE_MSG_MAX ){
        //Avoid overflow - clear buffer
        blueMsgPos=0;            
        clearBlueBuffer();
    }else if( inBlueChar == 10 || inBlueChar == 13 ){
        //Line Feed - Time to break into new line!!
        //store in mesg for latter storage
        if( strncmp(blue_buffer, "BGET", 4 ) == 0 ){                    
            //Set Mode to flush data via bluetooth
            BLUE_PutString( "OK CONNECTED" );
            runMode = SEND_BLUETOOTH;
        }else{ 
            runMode = RECORDING;
        }
        blueMsgStarted = 0;
        blueMsgPos=0;            
    }else if( blueMsgStarted>0 && inBlueChar != 0 
        && inBlueChar != 10 && inBlueChar != 13 ){
        blue_buffer[blueMsgPos] = inBlueChar;
        blueMsgPos++;
        BLUE_PutChar( inBlueChar );
    }

    return runMode;
}*/

void flushGPSData(FS_FILE *pFile)
{    
    int readLen;    
    int pos;
    clearPSoC4Buffer();
    while ( !FS_FEof( pFile ) )
    {
        readLen = FS_Read( pFile,  psoc4_buffer, PSOC4_MSG_MAX );
        if( readLen >0 )
        {
            for( pos = 0; pos < readLen; pos++ ){
                PSOC4_PutChar( psoc4_buffer[pos] );
                CyDelay(2);
            }
            //Flush to Bluetooth            
            //PSOC4_PutString( psoc4_buffer );            
            clearPSoC4Buffer();
            CyDelay(20);
        }        
    }
    PSOC4_PutChar('>');
}
/* [] END OF FILE */