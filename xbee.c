/* ========================================
 *
 * DESCRIPTION :
 *       XBee Rx & Tx Interface, no interrupt handler
 *
 *
 * NOTES :
 *       There is no warranty for this free software
 *       These routines is for XBee API2
 *       There is not discusion here for XBee configuration
 * 
 * AUTHOR :    Crash_
 *
 * ========================================
*/
#include "xbee.h"


/**
* init_vars()
*
* Just to be sure TAIL pointer = 0
*
**/
void init_vars()
{
    XBeeRX = 0;
}

/**
* delete_from_tail()
*
* If you need to delete from TAIL just call this function
*
**/
void delete_from_tail()
{
    if( XBeeRX )
    {
        xbee_pck *temp = XBeeRX;
        XBeeRX = XBeeRX->pck;
        if(temp->rfdata)
            free(temp->rfdata);
        if(temp->rfdata2escape)
            free(temp->rfdata2escape);
        free(temp);
    }
}

/**
* get_xbee_pck()
*
* Create a xbee_pck structure with necessary initialization
*
* @return xbee_pck
*
**/
xbee_pck* get_xbee_pck()
{
    xbee_pck *mypck = (xbee_pck*) malloc ( sizeof(xbee_pck) );
    mypck->rfdata = 0;
    mypck->rfdata2escape = 0;
    mypck->pck = 0;
    return mypck;
}

/**
* XBeePacketHandler()
*
* XBee RX data packet handler it just decide what kind of package it is
* just remember only API2 and this API ID's:
*
*   0x80 <-- 64 Bit Address
*   0x89 <-- TX Response
*   0x97 <-- AT Command Response
*
**/
void XBeePacketHandler()
{
    uint8 serr=0, sdata=0;
    xbee_pck *mypck;
    
    if(XBEE_UART_GetRxBufferSize() > 0)
    {	
        //HEADER
        get_from_uart(&serr, &sdata, 0);
        
        if( sdata == 0x7E ) // Start Header
        {
            //Creating package
            mypck = (xbee_pck*) malloc ( sizeof(xbee_pck) );
            mypck->rfdata = (uint8*) malloc ( sizeof(uint8)*100 );
            mypck->rfdata2escape = 0;
            //********** LENGTH **********//
            while(XBEE_UART_GetRxBufferSize() == 0)
                    CyDelayUs(1);
            get_from_uart(&serr, &sdata, 0);            
            mypck->length = (mypck->length | sdata) << 8;
            while(XBEE_UART_GetRxBufferSize() == 0)
                CyDelayUs(1);
            get_from_uart(&serr, &sdata, 0);
            mypck->length = mypck->length | sdata;
            
            //********** API ID **********//
            while(XBEE_UART_GetRxBufferSize() == 0)
                CyDelayUs(1);
            get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
            mypck->apiid = sdata;
            
            switch(mypck->apiid)
            {
                case 0x80:  
                            rx_64bitaddress(mypck);
                            break;
                case 0x89:  
                            rx_txstatus(mypck);
                            break;
                case 0x97:  
                            rx_atresponse(mypck);
                            break;
            }
            
            if( mypck->checksumcalc == mypck->checksum)
            {
                if( XBeeRX )
                    XBeeRX->pck = mypck;
                else
                    XBeeRX = mypck;
            }
        }
    }
}

/**
* rx_64bitaddress(xbee_pck *mypck)
*
* 64 Bit Address Manager
*
**/
void rx_64bitaddress(xbee_pck *mypck)
{
    int i, k;
    uint8 serr=0, sdata=0;
    
    //FRAME ID
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
    mypck->frameid = sdata;
    
    //MSB ADDRESS (Most Significant Byte)
    for(i=0; i < 4; i++)
    {    
        while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
        get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
        mypck->macmsb[i] = sdata;
    }                
    
    //LSB ADDRESS (Least Significant Byte)
    for(i=3; i >= 0; i--)
    {    
        while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
        get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
        mypck->maclsb[i] = sdata;
    }
    
    //RSSI
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
    mypck->rssi = sdata;
    
    //OPTIONS
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
    mypck->options = sdata;
    
    //*************
    // DATA
    //*************
    k = mypck->length - 11;
    for(i=0; i < k; i++)
    {
        while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
        get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
        mypck->rfdata[i] = sdata;
    }
    mypck->rflength = k;
    //++++++++++++++++++++++++++++++++++++++++++++++++++
    
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);

    mypck->checksum = sdata;
    mypck->checksumcalc = 0xFF - mypck->checksumcalc;    
}

/**
* rx_atresponse(xbee_pck *mypck)
*
* AT Response Manager
*
**/
void rx_atresponse(xbee_pck *mypck)
{
    int i, k;
    uint8 serr=0, sdata=0;
    
    //********** FRAME ID **********//
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
    mypck->frameid = sdata;
    
    //MSB MAC (Most Significant Byte)
    for(i=0; i < 4; i++)
    {    
        while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
        get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
        mypck->macmsb[i] = sdata;
    }                
    
    //LSB MAC (Least Significant Byte)
    for(i=3; i >= 0; i--)
    {    
        while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
        get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
        mypck->maclsb[i] = sdata;
    }
    
    //********** NET ADDRESS **********//
    while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);            
    mypck->netaddr[0] = sdata;
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);
    mypck->netaddr[1] = sdata;
    
    //********** COMMAND **********//
    while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);            
    mypck->command[0] = sdata;
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);
    mypck->command[1] = sdata;
    
    //********** STATUS **********//
    while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);            
    mypck->status = sdata;
    
    //********** DATA **********//
    k = mypck->length - 15;
    for(i=0; i < k; i++)
    {
        while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
        get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
        mypck->commdata[i] = sdata;
    }
    mypck->rflength = k;
    //++++++++++++++++++++++++++++++++++++++++++++++++++
    
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);

    mypck->checksum = sdata;
    mypck->checksumcalc = 0xFF - mypck->checksumcalc;
}

/**
* rx_txstatus(xbee_pck *mypck)
*
* TX Status Manager
*
**/
void rx_txstatus(xbee_pck *mypck)
{
    uint8 serr=0, sdata=0;
    
     //FRAME ID
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
    mypck->frameid = sdata;
    
    //STATUS
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, &(mypck->checksumcalc));
    mypck->status = sdata;
    //++++++++++++++++++++++++++++++++++++++++++++++++++
    
    while(XBEE_UART_GetRxBufferSize() == 0)
        CyDelayUs(1);
    get_from_uart(&serr, &sdata, 0);

    mypck->checksum = sdata;
    mypck->checksumcalc = 0xFF - mypck->checksumcalc; 
}

/**
* get_from_uart(uint8 *serr, uint8 *sdata, uint8 *checksum)
*
* One function to check error, take data, escape byte and checksum calc
* this help to obtain the right byte for frame
*
**/
void get_from_uart(uint8 *serr, uint8 *sdata, uint8 *checksum)
{
    uint16 tempchar;
    
    tempchar = XBEE_UART_GetByte();
    *serr = (0xFF00 & tempchar) >> 8; //Maybe you want to check data error, i do not...
    *sdata = 0x00FF & tempchar;
    
    if( *sdata == 0x7D )
    {
        while(XBEE_UART_GetRxBufferSize() == 0)
            CyDelayUs(1);
        tempchar = XBEE_UART_GetByte();
        *serr = (0xFF00 & tempchar) >> 8;
        *sdata = 0x00FF & tempchar;
        *sdata = *sdata^0x20;
    }
    if(checksum)
        *checksum = *checksum + *sdata;
}

/**
* XBeeSendPackage(xbee_pck *mypck)
*
* Recieve the package and process it to make an API2 frame with
* escaped characters and send frame.
*
* @return int
*
**/
int XBeeSendPackage(xbee_pck *mypck)
{
    int i;
    //Checksum init
    mypck->checksum = 0;
    mypck->rfdata2escape = (uint8*) malloc ( sizeof(uint8)*(mypck->rflength+15) );
    
    //Frame init, Byte 1//
    mypck->rfdata2escape[0] = xbee_settings[0];

    //Frame length Byte 2-3//
    mypck->rfdata2escape[1] = 0x00;
    mypck->rfdata2escape[2] = mypck->rflength - 4;

    //ApiFrame,Byte 4//
    mypck->rfdata2escape[3] = xbee_settings[1];
    mypck->checksum += xbee_settings[1];

    //Frame ID for Response,Byte 5//
    mypck->rfdata2escape[4] = xbee_settings[2]; 
    mypck->checksum += xbee_settings[2];

    //Destination MAC Address,Byte 6-13//
    for(i=0; i < 8; i++) //
    {
      mypck->rfdata2escape[i] = mypck->destination_address[i];
      mypck->checksum += mypck->destination_address[i];
    }

    //Setting options,Byte 14//
    mypck->rfdata2escape[13] = xbee_settings[3];
    mypck->checksum += xbee_settings[3];

    //**Data, Byte 15-N < 100**//
    for(i=14; i < 14+mypck->rflength; i++)
    {
      mypck->rfdata2escape[i] = mypck->rfdata[i-14];
      mypck->checksum += mypck->rfdata[i-14];
    }
    
    //Checksum,Byte N+1//
    mypck->checksum = 0xFF - mypck->checksum; 
    mypck->rfdata2escape[14 + mypck->rflength] = mypck->checksum;
    
    //Escaping frame mandatory for API2//
    mypck->rflength += 15;
    free(mypck->rfdata);
    mypck->rfdata = scape_frame(mypck->rfdata2escape, &(mypck->rflength));
    
    if( mypck->rfdata )
    {
        //Sending data through UART//
        for(i=0; i < mypck->rflength; i++)
          XBEE_UART_PutChar(mypck->rfdata[i]);
    }
    
    //Free data **WARNING** 
    if(mypck->rfdata)
        free(mypck->rfdata);
    if(mypck->rfdata2escape)
        free(mypck->rfdata2escape);
    
    return 0;
}

/**
* scape_frame(uint8 *frame, uint8 *length)
*
* Recieve the rf data in package to send and create a new frame
* with escaped characters, this set new length for mypck->rflength
* to send new frame
*
* @return uint8*
*
**/
uint8* scape_frame(uint8 *frame, uint8 *length)
{
   int count = 0, i , j;
   uint8 *escaped_frame;
   
   for (i=3; i < *length-1; i++)
   {
      if (frame[i] == ESCAPE1 || frame[i] == ESCAPE2 || frame[i] == ESCAPE3 || frame[i] == ESCAPE4)
         ++count;
   }
   
   if (count > 0)
   {
      escaped_frame = (uint8*)malloc (sizeof(char)*(*length + count));
      escaped_frame[0] = frame[0];
      escaped_frame[1] = frame[1];
      escaped_frame[2] = frame[2];
      
      for(i=3, j=3; i < *length-1; i++, j++)
      {
         if( frame[i] == ESCAPE1 || frame[i] == ESCAPE2 || frame[i] == ESCAPE3 || frame[i] == ESCAPE4 )
         {
            escaped_frame[j] = 0x7D;
            ++j;
            escaped_frame[j] = frame[i]^0x20;
         }
         else
            escaped_frame[j] = frame[i];
      }
       escaped_frame[j] = frame[i];
      *length = *length+count;
      }
      if( !count )
         return 0;
      else
         return escaped_frame;

}
/* [] END OF FILE */
