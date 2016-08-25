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

#if !defined(__MYXBEE__)
#define __MYXBEE__

    #include <project.h>
    #include <stdio.h>
    #include <stdlib.h>

    
    typedef struct xbee_pck
    {
        //Regular Package Normalization
        uint16 length;
        uint8 apiid;
        uint8 frameid;
        uint8 macmsb[4];
        uint8 maclsb[4];
        uint8 netaddr[2];
        uint8 options;
        uint8 *rfdata;
        uint8 *rfdata2escape;
        uint8 rflength;
        uint8 checksum;
        uint8 checksumcalc;
        //RX additional parameters
        uint8 rssi;
        //AT Response
        uint8 command[2];
        uint8 status;
        uint8 commdata[100];
        //TX parameters
        uint8 destination_address[8];
        //Tail
        struct xbee_pck *pck;
    } xbee_pck;

    //XBee escape characters API2
    #define ESCAPE1         0x7E
    #define ESCAPE2         0x7D
    #define ESCAPE3         0x11
    #define ESCAPE4         0x13
    #define ESCAPE5         0x7B
    
    //** DEFINED XBEE BYTES **/
    static uint8 xbee_settings[4] = {0x7E, 0x00, 0x01, 0x04};
    
    extern xbee_pck *XBeeRX;
    xbee_pck *XBeeRX;

    void init_vars();
    void delete_from_tail();
    xbee_pck* get_xbee_pck();
    void XBeePacketHandler();
    void rx_64bitaddress(xbee_pck *mypck);
    void rx_atresponse(xbee_pck *mypck);
    void rx_txstatus(xbee_pck *mypck);
    void get_from_uart(uint8 *serr, uint8 *sdata, uint8 *checksum);
    int XBeeSendPackage(xbee_pck *mypck);
    uint8* scape_frame(uint8 *frame, uint8 *length);

#endif

/* [] END OF FILE */
