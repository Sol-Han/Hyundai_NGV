#ifndef STRUCT_H
#define STRUCT_H

#endif // STRUCT_H

#define PI 3.1415926535897931

typedef enum EnumPacketID //Packet type definition Enum
{
    PACKET_CURRENTDEVICEINFO = 0,   // 0 : UV state info
    PACKET_SEARCHROUTE,             // 1 : search route
    PACKET_STANDBY,                 // 2 : standby
    PACKET_RESPOND,                 // 3 : respond to command
    PACKET_PATHFINDING,             // NOT NEEDED HERE 4 : Task Allocation info send
    PACKET_PATHFINDING_RESULT,      // NOT NEEDED HERE 5 : Task Allocation path send
    PACKET_MANUALCOMMAND,           // 6 : Manual Command
}EnumPacketID;

typedef struct structPacketSendDeviceSetting // UV->ICSUV
{
    int m_wPacketID ;           // Packet ID
    short m_wPacketSize ;       // Packet size
    int m_dwPacketNum ;         // Packet sequential number

    char Device_Name[20] ;
    int Device_Type ;           // 1 : USV , 2 : UAV
    int Device_SearchRange ;    // USV : FOV/degrees , UAV : radius/m
    int Device_ID ;

    double Current_Latitude ;   // 36.XXX   // GIVE UTM
    double Current_Longitude ;  // 126.XXX  // GIVE UTM
    double Current_Altitude ;   // until tenth decimal
    double Current_Cog ;        // Heading

    double Sog ;        // (Knot)

    double Length ;             // (m)
    double Breadth ;            // (m)

    unsigned int Current_Mode ; // 0 : manual , 1 : automatic , 2 : Standby WP Standby

    // double GPS_Time;
    // double Target_Latitude;
    // double Target_Longitude;
    // double Target_Altitude;

}structPacketSendDeviceSetting;

typedef struct structPacketSendRoute // ICSUV->UV
{
    int m_wPacketID ;                   // Packet ID
    short m_wPacketSize ;               // Packet size
    int m_dwPacketNum ;                 // Packet sequential number

    int WayCount ;
    int m_nRouteMode ;                  // route type (0:initial search, 1:thorough search)
    double m_dblSearchRoute[80][4] ;    // [number][latitude,longitude,altitude,destination type (0:wp, 1:standby)]

}structPacketSendSearchRoute;

typedef struct structPacketStandbyDevice // UV->ICSUV
{
    int m_wPacketID ;                  // Packet ID
    short m_wPacketSize ;              // Packet size
    int m_dwPacketNum ;                // Packet sequential number
    
    unsigned int m_uiIndex ;           // UV ID (UAV1: 1, UAV2: 2~, USV1: 11, USV2: 12~)
    
}structPacketStandbyDevice;

typedef struct structPacketCommandRespond // UV->ICSUV
{
    int m_wPacketID ;                   // Packet ID
    short m_wPacketSize ;               // Packet size
    int m_dwPacketNum ;                 // Packet sequential number

    unsigned int m_RecvCommandNum ;     // PACKET ID of received message
    
}structPacketCommandRespond;

typedef struct structManualCommand // ICSUV->UV
{
    int m_wPacketID ;               // Packet ID
    short m_wPacketSize ;           // Packet size
    int m_dwPacketNum ;             // Packet sequential number

    double m_dbSurgeSpeed ;         // (-1~1) Forward Speed 
    double m_dbSwaySpeed ;          // (-1~1) Sideways Speed ONLY FOR UAV (0 for USV)
    double m_dbHeaveSpeed ;         // (-1~1) Updown Speed ONLY FOR UAV (0 for USV)

    double m_dbYawSpeed ;           // (-1~1) Angular Speed 

    unsigned int Current_Mode ;     // 0 : manual , 1 : automatic
    
}structManualCommand;
