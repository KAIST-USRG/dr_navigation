
#ifndef DefineList_H
#define DefineList_H

// essential header for ROS
#include <ros/ros.h>

// for topic message
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

// for using serial communication
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>


#define sizeofdata    40

#define header1       0x12
#define header2       0x34
#define IDs1          0x01
#define IDs2          0x01


#define PORT1 		"/dev/ttyUSB0"
#define BAUDRATE 	921600

// Main Function Argument
#define DF_MAIN_FUNC_ARGS_MAX    64
#define DF_MAIN_FUNC_ARG_00     "aLOG"
#define DF_MAIN_FUNC_ARG_01     "aWPN"
#define DF_MAIN_FUNC_ARG_02     "pGNC"


#define D2R	   3.141592653/180.0
#define eps        0.000001

#define Vx_max     3.0
#define Vz_max     1.0


char  filename[50];

int   FdPort1;
int   count_ros = 0;
int   flag_input;

unsigned char flagA = 0;
unsigned char flagB = 0;
unsigned char flagC = 0;
unsigned char flagD = 0;

float t_cur = 0.0;
float velx  = 0.0;
float vely  = 0.0;
float velz  = 0.0;
float cmdr  = 0.0;

float sat(float data, float max);

void Initialization(void);

int   OpenSerial(char* device_name);
void  SerialSend(int fd);
void  SerialReceive(int FdPort1);

void* receive_p_thread(void *fd);
void  OnboardLog(void);

void  UpdateCommand(void);

#pragma pack(1)
struct struct_t_MessageBody // Always Make 40 Bytes
{
	unsigned char   FlagA;
	unsigned char   FlagB;
	unsigned char   FlagC;
	unsigned char   FlagD;
        float 		CmdVelAil;
        float 		CmdVelEle;
        float 		CmdVelDown;
	float           CmdR_dps;
        float 		unresolv[2];
        uint16_t        PWMcmd[6];
};
#pragma pack()

#pragma pack(1)
struct struct_Senddata
{
        unsigned char   header[2];
        unsigned char 	IDs[2];
        unsigned char 	Data[40];
        unsigned char   checksum[2];
};
#pragma pack()

#pragma pack(1)
struct struct_t_RXttyO
{
    // -------------
    // Packet Header
    // -------------
    uint8_t HeaderA; // 0x12;
    uint8_t HeaderB; // 0x34;
    uint8_t HeaderC; // 0x56;
    uint8_t HeaderD; // 0x78;

    // ----
    // Time
    // ----
    float Cur_Time_sec;


    // -----
    // Flags
    // -----
    int8_t UsedSBUSNum;
    int8_t Mode_MotorAct;
    int8_t Mode_FlightMode;
    int8_t Mode_VehicleFlyingStatus;


    // ---------------
    // ADC Information
    // ---------------
    float ADC_volt[6];


    // ----------------
    // GNSS Information
    // ----------------
    float  GNSS_SatNum;
    float  GNSS_hAcc_m;
    float  GNSS_vAcc_m;
    double GNSS_iTOW_msec;
    double GNSS_ECEF_m[3];
    double GNSS_LLH_degdegm[3];


    // ---------------------------------------
    // PosDown Information (Lidar & Barometer)
    // ---------------------------------------
    float LidarPosDown_m;
    float LidarPos_m;
    float BaroPosDown_m;


    // ----------------------------------
    // Optical Flow Information (PX4Flow)
    // ----------------------------------
    float FlowXY_mps[2];
    float SonarPosDown_m;
    float FlowQuality;


    // --------------------------
    // Transmitter Stick Position
    // --------------------------
    float TXStickPosAil_usec;
    float TXStickPosEle_usec;
    float TXStickPosThr_usec;
    float TXStickPosRud_usec;
    float TX_CH_8_9_13_14_15_16_usec[6];


    // ----------------------
    // Controller Information
    // ----------------------
    // Angular Velocity
    float Cmd_R_dps;
    float Cur_AngularRate_dps[3];
    // Attitude
    float Cmd_Att_deg[3];
    float Cur_Att_deg[3];
    // Linear Acceleration, AED
    float Cmd_LinAccAED_mpss[3];
    float Cur_LinAccAED_mpss[3];
    // Velocity, AED
    float Cmd_VelAED_mps[3];
    float Cur_VelAED_mps[3];


    // ---------
    // Actuation
    // ---------
    // Motor Actuation
    float ThrottleTrim_usec;
    float ThrottleOut_usec;
    float MotorPWM_usec[8];


    // -----------
    // Flight Path
    // -----------
    // NED Position
    float Cur_PosNED_m[3];
    // NED Velocity
    float Cur_VelNED_mps[3];


    // ------------------
    // Sensor Calibration
    // ------------------
    // Gyro Calibration
    float GyroTemper_C;
    float GyroRawXYZ_dps[3];
    // Accel Calibration
    float SpeciAccRawXYZ_mpss[3];
    // Magnetometer
    float MagRawXYZ_mG[3];


    // -------------
    // Sensor Status
    // -------------
    uint16_t SensorStatus;


    // --------------
    // Checksum Bytes
    // --------------
    uint8_t ChecksumA;
    uint8_t ChecksumB;

};
#pragma pack()


struct struct_t_MainFuncArgs
{
    const char *  PtrArr_Args[DF_MAIN_FUNC_ARGS_MAX];
    unsigned char Flag_Args[DF_MAIN_FUNC_ARGS_MAX];
    char          OnboardLogFileName[1025];
};

struct struct_Senddata         tx;
struct struct_t_MessageBody    tx_data;
struct struct_t_RXttyO         StrRXttyO;
struct struct_t_MainFuncArgs   StrMainFuncArgs;


float satmax(float data, float max)
{
    float res;

    if(fabs(data) > max)
        res = (data + eps)/fabs(data + eps)*max;
    else
        res = data;

    return res;
}


float satmin(float data, float min)
{
    float res;

    if(fabs(data) < min)
        res = (data + eps)/fabs(data + eps)*min;
    else
        res = data;

    return res;
}


#endif
