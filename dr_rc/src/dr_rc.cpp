#include "DefineList.h"
#include "CommModule.h"
#include "SystemModule.h"

// for topic message
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/Twist.h>

// setup the initial name
using namespace ros;
using namespace std;

//void RX_FCC_Data_Publish(void);

#define  ROS_FREQ  20.0    // while loop frequency [Hz]
#define  DEBUG_MODE 1

std::string g_port;
int g_baudrate;

std_msgs::Float32MultiArray RX_FCC_Data;

geometry_msgs::Twist Remote2Twist(int _throt, int _rud, int _mode) // _mode (1100: auto / 1520: manual / 1940: estop) // Throttle range : 1115~1940 // Rudder : 1100~1940
{
	geometry_msgs::Twist tmp;
	if(_mode < 1200) {
	  tmp.angular.z = double(_rud-1520) / double(1940-1520) * -0.3; 
	  tmp.linear.x = double(_throt-1115) / double(1940-1115);

	  if(tmp.linear.x < 0)
	    tmp.linear.x = 0;

	  ROS_INFO("forward");
	}
	else if(_mode < 1600) {
	  tmp.angular.z = double(_rud-1520) / double(1940-1520) * -0.3; 
	  tmp.linear.x = -(double(_throt-1115) / double(1940-1115));

	  if(tmp.linear.x > 0)
	    tmp.linear.x = 0;

	  ROS_INFO("backward");
	}
	else {
	  tmp.angular.z = 0;
	  tmp.linear.x = 0;
	  ROS_INFO("E-STOP");
	}


	return tmp;
}

int Agent_Number = 29;

// node main loop, for ROS
int main(int argc, char** argv)
{
//    System_Initialization(argc, argv);       // System initialization

    init(argc, argv, "fcc_serial_node");          // node    name initialization
    NodeHandle nh;                           // assign node handler
    NodeHandle nh_("~");                           // assign node handler

    nh_.param<std::string>("port", g_port, "/dev/ttyUSB0");
    nh_.param("baudrate", g_baudrate, 921600);

    printf("Initiate: fcc_serial_node\n");   // for debugging

    // Publish Topic
    ros::Publisher RX_FCC_Data_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);

    printf("Initiate: publish rostopic </UAV/FCC>\n");   // for debugging

    FdPort1 = OpenSerial(g_port.c_str());             // Open Serial
//    FdPort1 = OpenSerial("/dev/tty");             // Open Serial
    SerialReceive(FdPort1);                  // Serial Receive (pthread)
    Rate loop_rate(ROS_FREQ);                      // setup the loop speed, [Hz]

    Initialization();                        // Mission Initialization

    // node loop, for ROS, check ros status, ros::ok()
    while( ok() )
    {
        UpdateCommand();

        // Serial TX part
        SerialSend(FdPort1);

        count_ros = count_ros + 1;
        t_cur = count_ros/ROS_FREQ;

	

        RX_FCC_Data.data.resize(3);
        RX_FCC_Data.data[0] = StrRXttyO.TXStickPosThr_usec; //
        RX_FCC_Data.data[1] = StrRXttyO.TXStickPosRud_usec; //
        RX_FCC_Data.data[2] = StrRXttyO.TX_CH_8_9_13_14_15_16_usec[0]; //

	geometry_msgs::Twist tmp;

	tmp = Remote2Twist(RX_FCC_Data.data[0],RX_FCC_Data.data[1],RX_FCC_Data.data[2]);

        RX_FCC_Data_pub.publish(tmp);

#if DEBUG_MODE
        printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
        printf("\t[USRG] Serial data from FCC\n");
        printf("[count_ros] \t %d \t[time_ros] \t %.2f\n", count_ros, count_ros/20.0);
        printf("[Agent_Number] \t %d \t[time_fcc] \t %.3f \t[Motor] \t %d\n", Agent_Number, StrRXttyO.Cur_Time_sec, StrRXttyO.Mode_MotorAct);
        printf("[FlightMode] \t %d \t[VFlyingStat] \t %d \t[Voltage] \t %.2f \t[Current] \t %.2f\n", StrRXttyO.Mode_FlightMode, StrRXttyO.Mode_VehicleFlyingStatus, StrRXttyO.ADC_volt[0], StrRXttyO.ADC_volt[1]);
        printf("\n");
        printf("[Sat_Number] \t %2.f \t[Lon] \t %3.7f \t[Lat] \t %2.7f \t[Height] \t %3.3f\n", StrRXttyO.GNSS_SatNum, StrRXttyO.GNSS_LLH_degdegm[0], StrRXttyO.GNSS_LLH_degdegm[1], StrRXttyO.GNSS_LLH_degdegm[2]);
        printf("\n");
        printf("[roll_rate]  \t %2.2f \t[pitch_rate] \t %2.2f \t[yaw_rate] \t %3.2f\n", StrRXttyO.Cur_AngularRate_dps[0], StrRXttyO.Cur_AngularRate_dps[1], StrRXttyO.Cur_AngularRate_dps[2]);
        printf("[roll]  \t %2.2f \t[pitch] \t %2.2f \t[yaw] \t %3.2f\n", StrRXttyO.Cur_Att_deg[0], StrRXttyO.Cur_Att_deg[1], StrRXttyO.Cur_Att_deg[2]);
        printf("\n");
        printf("[LinAcc_0]  \t %2.2f \t[LinAcc_1] \t %2.2f \t[LinAcc_1] \t %3.2f\n", StrRXttyO.Cur_LinAccAED_mpss[0], StrRXttyO.Cur_LinAccAED_mpss[1], StrRXttyO.Cur_LinAccAED_mpss[2]);
        printf("[Vel_0]  \t %2.2f \t[Vel_1] \t %2.2f \t[Vel_2] \t %3.2f\n", StrRXttyO.Cur_VelAED_mps[0], StrRXttyO.Cur_VelAED_mps[1], StrRXttyO.Cur_VelAED_mps[2]);
        printf("\n");
        printf("[North_m]  \t %2.2f \t[East_m] \t %2.2f \t[Down_m] \t %3.2f\n", StrRXttyO.Cur_PosNED_m[0], StrRXttyO.Cur_PosNED_m[1], StrRXttyO.Cur_PosNED_m[2]);
        printf("[Vel_N_m]  \t %2.2f \t[Vel_E_m] \t %2.2f \t[Vel_D_m] \t %3.2f\n", StrRXttyO.Cur_VelNED_mps[0], StrRXttyO.Cur_VelNED_mps[1], StrRXttyO.Cur_VelNED_mps[2]);
        printf("\n");
        printf("[SensorStatus] %d\n", StrRXttyO.SensorStatus);
        printf("[stick_Ail] \t %4.f \t[stick_Ele] \t %4.f \t[stick_Thr] \t %4.f \t[stick_Rud] \t %4.f\n", StrRXttyO.TXStickPosAil_usec, StrRXttyO.TXStickPosEle_usec, StrRXttyO.TXStickPosThr_usec, StrRXttyO.TXStickPosRud_usec);
	printf("[stick_CH8] \t %4.f \t[stick_CH9] \t %4.f \t[stick_CH13] \t %4.f\n", StrRXttyO.TX_CH_8_9_13_14_15_16_usec[0], StrRXttyO.TX_CH_8_9_13_14_15_16_usec[1], StrRXttyO.TX_CH_8_9_13_14_15_16_usec[2]);
	printf("[stick_CH14] \t %4.f \t[stick_CH15] \t %4.f \t[stick_CH16] \t %4.f\n", StrRXttyO.TX_CH_8_9_13_14_15_16_usec[3], StrRXttyO.TX_CH_8_9_13_14_15_16_usec[4], StrRXttyO.TX_CH_8_9_13_14_15_16_usec[5]);

        printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
        printf("\n");
#endif

        // loop rate [Hz]
        loop_rate.sleep();

	// loop sampling, ros
        spinOnce();
    }

    // for debugging
    printf("Terminate: FCC_Serial_node\n");

    return 0;
}




void Initialization(void)
{

}


void UpdateCommand(void)
{

    // Command  TX2 ---> DS_Board

    // flagA = 0   : default        (None)
    // flagA = 1   : Auto take-off  (be careful!!!!!)
    // flagA = 4   : Motor Cut-off

    // velx        : forward(+) backward(-) (Body-axis)
    // vely        : right  (+) left    (-) (Body-axis)
    // velz        : down   (+) up      (-) (Body-axis)
    // cmdr        : right  (+) left    (-) (Body-axis)  **yaw rate command

    flagA = 0;
    flagB = 0;
    flagC = 0;
    flagD = 0;

    velx = 0.0;
    vely = 0.0;
    velz = 0.0;
    cmdr = 0.0;


    // tx_data update
    tx_data.FlagA      = flagA;
    tx_data.FlagB      = flagB;
    tx_data.FlagC      = flagC;
    tx_data.FlagD      = flagD;

    tx_data.CmdVelAil  = satmax(vely,Vx_max);
    tx_data.CmdVelEle  = satmax(velx,Vx_max);
    tx_data.CmdVelDown = satmax(velz,Vz_max);
    tx_data.CmdR_dps   = cmdr;

    unsigned char *data = (unsigned char *)&tx_data;
    memcpy((void *)(tx.Data),(void *)(data),sizeofdata);

}
