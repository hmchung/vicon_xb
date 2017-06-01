//============================================================================
// Name        : Vicon Xbee Communication
// Author      : Britsk Nguyen
// Version     :
// Copyright   : Love me if you use my code
// Description : ros node to communicate with vicon via serial interface
//============================================================================
//#define ROS_MASTER_URI		"http://localhost:11311"
//#define ROS_ROOT		"/opt/ros/indigo/share/ros"
#include <iostream>
#include "stdio.h"
#include <stdlib.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <cstring>
#include <unistd.h>
#include <unistd.h>
#include <sstream>
#include <signal.h>
#include <math.h>
#include <vicon_xb/viconPoseMsg.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign/list_of.hpp>
#include <errno.h>
#include <serial/serial.h>
#include "vicon_xb/viconXbSrv.h"
//For log file use
#include <fstream>
#include <pwd.h>
#include <glob.h>   //To search for port

//#include <mavros/Imu.h>

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

#define USE_MIT_SERIAL
#define BAUDRATE            57600
#define BAUD_MACRO          B57600
#define DFLT_PORT           "/dev/ttyUSB0"
#define RCV_THRESHOLD       46					//Receive frame's length
#define FPS                 40					//Frames per second
#define DFLT_NODE_RATE      20                  //Update frequency
#define DFLT_COMM_TIMEOUT   250
#define	UP                  0xFF
#define DOWN                0x00
#define HEADER_NAN          0x7FAAAAAA

using namespace std;
serial::Serial fd;

uint8_t		rcvdFrame[RCV_THRESHOLD];
uint32_t	seqCount = 0;
uint32_t    viconStartTimestamp;
ros::Time   viconRosStartTimestamp;

#define VICON_MSG_HDR       *(uint32_t *)rcvdFrame
#define VICON_MSG_X         *(float *)(rcvdFrame + 4)
#define VICON_MSG_Y         *(float *)(rcvdFrame + 8)
#define VICON_MSG_Z         *(float *)(rcvdFrame + 12)
#define VICON_MSG_XD        *(float *)(rcvdFrame + 16)
#define VICON_MSG_YD        *(float *)(rcvdFrame + 20)
#define VICON_MSG_ZD        *(float *)(rcvdFrame + 24)
#define VICON_MSG_ROLL      *(float *)(rcvdFrame + 28)
#define VICON_MSG_PITCH     *(float *)(rcvdFrame + 32)
#define VICON_MSG_YAW       *(float *)(rcvdFrame + 36)
#define VICON_MSG_STMP      *(uint32_t *)(rcvdFrame + 40)
#define VICON_MSG_CSA       *(uint8_t *)(rcvdFrame + 44)
#define VICON_MSG_CSB       *(uint8_t *)(rcvdFrame + 45)

int synchronize(ros::Rate rate, double timeOut);

bool logEnable = false;
bool publishEnable = true;
std::ofstream viconLog;

uint32_t viconPoseCount = 0;
bool blocked = false;
int fileIndx = -1;

bool viconXbSrvCallback(vicon_xb::viconXbSrv::Request &req, vicon_xb::viconXbSrv::Response &res)
{
    blocked = req.block;
    fileIndx = req.logfile_idx;
    ROS_INFO("vicon service request received: b/u=%d\tfileIndx=%d\n", blocked, fileIndx);
    res.result = 0;
    return true;
}

bool fileExists(const std::string& filename);

int main(int argc, char **argv)
{
    //Create ros handler to node
    ros::init(argc, argv, "viconXbeeNodeHandle");
    ros::NodeHandle viconXbeeNodeHandle("~");

    string serialPortName = string(DFLT_PORT);
    if(viconXbeeNodeHandle.getParam("viconSerialPort", serialPortName))
        printf(KBLU"Retrieved value %s for param 'viconSerialPort'!\n"RESET, serialPortName.data());
    else
    {
        printf(KRED "Couldn't retrieve param 'viconSerialPort', program closed!\n"RESET);
        return 0;
    }
    int viconNodeRate = DFLT_NODE_RATE;
    if(viconXbeeNodeHandle.getParam("viconNodeRate", viconNodeRate))
        printf(KBLU"Retrieved value %d for param 'viconnNodeRate'\n"RESET, viconNodeRate);
    else
        printf(KYEL "Couldn't retrieve param 'viconnNodeRate', applying default value %dHz\n"RESET, viconNodeRate);

    //ros Rate object to control the update rate
    ros::Rate rate = ros::Rate(viconNodeRate);
    if(viconXbeeNodeHandle.getParam("blocked", blocked))
    {
        if(blocked)
            printf(KBLU"Blocked until unblocked.!\n"RESET);
        else
            printf(KBLU"Not blocked!\n"RESET);
    }
    else
    {
        printf(KRED "Couldn't retrieve param 'blocked'. No blocking by default."RESET);
        blocked = false;
    }

    ros::ServiceServer viconServer = viconXbeeNodeHandle.advertiseService("/vicon_xb_srv", viconXbSrvCallback);

    while(ros::ok() && blocked)
    {
        printf("vicon_xb waiting to get unlocked.\n");
        ros::spinOnce();
        rate.sleep();
    }

    int viconCommTimeout = DFLT_COMM_TIMEOUT;
    if(viconXbeeNodeHandle.getParam("viconCommTimeout", viconCommTimeout))
        printf(KBLU"Retrieved value %d [ms] for param 'viconCommTimeout'\n"RESET, viconCommTimeout);
    else
        printf(KYEL "Couldn't retrieve param 'viconCommTimeout', applying default value %dms\n"RESET, viconCommTimeout);

    bool pollPort = true;
    if(viconXbeeNodeHandle.getParam("viconCommTimeout", viconCommTimeout))
        printf(KBLU"Retrieved value %d [ms] for param 'viconCommTimeout'\n"RESET, viconCommTimeout);
    else
        printf(KYEL "Couldn't retrieve param 'viconCommTimeout', applying default value %dms\n"RESET, viconCommTimeout);


    ros::Publisher viconPosePublisher = viconXbeeNodeHandle.advertise<vicon_xb::viconPoseMsg>("viconPoseTopic", 1);

    //-------------------------------Initialize Serial Connection---------------------------------
    glob_t glob_results;
    while(glob(serialPortName.c_str(), 0, NULL, &glob_results) == GLOB_NOMATCH)
    {
        printf("Port \"%s\" not found! Checking after 0.5s\n"RESET, serialPortName.data());
        ros::Duration(0.5).sleep();
    }

    fd.setPort(serialPortName);
    fd.setBaudrate(BAUDRATE);
    fd.setTimeout(5, 10, 0, 10, 0);
    fd.open();
    if (fd.isOpen())
    {
        fd.flushInput();
        printf(KBLU "Connection established\n\n" RESET);
    }
    else
    {
        printf(KRED "serialInit: Failed to open port %s..\n" RESET, serialPortName.data());
        ros::Duration(0.25).sleep();
    }

    //Looping to catch the intial of frame
    while( synchronize(rate, viconCommTimeout/1000.0) < 0)
        printf("Timeout synchronizing with stream.");
    //-----------------------------------Serial Initialization-------------------------------------//

    //---------------------------------------Logging & Publish configuration-------------------------------------------//
    if(viconXbeeNodeHandle.getParam("logEnable", logEnable))
    {
        if(logEnable)
            printf(KBLU"Logging enabled!\n"RESET);
        else
            printf(KBLU"Logging disabled!\n"RESET);
    }
    else
    {
        printf(KRED "Couldn't retrieve param 'logEnable'. Not used by default."RESET);
        logEnable = false;
    }

    if(viconXbeeNodeHandle.getParam("publishEnable", publishEnable))
    {
        if(publishEnable)
            printf(KBLU"Publishing enabled!\n"RESET);
        else
            printf(KBLU"Publishing disabled!\n"RESET);
    }
    else
    {
        printf(KRED "Couldn't retrieve param 'publishEnable'. 'True' by default."RESET);
        publishEnable = false;
    }

    //Create a log file
    struct passwd *pw = getpwuid(getuid());
    if(logEnable && fileIndx == -1)
    {
        std::stringstream filename;
        int filecount = 1;
        filename << string(pw->pw_dir) << "/vicon_log/vclog" << filecount << ".m";
        while(fileExists(filename.str()))
        {
            filecount++;
            filename.clear();
            filename.str("");
            filename << string(pw->pw_dir) << "/vicon_log/vclog" << filecount << ".m";
        }
        std::string ss = filename.str();
        viconLog.open(ss.c_str(), std::ofstream::out | std::ofstream::app);
        viconLog.precision(10);
        cout << "log_file: " << ss.c_str() << endl;
        viconLog << "X=" << VICON_MSG_X << ";Y=" << VICON_MSG_Y << ";Z=" << VICON_MSG_Z
                 << ";Xd=" << VICON_MSG_XD << ";Yd=" << VICON_MSG_YD << ";Zd=" << VICON_MSG_ZD
                 << ";Ro=" << VICON_MSG_ROLL << ";Pi=" << VICON_MSG_PITCH << ";Ya=" << VICON_MSG_YAW << ";tv=0;" << "tvr=" << ros::Time::now() << ";" << endl;
    }
    else if(logEnable)
    {
        std::stringstream filename;
        filename << string(pw->pw_dir) << "/vicon_log/vclog" << fileIndx << ".m";
        std::string ss = filename.str();
        viconLog.open(ss.c_str(), std::ofstream::out | std::ofstream::app);
        viconLog.precision(10);
        cout << "log_file: " << ss.c_str() << endl;
        viconLog << "X=" << VICON_MSG_X << ";Y=" << VICON_MSG_Y << ";Z=" << VICON_MSG_Z
                 << ";Xd=" << VICON_MSG_XD << ";Yd=" << VICON_MSG_YD << ";Zd=" << VICON_MSG_ZD
                 << ";Ro=" << VICON_MSG_ROLL << ";Pi=" << VICON_MSG_PITCH << ";Ya=" << VICON_MSG_YAW << ";tv=0;" << "tvr=" << ros::Time::now() << ";" << endl;
    }

    //---------------------------------------Logging & Publish configuration-------------------------------------------//

    viconStartTimestamp = VICON_MSG_STMP;
    viconRosStartTimestamp = ros::Time::now();
    viconLog << "tv0=" << viconRosStartTimestamp << ";tv0v=" << viconStartTimestamp << ";" << endl;

    ros::Time lastMsgTime = ros::Time::now();
    //start reading and publishing
    while(ros::ok())
    {
        uint32_t bytes_avail = fd.available();
        //printf("Bytes detected: %d\n", fd.available());
        //Extract and publish all messages if there are enough bytes in waiting

            if(bytes_avail >= RCV_THRESHOLD)
                while(fd.available() >= RCV_THRESHOLD)
                {
                    fd.read(rcvdFrame, RCV_THRESHOLD);
                    //Since intial synchronization, first 4 bytes must be headers otherwise there must have been an error
                    uint32_t headerWord = VICON_MSG_HDR;
                    if(headerWord != HEADER_NAN)
                    {
                        //If there is some error, call the synchronize procedure
                        printf("Lost synchronization. Attempting to synchronize back.\n");
                        if( synchronize(rate, viconCommTimeout/1000.0) < 0)
                            printf("Timeout synchronizing with stream. OMG!");
                        else
                        {
                            printf("Synchronization recovered.\n");
                            lastMsgTime = ros::Time::now();
                        }
                    }
                    else
                    {
                        lastMsgTime = ros::Time::now();

                        //All things are good so far
                        printf(KBLU"Aligned Header found, "RESET);
                       // for(int i = 0; i < RCV_THRESHOLD; i++)
                       //     printf(KGRN"%2x ", rcvdFrame[i]);
                       // printf("\n"RESET);

                        //Checksum
                        bool validCRC = false;
                        uint8_t CSA = 0, CSB = 0;
                        for(uint8_t i = sizeof(HEADER_NAN); i < RCV_THRESHOLD - 2; i++)
                        {
                            CSA = CSA + rcvdFrame[i];
                            CSB = CSB + CSA;
                        }

                        if(CSA == VICON_MSG_CSA && CSB == VICON_MSG_CSB)
                        {
                            validCRC = true;
                            printf(KBLU"CRC passed: X=%6.3f Y=%6.3f Z=%6.3f Xd=%6.3f Yd=%6.3f Zd=%6.3f Ro=%6.3f Pi=%6.3f Ya=%6.3f Tv=%d.\n"RESET,
                                   VICON_MSG_X,
                                   VICON_MSG_Y,
                                   VICON_MSG_Z,
                                   VICON_MSG_XD,
                                   VICON_MSG_YD,
                                   VICON_MSG_ZD,
                                   VICON_MSG_ROLL,
                                   VICON_MSG_PITCH,
                                   VICON_MSG_YAW,
                                   VICON_MSG_STMP);
                        }
                        else
                            printf(KRED"CRC error, msg discarded!!\n"RESET);

                        if(validCRC)
                        {
                            double timeDiff = (int32_t)(VICON_MSG_STMP - viconStartTimestamp)/1000.0;
                            if(timeDiff < 0)
                            {
                                printf(KYEL"Vicon time must have reset.\n");
                                viconStartTimestamp = VICON_MSG_STMP;
                                viconRosStartTimestamp = ros::Time::now();
                            }

                            ros::Duration timeStamp = ros::Duration((VICON_MSG_STMP - viconStartTimestamp)/1000.0);

                            if(publishEnable)
                            {
                                vicon_xb::viconPoseMsg viconPose;
                                viconPose.header.seq = seqCount++;
                                viconPose.header.stamp = ros::Time::now();
                                viconPose.header.frame_id = "/vicon_frame";
                                viconPose.pose.position.x = VICON_MSG_X;
                                viconPose.pose.position.y = VICON_MSG_Y;
                                viconPose.pose.position.z = VICON_MSG_Z;
                                
                                tf::Quaternion qV =  tf::Quaternion(sin(VICON_MSG_ROLL/2), 0, 0, cos(VICON_MSG_ROLL/2))
                                                    *tf::Quaternion(0, sin(VICON_MSG_PITCH/2), 0, cos(VICON_MSG_PITCH/2))
                                                    *tf::Quaternion(0, 0, sin(VICON_MSG_YAW/2), cos(VICON_MSG_YAW/2));

                                viconPose.pose.orientation.x = qV.x();
                                viconPose.pose.orientation.y = qV.y();
                                viconPose.pose.orientation.z = qV.z();
                                viconPose.pose.orientation.w = qV.w();

                                viconPose.vel.x = VICON_MSG_XD;
                                viconPose.vel.y = VICON_MSG_YD;
                                viconPose.vel.z = VICON_MSG_ZD;

                                viconPose.eulers.x = VICON_MSG_ROLL;
                                viconPose.eulers.y = VICON_MSG_PITCH;
                                viconPose.eulers.z = VICON_MSG_YAW;

                                viconPosePublisher.publish(viconPose);

                            }

                            if(logEnable)
                            {
                                viconLog << "X=[X " << VICON_MSG_X << "];Y=[Y " << VICON_MSG_Y <<"];Z=[Z " << VICON_MSG_Z <<"];"
                                         << "Xd=[Xd " << VICON_MSG_XD <<"];Yd=[Yd " << VICON_MSG_YD << "];Zd=[Zd " << VICON_MSG_ZD << "];"
                                         << "Ro=[Ro " << VICON_MSG_ROLL << "];Pi=[Pi " << VICON_MSG_PITCH <<"];Ya=[Ya " << VICON_MSG_YAW <<"];"
                                         << "tv=[tv " << ros::Time::now() << "];" << "tvr=[tvr " << ros::Time::now() << "];" << endl;

                            }
                        }
                    }
                }
            else
            {
                printf(KBLU"No message has arrived...\n"RESET);
                if( (ros::Time::now() - lastMsgTime).toSec() > viconCommTimeout/1000.0)
                {
                    printf(KYEL "PORT RESET DUE TO TIMEOUT\n" RESET);
                    fd.close();
                    fd.open();
                }
            }
        printf("Bytes still buffered: %d\n\n", fd.available());
        rate.sleep();
    }
    return 0;
}

int synchronize(ros::Rate rate, double timeOut)
{
    ros::Time synchStartTime = ros::Time::now();
    while(ros::ok())
    {
        uint32_t bytes_avail = fd.available();
        printf("Bytes in buffer%d\n", bytes_avail);

        if(bytes_avail < RCV_THRESHOLD*2)
            rate.sleep();
        else
        {
            //dummy read to clean up the buffer
            fd.read(rcvdFrame, 4);
            uint32_t headerWord = VICON_MSG_HDR;
            while((headerWord != HEADER_NAN) || (bytes_avail >= RCV_THRESHOLD*2))
            {
                bytes_avail = fd.available();
                headerWord = (headerWord >> 8) & (0x00FFFFFF);
                fd.read(rcvdFrame, 1);
                headerWord = headerWord | (rcvdFrame[0] << 24);
                printf("%x\n", headerWord);
                if(headerWord == HEADER_NAN)
                {
                    VICON_MSG_HDR = HEADER_NAN;
                    fd.read(rcvdFrame + 4, RCV_THRESHOLD - 4);
                    headerWord = HEADER_NAN;
                }
            }
            break;
        }

        if((ros::Time::now() - synchStartTime).toSec() > timeOut)
            return -1;
    }
    return (int)((ros::Time::now() - synchStartTime).toSec()*1000);
}

bool fileExists(const std::string& filename)
{

    struct stat buf;
    if(stat(filename.c_str(), &buf) != -1) {
    return true;
    }
    return false;
}
