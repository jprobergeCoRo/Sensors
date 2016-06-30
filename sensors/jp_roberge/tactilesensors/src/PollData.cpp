/****************************************************************************************
//PollData V1.1
//Author: Jean-Philippe Roberge Ing, M.Sc.A.
//Date: April 2nd 2015
//Affiliations: Laboratoire de commande et de robotique (École de technologie supérieure)
//
//Description:  PollData.cpp - Source code of the ros package responsible for extracting
//              and publishing the static or dynamic data from a sensor. The results are
//              then published on /TactileSensor/StaticData or /TactileSensor/DynamicData
//              respectively. The static data are published in format
//              std_msgs::Int32MultiArray while dynamic data are pusblish in format
//              std_msgs::Float64. A requirement for this node to work properly is to
//              have a USB/RS485 converter attached to /dev/ttyUSB0.
//
//Synopsis:     rosrun tactilesensors PollData [-sensor n] [-data static|dynamic]
//
//              Where [OPTIONS]:
//                  '-sensor n | n1:n2 | n1,n2,n3,...' will indicate that the data have
//                  to be extracted from sensor n, or from sensors n1 to n2, or from
//                  sensors n1, n2, n3, ... . If this option is not provided, by
//                  default data will be extracted from sensor #1.
//                  '-data static | dynamic' specifies if we want to extract "static"
//                  data or "dynamic" data. By default, this node will extract static
//                  data.
//
//Examples:     1)  rosrun tactilesensors PollData -sensor 1,2,5
//              -This will publish static data from sensors 1,2 and 5 to the topic
//              /TactileSensor/StaticData
//              2)  rosrun tactilesensors PollData -sensors 3:7 -data dynamic
//              -This will publish dynamic data from sensors 3, 4, 5, 6 and 7 to the
//              topic /TactileSensor/DynamicData
//
//______________________________________________________________________________________
//Version: 1.0 : April 2nd 2015
//Version: 1.1 : June 9th 2015
//
//Last Modified: April 2nd 2015 - Initial release
//               June 9th 2015  - Modified to include acquisition of multiple sensors at
//                                the same time --> e.g. by adding option -sensor 1:10
//                                on the command line
****************************************************************************************/

#include <stdint.h>
#include "ros/ros.h"
#include "tactilesensors/StaticData.h"
#include "tactilesensors/DynamicData.h"
#include <stdio.h>      // standard input / output functions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <algorithm>

//Using std namespace
using namespace std;

//Function prototypes:
bool cmdOptionExists(char** begin, char** end, const string& option);
char* getCmdOption(char ** begin, char ** end, const string & option);
bool OpenAndConfigurePort(int *USB);
void CreateCRCTable(vector<int>* CRCTable);
bool GetStaticData(int *USB, int SensorToPoll, tactilesensors::StaticData *StaticData, vector<int>* CRCTable);
bool GetDynamicData(int *USB, int SensorToPoll, tactilesensors::DynamicData *DynamicData);

//Main
int main(int argc, char **argv)
{
    //Variable declarations:
    ros::init(argc, argv, "PollData");
    ros::NodeHandle n;
    tactilesensors::StaticData StaticData;
    tactilesensors::DynamicData DynamicData;
    ros::Publisher StaticData_pub = n.advertise<tactilesensors::StaticData>("TactileSensor/StaticData", 1000);
    ros::Publisher DynamicData_pub = n.advertise<tactilesensors::DynamicData>("TactileSensor/DynamicData", 1000);
    ros::Rate loop_rate(1000);
    vector<int> CRCTable(256);
    int USB,SensorToPoll=1,SensorStartIndex, SensorEndIndex,NumOfSensorsToAcquire=1;
    bool StaticMode=true,MultipleSensorsAcquisition;
    vector<int> SensorsToAcquire(10);

    //Parsing the "sensor" option from command line:
    if(cmdOptionExists(argv, argv+argc, "-sensor"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-sensor");
        if (filename)
        {
            //Check if ":" is in "-sensor"'s argument:
            string AString = filename;
            if (strchr(filename,':')!=NULL)
            {
                SensorStartIndex= atoi(strtok(filename,":"));
                SensorEndIndex= atoi(strtok (NULL, ":"));
                for (int i=SensorStartIndex;i<=SensorEndIndex;i++)
                {
                    SensorsToAcquire[NumOfSensorsToAcquire-1]=i;
                    NumOfSensorsToAcquire++;
                }
                NumOfSensorsToAcquire--;
                MultipleSensorsAcquisition = true;
            }
            else if(strchr(filename,',')!=NULL)
            {
                char *TMPString;
                SensorsToAcquire[NumOfSensorsToAcquire-1]=atoi(strtok(filename,","));
                while (TMPString != NULL)
                {
                    TMPString=strtok (NULL, ",");
                    if (TMPString != NULL)
                    {
                        SensorsToAcquire[NumOfSensorsToAcquire] = atoi(TMPString);
                    }
                    NumOfSensorsToAcquire++;
                }
                NumOfSensorsToAcquire--;
                MultipleSensorsAcquisition = true;
            }
            else
            {
                SensorToPoll=atoi(filename);
                MultipleSensorsAcquisition = false;
            }
        }
    }

    //Parsing the "data" option from command line:
    if(cmdOptionExists(argv, argv+argc, "-data"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-data");
        if (filename)
        {
            if(strcmp(filename,"dynamic")==0)
            {
                StaticMode=false;
            }
        }
    }

    // Creating CRC16 Reference Array:
    CreateCRCTable(&CRCTable);

    // Opening and Configuring /dev/ttyUSB0:
    if(!OpenAndConfigurePort(&USB)) return 1;

    //Now we enter the ros loop where we will acquire and publish data
    while (ros::ok())
    {
        if (StaticMode == true && MultipleSensorsAcquisition == false) // If we are acquiring static data (default behavior) with only one sensor
        {
            // We start by cleaning the array:
            StaticData.Data.thedata.assign(0);

            if(GetStaticData(&USB,SensorToPoll,&StaticData,&CRCTable)) //Get the static data from sensor
            {
                // We fill the message to be published:
                StaticData.SensorID=SensorToPoll;
                //We finally publish the results on the topic if acquisition was successful:
                StaticData_pub.publish(StaticData);
            }
        }
        else if (StaticMode == false && MultipleSensorsAcquisition == false) // If we are acquiring dynamic data with one sensor
        {
            if (GetDynamicData(&USB,SensorToPoll,&DynamicData))
            {
                DynamicData.SensorID=SensorToPoll;
                DynamicData_pub.publish(DynamicData); //Publish dynamic data to the dynamic data topic
            }
        }
        else if (StaticMode == true && MultipleSensorsAcquisition == true) // If we are acquiring static data with more than one sensors
        {
            for (int i=0;i<NumOfSensorsToAcquire;i++)
            {
                // We start by cleaning the array:
                StaticData.Data.thedata.assign(0);

                //We finally publish the results on the topic if the acquisition was successful:
                if (GetStaticData(&USB,SensorsToAcquire[i],&StaticData,&CRCTable)) //Get the static data from sensor
                {
                    // We fill the message to be published:
                    StaticData.SensorID=SensorsToAcquire[i];
                    StaticData_pub.publish(StaticData);
                }
            }
        }
        else if (StaticMode == false && MultipleSensorsAcquisition == true) // If we are acquiring dynamic data with more than one sensors
        {
            for (int i=0;i<NumOfSensorsToAcquire;i++)
            {
                if (GetDynamicData(&USB,SensorsToAcquire[i],&DynamicData))
                {
                    DynamicData.SensorID=SensorsToAcquire[i];
                    DynamicData_pub.publish(DynamicData); //Publish dynamic data to the dynamic data topic
                }
            }
        }
        ros::spinOnce();    //Refresh publishing buffers
        loop_rate.sleep();  //If we have time, let sleep for a while.
    }
    close(USB);  // close and free /dev/ttyUSB0 peripheral
    return 0;
}

/*****************************************************************************************
//Function: cmdOptionExists
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns true, otherwise it will return false.
//
*****************************************************************************************/
bool cmdOptionExists(char** begin, char** end, const string& option)
{
    return find(begin, end, option) != end;
}

/****************************************************************************************
//Function: getCmdOption
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns a pointer pointing just after the string that was found.
//
****************************************************************************************/
char* getCmdOption(char ** begin, char ** end, const string & option)
{
    char ** itr = find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

/****************************************************************************************
//Function: OpenAndConfigurePort
//
//Description:  This function opens the /dev/ttyUSB0 port with read and write access
//              rights (that suppose that the user has done "sudo chmod 777 /dev/ttyUSB0"
//              before. If it succeeds, it returns true, otherwise it returns false.
//
****************************************************************************************/
bool OpenAndConfigurePort(int *USB)
{
    /* Open File Descriptor */
    //    *USB = open( "/tmp/interceptty", O_RDWR| O_NOCTTY ); // For debugging purposes (JP)
    *USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

    /* Error Handling */
    if ( (*USB) < 0 )
    {
        cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << endl;
        return false;
    }

    /**** Configure Port ****/
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if ( tcgetattr ( (*USB), &tty ) != 0 ) {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return false;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B460800);
    cfsetispeed (&tty, (speed_t)B460800);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_lflag     &=  ~ICANON;            // Remove canonical mode
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  0;                  // read doesn't block
    tty.c_cc[VTIME]  =  2;                  // 0.5 seconds read timeout between each byte max
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Flush Port, then applies attributes */
    tcflush( (*USB), TCIFLUSH );
    if ( tcsetattr ( (*USB), TCSANOW, &tty ) != 0) {
        cout << "Error " << errno << " from tcsetattr" << endl;
        return false;
    }
    return true;
}

/****************************************************************************************
//Function: CreateCRCTable
//
//Description:  This function fills up a vector with CRC16 values. The resulting values
//              depend on the ID of the sensor that we want to interact with. At the end
//              the user can thus ask for a CRC16 values directly
//              (crc=CRCTable[SensorID]). The polynom that was used for crc16 calculation
//              is 0xA001.
//
****************************************************************************************/
void CreateCRCTable(vector<int> *CRCTable)
{
    uint16_t i,j,crc;
    for (i=0;i<256;i++) // The following is CRC16 calculation, using polynom "0xa001":
    {
        crc=831^i; // This is 0x033f=831, which is CRC16 for 250
        for(j=0;j<8;j++)
        {
            if(crc & 0x0001)
            {
                crc=(crc>>1)^0xa001;
            }
            else
            {
                crc>>=1;
            }
        }
        (*CRCTable)[i]=((crc & 0x00FF)<<8)+((crc & 0xFF00)>>8);
    }
}

/****************************************************************************************
//Function: GetStaticData
//
//Description:  This function acquires static data from the sensor. If it successfully
//              acquire data, it returns true, otherwise it returns false.
//
//Variables:    int *USB: A pointer thats points file descriptor attached to ttyUSB0
//                          device.
//              int SensorToPoll: SensorToPoll is the number of the sensor we want to
//                          acquire.
//              std_msgs::Int32MultiArray *StaticData: An array that we fill with the 12
//                          taxel values. This will be used later for publishing on the
//                          StaticData topic.
//              vector<int> *CRCTable: This is the CRC16 value table. We need that while
//                          asking the sensor for its static data.
****************************************************************************************/
bool GetStaticData(int *USB, int SensorToPoll, tactilesensors::StaticData *StaticData, vector<int> *CRCTable)
{
    //Variable declarations:
    int n_written, n_read, spot=0;
    char MessageToSend[5],StaticDataTMP[25];
    char buf = '\0';
    bool AcquisitionSuccess=false;

    // This is the request we have to send for static data to sensor SensorToPoll with the right crc16 values:
    MessageToSend[0]=(char)(250&0xFF);
    MessageToSend[1]=(char)(SensorToPoll&0xFF);
    MessageToSend[2]=(char)(((*CRCTable)[SensorToPoll]&0xFF00)>>8);
    MessageToSend[3]=(char)((*CRCTable)[SensorToPoll]&0xFF);
    MessageToSend[4]='\r';

    //Send the request:
    do {
        n_written = write( (*USB), &MessageToSend[spot], 1 );
        spot += n_written;
    } while ((MessageToSend[spot] != '\r') && (n_written > 0));

    spot = 0;

    //Read the answer, i.e. the bytes received from the sensor:
    do {
        n_read = read( (*USB), &buf, 1 );
        if (n_read >0)
        {
            StaticDataTMP[spot]=buf;
        }
        spot += n_read;
    } while(n_read > 0);

    if (n_read < 0) {
        cout << "Error reading: " << strerror(errno) << endl;
    }

    //Parse the bytes that were received, verifying the two first header bytes:
    if((StaticDataTMP[0] & 0xFF)*256 + (StaticDataTMP[1] & 0xFF)==(250*256+(SensorToPoll)))
    {
        if (spot==26) // If data size equals 12 ((26-2)/2), then data belongs to a proximal sensor
        {
            StaticData->Data.thedata[0]=((static_cast<unsigned int>(StaticDataTMP[6]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[7]) & 0xFF));      //1
            StaticData->Data.thedata[1]=((static_cast<unsigned int>(StaticDataTMP[4]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[5]) & 0xFF));      //2
            StaticData->Data.thedata[2]=((static_cast<unsigned int>(StaticDataTMP[2]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[3]) & 0xFF));      //3
            StaticData->Data.thedata[3]=((static_cast<unsigned int>(StaticDataTMP[12]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[13]) & 0xFF));    //4
            StaticData->Data.thedata[4]=((static_cast<unsigned int>(StaticDataTMP[10]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[11]) & 0xFF));    //5
            StaticData->Data.thedata[5]=((static_cast<unsigned int>(StaticDataTMP[8]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[9]) & 0xFF));      //6
            StaticData->Data.thedata[6]=((static_cast<unsigned int>(StaticDataTMP[18]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[19]) & 0xFF));    //7
            StaticData->Data.thedata[7]=((static_cast<unsigned int>(StaticDataTMP[16]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[17]) & 0xFF));    //8
            StaticData->Data.thedata[8]=((static_cast<unsigned int>(StaticDataTMP[14]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[15]) & 0xFF));    //9
            StaticData->Data.thedata[9]=((static_cast<unsigned int>(StaticDataTMP[24]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[25]) & 0xFF));    //10
            StaticData->Data.thedata[10]=((static_cast<unsigned int>(StaticDataTMP[22]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[23]) & 0xFF));    //11
            StaticData->Data.thedata[11]=((static_cast<unsigned int>(StaticDataTMP[20]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[21]) & 0xFF));    //12
            AcquisitionSuccess=true;
        }
        else if (spot==18) // If data size equals 8 ((18-2)/2), then data belongs to a medial sensor
        {

            StaticData->Data.thedata[0]=((static_cast<unsigned int>(StaticDataTMP[4]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[5]) & 0xFF));      //1
            StaticData->Data.thedata[1]=((static_cast<unsigned int>(StaticDataTMP[2]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[3]) & 0xFF));      //2
            StaticData->Data.thedata[2]=((static_cast<unsigned int>(StaticDataTMP[8]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[9]) & 0xFF));      //3
            StaticData->Data.thedata[3]=((static_cast<unsigned int>(StaticDataTMP[6]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[7]) & 0xFF));      //4
            StaticData->Data.thedata[4]=((static_cast<unsigned int>(StaticDataTMP[12]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[13]) & 0xFF));    //5
            StaticData->Data.thedata[5]=((static_cast<unsigned int>(StaticDataTMP[10]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[11]) & 0xFF));    //6
            StaticData->Data.thedata[6]=((static_cast<unsigned int>(StaticDataTMP[16]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[17]) & 0xFF));    //7
            StaticData->Data.thedata[7]=((static_cast<unsigned int>(StaticDataTMP[14]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[15]) & 0xFF));    //8
            AcquisitionSuccess=true;
        }
        else if (spot==20) // If data size equals 9 ((20-2)/2), then data belongs to a distal sensor
        {
            StaticData->Data.thedata[0]=((static_cast<unsigned int>(StaticDataTMP[18]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[19]) & 0xFF));    //1
            StaticData->Data.thedata[1]=((static_cast<unsigned int>(StaticDataTMP[12]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[13]) & 0xFF));    //2
            StaticData->Data.thedata[2]=((static_cast<unsigned int>(StaticDataTMP[6]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[7]) & 0xFF));      //3
            StaticData->Data.thedata[3]=((static_cast<unsigned int>(StaticDataTMP[16]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[17]) & 0xFF));    //4
            StaticData->Data.thedata[4]=((static_cast<unsigned int>(StaticDataTMP[10]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[11]) & 0xFF));    //5
            StaticData->Data.thedata[5]=((static_cast<unsigned int>(StaticDataTMP[4]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[5]) & 0xFF));      //6
            StaticData->Data.thedata[6]=((static_cast<unsigned int>(StaticDataTMP[14]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[15]) & 0xFF));    //7
            StaticData->Data.thedata[7]=((static_cast<unsigned int>(StaticDataTMP[8]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[9]) & 0xFF));      //8
            StaticData->Data.thedata[8]=((static_cast<unsigned int>(StaticDataTMP[2]) & 0xFF)*256+(static_cast<unsigned int>(StaticDataTMP[3]) & 0xFF));      //9
            AcquisitionSuccess=true;
        }
    }
    return AcquisitionSuccess;
}


/****************************************************************************************
//Function: GetDynamicData
//
//Description:  This function acquires dynamic data from the sensor. If it successfully
//              acquire data, it returns true, otherwise it returns false.
//
//Variables:    int *USB: A pointer thats points file descriptor attached to ttyUSB0
//                          device.
//              int SensorToPoll: SensorToPoll is the number of the sensor we want to
//                          acquire.
//              std_msgs::Float64 *DynamicData: Float64 is the same typedef as "double"
//                          in c++. We use DynamicData to store the dynamic data that was
//                          read from the sensor.This will be used later for publishing
//                          on the StaticData topic.
****************************************************************************************/
bool GetDynamicData(int *USB, int SensorToPoll, tactilesensors::DynamicData *DynamicData)
{
    //Variable declarations:
    int n_written, n_read, spot=0;
    char MessageToSend[4],DynamicDataTMP[4];
    char buf = '\0';
    bool AcquisitionSuccess=false;

    //This is the request we have to send for dynamic data to sensor SensorToPoll:
    MessageToSend[0]=(char)(250&0xFF);
    MessageToSend[1]=(char)((128+SensorToPoll)&0xFF);
    MessageToSend[2]='\r';

    //Send the request:
    do {
        n_written = write( (*USB), &MessageToSend[spot], 1 );
        spot += n_written;
    } while ((MessageToSend[spot] != '\r') && (n_written > 0));

    spot = 0;

    //Read the answered bytes from the sensor:
    do {
        n_read = read( (*USB), &buf, 1 );
        if (n_read >0)
        {
            DynamicDataTMP[spot]=buf;
        }
        spot += n_read;
    } while(n_read > 0 && spot<4);

    //Parse the received bytes and compute the resulting dynamic data:
    if(((DynamicDataTMP[0] & 0xFF)*256 + (DynamicDataTMP[1] & 0xFF)==(250*256+(128+SensorToPoll)))&&(spot>=4))
    {
        DynamicData->Data = (double)static_cast<signed int16_t>((DynamicDataTMP[2] & 0xFF)*256 + (DynamicDataTMP[3] & 0xFF))*((1.024*2) / 65535);
        AcquisitionSuccess=true;
    }
    else
    {
        AcquisitionSuccess=false;
    }

    if (n_read < 0) {
        cout << "Error reading: " << strerror(errno) << endl;
    }
    return AcquisitionSuccess;
}
