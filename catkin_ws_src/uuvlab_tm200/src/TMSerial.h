#ifndef TMSERIAL_H
#define TMSERIAL_H

//linux serial lib:
#include <stdio.h>
#include "serialib/serialib.h"

// To use the communication library, we need to include the following
// two header files:
#include "EasyProfile/EasyObjectDictionary.h"
#include "EasyProfile/EasyProfile.h"

//ros message header file
#include "uuvlab_msgs/Imu.h"

class TMSerial
{
public:
    TMSerial();
    ~TMSerial();

private:
    serialib* serialib1; //we use linux serialib to interface with serial port.

public:
    char Serialport_Open(std::string device_type);
    void On_SerialRX(uuvlab_msgs::Imu* msg_imu);  // RX (Receive Sensor Data)

};

#endif // TMSERIAL_H
