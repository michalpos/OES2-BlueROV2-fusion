#define USE_USBCON //Use this for Arduino Due
#include <ros.h>
#include <uuvlab_msgs/ThrusterPWMs.h>
#include <Servo.h>

//Init servos and callbacks
Servo servos[8];

float T200controlmap(float speedRad)
{
  return speedRad + 1500.0;
}

//T200
void tservo( const uuvlab_msgs::ThrusterPWMs& msg){
  servos[0].writeMicroseconds(T200controlmap(msg.t0));
  servos[1].writeMicroseconds(T200controlmap(msg.t1));
  servos[2].writeMicroseconds(T200controlmap(msg.t2));
  servos[3].writeMicroseconds(T200controlmap(msg.t3));
  servos[4].writeMicroseconds(T200controlmap(msg.t4));
  servos[5].writeMicroseconds(T200controlmap(msg.t5));
  servos[6].writeMicroseconds(T200controlmap(msg.t6));
  servos[7].writeMicroseconds(T200controlmap(msg.t7));
}

//Init rosserial node
ros::NodeHandle nh;

ros::Subscriber<uuvlab_msgs::ThrusterPWMs> s0("thrusters/pwm", &tservo);
void setup()
{
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(s0);
  
  //nh.advertiseService(server);
  servos[0].attach(6, 1100, 1900); //Thrust 1
  servos[1].attach(7, 1100, 1900);//Thrust 2
  servos[2].attach(8, 1100, 1900);//Thrust 3
  servos[3].attach(9, 1100, 1900);//Thrust 4
  servos[4].attach(10, 1100, 1900); //Thrust 5
  servos[5].attach(11, 1100, 1900); //Thrust 6
  servos[6].attach(12, 1100, 1900); //Thrust 7
  servos[7].attach(13, 1100, 1900); //Thrust 8

 
  //Attempt at arming sequence
  for(int i = 0; i < 8; i += 1) //Zero signala
  { 
    servos[i].writeMicroseconds(1500);
  }
  delay(3000);
}

void loop()
{
  nh.spinOnce();
}
