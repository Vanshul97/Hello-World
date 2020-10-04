/*
 * CAN Bus Interface for rosserial
 * Noothless, 23/02/2020
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <mcp_can.h>
#include <SPI.h>
#include <std_msgs/Int32.h>

//CAN --------------------

MCP_CAN CAN0(10);

unsigned char data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//-------------------------

//ROS ---------------------

ros::NodeHandle  nh;

void Axis1Position( const std_msgs::Int32& msg){
  sendCANmsg_int(0x01, 0x00C, 0, 8, msg.data);
}

void Axis1Velocity( const std_msgs::Int32& msg){
  sendCANmsg_float(0x01, 0x00F, 0, 8, msg.data);
}

void Axis2Position( const std_msgs::Int32& msg){
  sendCANmsg_int(0x02, 0x00C, 0, 8, msg.data);
}

void Axis2Velocity( const std_msgs::Int32& msg){
  sendCANmsg_float(0x02, 0x00F, 0, 8, msg.data);
}

void Axis3Position( const std_msgs::Int32& msg){
  sendCANmsg_int(0x03, 0x00C, 0, 8, msg.data);
}

void Axis3Velocity( const std_msgs::Int32& msg){
  sendCANmsg_float(0x03, 0x00F, 0, 8, msg.data);
}

void Axis4Position( const std_msgs::Int32& msg){
  sendCANmsg_int(0x04, 0x00C, 0, 8, msg.data);
}

void Axis4Velocity( const std_msgs::Int32& msg){
  sendCANmsg_float(0x04, 0x00F, 0, 8, msg.data);
}

void Axis5Position( const std_msgs::Int32& msg){
  sendCANmsg_int(0x05, 0x00C, 0, 8, msg.data);
}

void Axis5Velocity( const std_msgs::Int32& msg){
  sendCANmsg_float(0x05, 0x00F, 0, 8, msg.data);
}

void Axis6Position( const std_msgs::Int32& msg){
  sendCANmsg_int(0x06, 0x00C, 0, 8, msg.data);
}

void Axis6Velocity( const std_msgs::Int32& msg){
  sendCANmsg_float(0x06, 0x00F, 0, 8, msg.data);
}

ros::Subscriber<std_msgs::Int32> A1P("RobotArm/Axis1/Position", &Axis1Position );
ros::Subscriber<std_msgs::Int32> A1V("RobotArm/Axis1/Velocity", &Axis1Velocity );
ros::Subscriber<std_msgs::Int32> A2P("RobotArm/Axis2/Position", &Axis2Position );
ros::Subscriber<std_msgs::Int32> A2V("RobotArm/Axis2/Velocity", &Axis2Velocity );
ros::Subscriber<std_msgs::Int32> A3P("RobotArm/Axis3/Position", &Axis3Position );
ros::Subscriber<std_msgs::Int32> A3V("RobotArm/Axis3/Velocity", &Axis3Velocity );
ros::Subscriber<std_msgs::Int32> A4P("RobotArm/Axis4/Position", &Axis4Position );
ros::Subscriber<std_msgs::Int32> A4V("RobotArm/Axis4/Velocity", &Axis4Velocity );
ros::Subscriber<std_msgs::Int32> A5P("RobotArm/Axis5/Position", &Axis5Position );
ros::Subscriber<std_msgs::Int32> A5V("RobotArm/Axis5/Velocity", &Axis5Velocity );
ros::Subscriber<std_msgs::Int32> A6P("RobotArm/Axis6/Position", &Axis6Position );
ros::Subscriber<std_msgs::Int32> A6V("RobotArm/Axis6/Velocity", &Axis6Velocity );

//-------------------------

void setup()
{
  Serial.begin(115200);
  //CAN -------------------

  CAN0.begin(CAN_500KBPS) == CAN_OK;

  //-----------------------

  //ROS -------------------

  nh.initNode();
  nh.subscribe(A1P);
  nh.subscribe(A1V);
  nh.subscribe(A2P);
  nh.subscribe(A2V);
  nh.subscribe(A3P);
  nh.subscribe(A3V);
  nh.subscribe(A4P);
  nh.subscribe(A4V);
  nh.subscribe(A5P);
  nh.subscribe(A5V);
  nh.subscribe(A6P);
  nh.subscribe(A6V);

  //-----------------------
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

//CAN ---------------------

void sendCANmsg_int(int ID, int CMD, int EXT, int DLC, uint32_t DATA)
{
  data[0] = (DATA & 0xFF);
  data[1] = ((DATA >> 8) & 0xFF);
  data[2] = ((DATA >> 16) & 0xFF);
  data[3] = ((DATA >> 24) & 0xFF);
  CAN0.sendMsgBuf( (ID << 5) + CMD, EXT, DLC, data);
}

void sendCANmsg_float(int ID, int CMD, int EXT, int DLC, float DATA)
{

  byte *b = (byte *)&DATA;

  data[0] = b[0];
  data[1] = b[1];
  data[2] = b[2];
  data[3] = b[3];
  /*
  byteArray[0] = (int)((longInt >> 24) & 0xFF) ;
  byteArray[1] = (int)((longInt >> 16) & 0xFF) ;
  byteArray[2] = (int)((longInt >> 8) & 0XFF);
  byteArray[3] = (int)((longInt & 0XFF));
  */
  CAN0.sendMsgBuf( (ID << 5) + CMD, EXT, DLC, data);
}

//-----------------------
