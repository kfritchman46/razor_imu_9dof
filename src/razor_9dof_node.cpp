#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

#include <cstdlib>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <cstring>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


// Returns index of start of key sequence
int findInBuffer(char *key, char *buffer, int _buflen)
{
  int key_match_len = 0;
  int key_match_idx = 0;
  for(int i=0; i<_buflen; i++)
  {
    if(key[key_match_len] == buffer[i])
    {
      if(key_match_len == 0) key_match_idx=i;
      key_match_len++;
      if(key_match_len == strlen(key)) return key_match_idx;
    }
    else
    {
      key_match_len=0;
    }
  }
  return -1;
}

// endianness changes the endianness
float bytesToFloat(char *buffer, bool endianness) {
  float val;
  char buf3[4];
  if(endianness) {
    buf3[0]=buffer[3];
    buf3[1]=buffer[2];
    buf3[2]=buffer[1];
    buf3[3]=buffer[0];
    memcpy(&val,&buf3,sizeof(float));
  } else {
    memcpy(&val,&buffer,sizeof(float));
  }
  return val;
}


int main(int argc, char **argv)
{
  // ROS setup
  ros::init(argc, argv, "razor_9dof");
  ros::NodeHandle n;
  std::string node_name;
  node_name="default";
  ros::Publisher imu_topic = n.advertise<sensor_msgs::Imu>("imu/"+node_name, 500);

  // Serial Device setup
  std::string port = "/dev/ttyACM0";
  int serial_device = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  if (serial_device < 0)
  {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    exit(1);
  }

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_device, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 1;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_device, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  char read_buf[256];
  std::memset(read_buf, '\0', sizeof(read_buf));
  int track_ptr = 0;

  char start_key[] = "Q|float";
  char buf='\0';

  sensor_msgs::Imu msg;
  msg.header.frame_id = "im an imu";

  while(ros::ok())
  {
    //Search for the start byte
    bool found=true;
    for(int i=0; i< strlen(start_key);i++)
    {
      read(serial_device, &buf, 1);
      if(buf==start_key[i]){
        found=true;
        //ROS_INFO("FINDING %c",start_key[i]);
        continue;
      }
      if(buf==start_key[0]) {
        i=0;
      } else {
        i=-1;
        found=false;
      }
    }

    if(found){
      ROS_INFO("FOUND");
      char bytes_per_float;
      char floats;
      read(serial_device, &bytes_per_float, 1);
      read(serial_device, &floats, 1);
      ROS_INFO("%d",(int)floats);
      int toRead = sizeof(float)*floats;
      int haveRead = 0;
      char buf2[toRead];
      do {
        int bytesIn = read(serial_device, buf2+haveRead, toRead-haveRead);
        haveRead += bytesIn;
      } while(haveRead < toRead);
      float val = bytesToFloat(buf2+0*sizeof(float),false);
      ROS_INFO("%f",val);
      msg.orientation.w = (double) val;
      val = bytesToFloat(buf2+1*sizeof(float),true);
      msg.orientation.x = (double) val;
      val = bytesToFloat(buf2+2*sizeof(float),true);
      msg.orientation.y = (double) val;
      val = bytesToFloat(buf2+3*sizeof(float),true);
      msg.orientation.z = (double) val;
      imu_topic.publish(msg);
    }
  }

  close(serial_device);
  return 0;
}
