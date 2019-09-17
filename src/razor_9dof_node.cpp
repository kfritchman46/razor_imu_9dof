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
    memcpy(&val,buf3,4);
  } else {
    memcpy(&val,buffer,4);
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

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 1;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_device, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  char read_buf[256];
  std::memset(read_buf, 0, 256);
  int track_ptr = 0;

  sensor_msgs::Imu msg;
  msg.header.frame_id = "im an imu";
  msg.orientation_covariance[0] = -1;
  msg.angular_velocity_covariance[0] = -1;
  msg.linear_acceleration_covariance[0] = -1;
  int seq = 0;

  // Message static data
  int toRead;
  int haveRead;
  char crc[2];
  char my_crc;
  char len;
  char message[256];

  char start_key[] = {0xFF, 0xF0, 0x00};
  char buf[] = {0,0,0};

  bool found;
  int tries;
  while(ros::ok())
  {
    //Search for the start byte
    std::memset(buf,0,3);
    found=false;
    while(!found && ros::ok())
    {
      if(read(serial_device, buf+2, 1) > 0)
      {
        //Compare to packetstart sequence
        found=true;
        for(int i=0;i<3;i++)
        {
          if(start_key[i]!=buf[i])
          {
            found=false;
          }
        }
        //Shift buffer for new value
        buf[0]=buf[1];
        buf[1]=buf[2];
      }
      else
      {
        ROS_INFO("failed to read trying again");
      }
    }

    if(found){
      // Found the start bytes
      //Read CRC
      toRead=2;
      do {
        toRead -= read(serial_device, &crc+(2-toRead), toRead);
      } while(toRead > 0 && ros::ok());
      // Read msg_len
      toRead=1;
      do {
        toRead -= read(serial_device, &len+(1-toRead), toRead);
      } while(toRead > 0 && ros::ok());
      // Read Full Message
      toRead=(int) len;
      do {
        toRead -= read(serial_device, &message+(len-toRead), toRead);
      } while(toRead > 0);
      // compare simple crc
      my_crc = 0;
      for(int i=0; i<len;i++)
      {
        my_crc |= message[i];
      }
      if(my_crc != crc[1])
      {
        ROS_INFO("message corrupt");
        continue;
      }

      for(int i=0;i<len;i++)
      {
        // Quaternion
        if(message[i]==0x1)
        {
          float val=bytesToFloat(message+i+1+0*sizeof(float), false);
          msg.orientation.w = (double) val;
          val=bytesToFloat(message+i+1+1*sizeof(float), false);
          msg.orientation.x = (double) val;
          val=bytesToFloat(message+i+1+2*sizeof(float), false);
          msg.orientation.y = (double) val;
          val=bytesToFloat(message+i+1+3*sizeof(float), false);
          msg.orientation.z = (double) val;
          i+=4*sizeof(float);
        }
        // Gyro
        else if(message[i]==0x2)
        {
          i+=3*sizeof(float);
        }
        // Accel
        else if(message[i]==0x4)
        {
          float val=bytesToFloat(message+i+1+0*sizeof(float), false);
          msg.linear_acceleration.x = (double) val;
          val=bytesToFloat(message+i+1+1*sizeof(float), false);
          msg.linear_acceleration.y = (double) val;
          val=bytesToFloat(message+i+1+2*sizeof(float), false);
          msg.linear_acceleration.z = (double) val;
          i+=3*sizeof(float);
        }
        // Magn
        else if(message[i]==0x6)
        {
          i+=3*sizeof(float);
        }
        // Time
        else if(message[i]==0x8)
        {
          i+=1*sizeof(long);
        }
      }
      msg.header.stamp = ros::Time::now();
      msg.header.seq = seq;
      imu_topic.publish(msg);
      seq++;
    }
    ros::spinOnce();
  }

  close(serial_device);
  return 0;
}
