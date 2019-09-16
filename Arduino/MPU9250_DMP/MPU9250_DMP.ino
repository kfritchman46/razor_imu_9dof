/************************************************************
Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>
#include <Wire.h>

#define SerialPort SerialUSB

//#define STATUS_LED_PIN 13

MPU9250_DMP imu;

boolean output_stream_on;
boolean output_single_on;

void setup() 
{
  SerialPort.begin(115200);
  output_stream_on=true;
  output_single_on=false;

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL | // Use gyro calibration
               DMP_FEATURE_SEND_RAW_ACCEL, //Get Acceleration as well
              100); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

char readChar()
{
  while (SerialPort.available() < 1) { } // Block
  return SerialPort.read();
}

void turn_output_stream_on()
{
  output_stream_on = true;
  //digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  //digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() 
{

  int print = 0;

  // Accept commands
  if ( SerialPort.available() >= 2 )
  {
    SerialPort.println("got input");
    if (SerialPort.read() == '#')
    {
      SerialPort.println("got a command");
      int command = SerialPort.read(); // Commands
      if ( command == 'o' )
      {
        
        char output_param = readChar();
        if (output_param == '0') // Disable continuous streaming output
        {
          SerialPort.println("turing stream off");
          turn_output_stream_off();
        }
        else if (output_param == '1') // Enable continuous streaming output
        {
          turn_output_stream_on();
        }
      }
      else if ( command == 'f' )
      {
        output_single_on=true;
        SerialPort.println("single");
      }
    }
  }
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      if ( output_stream_on || output_single_on )
      {
        printIMUDataBytes();
        output_single_on=false;
      }
      else
      {
        //SerialPort.println("skipping");
      }
    }
  }
}


byte message[256];
byte msgStart[3] = {0xFF, 0xF0, 0x00};

byte calculate_checksum(byte *message, int _len)
{
  byte checksum = 0;
  for(int i=0; i< _len; i++)
  {
    checksum |= message[i];
  }
  return checksum;
}

//Sub messages  Label   Data Type   Length Values
//              1 byte  1 byte      1 byte Length*Type_Length
int add_sub_to_message(byte *message, int _mlen, byte label, byte * data, int _dlen)
{
  memcpy(message+_mlen, &label, 1);
  memcpy(message+_mlen+1,data,_dlen);
  return _mlen+_dlen+1;
}

//Total message contents Start    checksum  Length  Message
//                       3 bytes  2 bytes   1 byte  1-256 bytes
void send_message(byte *message, int _mlen)
{
  byte cs[2] = {0x0, 0x0};
  cs[1] = calculate_checksum(message, _mlen);
  SerialPort.write(msgStart, 3);
  SerialPort.write(cs, 2);
  SerialPort.write((byte) _mlen);
  SerialPort.write(message, _mlen);
}

void printIMUDataBytes(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float quat[4];
  quat[0] = imu.calcQuat(imu.qw);
  quat[1] = imu.calcQuat(imu.qx);
  quat[2] = imu.calcQuat(imu.qy);
  quat[3] = imu.calcQuat(imu.qz);

  float rpy[3];
  rpy[0] = imu.roll;
  rpy[1] = imu.pitch;
  rpy[2] = imu.yaw;

  float accel[3];
  accel[0] = imu.calcAccel(imu.ax);
  accel[1] = imu.calcAccel(imu.ay);
  accel[2] = imu.calcAccel(imu.az);

  
  memset(message, 0, sizeof(message));
  int msg_len = 0;

  // Sub Message Labels
  // 0x0 Message
  // 0x1 Quaternion
  // 0x2 Gyro
  // 0x4 Accel
  // 0x6 Magn
  // 0x8 Time
  // Sub Message Types
  // 0x0 Byte
  // 0x1 Int
  // 0x2 Float
  
  msg_len = add_sub_to_message(message, msg_len, 0x1, (byte *) quat, 4*sizeof(quat[0]));
  msg_len = add_sub_to_message(message, msg_len, 0x2, (byte *) rpy, 3*sizeof(rpy[0]));
  msg_len = add_sub_to_message(message, msg_len, 0x4, (byte *) accel, 3*sizeof(accel[0]));
  msg_len = add_sub_to_message(message, msg_len, 0x8, (byte *) &imu.time, 1*sizeof(imu.time));

  send_message(message, msg_len);
}
