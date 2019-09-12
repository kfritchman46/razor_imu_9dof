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

void write_float_bytes(String label, float* data, int _len)
{
  SerialPort.print(label+"|float"+(char) sizeof(float)+(char) _len);
  SerialPort.write((byte *) data, sizeof(float)*_len);
}

void write_ulong_bytes(String label, long unsigned int* data, int _len)
{
  SerialPort.print(label+"|uint"+(char) sizeof(data[0])+(char) _len);
  SerialPort.write((byte *) data, sizeof(data[0])*_len);
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

//  SerialPort.println("Q: " + String(quat[0], 4) + ", " +
//                    String(quat[1], 4) + ", " + String(quat[2], 4) + 
//                    ", " + String(quat[3], 4));
  write_float_bytes("Q",quat,4);
//  SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
//            + String(imu.pitch) + ", " + String(imu.yaw));
  write_float_bytes("RPY",rpy,3);
//  SerialPort.println("X/Y/Z Accel: " + String(accelX, 4) + ", "
//            + String(accelY, 4) + ", " + String(accelZ, 4));
  write_float_bytes("A",accel,3);
  write_ulong_bytes("T",&imu.time, 1);
}
