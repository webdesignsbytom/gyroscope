#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <Math.h> // Include the Math library for trigonometric functions

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050.

int16_t acc_x, acc_y, acc_z; // Variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // Variables for gyro raw data
int16_t temp; // Variable for temperature data
int16_t last_acc_x, last_acc_y, last_acc_z; // Variables to store the last accelerometer values

char tmp_str[7]; // Temporary variable used in convert function

char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

// Check if the device is approximately vertical
bool isVertical(int16_t x, int16_t y, int16_t z) {
  const int16_t threshold = 2000; // Adjust this threshold as needed
  const int16_t gravity = 16384; // Raw value for 1g under ±2g sensitivity
  return abs(x) < threshold && abs(y) < threshold && abs(z - gravity) < threshold;
}

// Main setup
void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0); // Wake up MPU-6050
  Wire.endTransmission(true);

  // Initialize last accelerometer values
  last_acc_x = last_acc_y = last_acc_z = 0;
}

// Main code loop
void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7*2, true); // Request a total of 14 registers
  
  // Data inputs
  // Reading accelerometer data
  acc_x = Wire.read()<<8 | Wire.read();
  acc_y = Wire.read()<<8 | Wire.read();
  acc_z = Wire.read()<<8 | Wire.read();
  // Reading temperature data
  temp = Wire.read()<<8 | Wire.read();
  // Reading gyroscope data
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
  

  // Determine the change in acceleration
  int change_x = abs(acc_x - last_acc_x);
  int change_y = abs(acc_y - last_acc_y);
  int change_z = abs(acc_z - last_acc_z);


  int threshold = 2000; // Set your threshold here

  if (isVertical(acc_x, acc_y, acc_z)) {
    Serial.println("Device is approximately vertical");
  } else {
    // Calculate tilt angles
    float angleX = atan2(acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / PI;
    float angleY = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 180 / PI;
    float angleZ = atan2(sqrt(acc_x * acc_x + acc_y * acc_y), acc_z) * 180 / PI;

    Serial.print("Tilt Angle X: ");
    Serial.print(angleX);
    Serial.print(" degrees, Y: ");
    Serial.print(angleY);
    Serial.print(" degrees, Z: ");
    Serial.print(angleZ);
    Serial.println(" degrees");
  }
  
    float norm = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
  if (norm == 0) norm = 1; // prevent division by zero
  float ax = acc_x / norm;
  float ay = acc_y / norm;
  float az = acc_z / norm;

  // Calculate tilt angles
  float angleX = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;
  float angleY = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  float angleZ = atan2(sqrt(ax * ax + ay * ay), az) * 180 / PI;

  // Print tilt angles
  Serial.print("Tilt Angle X: ");
  Serial.print(angleX);
  Serial.print(" degrees, Y: ");
  Serial.print(angleY);
  Serial.print(" degrees, Z: ");
  Serial.print(angleZ);
  Serial.println(" degrees");
  
  // Update last accelerometer values
  last_acc_x = acc_x;
  last_acc_y = acc_y;
  last_acc_z = acc_z;

  // Print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(acc_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(acc_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(acc_z));
  Serial.print(" | tmp = "); Serial.print(temp/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();

  delay(1000); // Delay for a second before the next reading
}
