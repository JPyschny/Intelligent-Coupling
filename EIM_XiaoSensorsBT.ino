#include <Wire.h>

// Define constants
constexpr int GAIN_128 = 25;                    // Gain setting for the HX711
constexpr int GAIN_64 = 27;                     // Gain setting for the HX711
constexpr int DOUT_PIN = 0;                     // digital output pin for the HX711 data
constexpr int SCK_PIN = 1;                      // Serial Clock Input pin for the HX711
constexpr float CALIBRATION_SLOPE = -1.6417018914310938e-07;
constexpr float CALIBRATION_INTERCEPT = 1.2531342914214711;
constexpr int GYRO_CALIBRATION_STEPS = 2000;    // number of steps for gyroscope calibration
constexpr int TORQUE_CALIBRATION_STEPS = 100;   // number of steps for torque calibration

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
int RateCalibrationNumber;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float RateCalibrationAccX, RateCalibrationAccY, RateCalibrationAccZ;
bool MPU6050_status;
float HX711_offset = 0;
unsigned long lastBlinkTime = 0; // Track the last time the LED was toggled
const unsigned long blinkInterval = 500; // Blink interval in milliseconds

// Low Pass Filter implementation
template <int ORDER> // ORDER is 1 or 2
class LowPass
{
  private:
    float a[ORDER];
    float b[ORDER + 1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[ORDER + 1]; // Raw values
    float y[ORDER + 1]; // Filtered values

  public:
    LowPass(float f0, float fs, bool adaptive) {
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.

      omega0 = 6.28318530718 * f0;
      dt = 1.0 / fs;
      adapt = adaptive;
      tn1 = -dt;
      for (int k = 0; k < ORDER + 1; k++) {
        x[k] = 0;
        y[k] = 0;
      }
      setCoef();
    }

    void setCoef() {
      if (adapt) {
        float t = micros() / 1.0e6;
        dt = t - tn1;
        tn1 = t;
      }

      float alpha = omega0 * dt;
      if (ORDER == 1) {
        a[0] = -(alpha - 2.0) / (alpha + 2.0);
        b[0] = alpha / (alpha + 2.0);
        b[1] = alpha / (alpha + 2.0);
      }
      if (ORDER == 2) {
        float alphaSq = alpha * alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
        b[0] = alphaSq / D;
        b[1] = 2 * b[0];
        b[2] = b[0];
        a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
        a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
      }
    }

    float filt(float xn) {
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if (adapt) {
        setCoef(); // Update coefficients if necessary
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for (int k = 0; k < ORDER; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
      }
      y[0] += b[ORDER] * x[ORDER];

      // Save the historical values
      for (int k = ORDER; k > 0; k--) {
        y[k] = y[k - 1];
        x[k] = x[k - 1];
      }

      // Return the filtered value
      return y[0];
    }
};

// Filter instance
LowPass<2> lpRateRoll(5, 1e2, true);
LowPass<2> lpRatePitch(5, 1e2, true);
LowPass<2> lpRateYaw(5, 1e2, true);
LowPass<2> lpAccX(5, 1e2, true);
LowPass<2> lpAccY(5, 1e2, true);
LowPass<2> lpAccZ(5, 1e2, true);

void setup() {
  // Set the LED pin mode
  pinMode(LED_BUILTIN, OUTPUT);

  // Set the HX711 pin modes
  pinMode(DOUT_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);

  //Initialize serial at 57600 baud and wait for port to open:
  Serial.begin(57600);
  delay(1500);

  // Initialize Serial1 for communication with HC-05
  Serial1.begin(9600);

  // Set the I2C clock speed and initialize the I2C interface
  Wire.setClock(400000);
  Wire.begin();

  // Initialize the MPU-6050
  Serial.println("Initialize the MPU-6050"); Serial1.println("Initialize the MPU-6050");
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibrate the Gyroscope
  Serial.println("Calibrate the Gyroscope"); Serial1.println("Calibrate the Gyroscope");
  for (RateCalibrationNumber = 0;
       RateCalibrationNumber < GYRO_CALIBRATION_STEPS;
       RateCalibrationNumber++) {
    MPU_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
  }
  RateCalibrationRoll /= GYRO_CALIBRATION_STEPS;
  RateCalibrationPitch /= GYRO_CALIBRATION_STEPS;
  RateCalibrationYaw /= GYRO_CALIBRATION_STEPS;

  // Calibrate HX711
  Serial.println("Calibrate the HX711"); Serial1.println("Calibrate the HX711");
  float sum = 0;
  for (int i = 0; i < TORQUE_CALIBRATION_STEPS; i++) {
    sum += readHX711();
  }
  HX711_offset = sum / TORQUE_CALIBRATION_STEPS;
  Serial.println("HX711 offset value: " + String(HX711_offset));
  Serial1.println("HX711 offset value: " + String(HX711_offset));
}

void loop() {
  //  unsigned long startMillis = millis(); // Start time

  // Check the connection status of the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  uint8_t mpu6050_id = Wire.read();

  // Only attempt to read from the MPU-6050 if it is connected
  if (mpu6050_id == 0x68) {
    // Call the function to read data from the MPU-6050
    MPU_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    MPU6050_status = true;
  } else {
    Serial.println("MPU6050 not connected");
    Serial1.println("MPU6050 not connected");
    // Set default sensor data
    RateRoll = 0;
    RatePitch = 0;
    RateYaw = 0;
    AccX = 0;
    AccY = 0;
    AccZ = 0;
    MPU6050_status = false;
  }

  // Get torque in Ncm
  float Torque = readHX711() * 1E2;

  // Print the data to the Serial port
  Serial.println(String(RateRoll)  + "," +
                 String(RatePitch) + "," +
                 String(RateYaw)   + "," +
                 String(AccX)      + "," +
                 String(AccY)      + "," +
                 String(AccZ)      + "," +
                 String(Torque));

  // Print the data to the Serial1 port for Bluetooth
  Serial1.println(String(RateRoll)  + "," +
                  String(RatePitch) + "," +
                  String(RateYaw)   + "," +
                  String(AccX)      + "," +
                  String(AccY)      + "," +
                  String(AccZ)      + "," +
                  String(Torque));

  // Check if it's time to blink the LED
  if (millis() - lastBlinkTime >= blinkInterval) {
    // Save the last time the LED was toggled
    lastBlinkTime = millis();

    // Read the current state of the LED
    int ledState = digitalRead(LED_BUILTIN);

    // If the LED is off, turn it on. If it's on, turn it off.
    digitalWrite(LED_BUILTIN, !ledState);
  }

  //  unsigned long endMillis = millis(); // End time
  //  unsigned long elapsedTime = endMillis - startMillis; // Calculate elapsed time
  //  Serial.println("Time taken to read from MPU-6050: " + String(elapsedTime) + " ms"); // Print the time taken
}

float readHX711() {
  unsigned long data = 0;
  uint8_t dout;

  while (digitalRead(DOUT_PIN)) {} // wait until value is available
  for (uint8_t i = 0; i < GAIN_128; i++) { //highest Gain
    //delayMicroseconds(1); // uncomment for fast MCUs
    digitalWrite(SCK_PIN, 1);
    //delayMicroseconds(1); // uncomment for fast MCUs
    digitalWrite(SCK_PIN, 0);
    if (i < (24)) {
      dout = digitalRead(DOUT_PIN);
      data = (data << 1) | dout;
    }
  }
  data = data ^ 0x800000; // flip bit 23
  float HX711_readings = (float)data;
  float Torque = CALIBRATION_SLOPE * HX711_readings + CALIBRATION_INTERCEPT - HX711_offset;
  return Torque;
}

// Function to power down the HX711 (sleep mode)
void powerDown() {
  digitalWrite(SCK_PIN, LOW);
  digitalWrite(SCK_PIN, HIGH);
}

// Function to power up the HX711
void powerUp() {
  digitalWrite(SCK_PIN, LOW);
}

// Function to read gyro and accelerometer signals from the MPU-6050
void MPU_signals(void) {
  // Configure the accelerometer full-scale range
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  uint8_t error = Wire.endTransmission();

  // Check connection to MPU-6050
  if (error != 0) {
    Serial.println("Error ocurred reading MPU6050");
    // Set default sensor data
    RateRoll = 0;
    RatePitch = 0;
    RateYaw = 0;
    AccX = 0;
    AccY = 0;
    AccZ = 0;
    MPU6050_status = false;
  }
  else {
    MPU6050_status = true;

    // Request accelerometer data from the MPU-6050
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    // Read accelerometer data and store it in variables
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    // Configure the gyroscope full-scale range
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    // Request gyroscope data from the MPU-6050
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    // Read gyroscope data and store it in variables
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    // Convert raw gyroscope data to degrees per second
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    // Compute the filtered signals
    RateRoll = lpRateRoll.filt(RateRoll);
    RatePitch = lpRatePitch.filt(RatePitch);
    RateYaw = lpRateYaw.filt(RateYaw);

    // Convert raw accelerometer data to g
    AccX = (float)AccXLSB / 4096 * 9.81;
    AccY = (float)AccYLSB / 4096 * 9.81;
    AccZ = (float)AccZLSB / 4096 * 9.81;

    // Compute the filtered signals
    AccX = lpAccX.filt(AccX);
    AccY = lpAccY.filt(AccY);
    AccZ = lpAccZ.filt(AccZ);
  }
}
