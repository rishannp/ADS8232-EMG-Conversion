// Red electrode on Bicep Belly
// Yellow Electrode on upper chest - Non active region
// Green electrode on elbow Bone

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  500 // Maximum pulse length count (out of 4096)

const int sensorPin = A0;        // Pin number for the ADC
int sensorValue = 0;            // Sensor value

// EMA coefficients for the band-pass filter
float EMA_a_low = 0.05;    // Alpha for low-frequency cutoff (approximates 8 Hz)
float EMA_a_high = 0.1;    // Alpha for high-frequency cutoff (approximates 100 Hz)
float EMA_S_low = 0;       // EMA S for low-frequency filter
float EMA_S_high = 0;      // EMA S for high-frequency filter

// Filtered values
float highpass = 0;
float bandpass = 0;

// Envelope detection variables
const int envelopeWindowSize = 20; // Increased size for smoother envelope
float envelopeBuffer[envelopeWindowSize] = {0};
int envelopeIndex = 0;

void setup() {
  Serial.begin(9600);            // Initialize Serial communication
  pwm.begin();
  pwm.setPWMFreq(60);            // Set PWM frequency for servos
  EMA_S_low = analogRead(sensorPin);  // Initialize EMA S for low-frequency filter
  EMA_S_high = analogRead(sensorPin); // Initialize EMA S for high-frequency filter
}

void loop() {
  // Read the raw sensor value
  sensorValue = analogRead(sensorPin);
  
  // Apply Exponential Moving Average for low-frequency cutoff
  EMA_S_low = (EMA_a_low * sensorValue) + ((1 - EMA_a_low) * EMA_S_low);
  
  // Apply Exponential Moving Average for high-frequency cutoff
  EMA_S_high = (EMA_a_high * sensorValue) + ((1 - EMA_a_high) * EMA_S_high);
  
  // Calculate high-pass and band-pass filter outputs
  highpass = sensorValue - EMA_S_low;
  bandpass = EMA_S_high - EMA_S_low;
  
  // Rectify the band-pass signal
  float rectifiedSignal = abs(bandpass);
  
  // Calculate envelope using a moving average
  float envelope = getEnvelope(rectifiedSignal);
  
  // Normalize envelope for PWM control
  int pwmValue = map(envelope, 5, 7, SERVOMIN, SERVOMAX); // Adjust ranges as needed

  Serial.println(envelope);

  //Set PWM for servos (uncomment to use)
  for (int i = 11; i <= 15; i++) {
    pwm.setPWM(i, 0, pwmValue);  // Update all servos with the same value
  }
  
  delay(20); // Delay for readability and to avoid overloading the servos
}

// Apply moving average for envelope detection
float getEnvelope(float newSample) {
    envelopeBuffer[envelopeIndex] = newSample;
    envelopeIndex = (envelopeIndex + 1) % envelopeWindowSize;
    float sum = 0;
    for (int i = 0; i < envelopeWindowSize; i++) {
        sum += envelopeBuffer[i];
    }
    return sum / envelopeWindowSize;
}
