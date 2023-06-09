#include <Servo.h>

//
// PORT NUMBERS
//
#define SRV_PORT_1 4 // left
#define SRV_PORT_2 5 // right

// 
// SERVO PARAMETERS
//
#define SRV_1_OPEN 34
#define SRV_1_CLOSE 111
#define SRV_1_DIRECTION -1 // from close to open
#define SRV_1_RANGE 77

#define SRV_2_OPEN 137
#define SRV_2_CLOSE 60
#define SRV_2_DIRECTION 1 // from close to open
#define SRV_2_RANGE 77

// Declare Servo Objects
Servo Servo_1; // left
Servo Servo_2; // right

// buffer for Serial receiving (RX)
const int RX_BUFFER_SIZE = 2;
uint8_t rx_buffer[RX_BUFFER_SIZE];

// compute factor for servo angles
float factor_1 = float(SRV_1_RANGE) / 90.0;
float factor_2 = float(SRV_2_RANGE) / 90.0;


void setup() {
  // initialize Servos
  Servo_1.attach(SRV_PORT_1);
  Servo_2.attach(SRV_PORT_2);
  // initial values
  Servo_1.write(SRV_1_OPEN);
  Servo_2.write(SRV_2_OPEN);
  // initialize Serial
  Serial.begin(115200);
}


void loop() {
  /* SERIAL CONNECTION */
  if (Serial.available()) {
    // read in command
    Serial.readBytes(rx_buffer, RX_BUFFER_SIZE);

    // check command
    if (rx_buffer[0] < 0 || rx_buffer[0] > 90 ||
        rx_buffer[1] < 0 || rx_buffer[1] > 90) {
      // Serial.println("Gripper angles must be between 0 and 90 degrees.");
      return;
    }

    // write command to servo
    Servo_1.write(SRV_1_CLOSE + SRV_1_DIRECTION * factor_1 * float(rx_buffer[0])); // left
    Servo_2.write(SRV_2_CLOSE + SRV_2_DIRECTION * factor_2 * float(rx_buffer[1])); // right 
  }

  
  /* SERVO TEST */
  // Servo_1.write(SRV_1_OPEN);
  // Servo_2.write(SRV_2_OPEN);
  // delay(1000);
  // Servo_1.write(SRV_1_CLOSE);
  // Servo_2.write(SRV_2_CLOSE);
  // delay(1000);
}
