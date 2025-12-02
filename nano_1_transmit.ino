// =============================================================================
//       ARDUINO NANO REMOTE - FINAL (10-BUTTON, NO TILT)
// =============================================================================
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

struct ControlPacket {
  int8_t btn_forward, btn_forward_right, btn_right, btn_backward_right, btn_backward, 
         btn_backward_left, btn_left, btn_forward_left, btn_strfe_right, 
         btn_stfre_left, btn_pan_left, btn_pan_right;
};

// UPDATED: mmwave_distance_cm removed
struct __attribute__((packed)) RadarPacket {
  uint8_t gate_signals[9];
  int ultrasonic_distance_cm;
};

RF24 radio(9, 10);
const byte ROBOT_ADDRESS[6] = "robot";

enum btn_map{
  btn_forward         = PD4,
  btn_forward_right   = PD5, 
  btn_right           = PD3,
  btn_backward_right  = PC3,
  btn_backward        = PC2, 
  btn_backward_left   = PC1, 
  btn_left            = PC0, 
  btn_forward_left    = PD7, 
  btn_strfe_right     = PC5,
  btn_stfre_left      = PC4,
  btn_pan_left        = PD2,
  btn_pan_right       = PB0
};

void setup() {
  Serial.begin(115200);
  //pinMode(17, high);
  DDRD &= ~((1<<btn_forward) | (1<<btn_forward_right) | (1<<btn_right) | (1<<btn_forward_left) | (1<<btn_pan_left));
  DDRC &= ~((1<<btn_backward_right) | (1<<btn_backward) | (1<<btn_backward_left) | (1<<btn_left) | (1<<btn_strfe_right) | (1<<btn_stfre_left));
  DDRB &= ~((1<<btn_pan_right));
  PORTD |= (1<<btn_forward) | (1<<btn_forward_right) | (1<<btn_right) | (1<<btn_forward_left) | (1<<btn_pan_left);
  PORTC |= (1<<btn_backward_right) | (1<<btn_backward) | (1<<btn_backward_left) | (1<<btn_left) | (1<<btn_strfe_right) | (1<<btn_stfre_left);
  PORTB |= (1<<btn_pan_right);
  if (!radio.begin()) { Serial.println("Radio failed!"); while (1); }
  radio.setChannel(83); radio.setDataRate(RF24_250KBPS); radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(ROBOT_ADDRESS); radio.openReadingPipe(1, ROBOT_ADDRESS);
}

void loop() {
  ControlPacket controls;
  memset(&controls, 0, sizeof(controls));
  if      (!(PIND & (1<<btn_forward)))        { controls.btn_forward = 1; }
  else if (!(PIND & (1<<btn_forward_right)))  { controls.btn_forward_right = 1; }
  else if (!(PIND & (1<<btn_right)))          { controls.btn_right = 1; }
  else if (!(PINC & (1<<btn_backward_right))) { controls.btn_backward_right = 1; }
  else if (!(PINC & (1<<btn_backward)))       { controls.btn_backward = 1; }
  else if (!(PINC & (1<<btn_backward_left)))  { controls.btn_backward_left = 1; }
  else if (!(PINC & (1<<btn_left)))           { controls.btn_left = 1; }
  else if (!(PIND & (1<<btn_forward_left)))   { controls.btn_forward_left = 1; }
  else if (!(PINC & (1<<btn_stfre_left)))     { controls.btn_stfre_left = 1; }
  else if (!(PINC & (1<<btn_strfe_right)))    { controls.btn_strfe_right = 1; }
  if (!(PIND & (1<<btn_pan_left)))  { controls.btn_pan_left = 1; }
  if (!(PINB & (1<<btn_pan_right))) { controls.btn_pan_right = 1; }

  radio.stopListening();
  radio.write(&controls, sizeof(controls));
  radio.startListening();
  
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while (!radio.available() && !timeout) { if (millis() - started_waiting_at > 50) { timeout = true; } }

  if (!timeout) {
    RadarPacket telemetry;
    radio.read(&telemetry, sizeof(telemetry));
    String output = "";
    for (int i = 0; i < 9; i++) { output += telemetry.gate_signals[i]; output += ","; }
    output += telemetry.ultrasonic_distance_cm;
    Serial.println(output);
  }
  delay(100);
}