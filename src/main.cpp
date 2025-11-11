
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "MyLD2410.h"
#include "driver/ledc.h"
#include <ESP32Servo.h>

// =============================================================================
// --- CONFIGURATION & PIN DEFINITIONS ---
// =============================================================================
#define NRF_CE_PIN  4
#define NRF_CSN_PIN 5
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte ROBOT_ADDRESS[6] = "robot";

#define FL_IN1_PIN 25
#define FL_IN2_PIN 26
#define RL_IN3_PIN 27
#define RL_IN4_PIN 14
#define FR_IN1_PIN 12
#define FR_IN2_PIN 13
#define RR_IN3_PIN 21
#define RR_IN4_PIN 22

#define FL_PWM_PIN 32
#define RL_PWM_PIN 33
#define FR_PWM_PIN 15
#define RR_PWM_PIN 2

#define PWM_FREQUENCY 5000 
#define PWM_RESOLUTION 8   
const int DRIVE_SPEED = 200;

#define PAN_SERVO_PIN 9
Servo panServo;

// 4. Create a variable to hold the servo's position.
// We make it static so it remembers its value between loops.
static int panAngle = 90; // Start looking straight ahead

#define RADAR_SERIAL Serial2
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17
MyLD2410 sensor(RADAR_SERIAL);

// =============================================================================
// --- DATA STRUCTURES & SHARED VARIABLES ---
// =============================================================================
struct ControlPacket {
  int8_t btn_forward; int8_t btn_forward_right; int8_t btn_right;
  int8_t btn_backward_right; int8_t btn_backward; int8_t btn_backward_left;
  int8_t btn_left; int8_t btn_forward_left; int8_t btn_strfe_right;
  int8_t btn_stfre_left;
  int8_t btn_pan_left;
  int8_t btn_pan_right;
  
};
struct RadarPacket {
  uint8_t gate_signals[9];
};

volatile RadarPacket latestRadarData = {0};
SemaphoreHandle_t radarDataMutex;

// =============================================================================
// --- INITIALIZATION & MOTOR CONTROL ---
// =============================================================================
void init_motors() {
  pinMode(FL_IN1_PIN, OUTPUT); pinMode(FL_IN2_PIN, OUTPUT);
  pinMode(RL_IN3_PIN, OUTPUT); pinMode(RL_IN4_PIN, OUTPUT);
  pinMode(FR_IN1_PIN, OUTPUT); pinMode(FR_IN2_PIN, OUTPUT);
  pinMode(RR_IN3_PIN, OUTPUT); pinMode(RR_IN4_PIN, OUTPUT);
  
  ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION, .timer_num = LEDC_TIMER_0, .freq_hz = PWM_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK };
  ledc_timer_config(&ledc_timer);
  
  ledc_channel_config_t ledc_channels[4] = {
    { .gpio_num = FL_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0 },
    { .gpio_num = RL_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0 },
    { .gpio_num = FR_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_0, .duty = 0 },
    { .gpio_num = RR_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_3, .timer_sel = LEDC_TIMER_0, .duty = 0 }
  };
  for (int i = 0; i < 4; i++) { ledc_channel_config(&ledc_channels[i]); }
}

void stop_all() {
  digitalWrite(FL_IN1_PIN, LOW); digitalWrite(FL_IN2_PIN, LOW);
  digitalWrite(RL_IN3_PIN, LOW); digitalWrite(RL_IN4_PIN, LOW);
  digitalWrite(FR_IN1_PIN, LOW); digitalWrite(FR_IN2_PIN, LOW);
  digitalWrite(RR_IN3_PIN, LOW); digitalWrite(RR_IN4_PIN, LOW);
  
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void set_motor_power(int ch, int speed) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

void move_forward() { stop_all(); digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(FR_IN2_PIN, HIGH); digitalWrite(RL_IN4_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }
void move_backward() { stop_all(); digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(FR_IN1_PIN, HIGH); digitalWrite(RL_IN3_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }
void move_right() { stop_all(); digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(RL_IN4_PIN, HIGH); digitalWrite(FR_IN1_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }
void move_left() { stop_all(); digitalWrite(FR_IN2_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH); digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(RL_IN3_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }
void move_strfe_right() { stop_all(); digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH); digitalWrite(FR_IN1_PIN, HIGH); digitalWrite(RL_IN3_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }
void move_strfe_left() { stop_all(); digitalWrite(FR_IN2_PIN, HIGH); digitalWrite(RL_IN4_PIN, HIGH); digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }
void move_forward_right() { stop_all(); digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, 0); set_motor_power(LEDC_CHANNEL_1, 0); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }
void move_forward_left() { stop_all(); digitalWrite(FR_IN2_PIN, HIGH); digitalWrite(RL_IN4_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, 0); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, 0); }
void move_backward_right() { stop_all(); digitalWrite(FR_IN1_PIN, HIGH); digitalWrite(RL_IN3_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, 0); set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, 0); }
void move_backward_left() { stop_all(); digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH); set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, 0); set_motor_power(LEDC_CHANNEL_1, 0); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED); }

// =============================================================================
// --- FREE RTOS TASKS (CORRECTED) ---
// =============================================================================
int gate_index = 0;
void copy_gate_value(const byte &val, uint8_t* arr) {
  if (gate_index < 9) {
    arr[gate_index] = val;
    gate_index++;
  }
}

void radar_task(void *pvParameters) {
  if (!sensor.begin()) { vTaskDelete(NULL); }

  sensor.enhancedMode();
  delay(500);

  while (1) {
    sensor.check();
    uint8_t current_signals[9] = {0};
    if (sensor.presenceDetected()) {
      auto signals = sensor.getMovingSignals();
      gate_index = 0;
      signals.forEach([&](const byte &val){ copy_gate_value(val, current_signals); });
    }
    
    if (xSemaphoreTake(radarDataMutex, (TickType_t)10) == pdTRUE) {
      for(int i = 0; i < 9; i++) {
        latestRadarData.gate_signals[i] = current_signals[i];
      }
      xSemaphoreGive(radarDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void radio_and_motor_task(void *pvParameters) {
  radio.startListening();
  while(1) {
    if (radio.available()) {
      ControlPacket controls;
      radio.read(&controls, sizeof(controls));
      
      if (controls.btn_forward) { move_forward(); }
      else if (controls.btn_forward_right) { move_forward_right(); }
      else if (controls.btn_right) { move_right(); }
      else if (controls.btn_backward_right) { move_backward_right(); }
      else if (controls.btn_backward) { move_backward(); }
      else if (controls.btn_backward_left) { move_backward_left(); }
      else if (controls.btn_left) { move_left(); }
      else if (controls.btn_forward_left) { move_forward_left(); }
      else if (controls.btn_strfe_right) { move_strfe_right(); }
      else if (controls.btn_stfre_left) { move_strfe_left(); }
      else { stop_all(); }


      // --- NEW: Pan Servo Logic (separate block) ---
      if (controls.btn_pan_left) {
        panAngle += 2; // Increment angle
      }
      if (controls.btn_pan_right) {
        panAngle -= 2; // Decrement angle
      }

       // Constrain the angle to prevent the servo from breaking itself
      panAngle = constrain(panAngle, 0, 180);

      // Write the final position to the servo
      panServo.write(panAngle);

      RadarPacket response_packet;
      if (xSemaphoreTake(radarDataMutex, (TickType_t)10) == pdTRUE) {
        for(int i = 0; i < 9; i++) {
          response_packet.gate_signals[i] = latestRadarData.gate_signals[i];
        }
        xSemaphoreGive(radarDataMutex);
      }
      
      radio.stopListening();
      radio.write(&response_packet, sizeof(response_packet)); 
      radio.startListening();
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
// --- MAIN SETUP AND LOOP ---
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 Final Enhanced Code Booting...");
  
  init_motors();
  Serial.println("Motors Initialized.");
  
  RADAR_SERIAL.begin(LD2410_BAUD_RATE, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  
  if (!radio.begin()) { Serial.println("FATAL: Radio failed!"); while(1); }
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(ROBOT_ADDRESS);
  radio.openReadingPipe(1, ROBOT_ADDRESS);
  Serial.println("Radio Initialized.");
  
  radarDataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(radar_task, "RadarTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(radio_and_motor_task, "RadioMotorTask", 4096, NULL, 2, NULL, 0);

  ESP32PWM::allocateTimer(0); // This is needed for the servo library
  panServo.setPeriodHertz(50);    // Standard servo frequency
  panServo.attach(PAN_SERVO_PIN, 500, 2500); // Attach to pin with standard pulse widths
  panServo.write(panAngle); // Move to initial center position
  Serial.println("Pan Servo Initialized.");
      
  Serial.println("All systems go. Ready for commands.");
}

void loop() {
  delay(1000);
}