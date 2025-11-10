/********************************************************************************
 * 
 *          MECANUM ROBOT CONTROLLER - ESP32 (DEFINITIVE - IDF COMPATIBLE)
 * 
 * This version uses the low-level IDF functions for PWM to guarantee compilation
 * even if the Arduino-ESP32 environment has issues.
 * 
 * THIS IS THE FINAL VERSION FOR MOTOR CONTROL.
 * 
 ********************************************************************************/

// --- LIBRARIES ---
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "driver/ledc.h" // This header is ESSENTIAL for the functions below

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
const int DRIVE_SPEED = 230;

struct ControlPacket {
  int8_t btn_forward; int8_t btn_forward_right; int8_t btn_right;
  int8_t btn_backward_right; int8_t btn_backward; int8_t btn_backward_left;
  int8_t btn_left; int8_t btn_forward_left; int8_t btn_strfe_right;
  int8_t btn_stfre_left;
};

// =============================================================================
// --- MOTOR CONTROL FUNCTIONS (BARE-METAL / IDF STYLE) ---
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

// Helper function to make movement code cleaner
void set_motor_power(int ch, int speed) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

// --- MOVEMENT FUNCTIONS ---
void move_forward() {
    stop_all();
    digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(FR_IN2_PIN, HIGH);
    digitalWrite(RL_IN4_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_backward() {
    stop_all();
    digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(FR_IN1_PIN, HIGH);
    digitalWrite(RL_IN3_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_right() { // Rotate Right
    stop_all();
    digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(RL_IN4_PIN, HIGH);
    digitalWrite(FR_IN1_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_left() { // Rotate Left
    stop_all();
    digitalWrite(FR_IN2_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH);
    digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(RL_IN3_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_strfe_right() {
    stop_all();
    digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH);
    digitalWrite(FR_IN1_PIN, HIGH); digitalWrite(RL_IN3_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_strfe_left() {
    stop_all();
    digitalWrite(FR_IN2_PIN, HIGH); digitalWrite(RL_IN4_PIN, HIGH);
    digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_forward_right() {
    stop_all();
    digitalWrite(FL_IN2_PIN, HIGH); digitalWrite(RR_IN4_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, 0);
    set_motor_power(LEDC_CHANNEL_1, 0);           set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_forward_left() {
    stop_all();
    digitalWrite(FR_IN2_PIN, HIGH); digitalWrite(RL_IN4_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, 0);           set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, 0);
}
void move_backward_right() {
    stop_all();
    digitalWrite(FR_IN1_PIN, HIGH); digitalWrite(RL_IN3_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, 0);           set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
    set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_3, 0);
}
void move_backward_left() {
    stop_all();
    digitalWrite(FL_IN1_PIN, HIGH); digitalWrite(RR_IN3_PIN, HIGH);
    set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED); set_motor_power(LEDC_CHANNEL_2, 0);
    set_motor_power(LEDC_CHANNEL_1, 0);           set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}

// =============================================================================
// --- FREE RTOS TASK ---
// =============================================================================
void robot_control_task(void *pvParameters) {
  radio.startListening();
  while(1) {
    if (radio.available()) {
      ControlPacket controls;
      radio.read(&controls, sizeof(controls));
      
      if (controls.btn_forward) { Serial.println("Cmd: FWD"); move_forward(); }
      else if (controls.btn_forward_right) { Serial.println("Cmd: FWD-R"); move_forward_right(); }
      else if (controls.btn_right) { Serial.println("Cmd: ROT-R"); move_right(); }
      else if (controls.btn_backward_right) { Serial.println("Cmd: BCK-R"); move_backward_right(); }
      else if (controls.btn_backward) { Serial.println("Cmd: BCK"); move_backward(); }
      else if (controls.btn_backward_left) { Serial.println("Cmd: BCK-L"); move_backward_left(); }
      else if (controls.btn_left) { Serial.println("Cmd: ROT-L"); move_left(); }
      else if (controls.btn_forward_left) { Serial.println("Cmd: FWD-L"); move_forward_left(); }
      else if (controls.btn_strfe_right) { Serial.println("Cmd: STRF-R"); move_strfe_right(); }
      else if (controls.btn_stfre_left) { Serial.println("Cmd: STRF-L"); move_strfe_left(); }
      else { Serial.println("Cmd: STOP"); stop_all(); }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// =============================================================================
// --- MAIN SETUP AND LOOP ---
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 Definitive Motor Sketch (IDF-Safe) Booting...");

  init_motors();
  Serial.println("Motors Initialized (IDF Style).");
  
  if (!radio.begin()) {
    Serial.println("FATAL: Radio hardware not responding!");
    while(1);
  }
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, ROBOT_ADDRESS);
  Serial.println("nRF24L01 Radio Initialized.");
  
  xTaskCreatePinnedToCore(
      robot_control_task, "RobotControlTask", 4096, NULL, 1, NULL, 0);
      
  Serial.println("System Ready. Listening for commands.");
}

void loop() {
  delay(1000);
}