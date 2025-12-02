/********************************************************************************
 *
 *          MECANUM ROBOT CONTROLLER - FINAL POLISHED EDITION
 *
 * This is the definitive, feature-complete, stable version.
 * - Architecture: Robust multi-tasking to prevent sensor starvation.
 * - Fail-Safe: Implements the autonomous "Dynamic Patrol" logic.
 * - mmWave Distance: REMOVED as requested for simplification.
 * - LED Logic: RESTORED full, detailed RGB status indicators for all movements.
 *
 ********************************************************************************/

// --- LIBRARIES ---
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "MyLD2410.h"
#include "driver/ledc.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include "PCF8574.h"

// =============================================================================
// --- CONFIGURATION & PIN DEFINITIONS ---
// =============================================================================
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte ROBOT_ADDRESS[6] = "robot";

#define FL_PWM_PIN 32
#define RL_PWM_PIN 33
#define FR_PWM_PIN 15
#define RR_PWM_PIN 2
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
const int DRIVE_SPEED = 180;
const int TURN_SPEED = 180;

PCF8574 pcf8574(0x20);
#define EXP_FL_IN1 0
#define EXP_FL_IN2 1
#define EXP_RL_IN3 2
#define EXP_RL_IN4 3
#define EXP_FR_IN1 4
#define EXP_FR_IN2 5
#define EXP_RR_IN3 6
#define EXP_RR_IN4 7

#define PAN_SERVO_PIN 25
Servo panServo;

#define ULTRASONIC_TRIG_PIN 27
#define ULTRASONIC_ECHO_PIN 34
#define ULTRASONIC_SAMPLES 5
#define ULTRASONIC_TIMEOUT_US 30000
#define ULTRASONIC_MIN_DISTANCE 2
#define ULTRASONIC_MAX_DISTANCE 400

#define RADAR_SERIAL Serial2
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17
MyLD2410 radarSensor(RADAR_SERIAL);

#define RED_LED_PIN 26
#define GREEN_LED_PIN 14
#define BLUE_LED_PIN 12
#define BUZZER_PIN 13
#define FAILSAFE_TIMEOUT_MS 1000

#define OBSTACLE_DISTANCE_CM 30
#define MANEUVER_TURN_DURATION_MS 500
#define MANEUVER_BACKUP_DURATION_MS 400
#define MANEUVER_PAUSE_MS 300

// =============================================================================
// --- DATA STRUCTURES & SHARED VARIABLES ---
// =============================================================================
struct ControlPacket
{
  int8_t btn_forward, btn_forward_right, btn_right, btn_backward_right, btn_backward,
      btn_backward_left, btn_left, btn_forward_left, btn_strfe_right,
      btn_stfre_left, btn_pan_left, btn_pan_right;
};
// UPDATED: mmwave_distance_cm removed
struct __attribute__((packed)) RadarPacket
{
  uint8_t gate_signals[9];
  int ultrasonic_distance_cm;
};

volatile ControlPacket last_controls = {0};
volatile unsigned long lastPacketTime;
volatile int ultrasonic_dist_cm = ULTRASONIC_MAX_DISTANCE;
volatile uint8_t shared_gate_signals[9] = {0};
SemaphoreHandle_t radarDataMutex;

volatile long echo_start_time = 0;
volatile long echo_end_time = 0;
volatile bool new_distance_available = false;
volatile int ultrasonic_samples[ULTRASONIC_SAMPLES] = {0};
volatile int sample_index = 0;

// --- ISR, Prototypes, and Sensor Filtering Functions ---
void ICACHE_RAM_ATTR echo_isr();
int apply_moving_average(int new_distance);
void set_motor_power(int ch, int speed);
void init_motors();
void stop_all();
void turn_right();
void turn_left();
void move_forward();
void move_backward();
void move_strfe_right();
void move_strfe_left();
void move_forward_right();
void move_forward_left();
void move_backward_right();
void move_backward_left();

// =============================================================================
// --- ENHANCED ULTRASONIC ISR ---
// =============================================================================
void ICACHE_RAM_ATTR echo_isr()
{
  if (digitalRead(ULTRASONIC_ECHO_PIN) == HIGH)
  {
    echo_start_time = micros();
  }
  else
  {
    echo_end_time = micros();
    long duration = echo_end_time - echo_start_time;
    if (duration > 0 && duration < ULTRASONIC_TIMEOUT_US)
    {
      new_distance_available = true;
    }
  }
}

// =============================================================================
// --- MOVING AVERAGE FILTER ---
// =============================================================================
int apply_moving_average(int new_distance)
{
  if (new_distance >= ULTRASONIC_MIN_DISTANCE && new_distance <= ULTRASONIC_MAX_DISTANCE)
  {
    ultrasonic_samples[sample_index] = new_distance;
    sample_index = (sample_index + 1) % ULTRASONIC_SAMPLES;
    int sum = 0, valid_count = 0;
    for (int i = 0; i < ULTRASONIC_SAMPLES; i++)
    {
      if (ultrasonic_samples[i] > 0)
      {
        sum += ultrasonic_samples[i];
        valid_count++;
      }
    }
    if (valid_count > 0)
    {
      return sum / valid_count;
    }
  }
  return ULTRASONIC_MAX_DISTANCE;
}

// =============================================================================
// --- FREE RTOS TASKS ---
// =============================================================================

void radar_task(void *pvParameters)
{
  if (!radarSensor.begin())
  {
    vTaskDelete(NULL);
  }
  radarSensor.enhancedMode();
  delay(500);
  while (1)
  {
    radarSensor.check();
    uint8_t temp_signals[9] = {0};
    if (radarSensor.presenceDetected())
    {
      auto signals = radarSensor.getMovingSignals();
      int gate_index = 0;
      signals.forEach([&](const byte &val)
                      {
        if (gate_index < 9) {
          temp_signals[gate_index++] = val;
        } });
    }
    if (xSemaphoreTake(radarDataMutex, (TickType_t)10) == pdTRUE)
    {
      for (int i = 0; i < 9; i++)
      {
        shared_gate_signals[i] = temp_signals[i];
      }
      xSemaphoreGive(radarDataMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void radio_telemetry_task(void *pvParameters)
{
  radio.startListening();
  while (1)
  {
    if (radio.available())
    {
      radio.read((void *)&last_controls, sizeof(last_controls));
      lastPacketTime = millis();
    }
    static unsigned long lastTelemetrySend = 0;
    if (millis() - lastTelemetrySend > 50)
    {
      lastTelemetrySend = millis();
      RadarPacket response_packet = {0};
      if (xSemaphoreTake(radarDataMutex, (TickType_t)10) == pdTRUE)
      {
        for (int i = 0; i < 9; i++)
        {
          response_packet.gate_signals[i] = shared_gate_signals[i];
        }
        xSemaphoreGive(radarDataMutex);
      }
      response_packet.ultrasonic_distance_cm = (ultrasonic_dist_cm == ULTRASONIC_MAX_DISTANCE) ? 0 : ultrasonic_dist_cm;
      radio.stopListening();
      radio.write(&response_packet, sizeof(response_packet));
      radio.startListening();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void ultrasonic_task(void *pvParameters)
{
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN), echo_isr, CHANGE);
  while (1)
  {
    if (new_distance_available)
    {
      new_distance_available = false;
      if (echo_end_time > echo_start_time)
      {
        long duration = echo_end_time - echo_start_time;
        int raw_distance = duration * 0.0343 / 2;
        ultrasonic_dist_cm = apply_moving_average(raw_distance);
      }
    }
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}

void robot_action_task(void *pvParameters)
{
  enum RobotState
  {
    NORMAL_MODE,
    FAILSAFE_MODE
  } currentState = NORMAL_MODE;
  enum PatrolState
  {
    PATROL_FORWARD,
    MANEUVER_START,
    MANEUVER_BACKING_UP,
    MANEUVER_PAUSE_1,
    MANEUVER_LOOKING_RIGHT,
    MANEUVER_SCANNING_RIGHT,
    MANEUVER_LOOKING_LEFT,
    MANEUVER_SCANNING_LEFT,
    MANEUVER_DECIDING,
    MANEUVER_COMMITTING_TURN
  } patrolState = PATROL_FORWARD;

  int panAngle = 90;
  unsigned long lastIdleLedTime = 0, state_entry_time = 0;
  int idleLedState = 0, left_dist = 0, right_dist = 0, pan_sweep_direction = 1;
  unsigned long lastPanSweepTime = 0;
  bool wasFailsafeActive = false;

  while (1)
  {
    if (millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS)
    {
      currentState = FAILSAFE_MODE;
    }
    else
    {
      if (currentState == FAILSAFE_MODE)
      {
        currentState = NORMAL_MODE;
        stop_all();
        panServo.write(90);
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("[✓] Signal Restored. Manual control resumed.");
      }
    }

    if (currentState == NORMAL_MODE)
    {
      wasFailsafeActive = false;

      if (last_controls.btn_forward)
      {
        move_forward();
      }
      else if (last_controls.btn_right)
      {
        turn_right();
      }
      else if (last_controls.btn_backward)
      {
        move_backward();
      }
      else if (last_controls.btn_left)
      {
        turn_left();
      }
      else if (last_controls.btn_strfe_right)
      {
        move_strfe_right();
      }
      else if (last_controls.btn_stfre_left)
      {
        move_strfe_left();
      }
      else if (last_controls.btn_forward_right)
      {
        move_forward_right();
      }
      else if (last_controls.btn_forward_left)
      {
        move_forward_left();
      }
      else if (last_controls.btn_backward_right)
      {
        move_backward_right();
      }
      else if (last_controls.btn_backward_left)
      {
        move_backward_left();
      }
      else
      {
        stop_all();
      }

      if (last_controls.btn_pan_left)
      {
        panAngle -= 1;
      } // Slower servo speed
      if (last_controls.btn_pan_right)
      {
        panAngle += 1;
      }
      panAngle = constrain(panAngle, 0, 180);
      panServo.write(panAngle);

      // --- NEW DETAILED LED LOGIC ---
      bool isMovingForward = last_controls.btn_forward || last_controls.btn_forward_left || last_controls.btn_forward_right;
      bool isMovingBackward = last_controls.btn_backward || last_controls.btn_backward_left || last_controls.btn_backward_right;
      bool isTurningOrStrafing = last_controls.btn_left || last_controls.btn_right || last_controls.btn_stfre_left || last_controls.btn_strfe_right || last_controls.btn_forward_left || last_controls.btn_forward_right || last_controls.btn_backward_left || last_controls.btn_backward_right;

      if (isMovingForward || isMovingBackward || isTurningOrStrafing)
      {
        digitalWrite(GREEN_LED_PIN, isMovingForward);
        digitalWrite(RED_LED_PIN, isMovingBackward);
        digitalWrite(BLUE_LED_PIN, isTurningOrStrafing);
      }
      else
      { // Idle "Knight Rider" sequence
        if (millis() - lastIdleLedTime > 150)
        {
          lastIdleLedTime = millis();
          digitalWrite(RED_LED_PIN, idleLedState == 0 || idleLedState == 4);
          digitalWrite(GREEN_LED_PIN, idleLedState == 1 || idleLedState == 3);
          digitalWrite(BLUE_LED_PIN, idleLedState == 2);
          idleLedState = (idleLedState + 1) % 5;
        }
      }
    }
    else
    { // FAILSAFE_MODE
      if (!wasFailsafeActive)
      {
        patrolState = PATROL_FORWARD;
        Serial.println("\n[!] FAILSAFE ACTIVATED: Starting Dynamic Exploration Patrol");
      }
      wasFailsafeActive = true;

      if (millis() - lastPanSweepTime > 20)
      {
        lastPanSweepTime = millis();
        panAngle += pan_sweep_direction * 2;
        panServo.write(panAngle);
        if (panAngle >= 170 || panAngle <= 10)
        {
          pan_sweep_direction *= -1;
        }
      }
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, (millis() % 1000) < 50);

      switch (patrolState)
      {
      case PATROL_FORWARD:
        move_forward();
        if (ultrasonic_dist_cm < OBSTACLE_DISTANCE_CM)
        {
          Serial.println("[!] Obstacle detected! Starting maneuver.");
          state_entry_time = millis();
          patrolState = MANEUVER_START;
        }
        break;
      case MANEUVER_START:
        stop_all();
        state_entry_time = millis();
        patrolState = MANEUVER_BACKING_UP;
        break;
      case MANEUVER_BACKING_UP:
        move_backward();
        if (millis() - state_entry_time > MANEUVER_BACKUP_DURATION_MS)
        {
          stop_all();
          state_entry_time = millis();
          patrolState = MANEUVER_PAUSE_1;
        }
        break;
      case MANEUVER_PAUSE_1:
        if (millis() - state_entry_time > MANEUVER_PAUSE_MS)
        {
          state_entry_time = millis();
          patrolState = MANEUVER_LOOKING_RIGHT;
        }
        break;
      case MANEUVER_LOOKING_RIGHT:
        turn_right();
        if (millis() - state_entry_time > MANEUVER_TURN_DURATION_MS)
        {
          stop_all();
          state_entry_time = millis();
          patrolState = MANEUVER_SCANNING_RIGHT;
        }
        break;
      case MANEUVER_SCANNING_RIGHT:
        if (millis() - state_entry_time > MANEUVER_PAUSE_MS)
        {
          right_dist = ultrasonic_dist_cm;
          Serial.printf("[>] Right path clear: %d cm\n", right_dist);
          state_entry_time = millis();
          patrolState = MANEUVER_LOOKING_LEFT;
        }
        break;
      case MANEUVER_LOOKING_LEFT:
        turn_left();
        if (millis() - state_entry_time > (MANEUVER_TURN_DURATION_MS * 2))
        {
          stop_all();
          state_entry_time = millis();
          patrolState = MANEUVER_SCANNING_LEFT;
        }
        break;
      case MANEUVER_SCANNING_LEFT:
        if (millis() - state_entry_time > MANEUVER_PAUSE_MS)
        {
          left_dist = ultrasonic_dist_cm;
          Serial.printf("[<] Left path clear: %d cm\n", left_dist);
          state_entry_time = millis();
          patrolState = MANEUVER_DECIDING;
        }
        break;
      case MANEUVER_DECIDING:
        stop_all();
        if (right_dist > left_dist)
        {
          Serial.println("[→] Decision: Turning right.");
          turn_right();
        }
        else
        {
          Serial.println("[←] Decision: Turning left.");
          turn_left();
        }
        state_entry_time = millis();
        patrolState = MANEUVER_COMMITTING_TURN;
        break;
      case MANEUVER_COMMITTING_TURN:
        if (millis() - state_entry_time > MANEUVER_TURN_DURATION_MS)
        {
          Serial.println("[✓] Maneuver complete. Resuming patrol.");
          patrolState = PATROL_FORWARD;
        }
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =============================================================================
// --- MAIN SETUP AND LOOP ---
// =============================================================================
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32 Mecanum Robot 'Final Polished' ===");
  lastPacketTime = millis();
  init_motors();
  Serial.println("[✓] Motors Initialized");
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("[✓] Status Indicators Initialized.");
  ESP32PWM::allocateTimer(2);
  panServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN);
  panServo.write(83);
  Serial.println("[✓] Pan Servo Initialized.");
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  Serial.println("[✓] mmWave Radar Initialized.");
  if (!radio.begin())
  {
    Serial.println("[✗] FATAL: Radio failed!");
    while (1)
      ;
  }
  radio.setChannel(83);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(ROBOT_ADDRESS);
  radio.openReadingPipe(1, ROBOT_ADDRESS);
  Serial.println("[✓] Radio Initialized.");

  radarDataMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(radar_task, "RadarTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ultrasonic_task, "UltrasonicTask", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(robot_action_task, "ActionTask", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(radio_telemetry_task, "RadioTask", 4096, NULL, 4, NULL, 0);

  Serial.println("\n[✓] All systems go. Ready for commands.\n");
}

void loop()
{
  vTaskDelete(NULL);
}

// =============================================================================
// --- FULL FUNCTION DEFINITIONS ---
// =============================================================================
void init_motors()
{
  Wire.begin();
  if (!pcf8574.begin())
  {
    Serial.println("[✗] FATAL: PCF8574 not found!");
    while (1)
      ;
  }
  pcf8574.write8(0x00);
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION, .timer_num = LEDC_TIMER_0, .freq_hz = PWM_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);
  ledc_channel_config_t ledc_channels[4] = {
      {.gpio_num = FL_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0},
      {.gpio_num = RL_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0},
      {.gpio_num = FR_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_0, .duty = 0},
      {.gpio_num = RR_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_3, .timer_sel = LEDC_TIMER_0, .duty = 0}};
  for (int i = 0; i < 4; i++)
  {
    ledc_channel_config(&ledc_channels[i]);
  }
}
void set_motor_power(int ch, int speed)
{
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, speed);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}
void stop_all()
{
  pcf8574.write8(0x00);
  set_motor_power(LEDC_CHANNEL_0, 0);
  set_motor_power(LEDC_CHANNEL_1, 0);
  set_motor_power(LEDC_CHANNEL_2, 0);
  set_motor_power(LEDC_CHANNEL_3, 0);
}
void turn_right()
{
  stop_all();
  pcf8574.write(EXP_FL_IN2, HIGH);
  pcf8574.write(EXP_RL_IN4, HIGH);
  pcf8574.write(EXP_FR_IN1, HIGH);
  pcf8574.write(EXP_RR_IN3, HIGH);
  set_motor_power(LEDC_CHANNEL_0, TURN_SPEED);
  set_motor_power(LEDC_CHANNEL_1, TURN_SPEED);
  set_motor_power(LEDC_CHANNEL_2, TURN_SPEED);
  set_motor_power(LEDC_CHANNEL_3, TURN_SPEED);
}
void turn_left()
{
  stop_all();
  pcf8574.write(EXP_FR_IN2, HIGH);
  pcf8574.write(EXP_RR_IN4, HIGH);
  pcf8574.write(EXP_FL_IN1, HIGH);
  pcf8574.write(EXP_RL_IN3, HIGH);
  set_motor_power(LEDC_CHANNEL_0, TURN_SPEED);
  set_motor_power(LEDC_CHANNEL_1, TURN_SPEED);
  set_motor_power(LEDC_CHANNEL_2, TURN_SPEED);
  set_motor_power(LEDC_CHANNEL_3, TURN_SPEED);
}
void move_forward()
{
  stop_all();
  pcf8574.write(EXP_FL_IN2, HIGH);
  pcf8574.write(EXP_FR_IN2, HIGH);
  pcf8574.write(EXP_RL_IN4, HIGH);
  pcf8574.write(EXP_RR_IN4, HIGH);
  set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_backward()
{
  stop_all();
  pcf8574.write(EXP_FL_IN1, HIGH);
  pcf8574.write(EXP_FR_IN1, HIGH);
  pcf8574.write(EXP_RL_IN3, HIGH);
  pcf8574.write(EXP_RR_IN3, HIGH);
  set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_strfe_right()
{
  stop_all();
  pcf8574.write(EXP_FL_IN2, HIGH);
  pcf8574.write(EXP_RR_IN4, HIGH);
  pcf8574.write(EXP_FR_IN1, HIGH);
  pcf8574.write(EXP_RL_IN3, HIGH);
  set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_strfe_left()
{
  stop_all();
  pcf8574.write(EXP_FR_IN2, HIGH);
  pcf8574.write(EXP_RL_IN4, HIGH);
  pcf8574.write(EXP_FL_IN1, HIGH);
  pcf8574.write(EXP_RR_IN3, HIGH);
  set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_forward_right()
{
  stop_all();
  pcf8574.write(EXP_FL_IN2, HIGH);
  pcf8574.write(EXP_RR_IN4, HIGH);
  set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_2, 0);
  set_motor_power(LEDC_CHANNEL_1, 0);
  set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}
void move_forward_left()
{
  stop_all();
  pcf8574.write(EXP_FR_IN2, HIGH);
  pcf8574.write(EXP_RL_IN4, HIGH);
  set_motor_power(LEDC_CHANNEL_0, 0);
  set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_3, 0);
}
void move_backward_right()
{
  stop_all();
  pcf8574.write(EXP_FR_IN1, HIGH);
  pcf8574.write(EXP_RL_IN3, HIGH);
  set_motor_power(LEDC_CHANNEL_0, 0);
  set_motor_power(LEDC_CHANNEL_2, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_1, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_3, 0);
}
void move_backward_left()
{
  stop_all();
  pcf8574.write(EXP_FL_IN1, HIGH);
  pcf8574.write(EXP_RR_IN3, HIGH);
  set_motor_power(LEDC_CHANNEL_0, DRIVE_SPEED);
  set_motor_power(LEDC_CHANNEL_2, 0);
  set_motor_power(LEDC_CHANNEL_1, 0);
  set_motor_power(LEDC_CHANNEL_3, DRIVE_SPEED);
}