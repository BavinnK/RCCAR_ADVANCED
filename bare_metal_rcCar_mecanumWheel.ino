
#include "MyUSART.h"
#define clk_speed 16000000
#define baud 9600
#define my_ubrr (clk_speed/16/baud-1)

//L298N_1 
#define ENA_L PB7  //left motor speed connected to OC0A
#define FL_IN1 PA1 //1nput 1
#define FL_IN2 PA3 //input 2
#define RL_IN3 PA5 //input 3
#define RL_IN4 PA7 //input 4

//L298N_2
#define ENB_R PG5  //left motor speed connected to OC0B
#define FR_IN1 PA0 //1nput 1
#define FR_IN2 PA2 //1nput 2
#define RR_IN3 PA4 //1nput 3
#define RR_IN4 PA6 //1nput 4

#define BUZZ PC4
const uint8_t leds[3]={PC6,PC4,PC2};//2=RED,3=BLUE,4=GREEN



//TCCR1A=(1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);
//TCCR1B=(1<<WGM12)|(1<<CS11)|(1<<CS10);
void ENA_speed(uint8_t duty){
  OCR0A=duty;
}
void ENB_speed(uint8_t duty){
  OCR0B=duty;
}
//this one is for ENB AND ENA for the L298N motor driver
void set_timer0(void){
  TCCR0A|=(1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00);//FASTPWM 
  TCCR0B|=(1<<CS01)|(1<<CS00);//64 PRESCALER
}
//////////////////////////////////////////////////////////////////////////////////////////////// direction functions
void stop_all(void){
  PORTA&=~((1<<FR_IN2)|(1<<RR_IN4)|(1<<FL_IN2)|(1<<RL_IN4)|(1<<FR_IN1)|(1<<RR_IN3)|(1<<FL_IN1)|(1<<RL_IN3));

}
void move_forward(void){
  stop_all();
  PORTA|=(1<<FR_IN2)|(1<<RR_IN4)|(1<<FL_IN2)|(1<<RL_IN4);
  PORTC|=(1<<leds[2]);
}
void move_backward(void){
  stop_all();
  PORTA|=(1<<FR_IN1)|(1<<RR_IN3)|(1<<FL_IN1)|(1<<RL_IN3);
  PORTC|=(1<<leds[0]);
}

void move_left_turning(void){
  stop_all();
  PORTA|=(1<<FL_IN1)|(1<<RL_IN3)|(1<<FR_IN2)|(1<<RR_IN4);
  PORTC|=(1<<leds[1]);
}
void move_right_turning(void){
  stop_all();
  PORTA|=(1<<FL_IN2)|(1<<RL_IN4)|(1<<FR_IN1)|(RR_IN3);
  PORTC|=(1<<leds[1]);
}
void move_right(void){//carb walk right 
  stop_all();
  PORTA|=(1<<FL_IN1)|(1<<RL_IN4)|(1<<FR_IN2)|(1<<RR_IN3);
  PORTC|=(1<<leds[1]);
}
void move_left(void){//crab walk left
  PORTA|=(1<<FL_IN2)|(1<<RL_IN3)|(1<<FR_IN1)|(1<<RR_IN4);
  PORTC|=(1<<leds[1]);
}
void move_forward_right(void){
  stop_all();
  PORTA|=(1<<RL_IN4)|(1<<FR_IN2);
}
void move_forward_left(void){
  stop_all();
  PORTA|=(1<<FL_IN2)|(1<<RR_IN4);
}
void move_backward_right(void){
  stop_all();
  PORTA|=(1<<FL_IN1)|(1<<RR_IN3);
}
void move_backward_left(void){
  stop_all();
  PORTA|=(1<<RL_IN3)|(1<<FR_IN1);
}



//////////////////////////////////////////////////////////////////////////////////////////////// end

void off_leds(void){
  PORTC&=~((1<<leds[0])|(1<<leds[1])|(1<<leds[2]));
}

void setup(){
  USART_init(my_ubrr);
  set_timer0();
  //set_timer1();//for the buzzer

  
  DDRA|=(1<<FL_IN1)|(1<<FL_IN2)|(1<<FR_IN1)|(1<<FR_IN2)|(1<<RL_IN3)|(1<<RL_IN4)|(1<<RR_IN3)|(RR_IN4);
  DDRC|=(1<<leds[0])|(1<<leds[1])|(1<<leds[2])|(1<<BUZZ);
  DDRB|=(1<<ENA_L);
  DDRG|=(1<<ENB_R);

  ENA_speed(127);//means half speed
  ENB_speed(127);//same
  off_leds();//making sure that we set those to low
  PORTC&=~(1<<BUZZ);
  

}
void loop() {
  
  

    
  }
  