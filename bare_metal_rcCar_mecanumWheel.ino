

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

//#define BUZZ PC4
const uint8_t leds[3]={PC6,PC4,PC2};//2=RED,3=BLUE,4=GREEN
#define x_pin PC7 //channel 3
#define y_pin PC5 //channel 1
#define turn_pin PC3 // channel 4
#define dead_zone 25 





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
void stop_all(void){
  PORTA&=~((1<<FR_IN2)|(1<<RR_IN4)|(1<<FL_IN2)|(1<<RL_IN4)|(1<<FR_IN1)|(1<<RR_IN3)|(1<<FL_IN1)|(1<<RL_IN3));

}
//////////////////////////////////////////////////////////////////////////////////////////////// direction functions
//these functions are useleess like wtf i was thinking i m using a tranmitter with joy stick and i wrote this omg im an idiot lets right another one but for joy stick
#if 0

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


#endif
//////////////////////////////////////////////////////////////////////////////////////////////// end

void off_leds(void){
  PORTC&=~((1<<leds[0])|(1<<leds[1])|(1<<leds[2]));
}
void set_motor(uint8_t motor,int speed){
  uint8_t IN_pin1,IN_pin2;
  switch(motor){
    case 1:
      IN_pin1=FL_IN1;
      IN_pin2=FL_IN2;
      break;
    case 2:
      IN_pin1=FR_IN1;
      IN_pin2=FR_IN2;
      break;
    case 3:
      IN_pin1=RL_IN3;
      IN_pin2=RL_IN4;
      break;
    case 4:
      IN_pin1=RR_IN3;
      IN_pin2=RR_IN4;
      break;
  }
  if(speed>dead_zone){//if speed is bigger than deadzone means that specific motor go forward
    stop_all();
    PORTA&=~(1<<IN_pin1);
    PORTA|=(1<<IN_pin2);
  }
  else if(speed<-dead_zone){//if speed is smaller than deadzone means that specific motor go backward
    PORTA|=(1<<IN_pin1);
    PORTA&=(1<<IN_pin2);
  }
  else{
    PORTA&=~((1<<IN_pin1)|(1<<IN_pin2));
  }
}

void setup(){
  //USART_init(my_ubrr);
  set_timer0();
  //set_timer1();//for the buzzer
  Serial.begin(9600);

  
  DDRA|=(1<<FL_IN1)|(1<<FL_IN2)|(1<<FR_IN1)|(1<<FR_IN2)|(1<<RL_IN3)|(1<<RL_IN4)|(1<<RR_IN3)|(1<<RR_IN4);
  DDRC|=(1<<leds[0])|(1<<leds[1])|(1<<leds[2]);
  DDRB|=(1<<ENA_L);
  DDRG|=(1<<ENB_R);
  DDRC&=~((1<<x_pin)|(1<<y_pin)|(1<<turn_pin));

  ENA_speed(255);//means half speed
  ENB_speed(255);//same
  off_leds();//making sure that we set those to low
  //PORTC&=~(1<<BUZZ);
  

}
void loop() {
    int xVal_raw=pulseIn(x_pin,1, 25000);//means read the data for 25 ms straight 
    int yVal_raw=pulseIn(y_pin,1, 25000);
    int turnVal_raw=pulseIn(turn_pin,1, 25000); 
    //and now we will map that raw data to usable data
    uint16_t xVal_data=map(xVal_raw, 1000, 2000, -255, 255);//u may ask why from 1000 to 2000 bc thats the high time of the pwm we get from the IA8 the value is bettween those two if
    //we gett 2000 means the joystick is in the most foward , 1000 most backward, 1500 in the middle
    uint16_t yVal_data=map(yVal_raw, 1000, 2000, -255, 255);
    uint16_t turnVal_data=map(turnVal_raw, 1000, 2000, -255, 255);
    //now this math i got from AI LOL the speed calculation of each motor 
    int fl_power = yVal_data - xVal_data + turnVal_data;
    int fr_power = yVal_data + xVal_data - turnVal_data;
    int rl_power = yVal_data + xVal_data + turnVal_data;
    int rr_power = yVal_data - xVal_data - turnVal_data;

    set_motor(1, fl_power);
    set_motor(2, fr_power);
    set_motor(3, rl_power);
    set_motor(4, rr_power);
   
   Serial.print("FL: "); Serial.print(fl_power);
   Serial.print("\t FR: "); Serial.print(fr_power);
   Serial.print("\t RL: "); Serial.print(rl_power);
   Serial.print("\t RR: "); Serial.println(rr_power);

  delay(100); // Slow down the printing
   

    
  }
  