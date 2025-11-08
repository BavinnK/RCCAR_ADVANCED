#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <avr/interrupt.h>
struct packet_to_nano{
  int8_t btn_forward;
  int8_t btn_forward_right;
  int8_t btn_right;
  int8_t btn_backward_right;
  int8_t btn_backward;
  int8_t btn_backward_left;
  int8_t btn_left;
  int8_t btn_forward_left;
  int8_t btn_strfe_right;
  int8_t btn_stfre_left;
  
  //int16_t btn_right;
  //int16_t btn_left;

};
enum btn_map{
  btn_forward=PD7,
  btn_forward_right=PD6,
   btn_right=PD5,
   btn_backward_right=PD4,
   btn_backward=PD3,
   btn_backward_left=PD2,
   btn_left=PC0,
   btn_forward_left=PC1,
   btn_strfe_right=PC2,
   btn_stfre_left=PC3

};
/*void adc_begin(void){
  ADMUX|=(1<<REFS0);
  ADCSRA|=(1<<ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
}
uint16_t adc_read(uint8_t pin){
  ADMUX=(ADMUX&0b11110000)|(pin&0b00001111);
  ADCSRA|=(1<<ADSC);
  while(ADCSRA&(1<<ADSC));
  return ADC;
}*/
volatile uint64_t counter=0;
void set_timer0(void){
  //WE SET THE PRESCALER TO 64
  cli();
  TCCR0A=(1<<WGM01);
  TCCR0B=(1<<CS01)|(1<<CS00);
  OCR0A=250;
  TIMSK0=(1<<OCIE0A);
  sei();
  
}

void delay_ms(uint16_t ms){
  uint16_t now=counter;
   while((uint16_t)(counter-now)<ms);
    
  }

ISR(TIMER0_COMPA_vect){
  counter++;
}
RF24 radio(9, 10); // CE, CSN
const byte ADDRESS[6] = "93221";



void setup() {
  Serial.begin(115200);

  //adc_begin();
  set_timer0();
  delay(1000); // give Mega time to start listening
  DDRD&=~((1<<btn_forward)|(1<<btn_forward_right)|(1<<btn_right)|(1<<btn_backward_right)|(1<<btn_backward)|(1<<btn_backward_left));
  DDRC&=~((1<<btn_left)|(1<<btn_forward_left)|(1<<btn_strfe_right)|(1<<btn_stfre_left));
  PORTD|=(1<<btn_forward)|(1<<btn_forward_right)|(1<<btn_right)|(1<<btn_backward_right)|(1<<btn_backward)|(1<<btn_backward_left);//enable pullup
  PORTC|=(1<<btn_left)|(1<<btn_forward_left)|(1<<btn_strfe_right)|(1<<btn_stfre_left);

  //Serial.println("=== uno TX Test ===");

  if (!radio.begin()) {
    Serial.println("ERROR: radio.begin() failed");
    while(1) delay(1000);
  }

  Serial.print("isChipConnected(): ");
  Serial.println(radio.isChipConnected() ? "YES" : "NO");

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(5, 15);


  radio.openWritingPipe(ADDRESS);
  radio.stopListening();
  Serial.println("Nano ready to send.");
}

void loop() {
  packet_to_nano packet;
  memset(&packet, 0, sizeof(packet));//we will assign all to zero before the if chain, and we dont need an else
  if(!(PIND&(1<<btn_forward)))            packet.btn_forward=1;
  else if(!(PIND&(1<<btn_forward_right))) packet.btn_forward_right=1;
  else if(!(PIND&(1<<btn_right)))         packet.btn_right=1;
  else if(!(PIND&(1<<btn_backward_right)))packet.btn_backward_right=1;
  else if(!(PIND&(1<<btn_backward)))      packet.btn_backward=1;
  else if(!(PIND&(1<<btn_backward_left))) packet.btn_backward_left=1;
  else if(!(PINC&(1<<btn_left)))          packet.btn_left=1;
  else if(!(PINC&(1<<btn_forward_left)))  packet.btn_forward_left=1;
  else if(!(PINC&(1<<btn_stfre_left)))    packet.btn_stfre_left=1;
  else if(!(PINC&(1<<btn_strfe_right)))   packet.btn_strfe_right=1;
  


  bool success = radio.write(&packet, sizeof(packet));
 
  delay_ms(30);//we use our driver so the mcu doesnt float in that 30ms, wee need all the time and power we have
  

  
}
