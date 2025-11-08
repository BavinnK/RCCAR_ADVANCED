//nano as a middle man
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

struct packet_to_nano{
  int16_t x;
  int16_t y;
  //int16_t btn_right;
  //int16_t btn_left;

};
RF24 radio(9,10); // CE, CSN 
const byte ADDR[6] = "93221";


void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println("RX diag start");
  if(!radio.begin()){ Serial.println("ERROR: radio.begin failed"); while(1); }
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_HIGH);
radio.setRetries(5, 15);

  radio.openReadingPipe(0, ADDR);
  radio.startListening();
  Serial.print("isChipConnected(): ");
  Serial.println(radio.isChipConnected() ? "YES":"NO");
  radio.printDetails();
  DDRB|=(1<<PB2);//we make sure that SS pin is high 
  PORTB|=(1<<PB2);

}

void loop(){
  const uint8_t HEADER = 0xBB; // any value you like
  
  packet_to_nano packet;
  if(radio.available()){
    //Serial.println("gay");
    
    radio.read(&packet, sizeof(packet));
    Serial.write(HEADER);
    Serial.write((uint8_t *)&packet,sizeof(packet));
    //Serial.println(packet.x);
    //Serial.println(packet.y);
    
    delay(30);
   
  }
  
}
