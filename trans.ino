#include <SPI.h>                  /* to handle the communication interface with the modem*/
#include <nRF24L01.h>             /* to handle this particular modem driver*/
#include <RF24.h>                 /* the library which helps us to control the radio modem*/
#include <printf.h>

RF24 radio(7,8);                    /* Creating instance 'radio'  ( CE , CSN )   CE -> D7 | CSN -> D8 */ 
/*  PINS
 *  VCC -> 3.3V                             
 *  CE -> D7 | CSN -> D8
 *  MOSI -> D11 | SCK -> D13
 *  MISO > D12
 */
const byte Address[5] = {0xe7,0xe7,0xe7,0xe7,0xe7} ;     /* Address to which data to be transmitted*/

void setup() {
  Serial.begin(9600);
  bool temp = radio.begin ();                 /* Activate the modem*/
  Serial.println(temp);
  
  radio.setAddressWidth(5); // 5bytes for address
  radio.setPALevel(RF24_PA_MIN); // power 
  radio.setPayloadSize(1);
  radio.setChannel(2); 
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_8);

  radio.stopListening (); // set as transmitter 
  radio.openWritingPipe (Address); /* Sets the address of transmitter to which program will send the data */

  printf_begin();
  radio.printDetails() ;//////////////////
  radio.stopListening ();
}


void loop() {
  radio.stopListening ();
  char data ='A';
  while (1){
  bool transmissionSuccess = radio.write(&data, sizeof(data));            /* Sending data over NRF 24L01*/
  
   if (transmissionSuccess) {
    //Transmission successful
    Serial.println("Transmission successful!");
  } else {
    // No ACK received or transmission failed
    Serial.println("Transmission failed or no ACK received!");
  }
  Serial.println(data);
  delay(1000);  // Add a delay between transmissions
  }
}
