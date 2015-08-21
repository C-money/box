/* UART Example, any character received on either the real
   serial port, or USB serial (or emulated serial to the
   Arduino Serial Monitor when using non-serial USB types)
   is printed as a message to both ports.

   This example code is in the public domain.
*/

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1
int led = 13;
int sample_enable = 14;

#define N_MOTION_SENSORS_ACTIVE  4
int motion_sensor_pins[4] = {0, 1, 2, 3};

#define N_MOTION_SENSORS           30
#define OUTPUT_PACKET_HEADER_SIZE  2
#define OUTPUT_PACKET_SIZE  (N_MOTION_SENSORS + OUTPUT_PACKET_HEADER_SIZE)
uint8_t output_packet[OUTPUT_PACKET_SIZE];

void setup() {
  Serial.begin(115200);
  //HWSERIAL.begin(115200);
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  pinMode(sample_enable, INPUT_PULLUP);
  /*digitalWrite(2, LOW);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);*/
  for (int i = 0; i < N_MOTION_SENSORS_ACTIVE; i++) {
    pinMode(motion_sensor_pins[i], INPUT_PULLUP);
  }
  
  output_packet[0] = 254;
  
}

void loop() {
  int us, i, j, k, mot_sense_pin;
  uint8_t rx_char;
  //static uint8_t last_rx_char;
  
  //rx_char = 0;
  
  cli();
  
  for (int i = 0; i < N_MOTION_SENSORS_ACTIVE; i++) {
    rx_char = 0;
    mot_sense_pin = motion_sensor_pins[i];
    pinMode(mot_sense_pin, OUTPUT);
    digitalWrite(mot_sense_pin, LOW);
    digitalWrite(mot_sense_pin, LOW);
    digitalWrite(mot_sense_pin, LOW);
    digitalWrite(mot_sense_pin, HIGH);
    pinMode(mot_sense_pin, INPUT_PULLUP);
    for (j = 0; j < 1000; j++) {    // give mot sensor some time to respond
      if (digitalRead(mot_sense_pin) == LOW) {    // first low indicated motion sensor is driving comm line
        break;
      }
    }
    //us = micros();
    for (j = 0; j < 41; j++);    // tuned delay
    for (j = 0; j < 8; j++) {
      rx_char |= (digitalRead(mot_sense_pin) << j);
      for (k = 0; k < 49; k++);    // tuned delay
    }
    output_packet[i + OUTPUT_PACKET_HEADER_SIZE] = rx_char;
  }
    
  for (i = 0; i < OUTPUT_PACKET_SIZE; i++) {
    //Serial.printf("%03d\n", output_packet[i]);
    Serial.printf("%c", output_packet[i]);
  }
  //Serial.printf("\n");
    
 /* pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  digitalWrite(12, LOW);
  digitalWrite(12, LOW);
  digitalWrite(12, HIGH);
  pinMode(12, INPUT_PULLUP);
  for (i = 0; i < 1000; i++) {    // give mot sensor some time to respond
    if (digitalRead(12) == LOW) {    // first low indicated motion sensor is driving comm line
      break;
    }
  }
  //us = micros();
  for (j = 0; j < 41; j++);    // tuned delay
  for (i = 0; i < 8; i++) {
    rx_char |= (digitalRead(12) << i);
    for (j = 0; j < 49; j++);    // tuned delay
  }
  us = micros();
  
  for (j = 0; j < 18; j++);    // tuned delay
  us = micros() - us;
  sei();
  //Serial.printf("rcvd: 0x%02x\n", rx_char);
  //Serial.printf("%c", rx_char);
  //HWSERIAL.printf("%c", rx_char);
  Serial.printf("%03d\n", rx_char);
  //Serial.printf("HI EVAN, %02x\n", rx_char);*/
  /*for (i = 7; i >= 0; i--) {
    if ((rx_char >> i) & 1) {
      Serial.printf("1");
    } else {
      Serial.printf("0");
    }
  } 
  Serial.printf("\n");*/
  //Serial.printf("%03d\n", rx_char);
  //if (rx_char != 0x55 && rx_char != 0xaa) {
  /*if (rx_char != ((last_rx_char + 1) & 0xff)) {
    Serial.printf("BAD COMM---------------------\n");
  }*/
  //last_rx_char = rx_char;
  
  /*if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("USB received: ");
    Serial.println(incomingByte, DEC);
  }*/


  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(5);               // wait for a second
  while (digitalRead(sample_enable) == HIGH);
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(5);               // wait for a second
}

