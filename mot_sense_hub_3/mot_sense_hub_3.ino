/* UART Example, any character received on either the real
   serial port, or USB serial (or emulated serial to the
   Arduino Serial Monitor when using non-serial USB types)
   is printed as a message to both ports.

   This example code is in the public domain.
*/

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1
int led = 13;
int sample_enable = 3;

#define N_MOTION_SENSORS_ACTIVE  29
int motion_sensor_pins[29] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};

int sound_tap_queue[29];
int sound_tap_timeout[29];

#define N_SCENES 10
// scene timeouts in seconds
int scene_timeouts[N_SCENES];// = {0, 0, 0, 5};
#define DEFAULT_SCENE    0
#define SCENE_CHANGE_IGNORE_TIME    1    // tilt for scene change ignored for this many seconds after last scene change
#define US_IN_S    1000000     // one million us in a second

#define SOUND_SCENE_CODE_OFFSET    232    // 232 is scene 0
#define SOUND_NO_UPDATE_CODE       255

#define MOTION_TILT_TIMEOUT_RESET   50
// scene control unresponsive to tilt events when sensor-specific timeout is non-zero
int motion_tilt_timeouts[N_SCENES] = {0, 0, 0, 0};

#define N_MOTION_SENSORS           30
#define OUTPUT_PACKET_START_INDEX  0
#define OUTPUT_PACKET_SCENE_INDEX  1
#define OUTPUT_PACKET_HEADER_SIZE  2
#define OUTPUT_PACKET_SIZE  (N_MOTION_SENSORS + OUTPUT_PACKET_HEADER_SIZE)
uint8_t output_packet[OUTPUT_PACKET_SIZE];

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200);
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  pinMode(sample_enable, INPUT_PULLUP);
  digitalWrite(2, LOW);    // serial GND here for now
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  for (int i = 0; i < N_MOTION_SENSORS_ACTIVE; i++) {
    pinMode(motion_sensor_pins[i], INPUT_PULLUP);
    sound_tap_timeout[i] = 1;
    sound_tap_queue[i] = 0;
  }

  output_packet[OUTPUT_PACKET_START_INDEX] = 254;

}

void loop() {
  int i, j, k, mot_sense_pin;
  uint8_t rx_char;
  int sound_byte;
  //static uint8_t last_rx_char;
  static long scene_start_time = 0;
  long clock_time_us;
  long time_since_scene_start = 0;
  static int current_scene = 0;
  static int last_scene = 0;

  //rx_char = 0;

  cli();

  for (i = 0; i < N_MOTION_SENSORS_ACTIVE; i++) {
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

  sei();

  clock_time_us = micros();
  time_since_scene_start = (clock_time_us - scene_start_time) / US_IN_S;

  for (i = 0; i < N_SCENES; i++) {
    if ((output_packet[i + OUTPUT_PACKET_HEADER_SIZE] & 0x80) == 0x80 && output_packet[i + OUTPUT_PACKET_HEADER_SIZE] != 255) {
      if (motion_tilt_timeouts[i] == 0) {
        if (time_since_scene_start >= SCENE_CHANGE_IGNORE_TIME) {
          current_scene = i;
          scene_start_time = clock_time_us;
          time_since_scene_start = 0;
        }
      }
      motion_tilt_timeouts[i] = MOTION_TILT_TIMEOUT_RESET;
    } else {
      if (motion_tilt_timeouts[i] > 0) {
        (motion_tilt_timeouts[i])--;
      }
    }
  }
  
  if (scene_timeouts[current_scene] != 0 && time_since_scene_start >= scene_timeouts[current_scene]) {
    current_scene = DEFAULT_SCENE;
    //scene_start_time = clock_time_us;
  }

  output_packet[OUTPUT_PACKET_SCENE_INDEX] = current_scene;

  for (i = 0; i < OUTPUT_PACKET_SIZE; i++) {
    if (output_packet[i] == 255) {
      output_packet[i] = 0;
    }
    if (i >= OUTPUT_PACKET_HEADER_SIZE) {
      output_packet[i] &= 0x7f;
    }
    //Serial.printf("%03d\n", output_packet[i]);
    //Serial.printf("%c", output_packet[i]);
    HWSERIAL.printf("%c", output_packet[i]);
  }
  //Serial.printf("\n");

  // SOUND BYTE TRANSMITION 
  
  // load motion events into tap queue, overwriting with higher tap values
  for (i = 0; i < N_MOTION_SENSORS_ACTIVE; i++) {
    if (sound_tap_timeout[i] > 1) {
      (sound_tap_timeout[i])--;
    }
    if (output_packet[i + OUTPUT_PACKET_HEADER_SIZE] > 48) { // consider cutting off taps < 16 or 32
      if (sound_tap_timeout[i] == 1) {
        if (output_packet[i + OUTPUT_PACKET_HEADER_SIZE] / 16 + 1 >= sound_tap_queue[i]) { // and if queue ready isn't counting down
          sound_tap_queue[i] = output_packet[i + OUTPUT_PACKET_HEADER_SIZE] / 16 + 1;
        } else {
          if (sound_tap_queue[i] != 0) {
            sound_tap_timeout[i] = 0;
          }
        }
      } //else { // set queue ready if queue wasn't
    }
  }

  sound_byte = SOUND_NO_UPDATE_CODE;
  
  if (current_scene != last_scene) {
    last_scene = current_scene;
    sound_byte = current_scene + SOUND_SCENE_CODE_OFFSET;
  } else {
    for (i = 0; i < N_MOTION_SENSORS_ACTIVE; i++) {
      if (sound_tap_queue[i] != 0) {
        sound_byte = i * 8 + (sound_tap_queue[i] - 1);
        sound_tap_queue[i] = 0;
        sound_tap_timeout[i] = 120;
        break;
      }
    }
  }
  
  
  if (sound_byte != 255) {
    Serial.printf("%c", sound_byte);
    //Serial.printf("%i\n", sound_byte);
  }
  
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

