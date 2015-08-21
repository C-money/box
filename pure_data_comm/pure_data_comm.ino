/*

Message system Tester

 
 */
int LedPin = 13;
int stop = 0;
int VarSign = 1;
int SerIn = -1;          // incoming byte from serial RX
long VarN[9] = {  
  0, 0, 0, 0, 0, 0, 0, 0, 0 };      // number array
int PosN = -1;       // index counter
char VarHandlerChar = '@';       // 

// SERIAL PARSER **********************************
void SerialParser(void) {
  while (Serial.available()) {
    SerIn = '@';
    SerIn = Serial.read();
    // debugging CODE
    // Serial.print(' ');
    // Serial.print(SerIn);
    if ((SerIn >= 65) && (SerIn <= 90)) {          // if ASCII 'A' - 'Z'
      VarHandlerChar = SerIn;  
      for (PosN = 0; PosN < 9; PosN++) {           // clean up
        VarN[PosN] =  0;                           // reset Var array
      }  
      PosN = -1;                              
    }
    if (SerIn == 45) {
      VarSign = -1;
    }
    if ((SerIn >= 48) && (SerIn <= 57)) {          // if ASCII numeric '0' - '9'
      VarN[PosN] = VarN[PosN] * 10 + (SerIn - 48);
    }
    if (SerIn == ' ') {                            // if ASCII " " detected
      if (PosN > -1) {
        VarN[PosN] = VarSign * VarN[PosN];           // assign sign 
        VarSign = 1;
      }
      PosN++;
    }
    if (SerIn == 33 || SerIn == 10 || SerIn == 13) {  // '!' or CR or LF ends all this spooky things ...
      VarN[PosN] = VarSign * VarN[PosN];           // assign sign 
      VarSign = 1;
      CallHandler();                               // Call funktion by first Char
    }   // IfEND CR spooky things
  }
}

// CALLHANDLER SCHEDULER ****************************** 
// which of YOUR functions are called?
//

void CallHandler(void) {
  switch(VarHandlerChar) {
  case 'A': // do something when A
    //Function_Char('A');
    break;    
  case 'B':  // do something when B
    DebugOut();
    break;
  case 'L':   // 
    LED(VarN[0]);
    break;
  case 'P':  // 
    stopToggle();
    break;           
  } // Switch END
}

//
// CALLHANDLER FUNCTIONS *******************************
// YOUR funktions here:

void DebugOut(void) {
  //Serial.print("got an: ");
  
  Serial.print(VarHandlerChar, BYTE);
  for (int  i = 0; i < PosN; i++) {
    Serial.print(' ');
    Serial.print(VarN[i]);
  }
  Serial.println(" !");
}

void LED(int OnOff){
  if (OnOff <= 0 ){
    digitalWrite(LedPin, LOW);
  }
  else {
    digitalWrite(LedPin, HIGH);
  }
}

void stopToggle(void){
  stop = VarN[0];
}

void setup(void) {
  Serial.begin(115200);                 // Serialle Schnittstelle mit 115200 Baud 
  pinMode(LedPin, OUTPUT);
}

void loop(void) {
  SerialParser();
  stop = 1;
  if (stop == 1){
  Serial.print("A ");
  for(int i=0; i < 4; i++){
    Serial.print(random(1000));
    Serial.print(' ');
  } 
  Serial.println('!');
  delay(50);
  SerialParser();
  Serial.print(random(10)+65, BYTE);
  Serial.print(' ');
  for(int i=0; i < 4; i++){
    Serial.print(random(1000));
    Serial.print(' ');
  } 
  Serial.println('!');
  delay(50);
  }
}
