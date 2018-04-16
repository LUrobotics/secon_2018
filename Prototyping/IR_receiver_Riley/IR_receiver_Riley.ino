
#include <IRremote.h>

int RECV_PIN = 2;

IRrecv irrecv(RECV_PIN);

decode_results results;

unsigned long irCode;
unsigned int xMarksTheSpot;

void setup()
{
  Serial.begin(9600);
  // In case the interrupt driver crashes on setup, give a clue
  // to the user what's going on.
  Serial.println("Enabling IRin");
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("Enabled IRin");
}

void loop() {
  if (irrecv.decode(&results)) {
    irCode = results.value;
    Serial.println(irCode, HEX);
    xMarksTheSpot = decodeBootyLocation(irCode);
    Serial.println(xMarksTheSpot);
    irrecv.resume(); // Receive the next value 
  }
  delay(100);
}

int decodeBootyLocation(unsigned long bootyLocation) {
  int roadMap;
  switch(bootyLocation) {
    case(0X1AF66ED4):
      roadMap = 1;
      break;
    case(0X17F66A1D):
      roadMap = 2;
      break;
    case(0XA4E2155E):
      roadMap = 3;
      break;
    case(0XA3E213CD):
      roadMap = 4;
      break;
    case(0XA8726262):
      roadMap = 5;
      break;
    case(0XA97263F7):
      roadMap = 6;
      break;
    case(0XB490A256):
      roadMap = 7;
      break;
    case(0XB390A0C5):
      roadMap = 8;
      break;
  }
  return roadMap;
}


