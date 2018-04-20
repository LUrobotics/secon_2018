#define IOPIN 0
bool hasSentMessage = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(IOPIN, INPUT);
  delay(1000);
  while(digitalRead(IOPIN) != HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(hasSentMessage == false){
    delay(2000);
    pinMode(IOPIN, OUTPUT);
    digitalWrite(IOPIN, LOW);
    hasSentMessage = true;
  }

  delay(10);
  
  
}
