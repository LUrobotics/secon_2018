#define IOPIN 5
//bool hasSentMessage = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(IOPIN, OUTPUT);
  digitalWrite(IOPIN, LOW);
  delay(5000);
  digitalWrite(IOPIN, HIGH);
  pinMode(IOPIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  if(hasSentMessage == false){
    if(digitalRead(IOPIN) == LOW){
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
  }

  delay(10);
  
}
