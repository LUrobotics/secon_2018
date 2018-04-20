#define FORWARDHIGH 1
#define REVERSEHIGH 2
#define SWITCH 3
#define INTERRUPT 4

bool hasArmMoved = false;
void setup() {
  // put your setup code here, to run once:
  pinMode(FORWARDHIGH, OUTPUT);
  pinMode(REVERSEHIGH, OUTPUT);
  pinMode(INTERRUPT, INPUT);
  digitalWrite(INTERRUPT, HIGH);
delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // motor power connected to out1
  // motor ground connected to out2
      if(digitalRead(INTERRUPT) == LOW && hasArmMoved == false){
        armForward(2000);
        delay(500);
        armReverse(2100);
        hasArmMoved = true;
       }
      delay(10);

//      armForward(1500);
//      delay(2000);
//      armReverse(1500);
//      delay(20000); // keep trinket from resetting
  
}

void armForward(int delayTime){
  digitalWrite(FORWARDHIGH, HIGH);
  digitalWrite(REVERSEHIGH, LOW);
  delay(delayTime);
  digitalWrite(FORWARDHIGH, LOW);
//  digitalWrite(REVERSEHIGH, LOW);
}

void armReverse(int delayTime){
  digitalWrite(FORWARDHIGH, LOW);
  digitalWrite(REVERSEHIGH, HIGH);
  delay(delayTime);
//  digitalWrite(FORWARDHIGH, LOW);
  digitalWrite(REVERSEHIGH, LOW);
}

