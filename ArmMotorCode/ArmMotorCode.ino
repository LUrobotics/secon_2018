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
  pinMode(SWITCH, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  // motor power connected to out1
  // motor ground connected to out2

  // check to see if time to move the arm
  if (digitalRead(INTERRUPT) == LOW && hasArmMoved == false){
  // moves arm forward / lowers arm
  armForward(2000);

  // moves arm back / lifts arm 
  armReverse(2000);
  hasArmMoved = true;
  }

  delay(10); // keep trinket from resetting
  
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

