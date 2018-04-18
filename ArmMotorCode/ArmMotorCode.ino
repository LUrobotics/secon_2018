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
  digitalWrite(SWITCH, HIGH);

  Serial.begin(9600);
  while(!Serial);
  Serial.println("Code started");
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // motor power connected to out1
  // motor ground connected to out2
//  while(digitalRead(SWITCH) != LOW){
//    armReverse(50);
//  }
  // check to see if time to move the arm
//  if (digitalRead(INTERRUPT) == LOW && hasArmMoved == false){
  // moves arm forward / lowers arm
  if(digitalRead(SWITCH) == LOW){
    Serial.println("Switch Pressed");
//    armForward(100);
  }
//  armForward(100);

  // moves arm back / lifts arm 
//  armReverse(500);
//  hasArmMoved = true;
//  }

//  delay(20000); // keep trinket from resetting
  
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

