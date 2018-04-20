#define FORWARDHIGH 1
#define REVERSEHIGH 2
#define SWITCH 3
#define INTERRUPT 4

bool hasArmMoved = false;
void setup() {
  delay(15000);
  pinMode(FORWARDHIGH, OUTPUT);
  pinMode(REVERSEHIGH, OUTPUT);
  pinMode(INTERRUPT, INPUT);
  while(digitalRead(INTERRUPT) != HIGH); // do nothing until feather tells me to do my thing
}

void loop() 
{
  // motor power connected to out1
  // motor ground connected to out2
      if(hasArmMoved == false)
      {
        pinMode(INTERRUPT, OUTPUT); // so it can tell feather I'm done
        armForward(2000);
        delay(500);
        armReverse(2100);
        hasArmMoved = true;
        digitalWrite(INTERRUPT, LOW); // tell feather that I'm done
       }
      delay(1000);
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

