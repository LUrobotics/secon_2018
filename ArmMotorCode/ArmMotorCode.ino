#define FORWARDHIGH 1
#define REVERSEHIGH 2

void setup() {
  // put your setup code here, to run once:
  pinMode(FORWARDHIGH, OUTPUT);
  pinMode(REVERSEHIGH, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  // motor power connected to out1
  // motor ground connected to out2
  
  // moves arm forward / lowers arm
  armForward(2000);

  // moves arm back / lifts arm 
  armReverse(2000);
  
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

