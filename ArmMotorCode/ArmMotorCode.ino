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
  digitalWrite(FORWARDHIGH, HIGH);
  digitalWrite(REVERSEHIGH, LOW);
  delay(10000);
  digitalWrite(FORWARDHIGH, LOW);
  digitalWrite(REVERSEHIGH, LOW);
  delay(5000);

  // moves arm back / lifts arm 
  digitalWrite(FORWARDHIGH, LOW);
  digitalWrite(REVERSEHIGH, HIGH);
  delay(10000);
  digitalWrite(FORWARDHIGH, LOW);
  digitalWrite(REVERSEHIGH, LOW);
  delay(5000);
  
}

