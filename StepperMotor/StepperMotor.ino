/*-----( Import needed libraries )-----*/
#include <Stepper.h>

/*-----( Declare Constants, Pin Numbers )-----*/
//---( Number of steps per revolution of INTERNAL motor in 4-step mode )---
#define STEPS_PER_MOTOR_REVOLUTION 32   

//---( Steps per OUTPUT SHAFT of gear reduction )---
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048  

#define INTERRUPT 4

/*-----( Declare objects )-----*/
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to the pin connections need to be pins 0,1,2,3 connected
// to Motor Driver In1, In2, In3, In4 

// Then the pins are entered here in the sequence 1-3-2-4 for proper sequencing
Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 0, 2, 1, 3);

bool hasWheelTurned = false;


/*-----( Declare Variables )-----*/
int  Steps2Take;

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
// Nothing  (Stepper Library sets pins as outputs)
  delay(15000);
  pinMode(INTERRUPT, INPUT);
  while(digitalRead(INTERRUPT) != HIGH); // do nothing until feather tells me to do my thing
}/*--(end setup )---*/

void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if(hasWheelTurned == false)
  {
    pinMode(INTERRUPT, OUTPUT); // so I can talk back to feather
    small_stepper.setSpeed(1000);   // SLOWLY Show the 4 step sequence 
    Steps2Take  =  5*STEPS_PER_OUTPUT_REVOLUTION;  // Rotate CW
    small_stepper.step(Steps2Take);
    hasWheelTurned = true;
    digitalWrite(INTERRUPT, LOW); // tell feather that I'm done
  }

  delay(1000);
  
}/* --(end main loop )-- */

/* ( THE END ) */
