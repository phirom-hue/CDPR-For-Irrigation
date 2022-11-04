// For RAMPS 1.4
#define X_DIR_PIN          22
#define X_STEP_PIN         23


#define Y_DIR_PIN          24
#define Y_STEP_PIN         25


#define Z_DIR_PIN          26
#define Z_STEP_PIN         27


#define A_DIR_PIN          28
#define A_STEP_PIN         29



#define X_STEP_HIGH             PORTA |=  0b00000010;
#define X_STEP_LOW              PORTA &= ~0b00000010;

#define Y_STEP_HIGH             PORTA |=  0b00001000;
#define Y_STEP_LOW              PORTA &= ~0b00001000;

#define Z_STEP_HIGH             PORTA |=  0b00100000;
#define Z_STEP_LOW              PORTA &= ~0b00100000;

#define A_STEP_HIGH             PORTA |=  0b10000000;
#define A_STEP_LOW              PORTA &= ~0b10000000;


#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned int minStepInterval;   // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
};

void xStep() {
  X_STEP_HIGH
  X_STEP_LOW
}
void xDir(int dir) {
  digitalWrite(X_DIR_PIN, dir);
}

void yStep() {
  Y_STEP_HIGH
  Y_STEP_LOW
}
void yDir(int dir) {
  digitalWrite(Y_DIR_PIN, dir);
}

void zStep() {
  Z_STEP_HIGH
  Z_STEP_LOW
}
void zDir(int dir) {
  digitalWrite(Z_DIR_PIN, dir);
}

void aStep() {
  A_STEP_HIGH
  A_STEP_LOW
}
void aDir(int dir) {
  digitalWrite(A_DIR_PIN, dir);
}

void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

#define NUM_STEPPERS 4

volatile stepperInfo steppers[NUM_STEPPERS];

void setup() {

  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);


  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);


  pinMode(Z_STEP_PIN,   OUTPUT);
  pinMode(Z_DIR_PIN,    OUTPUT);


  pinMode(A_STEP_PIN,   OUTPUT);
  pinMode(A_DIR_PIN,    OUTPUT);


  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  
  interrupts();

  steppers[0].dirFunc = xDir;
  steppers[0].stepFunc = xStep;
  steppers[0].acceleration = 1000;
  steppers[0].minStepInterval = 1000;

  steppers[1].dirFunc = yDir;
  steppers[1].stepFunc = yStep;
  steppers[1].acceleration = 1000;
  steppers[1].minStepInterval = 1000;

  steppers[2].dirFunc = zDir;
  steppers[2].stepFunc = zStep;
  steppers[2].acceleration = 1000;
  steppers[2].minStepInterval = 1000;

  steppers[3].dirFunc = aDir;
  steppers[3].stepFunc = aStep;
  steppers[3].acceleration = 1000;
  steppers[3].minStepInterval = 1000;

}

void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
}

volatile byte remainingSteppersFlag = 0;

void prepareMovement(int whichMotor, int steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  remainingSteppersFlag |= (1 << whichMotor);
}

volatile byte nextStepperFlag = 0;

volatile int ind = 0;
volatile unsigned int intervals[100];

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned int mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;

    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}

void runAndWait() {
  setNextInterruptInterval();
  while ( remainingSteppersFlag );
}

void loop() {

  TIMER1_INTERRUPTS_ON

 for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, -300 );
    prepareMovement( 2, -300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(1000);

/////////////////////////////////////////////
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 400 );
    prepareMovement( 1, 400 );
    prepareMovement( 2, -400 );
    prepareMovement( 3, -400 );
  runAndWait();
  delay(500);
  
/////////////////////////////////////////////////////
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 600 );
    prepareMovement( 1, -600 );
    prepareMovement( 2, -600 );
    prepareMovement( 3, 600 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);
  
/////////////////////////////////////////////////////
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -200 );
    prepareMovement( 1, -200 );
    prepareMovement( 2, 200 );
    prepareMovement( 3, 200 );
  runAndWait();
  delay(500);
  
////////////////////////////////////////////////////  
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 600 );
    prepareMovement( 1, -600 );
    prepareMovement( 2, -600 );
    prepareMovement( 3, 600 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);
  
//////////////////////mid/////////////////////////  
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -200 );
    prepareMovement( 1, -200 );
    prepareMovement( 2, 200 );
    prepareMovement( 3, 200 );
  runAndWait();
  delay(500);

 /////////////////////////////////////////////
   for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 600 );
    prepareMovement( 1, -600 );
    prepareMovement( 2, -600 );
    prepareMovement( 3, 600 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);

/////////////////////////////////////////////////////
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -200 );
    prepareMovement( 1, -200 );
    prepareMovement( 2, 200 );
    prepareMovement( 3, 200 );
  runAndWait();
  delay(500);

 /////////////////////////////////////////////
   for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 600 );
    prepareMovement( 1, -600 );
    prepareMovement( 2, -600 );
    prepareMovement( 3, 600 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);  

///////////////////////////////////////////////  
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -200 );
    prepareMovement( 1, -200 );
    prepareMovement( 2, 200 );
    prepareMovement( 3, 200 );
  runAndWait();
  delay(500);

  /////////////////////////////////////////////
   for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 600 );
    prepareMovement( 1, -600 );
    prepareMovement( 2, -600 );
    prepareMovement( 3, 600 );
  runAndWait();
  delay(500);

  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, -300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, -300 );
  runAndWait();
  delay(500);  
  
 //////////////////////////////////////////////
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 400 );
    prepareMovement( 1, 400 );
    prepareMovement( 2, -400 );
    prepareMovement( 3, -400 );
  runAndWait();
  delay(1000);
  
 ////////////////////////////////
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( 0, 300 );
    prepareMovement( 1, 300 );
    prepareMovement( 2, 300 );
    prepareMovement( 3, 300 );
  runAndWait();

  while (true);

}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
