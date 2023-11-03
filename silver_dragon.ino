///////////////////////////////////////////////////////////////
/////////////////////// SETINGS ///////////////////////////////
///////////////////////////////////////////////////////////////
// motor's pins
int PWMA = 3;
int PWMB = 5;
int AIN_1 = 6;
int AIN_2 = 7;
int BIN_1 = 11;
int BIN_2 = 10;
int stdby = 13;

int step = 9;          // шаг изменения скорости
//#define GyverMotor      // Использование библиотеки с плавным управлением скорости
#define SimplaDrive      // Использование функции DRIVE

#define SVAP_MOTORS 
#define USE_IMP       // Использования импеллера

#define SPEED_MIN (1000)         // Минимальная скорость импеллера
#define SPEED_MAX (1650)         // Максимальная скорость импеллера

float max = 180; //142

float KP = 0.052;
float KD = 1.0;       


//float KI = 0.000; //0.000;

#define FIRST_FILTRE 550         // Первый фильтр значений
#define SECOND_FILTRE 110         // Второй фильтр значений

const int START_DELAY = 200;    // Задержка запуска
const char cmdSTART = 'l';       // Команда старта
const char cmdSTOP = 's';        // Команда стопа
const char cmdCOLIB = 'a';       // Команда Автокалибровки

///////////////////////////////////////////////////////////////

#define LED 13
#define BUTTON 8
const byte NUM_SENS = 8;


#ifdef USE_IMP
#define IMP 9
#include "ESC.h"

ESC myESC (IMP, SPEED_MIN, SPEED_MAX, 1000);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
int oESC;                                                 // Variable for the speed sent to the ESC
#endif


float lastError;
float _lastPosition;
float leftSpeed, rightSpeed;
int integral, last_proportional;
// address in EEPROM to save and load calibrations
const unsigned ADDRESS = 400;
int calibratedMinimum[NUM_SENS];
int calibratedMaximum[NUM_SENS];
int sensorMin[NUM_SENS];
int sensorMax[NUM_SENS];
int sensorValues[NUM_SENS];



// functions declaration
void pinsSetup();
//void loadEEPROM();
//void saveEEPROM();
bool pressButton();
bool isCmdBT(char cmd);

void setup() {

  pinsSetup();
  //loadEEPROM();
  Serial.begin(9600);
  digitalWrite(stdby, HIGH);
  while(true)
  {    
    if (pressButton() || isCmdBT(cmdCOLIB)) 
    {
      delay(500);
      #ifdef USE_IMP
        myESC.arm();
      #endif
      LINE_COLIB();
      break;
    } 
  }

}

void loop() {
  
  if (pressButton() || isCmdBT(cmdSTART)) 
  {
      #ifdef USE_IMP
        myESC.arm();
      #endif
    #ifdef USE_IMP
      for (oESC = SPEED_MIN; oESC <= SPEED_MAX; oESC += 1)  // goes from 1000 microseconds to 2000 microseconds
    {  
      myESC.speed(oESC);                                    // tell ESC to go to the oESC speed value
      delay(10);                                            // waits 10ms for the ESC to reach speed
    }
    #endif
    
    //here is your main logic following line
    delay(START_DELAY);   
    PID();
  }
} // loop


//**********************************************************



void pinsSetup() 
{
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(stdby, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  digitalWrite(LED, LOW);


#ifdef USE_IMP
  myESC.arm();
  delay(5000);
  digitalWrite(LED, HIGH);
#endif

}
/*
void loadEEPROM() 
{
  unsigned address = ADDRESS;
  for (int i = 0; i < NUM_SENS; ++i) {
    EEPROM.get(address, calibratedMinimum[i]);
    EEPROM.get(address + 2, calibratedMaximum[i]);
    address += 4;
  }
}
void saveEEPROM() 
{
  unsigned address = ADDRESS;
  for (int i = 0; i < NUM_SENS; i++) {
    EEPROM.put(address, calibratedMinimum[i]);
    EEPROM.put(address + 2, calibratedMaximum[i]);
    address += 4;
  }
}
*/

bool pressButton() 
{
  return !digitalRead(BUTTON);
}

bool isCmdBT(char cmd) 
{
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == cmd) {
      return true;
    }
  }
  return false;
}

void LINE_COLIB()
{
  delay(500);
  for (int i = 0; i < 8; i++)
  {
    sensorMin[i] = 4096;
  }
  int startColobTime = millis();
  while (millis() < (startColobTime + 5000)) 
  {

        #ifdef SimplaDrive
          drive(50, -50);
        #endif

        #ifdef GyverMotor
          drive(50, -50);
        #endif

    // Grabs incoming data from the photosensor
    for (int i = 0; i < 8; i++)
    {

      sensorValues[0] = analogRead(A0);
      if ( sensorValues[0] > sensorMax[0]) {
        sensorMax[0] = sensorValues[0];
      }
      if ( sensorValues[0] < sensorMin[0]) {
        sensorMin[0] = sensorValues[0];
      }

      sensorValues[1] = analogRead(A1);
      if ( sensorValues[1] > sensorMax[1]) {
        sensorMax[1] = sensorValues[1];
      }
      if ( sensorValues[1] < sensorMin[1]) {
        sensorMin[1] = sensorValues[1];
      }

      sensorValues[2] = analogRead(A2);
      if ( sensorValues[2] > sensorMax[2]) {
        sensorMax[2] = sensorValues[2];
      }
      if ( sensorValues[2] < sensorMin[2]) {
        sensorMin[2] = sensorValues[2];
      }

      sensorValues[3] = analogRead(A3);
      if ( sensorValues[3] > sensorMax[3]) {
        sensorMax[3] = sensorValues[3];
      }
      if ( sensorValues[3] < sensorMin[3]) {
        sensorMin[3] = sensorValues[3];
      }

      sensorValues[4] = analogRead(A4);
      if ( sensorValues[4] > sensorMax[4]) {
        sensorMax[4] = sensorValues[4];
      }
      if ( sensorValues[4] < sensorMin[4]) {
        sensorMin[4] = sensorValues[4];
      }


      sensorValues[5] = analogRead(A5);
      if ( sensorValues[5] > sensorMax[5]) {
        sensorMax[5] = sensorValues[5];
      }
      if ( sensorValues[5] < sensorMin[5]) {
        sensorMin[5] = sensorValues[5];
      }

      sensorValues[6] = analogRead(A6);
      if ( sensorValues[6] > sensorMax[6]) {
        sensorMax[6] = sensorValues[6];
      }
      if ( sensorValues[6] < sensorMin[6]) {
        sensorMin[6] = sensorValues[6];
      }

      sensorValues[7] = analogRead(A7);
      if ( sensorValues[7] > sensorMax[7]) {
        sensorMax[7] = sensorValues[7];
      }
      if ( sensorValues[7] < sensorMin[7]) {
        sensorMin[7] = sensorValues[7];
      }
    }
      #ifdef SimplaDrive
          drive(0, 0);
      #endif

      #ifdef GyverMotor
         drive(0,0);
      #endif
  }

  for(int i =0; i<8; i++)
  {
    Serial.print(sensorMin[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
  for(int i =0; i<8; i++)
  {
    Serial.print(sensorMax[i]);
    Serial.print(" ");
  }
  
  Serial.println();

}

void PID() {
  while (true) {
    if (pressButton() || isCmdBT(cmdSTOP)) {
      while(true)
      {

        #ifdef USE_IMP
          myESC.stop();
        #endif

        #ifdef SimplaDrive
          drive(0, 0);
        #endif

        #ifdef GyverMotor
          drive(0,0);
        #endif

        digitalWrite(LED, 1);
        delay(1000);
        digitalWrite(LED, 0);
        delay(1000);
      }      
    }
  unsigned int position = bot_position();

  int proportional = ((int)position - 3500);
  int derivative = proportional - last_proportional;
  integral += proportional;
  last_proportional = proportional;

  float power_difference = proportional * KP + derivative * KD;
  leftSpeed = max + power_difference;
  rightSpeed = max - power_difference;
  if (leftSpeed > 255)
    leftSpeed = 255;

  if (rightSpeed > 255)
    rightSpeed = 255;

/*#ifdef GyverMotor
  smoothControl(rightSpeed, leftSpeed);
#endif*/

#ifdef SimplaDrive
  //drive(leftSpeed, rightSpeed);
  drive(rightSpeed, leftSpeed);
#endif
  }
}


float bot_position()
{
  bool onLine = false;
  uint32_t avg = 0; // this is for the weighted total
  uint16_t sum = 0; // this is for the denominator, which is <= 64000



  sensorValues[0] = analogRead(A0);
  sensorValues[0] = map(sensorValues[0], sensorMin[0], sensorMax[0], 0, 1000);
  sensorValues[0] = constrain(sensorValues[0], 0, 1000);

  sensorValues[1] = analogRead(A1);
  sensorValues[1] = map(sensorValues[1], sensorMin[1], sensorMax[1], 0, 1000);
  sensorValues[1] = constrain(sensorValues[1], 0, 1000);

  sensorValues[2] = analogRead(A2);
  sensorValues[2] = map(sensorValues[2], sensorMin[2], sensorMax[2], 0, 1000);
  sensorValues[2] = constrain(sensorValues[2], 0, 1000);

  sensorValues[3] = analogRead(A3);
  sensorValues[3] = map(sensorValues[3], sensorMin[3], sensorMax[3], 0, 1000);
  sensorValues[3] = constrain(sensorValues[3], 0, 1000);

  sensorValues[4] = analogRead(A4);
  sensorValues[4] = map(sensorValues[4], sensorMin[4], sensorMax[4], 0, 1000);
  sensorValues[4] = constrain(sensorValues[4], 0, 1000);

  sensorValues[5] = analogRead(A5);
  sensorValues[5] = map(sensorValues[5], sensorMin[5], sensorMax[5], 0, 1000);
  sensorValues[5] = constrain(sensorValues[5], 0, 1000);

  sensorValues[6] = analogRead(A6);
  sensorValues[6] = map(sensorValues[6], sensorMin[6], sensorMax[6], 0, 1000);
  sensorValues[6] = constrain(sensorValues[6], 0, 1000);

  sensorValues[7] = analogRead(A7);
  sensorValues[7] = map(sensorValues[7], sensorMin[7], sensorMax[7], 0, 1000);
  sensorValues[7] = constrain(sensorValues[7], 0, 1000);

  //Serial.println();
  for (uint8_t i = 0; i < 8; i++)
  {
    uint16_t value = sensorValues[i];

    // keep track of whether we see the line at all
    if (value > FIRST_FILTRE) {
      onLine = true;
    }

    // only average in values that are above a noise threshold
    if (value > SECOND_FILTRE)
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  if (!onLine)
  {
    // If it last read to the left of center, return 0.
    if (_lastPosition < (8 - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (8 - 1) * 1000;
    }
  }

  _lastPosition = avg / sum;
  return _lastPosition;
}

//void smoothTick(max)

void drive(int leftS, int rightS) 
{

#ifdef SVAP_MOTORS
int left = rightS;
int right = leftS;
#else
int left = leftS;
int right = rightS;
#endif

  left = constrain(left, -255, 255);   // жестко ограничиваем диапозон значений
  right = constrain(right, -255, 255); // жестко ограничиваем диапозон значений


  if (left >= 0)
  {
    digitalWrite(BIN_1, LOW);
    digitalWrite(BIN_2, HIGH);
  }
  else
  {
    digitalWrite(BIN_1, HIGH);
    digitalWrite(BIN_2, LOW);

  }

  if (right >= 0)
  {
    digitalWrite(AIN_1, LOW);
    digitalWrite(AIN_2, HIGH);
  }
  else
  {
    digitalWrite(AIN_1, HIGH);
    digitalWrite(AIN_2, LOW);
  }


  analogWrite(PWMA, abs(right));
  analogWrite(PWMB, abs(left));
}

  static int lastSpeedR = 0; // начальное значение
  static int lastSpeedL = 0; // начальное значение

void smoothControl(int left, int right) 
{
  static uint32_t tmr = 0;
  if (millis() - tmr >= 1) 
  {  // каждые 5 мс
    tmr = millis();
    // если разница текущей и установленной больше шага изменения
    if (abs(lastSpeedR - right) > step)
    {  
      lastSpeedR += (lastSpeedR < right) ? step : -step;  // прибавлем или вычитаем
    } 
    else 
    {  // иначе
      lastSpeedR = right;  // новая скорость равна заданной
    }

    if (abs(lastSpeedL - left) > step) 
    {  
      lastSpeedL += (lastSpeedL < left) ? step : -step;  // прибавлем или вычитаем
    } 
    else 
    {  // иначе
      lastSpeedL = left;  // новая скорость равна заданной
    }
    drive(lastSpeedL, lastSpeedR);
  }
}
