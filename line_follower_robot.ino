#include <EEPROM.h>
#include <QTRSensors.h>
//#include <SoftwareSerial.h>// import the serial library

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {7, 6, 5, 4, 3, 35, 33, 31},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];


//const int rSpeed = 6;
//const int lSpeed = 5 ;
const int motor_a0 = 8;
const int motor_b0 = 9;
const int motor_a1 = 11;
const int motor_b1 = 10;

double correction;


int baseSpeed = 100;
int maxSpeed = 200;
int minSpeed = -240;

int lastError = 0;
int error = 0;
int p, intg, d;

double kp = 0.032; //80// 0.04
double ki = 0; //0.5//0
double kd = 0.032; //70//0.08

bool temp1;


unsigned int position;

String readText;
int BluetoothData;


bool bSV[NUM_SENSORS]; //sensor values in binary
int crit = 500;

int check30;

//int x= 0;
int count = 0;

void setup()
{
//  setPwmFrequency(5,1024);
 Serial.begin(9600);
  Serial3.begin(9600);
  
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  analogWrite(motor_a0, 120);
  analogWrite(motor_b0, LOW);
  analogWrite(motor_a1, LOW);
  analogWrite(motor_b1, 120);

  // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  stp();
  
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }


  Serial.println();
  Serial.println();
  delay(1000);

  
}


void loop()
{
//  bool aaa = false;
  if(Serial3.available()) BT();
  readIR();

  //first counter
  if( count == 0 && sensorCheck("xxxxxxx1"))
  {
    movRig();
    count++;
  }

  //second counter
  if(count== 1 && sensorCheck("xxxxxxx1"))
  {
     while ( sensorCheck("0xx||xx0") ) 
    {
      goForward();
      readIR();
    }
     count++;
  }


  if(count== 2&& sensorCheck("1xxxxxxx"))
  {
     movLef();
     count++;
  }

  if(count == 3 && sensorCheck("11111111"))
  {
        while ( !sensorCheck("xx||||xx") && sumOfArrays() <3 ) 
    {
      goForward();
      readIR();
    }
    count++;
  }



  if(count == 4 && sensorCheck("11111111"))
  {
        while ( !sensorCheck("xx||||xx") && sumOfArrays() <3 ) 
    {
      goForward();
      readIR();
    }
    count++;
  }

  if(sensorCheck("11x00x11"))
  {
    invertSensor();
    
  }

  if(count == 5 && sensorCheck("xxxxxxx1"))
  {
        while ( !sensorCheck("|||xxxx0") ) 
    {
      goForward();
      readIR();
    }
    
  }


// add two circle logic with counters here!!!!!!!!!!!!!!!!!
  

  if(count == 5 && sensorCheck("xxxxxxx1"))
  {
     movRig();
     count++;
  }

   if(count == 6 && sensorCheck("xxxxxxx1"))
  {
     movRig();
     count++;
  }

//if (sensorCheck("11111111")) goForward();
//if (sensorCheck("00xx1111")) right_prior();
//else if (sensorCheck("1111xx00")) left_prior();

pidMove();
 
}// end of loop

void invertSensor()
{

  ///////////////////////////////////////////////////
}


void readIR()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);

   position = qtrrc.readLine(sensorValues);
 
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if ( sensorValues[i]>crit ) bSV[i] = 1;
      else bSV[i] = 0;
    
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values
  
  //delay(250);


}

void movRig()
{

    while ( !sensorCheck("0xx||xx0") ) /////////////////////////////////////////////////////////////////////////////////////////////////
    {
      goForward();
      readIR();
    }
    while ( !sensorCheck("00000000") ) 
    {
      goRight();
      readIR();
    }
    while (sumOfArrays() <3  ) 
    {
      goRight();
      readIR();
    }
  
}


void movLef()
{
   while ( !sensorCheck("0xx||xx0") ) 
    {
      goForward();
      readIR();
    }
    while ( !sensorCheck("0xx||xx0") || sumOfArrays() < 3) 
    {
      goLeft();
      readIR();
    }
    
}

void goForward()
{
  analogWrite(motor_a0, baseSpeed);
  analogWrite(motor_b0, LOW);
  analogWrite(motor_a1, baseSpeed);
  analogWrite(motor_b1, LOW);

}

void goBackward()
{
  digitalWrite(motor_a0, LOW);
  digitalWrite(motor_b0, HIGH);
  digitalWrite(motor_a1, LOW);
  digitalWrite(motor_b1, HIGH);
}

void goRight()
{

  digitalWrite(motor_a0, baseSpeed);
  digitalWrite(motor_b0, LOW);
  digitalWrite(motor_a1, LOW);
  digitalWrite(motor_b1, baseSpeed);


}

void goLeft()
{
  digitalWrite(motor_a0, LOW);
  digitalWrite(motor_b0, baseSpeed);
  digitalWrite(motor_a1, baseSpeed);
  digitalWrite(motor_b1, LOW);


}

void stp()
{

//  analogWrite(rSpeed, 0);
//  analogWrite(lSpeed, 0);
  digitalWrite(motor_a0, LOW);
  digitalWrite(motor_b0, LOW);
  digitalWrite(motor_a1, LOW);
  digitalWrite(motor_b1, LOW);


}

boolean noLineIsThere()
{
  temp1 = true;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] == 0)
    {
      temp1 = temp1 && true;
    }

    else if  (sensorValues[i] == 1) temp1 = temp1 && false ;

  }



  return temp1;
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}



void BT()
{
  while (Serial3.available())
  {
    delay(3);
    char c =  Serial3.read();
    readText += c;
    
  }

  if (readText.length() > 0)
  {
    //Serial.println(readText);
    if (readText[0] == 'P') kp = converter(readText);
    else if (readText[0] == 'I') ki = converter(readText);
    else if (readText[0] == 'D') kd = converter(readText);
    else if (readText[0] == 'B') baseSpeed = converter(readText);
    else if (readText[0] == 'H') maxSpeed = converter(readText);
    else if (readText[0] == 'L') minSpeed = -converter(readText);
    else if (readText[0] == 'S'){

      

    }
  } 
    readText=""; 
  
  



}


float converter (String x)
{
  int l = x.length();
  char z[l];
  for ( int i=0; i<l; i++)
    z[i] = x[i+1];
  return atof(z);
}


int sensorCheck(String val)
{
    //int bSV[]={0,0,0,1,1,1,0,1};
    //char* val = "0xx||oo1";
    
    int rarray[8];
    int returnVal = 1;
    
    for (int i=7; i>=0; i--)
    {
        if ( val[i]=='x' || ( bSV[i]==1 && val[i]=='1' ) || (bSV[i]==0 && val[i]=='0') ) rarray[i]=1;
        else if ( val[i]=='|' && bSV[i]==1 )  rarray[i]=2;
        else if ( val[i]=='|' && bSV[i]==0 ) rarray[i]=3;
        else if ( val[i]=='o' && bSV[i]==0 ) rarray[i]=8;
        else if ( val[i]=='o' && bSV[i]==1 ) rarray[i]=9;
        else rarray[i]=0;
        //printf("%d", rarray[i]);
    }
    
    //printf("\n");
    
    for (int i=0; i<8; i++)
    {
        
        int chkr = 0;
        if (rarray[i]==2 || rarray[i]==3) 
        {   
            int j = i;
            int k = i;
            
            while(j<8 && (rarray[j]==2 || rarray[j]==3))
            {
                if ( rarray[j]==2 ) chkr = 1;
                j++;
            }
            while(k<8 && (rarray[k]==2 || rarray[k]==3))
            {
                if ( chkr == 1) rarray[k]=1;
                else rarray[k]=0;
                k++;
            }
            
        }
        
        if (rarray[i]==8 || rarray[i]==9) 
        {   
            int j = i;
            int k = i;
            
            while(j<8 && (rarray[j]==8 || rarray[j]==9))
            {
                if ( rarray[j]==8 ) chkr = 1;
                j++;
            }
            while(k<8 && (rarray[k]==8 || rarray[k]==9))
            {
                if ( chkr == 1) rarray[k]=1;
                else rarray[k]=0;
                k++;
            }
            
        }
        
        //printf("%d", rarray[i]);
        returnVal*=rarray[i];
    }
    //printf("\n\n\n%d\n\n",returnVal);
    
    return returnVal; //0;
}   

int sumOfArrays()
{
  int sum;
  for (int i=7; i>=0; i--)
  {
    sum=+bSV[i];
  }
  return sum;
}

void pidMove()
{
  
  error=position-3500;

  p = error;
  intg += error;
  d = error - lastError;
  lastError = error;

  if (intg > 255) {
    intg = 255;
  };
  if (intg < -255) {
    intg = -255;
  };


  
  correction = (kp * p) + (ki * intg) + (kd * d);
  
  int  rightMotorSpeed = baseSpeed + correction;
  int leftMotorSpeed = baseSpeed - correction;

  if (rightMotorSpeed < minSpeed) rightMotorSpeed = minSpeed;
  else if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;

  if (leftMotorSpeed < minSpeed) leftMotorSpeed = minSpeed;
  else if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

  if ( leftMotorSpeed > 0 && rightMotorSpeed > 0)
  {
      analogWrite(motor_a0, leftMotorSpeed);
      analogWrite(motor_b0, LOW);
      analogWrite(motor_a1, rightMotorSpeed);
      analogWrite(motor_b1, LOW);
  }
  else if ( leftMotorSpeed < 0 && rightMotorSpeed > 0)
  {

      analogWrite(motor_b0, leftMotorSpeed);
      analogWrite(motor_a0, LOW);
      analogWrite(motor_b1, LOW);
      analogWrite(motor_a1, rightMotorSpeed);
  }

  else if ( leftMotorSpeed > 0 && rightMotorSpeed < 0)
  {
      analogWrite(motor_b0, LOW);
      analogWrite(motor_a0, leftMotorSpeed);
      analogWrite(motor_b1, rightMotorSpeed);
      analogWrite(motor_a1, LOW);
      
  }
  