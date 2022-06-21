 #include <ECE3.h>

// Encoder Variables
long enc_bin_cnt;
const unsigned long enc_bin_len = 50; // 50 ms bins
    // Encoder Speed Calculation Explanation:
    // We wait for a set amount of time (enc_bin_len), and find how many
    // times the encoder has incremented in that period. We call 
    // this period a bin when refering to the encoder. The number 
    // encoder counts per bin is a proportional to speed.

// Serial Communication
char new_duty_cycle[4];

uint16_t sensorValues[8];

const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;
const int ledRF = 41; 

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const short weights[4] = {8, 4, 2, 1};

int right_spd = 0;
int left_spd = 0;

float curError;
float prevError;

float kp = 0.03;
float kd = 0.2;

float second = 0;


int calcSlope(float prevError, float curError);
float calcError(float curError);
void changeSpeed(int& right_spd, int& left_spd);
//void doDonut();

const int minSpeed = 20;
const int maxSpeed = 100;

int donutCount = 0;
int sumReadings = 0;

float encoderCount = 0;
float donutEncoder = 0;



//int get_spd(int prev_spd);

void setup()
{

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // Setting Initial Values
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(ledRF, OUTPUT);
  digitalWrite(ledRF, LOW);


  
  curError = 0;
  prevError = 0;
  second = 0;
  
  donutCount = 0;
  
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);

}


void loop()
{

  
  // Set the new speed
  if (donutCount <= 1)
  {
    float leftencode = getEncoderCount_left();
    float rightencode = getEncoderCount_right();
    if ( ((leftencode + rightencode)/2 >= 360 * 4.5 && (leftencode + rightencode)/2 <= 360 * 10.41)
      || ((leftencode + rightencode)/2 >= 360 * 25.50 && (leftencode + rightencode)/2 <= 360 * 31.4)
      || ((leftencode + rightencode)/2 >= 360 * 35.0 && (leftencode + rightencode)/2 <= 360 * 36.2)
      )
    {
      if ( ((leftencode + rightencode) / 2 >= 360 * 4.5 && (leftencode + rightencode) / 2 <= 360 * 5.5)
      || ( ((leftencode + rightencode) / 2 >= 360 * 23.5 && (leftencode + rightencode) / 2 <= 360 * 25.0))
      )
      {
        left_spd = 235;
        right_spd = 235;
        kd = 0.8;
        kp = 0.03;
      }
      else
      {
        left_spd = 250;
        right_spd = 250;
        kd = 0.8;
        kp = 0.03;
      }
      digitalWrite(ledRF, HIGH);
    

      if (((leftencode + rightencode)/2 >= 360 * 35.0 && (leftencode + rightencode)/2 <= 360 * 36.2))
       {
          left_spd = 160;
          right_spd = 160;
          kd = 0.8;
          kp = 0.03;
       }
       else if (((leftencode + rightencode)/2 >= 360 * 36.2 && (leftencode + rightencode)/2 <= 360 * 36.5))
       {
          kd = 0.5;
          left_spd = 60;
          right_spd = 60;
       }
    }
    else if (((leftencode + rightencode)/2 >= 360 * 0.50 && (leftencode + rightencode)/2 <= 360 * 1.4))
    {
      //initial speedup
      left_spd = 160;
      right_spd = 160;
      kd = 0.2;
      kp = 0.03;
    }
    else if ( 
      ((leftencode + rightencode)/2 > 360 * 21.2 && (leftencode + rightencode)/2 <= 360 * 25.5)
      || ((leftencode + rightencode)/2 > 360 * 10.41 && (leftencode + rightencode)/2 <= 360 * 15.5)
      )
    {

      if ( ((leftencode + rightencode) / 2 >= 360 * 14.0 && (leftencode + rightencode) / 2 <= 360 * 15.5)
      || ( ((leftencode + rightencode) / 2 >= 360 * 21.2 && (leftencode + rightencode) / 2 <= 360 * 23.0)
      || ( (donutCount > 1) && (leftencode + rightencode) / 2 >= donutEncoder && (leftencode + rightencode ) / 2 <= donutEncoder + 0.45 * 360))
      )
      {
        kd = 0.3;
        left_spd = 160;
        right_spd = 160;
        kp = 0.03;
      }
      else if ( ((leftencode + rightencode) / 2 >= 360 * 24.5 && (leftencode + rightencode) / 2 <= 360 * 25.5) )
      {
         kd = 0.45;
        left_spd = 160;
        right_spd = 160;
        kp = 0.03;
      }
      else if ( (donutCount > 1) && (leftencode + rightencode) / 2 >= donutEncoder && (leftencode + rightencode ) / 2 <= donutEncoder + 0.45 * 360)
      {
        kd = 0.45;
         left_spd = 160;
        right_spd = 160;
      }
      else
      {
        kd = 0.45;
        left_spd = 180;
        right_spd = 180;
        kp = 0.03;
      }
      
      /*
      if ((leftencode + rightencode)/2 >= 360 * 19.5 && (leftencode + rightencode)/2 <= 360 * 20.0)
      {
        left_spd = 80;
        right_spd = 80;
      }
      */ 
    }
    else 
    {
      digitalWrite(ledRF, LOW);
      left_spd = 60;
      right_spd = 60;
      kd = 0.25;
      kp = 0.04;
    }
    
  ECE3_read_IR(sensorValues);
  sumReadings = 0;
  for (int i = 0; i < 7; i++)
    sumReadings += sensorValues[i];

   if (sumReadings / 8 >= 1900)
   {
      ECE3_read_IR(sensorValues);
      sumReadings = 0;
      for (int i = 0; i < 7; i++)
        sumReadings += sensorValues[i];
        
      if (sumReadings / 8 >= 1900)
        {
          donutCount++;
          if (donutCount > 1)
          {
            left_spd = 0;
            right_spd = 0;
          }
          else
          {
            float initialleft = getEncoderCount_left();

            while(1)
            {
              float curLeft = getEncoderCount_left();
              if (curLeft - initialleft > 360)
                break;

              left_spd = 180;
              right_spd = 180;
              digitalWrite(right_dir_pin,HIGH);
              analogWrite(left_pwm_pin, left_spd);
              analogWrite(right_pwm_pin,right_spd);
            }

            donutEncoder = (getEncoderCount_left() + getEncoderCount_right()) / 2;
          }
        }
        left_spd = 60;
        right_spd = 60;
        digitalWrite(right_dir_pin,LOW);
   }
    
  
  curError = calcError(curError);
  
  float slope = curError - prevError; //50 - 200
  float adjust = kd * slope;

 
  prevError = curError;
 
 
  left_spd += (kp * curError * -1 - adjust);
  right_spd += (kp * curError + adjust);
  //left_spd *= (1 - adjust / 10);
  //right_spd *= (1 - adjust / 10);

   
  
  if (left_spd <= 255)
    analogWrite(left_pwm_pin, left_spd);
  else analogWrite(left_pwm_pin, 255);

  if (right_spd <= 255)
    analogWrite(right_pwm_pin,right_spd);
   else analogWrite(right_pwm_pin, 255);

  
  // read raw sensor values
  /*
  ECE3_read_IR(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  */
  }

  else
  {
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin,0);
  }
  

  
}

int calcSlope(float prevError, float curError)
{
  return ( (curError - prevError) );
}

float calcError(float curError)
{
  float sensorValues0 = ((sensorValues[0] - 549) * 1000.0) / 1636.0;
  float sensorValues1 = ((sensorValues[1] - 458) * 1000.0) / 1249.0;
  float sensorValues2 = ((sensorValues[2] - 458) * 1000.0) / 1377.9;
  float sensorValues3 = ((sensorValues[3] - 432) * 1000.0) / 919.3;
  float sensorValues4 = ((sensorValues[4] - 549) * 1000.0) / 1158.0;
  float sensorValues5 = ((sensorValues[5] - 572) * 1000.0) / 1348.0;
  float sensorValues6 = ((sensorValues[6] - 549) * 1000.0) / 1253.0;
  float sensorValues7 = ((sensorValues[7] - 549) * 1000.0) / 1779.0;
  
  
  float errorVal = 0;
  errorVal += weights[0] * (sensorValues7 - sensorValues0);
  errorVal += weights[1] * (sensorValues6 - sensorValues1);
  errorVal += weights[2] * (sensorValues5 - sensorValues2);
  errorVal += weights[3] * (sensorValues4 - sensorValues3);

  curError = errorVal / 4;
  curError -= 126.47;
 //Serial.print(curError);
  //Serial.print("\n");

  return curError;
}

void changeSpeed(int& right_spd, int& left_spd)
{
      left_spd += (kp * curError * -1);
      right_spd += (kp * curError);
}


int get_spd(int prev_spd) {

  int bytes_avail = Serial.available();
  if(bytes_avail == 0) // no new serial inputs
    return prev_spd;
    
  else if (bytes_avail > 4) { // invalid serial input
    
    do { // eat invalid buffered input
      delay(100);
      Serial.read();
    } while(Serial.available() > 0);  
    Serial.println("INVALID"); 
    //Serial.println(bytes_avail);  
    return prev_spd;
  }
  
  else {
    for (int i = 0; i < bytes_avail; i++) // read out buffer
      new_duty_cycle[i] = Serial.read();
    int sum = 0;
    for (int i = 0; i < bytes_avail; i++) { // calculate new speed
      
      int num = new_duty_cycle[i] - '0';
      if (num == -38)
        break;
      else if (num > 9 || num < 0) { // invalid character
        Serial.println("INVALID"); 
        return prev_spd;
      }
      sum += pow(10, bytes_avail - 2 - i) * num;
    }

    if (sum >= 255)
      return 255;
    else
      return sum;
  }
}
