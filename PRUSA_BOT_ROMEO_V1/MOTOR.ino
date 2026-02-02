#define MOTOR_PWM_RANGE        ((1 << MOTOR_PWM_RESOLUTION) - 1)

// *****************************************************
// MOTOR CONTROL MODE
#ifdef  USE_LEDC
#define MOTOR_PWM_RESOLUTION  10
#else
#define MOTOR_PWM_RESOLUTION  8
#endif

bool  motA_HiFreq = false;
bool  motB_HiFreq = false;


void setupMotorPins()
{
  pinMode(MOTA_DIR, OUTPUT);
  pinMode(MOTB_DIR, OUTPUT);
  digitalWrite(MOTA_DIR,0);
  digitalWrite(MOTB_DIR,0);
  #ifdef USE_LEDC
  ledcAttach(MOTA_PWM, MOTOR_PWM_LO_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcAttach(MOTB_PWM, MOTOR_PWM_LO_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcWrite(MOTA_PWM,0);
  ledcWrite(MOTB_PWM,0);
  #else
  analogWriteFrequency(MOTA_PWM,MOTOR_PWM_LO_FREQUENCY);
  analogWriteFrequency(MOTB_PWM,MOTOR_PWM_LO_FREQUENCY);
  analogWriteResolution(MOTA_PWM,MOTOR_PWM_RESOLUTION);
  analogWriteResolution(MOTB_PWM,MOTOR_PWM_RESOLUTION);
  analogWrite(MOTA_PWM,0);
  analogWrite(MOTB_PWM,0);
  #endif
}

void updateMotorAPWM(float duty)
{

  float aduty = fabs(duty);
  float a = interpolate(aduty,0.0f,1.0f,0,MOTOR_PWM_RANGE);

  if( motorsEnabled == false ) a = 0;

  #ifdef PWM_FREQUENCY_SWITCHING
  configurePWMFrequency(MOTA_PWM,aduty,&motA_HiFreq);
  #endif

  if(duty < 0 ) 
  {
    if( !digitalRead(MOTA_DIR) ) 
    {
      #ifdef USE_LEDC
      ledcWrite(MOTA_PWM,0);
      #else
      analogWrite(MOTA_PWM,0);
      #endif
      digitalWrite(MOTA_DIR,1);
    }
    #ifdef USE_LEDC
    ledcWrite(MOTA_PWM,a);
    #else
    analogWrite(MOTA_PWM,a);
    #endif
  }
else 
  {
    if( digitalRead(MOTA_DIR) ) 
    {
      #ifdef USE_LEDC
      ledcWrite(MOTA_PWM,0);
      #else
      analogWrite(MOTA_PWM,0);
      #endif
      digitalWrite(MOTA_DIR,0);
    }
    #ifdef USE_LEDC
    ledcWrite(MOTA_PWM,a);
    #else
    analogWrite(MOTA_PWM,a);
    #endif
  }
}

void updateMotorBPWM(float duty)
{
  float aduty = fabs(duty);
  float a = interpolate(aduty,0.0f,1.0f,0,MOTOR_PWM_RANGE);

  if( motorsEnabled == false ) a = 0;

  #ifdef PWM_FREQUENCY_SWITCHING
  configurePWMFrequency(MOTB_PWM,aduty,&motB_HiFreq);
  #endif

  if(duty < 0 ) 
  {
    if( !digitalRead(MOTB_DIR) ) 
    {
      #ifdef USE_LEDC
      ledcWrite(MOTB_PWM,0);
      #else
      analogWrite(MOTB_PWM,0);
      #endif
      digitalWrite(MOTB_DIR,1);
    }
    #ifdef USE_LEDC
    ledcWrite(MOTB_PWM,a);
    #else
    analogWrite(MOTB_PWM,a);
    #endif
  }
else 
  {
    if( digitalRead(MOTB_DIR) ) 
    {
      #ifdef USE_LEDC
      ledcWrite(MOTB_PWM,0);
      #else
      analogWrite(MOTB_PWM,0);
      #endif
      digitalWrite(MOTB_DIR,0);
    }
    #ifdef USE_LEDC
    ledcWrite(MOTB_PWM,a);
    #else
    analogWrite(MOTB_PWM,a);
    #endif
  }
}


void configurePWMFrequency(int motPin, float aduty, bool *mode)
{
if( aduty > PWM_HI_FREQ_DUTY && *mode == false ) 
  {
    #ifdef USE_LEDC
    ledcChangeFrequency(motPin,MOTOR_PWM_HI_FREQUENCY,MOTOR_PWM_RESOLUTION);
    #else
    analogWriteFrequency(motPin,MOTOR_PWM_HI_FREQUENCY);
    #endif
    *mode = true;
  }
else if ( aduty < PWM_LO_FREQ_DUTY && *mode == true )
  {
    #ifdef USE_LEDC
    ledcChangeFrequency(motPin,MOTOR_PWM_LO_FREQUENCY,MOTOR_PWM_RESOLUTION);
    #else
    analogWriteFrequency(motPin,MOTOR_PWM_LO_FREQUENCY);
    #endif
    *mode = false;
  }
}
