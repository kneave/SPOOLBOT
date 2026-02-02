#define PWM_INVERT 0
#define PWM_FOF 0.1f    // PWM First Order Filter
#define PWM_DEADBAND  0.015f // PWM deadband to return to zero 
#define PWM_CURVE   0.5f  // sigmoid curve, positive = slower near neutral, 0 = linear
                          // negative = hotter near neutral, 1.0f == LINEAR
#define PWM_PITCH_ACCEL   0.02f // % /s /s
#define PWM_PITCH_DECEL   0.08f // % /s /s
#define PWM_YAW_ACCEL   0.06f // % /s /s
#define PWM_YAW_DECEL   0.06f // % /s /s



// PWM CAPTURE
// ====== Captured data ======
volatile uint16_t pitch_riseTime_us = 0;       // 
volatile uint16_t pitch_pulseWidth_us = 1500;  // latest pulse width in microseconds
volatile bool     pitch_newPulse = false;
volatile uint16_t yaw_riseTime_us = 0;       // 
volatile uint16_t yaw_pulseWidth_us = 1500;  // latest pulse width in microseconds
volatile bool     yaw_newPulse = false;
volatile uint16_t aux_riseTime_us = 0;       // 
volatile uint16_t aux_pulseWidth_us = 1500;  // latest pulse width in microseconds
volatile bool     aux_newPulse = false;

bool              pitch_NEW = false;
bool              yaw_NEW = false;
bool              aux_NEW = false;

void  updatePWMControls()
{
  noInterrupts();
  pitch_NEW = pitch_newPulse; 
  pitch_newPulse = false; 
  pitch_PWM = pitch_pulseWidth_us;
  yaw_NEW = yaw_newPulse; 
  yaw_newPulse = false;
  yaw_PWM = yaw_pulseWidth_us;
  aux_NEW = aux_newPulse; 
  aux_newPulse = false; 
  aux_PWM = aux_pulseWidth_us;
  interrupts();
}

void pwm_pitch_isr() {

// Read pin level quickly
  bool level = digitalRead(PWM_PITCH);
  if (PWM_INVERT) level = !level;

  uint16_t now = micros();

  if (level) {
    // Rising edge: store time
    pitch_riseTime_us = now;
  } else {
    // Falling edge: compute pulse width (wrap-safe for uint16)
    uint16_t width = (uint16_t)(now - pitch_riseTime_us);

    // Optional sanity clamp for RC PWM (helps ignore glitches)
    if (width >= 800 && width <= 2200) {
      pitch_pulseWidth_us = width;
      pitch_newPulse = true;
    }
  }
}

void pwm_yaw_isr() {

// Read pin level quickly
  bool level = digitalRead(PWM_YAW);
  if (PWM_INVERT) level = !level;

  uint16_t now = micros();

  if (level) {
    // Rising edge: store time
    yaw_riseTime_us = now;
  } else {
    // Falling edge: compute pulse width (wrap-safe for uint16)
    uint16_t width = (uint16_t)(now - yaw_riseTime_us);

    // Optional sanity clamp for RC PWM (helps ignore glitches)
    if (width >= 800 && width <= 2200) {
      yaw_pulseWidth_us = width;
      yaw_newPulse = true;
    }
  }
}

void pwm_aux_isr() {

// Read pin level quickly
  bool level = digitalRead(PWM_AUX);
  if (PWM_INVERT) level = !level;

  uint16_t now = micros();

  if (level) {
    // Rising edge: store time
    aux_riseTime_us = now;
  } else {
    // Falling edge: compute pulse width (wrap-safe for uint16)
    uint16_t width = (uint16_t)(now - aux_riseTime_us);

    // Optional sanity clamp for RC PWM (helps ignore glitches)
    if (width >= 800 && width <= 2200) {
      aux_pulseWidth_us = width;
      aux_newPulse = true;
    }
  }
}

// signal conditioning for user PWM controls

void filterPWM()
{
  
  // normailse PWM with 3 point calibration
  float pitch_control;
  if( pitch_PWM >= calData.pitch.mid_raw ) pitch_control = interpolate((float)pitch_PWM,(float)calData.pitch.mid_raw,(float)calData.pitch.max_raw,0.0f,1.0f);
  else pitch_control = interpolate((float)pitch_PWM,(float)calData.pitch.min_raw,(float)calData.pitch.mid_raw,-1.0f,0.0f);
  // FOF pwm
  pitch_controlFiltered = firstOrderLPF(pitch_control,pitch_controlFiltered,loopDelta,PWM_FOF);

  #ifdef PWM_DEADBAND
  // clamp
  if( fabs(pitch_controlFiltered) < PWM_DEADBAND ) pitch_controlFiltered = 0;
  // re interpolate including deadband
  if( pitch_controlFiltered >= PWM_DEADBAND )  
    pitch_controlFiltered = interpolate(pitch_controlFiltered,PWM_DEADBAND,1.0f,0.0f,1.0f);
  else if( pitch_controlFiltered <= -PWM_DEADBAND )  
    pitch_controlFiltered = interpolate(pitch_controlFiltered,-1.0,-PWM_DEADBAND,-1.0f,0.0f);
  #endif

  // add curve to user controls
  float pitch_controlSigmoid = sigmoidCurve(pitch_controlFiltered,PWM_CURVE);
  // limit acceleration/deceleration
  pitch_controlFinal = accelerationFilter(pitch_controlSigmoid,pitch_controlFinal,PWM_PITCH_ACCEL,PWM_PITCH_DECEL);

  // normailse PWM with 3 point calibration
  float yaw_control;
  if( yaw_PWM >= calData.yaw.mid_raw ) yaw_control = interpolate((float)yaw_PWM,(float)calData.yaw.mid_raw,(float)calData.yaw.max_raw,0.0f,1.0f);
  else yaw_control = interpolate((float)yaw_PWM,(float)calData.yaw.min_raw,(float)calData.yaw.mid_raw,-1.0f,0.0f);
  // FOF pwm
  yaw_controlFiltered = firstOrderLPF(yaw_control,yaw_controlFiltered,loopDelta,PWM_FOF);
  // clamp
  #ifdef PWM_DEADBAND
  if( fabs(yaw_controlFiltered) < PWM_DEADBAND ) yaw_controlFiltered = 0;
  // re interpolate including deadband
  if( yaw_controlFiltered >= PWM_DEADBAND )  
    yaw_controlFiltered = interpolate(yaw_controlFiltered,PWM_DEADBAND,1.0f,0.0f,1.0f);
  else if( yaw_controlFiltered <= -PWM_DEADBAND )  
    yaw_controlFiltered = interpolate(yaw_controlFiltered,-1.0,-PWM_DEADBAND,-1.0f,0.0f);
  #endif

  // add curve to user controls
  float yaw_controlSigmoid = sigmoidCurve(yaw_controlFiltered,PWM_CURVE);
  // limit acceleration/deceleration
  yaw_controlFinal = accelerationFilter(yaw_controlSigmoid,yaw_controlFinal,PWM_YAW_ACCEL,PWM_YAW_DECEL);


  // normailse PWM
  float newPWM = interpolate((float)aux_PWM,PWM_MIN,PWM_MAX,-1.0f,1.0f);
  // FOF pwm
  aux_controlFiltered = firstOrderLPF(newPWM,aux_controlFiltered,loopDelta,PWM_FOF);
  // clamp
  #ifdef PWM_DEADBAND
  if( fabs(aux_controlFiltered) < PWM_DEADBAND ) aux_controlFiltered = 0;
  #endif


}

