// RPM First Order Filter 
#define RPM_Fof (0.01f)
#define RPM_Fof_Snap_Zero (3.0f) // smaller than this rpm = 0

typedef struct
{
  float alpha = 0.5f;   // position correction (0..1)
  float beta  = 0.0625f;   // velocity correction (0..1) beta ≈ alpha² / 4 
  float residualDeadband = 0.0f; // residual deadband (quantisation / noise)
  uint8_t stillCountThreshold = 5; // 100ms @ 20ms frame rate
  uint8_t  stillCount = 0;
  // State
  float CDF_x = 0.0f;        // estimated position (counts)
  float CDF_v = 0.0f;        // estimated velocity (counts/sec)
  uint32_t  loopLastUs = 0;
  float     lastValue = 0;
}CDF_s;

int criticalDampedFilter(CDF_s *p, float newValue);
int CDF_Filter(CDF_s *p, float newValue);

CDF_s RPMA_Cdf;
CDF_s RPMB_Cdf;
CDF_s speed_Cdf;

void initSpeedCDF()
{
  speed_Cdf.CDF_x = 0;
  speed_Cdf.CDF_v = 0;
  speed_Cdf.alpha = 0.2f;   // position correction (0..1) 1 = copy of source, lower value more smooth
  speed_Cdf.beta  = (speed_Cdf.alpha*speed_Cdf.alpha)/4.0f; // velocity correction
  speed_Cdf.residualDeadband = 0.0f;
  speed_Cdf.stillCountThreshold = 5;
  speed_Cdf.stillCount = 0;
}

void initRPMCDF()
{
  RPMA_Cdf.CDF_x = EncA_encLastCount;
  RPMB_Cdf.CDF_x = EncB_encLastCount;

  RPMA_Cdf.CDF_v = 0;
  RPMA_Cdf.alpha = 0.6f;   // position correction (0..1)
  RPMA_Cdf.beta  = 0.12f; // velocity correction (0..1) beta ≈ alpha² / 4 
  RPMA_Cdf.residualDeadband = 0.5f;
  RPMA_Cdf.stillCountThreshold = 5;
  RPMA_Cdf.stillCount = 0;

  RPMB_Cdf.CDF_v = 0;
  RPMB_Cdf.alpha = 0.6f;   // position correction (0..1)
  RPMB_Cdf.beta  = 0.12f; // velocity correction (0..1) beta ≈ alpha² / 4 
  RPMB_Cdf.residualDeadband = 0.5f;
  RPMB_Cdf.stillCountThreshold = 5;
  RPMB_Cdf.stillCount = 0;

}

void RPMCdf()
{
  // predict new value
  if( !criticalDampedFilter(&RPMA_Cdf,(float)EncA_newCount) ) {
    // convert to RPM
    EncA_SpeedFiltered = (RPMA_Cdf.CDF_v / ENC_CPR) * 60.0f; 
  }
  // predict new value
  if( !criticalDampedFilter(&RPMB_Cdf,(float)EncB_newCount) ) {
    // convert to RPM
    EncB_SpeedFiltered = (RPMB_Cdf.CDF_v / ENC_CPR) * 60.0f;
  }

  MotA_OutputSpeed = EncA_SpeedFiltered / MOTOR_GEAR_RATIO;
  MotB_OutputSpeed = EncB_SpeedFiltered / MOTOR_GEAR_RATIO;
  WheelA_OutputSpeed = MotA_OutputSpeed / PRINTED_GEAR_RATIO;
  WheelB_OutputSpeed = MotB_OutputSpeed / PRINTED_GEAR_RATIO;
}

float speedCdf()
{
  if( !criticalDampedFilter(&speed_Cdf,measuredSpeed) ) {
    return speed_Cdf.CDF_x;
  }
}

int criticalDampedFilter(CDF_s *p, float newValue)
{
  if( p == NULL ) return -1;

  if( !CDF_Filter(p,newValue) ) 
  {
    
    // standstill lock (no counts for 100ms -> v=0)
    if ((newValue - p->lastValue) == 0) 
    {
      if (++p->stillCount >= p->stillCountThreshold) p->CDF_v = 0.0f;
    } 
    else 
    {
      p->stillCount = 0;
    }

    p->lastValue = newValue;

    return 0;
  }

  return -2;
}

int CDF_Filter(CDF_s *p, float newValue)
{
  if( p == NULL ) return -1;

  uint32_t usNow = micros();
  float loopDelta = (usNow - p->loopLastUs) * 1e-6f;
  p->loopLastUs = usNow;

  // predict
  float x_pred = p->CDF_x + p->CDF_v * loopDelta;

  // residual
  float r = newValue - x_pred;

  // residual deadband (quantisation / noise)
  // Gate the velocity correction when residual is tiny
  if (fabsf(r) < p->residualDeadband) r = 0.0f;

  // update (alpha-beta)
  p->CDF_x = x_pred + p->alpha * r;

  // only push velocity if residual is meaningful
  if (r != 0.0f) {
    p->CDF_v = p->CDF_v + (p->beta / loopDelta) * r;
  }

  return 0;
}

void FOF_EncoderFilter()
{

  // raw RPM
  EncA_Speed = ( (float)EncA_deltaCounts * 60.0f ) / (ENC_CPR * loopDelta);// ((vA - EncA_encLastCount) / ENC_CPR) * (60.0 / (displayPeriod/1000.0));
  EncB_Speed = ( (float)EncB_deltaCounts * 60.0f ) / (ENC_CPR * loopDelta);

  // filtered RPM
  EncA_SpeedFiltered = firstOrderLPF(EncA_Speed,EncA_SpeedFiltered,loopDelta,RPM_Fof);//firstOrderFilter_f(EncA_Speed,EncA_SpeedFiltered,ENCODER_SPEED_FOF);
  EncB_SpeedFiltered = firstOrderLPF(EncB_Speed,EncB_SpeedFiltered,loopDelta,RPM_Fof);

  // zero-speed snap
  if (abs(EncA_deltaCounts) == 0 && fabs(EncA_SpeedFiltered) < RPM_Fof_Snap_Zero) EncA_SpeedFiltered = 0;
  if (abs(EncB_deltaCounts) == 0 && fabs(EncB_SpeedFiltered) < RPM_Fof_Snap_Zero) EncB_SpeedFiltered = 0;

  MotA_OutputSpeed = EncA_SpeedFiltered / MOTOR_GEAR_RATIO;
  MotB_OutputSpeed = EncB_SpeedFiltered / MOTOR_GEAR_RATIO;
  WheelA_OutputSpeed = MotA_OutputSpeed / PRINTED_GEAR_RATIO;
  WheelB_OutputSpeed = MotB_OutputSpeed / PRINTED_GEAR_RATIO;
}

float firstOrderLPF( float xNew, float yOld, float dt, float tauSec ) 
{
    if (tauSec <= 0.0f || dt == 0) return xNew;
    float alpha = dt / (tauSec + dt);
    return yOld + alpha * (xNew - yOld);
}

float  firstOrderFilter_f( float newX, float oldX, float filter )
{
  if( filter <= 0 ) return newX;

  return ((newX + (filter*oldX)) / (filter+1.0f));
}


float accelerationFilter(float newVal, float lastVal, float accel, float decel)
{
  // accelerating
  if( fabs(newVal) > fabs(lastVal) )
  {
    if( newVal > (lastVal + accel) ) newVal = lastVal + accel;
    else if( newVal < (lastVal - accel) ) newVal = lastVal - accel;
  }
  // decelerating
  else if( fabs(newVal) < fabs(lastVal) )
  {
    if( newVal < (lastVal - decel) ) newVal = lastVal - decel;
    else if( newVal > (lastVal + decel) ) newVal = lastVal + decel;
  }

  return newVal;
}
