#define ROTATE_IMU_90 // mounted side on.

#define GYRO_FOF (0.05f)
#define ATTI_FOF (0.05f)

#ifndef RAD_TO_DEG
const float RAD_TO_DEG = 57.295779513f;
#endif

typedef struct {
  float yaw;
  float pitch;
  float roll;
} euler_s;

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_s* ypr, bool degrees ) ;

euler_s ypr;
gyro_s gyro;
//gyro_s linaccel;
//gyro_s linvelo;

void normailseIMUData()
{
  static float oldpitch = 0;
  static float oldroll = 0;

  // filtered IMU gyros
  gyroF.x = firstOrderLPF(gyro.x,gyroF.x,loopDelta,GYRO_FOF);
  gyroF.y = firstOrderLPF(gyro.y,gyroF.y,loopDelta,GYRO_FOF);

  // Normalise, filter and limit IMU data
  normPitch = firstOrderLPF(ypr.pitch * (1.0F/IMU_PITCH_MAX),oldpitch,loopDelta,ATTI_FOF);
  oldpitch = normPitch;

  // Clamp
  normPitch = constrain(normPitch,-1.0f,1.0f);

  // Normalise filtered pitch rate
  normPitchRate = gyroF.y * (1.0F/IMU_PITCH_RATE_MAX);
  normPitchRate = constrain(normPitchRate,-1.0f,1.0f);

  // Normalise roll ( no need to filter as only used for tilt sensor )
  normRoll = ypr.roll * (1.0F/IMU_ROLL_MAX);
  normRoll = constrain(normRoll,-1.0f,1.0f);
  
  // Normalise filtered roll rate
  normYawRate = gyroF.x * (-1.0F/IMU_YAW_RATE_MAX);
  normYawRate = constrain(normYawRate,-1.0f,1.0f);

}

void BNOSetEvents()
{
  setReports(SH2_ARVR_STABILIZED_RV, 10000);
  setReports(SH2_GYROSCOPE_CALIBRATED, 10000);
  //setReports(SH2_LINEAR_ACCELERATION, 5000);
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_s* ypr, bool degrees = false) 
{
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_s* ypr, bool degrees = false) 
{
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void BNOService()
{
  static uint32_t  BNOLastTime = 0;

  if (bno08x.wasReset()) 
  {
    Serial.print("sensor was reset ");
    BNOSetEvents();
  }

  if (bno08x.getSensorEvent(&sensorValue)) 
  {
    uint32_t nowTime = micros();
    BNOLastTime = nowTime;
    //float dt = (nowTime - BNOLastTime) * 1e-6f;
    //Serial.println("Event");
    switch (sensorValue.sensorId) 
    {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        //Serial.println("G");
      #ifdef ROTATE_IMU_90
        gyro.x = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        gyro.y = sensorValue.un.gyroscope.x * RAD_TO_DEG;
        gyro.z = sensorValue.un.gyroscope.y * RAD_TO_DEG;
      #else
        gyro.x = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        gyro.y = sensorValue.un.gyroscope.y * RAD_TO_DEG;
        gyro.z = sensorValue.un.gyroscope.x * RAD_TO_DEG;
      #endif
        break;
      case SH2_LINEAR_ACCELERATION:
        break;

    }
  }
}

void setReports(sh2_SensorId_t reportType, long report_interval) 
{
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) 
  {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_s* ypr, bool degrees = false) 
{

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

#ifdef ROTATE_IMU_90
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->pitch = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
#else
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
#endif

    if (degrees) 
    {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}


void debugAHRS()
{
    Serial.print("AP:");
    Serial.print(ypr.pitch, 2);
    Serial.print(",GP:");
    Serial.print(gyroF.y, 2);
    Serial.print(",GZ:");
    Serial.println(gyroF.z, 2);  
}
