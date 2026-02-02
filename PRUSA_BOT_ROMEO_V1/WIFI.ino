#if defined(ENABLE_OTA) || defined(ENABLE_UDP_DEBUG)

#include <WiFi.h>
#include <WiFiUdp.h>
#include "WiFiSecrets.h"

#ifdef ENABLE_UDP_DEBUG
WiFiUDP udp;
#endif

const int port = 9000;
unsigned long nextWifiRetryMs = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
bool  notifiedConnected = false;

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);    // keep trying after drops
  WiFi.persistent(false);         // don’t write creds to flash each time  
  WiFi.begin(ssid, pass);
  //while (WiFi.status() != WL_CONNECTED) delay(100);
  //Serial.println("WiFi Connected");
  nextWifiRetryMs = millis() + WIFI_RETRY_INTERVAL_MS;
}

void wifiService() 
{
  #ifdef ENABLE_OTA
  ArduinoOTA.handle();  // Handles OTA
  #endif

  // Fire once per connection, after we *know* we're connected
  if (WiFi.status() == WL_CONNECTED && !notifiedConnected) 
  {
    notifiedConnected = true;
    Serial.printf("# WIFI CONNECTED ssid=%s ip=%s rssi=%d\n",
               WiFi.SSID().c_str(),
               WiFi.localIP().toString().c_str(),
               WiFi.RSSI());

    #ifdef ENABLE_OTA
    ArduinoOTA.begin();  // Starts OTA
    #endif

    #ifdef ENABLE_UDP_DEBUG
    udp.begin(port);
    #endif
  }  

  // If not connected, retry occasionally (non-blocking)
  if (WiFi.status() != WL_CONNECTED) 
  {
    notifiedConnected = false;
    unsigned long now = millis();
    if (now >= nextWifiRetryMs) {
      nextWifiRetryMs = now + WIFI_RETRY_INTERVAL_MS;
      Serial.println("Wifi Retry..");

      // If stuck in weird states, restart the attempt:
      //WiFi.disconnect(true /*wifioff*/);
      //delay(50);
      //WiFi.mode(WIFI_STA);
      //WiFi.begin(ssid, pass);
      WiFi.reconnect();
    }
  }
}

#endif

#ifdef ENABLE_UDP_DEBUG

bool wifiSend(const uint8_t* data, size_t len) {
  if (WiFi.status() != WL_CONNECTED) return false;
  udp.beginPacket(pcIP, port);
  udp.write(data, len);
  return udp.endPacket() == 1;
}

bool wifiPrintf(const char *fmt, ...) {
  char buf[256];

  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (n <= 0) return false;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

  return wifiSend((uint8_t*)buf, n);
}

// UDP debugging stream sent to serial lotter via COM port re-direction
void sendWiFiUDPDataStream()
{
    float EASpeed = (( (float)EncA_deltaCounts * 60.0f ) / (ENC_CPR * loopDelta))/MOTOR_MAX_RPM;
    if( !wifiPrintf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n"
    ,EASpeed,lMeasuredRpm
    ,rDesiredRpm,rMeasuredRpm
    ,normPitch,normPitchRate
    ,normYawRate
    ,pitchCurve
    ,pitch_controlFiltered,yaw_controlFiltered
    ,normRoll
    ,pitch_controlFinal,yaw_controlFinal
    ,measuredSpeedFiltered
      ) )
    {
      //Serial.println("WiFi Send Failed!");
    }
}

//  UDP remote tuning
void  udpPacketReceive()
{
  int packetSize = udp.parsePacket();
  if( packetSize )
  {
    Serial.print("Got UDP Packet = ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(udp.remotePort());

    while(packetSize--)
    {
      char c = udp.read();
      switch(c)
      {
        case 'P':
          pTrim += PTRIM_INC;
          break;
        case 'p':
          pTrim -= PTRIM_INC;
          if( pTrim < 0 ) pTrim = 0;
          break;
        case 'I':
          iTrim += ITRIM_INC;
          break;
        case 'i':
          iTrim -= ITRIM_INC;
          if( iTrim < 0 ) iTrim = 0;
          break;
        case 'D':
          dTrim += DTRIM_INC;
          break;
        case 'd':
          dTrim -= DTRIM_INC;
          if( dTrim < 0 ) dTrim = 0;
          break;            
        case 'T':
          trimTemp += 0.01f;
          if( trimTemp > 2 ) trimTemp = 2;
          break;
        case 't':
          trimTemp -= 0.01f;
          if( trimTemp < -2 ) trimTemp = -2;
          break;            
      }
    }
    //CDF_alpha = 0.5f + trimTemp;
    //if( CDF_alpha > 1 ) CDF_alpha = 1;
    //if( CDF_alpha < 0 ) CDF_alpha = 0;
    //CDF_beta = (CDF_alpha*CDF_alpha)/ 4;

    //CDF_beta = 0.1f + trimTemp;
    //if( CDF_beta > 1 ) CDF_beta = 1;
    //if( CDF_beta < 0 ) CDF_beta = 0;
    //CDF_beta = (CDF_alpha*CDF_alpha)/ 4;

    Serial.printf("PID=%.6f,%.6f,%.6f,%.6f\r\n",pTrim,iTrim,dTrim,trimTemp);
    wifiPrintf("PID=%.6f,%.6f,%.6f,%.6f\r\n",pTrim,iTrim,dTrim,trimTemp);

    // uncomment depending on which PID is being tuned
    //lPid.SetTunings(pTrim,iTrim,dTrim);
    //rPid.SetTunings(pTrim,iTrim,dTrim);
    //yawPid.SetTunings(pTrim,iTrim,dTrim);
    //speedPid.SetTunings(pTrim,iTrim,dTrim);

  }
}

#endif

