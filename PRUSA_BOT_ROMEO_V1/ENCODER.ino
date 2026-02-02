volatile int32_t EncA_encCount = 0;
volatile uint8_t EncA_prevState = 0;
volatile int32_t EncA_encLastCount = 0;
int32_t          EncA_newCount = 0;
int32_t          EncA_deltaCounts = 0;

volatile int32_t EncB_encCount = 0;
volatile uint8_t EncB_prevState = 0;
volatile int32_t EncB_encLastCount = 0;
int32_t          EncB_newCount = 0;
int32_t          EncB_deltaCounts = 0;

float EncA_Speed = 0;
float EncB_Speed = 0;
float MotA_OutputSpeed = 0;
float MotB_OutputSpeed = 0;

void  initEncoders()
{
  EncA_prevState = readAB(MOTA_ENCA,MOTA_ENCB);
  EncB_prevState = readAB(MOTB_ENCA,MOTB_ENCB);
}

void resetEncoders()
{
  EncA_encLastCount = EncA_encCount;
  EncB_encLastCount = EncB_encCount;
}

void  updateEncoders()
{
  noInterrupts();
  EncA_newCount = EncA_encCount;
  EncB_newCount = EncB_encCount;
  interrupts();

  EncA_deltaCounts = EncA_newCount - EncA_encLastCount;
  EncB_deltaCounts = EncB_newCount - EncB_encLastCount;
  EncA_encLastCount = EncA_newCount;
  EncB_encLastCount = EncB_newCount;

}

inline uint8_t readAB(int A, int B) {
  // pack A as bit1, B as bit0
  uint8_t a = digitalRead(A) ? 1 : 0;
  uint8_t b = digitalRead(B) ? 1 : 0;
  return (a << 1) | b;
}

// Lookup table for quadrature transitions.
// Index = (prev<<2) | curr. Value = -1,0,+1
const int8_t qdec[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

void EncA_isrAB() {
  uint8_t curr = readAB(MOTA_ENCA,MOTA_ENCB);
  uint8_t idx = (EncA_prevState << 2) | curr;
  EncA_encCount += qdec[idx];
  EncA_prevState = curr;
}

void EncB_isrAB() {
  uint8_t curr = readAB(MOTB_ENCA,MOTB_ENCB);
  uint8_t idx = (EncB_prevState << 2) | curr;
  EncB_encCount += qdec[idx];
  EncB_prevState = curr;
}


