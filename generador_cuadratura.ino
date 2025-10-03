#include <si5351.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ======= OLED =======
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// ======= PINES =======
#define BUTTON_PIN 4
#define ENCODER_A 3
#define ENCODER_B 2
#define calPin    11

// ======= VFO =======
volatile unsigned long currentFrequency = 27455000UL;
const unsigned long minFrequency = 100000UL;
const unsigned long maxFrequency = 220000000UL;

volatile long compensation = 0L;               // Offset de calibración (Hz)
volatile unsigned long stepSize = 10000UL;     // Paso de frecuencia
volatile unsigned long calStep  = 100UL;       // Paso de calibración
const long compensation_min = -2000000L;
const long compensation_max =  2000000L;

const char *fstep = "10kHz";

// ======= EEPROM =======
#define EEPROM_ADDR_FREQ   0
#define EEPROM_ADDR_COMP   (EEPROM_ADDR_FREQ  + sizeof(unsigned long))
#define EEPROM_ADDR_DRIVE  (EEPROM_ADDR_COMP  + sizeof(long))
unsigned long lastSavedFrequency = 0;
long lastSavedComp = 0;

// ======= ESTADO/ENCODER =======
volatile bool change = false;
volatile bool encoderMoved = false;
volatile bool encoderDirFlag = false; // true = CW, false = CCW

// ======= SI5351 =======
Si5351 si5351;

// ======= DRIVE STRENGTH =======
const uint8_t driveLevels[4] = {
  SI5351_DRIVE_2MA, SI5351_DRIVE_4MA, SI5351_DRIVE_6MA, SI5351_DRIVE_8MA
};
const char *driveLabels[4] = { "2mA", "4mA", "6mA", "8mA" };
uint8_t driveIndex = 3; // 8mA por defecto
const unsigned long DOUBLE_CLICK_MS = 350UL;

// ======= BUFFERS/UI =======
char frequency_string[20];
bool g_phaseLimited = false; // true si no cabe R en 7 bits

// ======= PROTOTIPOS =======
void actualizar();
void saveFrequency();
void handleEncoder(bool direction);
void clearEEPROM();
void encoderISR();
void cycleDriveStrength();
bool setQuadrature(uint32_t fout_hz, int32_t comp_hz);

// ======= SETUP =======
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(calPin,    INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  // Borrar EEPROM si se mantiene CAL al arrancar (opcional)
  if (digitalRead(calPin) == LOW) {
    clearEEPROM();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(F("EEPROM BORRADA"));
    display.display();
    delay(800);
  }

  // Encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, FALLING);

  // Si5351
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Drive strength desde EEPROM
  EEPROM.get(EEPROM_ADDR_DRIVE, driveIndex);
  if (driveIndex > 3) driveIndex = 3;
  si5351.drive_strength(SI5351_CLK0, driveLevels[driveIndex]);
  si5351.drive_strength(SI5351_CLK1, driveLevels[driveIndex]);

  si5351.output_enable(SI5351_CLK0, 1); // I
  si5351.output_enable(SI5351_CLK1, 1); // Q

  // Frecuencia y compensación desde EEPROM
  EEPROM.get(EEPROM_ADDR_FREQ, currentFrequency);
  if (currentFrequency < minFrequency || currentFrequency > maxFrequency)
    currentFrequency = 27455000UL;
  lastSavedFrequency = currentFrequency;

  EEPROM.get(EEPROM_ADDR_COMP, compensation);
  if (compensation < compensation_min || compensation > compensation_max)
    compensation = 0;
  lastSavedComp = compensation;

  actualizar();
}

// ======= LOOP =======
void loop() {
  // Movimiento de encoder (frecuencia o calibración según calPin)
  if (encoderMoved) {
    noInterrupts();
    bool dir = encoderDirFlag;
    encoderMoved = false;
    interrupts();
    handleEncoder(dir);
  }

  // Botón: ciclo de pasos (si calPin HIGH -> pasos de F; si LOW -> pasos de CAL)
  static bool buttonWasPressed = false;
  bool buttonNow = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonNow && !buttonWasPressed) buttonWasPressed = true;

  if (!buttonNow && buttonWasPressed) {
    buttonWasPressed = false;

    if (digitalRead(calPin) == LOW) {
      // Cicla pasos de calibración
      if (calStep == 10UL)        calStep = 100UL;
      else if (calStep == 100UL)  calStep = 1000UL;
      else if (calStep == 1000UL) calStep = 10000UL;
      else if (calStep == 10000UL)calStep = 100000UL;
      else if (calStep == 100000UL)calStep = 1000000UL;
      else                        calStep = 10UL;
    } else {
      // Cicla pasos de frecuencia
      if (stepSize == 10UL)         { stepSize = 100UL;     fstep = "100Hz";  }
      else if (stepSize == 100UL)   { stepSize = 1000UL;    fstep = "1kHz";   }
      else if (stepSize == 1000UL)  { stepSize = 5000UL;    fstep = "5kHz";   }
      else if (stepSize == 5000UL)  { stepSize = 10000UL;   fstep = "10kHz";  }
      else if (stepSize == 10000UL) { stepSize = 100000UL;  fstep = "100kHz"; }
      else if (stepSize == 100000UL){ stepSize = 1000000UL; fstep = "1MHz";   }
      else                          { stepSize = 10UL;      fstep = "10Hz";   }
    }
    change = true;
  }

  // Doble clic en calPin: cycle drive strength; un clic: guarda compensación
  static int previousCalState = HIGH;
  static unsigned long lastCalReleaseTime = 0;
  int calState = digitalRead(calPin);
  if (previousCalState == LOW && calState == HIGH) {
    unsigned long now = millis();
    if (now - lastCalReleaseTime <= DOUBLE_CLICK_MS) {
      cycleDriveStrength();
    } else {
      if (compensation != lastSavedComp) {
        EEPROM.put(EEPROM_ADDR_COMP, compensation);
        lastSavedComp = compensation;
      }
      change = true;
    }
    lastCalReleaseTime = now;
  }
  previousCalState = calState;

  if (change) {
    actualizar();
    saveFrequency();
  }
}

// ======= ISR ENCODER =======
void encoderISR() {
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  if (now - lastTime > 500UL) {
    bool dir = digitalRead(ENCODER_B);
    encoderDirFlag = !dir; // CW/CCW
    encoderMoved = true;
    lastTime = now;
  }
}

// ======= LÓGICA =======
void handleEncoder(bool direction) {
  if (digitalRead(calPin) == LOW) {
    // Ajuste de calibración
    compensation += direction ? (long)calStep : -(long)calStep;
    if (compensation < compensation_min) compensation = compensation_min;
    if (compensation > compensation_max) compensation = compensation_max;
  } else {
    // Ajuste de frecuencia
    if (direction) currentFrequency += stepSize;
    else           currentFrequency -= stepSize;

    if (currentFrequency < minFrequency) currentFrequency = minFrequency;
    if (currentFrequency > maxFrequency) currentFrequency = maxFrequency;
  }
  change = true;
}

void actualizar() {
  change = false;

  // Copias locales atómicas
  noInterrupts();
  unsigned long localFreq = currentFrequency;
  long localComp = compensation;
  interrupts();

  // String de frecuencia: "MM.mmmmmm"
  unsigned long mhz  = localFreq / 1000000UL;
  unsigned long rest = localFreq % 1000000UL;
  snprintf(frequency_string, sizeof(frequency_string), "%02lu.%06lu", mhz, rest);

  // Programa I/Q (CLK0 = I, CLK1 = Q)
  uint32_t fout = (uint32_t)localFreq;
  int32_t  comp = (int32_t)localComp;
  bool ok = setQuadrature(fout, comp);

  // UI minimal
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(5, 6);
  display.println(frequency_string);

  display.setTextSize(1);
  display.setCursor(5, 30);
  if (digitalRead(calPin) == LOW) {
    display.print(F("CAL STEP: ")); display.println(calStep);
  } else {
    display.print(F("STEP: ")); display.println(fstep);
  }

  display.setCursor(5, 42);
  display.print(F("COMP: ")); display.println(localComp);

  display.setCursor(5, 54);
  if (!ok) {
    display.print(F("I/Q: NO LOCK"));
  } else if (g_phaseLimited) {
    display.print(F("I/Q: PH LIM")); // fase limitada (R>127)
  } else {
    display.print(F("I/Q: OK"));
  }

  display.display();
}

void saveFrequency() {
  if (currentFrequency != lastSavedFrequency) {
    EEPROM.put(EEPROM_ADDR_FREQ, currentFrequency);
    lastSavedFrequency = currentFrequency;
  }
}

void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) EEPROM.write(i, 0xFF);
}

void cycleDriveStrength() {
  driveIndex = (driveIndex + 1) % 4;
  si5351.drive_strength(SI5351_CLK0, driveLevels[driveIndex]);
  si5351.drive_strength(SI5351_CLK1, driveLevels[driveIndex]);
  EEPROM.put(EEPROM_ADDR_DRIVE, driveIndex);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(8, 24);
  display.print(F("DRIVE "));
  display.print(driveLabels[driveIndex]);
  display.display();

  delay(500);
  actualizar();
}

// ===== CUADRATURA I/Q con Si5351 (CLK0=I, CLK1=Q) =====
// Usa PLLA en modo entero. Fase = R pasos (90°), si R<=127.
bool setQuadrature(uint32_t fout_hz, int32_t comp_hz)
{
  g_phaseLimited = false;

  int64_t target = (int64_t)fout_hz + (int64_t)comp_hz;
  if (target < 0) target = 0;
  uint32_t fout = (uint32_t)target;

  // Rango VCO
  const uint32_t PLL_MIN = 600000000UL;
  const uint32_t PLL_MAX = 900000000UL;

  // Cotas de R (entero) y múltiplo de 4
  uint32_t Rmin = (PLL_MIN + fout - 1) / fout;  // ceil(PLL_MIN/fout)
  uint32_t Rmax = (PLL_MAX) / fout;             // floor(PLL_MAX/fout)
  if (Rmin < 4) Rmin = 4;
  if (Rmax < 4) Rmax = 4;
  if (Rmax > 900) Rmax = 900;

  auto roundDownTo4 = [](uint32_t x){ return x - (x % 4); };
  auto roundUpTo4   = [](uint32_t x){ return ((x + 3) / 4) * 4; };

  // 1) Intentar el mayor R múltiplo de 4 que NO exceda 127 (para fase exacta)
  uint32_t Rcap = (Rmax > 127 ? 127 : Rmax);
  uint32_t R = roundDownTo4(Rcap);
  if (R < Rmin) {
    // 2) Si no cabe <=127, usar el mínimo permitido (múltiplo de 4)
    R = roundUpTo4(Rmin);
    if (R > Rmax) return false; // no hay solución
  }

  uint64_t pll_freq = (uint64_t)fout * (uint64_t)R;
  if (pll_freq < PLL_MIN || pll_freq > PLL_MAX) return false;

  // Ambas salidas desde PLLA en modo entero
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLA);
  si5351.set_int(SI5351_CLK0, 1);
  si5351.set_int(SI5351_CLK1, 1);

  // Frecuencia/PLL explícitos
  si5351.set_freq_manual((uint64_t)fout * SI5351_FREQ_MULT,
                         pll_freq * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.set_freq_manual((uint64_t)fout * SI5351_FREQ_MULT,
                         pll_freq * SI5351_FREQ_MULT, SI5351_CLK1);

  // Fases: 0° (I) y 90° (Q) -> fase = R pasos (si R<=127). Si R>127, limitar.
  uint8_t phaseSteps;
  if (R > 127) {
    g_phaseLimited = true;
    phaseSteps = 127;            // mejor aproximación posible
  } else {
    phaseSteps = (uint8_t)R;     // 90° exactos
  }

  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phaseSteps);

  // Aplicar fases
  si5351.pll_reset(SI5351_PLLA);

  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 1);

  return true;
}
