/********************************************************************************************
 * Generador I/Q con Si5351 (CLK0 = I, CLK1 = Q)
 * ------------------------------------------------------------------------------------------
 * - Salidas en cuadratura (90°) usando el mismo PLL (PLLA), en modo entero (sin fraccional).
 * - El desfase en el Si5351 se programa en "pasos de 1/4 de periodo del PLL".
 *   Para 90° (1/4 del periodo de salida) y con R = fPLL/fOUT (entero), se debe escribir:
 *     fase = R  (no R/4)
 *   porque: 1 periodo de salida = R periodos de PLL -> 1/4 de salida = R/4 periodos de PLL
 *           y cada paso equivale a 1/4 periodo de PLL -> (R/4) * 4 = R pasos.
 *
 * - Límite de fase del chip: 7 bits (0..127).
 *   Si R > 127, no se puede cargar R completo; el código limita a 127 y avisa en pantalla.
 *
 * - UI mínima:
 *     * Frecuencia actual (MHz con 6 decimales)
 *     * STEP (paso de frecuencia) o CAL STEP (paso de calibración) según calPin
 *     * COMP (compensación en Hz)
 *     * Estado I/Q: OK (90° exactos), PH LIM (fase limitada), o NO LOCK (no se pudo programar)
 *
 * - Controles:
 *     * Encoder (A/B): cambia frecuencia o compensación (si calPin está LOW)
 *     * BUTTON_PIN: cicla paso de frecuencia (calPin HIGH) o de calibración (calPin LOW)
 *     * calPin:
 *         - Mantener LOW al arrancar: borra EEPROM (opcional)
 *         - Un clic (liberar): guarda compensación en EEPROM
 *         - Doble clic rápido: cambia drive strength (2/4/6/8 mA) en CLK0/CLK1
 *
 * - EEPROM:
 *     * Guarda frecuencia, compensación y drive strength
 *
 * Dependencias:
 *   - Etherkit Si5351 Library
 *   - Adafruit SSD1306
 *   - EEPROM
 ********************************************************************************************/

#include <si5351.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

/* ======================= OLED ======================= */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

/* ======================= PINES ======================= */
#define BUTTON_PIN 4     // Botón para ciclar "step" (o "calStep" si calPin está LOW)
#define ENCODER_A 3      // Canal A del encoder (INT)
#define ENCODER_B 2      // Canal B del encoder
#define calPin    11     // Pin de calibración: LOW -> modo calibración / doble clic -> drive

/* ============== VFO y parámetros de trabajo ============== */
volatile unsigned long currentFrequency = 27455000UL; // Hz (frecuencia de salida)
const unsigned long minFrequency = 100000UL;          // Límite inferior de salida (100 kHz)
const unsigned long maxFrequency = 220000000UL;       // Límite superior de salida (220 MHz)

volatile long compensation = 0L;               // Corrección en Hz que se suma a la salida
volatile unsigned long stepSize = 10000UL;     // Paso normal de frecuencia (default 10 kHz)
volatile unsigned long calStep  = 100UL;       // Paso de calibración (default 100 Hz)
const long compensation_min = -2000000L;       // Rango permitido de compensación (-2 MHz)
const long compensation_max =  2000000L;       // (+2 MHz)

const char *fstep = "10kHz";                   // Texto que refleja el step actual (UI)

/* ======================= EEPROM =======================
 * Memorizamos: frecuencia, compensación y drive strength.
 * Se usan direcciones consecutivas y tipos fijos (unsigned long, long, uint8_t implícito).
 */
#define EEPROM_ADDR_FREQ   0
#define EEPROM_ADDR_COMP   (EEPROM_ADDR_FREQ  + sizeof(unsigned long))
#define EEPROM_ADDR_DRIVE  (EEPROM_ADDR_COMP  + sizeof(long))
unsigned long lastSavedFrequency = 0;          // Para evitar escribir demasiado
long lastSavedComp = 0;

/* ============== Estado para encoder/actualización UI ============== */
volatile bool change = false;                  // Disparador para refrescar pantalla / programar Si5351
volatile bool encoderMoved = false;            // Señal de movimiento del encoder (se pone en ISR)
volatile bool encoderDirFlag = false;          // true = CW, false = CCW (sentido)

/* ======================= Si5351 ======================= */
Si5351 si5351;                                 // Instancia del sintetizador

/* ======================= Drive strength =======================
 * Niveles de corriente de salida del Si5351 por canal (CLKx).
 * Se controlan con doble clic en calPin.
 */
const uint8_t driveLevels[4] = {
  SI5351_DRIVE_2MA, SI5351_DRIVE_4MA, SI5351_DRIVE_6MA, SI5351_DRIVE_8MA
};
const char *driveLabels[4] = { "2mA", "4mA", "6mA", "8mA" };
uint8_t driveIndex = 3;                        // 8mA por defecto (valor "3")

/* Doble clic: ventana máxima entre clics para considerarlo doble */
const unsigned long DOUBLE_CLICK_MS = 350UL;

/* ======================= Buffers/UI ======================= */
char frequency_string[20];    // "MM.mmmmmm"
bool g_phaseLimited = false;  // true si no se pudo cargar fase=R (R>127), se limita a 127

/* ======================= Prototipos ======================= */
void actualizar();
void saveFrequency();
void handleEncoder(bool direction);
void clearEEPROM();
void encoderISR();
void cycleDriveStrength();
bool setQuadrature(uint32_t fout_hz, int32_t comp_hz);

/* ================================= SETUP ================================= */
void setup() {
  /* ---- Configuración de pines ---- */
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(calPin,    INPUT_PULLUP);

  /* ---- Inicializa OLED ---- */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  /* ---- Borrado de EEPROM si se arranca con calPin LOW (opcional) ---- */
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

  /* ---- Encoder: interrupción en flanco de bajada del canal A ----
   * En la ISR solo marcamos "encoderMoved" y el sentido; la lógica pesada va en loop().
   */
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, FALLING);

  /* ---- Inicializa Si5351 ----
   * Cristal por defecto (8 pF); el segundo y tercer parámetro permiten calibrar el XTAL.
   */
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  /* ---- Recupera drive strength desde EEPROM y aplica a CLK0/CLK1 ---- */
  EEPROM.get(EEPROM_ADDR_DRIVE, driveIndex);
  if (driveIndex > 3) driveIndex = 3; // Sanitiza lectura
  si5351.drive_strength(SI5351_CLK0, driveLevels[driveIndex]);
  si5351.drive_strength(SI5351_CLK1, driveLevels[driveIndex]);

  /* ---- Habilita salidas de I/Q ---- */
  si5351.output_enable(SI5351_CLK0, 1); // I
  si5351.output_enable(SI5351_CLK1, 1); // Q

  /* ---- Recupera frecuencia y compensación desde EEPROM ---- */
  EEPROM.get(EEPROM_ADDR_FREQ, currentFrequency);
  if (currentFrequency < minFrequency || currentFrequency > maxFrequency)
    currentFrequency = 27455000UL; // Valor seguro por defecto
  lastSavedFrequency = currentFrequency;

  EEPROM.get(EEPROM_ADDR_COMP, compensation);
  if (compensation < compensation_min || compensation > compensation_max)
    compensation = 0; // Valor seguro por defecto
  lastSavedComp = compensation;

  /* ---- Programa salidas y pinta UI ---- */
  actualizar();
}

/* ================================= LOOP ================================= */
void loop() {
  /* --- 1) Movimiento del encoder ---
   * Si calPin está LOW, el encoder ajusta "compensation".
   * Si calPin está HIGH, el encoder ajusta "currentFrequency".
   */
  if (encoderMoved) {
    noInterrupts();
    bool dir = encoderDirFlag;   // Copia local del sentido
    encoderMoved = false;        // Limpiamos la bandera
    interrupts();
    handleEncoder(dir);          // Aplica cambio y marca "change = true"
  }

  /* --- 2) Botón principal: cicla step o calStep ---
   * - calPin LOW  -> cicla calStep (tamaños del paso de calibración)
   * - calPin HIGH -> cicla stepSize (tamaños del paso de frecuencia)
   */
  static bool buttonWasPressed = false;
  bool buttonNow = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonNow && !buttonWasPressed) buttonWasPressed = true;

  if (!buttonNow && buttonWasPressed) {
    buttonWasPressed = false;

    if (digitalRead(calPin) == LOW) {
      // Ciclo de pasos de calibración (en Hz)
      if      (calStep == 10UL)        calStep = 100UL;
      else if (calStep == 100UL)       calStep = 1000UL;
      else if (calStep == 1000UL)      calStep = 10000UL;
      else if (calStep == 10000UL)     calStep = 100000UL;
      else if (calStep == 100000UL)    calStep = 1000000UL;
      else                             calStep = 10UL;
    } else {
      // Ciclo de pasos de frecuencia (y su etiqueta de UI)
      if      (stepSize == 10UL)         { stepSize = 100UL;     fstep = "100Hz";  }
      else if (stepSize == 100UL)        { stepSize = 1000UL;    fstep = "1kHz";   }
      else if (stepSize == 1000UL)       { stepSize = 5000UL;    fstep = "5kHz";   }
      else if (stepSize == 5000UL)       { stepSize = 10000UL;   fstep = "10kHz";  }
      else if (stepSize == 10000UL)      { stepSize = 100000UL;  fstep = "100kHz"; }
      else if (stepSize == 100000UL)     { stepSize = 1000000UL; fstep = "1MHz";   }
      else                               { stepSize = 10UL;      fstep = "10Hz";   }
    }
    change = true; // Pide refrescar UI/Si5351
  }

  /* --- 3) calPin: clic simple vs doble clic ---
   * - Al soltar (LOW->HIGH) dentro de DOUBLE_CLICK_MS desde el último: DOBLE CLIC -> cambia drive
   * - Si no fue doble clic: guarda "compensation" en EEPROM.
   */
  static int previousCalState = HIGH;
  static unsigned long lastCalReleaseTime = 0;
  int calState = digitalRead(calPin);

  if (previousCalState == LOW && calState == HIGH) {     // Se soltó calPin
    unsigned long now = millis();
    if (now - lastCalReleaseTime <= DOUBLE_CLICK_MS) {
      // Doble clic: rotamos drive strength (2/4/6/8 mA) en ambas salidas
      cycleDriveStrength();
    } else {
      // Clic simple: persistimos compensación si cambió
      if (compensation != lastSavedComp) {
        EEPROM.put(EEPROM_ADDR_COMP, compensation);
        lastSavedComp = compensation;
      }
      change = true;
    }
    lastCalReleaseTime = now;
  }
  previousCalState = calState;

  /* --- 4) Si hubo cambios, reprograma salidas y refresca pantalla --- */
  if (change) {
    actualizar();
    saveFrequency(); // Guarda frecuencia si cambió (protege EEPROM contra escrituras innecesarias)
  }
}

/* ======================= ISR del encoder =======================
 * Se ejecuta en interrupción (ligera). Evitamos tocar variables de 32 bits complejas aquí.
 * Solo detectamos sentido y marcamos que hubo movimiento.
 */
void encoderISR() {
  static unsigned long lastTime = 0;
  unsigned long now = micros();

  // Filtro antirrebote por tiempo (500 us)
  if (now - lastTime > 500UL) {
    bool dir = digitalRead(ENCODER_B); // Lee B para saber dirección relativa
    encoderDirFlag = !dir;             // Convención: !dir = CW (ajústalo si prefieres)
    encoderMoved = true;               // Señalamos al loop() que hay trabajo
    lastTime = now;
  }
}

/* ======================= Lógica de cambios por encoder =======================
 * Si calPin LOW: ajusta "compensation" (calibración fina, guarda con un clic en calPin).
 * Si calPin HIGH: ajusta "currentFrequency" dentro de los límites.
 */
void handleEncoder(bool direction) {
  if (digitalRead(calPin) == LOW) {
    // Modo calibración: suma/resta en pasos "calStep"
    compensation += direction ? (long)calStep : -(long)calStep;

    // Satura al rango permitido
    if (compensation < compensation_min) compensation = compensation_min;
    if (compensation > compensation_max) compensation = compensation_max;

  } else {
    // Modo frecuencia: suma/resta en pasos "stepSize"
    if (direction) currentFrequency += stepSize;
    else           currentFrequency -= stepSize;

    // Satura al rango permitido
    if (currentFrequency < minFrequency) currentFrequency = minFrequency;
    if (currentFrequency > maxFrequency) currentFrequency = maxFrequency;
  }

  change = true; // Pedir reprogramar salidas y refrescar UI
}

/* ======================= Actualiza salidas y UI =======================
 * 1) Formatea la frecuencia en texto.
 * 2) Llama a setQuadrature() para programar I/Q (CLK0, CLK1).
 * 3) Dibuja la UI mínima (step o calStep, compensación y estado I/Q).
 */
void actualizar() {
  change = false;

  // Copias locales atómicas (evita leer 32 bits mientras ISR pueda modificarlos)
  noInterrupts();
  unsigned long localFreq = currentFrequency;
  long localComp = compensation;
  interrupts();

  // Formatea "MM.mmmmmm"
  unsigned long mhz  = localFreq / 1000000UL;
  unsigned long rest = localFreq % 1000000UL;
  snprintf(frequency_string, sizeof(frequency_string), "%02lu.%06lu", mhz, rest);

  // Programa I/Q. 'ok' indica si fue posible configurar PLL y divisores.
  uint32_t fout = (uint32_t)localFreq;
  int32_t  comp = (int32_t)localComp;
  bool ok = setQuadrature(fout, comp);

  // ----------------- UI -----------------
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
    display.print(F("I/Q: NO LOCK"));   // No se pudo configurar (fuera de rango u otra razón)
  } else if (g_phaseLimited) {
    display.print(F("I/Q: PH LIM"));    // Fase limitada (R>127) -> no hay 90° exactos
  } else {
    display.print(F("I/Q: OK"));        // Todo correcto, 90° programados
  }

  display.display();
}

/* ======================= Guardado de frecuencia =======================
 * Minimiza escrituras a EEPROM: solo guarda si realmente cambió.
 */
void saveFrequency() {
  if (currentFrequency != lastSavedFrequency) {
    EEPROM.put(EEPROM_ADDR_FREQ, currentFrequency);
    lastSavedFrequency = currentFrequency;
  }
}

/* ======================= Borrar EEPROM (opcional) =======================
 * Útil para pruebas; se invoca manteniendo calPin LOW al inicio.
 */
void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) EEPROM.write(i, 0xFF);
}

/* ======================= Cambiar drive strength =======================
 * Con doble clic en calPin, rota 2/4/6/8 mA para CLK0 y CLK1.
 * Guarda el índice en EEPROM y muestra un aviso efímero en pantalla.
 */
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

  delay(500); // Mensaje visible un momento
  actualizar(); // Regresa a pantalla principal
}

/* ======================= setQuadrature() =======================
 * Programa dos salidas en cuadratura exacta (90°) si es posible.
 *
 * Entradas:
 *   fout_hz  : frecuencia objetivo (Hz)
 *   comp_hz  : compensación (Hz) a sumar/restar a fout_hz
 *
 * Requisitos para cuadratura estable:
 *   - Ambas salidas (CLK0, CLK1) deben salir del MISMO PLL (PLLA).
 *   - Modo entero (sin fraccional) para que la fase sea determinista.
 *   - R = fPLL / fOUT debe ser ENTERO y preferentemente múltiplo de 4.
 *   - Para cargar "fase = R" (90°), R debe ser <= 127 (límite de 7 bits).
 *
 * Estrategia:
 *   1) Calcula Rmin = ceil(PLL_MIN / fout), Rmax = floor(PLL_MAX / fout).
 *   2) Intenta elegir el mayor R múltiplo de 4 PERO <= 127 (para poder cargar fase exacta).
 *   3) Si no hay R <= 127, usa el mínimo múltiplo de 4 >= Rmin (y marca g_phaseLimited).
 *   4) Programa:
 *        - CLK0/1 desde PLLA
 *        - Modo entero para ambos
 *        - set_freq_manual() en ambos con el mismo PLL (plla_freq)
 *        - set_phase(CLK0, 0) y set_phase(CLK1, fase=R ó 127 si limitado)
 *        - pll_reset(PLLA) para aplicar fase
 */
bool setQuadrature(uint32_t fout_hz, int32_t comp_hz)
{
  g_phaseLimited = false; // Asumimos que podremos programar fase exacta

  // Suma compensación (signed) y protege contra underflow
  int64_t target = (int64_t)fout_hz + (int64_t)comp_hz;
  if (target < 0) target = 0;
  uint32_t fout = (uint32_t)target; // Frecuencia efectiva de salida

  // Rango de VCO permitido por el Si5351 (PLLA)
  const uint32_t PLL_MIN = 600000000UL; // 600 MHz
  const uint32_t PLL_MAX = 900000000UL; // 900 MHz

  // 1) Determina rango de R = fPLL/fOUT (entero) compatible con PLL_MIN..PLL_MAX
  //    Rmin = ceil(PLL_MIN / fout), Rmax = floor(PLL_MAX / fout)
  uint32_t Rmin = (PLL_MIN + fout - 1) / fout;
  uint32_t Rmax = (PLL_MAX) / fout;

  // Asegura rangos razonables (hoja de datos sugiere 4..900 aprox.)
  if (Rmin < 4)  Rmin = 4;
  if (Rmax < 4)  Rmax = 4;
  if (Rmax > 900) Rmax = 900;

  // Helpers para redondear a múltiplos de 4 (conveniente para cuadratura y extensiones -Q)
  auto roundDownTo4 = [](uint32_t x){ return x - (x % 4); };
  auto roundUpTo4   = [](uint32_t x){ return ((x + 3) / 4) * 4; };

  // 2) Preferimos un R múltiplo de 4 que QUEPA en 7 bits (<=127) para poder fijar fase=R
  uint32_t Rcap = (Rmax > 127 ? 127 : Rmax);  // cap a 127 (límite de fase)
  uint32_t R    = roundDownTo4(Rcap);

  // Si ese R quedó por debajo del mínimo, no hay opción <=127: tomamos el mínimo válido
  if (R < Rmin) {
    R = roundUpTo4(Rmin); // menor R posible, múltiplo de 4
    if (R > Rmax) return false; // No existe solución con PLL en rango
  }

  // 3) Calcula frecuencia del PLL con ese R (plla = fout * R) y verifica rango
  uint64_t pll_freq = (uint64_t)fout * (uint64_t)R;
  if (pll_freq < PLL_MIN || pll_freq > PLL_MAX) return false;

  // 4) Configura ambos multisintetizadores saliendo de PLLA y en modo entero
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLA);
  si5351.set_int(SI5351_CLK0, 1);
  si5351.set_int(SI5351_CLK1, 1);

  // 5) Programa frecuencia explícita con PLL explícito (evita que la lib use otro PLL)
  //    La librería espera Hz * SI5351_FREQ_MULT.
  si5351.set_freq_manual((uint64_t)fout * SI5351_FREQ_MULT,
                         pll_freq      * SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.set_freq_manual((uint64_t)fout * SI5351_FREQ_MULT,
                         pll_freq      * SI5351_FREQ_MULT, SI5351_CLK1);

  // 6) Programa fase:
  //    - CLK0: 0 pasos (0°)
  //    - CLK1: R pasos (90° exactos) si R <= 127; si no, limitamos a 127 y marcamos bandera.
  uint8_t phaseSteps;
  if (R > 127) {
    g_phaseLimited = true; // No cabe la fase exacta en 7 bits
    phaseSteps = 127;      // Mejor aproximación posible (126/127 a veces apenas varía)
  } else {
    phaseSteps = (uint8_t)R; // 90° exactos
  }

  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phaseSteps);

  // 7) Reset de PLL para aplicar las fases (requerido por el chip)
  si5351.pll_reset(SI5351_PLLA);

  // (Re)habilita salidas por si acaso
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 1);

  return true;
}
