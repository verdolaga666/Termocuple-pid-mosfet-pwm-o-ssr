// ================================================================
// LIBRERÍAS
// ================================================================
#include <U8x8lib.h>
#include <GyverMAX6675.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <EEPROM.h>

// ================================================================
// CONFIGURACIÓN DE PINES Y COMPONENTES
// ================================================================
#define THERMO_CS_PIN 10
#define THERMO_SO_PIN 12
#define THERMO_SCK_PIN 13
GyverMAX6675<THERMO_SCK_PIN, THERMO_SO_PIN, THERMO_CS_PIN> thermocouple;
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
#define ENCODER_BUTTON 4
Encoder myEnc(ENCODER_PIN_B, ENCODER_PIN_A);
#define MOSFET_PIN 9
#define BUZZER_PIN 11
#define LED_BUILTIN 13

// ================================================================
// VARIABLES GLOBALES Y ENUMERACIONES
// ================================================================
enum SystemState {
  BOOT_SCREEN,
  CONTROL,
  MENU,
  ADVANCED_CONFIG,
  ONOFF_CONFIG,
  PID_CONFIG,
  MANUAL_CONTROL,
  CALIBRATION_CONFIG,
  OUTPUT_TYPE_CONFIG,
  SSR_ACTIVATION_CONFIRMATION,
  MOSFET_ACTIVATION_CONFIRMATION,
  ABOUT_SCREEN,
  TEST_MODE_SCREEN,
  RESET_CONFIRMATION,
  // ➡️ NUEVA FUNCIÓN
  // Nuevo estado para la configuración de alarmas.
  ALARM_CONFIG,
  ERROR
};
SystemState currentState = BOOT_SCREEN;
unsigned long bootScreenStartTime = 0;
const unsigned long bootScreenDuration = 3000;
enum ControlMode {
  MODE_ONOFF,
  MODE_PID,
  MODE_MANUAL
};
ControlMode currentControlMode;

enum OutputType {
  OUTPUT_MOSFET_PWM,
  OUTPUT_SSR
};
OutputType currentOutputType;
bool outputActive = false;
bool serialConnected = false;

// ➡️ NUEVA FUNCIÓN
// Variables para las alarmas.
float highTempAlarm = 90.0;
bool highTempAlarmActive = false;
bool setpointReachedAlarm = true; // Por defecto activada
bool buzzerEnabled = true;
float highTempAlarmAux;
const char mainMenuOptions_0[] PROGMEM = "Activar / Desac.  ";
const char mainMenuOptions_1[] PROGMEM = "Opciones Avanzadas";
const char mainMenuOptions_2[] PROGMEM = "Modo Manual       ";
const char mainMenuOptions_3[] PROGMEM = "Calibracion       ";
const char mainMenuOptions_4[] PROGMEM = "Prueba Salida     ";
const char mainMenuOptions_5[] PROGMEM = "Acerca de         ";
const char mainMenuOptions_6[] PROGMEM = "Reiniciar         ";
const char mainMenuOptions_7[] PROGMEM = "Salir             ";
const char* const mainMenuOptions[] PROGMEM = {
  mainMenuOptions_0,
  mainMenuOptions_1,
  mainMenuOptions_2,
  mainMenuOptions_3,
  mainMenuOptions_4,
  mainMenuOptions_5,
  mainMenuOptions_6,
  mainMenuOptions_7,
};
const int totalMenuOptions = 8;

// ➡️ MODIFICACIÓN A CÓDIGO
// Agregamos "Config Alarmas" al menú avanzado.
const char advancedMenuOptions_0[] PROGMEM = "Config PID        ";
const char advancedMenuOptions_1[] PROGMEM = "Config ON/OFF     ";
const char advancedMenuOptions_2[] PROGMEM = "Config Alarmas    ";
const char advancedMenuOptions_3[] PROGMEM = "Salida: SSR       ";
const char advancedMenuOptions_4[] PROGMEM = "Salida: MOSFET    ";
const char advancedMenuOptions_5[] PROGMEM = "Salir             ";
const char* const advancedMenuOptions[] PROGMEM = {
  advancedMenuOptions_0,
  advancedMenuOptions_1,
  advancedMenuOptions_2,
  advancedMenuOptions_3,
  advancedMenuOptions_4,
  advancedMenuOptions_5
};
const int totalAdvancedMenuOptions = 6;

const int optionsPerPage = 6;

int selectedOption = 0;
int selectedAdvancedOption = 0;
int confirmationCount = 0;
int menuPage = 0;

float setpoint;
float currentTemp = 0.0;
double pidInput, pidOutput, pidSetpoint;
double kp, ki, kd;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, kp, ki, kd, DIRECT);
float onOffHysteresis;
float calibrationOffset;
float tempSetAux, onOffHysteresisAux, calibrationOffsetAux, kpAux, kiAux, kdAux;
ControlMode controlModeAux;
OutputType outputTypeAux;
int manualOutputValue = 0;
int manualOutputValueAux = 0;
long encoderPosition = 0;
long oldEncoderPosition = 0;
bool buttonClicked = false;
bool longPressDetected = false;
bool oldButtonState = HIGH;
unsigned long buttonPressStartTime = 0;
const unsigned long longPressTime = 1000;
unsigned long lastButtonClickTime = 0;
const unsigned long CONFIRMATION_TIMEOUT = 5000;

unsigned long lastSensorReadTime = 0;
unsigned long errorTimer = 0;
float lastTempForProtection = 0;
unsigned long lastTempCheckTime = 0;
const unsigned long SENSOR_TIMEOUT = 5000;
const unsigned long TEMP_RISE_TIMEOUT = 180000;
const float OVERSHOOT_LIMIT = 250.0;
const float TEMP_DROP_THRESHOLD = 5.0;
unsigned long lastTempDropCheckTime = 0;
float tempAtLastCheck = 0.0;
const int x_offset = 1;

unsigned long lastEncoderMovementTime = 0;
const unsigned long fastTurnThreshold = 200;
long encoderDelta = 0;

// ➡️ CAMBIO SOLICITADO
// Variables para el temporizador de inactividad
unsigned long lastInteractionTime = 0;
const unsigned long menuTimeout = 10000; // 30 segundos

// ================================================================
// PROTOTIPOS DE FUNCIONES
// ================================================================
void loadConfiguration();
void saveConfiguration();
void resetToFactorySettings();
byte calculateChecksum();
void displayBootScreen();
void handleControlState(long newPos);
void handleMenuState(long newPos);
void handleAdvancedMenuState(long newPos);
void handleOnOffConfigState(long newPos);
void handlePIDConfigState(long newPos);
void handleManualControlState(long newPos);
void handleCalibrationState(long newPos);
void handleOutputTypeConfigState();
void handleSSRActivationConfirmationState();
void handleMosfetActivationConfirmationState();
void handleErrorState(const char* errorMsg);
void displayAbout();
void displayTestMode();
void readSensor();
void readButton();
void checkProtections();
void checkHeatingFailure();
void updateOutput(byte value);
void updateControllerOutput();
void buzzerBeep(int count, int duration, int pause);
void checkSerialConnection();
void sendSerialData();
void handleResetConfirmationState();
void drawMenu(const char* menuTitle, const char* const menuOptions[], int totalOptions, int& selectedOption, long& oldSelectedOption, bool redraw);
// ➡️ NUEVA FUNCIÓN
// Nuevo prototipo para la función de alarmas.
void checkAlarms();
void handleAlarmConfigState(long newPos);
// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(9600);
  u8x8.begin();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);

  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);

  delay(500);

  u8x8.clearDisplay();
  u8x8.drawString(x_offset, 3, "Iniciando...");

  loadConfiguration();
  if (isnan(setpoint) || setpoint < 10.0 || setpoint > 250.0) {
    setpoint = 30.0;
  }
  if (isnan(currentTemp)) {
    currentTemp = 25.0;
  }

  myPID.SetTunings(kp, ki, kd);
  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);

  displayBootScreen();
  // ➡️ CAMBIO SOLICITADO
  // Inicializamos el temporizador de inactividad
  lastInteractionTime = millis();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  long newPos = myEnc.read() / 4;
  readButton();
  readSensor();
  checkProtections();
  checkHeatingFailure();
  checkSerialConnection();
  sendSerialData();
  // ➡️ NUEVA FUNCIÓN
  // Llamamos a la función que chequea las alarmas en cada ciclo.
  checkAlarms();

  // ➡️ CAMBIO SOLICITADO
  // Lógica de temporizador para salir del menú después de 30 segundos.
  if (currentState != BOOT_SCREEN && currentState != CONTROL && millis() - lastInteractionTime > menuTimeout) {
    currentState = CONTROL;
    u8x8.clearDisplay();
  }

  // ➡️ CAMBIO SOLICITADO
  // Actualizamos el tiempo de interacción con el encoder
  if (newPos != oldEncoderPosition) {
    lastInteractionTime = millis();
  }

  static SystemState oldState = BOOT_SCREEN;
  if (currentState != oldState) {
    u8x8.clearDisplay();
    oldState = currentState;
    if (currentState == MENU || currentState == ADVANCED_CONFIG) {
      oldEncoderPosition = -999;
    }
  }

  switch (currentState) {
    case BOOT_SCREEN:
      currentState = CONTROL;
      break;
    case MENU:
      handleMenuState(newPos);
      break;
    case ADVANCED_CONFIG:
      handleAdvancedMenuState(newPos);
      break;
    case ONOFF_CONFIG:
      handleOnOffConfigState(newPos);
      break;
    case PID_CONFIG:
      handlePIDConfigState(newPos);
      break;
    case MANUAL_CONTROL:
      handleManualControlState(newPos);
      break;
    case CALIBRATION_CONFIG:
      handleCalibrationState(newPos);
      break;
    case OUTPUT_TYPE_CONFIG:
      handleOutputTypeConfigState();
      break;
    case SSR_ACTIVATION_CONFIRMATION:
      handleSSRActivationConfirmationState();
      break;
    case MOSFET_ACTIVATION_CONFIRMATION:
      handleMosfetActivationConfirmationState();
      break;
    case ABOUT_SCREEN:
      displayAbout();
      break;
    case TEST_MODE_SCREEN:
      displayTestMode();
      break;
    case RESET_CONFIRMATION:
      handleResetConfirmationState();
      break;
    case CONTROL:
      handleControlState(newPos);
      break;
    // ➡️ NUEVA FUNCIÓN
    // Agregamos el manejo del nuevo estado de configuración de alarmas.
    case ALARM_CONFIG:
      handleAlarmConfigState(newPos);
      break;
    case ERROR:
      break;
  }

  if (currentState != ERROR) {
    updateControllerOutput();
  } else {
    updateOutput(0);
  }
}

// ================================================================
// IMPLEMENTACIÓN DE FUNCIONES
// ================================================================

// ➡️ MODIFICACIÓN A CÓDIGO
// Agregamos la carga de los nuevos valores de la EEPROM.
byte calculateChecksum() {
  byte checksum = 0;
  for (int i = 0; i < 41; i++) {
    checksum += EEPROM.read(i);
  }
  return checksum;
}

// ➡️ MODIFICACIÓN A CÓDIGO
// Agregamos la carga de los nuevos valores de la EEPROM.
void loadConfiguration() {
  byte storedChecksum;
  EEPROM.get(41, storedChecksum);

  if (storedChecksum == calculateChecksum()) {
    EEPROM.get(0, setpoint);
    EEPROM.get(4, onOffHysteresis);
    EEPROM.get(8, kp);
    EEPROM.get(12, ki);
    EEPROM.get(16, kd);
    EEPROM.get(20, calibrationOffset);
    int mode;
    EEPROM.get(24, mode);
    currentControlMode = (mode >= 0 && mode <= 2) ? (ControlMode)mode : MODE_ONOFF;
    int outputType;
    EEPROM.get(28, outputType);
    currentOutputType = (outputType >= 0 && outputType <= 1) ? (OutputType)outputType : OUTPUT_MOSFET_PWM;
    int manualOutput;
    EEPROM.get(32, manualOutput);
    manualOutputValue = (manualOutput >= 0 && manualOutput <= 255) ? manualOutput : 0;
    // Carga de las nuevas variables de alarma
    EEPROM.get(36, highTempAlarm);
    bool tempBuzzerEnabled;
    EEPROM.get(40, tempBuzzerEnabled);
    buzzerEnabled = tempBuzzerEnabled;
  } else {
    Serial.println("EEPROM Corrupta. Cargando valores de fábrica.");
    resetToFactorySettings();
  }

  tempSetAux = setpoint;
  onOffHysteresisAux = onOffHysteresis;
  kpAux = kp;
  kiAux = ki;
  kdAux = kd;
  calibrationOffsetAux = calibrationOffset;
  controlModeAux = currentControlMode;
  outputTypeAux = currentOutputType;
  manualOutputValueAux = manualOutputValue;
  highTempAlarmAux = highTempAlarm;
}

// ➡️ MODIFICACIÓN A CÓDIGO
// Agregamos el guardado de los nuevos valores en la EEPROM.
void saveConfiguration() {
  EEPROM.put(0, setpoint);
  EEPROM.put(4, onOffHysteresis);
  EEPROM.put(8, kp);
  EEPROM.put(12, ki);
  EEPROM.put(16, kd);
  EEPROM.put(20, calibrationOffset);
  EEPROM.put(24, (int)currentControlMode);
  EEPROM.put(28, (int)currentOutputType);
  EEPROM.put(32, manualOutputValue);
  // Guardado de las nuevas variables de alarma
  EEPROM.put(36, highTempAlarm);
  EEPROM.put(40, buzzerEnabled);

  byte checksum = calculateChecksum();
  EEPROM.put(41, checksum);
}

// ➡️ MODIFICACIÓN A CÓDIGO
// Agregamos los nuevos valores de alarma a los valores de fábrica.
void resetToFactorySettings() {
  setpoint = 30.0;
  onOffHysteresis = 2.0;
  kp = 4.0;
  ki = 0.5;
  kd = 0.1;
  calibrationOffset = 0.0;
  currentControlMode = MODE_ONOFF;
  currentOutputType = OUTPUT_MOSFET_PWM;
  manualOutputValue = 0;
  highTempAlarm = 90.0;
  buzzerEnabled = true;
  saveConfiguration();
}

void displayBootScreen() {
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
  u8x8.drawString(x_offset, 3, "Byte-Electronic");
  u8x8.drawString(x_offset + 4, 4, "LAB");
  delay(3000);

  buzzerBeep(1, 100, 0);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  delay(500);
}

// ➡️ MODIFICACIÓN A CÓDIGO
// Mejoras en la UI.
void handleControlState(long newPos) {
  if (isnan(currentTemp) || isnan(setpoint)) return;
  static float lastTemp = -999.0;
  static float lastSet = -999.0;
  static bool lastOutputActive = !outputActive;
  static ControlMode lastControlMode = currentControlMode;
  static int lastPidOutputPercent = -1;
  static bool lastSerialState = serialConnected;
  static OutputType lastOutputType = currentOutputType;
  static bool lastHighTempAlarmActive = false;
  if (newPos != oldEncoderPosition) {
    if (millis() - lastEncoderMovementTime < fastTurnThreshold) {
      setpoint += (float)(newPos - oldEncoderPosition) * 10;
    } else {
      setpoint += (float)(newPos - oldEncoderPosition);
    }

    if (setpoint > 250.0) setpoint = 250.0;
    if (setpoint < 10.0) setpoint = 10.0;
    saveConfiguration();
    oldEncoderPosition = newPos;
    lastEncoderMovementTime = millis();
  }

  if (buttonClicked) {
    buttonClicked = false;
    currentState = MENU;
    selectedOption = 0;
    myEnc.write(selectedOption * 4);
    return;
  }

  int pidOutputPercent = (int)(pidOutput / 2.55);
  // Redibujar solo si hay cambios o si la alarma se activó/desactivó
  if (abs(currentTemp - lastTemp) > 0.1 || abs(setpoint - lastSet) > 0.1 || outputActive != lastOutputActive || currentControlMode != lastControlMode || (currentControlMode == MODE_PID && pidOutputPercent != lastPidOutputPercent) || serialConnected != lastSerialState || lastOutputType != currentOutputType || highTempAlarmActive != lastHighTempAlarmActive) {
    u8x8.clearDisplay();
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);

    u8x8.drawString(x_offset, 0, "Temp:");
    u8x8.setCursor(x_offset + 6, 0);
    u8x8.print(currentTemp, 1);
    u8x8.print(" C");

    u8x8.drawString(x_offset, 2, "Set:");
    u8x8.setCursor(x_offset + 6, 2);
    u8x8.print(setpoint, 1);
    u8x8.print(" C");

    u8x8.drawString(x_offset, 4, "Estado:");
    u8x8.setCursor(x_offset + 8, 4);
    // ➡️ MODIFICACIÓN A CÓDIGO
    // Agregamos un indicador visual para "CALENTANDO"
    if (outputActive) {
        u8x8.print("ACTIVO");
    } else {
      u8x8.print("APAGADO");
    }

    u8x8.drawString(x_offset, 5, "Calef:");
    u8x8.setCursor(x_offset + 7, 5);
    if(pidOutput > 0) { // O si el modo ON/OFF está encendido
        u8x8.print("CALENTANDO");
    } else {
        u8x8.print("OFF");
    }

    if (serialConnected) {
      u8x8.drawString(x_offset, 6, "Serial OK");
    }
    
    u8x8.setCursor(x_offset, 7);
    if (currentOutputType == OUTPUT_SSR) {
      u8x8.print("Salida: SSR");
    } else {
      u8x8.print("Salida: MOSFET");
    }

    // ➡️ NUEVA FUNCIÓN
    // Si la alarma de alta temperatura está activa, mostramos un mensaje.
    if (highTempAlarmActive) {
        u8x8.clearDisplay();
        u8x8.drawString(x_offset, 2, "ALARMA!");
        u8x8.drawString(x_offset, 3, "TEMP ALTA");
        u8x8.drawString(x_offset, 5, "Presione para");
        u8x8.drawString(x_offset, 6, "silenciar");
    }

    lastTemp = currentTemp;
    lastSet = setpoint;
    lastOutputActive = outputActive;
    lastControlMode = currentControlMode;
    lastPidOutputPercent = pidOutputPercent;
    lastSerialState = serialConnected;
    lastOutputType = currentOutputType;
    lastHighTempAlarmActive = highTempAlarmActive;
  }
}

void drawMenu(const char* menuTitle, const char* const menuOptions[], int totalOptions, int& selectedOption, long& oldSelectedOption, bool redraw) {
    if (redraw) {
        char buffer[30];
        u8x8.clearDisplay();
        u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
        u8x8.drawString(x_offset, 0, menuTitle);

        int menuPage = selectedOption / optionsPerPage;
        int startOption = menuPage * optionsPerPage;
        int endOption = startOption + optionsPerPage;
        if (endOption > totalOptions) endOption = totalOptions;
        for (int i = startOption; i < endOption; i++) {
            strcpy_P(buffer, (PGM_P)pgm_read_word(&(menuOptions[i])));
            u8x8.drawString(x_offset + 1, (i - startOption) + 2, buffer);
        }

        u8x8.drawString(x_offset, (selectedOption - startOption) + 2, ">");
    }
}

void handleMenuState(long newPos) {
    static bool redrawMenu = true;
    if (newPos != oldEncoderPosition) {
        selectedOption += (newPos - oldEncoderPosition);
        if (selectedOption < 0) selectedOption = totalMenuOptions - 1;
        if (selectedOption >= totalMenuOptions) selectedOption = 0;
        oldEncoderPosition = newPos;
        redrawMenu = true;
    }

    drawMenu("Menu Principal", mainMenuOptions, totalMenuOptions, selectedOption, oldEncoderPosition, redrawMenu);
    redrawMenu = false;
    if (buttonClicked) {
      buttonClicked = false;
      redrawMenu = true;
      // ➡️ CAMBIO SOLICITADO
      // Actualizamos el tiempo de interacción del botón
      lastInteractionTime = millis();
      switch(selectedOption) {
          case 0:
              outputActive = !outputActive;
              if (outputActive) {
                  currentControlMode = controlModeAux;
              }
              saveConfiguration();
              currentState = CONTROL;
              myEnc.write(0);
              break;
          case 1: currentState = ADVANCED_CONFIG; myEnc.write(0); selectedAdvancedOption = 0; break;
          case 2: currentState = MANUAL_CONTROL; myEnc.write(0); break;
          case 3: currentState = CALIBRATION_CONFIG; myEnc.write(0); break;
          case 4: currentState = TEST_MODE_SCREEN; myEnc.write(0); break;
          case 5: currentState = ABOUT_SCREEN; myEnc.write(0);
              break;
          case 6: currentState = RESET_CONFIRMATION; myEnc.write(0); break;
          case 7: currentState = CONTROL; myEnc.write(0); break;
      }
    } else if (longPressDetected) {
      longPressDetected = false;
      redrawMenu = true;
      currentState = CONTROL;
    }
}

// ➡️ MODIFICACIÓN A CÓDIGO
// Agregamos el nuevo caso para el menú de alarmas.
void handleAdvancedMenuState(long newPos) {
    static bool redrawMenu = true;
    char buffer[30];
    if (newPos != oldEncoderPosition) {
        selectedAdvancedOption += (newPos - oldEncoderPosition);
        if (selectedAdvancedOption < 0) selectedAdvancedOption = totalAdvancedMenuOptions - 1;
        if (selectedAdvancedOption >= totalAdvancedMenuOptions) selectedAdvancedOption = 0;
        oldEncoderPosition = newPos;
        redrawMenu = true;
    }

    if (redrawMenu) {
        u8x8.clearDisplay();
        u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
        u8x8.drawString(x_offset, 0, "Opciones Avanzadas");
        // Ajustamos las opciones del menú para el nuevo tamaño
        for (int i = 0; i < totalAdvancedMenuOptions; i++) {
            strcpy_P(buffer, (PGM_P)pgm_read_word(&(advancedMenuOptions[i])));
            u8x8.drawString(x_offset + 1, i + 2, buffer);
        }

        u8x8.drawString(x_offset, selectedAdvancedOption + 2, ">");
        redrawMenu = false;
    }

    if (buttonClicked) {
      buttonClicked = false;
      redrawMenu = true;
      // ➡️ CAMBIO SOLICITADO
      // Actualizamos el tiempo de interacción del botón
      lastInteractionTime = millis();

      switch(selectedAdvancedOption) {
          case 0:
              if (currentOutputType == OUTPUT_SSR) {
                  u8x8.clearDisplay();
                  u8x8.drawString(x_offset, 0, "No disponible");
                  u8x8.drawString(x_offset, 1, "con salida SSR");
                  delay(2000);
                  currentState = ADVANCED_CONFIG;
              } else {
                  currentState = PID_CONFIG;
                  myEnc.write(0);
              }
              break;
          case 1: currentState = ONOFF_CONFIG; myEnc.write(0); break;
          // ➡️ NUEVA FUNCIÓN
          // Nuevo caso para el menú de configuración de alarmas.
          case 2: currentState = ALARM_CONFIG; myEnc.write(0); break;
          case 3: currentState = SSR_ACTIVATION_CONFIRMATION; myEnc.write(0); confirmationCount = 0; break;
          case 4: currentState = MOSFET_ACTIVATION_CONFIRMATION; myEnc.write(0); confirmationCount = 0; break;
          case 5: currentState = MENU; myEnc.write(0); break;
      }
    } else if (longPressDetected) {
      longPressDetected = false;
      redrawMenu = true;
      currentState = MENU;
    }
}

void handleOnOffConfigState(long newPos) {
  static int selectedOptionOnOff = 0;
  static bool redraw = true;
  static bool firstEnter = true;

  if (firstEnter) {
    tempSetAux = setpoint;
    onOffHysteresisAux = onOffHysteresis;
    firstEnter = false;
    redraw = true;
  }
  // ➡️ CAMBIO SOLICITADO
  // Actualizamos el tiempo de interacción del encoder
  if (newPos != oldEncoderPosition) {
    lastInteractionTime = millis();
  }

  if (buttonClicked) {
    buttonClicked = false;
    selectedOptionOnOff = (selectedOptionOnOff + 1) % 2;
    redraw = true;
    myEnc.write(0);
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del botón
    lastInteractionTime = millis();
  } else if (longPressDetected) {
    longPressDetected = false;
    setpoint = tempSetAux;
    onOffHysteresis = onOffHysteresisAux;
    currentControlMode = MODE_ONOFF;
    controlModeAux = MODE_ONOFF;
    saveConfiguration();
    
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 3, "Guardado!");
    buzzerBeep(1, 100, 0);
    delay(1000);

    currentState = ADVANCED_CONFIG;
    myEnc.write(selectedAdvancedOption * 4);
    firstEnter = true;
    return;
  }

  if (newPos != oldEncoderPosition) {
    int delta = newPos - oldEncoderPosition;
    if (millis() - lastEncoderMovementTime < fastTurnThreshold) {
      delta *= 10;
    }

    if (selectedOptionOnOff == 0) tempSetAux += (float)delta;
    if (selectedOptionOnOff == 1) onOffHysteresisAux += (float)delta * 0.1;
    if (tempSetAux < 10.0) tempSetAux = 10.0;
    if (tempSetAux > 250.0) tempSetAux = 250.0;
    if (onOffHysteresisAux < 0.1) onOffHysteresisAux = 0.1;
    if (onOffHysteresisAux > 10.0) onOffHysteresisAux = 10.0;

    oldEncoderPosition = newPos;
    redraw = true;
    lastEncoderMovementTime = millis();
  }

  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Config ON/OFF");
    u8x8.setCursor(x_offset, 2); u8x8.print("SetPoint: ");
    u8x8.setCursor(x_offset + 10, 2); u8x8.print(tempSetAux, 1);
    u8x8.setCursor(x_offset, 3); u8x8.print("Histeresis: "); u8x8.setCursor(x_offset + 10, 3); u8x8.print(onOffHysteresisAux, 1);
    u8x8.drawString(x_offset, 5, "Click: cambiar opc");
    u8x8.drawString(x_offset, 6, "Long: guardar/salir");

    u8x8.setCursor(x_offset - 1, selectedOptionOnOff + 2); u8x8.print(">");
    redraw = false;
  }
}

void handlePIDConfigState(long newPos) {
  static int selectedOptionPid = 0;
  static bool redraw = true;
  static bool firstEnter = true;
  if (firstEnter) {
    kpAux = kp;
    kiAux = ki;
    kdAux = kd;
    firstEnter = false;
    redraw = true;
  }
  // ➡️ CAMBIO SOLICITADO
  // Actualizamos el tiempo de interacción del encoder
  if (newPos != oldEncoderPosition) {
    lastInteractionTime = millis();
  }

  if (buttonClicked) {
    buttonClicked = false;
    selectedOptionPid = (selectedOptionPid + 1) % 3;
    redraw = true;
    myEnc.write(0);
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del botón
    lastInteractionTime = millis();
  } else if (longPressDetected) {
    longPressDetected = false;
    kp = kpAux;
    ki = kiAux;
    kd = kdAux;
    myPID.SetTunings(kp, ki, kd);
    currentControlMode = MODE_PID;
    controlModeAux = MODE_PID;
    saveConfiguration();
    
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 3, "Guardado!");
    buzzerBeep(1, 100, 0);
    delay(1000);
    currentState = ADVANCED_CONFIG;
    myEnc.write(selectedAdvancedOption * 4);
    firstEnter = true;
    return;
  }

  if (newPos != oldEncoderPosition) {
    int delta = newPos - oldEncoderPosition;
    if (millis() - lastEncoderMovementTime < fastTurnThreshold) {
      delta *= 10;
    }
    
    if (selectedOptionPid == 0) kpAux += delta * 0.1;
    if (selectedOptionPid == 1) kiAux += delta * 0.01;
    if (selectedOptionPid == 2) kdAux += delta * 0.01;
    oldEncoderPosition = newPos;
    redraw = true;
    lastEncoderMovementTime = millis();
  }

  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Config PID");
    u8x8.setCursor(x_offset, 2); u8x8.print("Kp:"); u8x8.setCursor(x_offset + 4, 2);
    u8x8.print(kpAux, 2);
    u8x8.setCursor(x_offset, 3); u8x8.print("Ki:"); u8x8.setCursor(x_offset + 4, 3);
    u8x8.print(kiAux, 2);
    u8x8.setCursor(x_offset, 4); u8x8.print("Kd:"); u8x8.setCursor(x_offset + 4, 4); u8x8.print(kdAux, 2);
    u8x8.drawString(x_offset, 6, "Click: cambiar opc");
    u8x8.drawString(x_offset, 7, "Long: guardar/salir");
    u8x8.setCursor(x_offset - 1, selectedOptionPid + 2); u8x8.print(">");
    redraw = false;
  }
}

// ➡️ NUEVA FUNCIÓN
// Función para manejar el nuevo estado de configuración de alarmas.
void handleAlarmConfigState(long newPos) {
    static int selectedOptionAlarm = 0;
    static bool redraw = true;
    static bool firstEnter = true;
    if (firstEnter) {
        highTempAlarmAux = highTempAlarm;
        myEnc.write(highTempAlarmAux);
        firstEnter = false;
        redraw = true;
    }

    newPos = myEnc.read();
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del encoder
    if (newPos != oldEncoderPosition) {
        lastInteractionTime = millis();
    }

    if (newPos != oldEncoderPosition) {
        int delta = newPos - oldEncoderPosition;
        if (millis() - lastEncoderMovementTime < fastTurnThreshold) {
            delta *= 10;
        }

        if (selectedOptionAlarm == 0) highTempAlarmAux += delta;
        if (highTempAlarmAux < setpoint) highTempAlarmAux = setpoint + 5.0; // Mínimo 5°C por encima del setpoint
        if (highTempAlarmAux > 250.0) highTempAlarmAux = 250.0;
        oldEncoderPosition = newPos;
        redraw = true;
        lastEncoderMovementTime = millis();
    }

    if (redraw) {
        u8x8.clearDisplay();
        u8x8.drawString(x_offset, 0, "Config Alarmas");
        u8x8.setCursor(x_offset, 2);
        u8x8.print("Temp alta:"); u8x8.setCursor(x_offset + 11, 2); u8x8.print(highTempAlarmAux, 0); u8x8.print("C");
        u8x8.drawString(x_offset, 4, "Click: guardar");
        u8x8.drawString(x_offset, 6, "Long: salir");
        redraw = false;
    }
    if (buttonClicked) {
      buttonClicked = false;
      highTempAlarm = highTempAlarmAux;
      saveConfiguration();
      u8x8.clearDisplay();
      u8x8.drawString(x_offset, 3, "Guardado!");
      buzzerBeep(1, 100, 0);
      delay(1000);
      // ➡️ CAMBIO SOLICITADO
      // Actualizamos el tiempo de interacción del botón
      lastInteractionTime = millis();
      currentState = ADVANCED_CONFIG;
      myEnc.write(selectedAdvancedOption * 4);
      firstEnter = true;
    } else if (longPressDetected) {
      longPressDetected = false;
      highTempAlarmAux = highTempAlarm;
      currentState = ADVANCED_CONFIG;
      myEnc.write(selectedAdvancedOption * 4);
      firstEnter = true;
    }
}

void handleManualControlState(long newPos) {
  static bool redraw = true;
  static bool firstEnter = true;
  if (firstEnter) {
    manualOutputValueAux = manualOutputValue;
    firstEnter = false;
    redraw = true;
  }
  // ➡️ CAMBIO SOLICITADO
  // Actualizamos el tiempo de interacción del encoder
  if (newPos != oldEncoderPosition) {
    lastInteractionTime = millis();
  }

  if (newPos != oldEncoderPosition) {
    int delta = newPos - oldEncoderPosition;
    if (millis() - lastEncoderMovementTime < fastTurnThreshold) {
      delta *= 10;
    }
    manualOutputValueAux += delta * 10;
    if (manualOutputValueAux < 0) manualOutputValueAux = 0;
    if (manualOutputValueAux > 255) manualOutputValueAux = 255;
    oldEncoderPosition = newPos;
    redraw = true;
    lastEncoderMovementTime = millis();
  }

  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Control Manual");
    u8x8.drawString(x_offset, 2, "Potencia:");
    u8x8.setCursor(x_offset + 10, 2); u8x8.print(manualOutputValueAux);
    u8x8.drawString(x_offset, 4, "Click: guardar");
    u8x8.drawString(x_offset, 6, "Long: salir");
    redraw = false;
  }
  if (buttonClicked) {
    buttonClicked = false;
    manualOutputValue = manualOutputValueAux;
    saveConfiguration();
    currentControlMode = MODE_MANUAL;
    updateOutput((byte)manualOutputValue);
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 3, "Guardado!");
    buzzerBeep(1, 100, 0);
    delay(1000);
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del botón
    lastInteractionTime = millis();
    currentState = MENU;
    myEnc.write(selectedOption * 4);
    firstEnter = true;
  } else if (longPressDetected) {
    longPressDetected = false;
    manualOutputValueAux = manualOutputValue;
    currentState = MENU;
    myEnc.write(selectedOption * 4);
    firstEnter = true;
  }
}

void handleCalibrationState(long newPos) {
  static bool redraw = true;
  static bool firstEnter = true;
  if (firstEnter) {
    calibrationOffsetAux = calibrationOffset;
    myEnc.write(calibrationOffsetAux * 10);
    firstEnter = false;
    redraw = true;
  }
  // ➡️ CAMBIO SOLICITADO
  // Actualizamos el tiempo de interacción del encoder
  newPos = myEnc.read() / 10;
  if (newPos != oldEncoderPosition) {
    lastInteractionTime = millis();
  }

  if (newPos != oldEncoderPosition) {
    calibrationOffsetAux = (float)newPos / 10.0;
    if (calibrationOffsetAux < -10.0) calibrationOffsetAux = -10.0;
    if (calibrationOffsetAux > 10.0) calibrationOffsetAux = 10.0;
    oldEncoderPosition = newPos;
    redraw = true;
  }

  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Calibracion");
    u8x8.drawString(x_offset, 2, "Offset:");
    u8x8.setCursor(x_offset + 8, 2); u8x8.print(calibrationOffsetAux, 1); u8x8.print("C");
    u8x8.drawString(x_offset, 4, "Click: guardar");
    u8x8.drawString(x_offset, 6, "Long: salir");
    redraw = false;
  }
  if (buttonClicked) {
    buttonClicked = false;
    calibrationOffset = calibrationOffsetAux;
    saveConfiguration();
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 3, "Guardado!");
    buzzerBeep(1, 100, 0);
    delay(1000);
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del botón
    lastInteractionTime = millis();
    currentState = MENU;
    myEnc.write(selectedOption * 4);
    firstEnter = true;
  } else if (longPressDetected) {
    longPressDetected = false;
    calibrationOffsetAux = calibrationOffset;
    currentState = MENU;
    myEnc.write(selectedOption * 4);
    firstEnter = true;
  }
}

void handleOutputTypeConfigState() {
}

void handleSSRActivationConfirmationState() {
  static bool redraw = true;
  static bool timerStarted = false;
  if (!timerStarted) {
    lastButtonClickTime = millis();
    timerStarted = true;
  }
  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Confirmar Cambio");
    u8x8.drawString(x_offset, 2, "Cambiar a SSR?");
    u8x8.setCursor(x_offset, 4); u8x8.print("Si");
    u8x8.setCursor(x_offset, 5); u8x8.print("No");
    u8x8.drawString(x_offset - 1, 4 + confirmationCount, ">");
    redraw = false;
  }
  if (buttonClicked) {
    buttonClicked = false;
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del botón
    lastInteractionTime = millis();
    if (confirmationCount == 0) {
      currentOutputType = OUTPUT_SSR;
      u8x8.clearDisplay();
      u8x8.drawString(x_offset, 3, "Cambio Guardado!");
      saveConfiguration();
      delay(1500);
    }
    currentState = ADVANCED_CONFIG;
    myEnc.write(selectedAdvancedOption * 4);
    timerStarted = false;
  } else if (longPressDetected) {
    longPressDetected = false;
    currentState = ADVANCED_CONFIG;
    myEnc.write(selectedAdvancedOption * 4);
    timerStarted = false;
  }

  if (millis() - lastButtonClickTime > CONFIRMATION_TIMEOUT) {
    currentState = ADVANCED_CONFIG;
    timerStarted = false;
  }
  
  long newPos = myEnc.read();
  if (newPos != oldEncoderPosition) {
    confirmationCount += (newPos - oldEncoderPosition) / 4;
    if (confirmationCount < 0) confirmationCount = 1;
    if (confirmationCount > 1) confirmationCount = 0;
    oldEncoderPosition = newPos;
    redraw = true;
  }
}

void handleMosfetActivationConfirmationState() {
  static bool redraw = true;
  static bool timerStarted = false;
  if (!timerStarted) {
    lastButtonClickTime = millis();
    timerStarted = true;
  }
  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Confirmar Cambio");
    u8x8.drawString(x_offset, 2, "Cambiar a MOSFET?");
    u8x8.setCursor(x_offset, 4); u8x8.print("Si");
    u8x8.setCursor(x_offset, 5); u8x8.print("No");
    u8x8.drawString(x_offset - 1, 4 + confirmationCount, ">");
    redraw = false;
  }

  if (buttonClicked) {
    buttonClicked = false;
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del botón
    lastInteractionTime = millis();
    if (confirmationCount == 0) {
      currentOutputType = OUTPUT_MOSFET_PWM;
      u8x8.clearDisplay();
      u8x8.drawString(x_offset, 3, "Cambio Guardado!");
      saveConfiguration();
      delay(1500);
    }
    currentState = ADVANCED_CONFIG;
    myEnc.write(selectedAdvancedOption * 4);
    timerStarted = false;
  } else if (longPressDetected) {
    longPressDetected = false;
    currentState = ADVANCED_CONFIG;
    myEnc.write(selectedAdvancedOption * 4);
    timerStarted = false;
  }

  if (millis() - lastButtonClickTime > CONFIRMATION_TIMEOUT) {
    currentState = ADVANCED_CONFIG;
    timerStarted = false;
  }

  long newPos = myEnc.read();
  if (newPos != oldEncoderPosition) {
    confirmationCount += (newPos - oldEncoderPosition) / 4;
    if (confirmationCount < 0) confirmationCount = 1;
    if (confirmationCount > 1) confirmationCount = 0;
    oldEncoderPosition = newPos;
    redraw = true;
  }
}

void handleErrorState(const char* errorMsg) {
  static bool redraw = true;
  
  if (millis() - errorTimer > SENSOR_TIMEOUT) {
    currentState = CONTROL;
  }
  
  // ➡️ CAMBIO SOLICITADO: Permitir salir del estado de error con el botón
  if (buttonClicked) {
    buttonClicked = false;
    currentState = CONTROL;
    redraw = true; // Reinicia la pantalla para la siguiente vez
    return;
  }
  
  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 2, "ERROR: ");
    u8x8.drawString(x_offset, 3, errorMsg);
    u8x8.drawString(x_offset, 5, "Presione el boton");
    u8x8.drawString(x_offset, 6, "para continuar");
    redraw = false;
  }
}

void displayAbout() {
  static bool redraw = true;
  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Acerca de:");
    u8x8.drawString(x_offset, 2, "Controlador Temp");
    u8x8.drawString(x_offset, 3, "Version 2.0");
    u8x8.drawString(x_offset, 5, "Byte-Electronic");
    redraw = false;
  }
  if (buttonClicked || longPressDetected) {
    buttonClicked = false;
    longPressDetected = false;
    currentState = MENU;
    redraw = true;
  }
}

void displayTestMode() {
  static bool redraw = true;
  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Modo de Prueba");
    u8x8.drawString(x_offset, 2, "Presione para");
    u8x8.drawString(x_offset, 3, "activar/desactivar");
    redraw = false;
  }

  if (buttonClicked) {
    buttonClicked = false;
    outputActive = !outputActive;
  }

  if (longPressDetected) {
    longPressDetected = false;
    currentState = MENU;
    redraw = true;
  }
}

void readSensor() {
  if (millis() - lastSensorReadTime > 250) {
    lastTempForProtection = currentTemp;
    currentTemp = thermocouple.getTemp();
    if (isnan(currentTemp)) {
      u8x8.clearDisplay();
      u8x8.drawString(x_offset, 3, "Error de sensor!");
      delay(2000);
      return;
    }
    currentTemp += calibrationOffset;
    lastSensorReadTime = millis();
  }
}

void readButton() {
  bool currentButtonState = digitalRead(ENCODER_BUTTON);
  if (currentButtonState == LOW && oldButtonState == HIGH) {
    buttonPressStartTime = millis();
  }
  if (currentButtonState == HIGH && oldButtonState == LOW) {
    unsigned long pressDuration = millis() - buttonPressStartTime;
    if (pressDuration >= longPressTime) {
      longPressDetected = true;
    } else {
      buttonClicked = true;
    }
  }
  oldButtonState = currentButtonState;
}

void checkProtections() {
    if (currentTemp > OVERSHOOT_LIMIT) {
        handleErrorState("Overshoot Temp.");
        errorTimer = millis();
    }
    if (highTempAlarmActive && currentTemp > highTempAlarm) {
        highTempAlarmActive = true;
    }
}

void checkHeatingFailure() {
    if (outputActive && (currentControlMode == MODE_PID || currentControlMode == MODE_ONOFF)) {
        if (millis() - lastTempCheckTime > TEMP_RISE_TIMEOUT) {
            if (currentTemp < tempAtLastCheck + 5.0) {
                handleErrorState("Fallo de Calefaccion");
                errorTimer = millis();
            }
            tempAtLastCheck = currentTemp;
            lastTempCheckTime = millis();
        }
    }
}

void updateOutput(byte value) {
  if (currentOutputType == OUTPUT_MOSFET_PWM) {
    analogWrite(MOSFET_PIN, value);
  } else if (currentOutputType == OUTPUT_SSR) {
    if (value > 0) {
      digitalWrite(MOSFET_PIN, HIGH);
    } else {
      digitalWrite(MOSFET_PIN, LOW);
    }
  }
}

void updateControllerOutput() {
    pidSetpoint = setpoint;
    pidInput = currentTemp;

    if (outputActive) {
      if (currentControlMode == MODE_PID) {
        myPID.Compute();
      } else if (currentControlMode == MODE_ONOFF) {
        if (currentTemp < setpoint - onOffHysteresis) {
          pidOutput = 255;
        } else if (currentTemp > setpoint + onOffHysteresis) {
          pidOutput = 0;
        }
      } else if (currentControlMode == MODE_MANUAL) {
        pidOutput = manualOutputValue;
      }
    } else {
      pidOutput = 0;
    }

    updateOutput((byte)pidOutput);
}

void buzzerBeep(int count, int duration, int pause) {
  if (buzzerEnabled) {
    for(int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration);
        digitalWrite(BUZZER_PIN, LOW);
        delay(pause);
    }
  }
}

void checkSerialConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 1000) {
    if (Serial.available() > 0) {
      serialConnected = true;
    } else {
      serialConnected = false;
    }
    lastCheck = millis();
  }
}

void sendSerialData() {
    if (serialConnected) {
        Serial.print("T:");
        Serial.print(currentTemp, 1);
        Serial.print("S:");
        Serial.print(setpoint, 1);
        Serial.print("O:");
        Serial.println((int)pidOutput);
    }
}

void handleResetConfirmationState() {
  static bool redraw = true;
  static bool timerStarted = false;
  if (!timerStarted) {
    lastButtonClickTime = millis();
    timerStarted = true;
  }
  if (redraw) {
    u8x8.clearDisplay();
    u8x8.drawString(x_offset, 0, "Confirmar Reinicio");
    u8x8.drawString(x_offset, 2, "Valores de Fabrica?");
    u8x8.setCursor(x_offset, 4); u8x8.print("Si");
    u8x8.setCursor(x_offset, 5); u8x8.print("No");
    u8x8.drawString(x_offset - 1, 4 + confirmationCount, ">");
    redraw = false;
  }

  if (buttonClicked) {
    buttonClicked = false;
    // ➡️ CAMBIO SOLICITADO
    // Actualizamos el tiempo de interacción del botón
    lastInteractionTime = millis();
    if (confirmationCount == 0) {
      resetToFactorySettings();
      u8x8.clearDisplay();
      u8x8.drawString(x_offset, 3, "Reiniciado!");
      delay(1500);
    }
    currentState = MENU;
    myEnc.write(selectedOption * 4);
    timerStarted = false;
  } else if (longPressDetected) {
    longPressDetected = false;
    currentState = MENU;
    myEnc.write(selectedOption * 4);
    timerStarted = false;
  }

  if (millis() - lastButtonClickTime > CONFIRMATION_TIMEOUT) {
    currentState = MENU;
    timerStarted = false;
  }
  
  long newPos = myEnc.read();
  if (newPos != oldEncoderPosition) {
    confirmationCount += (newPos - oldEncoderPosition) / 4;
    if (confirmationCount < 0) confirmationCount = 1;
    if (confirmationCount > 1) confirmationCount = 0;
    oldEncoderPosition = newPos;
    redraw = true;
  }
}

// ➡️ NUEVA FUNCIÓN
// Nuevo prototipo para la función de alarmas.
void checkAlarms() {
    // Si la temperatura actual supera la alarma, activamos un aviso visual y sonoro
    if (currentTemp >= highTempAlarm && highTempAlarmActive == false && buzzerEnabled) {
        buzzerBeep(3, 100, 100);
        highTempAlarmActive = true;
    }

    if (highTempAlarmActive && buttonClicked) {
        highTempAlarmActive = false;
        buttonClicked = false;
        u8x8.clearDisplay(); // Limpiamos la pantalla para volver al estado normal
    }
}