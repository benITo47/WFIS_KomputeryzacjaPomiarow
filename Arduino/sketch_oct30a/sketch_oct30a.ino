/*
--------------------------------------------------------------------------------
 SCHEMAT POŁĄCZEŃ SPRZĘTOWYCH (ARDUINO <-> KOMPONENTY)
--------------------------------------------------------------------------------

### Arduino <-> Czujnik Wilgotności (Sensor)
- **Połączenia:**
  - `Arduino 3.3V` -> `VCC` czujnika
  - `Arduino GND`  -> `GND` czujnika
  - `Arduino A0`   -> `AOUT` czujnika

### Arduino <-> Moduł Przekaźnika (Relay Module)
- **Połączenia:**
  - `Arduino 5V`   -> `VCC` przekaźnika
  - `Arduino GND`  -> `GND` przekaźnika
  - `Arduino D7`   -> `IN1` przekaźnika (steruje kanałem **K1**)

### Układ Zasilania Pompy (Pump Power Circuit)
- **Połączenia:**
  - `Zasilacz (+)` -> `Pompa (+)`
  - `Zasilacz (-)` -> `Przekaźnik COM` (na kanale **K1**)
  - `Pompa (-)`    -> `Przekaźnik NO` (na kanale **K1**)

--------------------------------------------------------------------------------
*/

#include <PID_v1.h> // Dołączamy bibliotekę PID

/*
 * - Sterowanie parametrami PID i Setpoint na żywo przez port szeregowy.
 * - Wysyła dane w formacie CSV (wilgotnosc,cel,wyjscie_pid) do wizualizacji w Pythonie.
 */

// --- Konfiguracja Pinów ---
const int SENSOR_PIN_AOUT = A0;  // Wejście analogowe dla czujnika wilgotności
const int RELAY_PIN = 7;         // Pin cyfrowy do sterowania przekaźnikiem (IN1)

// --- Kalibracja Czujnika ---
const int ADC_MIN_HUMIDITY = 466; // Wartość dla SUCHEGO czujnika
const int ADC_MAX_HUMIDITY = 315; // Wartość dla MOKREGO czujnika

// --- Ustawienia Kontrolera PID ---
double Setpoint = 70.0; 
double Input, Output;
double Kp = 5, Ki = 0.1, Kd = 1; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Ustawienia Czasowe dla Pompy (PWM) ---
unsigned long windowSize = 10000; 
unsigned long windowStartTime;

// --- Bufor do komunikacji szeregowej ---
String serialBuffer = "";

void setup() {
  Serial.begin(9600);
  serialBuffer.reserve(32); // Rezerwujemy pamięć dla komend

  pinMode(SENSOR_PIN_AOUT, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  windowStartTime = millis();
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  handleSerialCommands();
  
  int sensorValue = analogRead(SENSOR_PIN_AOUT);
  int humidity = map(sensorValue, ADC_MIN_HUMIDITY, ADC_MAX_HUMIDITY, 0, 100);
  humidity = constrain(humidity, 0, 100);
  Input = humidity;

  myPID.Compute();

  unsigned long now = millis();
  if (now - windowStartTime > windowSize) {
    windowStartTime += windowSize;
  }
  if (millis() - windowStartTime < Output) {
    digitalWrite(RELAY_PIN, LOW);
  } else {
    digitalWrite(RELAY_PIN, HIGH);
  }

  Serial.print(Input);
  Serial.print(",");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.println(Output);
  
  delay(200);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      parseCommand(serialBuffer);
      serialBuffer = ""; // Czyścimy bufor
    } else {
      serialBuffer += receivedChar;
    }
  }
}

void parseCommand(String command) {
  char commandType = command.charAt(0);
  float value = command.substring(2).toFloat();

  switch (commandType) {
    case 'P':
      Kp = value;
      break;
    case 'I':
      Ki = value;
      break;
    case 'D':
      Kd = value;
      break;
    case 'S':
      Setpoint = value;
      return; // Dla Setpoint nie trzeba aktualizować nastaw PID
  }
  myPID.SetTunings(Kp, Ki, Kd);
}
