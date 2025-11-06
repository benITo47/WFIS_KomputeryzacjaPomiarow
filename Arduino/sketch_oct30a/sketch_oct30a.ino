/*
--------------------------------------------------------------------------------
 SCHEMAT POŁĄCZEŃ SPRZĘTOWYCH (ARDUINO <-> KOMPONENTY)
--------------------------------------------------------------------------------

Ten skrypt steruje systemem o poniższej konfiguracji:

1. Arduino <-> Czujnik Wilgotności (Sensor)
   ------------------------------------------
   - Cel: Odczyt poziomu wilgotności gleby.
   - Połączenia:
     - Arduino 3.3V -> VCC czujnika (Zasilanie czujnika)
     - Arduino GND  -> GND czujnika (Wspólna masa)
     - Arduino A0   -> AOUT czujnika (Sygnał analogowy z wilgotnością)

2. Arduino <-> Moduł Przekaźnika (Relay Module)
   ---------------------------------------------
   - Cel: Sterowanie włączaniem/wyłączaniem pompy.
   - Połączenia:
     - Arduino 5V   -> VCC przekaźnika (Zasilanie logiki przekaźnika)
     - Arduino GND  -> GND przekaźnika (Wspólna masa)
     - Arduino D7   -> IN1 przekaźnika (Sygnał sterujący dla pierwszego kanału, czyli K1)

3. Układ Zasilania Pompy (Pump Power Circuit)
   ------------------------------------------
   - Cel: Bezpieczne zasilanie pompy z użyciem zewnętrznego źródła, 
     przekaźnik działa jako elektroniczny przełącznik.
   - Komponenty: Pompa, zewnętrzne źródło zasilania (np. zasilacz 5V/12V).
   - Połączenia:
     - Zasilacz (+) -> Pompa (+)         [Bezpośrednie połączenie plusów]
     - Zasilacz (-) -> Przekaźnik COM    [Minus zasilania do wejścia wspólnego przekaźnika K1]
     - Pompa (-)    -> Przekaźnik NO     [Minus pompy do wyjścia "Normalnie Otwartego" przekaźnika K1]

--------------------------------------------------------------------------------
*/

#include <PID_v1.h> // Dołączamy bibliotekę PID

/*
 * Zaawansowany program do automatycznego nawadniania z użyciem kontrolera PID.
 * Wysyła dane w formacie CSV (wilgotnosc,cel,wyjscie_pid) do wizualizacji w Pythonie.
 */

// --- Konfiguracja Pinów ---
const int SENSOR_PIN_AOUT = A0;  // Wejście analogowe dla czujnika wilgotności
const int RELAY_PIN = 7;         // Pin cyfrowy do sterowania przekaźnikiem (IN1)

// --- Kalibracja Czujnika (Twoje wartości z testów) ---
const int ADC_MIN_HUMIDITY = 466; // Wartość dla SUCHEGO czujnika
const int ADC_MAX_HUMIDITY = 315; // Wartość dla MOKREGO czujnika

// --- Ustawienia Kontrolera PID ---
double Setpoint = 70.0; 
double Input, Output;
double Kp = 5, Ki = 0.1, Kd = 1; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Ustawienia Czasowe dla Pompy (PWM) ---
unsigned long windowSize = 10000; // Długość okna czasowego w milisekundach (np. 10 sekund)
unsigned long windowStartTime;

void setup() {
  Serial.begin(9600);
  Serial.println("Start programu - Nawadnianie z kontrolerem PID");

  pinMode(SENSOR_PIN_AOUT, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  windowStartTime = millis();
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // 1. Odczytaj i przelicz wilgotność
  int sensorValue = analogRead(SENSOR_PIN_AOUT);
  int humidity = map(sensorValue, ADC_MIN_HUMIDITY, ADC_MAX_HUMIDITY, 0, 100);
  humidity = constrain(humidity, 0, 100);
  Input = humidity;

  // 2. Uruchom obliczenia PID
  myPID.Compute();

  // 3. Steruj pompą na podstawie wyniku PID
  unsigned long now = millis();
  if (now - windowStartTime > windowSize) {
    windowStartTime += windowSize;
  }
  if (millis() - windowStartTime < Output) {
    digitalWrite(RELAY_PIN, LOW);
  } else {
    digitalWrite(RELAY_PIN, HIGH);
  }

  // 4. Wysyłamy dane w formacie CSV dla Pythona: wilgotnosc,cel,wyjscie_pid
  Serial.print(Input);
  Serial.print(",");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.println(Output);
  
  delay(200); // Czekamy chwilę, aby dane były wysyłane w rozsądnym tempie
}
