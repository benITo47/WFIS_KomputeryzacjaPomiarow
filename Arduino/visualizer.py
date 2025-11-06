'''
--------------------------------------------------------------------------------
 SCHEMAT POŁĄCZEŃ SPRZĘTOWYCH (ARDUINO <-> KOMPONENTY)
--------------------------------------------------------------------------------

Ten skrypt wizualizuje dane z poniższego systemu:

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
     - Arduino D7   -> IN1 przekaźnika (Sygnał sterujący dla pierwszego kanału)

3. Układ Zasilania Pompy (Pump Power Circuit)
   ------------------------------------------
   - Cel: Bezpieczne zasilanie pompy z użyciem zewnętrznego źródła, 
     przekaźnik działa jako elektroniczny przełącznik.
   - Komponenty: Pompa, zewnętrzne źródło zasilania (np. zasilacz 5V/12V).
   - Połączenia:
     - Zasilacz (+) -> Pompa (+)         [Bezpośrednie połączenie plusów]
     - Zasilacz (-) -> Przekaźnik COM    [Minus zasilania do wejścia wspólnego przekaźnika]
     - Pompa (-)    -> Przekaźnik NO     [Minus pompy do wyjścia "Normalnie Otwartego"]

--------------------------------------------------------------------------------
'''
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Konfiguracja ---
HISTORY_SIZE = 100  # Ile punktów danych przechowywać i wyświetlać na wykresie

# --- Funkcje pomocnicze ---
def get_serial_port():
    """Listuje dostępne porty i prosi użytkownika o wybór."""
    ports = serial.tools.list_ports.comports()
    print("\nDostępne porty szeregowe:")
    if not ports:
        print("BŁĄD: Nie znaleziono żadnych portów szeregowych. Upewnij się, że Arduino jest podłączone.")
        exit()

    for i, port in enumerate(ports):
        print(f"  {i + 1}: {port.device} - {port.description}")

    while True:
        try:
            choice = int(input("\nWybierz numer portu, do którego podłączone jest Arduino: "))
            if 1 <= choice <= len(ports):
                return ports[choice - 1].device
            else:
                print("Nieprawidłowy numer, spróbuj ponownie.")
        except ValueError:
            print("Proszę wpisać numer.")

# --- Główna klasa wizualizatora ---
class RealTimePlotter:
    def __init__(self, port, baud_rate=9600):
        """Inicjalizuje połączenie szeregowe i struktury danych."""
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)
            print(f"Połączono z {port} przy {baud_rate} baud.")
        except serial.SerialException as e:
            print(f"BŁĄD: Nie można otworzyć portu {port}. {e}")
            print("Upewnij się, że Serial Monitor w Arduino IDE jest zamknięty i spróbuj ponownie.")
            exit()

        self.time_data = deque(maxlen=HISTORY_SIZE)
        self.humidity_data = deque(maxlen=HISTORY_SIZE)
        self.setpoint_data = deque(maxlen=HISTORY_SIZE)
        self.output_data = deque(maxlen=HISTORY_SIZE)
        self.time_counter = 0
        # Potrzebujemy znać windowSize z Arduino, żeby dobrze skalować oś Y
        self.pid_window_size = 10000 

    def animate(self, i):
        """Główna funkcja animacji, wywoływana cyklicznie przez Matplotlib."""
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            data = line.split(',')
            if len(data) == 3:
                humidity, setpoint, pid_output = map(float, data)
                self.time_data.append(self.time_counter)
                self.humidity_data.append(humidity)
                self.setpoint_data.append(setpoint)
                self.output_data.append(pid_output)
                self.time_counter += 1

        except (ValueError, UnicodeDecodeError):
            return
        except serial.SerialException as e:
            print(f"Błąd odczytu z portu szeregowego: {e}")
            return

        # Czyszczenie osi
        self.ax1.clear()
        self.ax2.clear()

        # Rysowanie danych
        self.ax1.plot(self.time_data, self.humidity_data, label='Wilgotność (%)', color='#33AFFF', linewidth=2.5, marker='o', markersize=3, markevery=5)
        self.ax1.plot(self.time_data, self.setpoint_data, label='Cel (Setpoint)', color='#44FF33', linewidth=2, linestyle='--')
        self.ax2.fill_between(self.time_data, self.output_data, label='Moc Pompy (ms)', color='#FF5733', alpha=0.4)
        self.ax2.plot(self.time_data, self.output_data, color='#FF5733', linewidth=1, linestyle=':')

        # Ustawienia osi i legendy
        self.ax1.set_xlabel('Czas (kroki pomiarowe)', fontsize=12)
        self.ax1.set_ylabel('Wilgotność (%)', color='#33AFFF', fontsize=12)
        self.ax1.tick_params(axis='y', labelcolor='#33AFFF')
        self.ax1.set_ylim(min(list(self.humidity_data) + list(self.setpoint_data)) - 10, max(list(self.humidity_data) + list(self.setpoint_data)) + 10)

        self.ax2.set_ylabel('Czas pracy pompy (ms)', color='#FF5733', fontsize=12)
        self.ax2.tick_params(axis='y', labelcolor='#FF5733')
        self.ax2.set_ylim(0, self.pid_window_size * 1.1)

        # Łączenie legend z obu osi w jedną
        lines, labels = self.ax1.get_legend_handles_labels()
        lines2, labels2 = self.ax2.get_legend_handles_labels()
        self.ax2.legend(lines + lines2, labels + labels2, loc='upper left')

        self.fig.suptitle('Live PID Controller - System Nawadniania', fontsize=16, fontweight='bold')
        self.ax1.grid(True, linestyle='--', alpha=0.2)

    def run(self):
        """Uruchamia pętlę animacji."""
        plt.style.use('dark_background')
        self.fig, self.ax1 = plt.subplots()
        self.ax2 = self.ax1.twinx()
        self.fig.tight_layout(pad=3.0)
        ani = animation.FuncAnimation(self.fig, self.animate, interval=150)
        plt.show()

if __name__ == '__main__':
    serial_port = get_serial_port()
    plotter = RealTimePlotter(serial_port)
    plotter.run()
