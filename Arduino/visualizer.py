import csv
import datetime
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
from matplotlib.widgets import Button, Slider


class PIDVisualizerGUI:
    """
    Interfejs graficzny do wizualizacji i sterowania regulatorem PID
    dla systemu automatycznego nawadniania z Arduino.
    """

    def __init__(self):
        self.ser = None
        self.is_saving = False
        self.csv_writer = None
        self.csv_file = None
        self.pid_window_size = 10000

        self.time_data = deque(maxlen=100)
        self.humidity_data = deque(maxlen=100)
        self.setpoint_data = deque(maxlen=100)
        self.output_data = deque(maxlen=100)
        self.time_counter = 0

        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(14, 9), constrained_layout=True)
        self.fig.suptitle(
            "Live PID Controller - System Nawadniania", fontsize=18, fontweight="bold"
        )

        gs = self.fig.add_gridspec(2, 1, height_ratios=[3, 1.2])

        self.ax1 = self.fig.add_subplot(gs[0, :])
        self.ax2 = self.ax1.twinx()

        controls_gs = gs[1, :].subgridspec(
            2, 4, wspace=0.4, hspace=0.6, width_ratios=[1, 1, 1, 0.5]
        )

        ax_s = self.fig.add_subplot(controls_gs[0, 0])
        ax_p = self.fig.add_subplot(controls_gs[0, 1])
        ax_i = self.fig.add_subplot(controls_gs[1, 1])
        ax_d = self.fig.add_subplot(controls_gs[0, 2])
        ax_save = self.fig.add_subplot(controls_gs[:, 3])

        self.create_widgets(ax_s, ax_p, ax_i, ax_d, ax_save)
        self.configure_plot_appearance()

    def create_widgets(self, ax_s, ax_p, ax_i, ax_d, ax_save):
        """Tworzy i konfiguruje suwaki oraz przyciski."""
        self.s_slider = Slider(
            ax=ax_s,
            label="Setpoint [%]",
            valmin=0,
            valmax=100,
            valinit=70,
            color="#44FF33",
        )
        self.p_slider = Slider(
            ax=ax_p, label="Kp", valmin=0, valmax=20, valinit=5, color="#FFC300"
        )
        self.i_slider = Slider(
            ax=ax_i, label="Ki", valmin=0, valmax=5, valinit=0.1, color="#FF5733"
        )
        self.d_slider = Slider(
            ax=ax_d, label="Kd", valmin=0, valmax=10, valinit=1, color="#C70039"
        )

        self.save_button = Button(
            ax_save, "Zapisz CSV", color="#4a4a4a", hovercolor="#6d6d6d"
        )

        for slider in [self.s_slider, self.p_slider, self.i_slider, self.d_slider]:
            slider.label.set_color("white")
            slider.valtext.set_color("white")
        self.save_button.label.set_color("white")

        self.s_slider.on_changed(self.update_setpoint)
        self.p_slider.on_changed(self.update_kp)
        self.i_slider.on_changed(self.update_ki)
        self.d_slider.on_changed(self.update_kd)
        self.save_button.on_clicked(self.toggle_save)

    def configure_plot_appearance(self):
        """Konfiguruje stałe elementy estetyczne wykresu."""
        self.ax1.set_xlabel("Czas (kroki pomiarowe)", fontsize=12)
        self.ax1.set_ylabel("Wilgotność [%]", color="#33AFFF", fontsize=12)
        self.ax2.set_ylabel("Czas pracy pompy [ms]", color="#FF5733", fontsize=12)
        self.ax1.tick_params(axis="y", labelcolor="#33AFFF")
        self.ax2.tick_params(axis="y", labelcolor="#FF5733")
        self.ax1.tick_params(axis="x", colors="white")
        self.ax1.grid(True, linestyle="--", alpha=0.3)
        self.ax1.set_ylim(0, 110)
        self.ax2.set_ylim(0, self.pid_window_size * 1.1)

    def get_serial_port(self):
        """Wyświetla listę portów i prosi użytkownika o wybór."""
        ports = serial.tools.list_ports.comports()
        print("\nDostępne porty szeregowe:")
        if not ports:
            print(
                "BŁĄD: Nie znaleziono żadnych portów. Upewnij się, że Arduino jest podłączone."
            )
            exit()
        for i, port in enumerate(ports):
            print(f"  {i + 1}: {port.device} - {port.description}")
        while True:
            try:
                choice = int(input("\nWybierz numer portu: "))
                if 1 <= choice <= len(ports):
                    return ports[choice - 1].device
            except (ValueError, IndexError):
                print("Proszę wpisać poprawny numer.")

    def _send_command(self, cmd, val):
        """Wysyła pojedynczą komendę do Arduino."""
        if self.ser and self.ser.is_open:
            try:
                line = f"{cmd}:{val:.2f}\n"
                self.ser.write(line.encode("utf-8"))
            except serial.SerialException as e:
                print(f"Błąd zapisu do portu: {e}")

    def update_setpoint(self, val):
        self._send_command("S", val)

    def update_kp(self, val):
        self._send_command("P", val)

    def update_ki(self, val):
        self._send_command("I", val)

    def update_kd(self, val):
        self._send_command("D", val)

    def sync_initial_values(self):
        """Wysyła początkowe wartości suwaków do Arduino po nawiązaniu połączenia."""
        print("Synchronizuję nastawy PID z Arduino...")
        self.update_setpoint(self.s_slider.val)
        self.update_kp(self.p_slider.val)
        self.update_ki(self.i_slider.val)
        self.update_kd(self.d_slider.val)

    def toggle_save(self, event):
        """Obsługuje kliknięcie przycisku zapisu do CSV."""
        if self.is_saving:
            self.is_saving = False
            self.save_button.color = "#4a4a4a"
            self.save_button.label.set_text("Zapisz CSV")
            if self.csv_file:
                self.csv_file.close()
                print(f"\nZakończono zapis do pliku: {self.csv_filename}")
        else:
            self.is_saving = True
            self.save_button.color = "green"
            self.save_button.label.set_text("Zapisywanie...")
            self.csv_filename = (
                f"pid_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            self.csv_file = open(self.csv_filename, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["czas", "wilgotnosc", "setpoint", "pid_output"])
            print(f"\nRozpoczęto zapis do pliku: {self.csv_filename}")

    def animate(self, i):
        """Główna funkcja animacji, odczytuje dane i aktualizuje wykres."""
        try:
            if not self.ser or not self.ser.is_open:
                return

            self.ser.reset_input_buffer()
            line = self.ser.readline().decode("utf-8").strip()

            if not line:
                return

            data = line.split(",")
            if len(data) == 3:
                humidity, setpoint, pid_output = map(float, data)

                self.time_data.append(self.time_counter)
                self.humidity_data.append(humidity)
                self.setpoint_data.append(setpoint)
                self.output_data.append(pid_output / self.pid_window_size * 100)
                self.time_counter += 1

                if self.is_saving:
                    self.csv_writer.writerow(
                        [
                            datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
                            humidity,
                            setpoint,
                            pid_output,
                        ]
                    )
        except (ValueError, UnicodeDecodeError, serial.SerialException) as e:
            return

        self.ax1.clear()
        self.ax2.clear()
        self.configure_plot_appearance()

        (line1,) = self.ax1.plot(
            self.time_data,
            self.humidity_data,
            label="Wilgotność",
            color="#33AFFF",
            linewidth=2.5,
        )
        (line2,) = self.ax1.plot(
            self.time_data,
            self.setpoint_data,
            label="Cel (Setpoint)",
            color="#44FF33",
            linewidth=2,
            linestyle=":",
        )
        fill1 = self.ax2.fill_between(
            self.time_data,
            self.output_data,
            label="Moc Pompy [%]",
            color="#FF5733",
            alpha=0.5,
        )

        # Aktualizacja legendy
        self.ax1.legend(
            handles=[line1, line2, fill1],
            loc="upper left",
            frameon=True,
            facecolor="#2a2a2a",
            edgecolor="none",
        )

    def run(self):
        """Główna pętla programu."""
        port = self.get_serial_port()
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
            print(f"Nawiązano połączenie z {port}. Czekam na gotowość Arduino...")
            time.sleep(2)  # Kluczowe opóźnienie na reset Arduino
            self.sync_initial_values()
            print("Arduino gotowe. Uruchamiam wizualizację.")
        except serial.SerialException as e:
            print(
                f"KRYTYCZNY BŁĄD: {e}. Sprawdź połączenie i zamknij Serial Monitor w Arduino IDE."
            )
            exit()

        ani = animation.FuncAnimation(self.fig, self.animate, interval=200, blit=False)
        plt.show()

        # Sprzątanie po zamknięciu okna
        if self.is_saving:
            self.csv_file.close()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Zamknięto port szeregowy.")


if __name__ == "__main__":
    gui = PIDVisualizerGUI()
    gui.run()
