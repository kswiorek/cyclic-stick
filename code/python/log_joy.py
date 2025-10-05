import sys
import serial
import threading
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

def decodeMSBY(byte1):
    ret = byte1 ^ 0x89
    ret = (ret & 0x80) | ((ret & 0x0F) << 3)
    return ret << 4

def decodeMSBX(byte1):
    ret = byte1 ^ 0x8E
    ret = (ret & 0x80) | ((ret & 0x0F) << 3)
    return ret << 4

def decodeLSBY(byte0):
    ret = (byte0 ^ 0xD8)
    ret = (ret & 0xFE) >> 1      
    return ret

def decodeLSBX(byte0):
    ret = (byte0 ^ 0xC8)
    ret = (ret & 0xFE) >> 1    
    return ret

class JoystickMonitor(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Joystick Monitor")
        self.resize(600, 500)

        # Layouts
        layout = QtWidgets.QVBoxLayout(self)
        self.button_labels = [QtWidgets.QLabel("0") for _ in range(40)]
        button_grid = QtWidgets.QGridLayout()
        for i, lbl in enumerate(self.button_labels):
            button_grid.addWidget(lbl, i // 8, i % 8)
        layout.addLayout(button_grid)

        self.x_label = QtWidgets.QLabel("X: 0")
        self.y_label = QtWidgets.QLabel("Y: 0")
        layout.addWidget(self.x_label)
        layout.addWidget(self.y_label)

        # XY scatter plot
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setYRange(0, 2**12)
        self.plot_widget.setXRange(0, 2**12)
        self.plot_widget.setLabel('left', 'Y')
        self.plot_widget.setLabel('bottom', 'X')
        self.scatter = pg.ScatterPlotItem(size=15, brush=pg.mkBrush(255, 0, 0, 120))
        self.plot_widget.addItem(self.scatter)
        layout.addWidget(self.plot_widget)

        # Time series plot for X and Y
        self.time_plot = pg.PlotWidget()
        self.time_plot.setLabel('left', 'Value')
        self.time_plot.setLabel('bottom', 'Sample')
        self.time_plot.addLegend()
        self.x_curve = self.time_plot.plot(pen='r', name='X')
        self.y_curve = self.time_plot.plot(pen='b', name='Y')
        layout.addWidget(self.time_plot)

        self.x = 0
        self.y = 0
        self.x_history = []
        self.y_history = []
        self.max_history = 500  # Number of samples to show

        # Serial thread
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

        # Timer to update plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

    def read_serial(self):
        try:
            ser = serial.Serial('COM5', 115200, timeout=1)
        except Exception as e:
            print("Serial error:", e)
            return
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            try:
                hex_part = line.split('DAC1')[0].strip()
                byte_strs = hex_part.split()
                if len(byte_strs) < 38:
                    continue  # Not enough data
                data = [int(b, 16) for b in byte_strs]

                # Button states: bytes 34-38 (index 33-37)
                btn_bits = ""
                for i in range(33, 38):
                    btn_bits += f"{data[i]:08b}"
                for i, lbl in enumerate(self.button_labels):
                    if i < len(btn_bits):
                        lbl.setText(btn_bits[i])
                    else:
                        lbl.setText("0")

                # Use decode functions for X and Y
                # Example: X uses bytes 21 (LSB) and 22 (MSB), Y uses 23 (LSB) and 24 (MSB)
                lsb_x = decodeLSBX(data[21])
                msb_x = decodeMSBX(data[22])
                self.x = msb_x | lsb_x

                lsb_y = decodeLSBY(data[23])
                msb_y = decodeMSBY(data[24])
                self.y = msb_y | lsb_y

                # Append to history
                self.x_history.append(self.x)
                self.y_history.append(self.y)
                if len(self.x_history) > self.max_history:
                    self.x_history.pop(0)
                    self.y_history.pop(0)

                self.x_label.setText(f"X: {self.x}")
                self.y_label.setText(f"Y: {self.y}")
            except Exception as e:
                print("Parse error:", e)

    def update_plot(self):
        self.scatter.setData([self.x], [self.y])
        if self.x_history and self.y_history:
            self.x_curve.setData(self.x_history, pen='r')
            self.y_curve.setData(self.y_history, pen='b')

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = JoystickMonitor()
    win.show()
    sys.exit(app.exec_())