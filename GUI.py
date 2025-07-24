import sys
import PySide6.QtWidgets as Qw
import PySide6.QtCore as Qc

def run_gui(shared_data):
  class MainWindow(Qw.QMainWindow):
    def __init__(self):
      super().__init__()
      self.setWindowTitle('PID Tuning GUI')

      layout = Qw.QVBoxLayout()
      self.central_widget = Qw.QWidget()
      self.central_widget.setLayout(layout)
      self.setCentralWidget(self.central_widget)

      # スライダーを作成
      self.kp_slider = self.create_slider(
          "KP Angle", shared_data, "kp", 200)
      self.ki_slider = self.create_slider(
          "KI Angle", shared_data, "ki", 100)
      self.kd_slider = self.create_slider(
          "KD Angle", shared_data, "kd", 100)

      self.kp_speed_slider = self.create_slider(
          "KP Speed", shared_data, "kp_speed", 200)
      self.ki_speed_slider = self.create_slider(
          "KI Speed", shared_data, "ki_speed", 100)
      self.kd_speed_slider = self.create_slider(
          "KD Speed", shared_data, "kd_speed", 100)

    def create_slider(self, label, shared_data, key, max_value):
      layout = Qw.QVBoxLayout()
      self.central_widget.setLayout(layout)
      slider = Qw.QSlider(Qc.Qt.Orientation.Horizontal)
      slider.setMinimum(0)
      slider.setMaximum(max_value)
      slider.setValue(int(shared_data[key] * 100))
      slider.valueChanged.connect(
          lambda value: shared_data.update({key: value / 100.0}))
      layout.addWidget(Qw.QLabel(label))
      layout.addWidget(slider)
      return slider

  app = Qw.QApplication(sys.argv)
  main_window = MainWindow()
  main_window.show()
  app.exec()
