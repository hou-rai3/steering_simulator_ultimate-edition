import sys
import PySide6.QtWidgets as Qw
import PySide6.QtCore as Qc

def run_gui(shared_data):
  class MainWindow(Qw.QMainWindow):
    def __init__(self):
      super().__init__()
      self.setWindowTitle('Robot Control PID Tuning')
      self.setGeometry(100, 100, 400, 300)

      layout = Qw.QVBoxLayout()
      self.central_widget = Qw.QWidget()
      self.central_widget.setLayout(layout)
      self.setCentralWidget(self.central_widget)

      # 説明ラベル
      info_label = Qw.QLabel(
          "Robot Control Parameters\nUse WASD or Arrow Keys to control the robot")
      layout.addWidget(info_label)

      # スライダーを作成
      self.kp_slider = self.create_slider(
          "KP (Proportional)", shared_data, "kp", 300, layout)
      self.ki_slider = self.create_slider(
          "KI (Integral)", shared_data, "ki", 100, layout)
      self.kd_slider = self.create_slider(
          "KD (Derivative)", shared_data, "kd", 100, layout)

      layout.addWidget(Qw.QLabel("Speed Control:"))
      self.kp_speed_slider = self.create_slider(
          "KP Speed", shared_data, "kp_speed", 200, layout)
      self.ki_speed_slider = self.create_slider(
          "KI Speed", shared_data, "ki_speed", 100, layout)
      self.kd_speed_slider = self.create_slider(
          "KD Speed", shared_data, "kd_speed", 100, layout)

    def create_slider(self, label, shared_data, key, max_value, layout):
      slider_layout = Qw.QHBoxLayout()

      label_widget = Qw.QLabel(label + ":")
      label_widget.setMinimumWidth(120)

      slider = Qw.QSlider(Qc.Qt.Orientation.Horizontal)
      slider.setMinimum(0)
      slider.setMaximum(max_value)
      slider.setValue(int(shared_data[key] * 100))

      value_label = Qw.QLabel(f"{shared_data[key]:.2f}")
      value_label.setMinimumWidth(50)

      def update_value(value):
        new_val = value / 100.0
        shared_data.update({key: new_val})
        value_label.setText(f"{new_val:.2f}")

      slider.valueChanged.connect(update_value)

      slider_layout.addWidget(label_widget)
      slider_layout.addWidget(slider)
      slider_layout.addWidget(value_label)

      layout.addLayout(slider_layout)
      return slider

  app = Qw.QApplication(sys.argv)
  main_window = MainWindow()
  main_window.show()
  app.exec()
