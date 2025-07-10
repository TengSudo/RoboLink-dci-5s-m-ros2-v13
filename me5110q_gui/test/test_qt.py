import sys
from PyQt5 import QtWidgets
from untitled import Ui_ME5110Q

app = QtWidgets.QApplication(sys.argv)
window = QtWidgets.QMainWindow()
ui = Ui_ME5110Q()
ui.setupUi(window)
window.show()
sys.exit(app.exec_())