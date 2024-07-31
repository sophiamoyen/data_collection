import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QFileDialog, QLabel, QMainWindow, QWidget, QVBoxLayout
from PyQt5.uic import loadUi
from PyQt5.QtGui import QPixmap

class MainWindow(QDialog):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.title = "Demo Recorder"
        self.setWindowTitle(self.title)

        loadUi("demo_collection.ui",self)
        self.browse.clicked.connect(self.browsefiles)

        label = QLabel(self)
        pixmap = QPixmap('cat.png')
        label.setPixmap(pixmap)
        self.resize(pixmap.width(), pixmap.height())

    def browsefiles(self):
        folder_save=QFileDialog.getExistingDirectory(None, 'Select a folder:', 'bla', QFileDialog.ShowDirsOnly)
        self.lineEdit.setText(folder_save)

app=QApplication(sys.argv)
mainwindow=MainWindow()

widget=QtWidgets.QStackedWidget()
widget.addWidget(mainwindow)
widget.show()
sys.exit(app.exec_())