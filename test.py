import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QDialog, QApplication, QFileDialog
from PyQt5.uic import loadUi

class MainWindow(QDialog):
    def __init__(self):
        super(MainWindow,self).__init__()
        loadUi("demo_collection.ui",self)
        self.browse.clicked.connect(self.browsefiles)

    def browsefiles(self):
        folder_save=QFileDialog.getExistingDirectory(None, 'Select a folder:', 'bla', QFileDialog.ShowDirsOnly)
        self.lineEdit.setText(folder_save)

app=QApplication(sys.argv)
mainwindow=MainWindow()
widget=QtWidgets.QStackedWidget()
widget.addWidget(mainwindow)
widget.setFixedWidth(400)
widget.setFixedHeight(300)
widget.show()
sys.exit(app.exec_())