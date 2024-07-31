# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'demo_collection.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(612, 100)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(20, 29, 231, 16))
        self.label.setObjectName("label")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(250, 30, 201, 21))
        self.lineEdit.setObjectName("lineEdit")
        self.file_push_button = QtWidgets.QPushButton(self.centralwidget)
        self.file_push_button.setGeometry(QtCore.QRect(460, 30, 41, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.file_push_button.setFont(font)
        self.file_push_button.setObjectName("file_push_button")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 612, 22))
        self.menubar.setObjectName("menubar")
        self.menuDemo = QtWidgets.QMenu(self.menubar)
        self.menuDemo.setObjectName("menuDemo")
        MainWindow.setMenuBar(self.menubar)
        self.actionRecord = QtWidgets.QAction(MainWindow)
        self.actionRecord.setObjectName("actionRecord")
        self.actionCameras = QtWidgets.QAction(MainWindow)
        self.actionCameras.setObjectName("actionCameras")
        self.menubar.addAction(self.menuDemo.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Folder in which to save the demo:"))
        self.file_push_button.setText(_translate("MainWindow", "..."))
        self.menuDemo.setTitle(_translate("MainWindow", "Demo"))
        self.actionRecord.setText(_translate("MainWindow", "Record"))
        self.actionCameras.setText(_translate("MainWindow", "Cameras"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())