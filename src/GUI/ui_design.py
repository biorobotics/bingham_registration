# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test.ui'
#
# Created: Tue Jan 24 15:53:51 2017
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(517, 647)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.ButtonsLayout = QtGui.QFrame(self.centralwidget)
        self.ButtonsLayout.setMaximumSize(QtCore.QSize(16777215, 84))
        self.ButtonsLayout.setFrameShape(QtGui.QFrame.StyledPanel)
        self.ButtonsLayout.setFrameShadow(QtGui.QFrame.Raised)
        self.ButtonsLayout.setObjectName(_fromUtf8("ButtonsLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.ButtonsLayout)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.MovingLayout = QtGui.QVBoxLayout()
        self.MovingLayout.setObjectName(_fromUtf8("MovingLayout"))
        self.ptcldMovingButton = QtGui.QPushButton(self.ButtonsLayout)
        self.ptcldMovingButton.setMaximumSize(QtCore.QSize(149, 27))
        self.ptcldMovingButton.setObjectName(_fromUtf8("ptcldMovingButton"))
        self.MovingLayout.addWidget(self.ptcldMovingButton)
        self.ptcldMovingText = QtGui.QLineEdit(self.ButtonsLayout)
        self.ptcldMovingText.setObjectName(_fromUtf8("ptcldMovingText"))
        self.MovingLayout.addWidget(self.ptcldMovingText)
        self.horizontalLayout.addLayout(self.MovingLayout)
        self.FixedLayout = QtGui.QVBoxLayout()
        self.FixedLayout.setObjectName(_fromUtf8("FixedLayout"))
        self.ptcldFixedButton = QtGui.QPushButton(self.ButtonsLayout)
        self.ptcldFixedButton.setMaximumSize(QtCore.QSize(137, 27))
        self.ptcldFixedButton.setObjectName(_fromUtf8("ptcldFixedButton"))
        self.FixedLayout.addWidget(self.ptcldFixedButton)
        self.ptcldFixedText = QtGui.QLineEdit(self.ButtonsLayout)
        self.ptcldFixedText.setObjectName(_fromUtf8("ptcldFixedText"))
        self.FixedLayout.addWidget(self.ptcldFixedText)
        self.horizontalLayout.addLayout(self.FixedLayout)
        self.registrationButton = QtGui.QPushButton(self.ButtonsLayout)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.registrationButton.sizePolicy().hasHeightForWidth())
        self.registrationButton.setSizePolicy(sizePolicy)
        self.registrationButton.setFocusPolicy(QtCore.Qt.TabFocus)
        self.registrationButton.setStyleSheet(_fromUtf8(" QPushButton#registrationButton {\n"
"    background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgb(80,200,130), stop:1 rgb(50,180,110));\n"
"    color:white;\n"
"    font-weight:bold;\n"
"    border-radius:10px;\n"
"    padding:10px;\n"
"    margin-left:10px;\n"
"    border-width: 1px;\n"
"    border-color:rgb(200,200,200);\n"
"    border-style: solid;\n"
" }\n"
" QPushButton#registrationButton:pressed {\n"
"    background-color:  qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgb(50,150,100), stop:1 rgb(50,180,110));\n"
" }\n"
"QPushButton:hover:!pressed\n"
"{\n"
"  background-color: rgb(40,170,100);\n"
"}"))
        self.registrationButton.setAutoDefault(False)
        self.registrationButton.setDefault(False)
        self.registrationButton.setFlat(False)
        self.registrationButton.setObjectName(_fromUtf8("registrationButton"))
        self.horizontalLayout.addWidget(self.registrationButton)
        self.gridLayout.addWidget(self.ButtonsLayout, 0, 0, 1, 1)
        self.vtkFrame = QtGui.QFrame(self.centralwidget)
        self.vtkFrame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.vtkFrame.setFrameShadow(QtGui.QFrame.Raised)
        self.vtkFrame.setObjectName(_fromUtf8("vtkFrame"))
        self.gridLayout.addWidget(self.vtkFrame, 1, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.ptcldMovingButton.setText(_translate("MainWindow", "Import Moving Data", None))
        self.ptcldFixedButton.setText(_translate("MainWindow", "Import Fixed Data", None))
        self.registrationButton.setText(_translate("MainWindow", "Register", None))

