# 
# File header:
#   A Qt GUI that visualizes the registration
#
from PyQt4 import QtGui
import sys
import os
import ctypes
lib = ctypes.cdll.LoadLibrary('/home/olivia/qf_registration_ws/devel/lib/libdual_quaternion_registration.so')

import vtk
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import ui_design

class RegistrationApp(QtGui.QMainWindow, ui_design.Ui_MainWindow):
    def __init__(self):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
                            # It sets up layout and widgets that are defined
        self.ptcldMovingButton.clicked.connect(self.import_data)  # When the button is pressed
                                                            # Import moving data
        self.ptcldFixedButton.clicked.connect(self.import_data)  # When the button is pressed
                                                            # Import fixed data                                                   
        self.registrationButton.clicked.connect(self.qf_register)  # When the button is pressed
                                                            # Execute qr_register function
        self.vl = QtGui.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.vtkFrame)
        self.vl.addWidget(self.vtkWidget)
 
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
 
        # Create source
        source = vtk.vtkSphereSource()
        source.SetCenter(0, 0, 0)
        source.SetRadius(5.0)
 
        # Create a mapper
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
 
        # Create an actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
 
        self.ren.AddActor(actor)
 
        self.ren.ResetCamera()
 
        self.vtkFrame.setLayout(self.vl)

    # Function for opening .txt file and import the file path into the lineEdit
    def import_data(self):
    	fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file', 
        '/home')
    	sending_button = str(self.sender().objectName())
    	if sending_button == "ptcldMovingButton":
    		self.ptcldMovingText.setText(fname)
    	elif sending_button == "ptcldFixedButton":
    		self.ptcldFixedText.setText(fname)

    # Take file path from the lineEdit and perform registration using those dataset
    def qf_register(self):
        print("Registration starts\n")
        print(str(self.ptcldMovingText.text()))
        print(str(self.ptcldFixedText.text()))
        test = lib.qf_register(str(self.ptcldMovingText.text()), str(self.ptcldFixedText.text()));
    	
def main():
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = RegistrationApp()                 # We set the form to be our RegistrationApp (design)
    form.show()                         # Show the form
    form.iren.Initialize()
    app.exec_()                         # and execute the app

if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function