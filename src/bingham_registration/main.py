import sys
import os
import vtk
# Which PyQt we use depends on our vtk version. QT4 causes segfaults with vtk > 6
if(int(vtk.vtkVersion.GetVTKVersion()[0]) >= 6):
    import PyQt5.QtWidgets as QtGui
    import PyQt5.uic as uic
    _QT_VERSION = 5
    from QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
else:
    from PyQt4 import QtGui, uic
    _QT_VERSION = 4
    from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import numpy as np
from math import radians, degrees
from register_txt import register_txt, reg_params_to_transformation_matrix

startPath = os.getcwd()
functionPath = os.path.dirname(os.path.realpath(__file__))

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        os.chdir(functionPath)
        uic.loadUi("registration_gui.ui", self)
        os.chdir(startPath)
        # Connect buttons to functions
        # import moving data
        self.ptcldMovingButton.clicked.connect(self.import_data)
        # import fixed data  
        self.ptcldFixedButton.clicked.connect(self.import_data)  
        # execute qr_register function                                                                                               
        self.registrationButton.clicked.connect(self.register)

        self.vl = QtGui.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.vtkFrame)
        self.vl.addWidget(self.vtkWidget)
 
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        self.actor_moving = vtk.vtkActor()
        self.moving_color = (0,0.2)
        self.actor_fixed = vtk.vtkActor()
        self.fixed_color = (.8,1)
        
        self.ren.AddActor(self.actor_moving)
        self.ren.AddActor(self.actor_fixed)
 
        self.ren.ResetCamera()
        
        self.vtkFrame.setLayout(self.vl)
        self.show()
        self.iren.Initialize()
        self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

    def _readFile(self,filename):
        # Takes filename as string and returns polyData
        extension = os.path.splitext(filename)[1].lower()
        print filename

        # # Read PLY into vtkPoints
        # if extension == ".ply" or extension == ".obj" or extension == ".stl":
        #     if (extension == ".ply"):
        #         reader = vtk.vtkPLYReader()
        #     if (extension == ".obj"):
        #         reader = vtk.vtkOBJReader()
        #     if (extension == ".stl"):
        #         reader = vtk.vtkSTLReader()
        #     reader.SetFileName(filename)
        #     reader.Update()
        #     points = reader.GetOutput().GetPoints()
        #     vertices = vtk.vtkCellArray()
        #     for i in range(0,reader.GetOutput().GetNumberOfPoints()):
        #         vertices.InsertNextCell(1)
        #         vertices.InsertCellPoint(i)
        #     # Add all generated data to polydata
        #     polydata = vtk.vtkPolyData()
        #     polydata.SetPoints(points)
        #     polydata.SetVerts(vertices)

        # Read TXT into vtkPoints
        if extension == ".txt":
            textReader = vtk.vtkSimplePointsReader()
            textReader.SetFileName(filename)
            textReader.Update()
            polydata = textReader.GetOutput()
        else:
            print("input must be a .txt file")
            return None
            # raise InputError("file must be ply, stl, obj or txt")

        print("Bounds: ", polydata.GetBounds())

        return polydata

    def _updateActorPolydata(self,actor,polydata,color):
        # Modifies an actor with new polydata
        bounds = polydata.GetBounds()

        # Generate colors
        colors = vtk.vtkElevationFilter()
        #colors.SetInputData(polydata)
        if vtk.VTK_MAJOR_VERSION <= 5:
            colors.SetInputConnection(polydata.GetProducerPort())
        else:
            colors.SetInputData(polydata)
        colors.SetLowPoint(0, 0, bounds[5])
        colors.SetHighPoint(0, 0, bounds[4])
        colors.SetScalarRange(color)
        # Visualization
        mapper = actor.GetMapper()
        if mapper == None:
            mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(colors.GetOutput())
        else:
            colors.Update()
            mapper.SetInputData(colors.GetOutput())

        actor.SetMapper(mapper)
        transform = vtk.vtkTransform()
        actor.SetPosition(transform.GetPosition())
        actor.SetOrientation(transform.GetOrientation())
        size = 4 / len(str(polydata.GetNumberOfPoints())) + 1
        actor.GetProperty().SetPointSize(size)

    # Function for opening .txt file and import the file path into the lineEdit
    def import_data(self):
        cwd = os.getcwd()
        fname = QtGui.QFileDialog.getOpenFileName(self, "Open file", cwd)
        if(_QT_VERSION >= 5): # QT5 stores filename in a tuple instead of just a string
            fname = fname[0]
        filename = str(fname)
        if fname == "":
            print("Using previous file")
            return
        sending_button = str(self.sender().objectName())
        polydata = self._readFile(filename)
        if polydata == None:
            print "File not readable"
            return
        if sending_button == "ptcldMovingButton":
            self.ptcldMovingText.setText(filename)
            self._updateActorPolydata(self.actor_moving,polydata,self.moving_color)
        elif sending_button == "ptcldFixedButton":
            self.ptcldFixedText.setText(filename)
            self._updateActorPolydata(self.actor_fixed,polydata,self.fixed_color)
        self.ren.ResetCamera()
        self.iren.Render()

    # Take file path from the lineEdit and perform registration using those dataset
    def register(self):
        transform = vtk.vtkTransform()
        self.actor_moving.SetPosition(transform.GetPosition())
        self.actor_moving.SetOrientation(transform.GetOrientation())
        self.iren.Render()
        print("Registration starts\n")
        print(str(self.ptcldMovingText.text()))
        print(str(self.ptcldFixedText.text()))

        maxIter = self.maxIterations.value()
        inlierRatio = self.inlierRatio.value()
        windowSize = self.windowSize.value()
        rotTolerance = self.rotationTolerance.value()
        transTolerance = self.translationTolerance.value()


        b_string1 = str(self.ptcldMovingText.text()).encode("utf-8")
        b_string2 = str(self.ptcldFixedText.text()).encode("utf-8")
        output, error = register_txt(b_string1, b_string2, inlierRatio, maxIter,
                                    windowSize, rotTolerance,transTolerance)
        matrix = npMatrixToVtkMatrix(reg_params_to_transformation_matrix(output))
        transform.SetMatrix(matrix)
        self.actor_moving.SetPosition(transform.GetPosition())
        self.actor_moving.SetOrientation(transform.GetOrientation())
        self.iren.Render()
        
def npMatrixToVtkMatrix(matrix):
    retMat = vtk.vtkMatrix4x4()
    for r in range(0,4):
        for c in range(0,4):
            retMat.SetElement(r,c,matrix[r,c])
    return retMat


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
