import sys
import os
from PyQt4 import QtGui, uic
import vtk
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import numpy as np
from math import radians, degrees

# Load c library
import ctypes

functionPath = os.path.dirname(os.path.realpath(__file__))
path = os.path.join(functionPath,"..","..","precompiled_clibs")
os.chdir(path)

import glob
print glob.glob("*")

# If on windows
if os.name == "nt":
    lib = ctypes.CDLL("./lib_qf_registration_win_10.dll")
else:
    lib = ctypes.CDLL("./lib_qf_registration_linux.so")

lib.qf_register.argtypes = [ctypes.c_char_p,ctypes.c_char_p]

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        os.chdir(functionPath)
        uic.loadUi("registration_gui.ui", self)
        # Connect buttons to functions
        # import moving data
        self.ptcldMovingButton.clicked.connect(self.import_data)
        # import fixed data  
        self.ptcldFixedButton.clicked.connect(self.import_data)  
        # execute qr_register function                                                                                               
        self.registrationButton.clicked.connect(self.qf_register) 
 
        self.vl = QtGui.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.vtkFrame)
        self.vl.addWidget(self.vtkWidget)
 
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        filename_fixed = "./bunny/ptcld_fixed_5.txt"
        filename_moving = "./bunny/ptcld_moving_5.txt"
        #filename = "./bunny/reconstruction/bun_zipper.ply"

        self.actor_moving = vtk.vtkActor()
        self.moving_color = (0,0.2)
        self.actor_fixed = vtk.vtkActor()
        self.fixed_color = (.8,1)
        
        self.ren.AddActor(self.actor_moving)
        self.ren.AddActor(self.actor_fixed)
 
        self.ren.ResetCamera()
        
        self.vtkFrame.setLayout(self.vl)
        self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.show()
        self.iren.Initialize()

    def _readFile(self,filename,color):
        # Takes filename as string and returns polyData
        extension = os.path.splitext(filename)[1]

        # Read PLY into vtkPoints
        if extension == ".ply":
            reader = vtk.vtkPLYReader()
            reader.SetFileName(filename)
            reader.Update()
            points = reader.GetOutput().GetPoints()
            vertices = vtk.vtkCellArray()
            for i in range(0,reader.GetOutput().GetNumberOfPoints()):
                vertices.InsertNextCell(1)
                vertices.InsertCellPoint(i)

        # Read TXT into vtkPoints
        elif extension == ".txt":
            textReader = vtk.vtkDelimitedTextReader()
            textReader.SetFileName(filename)
            textReader.SetFieldDelimiterCharacters('\t ')
            textReader.DetectNumericColumnsOn()
            textReader.Update()
            table = textReader.GetOutput()
            points = vtk.vtkPoints()
            vertices = vtk.vtkCellArray()
            for i in range(0,table.GetNumberOfRows()):
                points.InsertNextPoint(table.GetValue(i,0).ToDouble(),
                                       table.GetValue(i,1).ToDouble(),
                                       table.GetValue(i,2).ToDouble())
                vertices.InsertNextCell(1)
                vertices.InsertCellPoint(i)

        else:
            raise InputError("file must be ply or txt")

        # Add all generated data to polydata
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetVerts(vertices)

        return polydata

    def _updateActorPolydata(self,actor,polydata,color):
        # Modifies an actor with new polydata
        bounds = polydata.GetBounds()

        # Generate colors
        colors = vtk.vtkElevationFilter()
        #colors.SetInputData(polydata)
        colors.SetInputConnection(polydata.GetProducerPort())
        colors.SetLowPoint(0, 0, bounds[5])
        colors.SetHighPoint(0, 0, bounds[4])
        colors.SetScalarRange(color)

        # Visualization
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(colors.GetOutput())
        else:
            mapper.SetInputData(colors)

        actor.SetMapper(mapper)
        size = 4 / len(str(polydata.GetNumberOfPoints())) + 1
        actor.GetProperty().SetPointSize(size)

    # Function for opening .txt file and import the file path into the lineEdit
    def import_data(self):
        cwd = os.getcwd()
        fname = QtGui.QFileDialog.getOpenFileName(self, "Open file", cwd)
        fname = str(fname)
        sending_button = str(self.sender().objectName())
        if sending_button == "ptcldMovingButton":
            self.ptcldMovingText.setText(fname)
            polydata = self._readFile(fname,(.8,1))
            self._updateActorPolydata(self.actor_moving,polydata,self.moving_color)
        elif sending_button == "ptcldFixedButton":
            self.ptcldFixedText.setText(fname)
            polydata = self._readFile(fname,(.8,1))
            self._updateActorPolydata(self.actor_fixed,polydata,self.fixed_color)
        self.ren.ResetCamera()

    # Take file path from the lineEdit and perform registration using those dataset
    def qf_register(self):
        print("Registration starts\n")
        print(str(self.ptcldMovingText.text()))
        print(str(self.ptcldFixedText.text()))

        b_string1 = str(self.ptcldMovingText.text()).encode("utf-8")
        b_string2 = str(self.ptcldFixedText.text()).encode("utf-8")
        lib.qf_register.argtypes = [ctypes.c_char_p, ctypes.c_char_p]

        output = lib.qf_register(b_string1, b_string2)
        ArrayType = ctypes.c_longdouble*6
        array_pointer = ctypes.cast(output,ctypes.POINTER(ArrayType))
        test = np.frombuffer(array_pointer.contents, dtype=np.longdouble)
        transform = vtk.vtkTransform()
        transform.SetMatrix(reg_params_to_transformation_matrix(test))
        self.actor_moving.SetPosition(transform.GetPosition())
        self.actor_moving.SetOrientation(transform.GetOrientation())
        self.iren.Render()
        print "TEST", test

def eul2quat(eul):
    eulHalf = np.array(eul) / 2
    c = np.cos(eulHalf)
    s = np.sin(eulHalf)

    w = c[0] * c[1] * c[2] + s[0] * s[1] * s[2]
    x = c[0] * c[1] * s[2] - s[0] * s[1] * c[2]
    y = c[0] * s[1] * c[2] + s[0] * c[1] * s[2]
    z = s[0] * c[1] * c[2] - c[0] * s[1] * s[2]

    d = np.linalg.norm(np.array([w,x,y,z]))

    w /= d 
    x /= d 
    y /= d 
    z /= d 

    transform = vtk.vtkTransform()
    transform.RotateWXYZ(w,x,y,z)
    print transform.GetOrientation()
    return transform

def eul2rotm(eul):
    R = np.identity(4)
    ct = np.cos(eul)
    st = np.sin(eul)

    #     The rotation matrix R can be construted (as follows by
    #     ct = [cz cy cx] and st =[sz sy sx]
    #
    #       R = [  cy*cz   sy*sx*cz-sz*cx    sy*cx*cz+sz*sx
    #              cy*sz   sy*sx*sz+cz*cx    sy*cx*sz-cz*sx
    #                -sy            cy*sx             cy*cx] 

    R[0,0] =ct[1] * ct[0]
    R[0,1] =st[2] * st[1] * ct[0] - ct[2] * st[0]
    R[0,2] =ct[2] * st[1] * ct[0] + st[2] * st[0]
    R[1,0] =ct[1] * st[0]
    R[1,1] =st[2] * st[1] * st[0] + ct[2] * ct[0]
    R[1,2] =ct[2] * st[1] * st[0] - st[2] * ct[0]
    R[2,0] =-st[1]
    R[2,1] =st[2] * ct[1]
    R[2,2] =ct[2] * ct[1] 
    return R

def reg_params_to_transformation_matrix(params):
    
    T = np.identity(4)

    for r in range(0,3):
        T[r, 3] = params[r]

    temp = np.zeros(3)
    temp[0] = params[3]
    temp[1] = params[4]
    temp[2] = params[5]

    R = eul2rotm(temp)

    # Not sure what this is about but it messes everything up
    #u,s,v = np.linalg.svd(R)
    #R = np.dot(np.transpose(v),u)

    for r in range(0,3):
        for c in range(0,3):
            T[r, c] = R[r, c]

    retMat = vtk.vtkMatrix4x4()
    for r in range(0,4):
        for c in range(0,4):
            retMat.SetElement(r,c,T[r,c])

    return retMat


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    quat = eul2quat([0.509564, 0.486538, 0.230276])
    sys.exit(app.exec_())
