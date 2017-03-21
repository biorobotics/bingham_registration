#!/usr/bin/env python
import os
import ctypes
import numpy as np

def write_txt(filename, points):
    # Writes an n x 3 matrix into a text file 
    points = points.reshape(-1, 3)
    with open(filename, 'w') as f:
        f.seek(0)
        np.savetxt(f,points,'%f %f %f')
        f.truncate()

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

    return (x,y,z,w)

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

    return T

# Get registration ctypes funciton
# Loading dlls only works if we are in the same directory, so move there first
startPath = os.getcwd()
functionPath = os.path.dirname(os.path.realpath(__file__))
path = os.path.join(functionPath,"..","..","precompiled_clibs")
os.chdir(path)
# If on windows
if os.name == "nt":
    lib = ctypes.CDLL("./lib_qf_registration_windows.dll")
# Else on linux
else:
    lib = ctypes.CDLL("./lib_qf_registration_linux.so")
# Set up function
lib.qf_register.argtypes = [ctypes.c_char_p,ctypes.c_char_p, ctypes.c_double, ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double]
lib.qf_register.restype = ctypes.POINTER(ctypes.c_longdouble*6)
os.chdir(startPath)

def qf_register(fileNameMoving,fileNameFixed,inlierRatio,maxIter,windowSize,rotTolerance,transTolerance):
    output = lib.qf_register(fileNameMoving, fileNameFixed,
                             ctypes.c_double(inlierRatio),
                             ctypes.c_int(maxIter),
                             ctypes.c_int(windowSize),
                             ctypes.c_double(rotTolerance),
                             ctypes.c_double(transTolerance))
    regParams = np.frombuffer(output.contents, dtype=np.longdouble)
    return regParams