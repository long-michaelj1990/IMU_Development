import os
import sys
import json
import math
import time
import socket
import serial
from serial.tools import list_ports
import ctypes
#import Arduino
import win32gui
import datetime
import numpy as np
import pyqtgraph as pg
#from Arduino import arduino
import pyqtgraph.opengl as gl
from PyQt5.QtCore import QSize
from threading import Thread, Lock
from ctypes import Structure, byref
from ctypes.util import find_library
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5 import QtCore, QtGui, QtWidgets
from distutils.core import setup, Extension
from PyQt5.QtGui import QImage, QPixmap, QIcon
from PyQt5.QtWidgets import (QToolBar, QAction, QMainWindow, QComboBox, QListWidget,
                             QPlainTextEdit,QFileDialog, QProgressBar, QLabel, QRadioButton, QLineEdit,QWidget,QSizePolicy)

import pywavefront






#--------------------------------------------------------------------------#
                        #Main GUI#
#--------------------------------------------------------------------------# 
class Ui_Dialog(QMainWindow):
    def setupUi(self, Dialog):
        self.calibrate=0
        self.IMU_position=0
        width = Dialog.frameGeometry().width()
        height = Dialog.frameGeometry().height()
        Dialog.setStyleSheet("background-color: rgb(36, 36, 36);")

        widget_width= width-200
        widget_width2=width+200
        widget_height= height-190
        widget_height2=height+155

        self.toolbarHor = QToolBar(Dialog)
        self.toolbarHor.setIconSize(QSize(40, 40))
        self.toolbarHor.setOrientation(QtCore.Qt.Horizontal)
        self.toolbarHor.setStyleSheet("QToolBar{spacing:200px;}")
        self.toolbarHor.setStyleSheet("QToolButton{padding-top:4px;}")
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        self.toolbarHor.addSeparator()
        
        Calibrate_action = QAction(QIcon('images/calibrate.png'),'calibrate', self)
        Calibrate_action.triggered.connect(self.calibrating)
        self.toolbarHor.addAction(Calibrate_action)

        closeDevice_action = QAction(QIcon('images/Disconnect.png'),'Disconnect Devices', self)
#        closecameras_action.triggered.connect(self.close_Cameras)
        self.toolbarHor.addAction(closeDevice_action)

        self.toolbar = QToolBar(Dialog)
        self.toolbar.setIconSize(QSize(40, 40))
        self.toolbar.setOrientation(QtCore.Qt.Vertical)

        connect_action = QAction(QIcon('images/connect.png'),'Connect Devices', self)
        connect_action.triggered.connect(self.Connect_Device)
        self.toolbar.addAction(connect_action)

        
        tool_action = QAction(QIcon('images/tools.png'),'Settings', self)
        self.toolbar.addAction(tool_action)
#        tool_action.triggered.connect(self.properties_window_A)

        exit_action = QAction(QIcon('images/exit.png'),'exit', self)
        self.toolbar.addAction(exit_action)
        exit_action.triggered.connect(self.window_close)
        
        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
        
        self.threeD_view()

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate  
        Dialog.setWindowTitle(_translate("Dialog", "Markerless Mocap"))

    def threeD_view(self):#-----------------------------------------------------# 3D gui graphic
        self.width = Dialog.frameGeometry().width()
        self.height = Dialog.frameGeometry().height()
        
        self.widget_width2=self.width+200
        self.widget_height2=self.height+155

        self.w = gl.GLViewWidget(Dialog)
        self.w.opts['distance'] = 40
        self.w.setGeometry(500, 55, self.widget_width2, self.widget_height2)
        self.w.setBackgroundColor('k')
        
        gz = gl.GLGridItem()
        gz.translate(0, 0, 0)
        self.w.addItem(gz)

        self.ax = gl.GLAxisItem()
        self.ax.translate(0,0,0.5)
        self.ax.rotate(180,0,1,0)
        self.ax.setSize(1,1,1)
        self.ax.show()
        self.w.addItem(self.ax)        

    def Connect_Device(self):
        ports=list_ports.comports()
        PORT="COM6"
        try:
            self.ard = serial.Serial(PORT, 115200,timeout=.1)

            verts = np.array([[1, 0, 0], #0
                     [0, 0, 0], #1
                     [0, 1, 0], #2
                     [0, 0, 1], #3
                     [1, 1, 0], #4
                     [1, 1, 1], #5
                     [0, 1, 1], #6
                     [1, 0, 1]])#7
            faces = np.array([[1,0,7], [1,3,7],
                  [1,2,4], [1,0,4],
                  [1,2,6], [1,3,6],
                  [0,4,5], [0,7,5],
                  [2,4,5], [2,6,5],
                  [3,6,5], [3,7,5]])
            colors = np.array([[1,0,0,1] for i in range(12)])
            
            self.m1 = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False)
            self.m1.translate(0, 0, 0)
            self.m1.setGLOptions('additive')
            self.w.addItem(self.m1)

            self.dataloopO=0
            self.dataloopM=0
            self.IMU1=Thread(target=self.IMU_stream)
            self.IMU1.setDaemon(True)
            self.IMU1.start()
            
        except:
            pass
        
    def IMU_stream(self):
        while True:
            if(self.ard.isOpen() == True):
                try:
                    self.IMU_data=self.ard.readline().decode().strip('\r\n')
                    if "Orientation" in self.IMU_data:                              #sensor orientation degrees
                        Data_Euler=self.IMU_data.split(" ")
                        if self.dataloopO==0:
                            self.IMU1_Euler_X_temp1=float(Data_Euler[1])                                
                            self.IMU1_Euler_Y_temp1=float(Data_Euler[2])
                            self.IMU1_Euler_Z_temp1=float(Data_Euler[3])
                            self.dataloopO=self.dataloopO+1
                        else:
                             self.dataloopO=0
                             self.IMU1_Euler_X_temp2=float(Data_Euler[1])                                
                             self.IMU1_Euler_X_temp3=math.floor(self.IMU1_Euler_X_temp2)-math.floor(self.IMU1_Euler_X_temp1)
                             IMU1_Euler_X_res =self.IMU1_Euler_X_temp3

                             if IMU1_Euler_X_res == 180:
                                 IMU1_Euler_X_res=IMU1_Euler_X_res
                             elif IMU1_Euler_X_res < 180:
                                 IMU1_Euler_X_res=self.IMU1_Euler_X_temp3
                             elif self.IMU1_Euler_X_temp2>self.IMU1_Euler_X_temp1:
                                 IMU1_Euler_X_res=IMU1_Euler_X_res-360
                             else:
                                 IMU1_Euler_X_res=360-IMU1_Euler_X_res
                                 
                             
                             self.IMU1_Euler_Y_temp2=float(Data_Euler[2])                                
                             self.IMU1_Euler_Y_temp3=self.IMU1_Euler_Y_temp2-self.IMU1_Euler_Y_temp1
                             IMU1_Euler_Y_res =self.IMU1_Euler_Y_temp3

                             if IMU1_Euler_Y_res == 180:
                                 IMU1_Euler_Y_res=IMU1_Euler_Y_res
                             elif IMU1_Euler_Y_res < 180:
                                 self.IMU1_Euler_Y_temp3=self.IMU1_Euler_Y_temp3
                             elif self.IMU1_Euler_Y_temp2>self.IMU1_Euler_Y_temp1:
                                 IMU1_Euler_Y_res=IMU1_Euler_Y_res-360
                             else:
                                 IMU1_Euler_Y_res=360-IMU1_Euler_Y_res
                             
                             self.IMU1_Euler_Z_temp2=float(Data_Euler[3])                                
                             self.IMU1_Euler_Z_temp3=self.IMU1_Euler_Z_temp2-self.IMU1_Euler_Z_temp1
                             IMU1_Euler_Z_res =self.IMU1_Euler_Z_temp3
                             
                             if IMU1_Euler_Z_res == 180:
                                 IMU1_Euler_Z_res=IMU1_Euler_Z_res
                             elif IMU1_Euler_Z_res < 180:
                                 self.IMU1_Euler_Z_temp3=self.IMU1_Euler_Z_temp3
                             elif self.IMU1_Euler_Z_temp2>self.IMU1_Euler_Z_temp1:
                                 IMU1_Euler_Z_res=IMU1_Euler_Z_res-360
                             else:
                                 IMU1_Euler_Z_res=360-IMU1_Euler_Z_res

                                    
                             IMU1_Euler_X_res=IMU1_Euler_X_res*2
                             IMU1_Euler_Y_res=IMU1_Euler_Y_res*2
                             IMU1_Euler_Z_res=IMU1_Euler_Z_res*2
       
                             self.m1.rotate(IMU1_Euler_X_res, 1,0,0, local=False) ###curenntly still flipping aroundS
                             self.m1.rotate(IMU1_Euler_Y_res, 0,1,0, local=False)
                             self.m1.rotate(IMU1_Euler_Z_res, 0,0,1, local=False)
                                
                            
                    if "Speed" in self.IMU_data:                    
                        Data_Speed=self.IMU_data.split(" ")
                        self.IMU1_Speed_X=float(Data_Speed[1])
                        self.IMU1_Speed_Y=float(Data_Speed[2])
                        self.IMU1_Speed_Z=float(Data_Speed[3])

                        
                    if "Movement" in self.IMU_data:
                        Data_Movement=self.IMU_data.split(" ")
                        if self.dataloopM==0:
                            self.IMU1_Movement_X_temp1=float(Data_Movement[1])
                            self.IMU1_Movement_Y_temp1=float(Data_Movement[2])
                            self.dataloopM=self.dataloopM+1
                        else:
                             self.dataloopM=0
                             self.IMU1_Movement_X_temp2=float(Data_Movement[1])
                             self.IMU1_Movement_Y_temp2=float(Data_Movement[2])
                             self.IMU1_Movement_X_res=math.floor(self.IMU1_Movement_X_temp2)-math.floor(self.IMU1_Movement_X_temp1)
                             self.IMU1_Movement_Y_res=math.floor(self.IMU1_Movement_Y_temp2)-math.floor(self.IMU1_Movement_Y_temp1)
                             self.m1.translate(self.IMU1_Movement_X_res,self.IMU1_Movement_Y_res, 0)

                        
                    if "Acceleration" in self.IMU_data:                             # m/s^2
                        Data_Acceleration=self.IMU_data.split(" ")
                        self.IMU1_Acceleration_X=float(Data_Acceleration[1])
                        self.IMU1_Acceleration_Y=float(Data_Acceleration[2])
                        self.IMU1_Acceleration_Z=float(Data_Acceleration[3])
                        
                    if "Magnatometer" in self.IMU_data:                             # uT
                        Data_Magnatometer=self.IMU_data.split(" ")
                        self.IMU1_Magnatometer_X=float(Data_Magnatometer[1])
                        self.IMU1_Magnatometer_Y=float(Data_Magnatometer[2])
                        self.IMU1_Magnatometer_Z=float(Data_Magnatometer[3])
                        
                    if "Gyroscope" in self.IMU_data:                                
                        Data_Gyroscope=self.IMU_data.split(" ")                     # rad/s 
                        self.IMU1_Gyroscope_X=float(Data_Gyroscope[1])
                        self.IMU1_Gyroscope_Y=float(Data_Gyroscope[2])
                        self.IMU1_Gyroscope_Z=float(Data_Gyroscope[3])
                        
                    if "LinearAcceleration" in self.IMU_data:                       # m/s^2
                        Data_LinearAcceleration=self.IMU_data.split(" ")
                        self.IMU1_LinearAcceleration_X=float(Data_LinearAcceleration[1])
                        self.IMU1_LinearAcceleration_Y=float(Data_LinearAcceleration[2])
                        self.IMU1_LinearAcceleration_Z=float(Data_LinearAcceleration[3])
                        
                    if "Gravity" in self.IMU_data:                                  # m/s^2
                        Data_Gravity=self.IMU_data.split(" ")
                        self.IMU1_Gravity_X=float(Data_Gravity[1])
                        self.IMU1_Gravity_Y=float(Data_Gravity[2])
                        self.IMU1_Gravity_Z=float(Data_Gravity[3])
                                                        
                    if "Calibration:" in self.IMU_data:
                        Data_Calibration=self.IMU_data.split(" ")
                        self.IMU1_Calibration_X=float(Data_Calibration[1])
                        self.IMU1_Calibration_Y=float(Data_Calibration[2])
                        self.IMU1_Calibration_Z=float(Data_Calibration[3])
  
                except:
                    pass
                    
            else:
                break
    def calibrating(self):
        if(self.ard.isOpen() == True):
            try:
                for i in range(2):
                    self.IMU_data_calibrate=self.IMU_data
                    if "Orientation:" in self.IMU_data_calibrate:
                        Data_calibO=self.IMU_data_calibrate.split(" ")
                        self.IMU1_calib_oX=float(Data_calibO[1])
                        self.IMU1_calib_oY=float(Data_calibO[2])
                        self.IMU1_calib_oZ=float(Data_calibO[3])
                        self.calibrate = 1
            except:
                pass
            
    def window_close(self):
        try:
            self.ard.close()
        except:
                pass
        Dialog.close()
#--------------------------------------------------------------------------#        
#--------------------------------------------------------------------------# 
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.showMaximized()
    sys.exit(app.exec_())
