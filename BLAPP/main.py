# This Python file uses the following encoding: utf-8
import sys
import os
import subprocess
import csv
import PyQt5
import roslaunch
from PySide2.QtWidgets import QApplication
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import *
import time

#class for implementing a signal that is emitted when WorkerAlgos has finished
class Signals(QtCore.QObject):
    completed = QtCore.pyqtSignal()


#Worker which launches algorithms and activates criteria parser
class WorkerAlgos(QtCore.QRunnable):
    #initialize class and all the variables
    def __init__(self, amount, algName, desttext, fname):
        super(WorkerAlgos,self).__init__()
        self.amount = amount
        self.algName = algName
        self.desttext = desttext
        self.fname = fname
        self.signals = Signals()
        self.flag = True
    #run the Worker
    @QtCore.pyqtSlot()
    def run(self):
        #get gazebo world name
        if self.fname == '':
            self.fname = 'turtlebot3_world.launch'
        else:
            spl = self.fname.split("/")
            self.fname = spl[len(spl)-1]
        #from destinations file get destination points line by line and trasfer them into destarray
        destarray = []
        destfile = open(self.desttext, 'r')
        destfile.seek(0)
        Lines = destfile.readlines()
        for line in Lines:
            if line[len(line)-1] == '\n':
                line = line[0:len(line)-1]
            spl = line.split(" ")
            destarray.append(spl)
        destfile.close()

        #start testing
        for i in range (0, self.amount):
            #check what is bigger: amount of dest points or amount of tests, break if there is not enough dest points in dest file
            if len(destarray) >= i+1 and self.flag == True:
                x = (destarray[i])[0]
                y = (destarray[i])[1]
            else:
                break
            
            time.sleep(2)
            #launch algorithm
            os.system('python launchalgos.py '+ self.algName + ' ' +str(x)+' '+str(y))
            #define log file name
            resultsName = "../.ros/results.txt"
            #launch parser
            os.system('python resultparser.py '+ resultsName + ' ' +str(x)+' '+str(y) + ' ' +self.fname + ' ' + self.algName)
            #quit all ros processes
            os.system("bash quit.sh")
        #emit signal when testing is done
        self.signals.completed.emit()

    #on closing, define a flag which helps to stop ros processes when app has closed prematurely during testing
    def close(self):
        self.flag =False

#Worker which restarts Gazebo for simulations
class WorkerMap(QtCore.QRunnable):
    #initializing inner class variables
    def __init__(self, fname, amount, desttext):
        QtCore.QRunnable.__init__(self)
        self.fname = fname
        self.amount = amount
        self.desttext = desttext
        self.flag = True
    
    #run the Worker
    @QtCore.pyqtSlot()
    def run(self):
        #get gazebo world name
        if self.fname == '':
            self.fname = 'turtlebot3_world.launch'
        else:
            spl = self.fname.split("/")
            self.fname = spl[len(spl)-1]
        #from destinations file get destination points line by line and trasfer them into destarray
        destarray = []
        destfile = open(self.desttext, 'r')
        destfile.seek(0)
        Lines = destfile.readlines()
        for line in Lines:
            if line[len(line)-1] == '\n':
                line = line[0:len(line)-1]
            spl = line.split(" ")
            destarray.append(spl)
        destfile.close()

        #run Gazebo for desired amount of times
        #check what is bigger: amount of dest points or amount of tests, break if there is not enough dest points in dest file
        for i in range (0, self.amount):
            if len(destarray) >= i+1 and self.flag  == True:
                os.system('python gazebolaunch.py ' + self.fname)

    #on closing, define a flag which helps to stop ros processes when app has closed prematurely during testing
    def close(self):
        self.flag = False


#Class for implementing window UI
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        # set a window and main widget
        MainWindow.resize(562, 686)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        #set grid layout in our widget
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        #Label amount of tests
        self.label_13 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_13.setFont(font)
        self.label_13.setObjectName("label_13")
        self.gridLayout.addWidget(self.label_13, 19, 0, 1, 1)
        # checkboxes below(cb)
        #cb repeated segments
        self.repeatingpoints = QtWidgets.QCheckBox(self.centralwidget)
        self.repeatingpoints.setText("")
        self.repeatingpoints.setObjectName("repeatingpoints")
        self.gridLayout.addWidget(self.repeatingpoints, 14, 1, 1, 1)
        #cb time
        self.Time = QtWidgets.QCheckBox(self.centralwidget)
        self.Time.setText("")
        self.Time.setObjectName("Time")
        self.gridLayout.addWidget(self.Time, 10, 3, 1, 1)
        #cb amount of degrees rotated
        self.degrees = QtWidgets.QCheckBox(self.centralwidget)
        self.degrees.setText("")
        self.degrees.setObjectName("degrees")
        self.gridLayout.addWidget(self.degrees, 14, 3, 1, 1)
        #cb calculation time
        self.calctime = QtWidgets.QCheckBox(self.centralwidget)
        self.calctime.setText("")
        self.calctime.setObjectName("calctime")
        self.gridLayout.addWidget(self.calctime, 15, 3, 1, 1)
        #cb memory usage
        self.memory = QtWidgets.QCheckBox(self.centralwidget)
        self.memory.setText("")
        self.memory.setObjectName("memory")
        self.gridLayout.addWidget(self.memory, 15, 1, 1, 1)
        #cb path length
        self.path = QtWidgets.QCheckBox(self.centralwidget)
        self.path.setText("")
        self.path.setObjectName("path")
        self.gridLayout.addWidget(self.path, 10, 1, 1, 1)
        #cb complexity
        self.complexity = QtWidgets.QCheckBox(self.centralwidget)
        self.complexity.setText("")
        self.complexity.setObjectName("complexity")
        self.gridLayout.addWidget(self.complexity, 12, 1, 1, 1)
        #amount of obstacles encountered
        self.Obstacleshit = QtWidgets.QCheckBox(self.centralwidget)
        self.Obstacleshit.setText("")
        self.Obstacleshit.setObjectName("Obstacleshit")
        self.gridLayout.addWidget(self.Obstacleshit, 12, 3, 1, 1)
        #cb real time factor
        self.rtf = QtWidgets.QCheckBox(self.centralwidget)
        self.rtf.setText("")
        self.rtf.setObjectName("rtf")
        self.gridLayout.addWidget(self.rtf, 16, 1, 1, 1)
        #amount of tests spinbox
        self.spinBox = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox.setObjectName("spinBox")
        self.spinBox.setValue(1)
        self.gridLayout.addWidget(self.spinBox, 19, 1, 1, 1)
        #calculation time label
        self.label_12 = QtWidgets.QLabel(self.centralwidget)
        self.label_12.setObjectName("label_12")
        self.label_12.setToolTip('Time invested in calculating the path during a single run')
        self.gridLayout.addWidget(self.label_12, 15, 2, 1, 1)
        #select Gazebo world label
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 2, 0, 1, 1)
        #edittable line for launch file from Gazebo
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.gridLayout.addWidget(self.lineEdit_2, 3, 0, 1, 3)
        #combobox for algorithms selection
        self.comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.gridLayout.addWidget(self.comboBox, 1, 0, 1, 1)
        #Total time label
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 10, 2, 1, 1)
        #choose an algorithm label
        self.label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        #spacer
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem, 6, 0, 1, 4)
        #Destinations(.txt) label
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 4, 0, 1, 1)
        #Total Rotation (in degrees)
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setObjectName("label_11")
        self.label_11.setToolTip('How many degrees robot rotated during a single run')
        self.gridLayout.addWidget(self.label_11, 14, 2, 1, 1)
        #Path length label
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 10, 0, 1, 1)
        #dest file line edit
        self.lineEdit_3 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.label_3.setToolTip('Check README for more information')
        self.gridLayout.addWidget(self.lineEdit_3, 5, 0, 1, 3)
        #browse luanch file destination
        def browsefiles():
            #open dialog window get file name
            fname = QFileDialog.getOpenFileName(self,'Open launch file',"../catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch",'Launch files(*.launch)')
            #enter file name into line edit
            self.lineEdit_2.setText(fname[0])
        # button to browse for launch files
        self.Browse1 = QtWidgets.QPushButton(self.centralwidget)
        self.Browse1.setObjectName("Browse1")
        self.Browse1.clicked.connect(browsefiles)
        self.gridLayout.addWidget(self.Browse1, 3, 3, 1, 1)
        #REal time factor label
        self.label_14 = QtWidgets.QLabel(self.centralwidget)
        self.label_14.setObjectName("label_14")
        self.label_14.setToolTip('Simulation Time/Real Time'+'\n'+'Good sign is RTF being close to 1.00')
        self.gridLayout.addWidget(self.label_14, 16, 0, 1, 1)

        #function to finad dest file using dialog window
        def browsedest():
            #open dialog window end get det file
            desttext = QFileDialog.getOpenFileName(self,'Open Destinations file(txt)',"/home",'TXT files(*.txt)')
            #enter dest file name into line edit
            self.lineEdit_3.setText(desttext[0])
        #button which is clicked to browse for dest file
        self.Browse2 = QtWidgets.QPushButton(self.centralwidget)
        self.Browse2.setObjectName("Browse2")
        self.Browse2.clicked.connect(browsedest)
        self.gridLayout.addWidget(self.Browse2, 5, 3, 1, 1)


        #launching algorithms and gazebo
        def launch():
            #get launch file name
            fname = self.lineEdit_2.text()
            #get dest file name
            desttext = self.lineEdit_3.text()
            #if no dest file give error, else run the programm
            if desttext == '':
                msg = QMessageBox(self)
                msg.setWindowTitle("Error!")
                msg.setText("Choose a destinations file, please!")
                msg.setIcon(QMessageBox.Warning)
                msg.exec_()
            else:
                #disable launch button
                self.pushButton.setEnabled(False)
                #activate threads
                threadpool = QtCore.QThreadPool.globalInstance()
                #get amount of tests
                amount = self.spinBox.value()
                #get algorithm name
                algName = self.comboBox.currentText()
                #activate worker to laucnh algos
                self.workeralgos = WorkerAlgos(amount, algName, desttext, fname)
                #connect signals to worker, when caught close ros processes
                self.workeralgos.signals.completed.connect(self.closeGazebo)
                #connect worker to thread
                threadpool.start(self.workeralgos)
                #activate worker to open gazebo map
                self.workermap = WorkerMap(fname, amount, desttext)
                #connect worker to thread
                threadpool.start(self.workermap)



        #launch button
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setObjectName("pushButton")
        self.pushButton.clicked.connect(launch)
        self.gridLayout.addWidget(self.pushButton, 22, 2, 1, 2)

        #memory usage label
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setObjectName("label_10")
        self.label_10.setToolTip('Memory used during a single run in kilobytes')
        self.gridLayout.addWidget(self.label_10, 15, 0, 1, 1)
        #complexity label
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setObjectName("label_7")
        self.label_7.setToolTip('Easy - Difficult.'+'\n'+ 'Subjective rating of implementing an algorithm' +'\n'+ 'using ROS')
        self.gridLayout.addWidget(self.label_7, 12, 0, 1, 1)
        #obstacles encountered label
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setObjectName("label_8")
        self.label_8.setToolTip('Amount of obstacles encountered during a single run')
        self.gridLayout.addWidget(self.label_8, 12, 2, 1, 1)
        #spacer
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem1, 18, 0, 1, 4)
        #Sum of repeated segments label
        self.label_16 = QtWidgets.QLabel(self.centralwidget)
        self.label_16.setObjectName("label_16")
        self.label_16.setToolTip('Sum of segments which robot' + '\n'+ 'drived through more than once')
        self.gridLayout.addWidget(self.label_16, 14, 0, 1, 1)
        #Criteria label
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 7, 0, 1, 4)
        MainWindow.setCentralWidget(self.centralwidget)
        #menubar and status bar
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 562, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        #clear csv table functioan if it exists
        def clearTable():
            if os.path.isfile('critFile.csv'):
                os.remove('critFile.csv')

        #button to clear csv file
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_2.clicked.connect(clearTable)
        self.gridLayout.addWidget(self.pushButton_2, 23, 3, 1, 1)

        #show table function
        def showTable():
            #create table
            self.table = QtWidgets.QTableWidget()
            checkheader = []
            i = 0
            #create header for table
            header = ['Algorithm','X','Y', 'Map Name','Path Length','Total Time','Complexity','Obstacles Hit','Sum of Repeated Segments','Total Rotation(in degrees)','Memory Usage','Calculation Time','Real Time Factor']
            #get clicked checkboxes
            checkboxinput =[True,True, True, True,self.path.isChecked(), self.Time.isChecked(),  self.complexity.isChecked(), self.Obstacleshit.isChecked(),  self.memory.isChecked(), self.calctime.isChecked(), self.degrees.isChecked(), self.repeatingpoints.isChecked(), self.rtf.isChecked()]
            #Construct a table based on checkboxes clicked
            for check in checkboxinput:
                if check:
                    checkheader.append(header[i])
                    i+=1
                else:
                    i+=1
            #read csv file
            if os.path.isfile('critFile.csv'):
              #set table size
              self.table.setColumnCount(len(checkheader))
              self.table.setHorizontalHeaderLabels(checkheader)
              critFile = open('critFile.csv', 'r')
              reader = csv.DictReader(critFile)
              row = 0
              #input data fron csv file to table
              for r in reader:
                  i=0
                  self.table.insertRow(row)
                  for name in checkheader:
                      self.table.setItem(row, i, QtWidgets.QTableWidgetItem(r[name]))
                      i+=1
                  row += 1
              critFile.close()
              #resize table so that the data looks good
              self.table.resize(1500, 500)
              head = self.table.horizontalHeader()
              for i in range(len(checkheader)):
                head.setSectionResizeMode(i, QHeaderView.ResizeToContents)
              self.table.show()
            else:
              self.table.setColumnCount(len(checkheader))
              self.table.setHorizontalHeaderLabels(checkheader)
              head = self.table.horizontalHeader()
              for i in range(len(checkheader)):
                head.setSectionResizeMode(i, QHeaderView.ResizeToContents)
              self.table.resize(1500, 500)
              #show table
              self.table.show()
        
        #button to show table
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_3.clicked.connect(showTable)
        self.gridLayout.addWidget(self.pushButton_3, 23, 2, 1, 1)

        MainWindow.setStatusBar(self.statusbar)

        #connect this generated UI to main window UI
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    #setting text which is showed in the objects, quick way to understand what label is what
    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Working title"))
        self.label_13.setText(_translate("MainWindow", "Amount of tests"))
        self.label_12.setText(_translate("MainWindow", "Calculation Time"))
        self.label_2.setText(_translate("MainWindow", "Select Gazebo world"))
        self.comboBox.setItemText(0, _translate("MainWindow", "bug1"))
        self.comboBox.setItemText(1, _translate("MainWindow", "bug2"))
        self.comboBox.setItemText(2, _translate("MainWindow", "visbug21"))
        self.comboBox.setItemText(3, _translate("MainWindow", "visbug22"))
        self.comboBox.setItemText(4, _translate("MainWindow", "distbug"))
        self.comboBox.setItemText(5, _translate("MainWindow", "class1"))
        self.comboBox.setItemText(6, _translate("MainWindow", "alg1"))
        self.comboBox.setItemText(7, _translate("MainWindow", "alg2"))
        self.label_6.setText(_translate("MainWindow", "Total Time"))
        self.label.setText(_translate("MainWindow", "Choose an algorithm"))
        self.label_3.setText(_translate("MainWindow", "Destinations(.txt)"))
        self.label_11.setText(_translate("MainWindow", "Total Rotation (in degrees)"))
        self.label_5.setText(_translate("MainWindow", "Path Length"))
        self.Browse1.setText(_translate("MainWindow", "Browse"))
        self.label_14.setText(_translate("MainWindow", "Real Time Factor"))
        self.Browse2.setText(_translate("MainWindow", "Browse"))
        self.pushButton.setText(_translate("MainWindow", "Launch"))
        self.label_10.setText(_translate("MainWindow", "Memory usage"))
        self.label_7.setText(_translate("MainWindow", "Complexity"))
        self.label_8.setText(_translate("MainWindow", "Obstacles Hit"))
        self.label_16.setText(_translate("MainWindow", "Sum of Repeated Segments"))
        self.label_4.setText(_translate("MainWindow", "Criteria"))
        self.pushButton_3.setText(_translate("MainWindow", "Print Table"))
        self.pushButton_2.setText(_translate("MainWindow", "Clear CSV File"))


#Main class which is responsible for main window activation
class Window(QMainWindow, Ui_MainWindow):
    #initializing main window, setting up window UI
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

    #close all the processes on closing the main window
    def closeEvent(self, event):
        self.workeralgos.close()
        self.workermap.close()
        os.system("bash quit.sh")
        self.close()

    #inner function to close running processes when gazebo is closed
    def closeGazebo(self):
        self.pushButton.setEnabled(True)
        os.system("bash quit.sh")

    #sample about
    def about(self):
        QMessageBox.about(
            self,
            "About Sample Editor",
            "<p>A sample text editor app built with:</p>"
            "<p>- PyQt</p>"
            "<p>- Qt Designer</p>"
            "<p>- Python</p>",
        )

#class to replace UI currently not being used
class FindReplaceDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        loadUi("ui/find_replace.ui", self)

if __name__ == "__main__":
    #defining QApplication
    app = QApplication(sys.argv)
    #creating Main Window
    win = Window()
    #Show UI
    win.show()
    #Exit main when app finishes its work
    sys.exit(app.exec())
