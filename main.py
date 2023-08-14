"""
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2021 Mathew Clutter
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Note that this project uses PQt5, which is licensed under GPL v3
https://pypi.org/project/PyQt5/
-------------------------------------------------------------------------------
"""

from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, QObject, pyqtSignal, Qt, pyqtSlot
from PyQt5.QtWidgets import QCompleter, QApplication
from PyQt5.QtGui import *
# from designerFile import Ui_MainWindow
from MaineDesigner import Ui_MainWindow
import sys
from Balloon_Coordinates import Balloon_Coordinates
from satelliteTrackingMath import trackMath
from Ground_Station_Arduino import Ground_Station_Arduino
import serial.tools.list_ports
import serial
import time
# from pylab import *
from sunposition import sunpos
from datetime import datetime
import csv
import cv2
from selenium import webdriver
from selenium.webdriver.common.by import By

# todo: clean up this code and better document

class Window(QtWidgets.QMainWindow, Ui_MainWindow):
    # This class connects functionality to buttons in the GUI
    # The "main" class
    def __init__(self):
        super(Window, self).__init__()

        self.setupUi(self)

        self.IMEIList = Balloon_Coordinates.list_IMEI()

        self.arduinoConnected = False
        self.IMEIAssigned = False
        self.GSLocationSet = False
        self.calibrated = False

        self.tracking = False

        self.GSArduino = None  # classes will be instantiated later
        self.Balloon = None

        self.trackThread = None
        self.worker = None

        self.GSLat = 0
        self.GSLong = 0
        self.GSAlt = 0

        self.startingAzimuth = 0
        self.startingElevation = 0

        self.IMEIComboBox.addItem("")
        for i in range(len(self.IMEIList)):
            self.IMEIComboBox.addItem(self.IMEIList[i])

        completer = QCompleter(self.IMEIList)
        completer.setFilterMode(Qt.MatchContains)
        self.IMEIComboBox.setEditable(True)
        self.IMEIComboBox.setCompleter(completer)

        self.ports = None
        self.portNames = []
        self.comPortCounter = 0
        self.refreshArduinoList()
        self.refreshRFDList()

        self.confirmIMEIButton.clicked.connect(self.assignIMEI)

        self.GPSRequestButton.clicked.connect(self.getGSLocation)
        self.confirmGSLocationButton.clicked.connect(self.setGSLocation)

        self.calibrateButton.clicked.connect(self.calibrate)

        self.loadPositionData.clicked.connect(self.loadPos)
        self.newPosition.clicked.connect(self.newPos)

        self.refreshCOMPortsButton.clicked.connect(self.refreshArduinoList)
        self.connectToArduinoButton.clicked.connect(self.connectToArduino)

        self.degreesPerClickBox.setCurrentIndex(1)
        self.COMPortComboBox.setCurrentIndex(self.comPortCounter - 1)
        self.tiltUpButton.clicked.connect(self.tiltUp)
        self.tiltDownButton.clicked.connect(self.tiltDown)
        self.panCounterClockwiseButton.clicked.connect(self.panClockwise)
        self.panClockwiseButton.clicked.connect(self.panCounterClockwise)

        self.calculateStartingPosButton.clicked.connect(self.getStartingPos)

        self.backToSunButton.clicked.connect(self.returnToSun)

        self.startButton.clicked.connect(self.checkIfReady)
        self.stopButton.clicked.connect(self.stopTracking)
        self.EStopButton.clicked.connect(self.EStop)

        self.predictionStartButton.clicked.connect(self.setPredictTrack)

        font = self.font()
        font.setPointSize(8)  # can adjust for sizing
        QApplication.instance().setFont(font)

        # self.showMaximized()
        # self.showFullScreen()

        self.predictingTrack = False

        self.Worker1 = Worker1()

        self.startStreamButton.clicked.connect(self.startStream)
        self.openPredictionButton.clicked.connect(self.autoPredict)
        self.getAscentRateButton.clicked.connect(self.ascentRate)
        self.RFDComboBox.setCurrentIndex(self.comPortCounter - 1)
        self.stopStreamButton.clicked.connect(self.stopStream)

        self.Worker2 = Worker2()

        self.RFDConnectButton.clicked.connect(self.connectToRFD)

        self.RefreshRFDCOM.clicked.connect(self.refreshRFDList)

        
        global packet_count
        global fileName
        global file

        packet_count = 0
        fileName = "RFD900x_Data.csv"
        file = open(fileName, "a")
        file.close()
        
    def ascentRate(self):
        self.ascentRateBox.setPlainText(str(self.Balloon.ascentRate))

    def autoPredict(self):
        try:
            self.Balloon.get_coor_alt()
        except:
            self.statusBox.setPlainText("No IMEI Number Selected")
        else:
            lat = self.Balloon.get_coor_alt()[0]
            lon = self.Balloon.get_coor_alt()[1]
            current_alt = self.Balloon.get_coor_alt()[2]
            ascent = self.ascentRateBox.toPlainText()
            descent = self.descentRateBox.toPlainText()

            if(Balloon_Coordinates.ascentRate >= 0):
                burst = self.burstAltitudeBox.toPlainText()
            else:
                burst = current_alt + 1

            options = webdriver.ChromeOptions()
            options.add_experimental_option("detach", True)
            options.add_argument("--start-maximized")
            # Using Chrome to access web
            driver = webdriver.Chrome(options=options)
            # Open the website
            driver.get('https://predict.sondehub.org/')

            # enters balloon current position
            driver.find_element(By.ID, "lat").clear()
            driver.find_element(By.ID, "lat").send_keys(lat)

            driver.find_element(By.ID, "lon").clear()
            driver.find_element(By.ID, "lon").send_keys(lon)

            driver.find_element(By.ID, "initial_alt").clear()
            driver.find_element(By.ID, "initial_alt").send_keys(current_alt)

            driver.find_element(By.ID, "ascent").clear()
            driver.find_element(By.ID, "ascent").send_keys(ascent)

            driver.find_element(By.ID, "drag").clear()
            driver.find_element(By.ID, "drag").send_keys(descent)

            driver.find_element(By.ID, "burst").clear()
            driver.find_element(By.ID, "burst").send_keys(burst)

            driver.find_element(By.ID, "run_pred_btn").click()

    def connectToRFD(self):
        global comport
        try:
            comport = self.portNames[self.RFDComboBox.currentIndex()]
        except:
            self.statusBox.setPlainText("Error Connecting to RFD")
        else:
            self.Worker2.start()
            self.Worker2.packetNumber.connect(self.displayRFD)
    
    def displayRFD(self, RFDInfo):

        global packet_count

        header = ["Packet Number", "SIV", "FixType", "Latitude", \
                "Longitude", "Altitude", "Year", "Month", "Day", \
                "Hour", "Min", "Sec", "NNV", "NEV", "NDV", "Battery" ,\
                "3v3 Supply", "5v Supply", "Radio Supply", "Analog Internal", \
                "Analog External", "Altimeter Temp", "Digital Internal", \
                "Digital Eternal", "Pressure", "Accel A", "Accel Y", "Accel z", \
                "Pitch", "Roll", "Yaw"]

        if packet_count == 0:
            with open(fileName, "a", newline = '\n') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(header)
                file.close()

        finalData = xx.split(",")
        if len(finalData) > 10:
            with open(fileName, "a", newline = '\n') as f:
                writer = csv.writer(f, delimiter=',')
                writer.writerow(finalData)
                packet_count = packet_count + 1
                file.close()

        if len(RFDInfo) >=31:
            self.currentPacketBox.setPlainText(str(RFDInfo[0].strip()))
            packet = RFDInfo[0].strip()
            self.packetsReceivedBox.setPlainText(str(packet_count)+ "/" + str(packet) + ", " + \
            str(round(((int(packet_count)/int(packet))*100),2)) + "%")
            self.dateBox.setPlainText(str(RFDInfo[6]) + "-" + str(RFDInfo[7]) + "-" + str(RFDInfo[8]))
            self.timeBox.setPlainText(str(RFDInfo[9]) + ":" + str(RFDInfo[10]) + ":" + str(RFDInfo[11]))
            self.batteryVoltageBox.setPlainText(str(RFDInfo[15]))
            self.voltage3v3Box.setPlainText(str(RFDInfo[16]))
            self.voltage5vBox.setPlainText(str(RFDInfo[17]))
            self.radioVoltageBox.setPlainText(str(RFDInfo[18]))
            self.analogInternalTempBox.setPlainText(str(RFDInfo[19]))
            self.analogExternalTempBox.setPlainText(str(RFDInfo[20]))
            self.digitalInternalTempBox.setPlainText(str(RFDInfo[22]))
            self.digitalExternalTempBox.setPlainText(str(RFDInfo[23]))
            self.pressureSensorTempBox.setPlainText(str(RFDInfo[21]))
            self.satInViewBox.setPlainText(str(RFDInfo[1]))
            self.latitudeBox.setPlainText(str(int(RFDInfo[3]) * .0000001))
            self.longitudeBox.setPlainText(str(int(RFDInfo[4]) * .0000001))
            self.altitudeBox.setPlainText(str(int(RFDInfo[5]) / 1000))
            self.pressureBox.setPlainText(str(RFDInfo[24]))
            self.nedNorthVelBox.setPlainText(str(RFDInfo[12]))
            self.nedEastVel.setPlainText(str(RFDInfo[13]))
            self.nedDownVel.setPlainText(str(RFDInfo[14]))
            self.accelXBox.setPlainText(str(RFDInfo[25]))
            self.accelYBox.setPlainText(str(RFDInfo[26]))
            self.accelZBox.setPlainText(str(RFDInfo[27]))
            self.pitchBox.setPlainText(str(RFDInfo[28]))
            self.rollBox.setPlainText(str(RFDInfo[29]))
            self.yawBox.setPlainText(str(RFDInfo[30]))

            fix = str(RFDInfo[2])
            a1 = ""
            if fix != "":
                if int(fix) == 0:
                    a1 = "No Fix"
                elif int(fix) == 1:
                    a1 ="Dead Reckoning"
                elif int(fix) == 2:
                    a1 ="2D"
                elif int(fix) == 3:
                    a1 ="3D"
                elif int(fix) == 4:
                    a1 ="GNSS + Dead Reckoning"

            self.fixTypeBox.setPlainText(a1)

    def startStream(self):
        self.Worker1.stop()
        global streamLink
        if(self.streamLinkBox.toPlainText() == ''):
            streamLink = 0
            self.statusBox.setPlainText("Using Webcam for Video Stream")
        else:
            streamLink = self.streamLinkBox.toPlainText()
            self.statusBox.setPlainText('Video Stream from: {}'.format(streamLink))

        self.Worker1.start()
        self.Worker1.ImageUpdate.connect(self.ImageUpdateSlot)
    
    def stopStream(self):
        self.Worker1.stop()

    def ImageUpdateSlot(self, Image):
        self.videoStream.setPixmap(QPixmap.fromImage(Image))

    def assignIMEI(self):
        # this function checks if an IMEI has been selected
        # if an IMEI has been selected, it creates an instance of the balloon coordinates class using the IMEI
        # if an IMEI has not been selected, it simply returns
        if self.IMEIComboBox.currentIndex() != 0:  # SHOULD BE != FOR BOREALIS WEBSITE!
            self.IMEIAssigned = True
            print(self.IMEIComboBox.currentText())
            self.Balloon = Balloon_Coordinates(self.IMEIComboBox.currentText())
            testStr = self.Balloon.print_info()
            self.statusBox.setPlainText(testStr)
            # self.Balloon.getTimeDiff()
        else:
            print("select a balloon ")
            self.statusBox.setPlainText("Please select a balloon IMEI")
            self.IMEIAssigned = False
        return

    def refreshArduinoList(self):
        # this function searches the list of COM ports, and adds devices that it finds to the COM port combobox
        self.COMPortComboBox.clear()
        self.ports = serial.tools.list_ports.comports()
        self.portNames = []
        self.comPortCounter = 0
        for port, desc, hwid in sorted(self.ports):
            # self.COMPortComboBox.addItem("[{}] {}: {}".format(i, port, desc))
            self.COMPortComboBox.addItem(desc)
            self.portNames.append("{}".format(port))
            self.comPortCounter += 1

    def refreshRFDList(self):
            # this function searches the list of COM ports, and adds devices that it finds to the COM port combobox
        self.RFDComboBox.clear()
        self.ports = serial.tools.list_ports.comports()
        self.portNames = []
        self.comPortCounter = 0
        for port, desc, hwid in sorted(self.ports):
            # self.COMPortComboBox.addItem("[{}] {}: {}".format(i, port, desc))
            self.RFDComboBox.addItem(desc)
            self.portNames.append("{}".format(port))
            self.comPortCounter += 1

    def connectToArduino(self):
        # checks if arduino is selected, and if the connection is not already made, instantiates an instance of
        # the Ground_Station_Arduino class
        # if an arduino is connected, or one is not selected, the function returns
        if not self.arduinoConnected and self.COMPortComboBox.currentText():
            self.GSArduino = Ground_Station_Arduino(self.portNames[self.COMPortComboBox.currentIndex()], 9600)
            self.statusBox.setPlainText("connected to arduino!")
            self.arduinoConnected = True
        elif self.arduinoConnected:
            print("Arduino already connected")
            self.statusBox.setPlainText("Arduino already connected")
        else:
            self.statusBox.setPlainText("Unable to connect to Arduino")

        return
    
    def newPos(self):
        self.statusBox.setPlainText("Manually enter coordinates and altitude")

        return
    
    def loadPos(self):
        self.statusBox.setPlainText("Opening gs_log.csv")
        with open("gs_log.csv", "r") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                latStr = row[0]
                self.GSLatBox.setPlainText(str(latStr))
                lonStr = row[1]
                self.GSLongBox.setPlainText(str(lonStr))
                altStr = row[2]
                self.GSAltBox.setPlainText(str(altStr))

        return

    def tiltUp(self):
        # if an arduino is connected, uses GSArduino to adjust the tilt up
        if self.arduinoConnected:
            self.GSArduino.adjustTiltUp(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting tilt up " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return

    def tiltDown(self):
        # if an arduino is connected, uses GSArduino to adjust the tilt down
        if self.arduinoConnected:
            self.GSArduino.adjustTiltDown(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting tilt down " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return

    def panCounterClockwise(self):
        # if an arduino is connected, uses GSArduino to adjust the pan counter-clockwise
        if self.arduinoConnected:
            self.GSArduino.adjustPanNegative(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " degrees negative")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return

    def panClockwise(self):
        # if an arduino is connected, uses GSArduino to adjust the pan clockwise
        if self.arduinoConnected:
            self.GSArduino.adjustPanPositive(self.degreesPerClickBox.currentText())
            self.statusBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " degrees positive")
        else:
            print("Unable to connect to ground station motors")
            self.statusBox.setPlainText("Not connected to ground station motors")

        return

    def getGSLocation(self):
        # if the arduino is connected, this uses a GPS shield to request the coordinates of the ground station
        # it includes a check to ensure that the shield has a gps lock
        # if all conditions are met, the location of the ground station is set
        # if something fails, the function returns and nothing is set
        if self.arduinoConnected:
            check = self.GSArduino.warm_start()
            if not check:  # if the coords cannot be retrieved, return
                print("Failed to get GPS coords, please try again")
                self.statusBox.setPlainText("Failed to get GPS coordinates, please try again")
                return
            time.sleep(.25)

            GSCoords = self.GSArduino.req_GPS()
            self.GSArduino.print_GPS()
            self.GSLat = GSCoords[0]
            self.GSLong = GSCoords[1]
            self.GSAlt = GSCoords[2]

            self.GSLatBox.setPlainText(str(self.GSLat))
            self.GSLongBox.setPlainText(str(self.GSLong))
            self.GSAltBox.setPlainText(str(self.GSAlt))
        else:
            print("arduino not connected")
            self.statusBox.setPlainText("Arduino not connected")

        return

    def setGSLocation(self):
        # this ensures that the arduino is connected, and valid text is present in the gs location text boxes
        # if the values present can be converted to floats, the starting location of the gs is set
        try:
            if self.arduinoConnected:
                latStr = self.GSLatBox.toPlainText()
                latStr = latStr.strip()
                self.GSLat = float(latStr)

                print(self.GSLat)

                longStr = self.GSLongBox.toPlainText()
                self.GSLong = float(longStr)
                print(self.GSLong)

                altStr = self.GSAltBox.toPlainText()
                self.GSAlt = float(altStr)
                print(self.GSAlt)

                with open('gs_log.csv', mode='w', newline='') as gs_log:
                    gs_writer = csv.writer(gs_log, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

                    gs_writer.writerow([latStr, longStr, altStr])

                self.statusBox.setPlainText("Ground station location entered successfully!")
                self.GSLocationSet = True
            else:
                self.statusBox.setPlainText("Please connect arduino")
                self.GSLocationSet = False
        except ValueError:
            print("numbers only for GPS location (decimal degrees)")
            self.statusBox.setPlainText("Invalid GPS location entered. Please only enter numbers")

    def getStartingPos(self):
        # this makes a call to sunposition to calculate the azimuth and elevation of the sun at the current location
        # of the ground station
        # it populates the starting aziumth and elevation boxes
        if self.GSLocationSet:
            now = datetime.utcnow()
            az, elev = sunpos(now, self.GSLat, self.GSLong, self.GSAlt)[:2]  # discard RA, dec, H

            self.startingAzimuth = az
            self.startingElevation = elev

            self.startingAzimuthBox.setPlainText(str(az))
            self.startingElevationBox.setPlainText(str(elev))

        else:
            self.statusBox.setPlainText("Please set ground station location "
                                              "and point at the sun using solar sight")

        return

    def calibrate(self):
        # sends the GSArduino class the starting azimuth and elevation
        if self.arduinoConnected:
            try:
                startingAzimuthStr = self.startingAzimuthBox.toPlainText()
                startingAzimuth = float(startingAzimuthStr)
                print(startingAzimuth)

                startingElevationStr = self.startingElevationBox.toPlainText()
                startingElevation = float(startingElevationStr)
                print(startingElevation)

                self.GSArduino.calibrate(startingAzimuth, startingElevation)
                self.calibrated = True
                self.statusBox.setPlainText("Successfully calibrated!")
            except ValueError:
                print("numbers only for initial azimuth and elevation")
                self.statusBox.setPlainText("Invalid input for initial azimuth and elevation")
        else:
            print("not connected to arduino")
            self.statusBox.setPlainText("Not connected to arduino")

        return

    def returnToSun(self):
        if self.arduinoConnected and self.GSLocationSet and self.calibrated:
            now = datetime.utcnow()
            az, elev = sunpos(now, self.GSLat, self.GSLong, self.GSAlt)[:2]  # discard RA, dec, H

            self.GSArduino.move_position(az, elev)

            self.startingAzimuth = az
            self.startingElevation = elev

            self.startingAzimuthBox.setPlainText(str(self.startingAzimuth))
            self.startingElevationBox.setPlainText(str(self.startingElevation))
            self.statusBox.setPlainText("at new sun position")

        else:
            self.statusBox.setPlainText("Ensure that arduino is connected, GS location is set and calibration is set")
            print("Cannot point back at the sun")

        return

    def setPredictTrack(self):
        # sets the predict track bool variable
        # then calls the checkIfReady function to ensure all conditions to track have been met
        self.predictingTrack = True
        self.checkIfReady()
        return

    def checkIfReady(self):
        # this function ensures that all conditions to track have been met
        # if they have been, it calls the appropriate function to either start tracking with/without predictions
        if self.calibrated:
            print("Calibrated!")
        else:
            print("starting position not set")
            self.statusBox.setPlainText("Please set staring azimuth and elevation")

        if self.GSLocationSet:
            print("Ground station location set!")
        else:
            print("Ground Station Location not assigned")
            self.statusBox.setPlainText("Ground Station location not assigned")

        if self.IMEIAssigned:
            print("IMEI assigned")
        else:
            print("IMEI not assigned")
            self.statusBox.setPlainText("Please select a balloon")

        if self.arduinoConnected:
            print("Arduino connected!")
        else:
            print("Please connect to the Arduino")
            self.statusBox.setPlainText("Please connect to the Arduino")

        print("\n")

        if self.arduinoConnected and self.IMEIAssigned and self.calibrated and self.GSLocationSet:
            if self.predictingTrack:
                self.statusBox.setPlainText("Starting tracking with predictions!")
                self.callPredictTrack()
            else:
                self.statusBox.setPlainText("Starting tracking!")
                print("starting tracking!")
                self.callTrack()
                return True
        else:
            print("not ready to track yet")
            return False

    def callTrack(self):
        # sets up the qt thread to start tracking, and starts the thread
        self.tracking = True
        self.statusBox.setPlainText("Tracking!")
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.track)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)

        self.trackThread.start()

    def callPredictTrack(self):
        # sets up the qt thread to start tracking with predictions and starts the thread
        self.statusBox.setPlainText("Tracking with predictions!")
        print("In predictTrack call")
        self.tracking = True
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.predictTrack)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)

        self.trackThread.start()

    def stopTracking(self):
        # this stops the tracking thread, thus stopping the tracking
        if self.tracking:
            self.tracking = False
            self.predictingTrack = False
            self.startButton.setEnabled(True)
            self.predictionStartButton.setEnabled(True)
            self.calibrateButton.setEnabled(True)
            self.statusBox.setPlainText("tracking stopped")
        return

    def EStop(self):
        if self.arduinoConnected:
            self.GSArduino.sendEStop()
            self.stopTracking()

            self.statusBox.setPlainText("E-Stop triggered \n Please recalibrate before starting again")
            print("E-Stopped must recalibrate before starting tracking")

        return

    def displayCalculations(self, distance, azimuth, elevation):
        # this displays the outputs from the tracking threads on the GUI
        self.distanceDisplay.setPlainText(str(distance))
        self.azimuthDisplay.setPlainText(str(azimuth))
        self.elevationDisplay.setPlainText(str(elevation))
        return

class Worker1(QThread):
    ImageUpdate = pyqtSignal(QImage)
    def run(self):
        self.ThreadActive = True
        Capture = cv2.VideoCapture(streamLink)
        while self.ThreadActive:
            ret, frame = Capture.read()
            if ret:
                Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                FlippedImage = cv2.flip(Image, 1)
                ConvertToQtFormat = QImage(FlippedImage.data, FlippedImage.shape[1], FlippedImage.shape[0], FlippedImage.shape[1] * FlippedImage.shape[2] , QImage.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(1080, 720)
                self.ImageUpdate.emit(Pic)
            else:
                break
        Capture.release()
    def stop(self):
        self.ThreadActive = False
        self.quit()

class Worker2(QThread):
    packetNumber = pyqtSignal(list)
    def run(self):
        self.ThreadActive = True
        ser = serial.Serial( port = comport, baudrate = 57600, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout = 1 )
        while self.ThreadActive:
            y = ser.readline()
            global xx
            xx = y.decode('utf-8')
            xxx = str(xx)
            xxxx = xxx.split(",")
            self.packetNumber.emit(xxxx)


class Worker(QObject):
    # worker class to track without making the GUI hang
    finished = pyqtSignal()

    calcSignal = pyqtSignal(float, float, float)

    i = 0

    def track(self):
        # basic tracking algorithm
        # checks for updated position every 5 seconds
        # if a new position has been found, calculate the azimuth and elevation to point at the new location
        # send the motors a command to move to the new position
        timer = time.time() - 4
        while MainWindow.tracking:
            if (time.time() - timer) > 5:
                timer = time.time()
                Balloon_Coor = MainWindow.Balloon.get_coor_alt()
                if not Balloon_Coor:
                    pass
                else:
                    # note that trackMath takes arguments as long, lat, altitude
                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, Balloon_Coor[1],
                                              Balloon_Coor[0], Balloon_Coor[2])

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    print(str(self.i) + " Distance " + str(distance) + " Azimuth: " + str(newAzimuth) + ", Elevation: " + str(newElevation))

                    self.calcSignal.connect(MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    with open('gs_azimuth_log.csv', mode='a', newline='') as gs_log:
                        gs_writer = csv.writer(gs_log, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

                        gs_writer.writerow([newAzimuth, newElevation])
                        print("All done!")

                    self.i += 1

        print("All done!")
        self.finished.emit()  # same pycharm bug as above
        return

    def predictTrack(self):
        # check for new location from server
        # if the new location is still the same, go to prediction
        # if there is new location, go to that latest location

        # find the difference between the latest lat/long location and the one before the last one and time
        # using last vertical velocity/altitude, find difference between altitudes/new altitude after ~1 second
        # find the amount that the position is changing each ~second
        # input the new predicted lat/long/alt into math equations to get new azimuth/elevation
        # go to the predicted elevation/azimuth

        print("In predictTrack")

        timer = time.time()
        newestLocation = MainWindow.Balloon.get_coor_alt()
        oldLocation = MainWindow.Balloon.get_coor_alt()
        i = 1

        calculations = open("predictedOutput.csv", "w")
        csvWriter = csv.writer(calculations)
        calcFields = ["Distance", "Azimuth", "Elevation", "r/p"]
        csvWriter.writerow(calcFields)

        azimuthList = []
        elevationList = []

        while MainWindow.predictingTrack:
            if (time.time() - timer) > 1:
                timer = time.time()
                currData = MainWindow.Balloon.get_coor_alt()

                if newestLocation == currData:
                    # need to predict!
                    print("predicted output")
                    timeDelta = MainWindow.Balloon.getTimeDiff()
                    print("The time delta is: " + str(timeDelta))
                    latStep = (newestLocation[0] - oldLocation[0]) / timeDelta
                    longStep = (newestLocation[1] - oldLocation[1]) / timeDelta
                    altStep = (newestLocation[2] - oldLocation[2]) / timeDelta

                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt,
                                              newestLocation[1] + (i * longStep), newestLocation[0] + (i * latStep), newestLocation[2] + (i * altStep))

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    elevationList.append(newElevation)
                    azimuthList.append(newAzimuth)

                    # keep average of azimuth/elevations
                    # if new calculation is outlier, throw it out, don't go to new spot
                    # reset average between pings
                    # alternatively, implement some type of filter (savitzky golay, kalman, etc)

                    """
                    if newElevation > np.mean(elevationList) + (2 * np.std(elevationList)) or newElevation < np.mean(elevationList) - (2 * np.std(elevationList)) \
                            or newAzimuth > np.mean(azimuthList) + (2 * np.std(azimuthList)) or newAzimuth < np.mean(azimuthList) - (2 * np.std(azimuthList)):
                        print("outlier detected! ")
                        pass
                    else:
                        print("distance: " + str(distance))
                        print("elevation: " + str(newElevation))
                        print("azimuth: " + str(newAzimuth) + "\n")
                    """
                    self.calcSignal.connect(MainWindow.displayCalculations)
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "p"]
                    csvWriter.writerow(row)

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    i += 1

                else:
                    # go to the new actual spot
                    oldLocation = newestLocation
                    newestLocation = currData

                    # note that trackMath takes arguments as long, lat, altitude
                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, currData[1],
                                              currData[0], currData[2])

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    self.calcSignal.connect(MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    print("Got new real ping!")
                    print("distance: " + str(distance))
                    print("elevation: " + str(newElevation))
                    print("azimuth: " + str(newAzimuth) + "\n")

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "r"]
                    csvWriter.writerow(row)

                    i = 1
                    azimuthList = []
                    elevationList = []

        print("All done tracking with predictions! :)")
        calculations.close()
        self.finished.emit()
        return


if __name__ == "__main__":

    # standard pyqt5 main
    # sets up and shows the window

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = Window()
    # MainWindow.show()
    MainWindow.showMaximized()

    sys.exit(app.exec_())