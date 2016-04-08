#!/usr/bin/env python
'''
Created on November 20, 2010

@author: Dr. Rainer Hessmer
'''
import threading
import serial
from cStringIO import StringIO
import time
import rospy

def _OnLineReceived(line):
    print line

class SerialDataGateway(object):
    '''
    Helper class for receiving lines from a serial port
    '''

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, lineHandler=_OnLineReceived):
        '''
        Initializes the receiver class.
        port: The serial port to listen to.
        receivedLineHandler: The function to call when a line was received.
        '''
        self._Port = port
        self._Baudrate = baudrate
        self.ReceivedLineHandler = lineHandler
        self._KeepRunning = False

    def Start(self):
        try:
            self._Serial = serial.Serial(port=self._Port, baudrate=self._Baudrate, timeout=1)
            time.sleep(1)   # Wait Arduino reset! This is fucking important!
        except:
            rospy.loginfo("SERIAL PORT Start Error")
            raise
        self._KeepRunning = True
        self._ReceiverThread = threading.Thread(target=self._Listen)
        self._ReceiverThread.setDaemon(True)
        self._ReceiverThread.start()

    def Stop(self):
        rospy.loginfo("Stopping serial gateway")
        self._KeepRunning = False
        time.sleep(.1)
        try:
            self._Serial.close()
        except:
            rospy.loginfo("SERIAL PORT Stop Error")
            raise

    def _Listen(self):
        stringIO = StringIO()
        while self._KeepRunning:
            try:
                data = self._Serial.read()
            except:
                rospy.loginfo("SERIAL PORT Listen Error")
                raise
            if data == '\r':
                pass
            if data == '\n':
                self.ReceivedLineHandler(stringIO.getvalue())
                stringIO.close()
                stringIO = StringIO()
            else:
                stringIO.write(data)

    def Write(self, data):
        #AttributeError: 'SerialDataGateway' object has no attribute '_Serial'
        try:
            # print("serial write : " + data)
            self._Serial.write(data)
        except AttributeError:
            rospy.loginfo("SERIAL PORT Write Error")
            raise

if __name__ == '__main__':
    dataReceiver = SerialDataGateway("/dev/ttyACM0", 9600)
    dataReceiver.Start()
    n, lst = 0,'abcde'
    while True:
        dataReceiver.Write(lst[n])
        n += 1
        n %= 5
        time.sleep(0.1)
    dataReceiver.Stop()

    #dataReceiver.Stop()
