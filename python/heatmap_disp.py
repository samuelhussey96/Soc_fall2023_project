import serial
import time
import struct
import numpy as np
import cv2
from cv2 import *

class Heatmap:
    def __init__(self):
        self.heatmap = np.zeros(shape=(4,4))
        self.baseline = np.zeros(shape=(4,4))
        self.baseline_frame_count = 0
        
    def plot_heatmap(self, mat):
        """ Plots Heatmap with OpenCV. """

        heatmap_img = np.zeros((mat.shape[0], mat.shape[1], 3), dtype=np.uint8)

        mat_max = 1 # dB
        mat_min = -8 # dB
        #print(mat)
        for i in range(mat.shape[0]):
            for j in range(mat.shape[1]):
                heatmap_img[i,j,:] = np.interp(mat[i,j],[mat_min, mat_max],[255,0])

        img_resize = cv2.resize(heatmap_img, None,fx=170, fy=170, interpolation = cv2.INTER_LINEAR)
        img_color = cv2.applyColorMap(img_resize, cv2.COLORMAP_JET)
        
        cv2.imshow("Touch Display", img_color)

    def convert_dB(self, val):
        return 20*np.log10(val)

    def do_baseline(self):
        self.heatmap -= self.baseline

    def process_frame(self):
        if self.baseline_frame_count == 0:
            self.baseline += self.heatmap
            self.baseline_frame_count += 1
        elif self.baseline_frame_count == 4:
            self.baseline /= 4
            self.baseline_frame_count += 1
        else:
            self.do_baseline()
            self.plot_heatmap(self.heatmap)
        
        self.heatmap = np.zeros(shape=(4,4))

def main():
    serialPort = serial.Serial(
        port="COM5", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
    )
    hm = Heatmap()
    
    serial_str = ""
    store_data = False
    i = 0
    j = 0
    while (True):
        if serialPort.in_waiting > 0:
            
            # Read data out of the buffer until \n is found
            serial_str = serialPort.readline()  
            val = serial_str.decode("Ascii")

            if val[0] == 'x':
                store_data = True
                continue
            if val[0] == 'z':
                store_data = False
                
                hm.process_frame()
                
                continue
            
            if store_data == True:
                try:
                    hm.heatmap[i,j] = hm.convert_dB(int(val)+1)
                    j += 1
                    if j >= 4:
                        j = 0
                        i += 1
                        if i >= 4:
                            i = 0
                except Exception as e:
                    print(e)

        pressedKey = cv2.waitKey(1)
        if pressedKey == ord('b'):
            hm.baseline = np.zeros(shape=(4,4))
            hm.baseline_frame_count = 0
        elif pressedKey == ord(' '):
            break
        
    
if __name__ == "__main__":
    main()