#from lab_2 import count
import numpy as np
import globals
def callback_1(self, data):
        global sound
        print("sound is ",sound)
        if sound==True:
            return
        if detect_noise(data):
            self.execute_behavior1(True)
            sound=True
            count+=1


def detect_noise(data):
    alert = False
    #print(np.average(np.absolute(data.data)), "            ",max(data.data))
    if np.average(np.absolute(data.data))>0.002 and max(data.data)>0.96:
        print("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOISE")
        alert=True
    return alert

def printing(ld):
      globals.count=globals.count+1
      print("I am here ",ld," count ",globals.count)
