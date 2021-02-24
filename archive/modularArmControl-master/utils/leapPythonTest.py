
import sys
sys.path.insert(0, "C:/Users/medgroup01/Documents/Julian/LeapSDK/LeapSDK/lib")
sys.path.insert(0, "C:/Users/medgroup01/Documents/Julian/LeapSDK/LeapSDK/lib/x64")
import Leap


def getController():
    controller = Leap.Controller()
    return controller

def getFrame(controller):
    frame = controller.frame()
    return frame

def main():
    print "nothing here"
    

if __name__ == "__main__":
    main()