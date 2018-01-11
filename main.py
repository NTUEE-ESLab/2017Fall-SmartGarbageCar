from picamera.array import PiRGBArray
from picamera import PiCamera
import warnings
import datetime
import imutils
import time
import cv2
import numpy as np

from BLEConnection import writeEvent
from BLEConnection import myThread

warnings.filterwarnings('ignore')

cam_warmup_time = 2.5
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 16
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(640, 480))

print('[INFO] warming up ...')
time.sleep(cam_warmup_time)

print('[INFO] Connecting to the stupid car ...') 
e = writeEvent()
thread = myThread(e) # The address is implicitly defined in the class

def send_message(sequences):
    total = ""
    for direction, time in sequences:
        if direction in ['F', 'B']:
            total += direction + '{:04d}'.format(time)
        else:
            total += direction + '{}'.format(time)
        
    if total != "":
        e.set_str(total)




class Object:
    def __init__(x, y, w, h):
        self.x = x
        self.y = y 
        self.w = w
        self.h = h

    def getDistance(self, obj):
        centerx = self.x + self.w/2
        centery = self.y + self.h/2
        objx = obj.x + obj.w/2
        objy = obj.y + obj.h/2

        return ((centerx - objx) ** 2 + (centery - objy) ** 2) ** 0.5


def FOD():
    obj_list = []
    last_seen = []
    bg_filter = cv2.bgsegm.createBackgroundSubtractorMOG()
    frame_i = 0
    for f in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        frame = f.array
        frame_i += 1
        frame = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        mask = bg_filter.apply(gray)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        for c in cnts:
            if cv2.contourArea(c) < 500:
                continue
            
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectange(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
            new_obj = Object(x, y, w, h)
            
            for i, (obj, count, _) in enumerate(last_seen):
                if new_obj.distance(obj) < 10:
                    print('Found someone like me')
                    last_seen[i] = (new_obj, count+1, frame_i)
                    break
            else:
                last_seen.append((new_obj, 1, frame_i))
        
        # Filter the stale records
        last_seen = list(filter(lambda x: frame_i - x[2] < 10, last_seen))

        # Add last seen long enough to obj_list
        for obj, count, _ in last_seen:
            if count > 2:
                for old_obj in obj_list:
                    if obj.distance(old_obj) < 10:
                        break
                else:
                    obj_list.append(obj)

        for obj in obj_list:
            cv2.rectange(frame, (obj.x, obj.y), (obj.x+obj.w, obj.y+obj.h), (0, 255, 0), 2)

        cv2.imshow('ORIGINAL', frame)
        cv2.imshow('MASKED', mask)

        rawCapture.truncate(0)


def init():
    avg = None # This is the average frame
    frame_i = 0
    
    for f in camera.capture_continuous(rawCapture, format='bgr', use_video_port=T):
        frame = f.array
        frame_i += 1
        timestamp = datetime.datetime.now()
        text = "Unoccupied"
    
        frame = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
    
        if avg is None:
            print('[INFO] starting background model...')
            avg = gray.copy().astype('float')
            rawCapture.truncate(0)
            
            one_turn = [('R' , 1)] * 4
            send_message(one_turn)
            continue
    
        cv2.accumulateWeighted(gray, avg, 0.5)
        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
    
        thresh = cv2.thresold(frameDelta, 5, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    
        for c in cnts:
            if cv2.contourArea(c) < 1000:
                continue
    
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
            text = "Occupied"
                
            new_obj = Object(x, y, w, h)
            
            for i, (obj, count, _) in enumerate(last_seen):
                if new_obj.distance(obj) < 10:
                    print('Found someone like me')
                    last_seen[i] = (new_obj, count+1, frame_i)
                    break
            else:
                last_seen.append((new_obj, 1, frame_i))
    
        ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
        cv2.putText(frame, "Room Status: {}".format(text), (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.35, (0, 0, 255), 1)
    
        cv2.imshow("Security Feed", frame)
        key = cv2.waitKey(1) & 0xFF
    
        if key == ord('q'):
            break
        
        rawCapture.truncate(0)

def calibration():
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((9*7,2), np.float32)
    objp[:,:] = np.mgrid[0:7,0:9].T.reshape(-1,2)
    objp *= 23
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    for f in camera.capture_continuous(rawCapture, format='bgr'):
        frame = f.array
        img = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,9),None)
        
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
        
            corners2 = cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
            imgpoints.append(corners2)
        
            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,9), corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey()
            rawCapture.truncate(0)
            break
        else:
            print("Not found")

        rawCapture.truncate(0)

    H, mask = cv2.findHomography(objpoints[0], imgpoints[0])
    
    cv2.destroyAllWindows()

    return H 

if __name__  == '__main__':
    mode = 'detection_only'
    if mode == 'calibration':
        H = calibration()
        np.save('homo.npy', H)
    elif mode == 'detection_only':
        H = np.load('homo.npy')
        FOD()

thread.join()
