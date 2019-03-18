import cv2
import numpy as np
import os
from multiprocessing import Queue
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import time
import serial
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM) 
pin = 4
#pin2=18
#GPIO.setup(pin2, GPIO.IN)
# choose BCM or BOARD  
GPIO.setup(pin, GPIO.IN)  # set a port/pin as an input  
ser=serial.Serial (port="/dev/ttyACM0",
baudrate=9600,
timeout=None,
GPIO.setmode(GPIO.BCM) 
pin = 4
#pin2=18
#GPIO.setup(pin2, GPIO.IN)
# choose BCM or BOARD  
GPIO.setup(pin, GPIO.IN)  # set a port/pin as an input  
ser=serial.Serial (port="/dev/ttyACM0",
baudrate=9600,
timeout=None,
write_timeout = 0)
ser.flushInput()
print('jai mata di')
s = ""
def labelize(img,size1,size2,scale_factor_distance=25,num_cluster=3,rand=6,batc$
    a = cv2.medianBlur(img,3)
    a = cv2.medianBlur(a,3)
    a = cv2.medianBlur(a,3)
    
    loc1=np.zeros(shape=[size1,size2])
    loc2=np.zeros(shape=[size1,size2])
    
    for y in range(size1):
        loc1[:,y]=(y+1)/scale_factor_distance
    for y in range(size2):
        loc2[y,:]=(y+1)/scale_factor_distance
    
    a=np.reshape(a,[-1,1])
    loc1=np.reshape(loc1,[-1,1])
    loc2=np.reshape(loc2,[-1,1])


    a=np.concatenate([a,loc1,loc2],axis=1)
    
    multiplier=int(np.floor(255/num_cluster))
    
    from sklearn.cluster import MiniBatchKMeans
    kmb=MiniBatchKMeans(n_clusters=num_cluster,batch_size=batch_size,rando$
    
    label=kmb.fit_predict(a)*multiplier
    label=label.reshape([size1,size2])
    return label

def segregate(label,strip_size,size1,size2):
    from scipy import stats
    ch=np.array(label)   
    ch=ch.reshape([size1,size2])
    x=ch[size1-strip_size:]
    mode=stats.mode(x.reshape([-1]))[0][0]
    ch[ch!=mode]=-1
    ch[ch==mode]=255
    ch=np.clip(ch,0,255)
def segregate(label,strip_size,size1,size2):
    from scipy import stats
    ch=np.array(label)   
    ch=ch.reshape([size1,size2])
    x=ch[size1-strip_size:]
    mode=stats.mode(x.reshape([-1]))[0][0]
    ch[ch!=mode]=-1
    ch[ch==mode]=255
    ch=np.clip(ch,0,255)

size1=200
size2=200
k_value=3
strip_size=75
path = Queue(maxsize = 10)
#cap=cv2.VideoCapture(0)
x=0
block = np.zeros((5,5))
windowsize_r = 5

#cmd='python3 training_chatbot.py'
#os.system('cd /home/pi/Desktop')
#a=os.system(cmd)
#if(GPIO.input(pin2)>0):
 #   ser.write(a)
vs=VideoStream(src=0,framerate=50).start()
#vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)
fps = FPS().start()

while True:
    #_,frame=cap.read()
    frame = vs.read()
    a=time.time()
    img1=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    img1=cv2.resize(img1,(size1,size2), interpolation = cv2.INTER_AREA)
    label=labelize(img1,num_cluster=k_value,size1=size1,size2=size2)
    ground=segregate(label,strip_size=strip_size,size1=size1,size2=size2)
    cv2.imshow('original_feed',np.array(img1, dtype = np.uint8 ))
    #cv2.imshow('label',np.array(label, dtype = np.uint8 ))
    cv2.imshow('ground',np.array(ground, dtype = np.uint8 ))
    print('frame rate ' ,1/(time.time()-a))
   # fps.update()
   # print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
   # print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    for y in range(5):
        for x in range(5):
            xx = 40*x
            yy = 200-40*y
            if( np.mean(ground[xx:xx+40, yy:yy+40]) > 150):
                block[x][y] = 1
    print(block)
    if(GPIO.input(pin)>0):
        string1=stringbanade(block.astype(int))
        ser.flushInput()
        #ser.write(string1)
       # print(string1)
        string1_encode=string1.encode()
        ser.write(string1_encode)

        print(string1_encode)

    if cv2.waitKey(1)==27:
        break
    x+=1
fps.stop()
ser.flush()
cv2.destroyAllWindows()
vs.stop()



