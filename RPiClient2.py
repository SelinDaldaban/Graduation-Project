import cv2
import numpy as np
import time
import RPi.GPIO as GPIO          
from time import sleep
import socket
import sys
from threading import Thread
import pigpio
import Adafruit_DHT

GPIO.setwarnings(False)
ledPin=17
BtnPin=3
motor = 23
en = 24
mz=5

GPIO.setmode(GPIO.BCM)

GPIO.setup(motor,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.output(motor,GPIO.LOW)
GPIO.setup(mz,GPIO.IN)
GPIO.setup(ledPin,GPIO.OUT)
GPIO.output(ledPin,GPIO.LOW)
GPIO.setup(3,GPIO.IN)

red_count=0
blue_count=0

p=GPIO.PWM(en,100)
p.start(0)
GPIO.output(ledPin,GPIO.HIGH)

class Servo():
    
    def __init__(self,name,pin,pi,f,angle,time=0.003):
       self.name =name
       self.pi=pi
       self.pin=pin
       self.f=f
       self.angle=angle
       self.time=time
       self.pi.set_mode(pin,pigpio.OUTPUT)
       self.pi.set_PWM_frequency(self.pin,self.f)

       
    def set_angle(self, angle):
        if angle>self.angle:
            while(angle>self.angle):
                self.pi.set_servo_pulsewidth(self.pin,self.angle)
                self.angle+=1
                time.sleep(self.time)
        elif angle<self.angle:
            while(angle<self.angle):
                self.pi.set_servo_pulsewidth(self.pin,self.angle)
                self.angle-=1
                time.sleep(self.time)
       
        self.pi.set_servo_pulsewidth(self.pin,self.angle)
        print(self.name,self.pin,self.angle)
        
class Arm():
    
    def __init__(self):
        self.pi = pigpio.pi("127.0.0.1",8888)
        self.f=33
        self.omuz = Servo("omuz",21,self.pi,self.f,800)
        self.dirsek = Servo("dirsek",13,self.pi,self.f,1700)
        self.govde = Servo("govde",20,self.pi,self.f,1150)
        self.agiz= Servo("agiz",26,self.pi,self.f,1100)
        self.taban= Servo("taban",16,self.pi,self.f,1400)
        
    def start(self):
        print("Starting robot arm")
        self.taban.set_angle(1400)
        self.agiz.set_angle(1100)
        self.dirsek.set_angle(1700)
        self.omuz.set_angle(850)
        self.govde.set_angle(1150)

    def hold_object(self):
        self.agiz.set_angle(700)
        self.dirsek.set_angle(2100)
        self.agiz.set_angle(1100) #agiz kapali
        self.dirsek.set_angle(1100)
        
    def position(self,pos):
        if pos == "right":
            self.taban.set_angle(750)
            self.dirsek.set_angle(2000)
            self.agiz.set_angle(800)
            self.dirsek.set_angle(1100)
        elif pos == "left":
             self.taban.set_angle(2023)
             self.dirsek.set_angle(2000)
             self.agiz.set_angle(800)
             self.dirsek.set_angle(1100)
             
    def product(self,value):
        global red_count,blue_count
        red_count=0
        blue_count=0
        count=0
        GPIO.output(motor,GPIO.HIGH)
        p.ChangeDutyCycle(50)
        print("Product value: ",value)
        blue_lower=np.array([99,115,150],np.uint8)
        blue_upper=np.array([110,255,255],np.uint8)
        red_lower = np.array([0,50,50])
        red_upper = np.array([10,255,255])
        kernelOpen=np.ones((5,5))
        kernelClose=np.ones((20,20))
        font=cv2.cv.InitFont(cv2.cv.CV_FONT_HERSHEY_SIMPLEX,2,0.5,0,3,1)
        global cam
        cam= cv2.VideoCapture(0)
        while count< int(value):
            ret,img=cam.read()
            imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            maskBlue=cv2.inRange(imgHSV,blue_lower,blue_upper)
            maskRed=cv2.inRange(imgHSV,red_lower,red_upper)
            maskOpen=cv2.morphologyEx(maskBlue,cv2.MORPH_OPEN,kernelOpen)
            maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
            maskFinal=maskClose
            conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            maskOpen2=cv2.morphologyEx(maskRed,cv2.MORPH_OPEN,kernelOpen)
            maskClose2=cv2.morphologyEx(maskOpen2,cv2.MORPH_CLOSE,kernelClose)
            maskFinal2=maskClose2
            conts2,h=cv2.findContours(maskFinal2.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            cv2.imshow("cam",img)
            if len(conts)==1:
                while True:
                    val=GPIO.input(mz)
                    if val == 0:
                        GPIO.output(motor,GPIO.LOW)
                        p.ChangeDutyCycle(0)
                        self.hold_object()
                        self.position("right")
                        self.start()               
                        GPIO.output(motor,GPIO.HIGH)
                        p.ChangeDutyCycle(50)
                        count+=1
                        break
                        
            if len(conts2)==1:
                while True:
                    val=GPIO.input(mz)
                    if val == 0:
                        GPIO.output(motor,GPIO.LOW)
                        p.ChangeDutyCycle(0)
                        self.hold_object()
                        self.position("left")
                        self.start()                
                        GPIO.output(motor,GPIO.HIGH)
                        p.ChangeDutyCycle(50)
                        count+=1
                        break

        cv2.destroyAllWindows()
        cam.release()
        GPIO.output(motor,GPIO.LOW)
        p.ChangeDutyCycle(0)             

class Client(object):
    def __init__(self,ip,port,name):
        self.ip = ip
        self.port = port
        self.name=name
        ADDR = ( self.ip , self.port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(ADDR)
        self.connect="Not Connected"
        
    def start(self):
        try:
            self.arm=Arm()
            self.arm.start()
            receive_thread = Thread(target=self.receive)
            send_thread = Thread(target=self.send)
            receive_thread.start()
            send_thread.start()
            self.connect="Connected to the Factory"

        except:
            print("Thread did not start." + str(sys.exc_info()))
            self.connect="Not Connected"
            self.socket.close()
    def receive(self):
        
        while True:
            try:
                msg = self.socket.recv(1024).decode("utf8")            
                if "Connect" in msg:
                    print(msg)
                    msg=self.connect
                    self.socket.send(bytes(msg.encode('utf-8')))

                elif self.connect== "Connected to the Factory":

                    if "-" in msg:
                        message = msg.split("-")
                        tip=message[0]
                        deger=message[1]               
                        print("Type:",tip)
                        print("Value: ",deger)
                        if tip == "led":
                            if deger=="true":
                                GPIO.output(ledPin,GPIO.HIGH)
                                self.socket.send(bytes("Success led on".encode('utf-8')))
                            elif deger=="false":
                                GPIO.output(ledPin,GPIO.LOW)                               
                                self.socket.send(bytes("Success led off".encode('utf-8')))                  
                        elif tip == "power":
                            if deger=="true":
                                GPIO.output(motor,GPIO.HIGH)
                                p.ChangeDutyCycle(50)
                                self.arm.start()
                                self.socket.send(bytes("Success power on".encode('utf-8')))
                            elif deger=="false":
                            #arm.pi.stop()
                                GPIO.output(motor,GPIO.LOW)
                                self.socket.send(bytes("Success power off".encode('utf-8')))
                                         
                     
                        elif tip == "product":
                            arm_thread = Thread(target=self.arm.start)
                            arm_thread.start()
                            if int(deger)>0:
                                count=1
                                while count<=int(deger):
                                    msg=str(count)+". product is being packed"
                                    self.socket.send(bytes(msg.encode('utf-8')))
                                    self.arm.product(1)
                                    count+=1
                                msg="Packaging of the products is completed"
                                self.socket.send(bytes(msg.encode('utf-8')))                
                     
                else:
                    print(msg)
                      
            except Exception as err:
                print("ben recieveden geliyorum :", err)
                self.socket.close()
                sys.exit(1)

    def send(self):
        msg=self.name
        self.socket.send(bytes(msg.encode('utf-8')))
        isinempin=12
        gaspin=3
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, isinempin)
        isinem='Temp: {0:0.1f} C  Humidity: {1:0.1f} %'.format(temperature, humidity)
        gas=GPIO.input(gaspin)
        msg=str(gas) + "/" + isinem
        self.socket.send(bytes(msg.encode('utf-8')))
        while True:
            humidity2, temperature2 = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, isinempin)
            gas2=GPIO.input(gaspin)
            if gas != gas2 or humidity != humidity2 or temperature != temperature2:
                isinem='Temp: {0:0.1f} C  Humidity: {1:0.1f} %'.format(temperature2, humidity2)
                msg=str(gas2) + "/" + isinem
                self.socket.send(bytes(msg.encode('utf-8')))
                gas=gas2
                humidity=humidity2
                temperature=temperature2
  
	
myclient= Client("3.18.221.185",4000,"raspiclient")
myclient.start()




