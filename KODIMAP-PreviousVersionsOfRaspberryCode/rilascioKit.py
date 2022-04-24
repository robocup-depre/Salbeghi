import RPi.GPIO as GPIO
import time


class MotoriRK():
    def __init__(self):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.pinIN1 = 7
        self.pinIN2 = 8
        self.pinEN = 24
        self.pinF1 = 22
        #self.pinF2 = 23

        GPIO.setup(self.pinF1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pinIN1, GPIO.OUT)
        GPIO.setup(self.pinIN2, GPIO.OUT)
        GPIO.setup(self.pinEN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pinEN, 300)
        self.pwm.start(0)
        self.tempoClick = 0
        self.stato = 0
        self.rotazioni = 0
        self.setPWM(0)
        self.stato = 0
        self.oldstato = 0
        self.servoPIN = 12
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servoPIN, GPIO.OUT)
        self.servo = None
        self.freqBaseServo = 50
        self.freqServo = 450
        self.kServo = self.freqServo/self.freqBaseServo
        

    def initServo(self,posIniz):
        if(self.servo == None):
            self.servo = GPIO.PWM(self.servoPIN, self.freqServo) # GPIO 17 for PWM with 50Hz
            self.servo.start(posIniz*self.kServo)
       

    def setServo(self,pos):
        #print("                   SET SERVO",pos)
        if(self.servo!=None):
            
            self.servo.ChangeDutyCycle(pos*self.kServo)
            

    def stopServo(self):
        if(self.servo!=None):
            self.servo.ChangeDutyCycle(0)
        #self.servo.stop()
        

    def Aggiorna(self):
        self.stato = GPIO.input(self.pinF1)
        if(self.stato== 1 and self.oldstato == 0):
            self.tempoFine = time.time()
            self.rotazioni +=1
        self.oldstato = self.stato

    def MotRuota(self, pwm,  nrotazioni,tmax = 1):
        self.rotazioni = 0
        self.setPWM(pwm)
        self.tempoFine = time.time()
        while ( not ( self.rotazioni >= nrotazioni or (time.time()-self.tempoFine)>tmax)):
            self.Aggiorna()
        self.setPWM(0)
    

    def MotAvvio(self, pwm, rmin = 20, tmin = 0.2, tmax = 1):
        self.rotazioni = 0
        self.setPWM(pwm)
        self.tempoFine = time.time()
        while( not ((time.time()-self.tempoFine)>tmin and (self.rotazioni>rmin or (time.time()-self.tempoFine)>tmax)) ):
            self.Aggiorna()
        
        self.setPWM(0)
        return self.rotazioni

    def clean(self):
        if(self.pwm!=None):
            self.pwm.stop()
        if(self.servo!=None):
            self.servo.stop()
        GPIO.cleanup()
        
    def MotStop(self):
        self.setPWM(0)
        
    def setPWM(self,pwm):
        #GPIO.setmode(GPIO.BCM)
        if (pwm>0):
            GPIO.output(self.pinIN1, GPIO.LOW)
            GPIO.output(self.pinIN2, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(pwm)
        elif (pwm==0):
            GPIO.output(self.pinIN1, GPIO.HIGH)
            GPIO.output(self.pinIN2, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(pwm)
        else:
            GPIO.output(self.pinIN1, GPIO.HIGH)
            GPIO.output(self.pinIN2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(-pwm)

if __name__ == '__main__':
    rk = MotoriRK()
    try:
        while(1):
            rk.MotAvvio(100)
            rk.MotAvvio(-100)
    except:
        rk.MotStop()
    
