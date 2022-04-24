import time
import RPi.GPIO as GPIO
import board
import busio
import smbus2 as smbus

import adafruit_vl6180x
import bno
from Tpa81Lib import Tpa81



class sensore:
    pin = 0
    ind = 0
    num = 0
    nome = ''
    misura = None
    disponibile = 1
    def definisci(self,numero,nome, pin = -1, indirizzo=-1):
        self.pin=pin
        self.indirizzo = indirizzo
        self.num = numero
        self.nome = nome
        
class sensori:

    def __init__(self):

        self.incPiano=0 
        self.incSalita =1
        self.incDiscesa =2
        self.incErrore = 3
        self.inc_old = self.incPiano

        self.giroscopio = sensore()
        self.giroscopio.definisci(0, nome = 'gyro', pin = 20, indirizzo = 0x29)
        
        self.ledCamere = sensore()
        self.ledCamere.definisci(0, nome = 'ledCamere', pin = 17)
        
        self.asseZ = 0
        self.asseY = 1
        self.asseX = 2
        
        self.giroscopio.correzioni = [0]*3
        self.giroscopio.valore = [0]*3
        self.giroscopio.oldValore = [0]*3
        
        self.tDx = 0
        self.tSx = 1

        self.temperaturaNum = 2
        
        self.temperatura = [sensore() for i in range(self.temperaturaNum)] #[sensore()]*self.temperaturaNum
        self.temperatura[self.tDx].definisci(numero = self.tDx, nome = "tempDx", indirizzo = 0x68)
        self.temperatura[self.tSx].definisci(numero = self.tSx, nome = "tempSx", indirizzo = 0x6f)
        
        self.lAvCe = 0
        self.lAvDx = 1
        self.lAvSx = 2
        self.lDiDx = 3
        self.lDiSx = 4

        self.laserNum = 5
        
        self.laser = [sensore() for i in range(self.laserNum)] #[sensore()]*self.laserNum
        self.laser[self.lAvCe].definisci(numero = self.lAvCe, nome = "lAvCe", pin = 10, indirizzo = 0x20)
        self.laser[self.lAvDx].definisci(numero = self.lAvDx, nome = "lAvDx", pin = 27, indirizzo = 0x22)
        self.laser[self.lAvSx].definisci(numero = self.lAvSx, nome = "lAvSx", pin = 25, indirizzo = 0x24)
        self.laser[self.lDiDx].definisci(numero = self.lDiDx, nome = "lDiDx", pin = 9, indirizzo = 0x26)
        self.laser[self.lDiSx].definisci(numero = self.lDiSx, nome = "lDiSx", pin = 11, indirizzo = 0x28)

        for i in range(self.laserNum):
            print(i, self.laser[i]. nome, self.laser[i].pin, self.laser[i].indirizzo)
        self.fAvCe = 4
        self.fAvDx = 3
        self.fAvSx = 2
        self.fDiDx = 1
        self.fDiSx = 0
        
        self.finecorsaNum = 5
        
        self.finecorsa = [sensore() for i in range(self.finecorsaNum)]  #[sensore()]*self.finecorsaNum
        self.finecorsa[self.fDiSx].definisci(numero = self.fDiSx, nome = "fDiSx", pin = 26)
        self.finecorsa[self.fDiDx].definisci(numero = self.fDiDx, nome = "fDiDx", pin = 13)
        self.finecorsa[self.fAvSx].definisci(numero = self.fAvSx, nome = "fAvSx", pin = 6)
        self.finecorsa[self.fAvDx].definisci(numero = self.fAvDx, nome = "fAvDx", pin = 16)
        self.finecorsa[self.fAvCe].definisci(numero = self.fAvCe, nome = "fAvCe", pin = 5)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for ind in range(self.finecorsaNum): #GPIO finecorsa
            GPIO.setup(self.finecorsa[ind].pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
 
        GPIO.setup(self.giroscopio.pin,GPIO.OUT)  # GPIO giroscopio
        
    
    def InitLedCamera(self):
        GPIO.setup(self.ledCamere.pin,GPIO.OUT)
        GPIO.output(self.ledCamere.pin,GPIO.LOW) 
    def AccendiLedCamera(self):
        GPIO.output(self.ledCamere.pin,GPIO.HIGH) 
    def SpegniLedCamera(self):
        GPIO.output(self.ledCamere.pin,GPIO.LOW) 
        
   
    def ReadEndRun(self,pos):
        if(GPIO.input(self.finecorsa[pos].pin)==0 ):
            return 1
        else:
            return 0

    def PrintEndRun(self):
        for ind in range(self.finecorsaNum):
            print(self.finecorsa[ind].nome,self.ReadEndRun(ind),end=' -')
        print(" ")

        
    #LASER
    def InitLaser(self):
        
        GPIO.output(self.giroscopio.pin,GPIO.LOW)  #spengo giroscopio, per poter inizializzare i laser
        time.sleep(0.01)
        
        for index in range(self.laserNum): #inizializzazione GPIO laser
            GPIO.setup(self.laser[index].pin,GPIO.OUT)
            GPIO.output(self.laser[index].pin,GPIO.LOW)
            
        i2c = busio.I2C(board.SCL, board.SDA)

        for index in range(self.laserNum):
            GPIO.output(self.laser[index].pin,GPIO.HIGH) #attivo singolo laser
            print("start sensor", self.laser[index].nome)
            try:
                sensor = adafruit_vl6180x.VL6180X(i2c)
                sensor._write_8(0x212, self.laser[index].indirizzo) #cambio indirizzo singolo laser
            except Exception as e:
                print("sensore",self.laser[index].nome,"gia presente")

            try:
                self.laser[index].misura = adafruit_vl6180x.VL6180X(i2c, self.laser[index].indirizzo) #inizializzo singolo laser e lo aggiungo alla lista
            except Exception as e:
                print("ERRORE sensore",self.laser[index].nome)
                self.laser[index].disponibile=0

    def ReadLaser(self,index,n=0): # non inserire n nella chiamta alla funzione
        if(self.laser[index].disponibile):
            try:
                dato=(self.laser[index].misura.range)/10 #provo a leggere il sensore
                if(dato==0): #se è 0 (la distaza minima misurabile dal ribot di un muro è circa 2cm)
                    dato=25 #viene messo alla distanza massima per evitare problemi di misurazione sopra i 25cm
            except Exception as e:
                dato = -1
                
            if(dato==-1):
                print("errore laser index:",index,"num:",n)
                if(n<5): #se il numero di errori è minore di 5 allora si continua a riprovare 
                    n+=1 #si aumenta il numero di errori
                    time.sleep(0.1)
                    dato = self.ReadLaser(index,n) #richiamo la funzione stessa passando il numero di errori
                else: #se non funziona dopo 5 volte allora si ritorna il valore -1
                    dato=-2
        else:
            dato = -3
        return dato

    #TPA81
    def InitTPA81(self):
        for ind in range(self.temperaturaNum):
            try:
                self.temperatura[ind].misura = Tpa81(self.temperatura[ind].indirizzo)
            
            except Exception as e:
                print("TPA:", self.temperatura[ind].nome, "NON disponibile")
                self.temperatura[ind].disponibile = 0
            
    def Temperatura(self,index,n=0):# non inserire n nella chiamta alla funzione
        if(self.temperatura[index].disponibile):
            try:
                dato = self.temperatura[index].misura.getThermalData()
                #print(dato) 
                dato = (dato[2]+dato[3]+dato[4]+dato[5])/4
            
            except Exception as e:
                '''
                print("errore TPA81 ",self.temperatura[index].nome," num:",n)
                if(n<5):
                    n+=1
                    time.sleep(0.1)
                    dato = self.Temperatura(index,n)
                else:
                '''
                dato=-1
            
        else:
            dato = -2
        return dato

    #GIROSCOPIO
    def InitBNO(self):
        try:
            GPIO.output(self.giroscopio.pin,GPIO.HIGH) #accendo giroscopio
            time.sleep(0.5)
            self.giroscopio.misura = bno.BNO055() #inizializzazione
            if self.giroscopio.misura.begin() is not True:
                print("Error initializing BNO")
                #exit()
            time.sleep(0.2)
            self.giroscopio.misura.setExternalCrystalUse(True)
            #CALIBRAZIONE
        except Exception as e:
            print("ERRORE INIZIALIZZAZIONE BNO")
        try:
            print("calibrazione BNO")
            c1, c2, c3, c4 = 0,0,0,0
            timeStartCal = time.time()
            while(not((c1>1 and c2>1) or (time.time()-timeStartCal)>20)):  #i due valori devono essere maggiori di 1 per una parziale calibrazione
                                                                           # o tempo massimo di 20 secondi
                c1, c2, c3, c4 = self.giroscopio.misura.getCalibration() # leggo i valori
                time.sleep(0.05)
            print("fine init BNO t:",round(time.time()-timeStartCal,3))
        except Exception as e:
            print("ERRORE CALIBRAZIONE BNO")
            
        self.giroscopio.valore = list(self.Giroscopio(1)) #prendo i primi valori del giroscopio
        for ind in range(3):
            self.giroscopio.correzioni[ind]=self.giroscopio.valore[ind]

        self.giroscopio.oldValore = [0]*3
        print("GYRO REALI:",self.Giroscopio(1),"CORRETI:",self.Giroscopio(0), "CORREZIONI:",self.giroscopio.correzioni)
        
    def ResettaBN0(self):
        GPIO.output(self.giroscopio.pin,GPIO.LOW) #viene spento 
        time.sleep(0.5)
        self.InitBNO()
        
        self.giroscopio.valore = self.ReadGiro() #leggo i nuovi valori

        for ind in range(3):
            self.giroscopio.correzioni[ind]=self.giroscopio.valore[ind]-self.giroscopio.oldValore[ind]
            
    def ReadGiro(self):
        try:
            self.giroscopio.valore = list(self.giroscopio.misura.getVector(bno.BNO055.VECTOR_EULER)) # leggo i valori del giroscopio
            prove = 0
            while(prove<10 and not(self.giroscopio.valore[self.asseZ] >= 0 and self.giroscopio.valore[self.asseZ]<360 and self.giroscopio.valore[self.asseY] >= -180 and self.giroscopio.valore[self.asseY]<=180)): #riprovo se c'è qualche errore 
                #print("errore valore gyro",self.giroscopio.valore)
                time.sleep(0.1)
                self.giroscopio.valore = list(self.giroscopio.misura.getVector(bno.BNO055.VECTOR_EULER))
                prove += 1
                
        except:
            time.sleep(0.02)
            self.giroscopio.contatoreErrore+=1
            print("errore lettura     #######, num:",self.giroscopio.contatoreErrore)
            if(self.giroscopio.contatoreErrore == 5): #se ci sono state già 5 prove di leggere il giroscopio - viene resettato completamente
                print("RESET BNO !!!")
                self.giroscopio.valore = [-2000,-2000,-2000]
                '''
                GPIO.output(self.giroscopio.pin,GPIO.LOW) #viene spento 
                time.sleep(0.5)
                self.InitBNO()
                
                self.giroscopio.valore = self.ReadGiro() #leggo i nuovi valori

                for ind in range(3):
                    self.giroscopio.correzioni[ind]=self.giroscopio.valore[ind]-self.giroscopio.oldValore[ind]
                    
                '''
            elif(self.giroscopio.contatoreErrore > 10): #se ho provato più di 10 volte 
                print("HAI BRUCIATO IL BNO MONA!!")
                self.giroscopio.valore = [-1000,-1000,-1000]
            else:
                self.giroscopio.valore = self.ReadGiro() #riprovo la misura

            
        
        return self.giroscopio.valore
            
    def Giroscopio(self,mod):
        self.giroscopio.contatoreErrore = 0
        self.giroscopio.valoriIN = self.ReadGiro()
        if(self.giroscopio.valoriIN[0]<999):
            if( mod==0): #modalità che ritorna i valori corretti
                for ind in range(3):
                    self.giroscopio.oldValore[ind] = self.giroscopio.valoriOUT[ind] = self.giroscopio.valoriIN[ind] - self.giroscopio.correzioni[ind]
                
                if(self.giroscopio.valoriOUT[self.asseZ]<0):
                    self.giroscopio.valoriOUT[self.asseZ]+=360
                if(self.giroscopio.valoriOUT[self.asseZ]>=360):
                    self.giroscopio.valoriOUT[self.asseZ]-=360

                self.giroscopio.valoriOUT[self.asseY]*=-1
            else: #modalità che ritorna i valori puri
                self.giroscopio.valoriOUT = self.giroscopio.valoriIN
        else:
            self.giroscopio.valoriOUT = self.giroscopio.valoriIN
        return tuple(self.giroscopio.valoriOUT)

    def set_correzione(self, corr, pos):
        self.giroscopio.correzioni[pos] = corr #setto le correzioni calcolate esternamente

    def Inclinazione(self,inc):
     
        #_, inc, _ = self.Giroscopio(0)   #leggo valore asse Y giroscopio  
        if(inc> 35 or inc< -35):  
            cas=self.incErrore 
        elif(inc>20 or (inc > 10 and self.inc_old == self.incSalita)): #10-5
            cas = self.incSalita
        elif(inc<-20 or (inc< -10 and self.inc_old == self.incDiscesa)):
            cas = self.incDiscesa
        else:
            cas=self.incPiano
        self.inc_old=cas
    
        return cas

    def clean(self): #funzione da richiamare per chiudere la libreria
        GPIO.cleanup()

if __name__ == '__main__':
    
    sensori=sensori()
    
    sensori.InitLaser()
    sensori.InitBNO()
    sensori.InitTPA81()
    sn = sensori
    print("while")
    while True:
        try:
            '''
            t = time.time()
            d_avdx = sn.ReadLaser(sn.lAvDx)
            d_avsx = sn.ReadLaser(sn.lAvSx)
            d_didx = sn.ReadLaser(sn.lDiDx)
            d_disx = sn.ReadLaser(sn.lDiSx)
            d_avfr = sn.ReadLaser(sn.lAvCe)
            diff = time.time()-t
            print(d_avdx,d_avsx,d_didx,d_disx,d_avfr,sensori.Giroscopio(0))
            '''
            #print(sensori.Giroscopio(0), sensori.giroscopio.misura.getCalibration())
            #sn.PrintEndRun()
            #print(sensori.readLaser(sensori.l_avdx))
            #
            A, B, C = sensori.Giroscopio(0)
            print(A,B,C,sensori.Inclinazione(B))
            #print(sn.Temperatura(sn.tDx),sn.Temperatura(sn.tSx))
            #sn.PrintEndRun()
            time.sleep(0.1)
            #print()
            
        except KeyboardInterrupt:
            break
        '''
        except:
            print('non va una sega')
        '''
