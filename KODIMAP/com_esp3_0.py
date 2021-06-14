import time
import serial
import struct
import RPi.GPIO as GPIO
import math

class LibESP:
    def __init__(self):
        self.ser = serial.Serial( #inizializzazione seriale
            port='/dev/ttyAMA0',
            #port='/dev/serial0',
            baudrate=1000000,
            parity= serial.PARITY_EVEN, #con parità serial.PARITY_NONE
            stopbits=serial.STOPBITS_ONE, #uno bit di parità
            bytesize=serial.EIGHTBITS, 
            timeout=0
        )
        self.pinAttivo = 4 #pin alto se l'esp32 è attivo
        self.pinAlimentato = 23
        self.pinReset = 21
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinAttivo, GPIO.IN)
        GPIO.setup(self.pinAlimentato, GPIO.IN)
        GPIO.setup(self.pinReset, GPIO.OUT) 
        GPIO.output(self.pinReset,GPIO.LOW)
        self.errore = "errore" #definizione errori (stringa non dipendente)
        self.noEsp = "noesp"

        self.coppia = 1 #coppia di applicazione nell'esp (1--> motori 0=1,2=3; 2 --> motori 0=2,1=3; 3-->0=3,1=2)

    def clean(self): # per chiudere libreria
        GPIO.cleanup()
        
    def Pesp(self):
        pinAlim =GPIO.input(self.pinAlimentato)
        pinAtt = GPIO.input(self.pinAttivo)
        
        if(pinAtt!=0): #se l'esp32 si dice attivo
            return 1
        elif(pinAlim!=0):
            ts = time.time()
            nr = 0
            while(pinAlim!=0 and pinAtt==0 and nr<2):
                time.sleep(0.01)
                if(time.time()-ts > 0.5):
                    GPIO.output(self.pinReset,GPIO.HIGH)
                    time.sleep(0.01)
                    GPIO.output(self.pinReset,GPIO.LOW)
                    ts = time.time()
                    nr +=1
                pinAlim =GPIO.input(self.pinAlimentato)
                pinAtt = GPIO.input(self.pinAttivo)
            
            if(pinAtt!=0):
                return 1
            else:
                if(pinAlim==0):
                    return self.noEsp
                else:
                    return self.errore
        else:
            return self.noEsp
    

    def Leggi(self,byte): #lettura da esp32
        controllo = self.Pesp()
        out = 0
        if(controllo!=1): #se l'esp non è attivo
            out =  controllo
        else:
            start=time.time() #tempo di partenza
            while(self.ser.in_waiting <= 0): #finchè non è diponibile qualcosa in seriale
                if((time.time()-start)>0.001): #0.001 tempo massimo di attesa
                    out=self.errore
                    break
            if(out == 0): #se non è in errore
                letto = self.ser.read(size = byte)#leggi i byte
                if(len(letto)!=byte): #se non è giusto il numero di byte letti 
                    out =  self.errore
                else: #aggiustamento risultato
                    out =0
                    for i in range(byte):
                        out |= letto[i]<<(8*i)
        return out

    def Invia(self, dato):
        #print("INVIOESP")
        self.ser.write(struct.pack('>B',dato)) #invio dato tramite seriale
        #temp = str(chr(dato))
        #self.ser.write(temp.encode())
        #print("FINEINVIO")
        

    def ModMot(self,x,bit):  #modfica valore motore (-2,2) per farlo stare tra 0 a 2^bit
        if(x>1.9):
            x=1.9
        if(x<=-1.9):
            x=-1.9
        xm=x+2
        xm*=(2**bit)/4;
        xm=round(xm)
        return int(xm)

    def ModDist(self,DATO): #modifca distanza ricevuta (da 0 a 32768) a distanza in cm
        if(DATO!=self.errore):
            DATO-=32768
            DATO/=100
        return DATO

    def ModFoto(self,DATO): #modifica valore fotoresistenza da 0-4096 a 0-1023
        if(DATO!=self.errore and DATO>=0):
            DATO*= 1023.0/4096
        return DATO

    def Pulisci(self): #pulire seriale in ingresso
        self.ser.flush()
        self.ser.flushInput()
        
    def CheckByte(self, dato): #calcolo del byte di controllo finale, conta il numero di "1" presente nei byte precedenti
        somm = 0
        if(isinstance(dato,int)): #per farlo funzionare sia con liste che con interi semplici
            dato = [dato] # se è un semplice int viene messo all'interno di una lista
            
        for ind in dato: #vengono passati tutti gli elementi del dato
            if ( ind!=0): #se non sono vuoti
                n = math.ceil( math.log(ind,2))+1 #viene calcolato il minimo numero di bit da controllare (0b0001 0110 --> n=5)
                for i in range(n):
                    somm+=(ind>>i)%2 #aggiunge 1 se il valore è dipasri quindi il resto è 1
        return somm #ritorna l'intera somma

    def SetDiametro(self, diametro,n=0): #invio del diametro, non specificare n nella chiamata della funzione
        if(diametro>=55 and diametro<75): # se è compatibile 
            self.Pulisci() #pulisce la seriale
            nByte = 3 #numero di byte d inviare
            byteOut = [0]*nByte #crea la lista di byte che dovrà inviare
            byteOut[0] = setByte = (1<<7) | (1<<6) | (0<<5) | 0b101 # 1<<7 primo bit da sinistra è sempre 1 per il primo byte di controllo
                                                                    # 1<<6 e 1<<5 modalità che serve all'esp per capire cosa li stiamo inviando
                                                                    # 101 valori senza significao specifico (aumenta numero di 1)
            byteOut[1] = int((diametro-55) * (128/20))   #converte il diamentro da cm (considerando intervalo 75-55) a 0-128
            checkByte = byteOut[2] = self.CheckByte(byteOut)  #calcolo controllo
            #print(byteOut,bin(byteOut[1]))
            for ind in range(nByte): #invio
                self.Invia(byteOut[ind])
            lettura = self.Leggi(1) #lettura conferma
            #print("checkByte",checkByte,lettura)
            errore = 0
            if (lettura==checkByte): #se la conferma corrisponde al chek byte inviato è andato tutto a buon fine
                risultato = 1
            else: #altrimenti
                if(lettura == self.noEsp): #errore perchè l'esp non è attivato
                    errore = 2
                    risultato = lettura
                else: #altro errore 
                    errore = 1 
                    risultato = 0
            if (errore == 1): #se c'è un errore non associabile
                n=n+1 #aumento numero errori
                print("riprovo esp set diam n:",n)
                
                if(n<10): #se è stato provato meno di 10 volte
                    if(n==5):
                        GPIO.output(self.pinReset,GPIO.HIGH)
                        time.sleep(0.01)
                        GPIO.output(self.pinReset,GPIO.LOW)
                        time.sleep(0.5)
                    time.sleep(0.05) 
                    risultato = self.SetDiametro(diametro,n) #richiama funzione
                else:
                    risultato = 0 #altrimenti errore
        else:
            risultato = 0
        return risultato

    #funzioni abbreviate
    def RicDistanza(self): #richiede solo distanza
        return self.set_mot(-3,-3,-3,-3,distanza=1)

    def RicFoto(self):  #rchiede solo fotoresistenza
        return self.set_mot(-3,-3,-3,-3,foto=1)

    def ResetDistanza(self): # resetta la Distanza
        return self.set_mot(-3,-3,-3,-3)
 
    def SetMot(self, AvDx = -5, AvSx = -5, DiDx = -5, DiSx = -5, foto = 0, distanza = 0): #funzione con nome diverso (per versioni vecchie)
        return self.set_mot(AvDx, AvSx, DiDx, DiSx, foto, distanza)
    
    def set_mot(self, AvDx = -5, AvSx = -5, DiDx = -5, DiSx = -5, foto = 0, distanza = 0, n=0): #funzione principale di controllo comunicazione
        #non indicare la n nella chiamata della funzione
        #foto = 1 incdica che volgiamo che ritoni il valore della fotoresistenza
        #distanza = 1 incdica che volgiamo che ritoni il valore della distanza (se foto e anche distanza, ritornerà entrabi)
        #se lasciati tutti i motori a -5 verrà impostata vecloità 0 a tutti
        #se messi tutti i motori a -3 non verrano inviati i motori, ma solo recuerati i valori di distanza e fotoresistenza, se questi sono entrabi a 0 verrà resettata la distanza
        #è possibile indicare solo un valore dei motori, e a tutti gli altri verrà impostato lo stesso (SetMot(0.5,...))
        #è possibile indicare i primi due valori come le coppie rispettivamente di Dx e Sx (SetMot(0.5,-0.8,...))
        #è possibile indicare tutti i valori dei motori singoli
        if(AvDx == -3 and AvSx == -3 and DiDx == -3 and DiSx == -3): # se sono tutti i motori a -3 si richiede solo, senza inviare valori motori
            self.Pulisci()
            nByte = 2
            byteOut = [0]*nByte
            byteOut[0]=setByte = (1<<7) | (1<<6) | (1<<5) | (foto<<4) | (distanza<<3) #viene specificato cosa mandare
                                #1<<7 primo bit da sinistra è sempre 1 per il primo byte di controllo
                                #1<<6 1<<5 modalità che serve all'esp per capire la comunicazione
                                # fotoresistenza e distanza indicano cosa vogliamo che ci ritorni (se ENTRABE a O verrà RESETTATA LA DISTANZA)
            checkByte = byteOut[1] = self.CheckByte(byteOut) #calcolo byte di controllo
            for ind in range(nByte):
                self.Invia(byteOut[ind])
        else:
            if(AvDx == -5 and AvSx == -5 and DiDx == -5 and DiSx == -5): #se non è stato specificato il valore dei motori
                AvDx = 0 #tutti a 0
                
            if(DiDx == -5 and AvDx!=-5): #se è specificato un valore si della coppia di dx e uno no
                DiDx = AvDx #coppia uguale
            if(AvDx == -5 and DiDx!=-5): 
                AvDx = DiDx #coppia uguale
                
            if(DiSx == -5 and AvSx!=-5): #se è specificato un valore si della coppia di sx e uno no
                DiSx = AvSx #coppia uguale
            if(AvSx == -5 and DiSx!=-5):
                AvSx = DiSx #coppia uguale
                
            if(AvSx == -5 and AvDx != -5 ): #se è stato specificato solo un valore alla coppia di Dx verrà riportato alla coppia di sx
                AvSx = AvDx
            if(DiSx == -5 and AvSx!=-5): # se manca un valore della coppia sx
                DiSx = AvSx
                
            if(foto != 1): #correzione valore di ingresso se si vuole fotoresistenza e distanza, se non è 1 è 0
                foto = 0
            if(distanza!=1):
                distanza = 0
            
            #print("INIESP")
            self.Pulisci() #pulisco ingresso 

            setByte = (1<<7) | (foto<<4) | (distanza<<3) #byte di controllo di base
                                #1<<7 primo bit da sinistra è sempre 1 per il primo byte di controllo
                                # fotoresistenza e distanza indicano cosa vogliamo che ci ritorni 
            if ( AvDx == DiDx and AvSx == DiSx):
                if (AvDx==AvSx):  #Modalità 1 tutti i motori uguali
                    mot = self.ModMot(AvDx,14) #preparo valore motore
                    #setByte|=(0<<6)|(0<<4)|(0<<3)|(0<<2)
                    nByte = 4 #numero byte di invio
                    byteOut = [0]*nByte #lista byte di invio
                    byteOut[0] = setByte   #setByte|=(0<<5)|(0<<4)|(0<<3)|(0<<2) modalità tutti i motori 00<<5 e coppia 0
                    byteOut[1] = mot>>7 #preparazione byte di invio (conformazione byte decisa a priori, in comune con esp)
                    byteOut[2] = mot & 0b1111111
                    checkByte = byteOut[3] = self.CheckByte(byteOut) #calcolo controllo
                    
                    for ind in range(nByte):
                        self.Invia(byteOut[ind])
                        #print(ind, bin(byteOut[ind]))
                    
                else: #Coppia standard
                    mot0= self.ModMot(AvDx,10) #preparo i valori delle coppie
                    mot1 = self.ModMot(AvSx,10)
                    setByte|=(0<<6)|(0<<5)|(self.coppia<<1) #modalità 00<<5 con coppia decisa nell'init
                    nByte = 5
                    byteOut = [0]*nByte
                    byteOut[0] = setByte
                    byteOut[1] = mot0>>3
                    byteOut[2] = ((mot0 & 0b111)<<4)|(mot1>>6)
                    byteOut[3] = (mot1 & 0b111111)<<1
                    checkByte = byteOut[4] = self.CheckByte(byteOut)
                    for ind in range(nByte):
                        self.Invia(byteOut[ind])
                        ##print(ind, bin(byteOut[ind]))
            else: #Diversi
                mot0= self.ModMot(AvDx,10) #caclolo tutti i motori singoli
                mot1 = self.ModMot(DiDx,10)
                mot2 = self.ModMot(AvSx,10)
                mot3 = self.ModMot(DiSx,10)
                setByte|=(0<<6)|(1<<5) # modalità 01<<5
                nByte = 8
                byteOut = [0]*nByte
                byteOut[0] = setByte
                byteOut[1] = mot0>>3
                byteOut[2] = ((mot0 & 0b111)<<4)|(mot1>>7)
                byteOut[3] = mot1 & 0b1111111
                byteOut[4] = mot2>>3
                byteOut[5] = ((mot2 & 0b111)<<4)|(mot3>>7)
                byteOut[6] = mot3 & 0b1111111
                checkByte = byteOut[7] = self.CheckByte(byteOut)
                for ind in range(nByte):
                    self.Invia(byteOut[ind])


        #LETTURA
        #print("LETTURA")
        errore = 0
        if(foto and distanza): #se volevlo il ritorno di entrabe
            lettura_foto = self.Leggi(2) #le leggo separatamente (ordine deciso a priori, lo sesso indicato nei paramteri (foto, distanza)
            lettura_dist = self.Leggi(2)
            lettura = self.Leggi(1) #leggo byte di controllo
            if(lettura_dist == self.noEsp or lettura_foto == self.noEsp or lettura==self.noEsp): #se ci sono errori di inattività esp
                errore = 2
                risultato = self.noEsp, self.noEsp
            elif(lettura_dist!=self.errore and lettura_foto!=self.errore and lettura == self.CheckByte([lettura_dist,lettura_foto]) ): #se non ci sono altri errori e il controllo letto è verificato 
                risultato = self.ModFoto(lettura_foto), self.ModDist(lettura_dist) #ritorno risultato giusto
            else:
                errore = 1 #altrimenti errore 1 --> farà ripovare 
                risultato = self.errore,self.errore
        elif (foto): #se volevo solo la fotoresistenza
            lettura_foto = self.Leggi(2)
            lettura = self.Leggi(1)
            if(lettura_foto == self.noEsp or lettura==self.noEsp):
                errore = 2
                risultato = self.noEsp
            elif(lettura_foto!=self.errore and lettura == self.CheckByte(lettura_foto) ):
                risultato = self.ModFoto(lettura_foto)
            else:
                errore = 1
                risultato = self.errore
        elif (distanza): #se volelo solo la distanza
            lettura_dist = self.Leggi(2)
            lettura = self.Leggi(1)
            if(lettura_dist == self.noEsp or lettura==self.noEsp):
                errore = 2
                risultato = self.noEsp
            elif(lettura_dist!=self.errore and lettura == self.CheckByte(lettura_dist) ):
                risultato = self.ModDist(lettura_dist)
            else:
                errore = 1
                risultato = self.errore
        else:# se non volevo niente di ritorno
            lettura = self.Leggi(1) 
            #print("checkByte",checkByte,lettura)
            if (lettura==checkByte): #il controllo vine fatto con il checkbyte inviato in precedenza
                risultato = 1
            else:
                if(lettura == self.noEsp):
                    errore = 2
                    risultato = lettura
                else:
                    errore = 1
                    risultato = 0

        if (errore == 1): #se l'errore è di tipo 1
            n=n+1 #aumenta conteggio errori (prove)
            if(n>1):
                print("riprovo esp n:",n)
            if(n<10): #se è minore di 10 riprova la funzione
                if(n==5):
                    GPIO.output(self.pinReset,GPIO.HIGH)
                    time.sleep(0.01)
                    GPIO.output(self.pinReset,GPIO.LOW)
                    time.sleep(0.5)
                time.sleep(0.05)
                risultato = self.set_mot(AvDx, AvSx, DiDx, DiSx , foto, distanza, n)
            else:
                risulatto = -1

            
        
        return risultato
        


if __name__ == '__main__':
    esp=LibESP()
    esp.SetDiametro(67.3)
    m=0
    n=0
    try:
        while(1):
            #r = esp.set_mot(1,-0.5,1.2,0.01,foto=1,distanza=1)
            #r = esp.reset_distanza()#esp.set_diametro(64.6) #esp.ric_foto()
            f,r=esp.SetMot(0,distanza=1,foto =1)
            '''
            if(r!=esp.noEsp):
                print(f)
                if(r>30):
                    esp.ResetDistanza()
                    esp.SetMot()
                    time.sleep(4)
                else:
                    time.sleep(0.1)
            else:
                time.sleep(0.5)
            '''
            print(r,f)
            time.sleep(0.1)
    except:
        esp.clean()
