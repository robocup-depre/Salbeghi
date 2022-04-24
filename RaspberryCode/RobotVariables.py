import numpy as np
import time
import sensoriV3 as MainSensors
## Resetta
## Avvia
## Sc -> Scrivi (formattazione varibili significative per print)
#LibDritto
#LibGira
#LibFinecorsa
    #AvviaAvDx
    #AvviaAvSx
#LibAllinea
#LibCentraY
#LibRampa
################################################################
class LibLogica:
    numKit = None #numro di kit disponibili
    numLack = 0
    okCamDx = 1
    okCamSx = 1
    checkpoint = []
    def Resetta(self):
        self.avvio = 1
        self.mod = 0
        self.velDx = 0
        self.velSx = 0
        self.velDxTemp = 0
        self.velSxTemp = 0
        self.velNorm = 1
        self.velSet = 1
        self.distanza = 0
        self.distanzaold = 0
        self.gradiZ = 0
        self.gradiX = 0
        self.gradiY = 0
        self.erroreZ = 1
        self.gradiVolutiZ = 0
        self.lackcheck = 0
        
        sn = sensori.sensori()
        self.nLaser = sn.laserNum
        self.laser = [0]*self.nLaser
        self.avvioLaser = [0]*self.nLaser
        self.avvioGiroscopio = 1
        self.block = 0
        '''
        self.laserAvDx = -1
        self.laserDiDx = -1
        self.laserAvSx = -1
        self.laserDiSx = -1
        self.laserFr = -1
        '''
        self.libero = "free"
        self.azione = [self.libero,self.libero,self.libero,self.libero]
        self.oldAzione = 0
        self.lack = 0
        self.contaErrori = 0
        self.luce = 0

        self.inclinazione = 0
        
        self.stato = 'zero'
        self.oldStato = 'zero'

        self.eccSx = 0
        self.eccDx = 0
        
        self.argento = 2
        self.bianco = 1
        self.nero = 3
        self.nuoveMisure = 0

        self.avvioCamDx = [0,0]
        self.avvioCamSx = [0,0]
        self.Resettino()
        
        self.nCicliT=0
        self.timeTemperaturaDx = 0 
        self.posTemperaturaDx = 0
        self.timeTemperaturaSx = 0 
        self.posTemperaturaSx = 0
        self.xTemperaturaSx = 0
        self.yTemperaturaSx = 0
        self.xTemperaturaDx = 0
        self.yTemperaturaDx = 0
        
        self.oldMuroSx = 0
        self.posVisualeSx = 0
        self.xVisualeSx = 0
        self.yVisualeSx = 0
        self.tipoVisualeSx = 0
        
        self.oldMuroDx = 0
        self.posVisualeDx = 0
        self.xVisualeDx = 0
        self.yVisualeDx = 0
        self.tipoVisualeDx = 0
        
        self.direzioneMuri = 0
        
        self.blocco = 1
        self.trovataVittimaDx = 0
        self.trovataVittimaSx = 0
        
        self.indietroNero =0
        
    def Resettino(self):
        self.pmdx = 0 #presenza muro a dx
        self.pmsx = 0
        self.pmfr = 0
        self.casella = 0
        self.noAvanti = 0
        self.noDestra = 0
        self.flAllinea = 0
        self.flFinecorsa = 0
        self.ripiano = 0
        self.trovatoDx = "N"
        self.trovatoSx = "N"
        self.vittimeDx = []
        self.vittimeSx = []
        self.inizioRampa = None
        self.distRampa = None
        self.distVittime = None
        self.ultimoC = time.time()
        self.fTrovatoDx =0
        self.fTrovatoSx = 0
        self.tempTrovatoDx = 0
        self.tempTrovatoSx = 0
        
        self.flTrovataVittima = 0
    def Libero(self,livello):
        if(self.azione[livello]==self.libero):
            return True
        else:
            return False
    def __init__(self):
        self.Resetta()
        
    def RSS(self,livello):
        for ind in range(livello,len(self.azione)):
            self.azione[ind]=self.libero
        
        
    def Limita180(self,val):
        while(val>180):
            val-=360
        while (val<-180):
            val+=360
        return val
    def ColoreCasella(self):
        if(self.luce==-1):
            self.casella=-1
        elif(self.luce<370): #330
            self.casella= self.argento
        elif(self.luce<600):#230
            self.casella=self.bianco
        else:
            self.casella= self.nero

    def ChiudiFunzione(self,obj):
        for ind in range(4-obj.livello):
            self.azione[obj.livello + ind] = self.libero
        obj.avvio = 0
    def Sc (self):
        a = self.velDx
        b = self.velSx
        c = self.azione
        return a, b, c

################################################################  
    
    
    
class LibPID:
    def Resetta(self, Kp, Ki, Kd):
        self.avvio = 1
        self.mod = 0
        self.oldMod = 0
        self.stato = 0
        self.errore = 0
        self.Kp = Kp #0.06#0.01
        self.Ki = Ki #0.4
        self.Kd = Kd
        print("init pid kp",self.Kp,"ki",self.Ki,"kd",self.Kd)
        self.effettoP = 0
        self.integrale = 0
        self.effettoD = 0
        self.oldErrore = 0
        self.correzione = 0
        self.limiteP = 3
        self.limiteI = 0.2
        self.limiteD = 1.5
        self.limiteC = 3
        self.oldTime = 0
        self.deltaTime = 0
        self.time = 0
        self.alfa = 0 #VARIABILE DI UTILIZZIO MISTO
        self.beta = 0
        self.misura = 0
        self.startBSx =0
        self.startBDx =0
    def __init__(self, Kp, Ki, Kd):
        self.Resetta(Kp, Ki, Kd)
        
    def Avvia (self):
        self.Resetta()
        self.avvio = 1
        
    def Resettino(self):
        self.oldErrore = 0
        self.oldTime = 0
        self.integrale = 0
        
    def CalcolaPID(self):
        self.time  = time.time() 
        self.deltaTime = self.time-self.oldTime
        self.oldTime = self.time
        
        #print("Dt:",self.deltaTime)
        self.effettoP = self.errore * self.Kp
        self.integrale += self.errore * self.Ki  * self.deltaTime
        if(self.deltaTime != 0):
            self.effettoD = ((self.errore-self.oldErrore)*self.Kd ) / self.deltaTime
        else:
            self.effettoD = 0
        
        if(self.effettoP>self.limiteP):
            self.effettoP = self.limiteP
        if(self.effettoP<-self.limiteP):
            self.effettoP = -self.limiteP
            
        if(self.effettoD>self.limiteD):
            self.effettoD = self.limiteD
        if(self.effettoD<-self.limiteD):
            self.effettoD = -self.limiteD

        if(self.integrale>self.limiteI):
            self.integrale = self.limiteI
        if(self.integrale<-self.limiteI):
            self.integrale = -self.limiteI

        self.correzione = self.effettoP + self.effettoD + self.integrale
         
        if(self.correzione>self.limiteC):
            self.correzione = self.limiteC
        if(self.correzione<-self.limiteC):
            self.correzione = -self.limiteC

        self.oldErrore = self.errore
        

        
    def limtaM(self,val):
        if(val>1.9):
            val=1.9
        if(val<-1.9):
            val=-1.9
        return val
    def Sc (self):
        a = 0
        b = 1
        c = 2
        return a, b, c
    
################################################################
class LibMappa:
    def SettaQueue(self, queueMappa):
        self.q_mappa = queueMappa
        
    def Resetta(self):
        
        self.numX = 30
        self.numY = 30
        self.initX = 15
        self.initY = 15
        self.oldX = self.initX
        self.oldY = self.initY
        self.checkPointX = self.initX
        self.checkPointY = self.initY
        self.robo = [self.initX,self.initY,0,0]
        self.inesplorata = 0
        self.argento = 2
        self.bianco = 1
        self.nero = 3

    def __init__(self):
        self.Resetta()

    def sempG(self,val):
        while(val<0):
            val+=360
        val = val%360

        if(val>315 or val<=45):
            res = 0
        elif(val>45 and val<=135):
            res = 3
        elif(val>135 and val<=225):
            res = 2
        else:
            res = 1
        return res

    def limit04(self,inv):
        if(inv>3):
            out=inv-4
        elif(inv<0):
            out=inv+4
        else:
            out = inv
        return out

    def limita180(self,val):
        while(val>180):
            val-=360
        while(val<-180):
            val+=360
        return val
    def ControlloMappa(self,mappa):
        errore = 0
        for indx in range(mappa.nx): #per ogni casella della mappa
            for indy in range(mappa.ny):
                casella = mappa.map[indx][indy]
                if(casella.esplorata==0):
                    if(casella.rampa!=None):
                        errore = 1
                        print(indx,indy,"rampa definita ma la casella non è stata esplorata")
                    if(casella.colore!=None):
                        errore = 2
                        print(indx,indy,"colore definito ma la casella non è stata esplorata")
                else:
                    if(casella.colore==None):
                        errore = 6
                        print(indx,indy,"colore non definito ma la casella è stata esplorata")
                
                for dA in range(4):
                    if(casella.esplorata==0):
                        if(casella.muri[dA]!=None):
                            print(indx,indy,dA,"muro definito ma la casella non è stata esplorata")
                            errore = 3
                        if(casella.vittime[dA]!=None):
                            print(indx,indy,dA,"vittima definita ma la casella non è stata esplorata")
                            errore = 4
                    else:
                        if(casella.muri[dA]==None):
                            print(indx,indy,dA,"muro non definito ma la casella è stata esplorata")
                            errore = 7
                    try:
                        indxmod, indymod = mappa.modXY(indx,indy,dA)
                        casellaMod = mappa.map[indxmod][indymod]
                        if(casella.esplorata!=0 and casellaMod.esplorata !=0):
                            if(casella.muri[dA]!=casellaMod.muri[2 -dA +2*(dA%2)]):
                                print(indx,indy,indxmod, indymod,dA,"cas",casella.esplorata,casellaMod.esplorata,"mur",casella.muri[dA],casellaMod.muri[2 -dA +2*(dA%2)],"muro non coincidente con la casella succesiva")
                                errore = 5
                    except:
                        None
        return errore
    
    def EseguiDijkstra(self,ix, iy ,mappa):
        iniz = time.time()
        frontiera = {}
        #fatti = []
        #mappa.Pulisci()
        mappa.map[ix][iy].distanza = 0
        ax = ix
        ay = iy
        
        while True:
            frontiera.pop((ax,ay),-1)
            for dAssoluta in range(4):
                nvx = mappa.modX(ax,dAssoluta)
                nvy = mappa.modY(ay,dAssoluta)
                colleg = not(mappa.map[ax][ay].muri[dAssoluta]==1) and (mappa.map[nvx][nvy].colore != 2)
                if(colleg):
                    nDistanza = mappa.map[ax][ay].distanza + 1
                    if (nDistanza < mappa.map[nvx][nvy].distanza):
                        if(mappa.map[nvx][nvy].esplorata >0):
                            frontiera[(nvx,nvy)]=nDistanza
                        mappa.map[nvx][nvy].distanza = nDistanza
                        mappa.map[nvx][nvy].precedente = 2 -dAssoluta +2*(dAssoluta%2)
            if(not(frontiera)):
                #print("HO FINITO DIJKSTRA ")
                break
            else:
                ax, ay = min(frontiera,key = frontiera.get)
                    
        return (time.time()-iniz)*1000


    def TrovaDirezione(self,mappa, atx, aty, inx = None, iny = None ):
        inizotd = time.time()
        minori = {}
        dOut =  None
        for indx in range(mappa.nx):
            for indy in range(mappa.ny):
                
                if(mappa.map[indx][indy].esplorata == 0 and 
                    mappa.map[indx][indy].distanza < 1000 and 
                    (inx == None or iny == None)):
                    minori[(indx,indy)] = mappa.map[indx][indy].distanza
                elif(inx == indx and iny == indy ):
                    minori[(indx,indy)] = mappa.map[indx][indy].distanza


        
        if(not(minori)):
            #print("NESSUNA INESPLORATA")
            dOut =  None
        else:
            minimi = {}
            tempx1, tempy1 = min(minori,key = minori.get)
            minimi[(tempx1, tempy1)] = ((tempx1-atx)**2 + (tempy1-aty)**2)**(1/2)
            #mappa.map[tempx1][tempy1].coloreVis = 5
            minori.pop((tempx1,tempy1),-1)
            while minori:
                tempx, tempy = min(minori,key = minori.get)
                
                minori.pop((tempx,tempy),-1)
                if(mappa.map[tempx][tempy].distanza > mappa.map[tempx1][tempy1].distanza):
                    break
                else:
                    tempx1, tempy1 = tempx, tempy
                    minimi[(tempx1, tempy1)] = ((tempx1-atx)**2 + (tempy1-aty)**2)**(1/2)

                    #mappa.map[tempx1][tempy1].coloreVis = 5
                    #time.sleep(1)

            nMinimi = len(minimi)
            puntomin = min(minimi,key = minimi.get)

            
            for punt in minimi:
                xt, yt = punt
                if(minimi[punt]==minimi[puntomin]):
                    mappa.map[xt][yt].coloreVis = 6
                else:
                    mappa.map[xt][yt].coloreVis = 5

            #if(nMinimi>1):
                #print("MINIMI:",minimi)
                #time.sleep(1.5)
            ax, ay = puntomin
            dPrecOld = 0
            while True:
                if(mappa.map[ax][ay].precedente == None):
                    #print("finito TrovaDirezione")
                    dOut = 2 -dPrecOld +2*(dPrecOld%2)
                    break
                else:
                    dPrecOld = mappa.map[ax][ay].precedente
                    ax = mappa.modX(ax,dPrecOld)
                    ay = mappa.modY(ay,dPrecOld)
        return dOut , (time.time()-inizotd)*1000
        
################################################################
class LibProsegui:
    def Resetta(self):
        self.nome = "prosegui"
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        
    def __init__(self):
        self.Resetta()

    def Avvia (self, livello, velocita, mod):
        self.Resetta()
        self.avvio = 1
        self.velocita = velocita
        self.livello = livello 
        self.mod = mod

################################################################
class LibDritto:
    def Resetta(self):
        self.nome = "dritto"
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        self.distanzaMancante = 0
        self.mRetta = 0 #coefficente angolare retta
        self.qRetta = 0 #intercetta retta (velocità minima)
        self.distanzaRetta = 5 #distanza di azione della retta
        self.velocita = 0 #velocità massima voluta
        self.errore = -1
        
        self.modalita=0
        self.Resettino()

    def Resettino(self):
        self.distanzaMancante = 0
        self.distanzaPercorsa = 0

    def __init__(self):
        self.Resetta()

    def Avvia (self, livello, velocita, distanzaVoluta, modalita = 0):
        self.Resetta()
        self.avvio = 1
        self.velocita = velocita
        self.livello = livello 
        self.distanzaVoluta = distanzaVoluta
        self.modalita = modalita

    def Sc (self):
        a = self.distanzaMancante
        b = self.distanzaVoluta
        c = self.velocita
        return a, b, c

################################################################
class LibGira:
    def Resetta(self):
        self.nome = "gira"
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        self.gradiVoluti = 0
        self.velocita = 1#1.2 #velocità massima
        self.gradiMaxRetta = 50 #gradi di massimo azione della retta
        self.gradiMinRetta =-20
        self.qRetta = 0.12 #intercetta retta (velocità minima)
        self.mRetta = (self.velocita-self.qRetta)/self.gradiMaxRetta#0.0224 #coefficente angolare retta        
        self.errore = -1
        self.Resettino()
        
    def Resettino(self):
        self.gradiMancanti = 0
        self.gradiPercorsi = 0
        self.gradiAdesso = 0 #gradi modificati
        self.gradiIniziali = 0
        self.gradiUltimo = 0 #misura precedente
        self.gradiModifica = 0 #gradi in fase di modifica
        self.versoGira = 0 # (1-> dx, -1 --> sx)
        self.kGiri = 0 #numero giri completi (360 * k)
        self.velocitaMod = 0
        
    def __init__(self):
        self.Resetta()

    def Avvia (self, livello, velocita, gradiVoluti):
        self.Resetta()
        self.avvio = 1
        self.velocita = velocita
        self.livello = livello 
        self.gradiVoluti = gradiVoluti

    def Sc (self):
        a = self.gradiMancanti
        b = self.gradiVoluti
        c = self.gradiIniziali
        return a, b, c

################################################################
class LibFinecorsa:
    def Resetta(self):
        self.nome = "finecorsa"
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        self.posizione = 0 # 0 AvDx, 1 AvSx, 2 Av , 3 DiDx, 4 DiSx

    def __init__(self):
        self.Resetta()
        
    def Controlla(self,livello):
        self.Resetta()
        self.avvio = 1
        self.posizione = posizione
        self.livello = livello
        self.stato = -1
        
    def Avvia (self, livello, posizione):
        self.Resetta()
        self.avvio = 1
        self.posizione = posizione
        self.livello = livello
        self.stato = 0
        
    def AvviaAvDx (self, livello):
        self.Resetta()
        self.avvio = 1
        self.posizione = 0
        self.livello = livello
        self.stato = 0
        
    def AvviaAvCe (self, livello):
        self.Resetta()
        self.avvio = 1
        self.posizione = 2
        self.livello = livello
        self.stato = 0
        
    def AvviaAvSx (self, livello):
        self.Resetta()
        self.avvio = 1
        self.posizione = 1
        self.livello = livello
        self.stato = 0
        
    def Sc (self):
        a = self.stato
        b = self.posizione
        return a, b

################################################################
class LibAllinea:
    def Resetta(self):
        self.nome = "allinea"
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        self.tempoInizio = 0
        self.tempoTotale = 5
        self.tempoMancante = 0
        self.uscita = 0
        self.finecorsaDx = 0
        self.finecorsaSx = 0
        self.fuoriCasella = 0
    def __init__(self):
        self.Resetta()

    def Avvia (self, livello):
        self.Resetta()
        self.avvio = 1
        self.livello = livello 

    def Sc (self):
        a = self.tempoMancante
        b = self.finecorsaDx
        c = self.finecorsaSx
        return a, b, c

################################################################
class LibCentraY:
    def Resetta(self):
        self.nome = "centraY"
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        self.verso = 0
        self.distanzaAggiunta = 5
        
    def __init__(self):
        self.Resetta()

    def Avvia (self, livello):
        self.Resetta()
        self.stato = 0
        self.avvio = 1
        self.livello = livello

    def Sc (self):
        a = self.verso
        return a

################################################################
class LibVittime:
    def Resetta(self):
        self.nome = "vittime"
        self.tipi = ['R','G','Y','H','S','U']
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        self.tipo = None
        self.posizione = 0
        self.vKit = 0 #kit che voglio rilasciare
        
        self.rilKit = 0 #rit rilasciati
        self.thl = None
        self.rotaz = 0
        self.osc = 0 #oscillazione
        self.amp = 0 #amplificazione
        self.rif = 0 #rifermiento servo
        self.rifCentr = 0
        self.nOSc = 0 #numero oscillazioni effetuate
        self.tPos = 0
    def __init__(self):
        self.Resetta()

    def Avvia (self, livello, tipo):
        self.Resetta()
        self.tipo = tipo
        self.stato = 0
        self.avvio = 1
        self.livello = livello


    def Sc (self):
        a = self.stato
        return a

################################################################
class LibControlloVittime:
    def Resetta(self):
        self.nome = "controlloVittime"
        self.avvio = 0
        self.stato = 0
        self.livello = 0
        self.modalita = 0 
    def __init__(self):
        self.Resetta()

    def Avvia (self, livello, mod = 0):
        self.Resetta()
        self.stato = 0
        self.avvio = 1
        self.livello = livello
        self.modalita = mod
        
################################################################


################################################################
################################################################


    
if __name__ == '__main__':
    libero = "libero"
    azione=[libero,libero,libero,libero]
    
    dritto = LibDritto()
    dritto.Avvia(0,1,10)
    
    gira = LibGira()
    gira.gradiVoluti = 90
    
    finecorsa = LibFinecorsa()
    finecorsa.livello=1
    
    print(dritto.Sc(), gira.Sc())
    
    azione[dritto.livello] = dritto.nome #  self.nome = "gira"    
    azione[finecorsa.livello] = finecorsa.nome
    print(azione)
    logica = LibLogica()
    print(logica.nLaser)
    
