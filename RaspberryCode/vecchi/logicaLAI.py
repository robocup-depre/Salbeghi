#POGETTA MATTEO - BERLATO MATTEO
#SALBEGHI 2019-2021

#Logica princiaple robot (logica mappatura + movimenti)

import time
import RPi.GPIO as GPIO
import board
import numpy as np
import threading
import queue as Queue
import math
import cv2
from termcolor import colored, cprint
from rilascioKit import MotoriRK
#from com_espNEW import LibESP
#from com_esp import LibESP
from com_esp3_0 import LibESP
import sensoriV3 as snv1
import robot #contiene tutte le sottolibrerie dei movimenti 
from neopix import Neopix
import copy


import classemappa

class logica():
    def __init__ (self):

        #classe per le variabili e funzioni secondarie logica
        self.logica  = robot.LibLogica()
        
    def correggiZ(self): #correzione valore girospocio asse Z
        g_temp = self.logica.gradiZ + self.sn.giroscopio.correzioni[0]
        if(g_temp<0):
            g_temp+=360
        if(g_temp>=360):
            g_temp-=360
        if(g_temp<=135 and g_temp>45):
            g_volut = 90
        elif(g_temp<=225 and g_temp>135):
            g_volut = 180
        elif(g_temp<=315 and g_temp>255):
            g_volut = 270
        else:
            g_volut = 0
        #g_temp, _, _ = self.sn.Giroscopio(1)  
        #self.sn.set_correzione(self.logica.Limita180(g_temp-self.logica.gradiVolutiZ),0)
        self.sn.set_correzione(self.logica.Limita180(g_temp-g_volut),0)
        self.logica.gradiVolutiZ = g_volut
            
    def PidThread(self): #ESECUZIONE DID
        lg = self.logica  #riporto in forma abbreviata le librerie che mi servono
        sn = self.sn
        pidG = self.pidG
        old = time.time()
        while True:
            if(lg.avvio==0):
                time.sleep(1)
            else:
                old = time.time()
                
                lg.gradiZ, lg.gradiY, lg.gradiX = sn.Giroscopio(0)
                if(lg.gradiZ<-999):
                    print("RESET BNO!!!!!!!!!!!!!!!!!!!!")
                    lg.velDxTemp = 0
                    lg.velSxTemp = 0
                    sn.InitBNO()
                    lg.gradiZ, lg.gradiY, lg.gradiX = sn.Giroscopio(0)
                
                #lg.gradiZ1, lg.gradiY1, lg.gradiX1 = sn.Giroscopio(1)

                pidG.errore = lg.Limita180(lg.gradiVolutiZ-lg.gradiZ) #calcolo errore PID giroscopio
                if(abs(pidG.errore)<10 or pidG.avvio==0): #erroreZ rileva se siamo esatti con una tolleranza di 5°
                    lg.erroreZ = 1
                else:
                    lg.erroreZ = 0
                #print(pidG.errore)
                if ( pidG.avvio and (lg.velDx == lg.velSx) ): #esegue il PID giroscopio sono se è stato attivato (avvio) e se le locità di coppia solo uguali (non gira)
                    pidG.mod = 1 #setto la modalità 
                    if(pidG.mod != pidG.oldMod): #se la modialità differisce da quella vecchia resetto le variabili del PID
                        print("reset pidG:",pidG.oldMod)
                        pidG.Resettino()
                    
                    pidG.CalcolaPID() #calcolo PID
                    pidG.velDxCalc= lg.velDx-pidG.correzione # lo applico alle coppie che verranno settate in un succesiivo momento nel while principale
                    pidG.velSxCalc = lg.velSx+pidG.correzione
                    if((abs(pidG.velDxCalc)<abs(0.3*lg.velDx)) and lg.velDx!=0):
                        if(pidG.flDx==0):
                            pidG.startBDx = time.time()
                            print("inizio tempo velocità pidG Dx",pidG.velDxCalc)
                        pidG.flDx = 1
                    else:
                        pidG.flDx = 0
                        
                    if((abs(pidG.velSxCalc)<abs(0.3*lg.velSx)) and lg.velSx!=0 ):
                        if(pidG.flSx==0):
                            pidG.startBSx = time.time()
                            print("inizio tempo velocità pidG Sx",pidG.velSxCalc)
                        pidG.flSx = 1
                    else:
                        pidG.flSx = 0
                        
                    if(pidG.flDx ==1 and (time.time()-pidG.startBDx)>0.8):
                        lg.velDxTemp= (time.time()-pidG.startBDx)*lg.velDx
                        print("imposto ",(time.time()-pidG.startBDx),"% velocità Dx")
                    else:
                        lg.velDxTemp = pidG.velDxCalc
                    
                    if(pidG.flSx ==1 and (time.time()-pidG.startBSx)>0.8):
                        lg.velSxTemp= (time.time()-pidG.startBSx)*lg.velSx
                        print("imposto",(time.time()-pidG.startBSx),"% velocità Sx")
                    else:
                        lg.velSxTemp = pidG.velSxCalc
                else: #se non posso essereguire il PID
                    pidG.mod = 0  #setto la modalità 
                    lg.velDxTemp = lg.velDx #imposto velocità normali
                    lg.velSxTemp = lg.velSx
                    
                pidG.oldMod = pidG.mod #ricordo l'ultima modalità

                dif = 0.05-(time.time()-old)
                if(dif>0):
                    time.sleep(dif)
                else:
                    None
                    #cprint("PidG supera i 0.05s",'blue')
                


    def thLed(self):
        if(self.led.errore):
            for i in range(3):
                #print("LED VITTIMA")
                self.sn.AccendiLedCamera()
                time.sleep(0.4)
                self.sn.SpegniLedCamera()
                time.sleep(0.4)
            self.sn.AccendiLedCamera()
        else:
            for i in range(4):
                #print("LED VITTIMA")
                self.led.Led(-1,(255,255,255))
                time.sleep(0.5)
                self.led.Led(-1,(0,0,255))
                time.sleep(0.5)
            self.led.Led(-1,(0,0,0))

        
    def clean(self):  #funzione da richimare per chiudere questa libreria (logica)
        print("LKI")
        #self.putmappa(maze,0)
        self.esp.set_mot(0)
        self.esp.clean()
        print("chiuso esp")
        self.sn.clean()
        print("chiuso sensori")
        self.rk.clean()
        print("chiuso rilascio kit")
        self.led.SpegniLed()
        print("LKO")
    def putmappa(self,val,pos):
        try:
            self.q_mappa.put_nowait([pos,val])
        except Queue.Full:
            None
    #MAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAINMAIN        
    def main(self,q_ric_dx,q_avv_dx,q_ric_sx,q_avv_sx,q_mappa,inRIC=1,inRIL=1,tMinVittime=36): #funzione principale in multiprocessing
        print("MAIN")
        
        self.led = led = Neopix() #creo il led
        
        led.Led(-1,(255,255,255)) #lo setto bianco durate l'init del robot
        
        mappa = robot.LibMappa() #creo la mappa
        #mappa.SettaQueue(q_mappa) #setto alla mappa la queue dove scriverà la mappa per essere visualizzata da "visualizzazione"
        mappa.map = classemappa.mappa()
        mappa.map.Vuoto(mappa.numX,mappa.numY)
        maze = mappa.map.map
        robo = mappa.robo
        mappa.modX = mappa.map.modX
        mappa.modY = mappa.map.modY
        q_mappa.put([0,maze])
        q_mappa.put([1,robo])
        self.q_mappa = q_mappa
        

        self.sn = sn = snv1.sensori() # creo i sensori come oggetto comune e nella sua forma abbreviata (sn)
    
        #creo le varie sottolibrerie che contengono le variabili di ogni movimento
        
        dritto = robot.LibDritto() 
        
        gira = robot.LibGira()
        
        finecorsa = robot.LibFinecorsa()
        
        prosegui = robot.LibProsegui()
        #rampa = robot.LibRampa()
        self.vittime = vittime = robot.LibVittime()
        controlloVittime = robot.LibControlloVittime()
        allinea = robot.LibAllinea()
        
        centra = robot.LibCentraY()
                
        self.pidG = pidG = robot.LibPID(0.06,0.4,0.005) #creo il pid con le relative kp ki kd
    
        lg = self.logica #creo forma abbreviata di logica
        
        self.esp = esp = LibESP() #creo la comunicazione con l'esp32s

        self.rk = rk = MotoriRK()
        
        #creo il thred che si occuperà del PID
        self.t_pid = threading.Thread(target=self.PidThread)
        self.t_pid.daemon = True            
        
        lg.lack= 1 #quando entrerà nel ciclo principale saprà che è in un ceeckpoint
        
        #inizializzo sensori
        sn.InitLaser()
        sn.InitBNO()
        sn.InitTPA81()
        sn.InitLedCamera()
        print("fine init sensori")
        
        self.t_pid.start()
        
        x=0
        y=1
        g=2
        mod=3
        
        lg.avvioCamDx[0]=1
        lg.avvioCamSx[0]=1
        q_avv_sx.put_nowait(lg.avvioCamSx[0])
        q_avv_dx.put_nowait(lg.avvioCamDx[0])
        #time.sleep(5)
        lg.ricVisuale=inRIC
        lg.rilsciaKit=inRIL
        
        
        if(lg.rilsciaKit):
            rk.MotAvvio(-100,20,1,6)
            print("ok ril 1")
            rk.MotRuota(100,200,10)
            print("ok ril 2")
        lg.avvio = 1 #imposto che il robot può muoversi

        lg.oldcasella = lg.bianco
        print("####### WHILE PRINCIPALE #######")
   
        #WHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILEWHILE
        while(esp.Pesp()==1):
            time.sleep(0.2)
        
        lg.avvioCamDx[0]=0
        lg.avvioCamSx[0]=0
        lg.avvioCamDx[1]=1
        lg.avvioCamSx[1]=1
        lg.eccDx = 0
        lg.eccSx = 0
        led.SpegniLed() # spengo led alla fine dell'init
        while(1):

          
            if(lg.ricVisuale==0):
                lg.avvioCamDx[0]=0
                lg.avvioCamSx[0]=0            
                
            #CONTROLLO TELECAMERA DESTRA
                       
            lg.avvioCamDx[0]=1
            lg.avvioCamSx[0]=1
            if(lg.okCamDx):
                '''
                if(lg.avvioCamDx[0]!=lg.avvioCamDx[1]): #se lo stato è cambito 
                    try:
                        q_avv_dx.put_nowait(lg.avvioCamDx[0]) #provo a inviarlo tramite la queue
                        print("IMPOSTO A",lg.avvioCamDx[0]," CAMERA DXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
                        lg.avvioCamDx[1] = lg.avvioCamDx[0]  #aggiorno lo stato precedente
                        lg.eccDx=0
                    except Queue.Full:
                        if(lg.eccDx>9):
                            print("ECCEZIONE N9 ",lg.avvioCamDx[0]," CAMERA DXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
                        #time.sleep(0.1)
                        lg.eccDx += 0.5
                '''
                if(lg.eccDx>10):
                    lg.okCamDx = 0
                    #lg.okCamSx = 0
                    cprint("CAMERA DI DX IMPOSTATA COME NON DISPONIBILE",'red')
                try:
                    lg.trovatoDx = str(q_ric_dx.get_nowait()) #provo a riceve dal riconoscimento
                    lg.eccDx = 0
                    lg.tempTrovatoDx = time.time()
                    #lg.avvioCamDx[0]=0
                    lg.fTrovatoDx = 0.5
                    '''
                    if(lg.trovatoDx!="N"): 
                        cprint("telecamera dx:"+ str(lg.trovatoDx),'cyan') #stampo cosa ha trovato (N niente) R G V H S U
                    '''
                    led.VisualizzaTrovato(1,lg.trovatoDx) #imposta il rispettivo led
                except Queue.Empty: #se non c'è niente di nuovo non fare niente
                    lg.fTrovatoDx = 0
                    if(lg.avvioCamDx[0]==1):
                        lg.eccDx += 0.005
                        #print("aggiungo 0.001 dx")
            else:
                lg.trovatoDx="N"
            
                
            #CONTROLLO TELECAMERA SINISTRA
            if(lg.okCamSx):
                '''
                if(lg.avvioCamSx[0]!=lg.avvioCamSx[1]):
                    try:
                        q_avv_sx.put_nowait(lg.avvioCamSx[0])
                        print("IMPOSTO A ",lg.avvioCamSx[0]," CAMERA SXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
                        lg.avvioCamSx[1] = copy.copy(lg.avvioCamSx[0])
                        lg.eccSx = 0
                    except Queue.Full:
                        if(lg.eccSx>9):
                            print("ECCEZIONE N9",lg.avvioCamSx[0]," CAMERA SXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
                        #time.sleep(0.1)
                        lg.eccSx += 1
                '''   
                if(lg.eccSx>10):
                    lg.okCamSx = 0
                    #lg.okCamDx = 0
                    cprint("CAMERA DI SX IMPOSTATA COME NON DISPONIBILE",'red')
                try:
                    
                    lg.trovatoSx = str(q_ric_sx.get_nowait())
                    lg.tempTrovatoSx = time.time()
                    #lg.avvioCamSx[0]=0
                    lg.fTrovatoSx = 1
                    lg.eccSx=0
                    '''
                    if(lg.trovatoSx!="N"):
                        cprint(" telecamera sx:"+ str(lg.trovatoSx),'cyan')
                    '''
                    led.VisualizzaTrovato(0,lg.trovatoSx)
                except Queue.Empty:
                    lg.fTrovatoSx = 0
                    if(lg.avvioCamSx[0]==1):
                        lg.eccSx += 0.005
                        #print("aggiungo 0.001 Sx", lg.eccSx)
            else:
                lg.trovatoSx="N"
                
            #LETTURASENSORILETTURASENSORILETTURASENSORILETTURASENSORILETTURASENSORILETTURASENSORILETTURASENSORILETTURASENSORILETTURASENSORILETTURASENSORI
            
            for ind in range(lg.nLaser):
                #print("ind", ind, lg.nLaser)
                if(lg.avvioLaser[ind]):
                    misura = sn.ReadLaser(ind)
                    lg.avvioLaser[ind] = 0
                    if(misura <0):
                        print("ERRORE LASER SU MAIN Ln:",ind)
                        lg.avvio = 0
                    else:
                        lg.laser[ind]=misura
            
            
            # RICEVO FOTORESISTENZA E DISTANZA DA ESP
            # INVIO VELOCITA' DELLE COPPIE A ESP
            
            lg.luce, lg.distanza = esp.SetMot(lg.velDxTemp,lg.velSxTemp,foto=1,distanza=1)

            
            
            lg.inclinazione = sn.Inclinazione(lg.gradiY)
            
            robo[g] = copy.copy(lg.gradiZ)
            
            self.putmappa(robo,1)
            
        
            if(lg.avvio==0):
                cprint("Robot fermato da AVVIO", 'red', attrs=['bold'])
                
                lg.velDxTemp = 0
                lg.velSxTemp = 0
                lg.ultimoC = time.time()
                time.sleep(1)
            elif(lg.luce == esp.errore or lg.luce==esp.noEsp): #esp errore (spento x lack o errore fatale comunicazione)
                if(lg.contaErrori!=-1): #se sta ancora contando gli errori scrivi l'errore
                    if(lg.contaErrori == 0 or lg.contaErrori==6):
                        if(lg.luce == esp.errore): 
                            cprint("erroreESP n:" + str( lg.contaErrori), 'red')
                        if(lg.luce==esp.noEsp):
                            cprint("spentoESP n:" + str( lg.contaErrori), 'red')
                            time.sleep(0.01)
                    lg.contaErrori+=1
                   
                if(lg.contaErrori>6): #se gli errori superano i 6 setta il lack
                    sn.SpegniLedCamera()
                    lg.contaErrori = -1
                    lg.lack = 1
                    lg.velDx = 0
                    lg.velSx = 0 
            else:
                #LOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICALOGICA
                if(lg.lack):
                    #RESETTO TUTTI I DATI RELATIVI ALLA LOGICA
                    sn.AccendiLedCamera()
                    lg.Resetta()
                    self.correggiZ()
                    if(lg.numLack==0):
                        
                        mappa.map.map[round(robo[x])][round(robo[y])].colore = 1
                        lg.checkpoint.insert(0,(round(robo[x]),round(robo[y])))
                        
                    trovatocp = 0
                    for ind in range(len(lg.checkpoint)):
                        ckpx = lg.checkpoint[ind][0]
                        ckpy = lg.checkpoint[ind][1]
                        print("guardo per checkpoint", ckpx, ckpy)
                        if(mappa.map.map[ckpx][ckpy].colore == 1 or
                            (ckpx == mappa.initX and ckpy==mappa.initY)):
                            trovatocp = 1
                            robo[x]=ckpx
                            robo[y]=ckpy
                            break
                    if(trovatocp==0):
                        print("ERRORE CHECKPOINT - non è stato trovato")
                        print(lg.checkpoint)
                    #robo[x]=mappa.checkPointX
                    #robo[y]=mappa.checkPointY
                    lg.lackcheck = 1
                    #lg.gradiZ, lg.gradiY, lg.gradiX = sn.Giroscopio(0)
                    lg.gradiVolutiZ = lg.gradiZ #sistemo giroscopio
                    print("riarmo", lg.gradiVolutiZ )
                    print("setDiam:",esp.SetDiametro(67.3)) #se "1" il settaggio è andato a buon fine
                    dritto.Resetta()
                    gira.Resetta()
                    centra.Resetta()
                    allinea.Resetta()
                    finecorsa.Resetta()
                    controlloVittime.Resetta()
                    vittime.Resetta()
                    prosegui.Resetta()
                    
                    self.led.Led(-1,(0,0,0))
                    
                    robo[g] = copy.copy(lg.gradiZ)
                    if(lg.numLack==0):
                        lg.stato = 'primo90'
                    else:
                        lg.stato = 'zero'
                        
                        
                    if(lg.numKit == -1):
                        lg.numKit = None
                    
                    svDx = 0
                    svSx = 0
                    while(svDx==0):
                        try:
                            q_ric_dx.get_nowait()
                        except Queue.Empty:
                            svDx=1
                    while(svSx==0):
                        try:
                            q_ric_sx.get_nowait()
                        except Queue.Empty:
                            svSx=1
                
                    lg.distanzaold=0
                    lg.distanza=0
                    esp.ResetDistanza() #resetto distanza esp
                    #lg.stato = 'prova'
                    lg.numLack += 1
                else:
                    lg.contaErrori = 0 #se rietro e ci sono stati meno di 6 errori dall'esp li riporto a 0
                    
                if(lg.stato != lg.oldStato or lg.azione!= lg.oldAzione): #se c'è stata una modifica nei flag princiali
                    cprint("X"+str(round(robo[x],2)) +" Y:" +str(round(robo[y],2)) +" st:"+ str(lg.stato) +str(lg.azione) + " t:" + str(round(time.time()-lg.ultimoC,3)) , 'yellow')
                    lg.ultimoC = time.time() #setto tempo dell'ultimo cambio

                lg.nCicliT +=1
                if(lg.nCicliT>20):
                    lg.nCicliT=0
                    if(sn.Temperatura(sn.tSx)>tMinVittime):
                        #lg.timeTemperaturaSx= time.time()
                        lg.xTemperaturaSx = robo[x]
                        lg.yTemperaturaSx = robo[y]
                        lg.posTemperaturaSx = robo[g]
                        cprint("segno temeratura sx",'blue')
                        lg.fCaloreSx = 1
                    else:
                        lg.fCaloreSx = 0
                    
                    if(sn.Temperatura(sn.tDx)>tMinVittime):
                        #lg.timeTemperaturaDx= time.time()
                        lg.posTemperaturaDx = robo[g]
                        lg.xTemperaturaDx = robo[x]
                        lg.yTemperaturaDx = robo[y]
                        cprint("segno temeratura dx",'blue')
                        #print("stdx")
                        lg.fCaloreDx = 1
                    else:
                        lg.fCaloreDx = 0
                if( (lg.okCamDx or lg.okCamSx) and vittime.avvio==0):
                    sn.AccendiLedCamera()
                else:
                    sn.SpegniLedCamera()
                
                    
                lg.ColoreCasella()
                
                if(lg.oldcasella != lg.casella):
                    print("cambio colore da",lg.oldcasella,"a",lg.casella,"luce",lg.luce)
                    lg.xColCas = robo[x]
                    lg.yColCas = robo[y]
                lg.oldcasella = copy.copy(lg.casella)
            
                #SPERIMENTALE
                if((time.time()-lg.ultimoC)>10): #se i flag non cambiano da più di 10 secondi
                    None
                    #cprint("SUPERATO TEMPO DI CAMBIO DI 10 SECONDI",'magenta')
                    
                lg.oldStato = lg.stato #aggiorno stati precedenti di controllo
                lg.oldAzione = lg.azione.copy()


                #LOGICAPRINCIPALELOGICAPRINCIPALELOGICAPRINCIPALELOGICAPRINCIPALELOGICAPRINCIPALELOGICAPRINCIPALELOGICAPRINCIPALELOGICAPRINCIPALELOGICAPRINCIPALE
                
                if( lg.velDx == lg.velSx and lg.erroreZ and allinea.fuoriCasella==0):
                    lg.difDist = (lg.distanza-lg.distanzaold)*(1.0/31.0)
                    lg.radg = (robo[g]*math.pi)/180
                    lg.cosY = math.cos((lg.gradiY*math.pi)/180)
                    
                    robo[x]+=lg.difDist*math.sin(lg.radg)*lg.cosY
                    robo[y]+=lg.difDist*math.cos(lg.radg)*lg.cosY
                    
                    #print(lg.difDist*math.sin(lg.radg)*lg.cosY,lg.difDist*math.cos(lg.radg)*lg.cosY,lg.distanza,lg.distanzaold,robo[x],robo[y])
                        
                    #print("dx",lg.difDist*math.sin(lg.radg)*lg.cosY,"dy",lg.difDist*math.cos(lg.radg)*lg.cosY,'radg',round(lg.radg,3),'cosY',round(lg.cosY,3),'difDist',round(lg.difDist,3))
                      
                if(abs(mappa.oldX-robo[x])>0.2 or abs(mappa.oldY-robo[y])>0.2):
                    print("CAMBIO CASELLA TROPPO VELOCE")
                    led.Led(-1,(255,0,0))
                    esp.SetMot(0,0,foto=0,distanza=0)
                    
                    
                    #time.sleep(5)
                mappa.oldX=robo[x]
                mappa.oldY=robo[y]
                if(vittime.avvio == 0 and controlloVittime.avvio and lg.ricVisuale): #or lg.stato == 7
                    lg.avvioCamDx[0]=1
                    lg.avvioCamSx[0]=1
                
                casella = mappa.map.map[round(robo[x])][round(robo[y])]

                
                direz = mappa.sempG(robo[g])
                aggX = (direz%2)*(direz-2)  * 0.16 * lg.velDx
                aggY = ((direz%2) -1) * (direz-1) * 0.16 * lg.velDx
                casellaVittime = mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)]
                            
                #print(lg.trovatoSx, round(robo[x]), round(robo[y]),lg.trovatoDx)
                if(lg.stato=="avantiLibero" and  gira.avvio==0 and finecorsa.avvio == 0):
                #if(True):
                    #print("controllo vittime")
                    #DESTRA
                    if(casellaVittime.muri[mappa.sempG(robo[g])]):
                        #print("controllo vittime a")
                        if(casellaVittime.colore == (lg.bianco -1)):
                            #print("controllo vittime b")
                            if(casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g]))] == None):
                                #print("controllo vittime cDX:",lg.avvioCamDx[0], lg.okCamDx , lg.fTrovatoDx , lg.trovatoDx, lg.oldMuroDx)
                                if(lg.okCamDx  and lg.trovatoDx!="N" and lg.fTrovatoDx):
                                    pidG.avvio = 0
                                    #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                    lg.trovataVittimaDx = 1
                                    lg.cosavittimeDx = 'D'+lg.trovatoDx
                                    #vittime.Avvia(controlloVittime.livello+1,'D'+lg.trovatoDx)
                                    cprint("TROVATO DX :" +str(lg.trovatoDx),'green')
                                    casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g]))] = lg.trovatoDx
                                elif (lg.fCaloreDx):
                                    pidG.avvio = 0
                                    #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                    lg.trovataVittimaDx = 1
                                    lg.cosavittimeDx = 'DC'
                                    #vittime.Avvia(controlloVittime.livello+1,'D'+'C')
                                    cprint("TROVATO DX CALORE",'green')
                                    casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g]))] = 'C'
                                elif(lg.oldMuroDx ==0):
                                    lg.oldMuroDx = 1
                                    print("controllo vittime d")
                                    if( ((((robo[x]-lg.xVisualeDx)**2 + (robo[y]-lg.yVisualeDx)**2)**(1/2))*30)<11
                                        and abs(lg.Limita180(abs(robo[g]-lg.posVisualeDx))) < 20):
                                            pidG.avvio = 0
                                            #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                            lg.trovataVittimaDx = 1
                                            lg.cosavittimeDx = 'D'+lg.tipoVisualeDx
                                            #vittime.Avvia(controlloVittime.livello+1,'D'+lg.tipoVisualeDx)
                                            cprint("TROVATO DX DA PITAGORA :" +str(lg.tipoVisualeDx),'green')
                                            casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g]))] = lg.tipoVisualeDx
                                    if( ((((robo[x]-lg.xTemperaturaDx)**2 + (robo[y]-lg.yTemperaturaDx)**2)**(1/2))*30)<11
                                        and abs(lg.Limita180(abs(robo[g]-lg.posTemperaturaDx))) < 20):
                                            pidG.avvio = 0
                                            #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                            lg.trovataVittimaDx = 1
                                            lg.cosavittimeDx = 'DC'
                                            #vittime.Avvia(controlloVittime.livello+1,'D'+'C')
                                            cprint("TROVATO DX CALORE DA PITAGORA",'green')
                                            casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g]))] = 'C'
                        
                    else:
                        lg.oldMuroDx = 0
                        if( lg.okCamDx and lg.fTrovatoDx and lg.trovatoDx!="N"):
                            lg.posVisualeDx = robo[g]
                            lg.xVisualeDx = robo[x]
                            lg.yVisualeDx = robo[y]
                            lg.tipoVisualeDx = lg.trovatoDx
                            
                    #SINISTRA
                    if(casellaVittime.muri[mappa.limit04(mappa.sempG(robo[g])+2)]):
                        if(casellaVittime.colore == (lg.bianco -1)):
                            if(casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g])+2)] == None):
                                #print("controllo vittime cSX:",lg.avvioCamDx[0], lg.okCamDx , lg.fTrovatoDx , lg.trovatoDx, lg.oldMuroDx)
                                if( lg.okCamSx and lg.trovatoSx!="N" and lg.fTrovatoSx):
                                    pidG.avvio = 0
                                    #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                    lg.trovataVittimaSx = 1
                                    lg.cosavittimeSx = 'S'+lg.trovatoSx
                                    #vittime.Avvia(controlloVittime.livello+1,'S'+lg.trovatoSx)
                                    cprint("TROVATO SX :" +str(lg.trovatoSx),'green')
                                    casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g])+2)] = lg.trovatoSx
                                elif (lg.fCaloreSx ):
                                    pidG.avvio = 0
                                    #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                    lg.trovataVittimaSx = 1
                                    lg.cosavittimeSx = 'SC'
                                    
                                    #vittime.Avvia(controlloVittime.livello+1,'S'+'C')
                                    cprint("TROVATO SX CALORE",'green')
                                    casellaVittime.vittime[mappa.limit04(mappa.sempG(robo[g])+2)] = 'C'
                                elif(lg.oldMuroSx ==0):
                                    lg.oldMuroSx = 1
                                    if( ((((robo[x]-lg.xVisualeSx)**2 + (robo[y]-lg.yVisualeSx)**2)**(1/2))*30)<11
                                        and abs(lg.Limita180(abs(robo[g]-lg.posVisualeSx))) < 20):
                                            pidG.avvio = 0
                                            #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                            lg.trovataVittimaSx = 1
                                            lg.cosavittimeSx = 'S'+lg.tipoVisualeSx
                                            #casellaVittime.Avvia(controlloVittime.livello+1,'S'+lg.tipoVisualeSx)
                                            cprint("TROVATO SX DA PITAGORA :" +str(lg.tipoVisualeSx),'green')
                                            casella.vittime[mappa.limit04(mappa.sempG(robo[g])+2)] = lg.tipoVisualeSx
                                    if( ((((robo[x]-lg.xTemperaturaSx)**2 + (robo[y]-lg.yTemperaturaSx)**2)**(1/2))*30)<11
                                        and abs(lg.Limita180(abs(robo[g]-lg.posTemperaturaSx))) < 20):
                                            pidG.avvio = 0
                                            #lg.velDxTemp=lg.velSxTemp=lg.velDx=lg.velSx=0
                                            #casellaVittime.Avvia(controlloVittime.livello+1,'S'+'C')
                                            lg.trovataVittimaSx = 1
                                            lg.cosavittimeSx = 'SC'
                                            cprint("TROVATO SX CALORE DA PITAGORA",'green')
                                            casella.vittime[mappa.limit04(mappa.sempG(robo[g])+2)] = 'C'
                        
                    else:
                        lg.oldMuroSx = 0
                        
                        if(lg.okCamSx and lg.fTrovatoSx and lg.trovatoSx!="N"):
                            lg.posVisualeSx = robo[g]
                            lg.xVisualeSx = robo[x]
                            lg.yVisualeSx = robo[y]
                            lg.tipoVisualeSx = lg.trovatoSx
                            
                if(lg.stato == 'prova' and lg.Libero(0)):
                    pidG.avvio = 0
                    #vittime.Avvia(0,'D'+'H')
                    lg.avvioLaser = [1]*lg.nLaser
                    print(lg.laser[sn.lAvCe])
                    #print("luce!", lg.luce,"colore",lg.casella)
                if(lg.stato == 'primo90' and lg.Libero(0)):
                    pidG.avvio = 0 
                    lg.avvioLaser = [1]*lg.nLaser
                    lg.stato = 'verificoMuri'
                    lg.statoDopoVM = 'zero'

                elif(lg.stato == 'verificoMuri' and lg.Libero(0)):
                    lg.direzioneMuri = mappa.sempG(robo[g]) #direzione muri destra
                    if(lg.laser[sn.lAvDx] < 15 and lg.laser[sn.lDiDx]<15): #se almeno uno conferma il muro
                        casella.muri[lg.direzioneMuri] = 1
                    else:
                        casella.muri[lg.direzioneMuri] = 0
                    casellaDopo = mappa.map.map[mappa.map.modX(round(robo[x]),lg.direzioneMuri)][mappa.map.modY(round(robo[y]),lg.direzioneMuri)]
                    if(casellaDopo.esplorata!=0):
                        casellaDopo.muri[2-lg.direzioneMuri+2*(lg.direzioneMuri%2)]=casella.muri[lg.direzioneMuri]
                    
                    
                    lg.direzioneMuri = mappa.limit04(mappa.sempG(robo[g])+2) #direzione muri sinistra
                    if(lg.laser[sn.lAvSx] < 15 and lg.laser[sn.lDiSx]<15):
                        casella.muri[lg.direzioneMuri] = 1                        
                        #casella.muri[mappa.limit04(mappa.sempG(robo[g])+2)] = 1         
                    else:
                        casella.muri[lg.direzioneMuri] = 0
                    casellaDopo = mappa.map.map[mappa.map.modX(round(robo[x]),lg.direzioneMuri)][mappa.map.modY(round(robo[y]),lg.direzioneMuri)]
                    if(casellaDopo.esplorata!=0):
                        casellaDopo.muri[2-lg.direzioneMuri+2*(lg.direzioneMuri%2)]=casella.muri[lg.direzioneMuri]
                    
                    lg.direzioneMuri = mappa.limit04(mappa.sempG(robo[g])+1) #direzione muri avanti 
                    if(lg.laser[sn.lAvCe] < 15):
                        casella.muri[lg.direzioneMuri] = 1
                        #casella.muri[mappa.limit04(mappa.sempG(robo[g])+1)] = 1
                    else:
                        casella.muri[lg.direzioneMuri] = 0
                    casellaDopo = mappa.map.map[mappa.map.modX(round(robo[x]),lg.direzioneMuri)][mappa.map.modY(round(robo[y]),lg.direzioneMuri)]
                    if(casellaDopo.esplorata!=0):
                        casellaDopo.muri[2-lg.direzioneMuri+2*(lg.direzioneMuri%2)]=casella.muri[lg.direzioneMuri]


                    
                    lg.cpda = mappa.limit04(mappa.sempG(robo[g])+3)#casella prima direzione assoluta
                    if(casella.muri[lg.cpda] == None):
                        lg.cpx = mappa.modX(round(robo[x]), lg.cpda)
                        lg.cpy = mappa.modY(round(robo[y]), lg.cpda)
                        if(mappa.map.map[lg.cpx][lg.cpy].esplorata != 0):
                            casella.muri[lg.cpda]=0
                            
                    if(lg.statoDopoVM == 'zero'):
                        gira.Avvia(0,1,90)
                        lg.stato = 'zero'
                    else:
                        lg.stato = lg.statoDopoVM
                
          
                elif(lg.stato == 'zero' and lg.Libero(0)): #se lo stato impostato è 0 e il programma non sta eseguendo nessun'altro programma
                    self.putmappa(maze,0)
                    pidG.avvio = 0 #non voglio il pid giroscopio
                    lg.Resettino() #resetto variabili apposite logica (prx, pry, nodestra ecc)
                    lg.stato = 'coloreCasella' #aumento lo stato
                    lg.distanza=0
                    esp.ResetDistanza()
                    
                    
                elif(lg.stato == 'coloreCasella' and lg.Libero(0)): 
                    #lg.ColoreCasella() #eseguo il calcolo per trovare il colore della casella (risultato in lg.casella
                    if(lg.lackcheck ==0):
                    
                        lg.distColCas = (((robo[x]-lg.xColCas)**2 + (robo[y]-lg.yColCas)**2)**(1/2))*30
                        print("@@@@@ distanza COl:",lg.distColCas,lg.casella,lg.luce)
                        if(round(robo[x]) == mappa.initX and round(robo[y]) == mappa.initX):
                            casella.colore = lg.argento-1
                            
                        elif(lg.distColCas>=6):
                            casella.colore = lg.casella-1
                            print("IMPOSTO COLORE NORMALE",casella.colore,lg.casella)
                        else:
                            casella.colore = lg.bianco-1
                        
                        if (casella.colore==lg.argento-1 ):
                            try:
                                lg.checkpoint.remove((round(robo[x]),round(robo[y])))
                            except ValueError:
                                None
                            
                            lg.checkpoint.insert(0,(round(robo[x]),round(robo[y])))
                            #mappa.checkPointX = round(robo[x])
                            #mappa.checkPointY = round(robo[y])
                    lg.lackcheck = 0
                    lg.stato = 'centro' # aumento  lo stato 


                elif(lg.stato == 'centro' and lg.Libero(0)):
                    pidG.avvio = 1
                    if(casella.colore!=lg.nero-1):
                        centra.Avvia(0) #avvio il CentraY nella casella
                    lg.stato = 'verificoMuri'
                    lg.statoDopoVM = 'centramentolaser'
                    lg.statoDopoCL = 'controlloDopoCentra'
                    lg.avvioLaser = [1]*lg.nLaser
                    
                elif(lg.stato == 'centramentolaser' and lg.Libero(0)):
                    #robo[x] = round(robo[x])
                    #robo[y] = round(robo[y])
                    lg.clg = mappa.sempG(robo[g])
                    if(casella.muri[mappa.limit04(mappa.sempG(robo[g]))]
                       and lg.laser[sn.lAvDx]<20
                       and lg.laser[sn.lDiDx]<20
                       and abs(lg.laser[sn.lAvDx]-lg.laser[sn.lDiDx])<3):
                        lg.ccldx = -(13-(lg.laser[sn.lAvDx]+lg.laser[sn.lDiDx]))/60 #correzione centramento laser dx 
                    else:
                        lg.ccldx = None
                     
                    if(casella.muri[mappa.limit04(mappa.sempG(robo[g])+2)]
                       and lg.laser[sn.lAvSx]<20
                       and lg.laser[sn.lDiSx]<20
                       and abs(lg.laser[sn.lAvSx]-lg.laser[sn.lDiSx])<3):
                        lg.cclsx = (13-lg.laser[sn.lAvSx]-lg.laser[sn.lDiSx])/60
                    else:
                        lg.cclsx = None
                        
                    if(lg.cclsx != None and lg.ccldx != None):
                        lg.ccl = (lg.cclsx+lg.ccldx)/2 #correzione centramento laser
                    elif(lg.cclsx != None):
                        lg.ccl = lg.cclsx
                    elif(lg.ccldx != None):
                        lg.ccl = lg.ccldx
                    else:
                        lg.ccl = 0
                        
                    lg.clvy =  (lg.clg%2) * (lg.clg-2)  #0->0, 1->-1, 2->0, 3->1
                    lg.clvx =  (lg.clg%2 -1) * (lg.clg-1) #0->1, 1->0, 2->-1, 3->0
                    robo[x] = round(robo[x]) - lg.clvx*lg.ccl
                    robo[y] = round(robo[y]) + lg.clvy*lg.ccl
                    #print(robo[x],robo[y],lg.cclsx,lg.ccldx,lg.ccl,lg.clvx,lg.clvy,lg.clg)
                    lg.stato = lg.statoDopoCL#'controlloDopoCentra'
                    
                    
                    
                elif(lg.stato == 'controlloDopoCentra' and lg.Libero(0)):
                    #if(casella.muri[mappa.limit04(mappa.sempG(robo[g])+1)]):
                    controlloVittime.Avvia(0)
                    lg.stato = 'scelta'
                    
                        

                
                elif(lg.stato == 'scelta' and lg.Libero(0)):
                    #print("inizio scelta")
                    casella.esplorata +=1
                    casella.rampa = None
                    lg.erroremappa = mappa.ControlloMappa(mappa.map)
                    if(lg.erroremappa>0):
                        #print("errore mappa",lg.erroremappa)
                        robo[mod]=2


                    
                    mappa.esci = 0
                    mappa.sRelEsci = 0
                    mappa.gira = 0
                    mappa.direzione = 1
                    mappa.numEsplorate = 0
                    if(casella.colore==2):
                        #print("scelgo nero")
                        mappa.gira = 0
                        mappa.direzione = -1
                    else:
                        mappa.inespolorate = {}
                        mappa.numInesplorate = 0
                        
                        mappa.dRelesciInesp = None
                        for dRelativa in range(4):
                            mappa.G = mappa.sempG(robo[g])
                            mappa.dAssoluta = mappa.limit04(mappa.G + dRelativa)
                            mappa.colleg = not(casella.muri[mappa.dAssoluta])
                            mappa.mox = mappa.modX(round(robo[x]),mappa.dAssoluta)
                            mappa.moy = mappa.modY(round(robo[y]),mappa.dAssoluta)
                            mappa.numInesplorate = 0
                            if(mappa.colleg):
                                #print("trovato colleg",dRelativa)
                                if(mappa.map.map[mappa.mox][mappa.moy].esplorata==0):
                                    #print("colleg ok")
                                    mappa.esci = 1
                                    mappa.dRelEsci = dRelativa
                                    break
                                else:
                                    mappa.numEsplorate+=1
                        mappa.gira = None
                        #print("scelgo gira")
                        '''
                        if(mappa.numInesplorate==1):
                            
                            mappa.dRelEsci = mappa.dRelesciInesp
                            mappa.esci = 1
                        elif(mappa.numInesplorate>1):
                            mappa.dRelEsci = max(mappa.inespolorate,key=mappa.inespolorate.get)
                            mappa.esci = 1
                        '''
                        mappa.map.Pulisci()

                        if(mappa.esci):
                            mappa.gira = mappa.limita180((5-mappa.dRelEsci)*90)
                            #print("esco per esci gira:",mappa.gira)
                        else:
                            '''
                            if(mappa.numEsplorate == 1):
                                mappa.gira = 180
                                print("esco gira:",mappa.gira)
                            '''
                            if(mappa.gira==None):
                                #print("eseguo DK")
                                tempd = mappa.EseguiDijkstra(round(robo[x]),round(robo[y]),mappa.map)
                                mappa.dAssoluta,temp = mappa.TrovaDirezione(mappa.map,round(robo[x]),round(robo[y]))
                                #print("EseguiDijkstra + TrovaDirezione" ,tempd+temp,temp)
                                if(mappa.dAssoluta==None):
                                    if(round(robo[x]) == mappa.initX and round(robo[y]) == mappa.initY):
                                        print("ABBIAMO FINITO TUTTO IL LABIRINTO -----------------------------------------------------------------------")
                                        print("RICOMINCIO")
                                        g_temp, _, _ = self.sn.Giroscopio(1)  
                                        #self.sn.set_correzione(self.logica.Limita180(g_temp-self.logica.gradiVolutiZ),0)
                                        self.sn.set_correzione(g_temp,0)
                                        
                                        self.logica.gradiVolutiZ = 0
                                        #rk.clean()
                                        #self.rk = rk = MotoriRK()
                                        #
                                        lg.numKit = None #numro di kit disponibili
                                        lg.numLack = 0
                                        pidG.avvio = 0
                                        lg.velDxTemp=0
                                        lg.velSxTemp=0
                                        lg.velDx = 0
                                        lg.velSx = 0
                                        self.pidG = pidG = robot.LibPID(0.06,0.4,0.005) 
                                        lg.RSS(0)
                                        sn.SpegniLedCamera()
                                        led.SpegniLed() 
                                        mappa.map.Vuoto(mappa.numX,mappa.numY)
                                        maze = mappa.map.map
                                        robo = mappa.robo
                                        mappa.modX = mappa.map.modX
                                        mappa.modY = mappa.map.modY
                                        robo[x] = mappa.initX
                                        robo[y] = mappa.initY
                                        #robo[g] = 0
                                        lg.lack= 1
                                        lg.avvioCamDx[0]=0
                                        lg.avvioCamSx[0]=0
                                        lg.avvioCamDx[1]=1
                                        lg.avvioCamSx[1]=1
                                        lg.eccDx = 0
                                        lg.eccSx = 0
                                        lg.numLack = 0
                                        lg.okCamDx = 1
                                        lg.okCamSx = 1
                                        lg.checkpoint = []
                                        while(esp.Pesp()==1):
                                            time.sleep(0.2)
                                        if(lg.rilsciaKit):
                                            rk.MotAvvio(-100,20,1,10)
                                            rk.MotRuota(100,200)
                                        
                                    else:
                                        mappa.dAssoluta,temp = mappa.TrovaDirezione(mappa.map,round(robo[x]),round(robo[y]),mappa.initX,mappa.initY)
                                        if(mappa.dAssoluta!=None):
                                    
                                            mappa.G = mappa.sempG(robo[g])
                                            mappa.gira = mappa.limita180((5-mappa.dAssoluta+mappa.G)*90)
                                else:
                                    
                                    mappa.G = mappa.sempG(robo[g])
                                    mappa.gira = mappa.limita180((5-mappa.dAssoluta+mappa.G)*90)

                    #print("fine scelta")
                    if(lg.lack != 1):
                        if(mappa.direzione==-1):
                            pidG.avvio = 1
                            dritto.Avvia(0,-1,30)
                            lg.stato = 'fine'
                        elif(abs(mappa.gira)==180 and casella.muri[mappa.limit04(mappa.sempG(robo[g]) + 1)] and
                            casella.vittime[mappa.limit04(mappa.sempG(robo[g] )+1)] == None ):
                            gira.Avvia(0,1,mappa.gira/2)
                            lg.sceltogira = mappa.gira/2
                            lg.stato = 'controlloDopo90-180'
                        else:
                            if(mappa.gira!=0):
                                gira.Avvia(0,1,mappa.gira)
                                lg.stato = 'allinea'
                            else:
                                lg.stato = 'inizioAvanti'
                            lg.sceltogira = mappa.gira

                    #if(abs(mappa.gira)==180):
                    #print("gira 18, muro:",casella.muri[mappa.limit04(mappa.sempG(robo[g]) + 1)] ,"vittima",casella.vittime[mappa.limit04(mappa.sempG(robo[g] +1 ))])
                elif(lg.stato == 'controlloDopo90-180' and lg.Libero(0)):
                    lg.stato = 'secondo90'
                    controlloVittime.Avvia(0)
                    
                    
                elif(lg.stato == 'secondo90' and lg.Libero(0)):
                    gira.Avvia(0,1,lg.sceltogira) #ultimo 90 del 180
                    lg.sceltogira = lg.sceltogira * 2
                    lg.stato = 'allinea'
                    
                        
                elif(lg.stato == 'allinea' and lg.Libero(0)): #Allinea
                    if(casella.muri[mappa.limit04(mappa.sempG(robo[g]) + 3)] == 1 and casella.vittime[mappa.limit04(mappa.sempG(robo[g]) + 3)]!='C' ):
                        print("ALLINEO")
                        pidG.avvio = 0 #non pid giroscopioo
                        allinea.Avvia(0)  #avvio l'allinea a livello 0
                        lg.flAllinea = 1
                        lg.stato = 'avantiAllinea'
                    else:
                        #lg.distanza=0
                        #esp.ResetDistanza()
                        lg.flAllinea = 0
                        lg.stato = 'inizioAvanti'
                    
                    
                elif(lg.stato == 'avantiAllinea' and lg.Libero(0)): #avanti dopo allinea
                    #dritto.Avvia(0,0.5,4)
                    lg.stato = 'inizioAvanti' #6
                    
      
                elif(lg.stato =='inizioAvanti' and lg.Libero(0)):
                    #if(not(lg.sceltogira==0 or (abs(lg.sceltogira)==180 and lg.okCamSx==1 and lg.okCamDx==1))):
                    if(lg.sceltogira!=0 and abs(lg.sceltogira)!=180):
                        controlloVittime.Avvia(0)
                    lg.stato = 'centramentolaser'
                    lg.statoDopoCL =  'avanti'
                    #lg.stato = 'avanti' #6.5
                    
                    
                elif(lg.stato == 'avanti' and lg.Libero(0)):
                    pidG.avvio = 1
                    if(lg.flAllinea):
                        dritto.Avvia(0,1.3,35.5) # normali 30
                    else:
                        dritto.Avvia(0,1.3,31.5) # normali 30
                    lg.stato = 'avantiLibero'
                    lg.flNero = 0
                    #lg.trovataVittima = 0
                    lg.flTrovataVittima = 0
                    lg.avvioLaser[sn.lAvCe]=1
                    lg.distVittime = None
                    lg.inizioRampa = None
                    
                ###############################################################################################################################
                elif(lg.stato == 'avantiLibero'): #AVANTI NON BLOCCANTE
                    #controllare inclinazione
                    lg.avvioLaser[sn.lAvCe]=1          
                    if(lg.casella == lg.nero and lg.flNero==0):
                        lg.distanzaColore =  (((robo[x]-lg.xColCas)**2 + (robo[y]-lg.yColCas)**2)**(1/2))*30
                        if(lg.distanzaColore>2):
                            lg.velSx = lg.velDx = 0 
                            lg.flNero = 1
                            dritto.Avvia(0,-1,lg.distanza)
                            
                            direz = mappa.sempG(robo[g])
                            aggX = (direz%2)*(direz-2)  * -0.8
                            aggY = ((direz%2) -1) * (direz-1) * -0.8
                            casellaDopo = mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)]
                            casellaDopo.colore = lg.nero-1
                            casellaDopo.esplorata +=1
                            #mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+1)]=0
                            #mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+3)]=0
                            for indiceMuro in range(4):
                                lg.direzioneMuri = mappa.limit04(direz+indiceMuro)
                                casellaDopo2X = mappa.map.modX(round(robo[x]-aggX),lg.direzioneMuri)
                                casellaDopo2Y = mappa.map.modY(round(robo[y]-aggY),lg.direzioneMuri)
                                casellaDopo2 = mappa.map.map[casellaDopo2X][casellaDopo2Y]
                                if(casellaDopo2.esplorata!=0):
                                    casellaDopo.muri[lg.direzioneMuri]=casellaDopo2.muri[ 2-lg.direzioneMuri+2*(lg.direzioneMuri%2)]
                                else:
                                    casellaDopo.muri[lg.direzioneMuri]=1
                    
                    
                    
                    
                    if(lg.flNero==1):
                        print("nero")
                        if(lg.Libero(0)):
                             lg.stato = 'fine'
                            
                    elif(lg.laser[sn.lAvCe]<=7 and lg.azione[0]!=finecorsa.nome): #se ho un muro vicino davanti e non sto facendo i finecorsa
                        print("Fermo per laser FR" ,lg.laser[sn.lAvCe])
                        lg.azione[0] = lg.libero  # fermo qualsiasi programma dichiarando il livello 0 libero
                        #controlloVittime.Avvia(0)
                        #dritto.Avvia(0,-0.8,5)
                        lg.stato ='fine'
                        
                    elif(lg.inclinazione!=sn.incPiano): #se sono in salita o discesa normale 
                        lg.RSS(0)
                        lg.azione[0] = "rampa"#lg.libero
                        if(lg.inclinazione==sn.incErrore):
                            #rint("errore inc",'grey')
                            if(lg.gradiY>0): #ribaltamento indietro
                                lg.velDx = lg.velSx = -1.5 
                            else: #ribaltamento in avanti
                                lg.velDx = lg.velSx = 1.5
                        else:
                            lg.velDx = lg.velSx = 1#0.6 #vado più piano
                        if(lg.inizioRampa == None):
                            print("inizio rampa")
                            #cprint("salita o discesa",'grey')
                            lg.inizioRampa = time.time()
                            lg.distRamp = lg.distanza
                            lg.incRamp = lg.inclinazione
                    elif(lg.inizioRampa != None and (time.time()-lg.inizioRampa) > 0.6):#c'è una rampa
                        print("rampa")
                        direz = mappa.sempG(robo[g])
                        aggX = (direz%2)*(direz-2)  * 0.8
                        aggY = ((direz%2) -1) * (direz-1) * 0.8
                        direzLim = mappa.limit04(mappa.sempG(robo[g])+1)
                        #print("IMPOSTO RAMPA",round(robo[x],3),round(robo[y],3),"inc:",lg.incRamp,"direz:",direz,"agg:",aggX,aggY,"finale",round(robo[x]-aggX),round(robo[y]-aggY))
                        if(lg.incRamp==sn.incSalita):
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].rampa = 2-direzLim+2*(direzLim%2) #mappa.limit04(mappa.sempG(robo[g])+1)
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].colore = 0
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].esplorata +=1
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g]))]=1
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+2)]=1
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+1)]=0
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+3)]=0
                        elif(lg.incRamp==sn.incDiscesa):
                            
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].rampa = direzLim #2-direzLim+2*(direzLim%2)
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].colore = 0
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].esplorata +=1
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g]))]=1
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+2)]=1
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+1)]=0
                            mappa.map.map[round(robo[x]-aggX)][round(robo[y]-aggY)].muri[mappa.limit04(mappa.sempG(robo[g])+3)]=0
                        dritto.Avvia(0,0.8,10)
                        lg.inizioRampa = None
                    elif(lg.inizioRampa != None and lg.distRamp!=None ):
                        print("fine rampa")
                        dritto.Avvia(0,0.8,32-lg.distRamp+3)#-(time.time()-lg.inizioRampa))
                        lg.inizioRampa = None
                    elif( sn.ReadEndRun(sn.fAvDx) and lg.azione[0]!=finecorsa.nome and lg.laser[sn.lAvCe]>7.5): #premuto finecorsa avanti di Dx
                        print("finecorsasx")
                        pidG.avvio = 1
                        finecorsa.AvviaAvDx(0)
                        lg.stato = 'ricentra'
                    elif( sn.ReadEndRun(sn.fAvSx) and lg.azione[0]!=finecorsa.nome and lg.laser[sn.lAvCe]>7.5):
                        print("finecorsaSx")
                        pidG.avvio = 1
                        finecorsa.AvviaAvSx(0)
                        lg.stato = 'ricentra'
                    elif( sn.ReadEndRun(sn.fAvCe) and lg.azione[0]!=finecorsa.nome): #premuto finecorsa avanti centrale
                        print("finecorsaCE")
                        pidG.avvio = 1
                        dritto.Avvia(0,-0.8,5)  #vado indietro di 5 cm (DA RIGUARDARE)
                        lg.stato = 'fine'
                        '''
                        elif(lg.azione[0]==finecorsa.nome):
                            print("ritorno stato 6 x allinea")
                            #lg.flAllinea = 0 #imposto che non sono allineato indietro il prossimo avanti 30
                        
                            lg.stato = 'fine' # ritorno allo stato 6 per ricominciare l'avanti
                        '''
                    elif ((lg.trovataVittimaDx or lg.trovataVittimaSx ) and lg.flTrovataVittima == 0):
                        print("cv1")
                        if(lg.flAllinea):
                            dipv = lg.distanza -5
                        else:
                            dipv = lg.distanza
                        if(dipv<0):
                            vipv = 1
                            dipv = -dipv
                        else:
                            vipv = -1
                        
                        print(lg.flAllinea,lg.distanza,vipv,dipv)
                            
                        if(dipv>0.5):
                            dritto.Avvia(0,vipv,dipv)
                        else:
                            lg.azione[0]=lg.libero
                        lg.flTrovataVittima = 2 
                    elif (lg.flTrovataVittima ==2 and lg.Libero(0)):
                        print("cv2")
                        if(lg.trovataVittimaDx):
                            vittime.Avvia(0,lg.cosavittimeDx)
                            lg.trovataVittimaDx = 0
                        lg.flTrovataVittima = 3
                    elif (lg.flTrovataVittima ==3 and lg.Libero(0)):
                        print("cv3")
                        if(lg.trovataVittimaSx):
                            vittime.Avvia(0,lg.cosavittimeSx)
                            lg.trovataVittimaSx = 0
                        lg.flTrovataVittima = 4
                    elif (lg.flTrovataVittima ==4 and lg.Libero(0)):
                        print("cv4")
                        lg.stato = 'fine'
                        lg.flTrovataVittima = 0
                        
                    elif(lg.laser[sn.lAvCe]<=20 and lg.Libero(0)): # continuo ad avanzare per avvicinarmi giusto al muro avanti
                        #print("Continua per laser FR" ,lg.laser[sn.lAvCe])
                        lg.azione[0] = "avantiLaser"#lg.libero
                        lg.velDx = lg.velSx = 0.4 #avanzo letamente per debug
                    elif(lg.Libero(0)): #se ha finito di fare qualsiasi cosa al livello 0
                        lg.stato = 'fine'
                        #print("esci per azione 0 free ",lg.laser[sn.lAvCe])

                elif(lg.stato == 'ricentra' and lg.Libero(0)): #Stato finale
                    #prosegui.Avvia(0,0.5,0)
                    lg.stato = 'fine'
                elif(lg.stato == 'fine' and lg.Libero(0)): #Stato finale
                    
                    lg.velDx = lg.velSx = 0 #fermo i motori per sicurezza
                    lg.stato ='zero' #ricomincio il cilo
                    
                
              
                    
                    
                
                
                #FINE LOGICA
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #FUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONIFUNZIONI
                
                #PROSEGUI
                if(prosegui.avvio):
                    if(prosegui.stato == 0):
                        lg.RSS(prosegui.livello)
                        lg.azione[prosegui.livello]=prosegui.nome 
                        prosegui.G = mappa.sempG(robo[g])
                        prosegui.vx =  (prosegui.G%2) * (prosegui.G-2)
                        prosegui.vy =  (prosegui.G%2 -1) * (prosegui.G-1)
                        prosegui.dx = None
                        prosegui.dy = None
                        if(prosegui.vx!=0):
                            prosegui.dx = 30*prosegui.vx*(round(robo[x])+(prosegui.mod*prosegui.vx)-robo[x])
                            if(prosegui.dx<0):
                                prosegui.velocitadx = -prosegui.velocita
                                prosegui.dx = -prosegui.dx
                            else:
                                prosegui.velocitadx = prosegui.velocita
                                    
                            dritto.Avvia(prosegui.livello+1,prosegui.velocitadx,prosegui.dx)
                        if(prosegui.vy!=0):
                            prosegui.dy = 30*prosegui.vy*(round(robo[y])+(prosegui.mod*prosegui.vy)-robo[y])
                            if(prosegui.dy<0):
                                prosegui.velocitady = -prosegui.velocita
                                prosegui.dy = -prosegui.dy
                            else:
                                prosegui.velocitady = prosegui.velocita
                            dritto.Avvia(prosegui.livello+1,prosegui.velocitady,prosegui.dy)
                        prosegui.stato = 1
                        cprint("PROSEGUI",'red')
                        print(prosegui.vx,prosegui.dy, prosegui.dx,prosegui.dy)
                    elif(prosegui.stato == 1 and lg.Libero(prosegui.livello+1)):
                        lg.azione[prosegui.livello]=lg.libero 
                        prosegui.avvio = 0
                    
                #FINE PROSEGUI
                
                #DRITTO
                if(dritto.avvio): #se è stato attivato (non usare .avvio per disattivarlo)
                    if(dritto.stato==0): #se lo stato del dritto è lo 0
                        lg.RSS(dritto.livello)
                        lg.azione[dritto.livello]=dritto.nome #imposto l'azione generale a livello (.livello) impostato a dritto (.nome)
                        esp.ResetDistanza() #resetto distanza esp
                        lg.distanza=0  
                        dritto.Resettino() #resetto le variabili secondarie del dritto, quelle che non sono di stato
                        dritto.distanzaVoluta = abs(dritto.distanzaVoluta)
                        if(dritto.velocita>0): #Calcolo componenti retta finale di rallentamento
                            dritto.qRetta = 0.2 #finirò con una velocità di 0.2 se sto andando in avanti (velocità>0)
                        else:
                            dritto.qRetta = -0.2
                        dritto.qRettaI = 0.6*dritto.velocita
                        dritto.distanzaRetta = 0.2*dritto.distanzaVoluta
                        dritto.mRetta = (dritto.velocita-dritto.qRetta)/dritto.distanzaRetta #il coefficente angolare è dipendente dal'intercetta (q), la velocità voluta e la grandezzza della retta finale
                        dritto.mRettaI = (dritto.velocita-dritto.qRettaI)/dritto.distanzaRetta
                        print("VADO DRITTO",dritto.distanzaVoluta,dritto.velocita,dritto.distanzaRetta,"Retta",dritto.mRetta,dritto.qRetta,"RettaI",dritto.mRettaI,dritto.qRettaI)
                        dritto.stato = 1 #aumento lo stato
                        dritto.startemp = time.time()
                    elif(dritto.stato == 1 and lg.azione[dritto.livello] == dritto.nome): #se lo stato è a 1 e l'azione appartiene ancora a dritto 
                        if(dritto.velocita<0): #calcolo distaza percorsa relativamnte alla velocità impostata (avanti o indietro)
                            dritto.distanzaPercorsa = -lg.distanza
                        else:
                            dritto.distanzaPercorsa = lg.distanza
                        dritto.distanzaMancante = dritto.distanzaVoluta-dritto.distanzaPercorsa #calcolo distanza che mi manca per finire il percorso
                        if(dritto.distanzaMancante>0 and (time.time()-dritto.startemp)<(dritto.distanzaVoluta*0.11)):
                            if(dritto.distanzaPercorsa>=0 and dritto.distanzaPercorsa<dritto.distanzaRetta):
                                lg.velSx = lg.velDx = (dritto.mRettaI * dritto.distanzaPercorsa) + dritto.qRettaI
                            elif(dritto.distanzaMancante<dritto.distanzaRetta):
                                lg.velSx = lg.velDx = (dritto.mRetta * dritto.distanzaMancante) + dritto.qRetta
                            else:
                                lg.velSx = lg.velDx = dritto.velocita
                        else:
                            lg.velSx = lg.velDx = 0 #mi fermo
                            lg.azione[dritto.livello]=lg.libero #imposto che ho finito il dirtto al mio livello
                            dritto.errore = dritto.distanzaVoluta-dritto.distanzaPercorsa #calcolo l'errore commesso
                            print(dritto.nome," e:",dritto.errore)
                            dritto.avvio = 0 #disattivo il dritto
                        '''
                        if(dritto.distanzaMancante>dritto.distanzaRetta): #se è fuori dall'azione della retta, imposta la massima velocità voluta
                            lg.velSx = lg.velDx = dritto.velocita
                        elif (dritto.distanzaMancante>0): # se non ho sforato la distanza calcolo la retta di rallentamento 
                            lg.velSx = lg.velDx = (dritto.mRetta * dritto.distanzaMancante) + dritto.qRetta
                        else:
                            lg.velSx = lg.velDx = 0 #mi fermo
                            lg.azione[dritto.livello]=lg.libero #imposto che ho finito il dirtto al mio livello
                            dritto.errore = dritto.distanzaVoluta-dritto.distanzaPercorsa #calcolo l'errore commesso
                            print(dritto.nome," e:",dritto.errore)
                            dritto.avvio = 0 #disattivo il dritto
                        '''
                    elif(lg.azione[dritto.livello] != dritto.nome): #se l'azione al livello voluto non è più il dritto (ma .avvio è ancora valido)
                                                                    # vuol dire che il dritto è stato bloccato o sostituito da un'altra azione
                        dritto.errore = dritto.distanzaVoluta-dritto.distanzaPercorsa #calcolo errore
                        print(dritto.nome," bloccato e:",dritto.errore, 'da',lg.azione[dritto.livello] ) 
                        dritto.avvio = 0 #disattivo l'avvio
                #FINE DRITTO
                        
                #GIRA 
                if(gira.avvio):
                    lg.avvioCamDx[0]=0
                    lg.avvioCamSx[0]=0
                    if(gira.stato==0):
                        
                        gira.Resettino()
                        lg.RSS(gira.livello)
                        lg.azione[gira.livello] = gira.nome
                        esp.ResetDistanza()
                        lg.distanza = 0
                        gira.gradiAdesso =  gira.gradiUltimo = gira.gradiIniziali = lg.gradiZ #inizializzo tutte le varibili dei gradi

                        gira.gradiModifica = lg.Limita180(lg.gradiVolutiZ + gira.gradiVoluti) #calcolo gradi finali assoluti a partire dalla posizione
                                                                                              # teorica in cui dovrei essere adesso (gradiVolutiZ)
                        lg.gradiVolutiZ = gira.gradiModifica #aggiorno i gradi voluti asse Z generali 
                        gira.gradiVoluti = lg.Limita180(gira.gradiModifica - gira.gradiIniziali) #calcolo quanti gradi devo effetivamnete girare dalla mia posizione effetiva
                        
                        if(gira.gradiVoluti>0): #scelgo il verso in cui girare (orario o antiorario)
                            gira.versoGira = 1
                        else:
                            gira.versoGira = -1
                        gira.stato = 1
                        gira.startemp = time.time()
                        lg.avvioGiroscopio = 1
                        gira.movDrittoA = 0
                        gira.movDrittoB = 0
                    elif(gira.stato == 1 and lg.azione[gira.livello] == gira.nome): #se l'azione al livello voluto è ancora del gira
                        lg.avvioGiroscopio = 1
                        gira.gradiAdesso = lg.gradiZ #prendo i gradi attuali 
                        if( (gira.gradiAdesso - gira.gradiUltimo)>180): #calolo il numero di giri completi (360 * kGiri) dalle variazioni estreme (>180° --> passa da 359 a 0)
                            gira.kGiri-=1
                        elif( (gira.gradiAdesso - gira.gradiUltimo)<-180):
                            gira.kGiri+=1
                        gira.gradiMancanti = gira.gradiVoluti - gira.gradiAdesso - (gira.kGiri*360) + gira.gradiIniziali #calcolo gradi che mi mancano per arrivare alla fine

                        if (gira.gradiVoluti<0): #imposti i gradi mancanti negativi se volevo girare di -tot gradi
                            gira.gradiMancanti*=-1
                        gira.gradiUltimo = gira.gradiAdesso #aggiorno il precendete valore dei gradi (per il calcolo dei giri completi)

                        if(gira.gradiMancanti>0): #se manca ancora alla fine
                            if(lg.Libero(gira.livello+1)):
                                if((time.time()-gira.startemp)>5 and gira.movDrittoA==0): #(gira.gradiVoluti*0.04)
                                    if(sn.ReadEndRun(sn.fAvSx) or sn.ReadEndRun(sn.fAvDx)  or sn.ReadEndRun(sn.fAvCe) ):
                                        dritto.Avvia(gira.livello+1,-1,3)
                                    elif(sn.ReadEndRun(sn.fDiSx) or sn.ReadEndRun(sn.fDiDx)):
                                        dritto.Avvia(gira.livello+1,1,3)
                                    else:
                                        dritto.Avvia(gira.livello+1,1,3)
                                    gira.movDrittoA = 1
                                    print("MOVDRITTOA GIRA",time.time()-gira.startemp,gira.gradiVoluti)
                                elif((time.time()-gira.startemp)>10 and gira.movDrittoB==0): #(gira.gradiVoluti*0.09)
                                    if(sn.ReadEndRun(sn.fAvSx) or sn.ReadEndRun(sn.fAvDx)  or sn.ReadEndRun(sn.fAvCe) ):
                                        dritto.Avvia(gira.livello+1,-1,6)
                                    elif(sn.ReadEndRun(sn.fDiSx) or sn.ReadEndRun(sn.fDiDx)):
                                        dritto.Avvia(gira.livello+1,1,6)
                                    else:
                                        dritto.Avvia(gira.livello+1,-1,6)
                                    gira.movDrittoB = 1
                                    print("MOVDRITTOB GIRA",time.time()-gira.startemp,gira.gradiVoluti)
                                elif( gira.gradiMancanti <= gira.gradiMaxRetta and gira.gradiMancanti>=gira.gradiMinRetta): #se i gradi sono dentro all'azione della retta 
                                    gira.velocitaMod = (gira.gradiMancanti * gira.mRetta) + gira.qRetta #0.0224/0.08#0.033 0.4  #calcolo velocità da retta
                                else:
                                    gira.velocitaMod = gira.velocita#1.4 #1.2 #altrimenti velocità massima
        
                                lg.velDx = gira.velocitaMod*gira.versoGira*-1.0 #imposto le coppie 
                                lg.velSx = lg.velDx* -1
                        else:
                            esp.ResetDistanza()
                            lg.distanza = 0
                            lg.velDx = lg.velSx = 0 #mi fermo
                            lg.azione[gira.livello] = lg.libero #imposto che ho finito il gira
                            lg.azione[gira.livello+1] = lg.libero
                            gira.errore = gira.gradiVoluti - gira.gradiAdesso - (gira.kGiri*360) + gira.gradiIniziali  #calcolo errore
                            #print(gira.nome," e:",gira.errore)
                            gira.avvio = 0 #disabilito il gira
                            
                        #if((time.time()-gira.startemp)>2 and (time.time()-gira.startemp)<2.4):
                        #    print("GIRA BLOCCATO manc:",gira.gradiMancanti,"vol:",gira.gradiVoluti,"adess",gira.gradiAdesso,"iniz:",gira.gradiIniziali,"kgiri:",gira.kGiri)#, "gyr1",sn.Giroscopio(1))
                    elif(lg.azione[gira.livello] != gira.nome): #se è stato bloccato (non ha finito in autonomia)
                        gira.errore = gira.gradiVoluti - gira.gradiAdesso - (gira.kGiri*360) + gira.gradiIniziali
                        lg.azione[gira.livello+1] = lg.libero
                        print(gira.nome," bloccato e:",gira.errore, 'da',lg.azione[gira.livello])
                        gira.avvio = 0
                        
                            
                #FINE GIRA 
            
                
                #FINECORSA
                if(finecorsa.avvio):
                    
                    if(finecorsa.stato==0):
                        lg.RSS(finecorsa.livello)
                        lg.azione[finecorsa.livello]=finecorsa.nome
                        if(finecorsa.posizione == 0 or finecorsa.posizione == 1): #se il finecorsa selezionato è lo 0 o 1 (possibilità di aggiungerne altri)
                            dritto.Avvia(finecorsa.livello+1,-0.6,5) #indietro di 5 cm, avvio il Dritto al livello sottostante (livello+1), altrimenti avrei sovrascitto la funzioen chiamante (fiencorsa)
                            finecorsa.stato=0.5
                            finecorsa.dx = 12
                            
                    elif(lg.azione[finecorsa.livello+1] == lg.libero and finecorsa.stato>0 and lg.azione[finecorsa.livello]==finecorsa.nome):
                        # se lo stato non è lo 0 e l'azione al livello sottostante è vuota (libero) e l'azione al livello voluto è ancora il finecorsa
                        if(finecorsa.stato==0.5):
                            finecorsa.stato=1
                            if(finecorsa.posizione == 0):
                                lg.avvioLaser[sn.lAvSx]=1
                                lg.avvioLaser[sn.lDiSx]=1
                            if(finecorsa.posizione == 1):
                                lg.avvioLaser[sn.lAvDx]=1
                                lg.avvioLaser[sn.lDiDx]=1
                        elif(finecorsa.stato==1):
                            if(finecorsa.posizione==0): #se è il finecorsa 0 (Dx)
                                if(lg.laser[sn.lAvSx]<22 and lg.laser[sn.lDiSx]<22 and False):
                                    finecorsa.dLaser = (lg.laser[sn.lAvSx]+lg.laser[sn.lDiSx])/2 + 8
                                    finecorsa.dy = 15-finecorsa.dLaser
                                    finecorsa.alfa = math.atan(finecorsa.dy/finecorsa.dx)
                                    if(finecorsa.alfa<-40):
                                        finecorsa.alfa=-40
                                    finecorsa.mAvanti = finecorsa.dy/math.sin(finecorsa.alfa)
                                    finecorsa.mIndietro = finecorsa.mAvanti * math.cos(finecorsa.alfa)
                                else:
                                    finecorsa.alfa = -30
                                    finecorsa.mAvanti = 10
                                    finecorsa.mIndietro = 10
                                
                                
                                gira.Avvia(finecorsa.livello+1, 0.8,finecorsa.alfa) #gira di -35 gradi 
                                
                                #gira -35
                            if(finecorsa.posizione==1):
                                if(lg.laser[sn.lAvDx]<22 and lg.laser[sn.lDiDx]<22 and False):
                                    finecorsa.dLaser = (lg.laser[sn.lAvDx]+lg.laser[sn.lDiDx])/2 + 8
                                    finecorsa.dy = finecorsa.dLaser-15
                                    finecorsa.alfa = math.atan(finecorsa.dy/finecorsa.dx)
                                    if(finecorsa.alfa>40):
                                        finecorsa.alfa=40
                                    finecorsa.mAvanti = finecorsa.dy/math.sin(finecorsa.alfa)
                                    finecorsa.mIndietro = finecorsa.mAvanti * math.cos(finecorsa.alfa)
                                else:
                                    finecorsa.alfa = 30
                                    finecorsa.mAvanti = 10
                                    finecorsa.mIndietro = 10
                                gira.Avvia(finecorsa.livello+1,0.8, finecorsa.alfa) #se è il finecorsa 1 (Sx)
                                #gira 35
                            finecorsa.stato=2
                        elif(finecorsa.stato==2):
                            dritto.Avvia(finecorsa.livello+1,0.6,finecorsa.mAvanti) # vai avati di 16cm
                            finecorsa.stato=3
                        elif(finecorsa.stato==3):
                            if(finecorsa.posizione==0):
                                gira.Avvia(finecorsa.livello+1,0.8,-finecorsa.alfa) #rimettersi dritto, gira di -35 gradi 
                            else:
                                gira.Avvia(finecorsa.livello+1,0.8,-finecorsa.alfa) #rimettersi dritto, gira di -35 gradi 
                            finecorsa.stato = 4
                        elif(finecorsa.stato == 4):
                            #
                            dritto.Avvia(finecorsa.livello+1,-0.6,finecorsa.mIndietro) # vai indietro di 12cm
                            finecorsa.stato=5
                        elif(finecorsa.stato==5):
                            lg.azione[finecorsa.livello]=lg.libero
                            finecorsa.avvio=0
                            
                    elif(lg.azione[finecorsa.livello] != finecorsa.nome):
                        print(finecorsa.nome," bloccato st:",finecorsa.stato, 'da',lg.azione[finecorsa.livello])
                        finecorsa.avvio = 0
                #FINE FINECORSA
                

                #ALLINEA
                if(allinea.avvio):
                
                    if(allinea.stato==0):
                        esp.ResetDistanza() #resetto distanza esp
                        lg.distanza=0
                        lg.RSS(allinea.livello)
                        lg.azione[allinea.livello]=allinea.nome
                        allinea.tempoTotale = 5 #imposto tempo di allineamento massimo a 5s
                        allinea.tempoInizio = time.time() #imposto tempo iniziale
                        allinea.stato=1
                        allinea.uscita = 0
                        allinea.finecorsaDx = 0
                        allinea.finecorsaSx = 0
                    elif(allinea.stato==1):
                        allinea.tempoMancante = allinea.tempoTotale - (time.time()-allinea.tempoInizio) #calcolo tempo che manca
                        lg.velDx = lg.velSx = -0.6#-0.5 #vado indietro
                        if(lg.distanza<-6):
                            allinea.fuoriCasella = 1
                        if(sn.ReadEndRun(sn.fDiDx)==1 and allinea.finecorsaDx==0):  #se è premuto il finecorsa dietro destra e non è ancora stato premuto
                            allinea.finecorsaDx = 1 #imposto che è stato premuto questo finecorso
                            allinea.tempoTotale = 1 #imposto tempo di allinamento ( 2s dal primo finecorsa che si preme)
                            allinea.tempoInizio = time.time() #reimposto tempo iniziale
                        if(sn.ReadEndRun(sn.fDiSx)==1 and allinea.finecorsaSx==0): 
                            allinea.finecorsaSx = 1
                            allinea.tempoTotale = 1
                            allinea.tempoInizio = time.time()
                        
                        if(allinea.finecorsaDx and allinea.finecorsaSx and allinea.uscita==0): #se entrabi i finecorsa sono premuti e non è ancora impostata l'uscita per finecorsa
                            lg.velDx = lg.velSx = 0 #fermo
                            
                            allinea.uscita = 1 #imposto l'uscita per finecorsa
                        elif(allinea.uscita ): # se è impostata l'uscita o è finito il tempo
                            lg.velDx = lg.velSx = 0
                            lg.avvioLaser[sn.lAvDx] = 1
                            lg.avvioLaser[sn.lDiDx] = 1
                            lg.avvioLaser[sn.lAvSx] = 1
                            lg.avvioLaser[sn.lDiSx] = 1
                            allinea.stato=2
                            
                        elif(allinea.tempoMancante<=0):
                            lg.velDx = lg.velSx = 0
                            allinea.stato=3
                    elif(allinea.stato==2):
                        if(abs(lg.laser[sn.lAvDx]-lg.laser[sn.lDiDx])<1 and abs(lg.laser[sn.lAvSx]-lg.laser[sn.lDiSx])<1):
                            print("allinea con correzione Z")
                            self.correggiZ()
                        else:
                            print("allinea senza correzione Z, causa sfalsamento laser")
                        allinea.stato=3
                    elif(allinea.stato==3):
                        esp.ResetDistanza() #resetto distanza esp
                        lg.distanza=0
                        allinea.fuoriCasella = 0
                        lg.azione[allinea.livello]=lg.libero
                        allinea.avvio=0
                            
                    elif(lg.azione[allinea.livello] != allinea.nome):
                        print(allinea.nome," bloccato st:",allinea.stato, 'da',lg.azione[allinea.livello])
                        allinea.avvio = 0
                #FINE ALLINEA
                


                #CENTRA Y
                if(centra.avvio):
                    if(centra.stato == 0):
                        lg.RSS(centra.livello)
                        lg.azione[centra.livello]=centra.nome
                        centra.stato = 1
                        centra.verso = 0
                        esp.ResetDistanza() #resetto distanza esp
                        lg.distanza=0  
                        lg.avvioLaser[sn.lAvDx]=1
                        lg.avvioLaser[sn.lDiDx]=1
                    elif(centra.stato == 1):
                        lg.avvioLaser[sn.lAvDx]=1
                        lg.avvioLaser[sn.lDiDx]=1
                        # A DESTRA 
                        if(lg.laser[sn.lAvDx]>=20 and lg.laser[sn.lDiDx]<20): #se non c'è un muro davanti e c'è dietro 
                            lg.velDx = lg.velSx = 0.8 #vai avanti
                            centra.verso = 1 # 1--> avanti
                        elif(lg.laser[sn.lAvDx]<20 and lg.laser[sn.lDiDx]>=20): #se c'è un muro davanti e non c'è dietro
                            lg.velDx = lg.velSx = -0.8
                            centra.verso = -1 #-1 --> dietro
                        elif(centra.verso!=0): # se c'è stato un movimento 
                            centra.stato = 3 
                        else: #se non c'è stato un movimento
                            centra.stato = 2
                            lg.avvioLaser[sn.lAvSx]=1
                            lg.avvioLaser[sn.lDiSx]=1
                    elif(centra.stato == 2):
                        lg.avvioLaser[sn.lAvSx]=1
                        lg.avvioLaser[sn.lDiSx]=1
                        if(lg.laser[sn.lAvSx]>=20 and lg.laser[sn.lDiSx]<20):
                            lg.velDx = lg.velSx = 0.8
                            centra.verso = 1
                        elif(lg.laser[sn.lAvSx]<20 and lg.laser[sn.lDiSx]>=20):
                            lg.velDx = lg.velSx = -0.8
                            centra.verso = -1
                        elif(centra.verso!=0):
                            centra.stato = 3
                        else:
                            centra.stato = 4
                    elif(centra.stato==3): 
                        lg.velDx = lg.velSx = 0 #mi fermo
                        dritto.Avvia(centra.livello+1,1*centra.verso,6) #vado avanti o indiero di 6 cm per copletare il movimento
                        centra.stato = 4
                    elif(centra.stato==4 and lg.azione[centra.livello+1] == lg.libero):#se ha finito l'avanti
                        centra.stato = 5
                        
                    elif(centra.stato==5):
                        lg.velDx = lg.velSx = 0
                        #lg.ChiudiFunzione(centra)
                        lg.azione[centra.livello]=lg.libero
                        centra.avvio = 0
                        
                    elif(lg.azione[centra.livello] != centra.nome):
                        print(centra.nome," bloccato st:",centra.stato,'da',lg.azione[centra.livello])
                        centra.avvio = 0

                #FINE CENTRA Y
                
                #CONTROLLOVITTIME
                
                if(controlloVittime.avvio):
                    #print("controlloVittime",controlloVittime.livello,controlloVittime.stato,"az0:",lg.azione[controlloVittime.livello],"az1",lg.azione[controlloVittime.livello+1])
                    if(controlloVittime.stato==0):
                        lg.RSS(controlloVittime.livello)
                        lg.azione[controlloVittime.livello]=controlloVittime.nome
                        pidG.avvio = 0 
                        lg.avvioLaser = [1]*lg.nLaser
                        controlloVittime.stato=3
                    elif(controlloVittime.stato==1 and lg.Libero(controlloVittime.livello+1)):
                        if((lg.laser[sn.lAvDx] > 15 or lg.laser[sn.lDiDx]>15 )
                            or mappa.map.map[round(robo[x])][round(robo[y])].colore != (lg.bianco -1)
                            or lg.okCamDx==0 or
                            mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g]))] != None):
                            
                            controlloVittime.stato=2
                        elif(lg.fTrovatoDx):
                            if(lg.trovatoDx!="N"):
                                pidG.avvio = 0 
                                vittime.Avvia(controlloVittime.livello+1,'D'+lg.trovatoDx)
                                cprint("TROVATO DX :" +str(lg.trovatoDx),'green')
                                mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g]))] = lg.trovatoDx
                            controlloVittime.stato=2
                    elif(controlloVittime.stato==2 and lg.Libero(controlloVittime.livello+1)):
                        if((lg.laser[sn.lAvSx] > 15 or lg.laser[sn.lDiSx]>15 )
                            or mappa.map.map[round(robo[x])][round(robo[y])].colore != (lg.bianco -1)
                            or lg.okCamSx==0 or
                            mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g])+2)]!= None):
                            
                            controlloVittime.stato=5
                        elif(lg.fTrovatoSx):
                            if(lg.trovatoSx!="N"):
                                pidG.avvio = 0 
                                vittime.Avvia(controlloVittime.livello+1,'S'+lg.trovatoSx)
                                cprint("TROVATO SX :" +str(lg.trovatoSx),'green')
                                mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g])+2)] = lg.trovatoSx
                            controlloVittime.stato=5
                    elif(controlloVittime.stato==3 and lg.Libero(controlloVittime.livello+1)):
                        print("pitagora dx",((((robo[x]-lg.xTemperaturaDx)**2 + (robo[y]-lg.yTemperaturaDx)**2)**(1/2))*30),abs(lg.Limita180(abs(robo[g]-lg.posTemperaturaDx))),robo[g],lg.posTemperaturaSx)
                        if(mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g]))] ==None
                            and mappa.map.map[round(robo[x])][round(robo[y])].colore == lg.bianco -1
                            and (sn.Temperatura(sn.tDx)>tMinVittime
                            or ( ((((robo[x]-lg.xTemperaturaDx)**2 + (robo[y]-lg.yTemperaturaDx)**2)**(1/2))*30)<10
                            and abs(lg.Limita180(abs(robo[g]-lg.posTemperaturaDx))) < 20))
                            and (lg.laser[sn.lAvDx] < 15 and lg.laser[sn.lDiDx] < 15) ): #or ( (time.time()-lg.timeTemperaturaDx)<5
                            
                            pidG.avvio = 0 
                            vittime.Avvia(controlloVittime.livello+1,'D'+'C')
                            cprint("TROVATO DX CALORE",'green')
                            #print( ((((robo[x]-lg.xTemperaturaDx)**2 + (robo[y]-lg.yTemperaturaDx)**2)**(1/2))*30), abs(lg.Limita180(abs(robo[g]-lg.posTemperaturaSx)))<20)
                            mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g]))] = 'C'
                        controlloVittime.stato=4
                    elif(controlloVittime.stato==4 and lg.Libero(controlloVittime.livello+1)):
                        if(mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g])+2)]==None
                            and mappa.map.map[round(robo[x])][round(robo[y])].colore == lg.bianco -1
                            and (sn.Temperatura(sn.tSx)>tMinVittime
                            or ( ((((robo[x]-lg.xTemperaturaSx)**2 + (robo[y]-lg.yTemperaturaSx)**2)**(1/2))*30)<10
                            and abs(lg.Limita180(abs(robo[g]-lg.posTemperaturaSx))) < 20))
                            and (lg.laser[sn.lAvSx] < 15 and lg.laser[sn.lDiSx] < 15)):
                            
                            pidG.avvio = 0 
                            vittime.Avvia(controlloVittime.livello+1,'S'+'C')
                            cprint("TROVATO SX CALORE",'green')
                            lg.posTemperaturaDx = robo[g]
              
                            #print( ((((robo[x]-lg.xTemperaturaSx)**2 + (robo[y]-lg.yTemperaturaSx)**2)**(1/2))*30),abs(lg.Limita180(abs(robo[g]-lg.posTemperaturaSx))) < 20)
                            mappa.map.map[round(robo[x])][round(robo[y])].vittime[mappa.limit04(mappa.sempG(robo[g])+2)] = 'C'
                        controlloVittime.stato=1
                    elif(controlloVittime.stato==5 and lg.Libero(controlloVittime.livello+1)):
                        self.putmappa(maze,0)
                        controlloVittime.avvio = 0
                        lg.azione[controlloVittime.livello]=lg.libero
                        print("fine controlloVittime")
                    elif(lg.azione[controlloVittime.livello] != controlloVittime.nome):
                        print(controlloVittime.nome," bloccato st:",controlloVittime.stato,'da',lg.azione[controlloVittime.livello])
                        controlloVittime.avvio = 0
                        #lg.azione[controlloVittime.livello+1]=lg.libero
                #FINE CONTROLLO VITTIME
            
            
                #RILEVA VITTIMA
                
                if(vittime.avvio):
                    #lg.avvioCamDx[0]=0
                    #lg.avvioCamSx[0]=0
                    lg.velSx = lg.velDx = 0
                    if(vittime.stato==0):
                        #pidG.avvio = 0 
                        if(vittime.tipo==None):
                            print("None tipo vittime")
                            vittime.stato = 6
                        else:
                            lg.RSS(vittime.livello)
                            lg.azione[vittime.livello]=vittime.nome
                            
                            if(vittime.tipo[0]== "D"):
                                vittime.posizione = 1
                            else:
                                vittime.posizione = -1
                                
                            if(vittime.tipo[1] == "C" or
                               vittime.tipo[1] == "R" or
                               vittime.tipo[1] == "Y"):
                                vittime.vKit = 1
                            elif(vittime.tipo[1] == "S"):
                                vittime.vKit = 2
                            elif(vittime.tipo[1]== "H"):
                                vittime.vKit = 3
                            elif(vittime.tipo[1]== "U" or
                               vittime.tipo[1]== "G"):
                                vittime.vKit = 0
                                
                            print("AVVIO RILASCIO KIT tipo:",vittime.tipo[1],"Kit da rils:",vittime.vKit,"posizione",vittime.posizione,"kit sul caricatore",lg.numKit)
                            vittime.thl = threading.Thread(target=self.thLed) #AVVIO LED
                            vittime.thl.daemon = True  
                            vittime.thl.start()
                            vittime.rilKit = 0 #kit rilasciati
                            vittime.rifCentr = 6.3 #riferimento servo centrale
                     
                            if(lg.numKit==None): #se non sono stati contati i kit
                                vittime.stato = 0.1 #conto kit
                            else:
                                vittime.stato = 0.5 #rilascio kit
                            if(lg.rilsciaKit==0):
                                vittime.stato = 6

                    elif(vittime.stato==0.1):
                        rk.initServo(vittime.rifCentr) #inizializzo servo
                        vittime.rotaz =rk.MotAvvio(-100) #porto il cursore tutto indietro
                        if(vittime.rotaz>20000): #se sono state fatte piu di 2000 rotazioni (circa 10 kit) è meglio caricare
                            print("CARICARE PETOLE TEMPO 30s")
                            time.sleep(30)
                        
                        rk.setServo(vittime.rifCentr) #mi assicuro che il servo sia in posizione centrale
                        vittime.rotaz = rk.MotAvvio(100) #porto tutto avanti il cursore
                        lg.numKit = ((25000-vittime.rotaz)/2000) +1 #calcolo quanti kit sono disponibili
                        print("petole iniziali:",vittime.rotaz,lg.numKit,round(lg.numKit))
                        lg.numKit = round(lg.numKit)                      
                        vittime.stato = 0.5
                        
                    elif(vittime.stato==0.5):
                        if(lg.numKit>0 and vittime.vKit>0): #se sono disponii kit
                            rk.setServo(vittime.rifCentr)
                            if(lg.numKit==1): #se ne manca solo uno faccio moviemnti ampi perchè non posso controllare il rilascio
                                vittime.osc = 4
                                vittime.amp = 2
                            else:
                                vittime.osc = 0
                                vittime.amp = 1.3
                                
                            if(vittime.posizione == 1): 
                                vittime.rif = 8.5 #associo rifeimento di destra
                            elif(vittime.posizione == -1): 
                                vittime.rif = 4   #associo rifeimento di sinistra
                            vittime.stato = 1
                        else:
                            print("KIT FINITI")
                            vittime.stato = 6
                                
                            
                    elif(vittime.stato==1):
                        rk.MotRuota(-100,300)#torno didietro di 100 rotazioni (0.5mm)
                        rk.setServo(vittime.rif)
                        vittime.tPos = time.time() #tempo di avviamento
                        vittime.nOsc = 0 #imposto a 0 il numero di oscillazioni effetuate
                        if(vittime.osc>0): #se ci sono da fare delle oscillazioni
                            vittime.stato = 2
                        else:
                            vittime.stato = 4
                            
                    elif(vittime.stato==2 and (time.time()-vittime.tPos)>0.4): #aspetto 0.4s per il posizionamento del servo
                        rk.setServo(vittime.rif + ( vittime.posizione * 0.2 * vittime.amp)) #imposto il servo un pò più distante
                        vittime.tPos = time.time()
                        vittime.stato = 3
                    elif(vittime.stato==3 and (time.time()-vittime.tPos)>0.4):
                        rk.setServo(vittime.rif - ( vittime.posizione * 0.2 * vittime.amp))
                        vittime.osc -= 1
                        vittime.tPos = time.time()
                        if(vittime.osc>0): #se ho anora oscillazioni da fare
                            vittime.stato = 2
                        else:
                            vittime.stato = 4

                    elif(vittime.stato==4 and (time.time()-vittime.tPos)>0.2):
                        rk.setServo(vittime.rifCentr) #riporto in centro il servo
                        time.sleep(0.2)
                        vittime.rotaz = rk.MotAvvio(100,20,0.5,2) #porto avanti il cursore
                        if(vittime.rotaz>1600 or lg.numKit==1): #se è stato rilasciato il kit (rotaz > 1600[8mm])
                            lg.numKit-=1 #abbasso di uno kit disponibili
                            vittime.vKit-=1
                            vittime.rilKit += 1 #auemnto numero kit kilasciati
                            #print("rilascio andaro a buon fine, rimanenti:",lg.numKit)
                            if(lg.numKit == 0): #se sono finiti i kit
                                print("KIT FINITI")
                                vittime.stato = 6
                            elif(vittime.vKit>0): #se devo ancora rilasciarne ritorno al 0.5
                                vittime.stato = 0.5
                            else:
                                vittime.stato = 6
                        else:
                            vittime.osc += 3
                            vittime.amp += 1
                            if(vittime.amp>=5 or (vittime.amp>=3 and lg.numKit<3)): #se l'aplificazione è arrivara sopra 5, il kit è bloccato
                                print("RILASCIO BLOCCATO !!!")
                                vittime.stato = 6
                                lg.numKit = -1
                                #lg.bloccatokit = 1
                            else:
                                print("RILASCIO FALLITO")
                                vittime.stato = 1
                                
                    elif(vittime.stato==6): #ho finito
                        print("STATOVITTIME",vittime.stato)
                        vittime.thl.join()
                        rk.MotStop()
                        rk.stopServo()
                        vittime.avvio = 0
                        lg.azione[vittime.livello]=lg.libero
                        
                    elif(lg.azione[vittime.livello] != vittime.nome): #sono stato bloccato
                        print(vittime.nome," bloccato st:",vittime.stato, 'da',lg.azione[vittime.livello])
                        rk.MotStop()
                        rk.stopServo()
                        vittime.avvio = 0

                
                #FINE RILEVA VITTIMA
                        
                lg.distanzaold = lg.distanza
            
        self.clean()
    

