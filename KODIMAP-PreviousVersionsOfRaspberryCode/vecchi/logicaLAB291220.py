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

#from com_espNEW import LibESP
#from com_esp import LibESP
from com_esp3_0 import LibESP
import sensoriV1 as snv1
import robot #contiene tutte le sottolibrerie dei movimenti 
from neopix import Neopix

class logica():
    def __init__ (self):

        #classe per le variabili e funzioni secondarie logica
        self.logica  = robot.LibLogica()
        
    def correggiZ(self): #correzione valore girospocio asse Z
        g_temp, _, _ = self.sn.giroscopio(1)
        self.sn.set_correzione(self.logica.Limita180(g_temp-self.logica.gradiVolutiZ),0)
        cprint("correggiZ gyro:"+str(g_temp)+"  volZ:"+str(self.logica.gradiVolutiZ))


    def PidThread(self): #ESECUZIONE DID
        lg = self.logica  #riporto in forma abbreviata le librerie che mi servono
        sn = self.sn
        pidG = self.pidG
        old = time.time()
        n_laser = 0
        l_errore = 1
        while True:
            if(lg.avvio==0):
                time.sleep(1)
            else:
                tim = time.time()
                if((tim-old)<0.05): #aspetto per eseguire il pid solo ogni 0.05 secondi
                    l_errore = 1
                    if(n_laser == 0):
                        lg.nuoveMisure = 1
                        lg.laserAvDx = sn.readLaser(sn.l_avdx)
                        if(lg.laserAvDx < 0):
                            l_errore = lg.laserAvDx
                            lg.avvio = 0
                    elif(n_laser == 2):
                        lg.laserAvSx = sn.readLaser(sn.l_avsx)
                        if(lg.laserAvSx < 0):
                            l_errore = lg.laserAvSx
                            lg.avvio = 0
                    elif(n_laser == 3):
                        lg.laserDiDx = sn.readLaser(sn.l_didx)
                        if(lg.laserDiDx < 0):
                            l_errore = lg.laserDiDx
                            lg.avvio = 0
                    elif(n_laser == 4):
                        lg.laserDiSx = sn.readLaser(sn.l_disx)
                        if(lg.laserDiSx < 0):
                            l_errore = lg.laserDiSx
                            lg.avvio = 0
                    elif(n_laser == 5):
                        lg.laserFr = sn.readLaser(sn.l_fronte)
                        if(lg.laserFr < 0):
                            l_errore = lg.laserFr
                            lg.avvio = 0

                    if(l_errore != 1):
                        print("ERRORE LASER laser:",n_laser,"errore:",l_errore)
                    n_laser += 1
                    if(n_laser>5):
                        n_laser = 0
                    #time.sleep(0.002)
                    #None
                    
                else:
                    #print("pidG:",time.time()-old)
                    old = tim # time.time() #tim
                    lg.gradiZ, lg.gradiY, lg.gradiX = sn.giroscopio(0)
                    pidG.errore = lg.Limita180(lg.gradiVolutiZ-lg.gradiZ) #calcolo errore PID giroscopio
                    if(abs(pidG.errore)<5): #erroreZ rileva se siamo esatti con una tolleranza di 5°
                        lg.erroreZ = 1
                    else:
                        lg.erroreZ = 0
                    
                    if ( pidG.avvio and (lg.velDx == lg.velSx) ): #esegue il PID giroscopio sono se è stato attivato (avvio) e se le locità di coppia solo uguali (non gira)
                        pidG.mod = 1 #setto la modalità 
                        if(pidG.mod != pidG.oldMod): #se la modialità differisce da quella vecchia resetto le variabili del PID
                            print("reset pidG:",pidG.oldMod)
                            pidG.Resettino()
                        
                        pidG.CalcolaPID() #calcolo PID
                        lg.velDxTemp = lg.velDx-pidG.correzione # lo applico alle coppie che verranno settate in un succesiivo momento nel while principale
                        lg.velSxTemp = lg.velSx+pidG.correzione
                                                    
                    else: #se non posso essereguire il PID
                        pidG.mod = 0  #setto la modalità 
                        lg.velDxTemp = lg.velDx #imposto velocità normali
                        lg.velSxTemp = lg.velSx
                        
                    pidG.oldMod = pidG.mod #ricordo l'ultima modalità
                '''
                dif = 0.05-(time.time()-old)
                if(dif>0):
                    time.sleep(dif)
                else:
                    cprint("PidG supera i 0.05s",'blue')
                '''
                
    def clean(self):  #funzione da richimare per chiudere questa libreria (logica)
        print("LKI")
        self.esp.set_mot(0)
        self.esp.clean()
        self.sn.clean()
        #self.t_pid.join()
       

        
    def main(self,q_ric_dx,q_avv_dx,q_ric_sx,q_avv_sx,q_mappa): #funzione principale in multiprocessing
        print("MAIN")
        
        led = Neopix() #creo il led
        
        led.Led(-1,(255,255,255)) #lo setto bianco durate l'init del robot
        
        self.sn = sn = snv1.sensori() # creo i sensori come oggetto comune e nella sua forma abbreviata (sn)
        
        
        mappa = robot.LibMappa() #creo la mappa
        mappa.SettaQueue(q_mappa) #setto alla mappa la queue dove scriverà la mappa per essere visualizzata da "visualizzazione"

        #creo le varie sottolibrerie che contengono le variabili di ogni movimento
        
        dritto = robot.LibDritto() 
        
        gira = robot.LibGira()
        
        finecorsa = robot.LibFinecorsa()
        
        rampa = robot.LibRampa()
        
        allinea = robot.LibAllinea()
        
        centra = robot.LibCentraY()
                
        self.pidG = pidG = robot.LibPID(0.06,0.4,0.005) #creo il pid con le relative kp ki kd
    
        lg = self.logica #creo forma abbreviata di logica
        
        self.esp = esp = LibESP() #creo la comunicazione con l'esp32
        
        #creo il thred che si occuperà del PID
        self.t_pid = threading.Thread(target=self.PidThread)
        self.t_pid.daemon = True            
            
        lg.lack= 1 #quando entrerà nel ciclo principale saprà che sarà in un ceeckpoint
        
        #inizializzo sensori
        sn.initLaser()
        sn.initBNO()

        self.t_pid.start()
        
        led.SpegniLed() # spengo led alla fine dell'init
        
        lg.avvio = 1 #imposto che il robot può muoversi

        print("####### WHILE PRINCIPALE #######")
        while(1):
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            #CONTROLLO TELECAMERA DESTRA
            if(lg.laserAvDx<13 and lg.laserDiDx<13): #se c'è il muro confermato da entrabi i muri
                lg.avvioCamDx[0]=1 #attivo la telecamera (setto lo stato per poi inviarlo)
            else:
                led.Led(1,(0,0,0))
                lg.avvioCamDx[0]=0 #disattivo la telecamera
            if(lg.avvioCamDx[0]!=lg.avvioCamDx[1]): #se lo stato è cambito 
                try:
                    q_avv_dx.put_nowait(lg.avvioCamDx[0]) #provo a inviarlo tramite la queue
                    lg.avvioCamDx[1] = lg.avvioCamDx[0]   #aggiorno lo stato precedente
                except Queue.Full:
                    None
            try:
                trovato = q_ric_dx.get_nowait() #provo a riceve dal riconoscimento 
                if(trovato!="N"): 
                    cprint("telecamera dx:"+ str(trovato),'cyan') #stampo cosa ha trovato (N niente) R G V H S U
                led.VisualizzaTrovato(1,trovato) #imposta il rispettivo led
            except Queue.Empty: #se non c'è niente di nuovo non fare niente
                None
                
            #CONTROLLO TELECAMERA SINISTRA
            if(lg.laserAvSx<13 and lg.laserDiSx<13):
                lg.avvioCamSx[0]=1
            else:
                led.Led(0,(0,0,0))
                lg.avvioCamSx[0]=0
            
            if(lg.avvioCamSx[0]!=lg.avvioCamSx[1]):
                try:
                    q_avv_sx.put_nowait(lg.avvioCamSx[0])
                    lg.avvioCamSx[1] = lg.avvioCamSx[0]
                except Queue.Full:
                    None
            try:
                trovato = q_ric_sx.get_nowait()
                if(trovato!="N"):
                    cprint(" telecamera sx:"+ str(trovato),'cyan')
                led.VisualizzaTrovato(0,trovato)
            except Queue.Empty:
                None

            #LETTURA SENSORI
            '''
            lg.laserAvDx = sn.readLaser(self.sn.l_avdx)
            lg.laserAvSx = sn.readLaser(self.sn.l_avsx)
                    
            lg.laserDiDx = sn.readLaser(self.sn.l_didx)
            lg.laserDiSx = sn.readLaser(self.sn.l_disx)

            lg.laserFr = sn.readLaser(self.sn.l_fronte)

            #print ("laser:", lg.laserAvDx,lg.laserAvSx,lg.laserDiDx,lg.laserDiSx,lg.laserFr)
            if(lg.laserAvDx==-1 or lg.laserAvSx==-1 or lg.laserDiDx==-1 or lg.laserDiSx==-1 or lg.laserFr==-1): #verifico se c'è un errore
                print("ERRORE LASER MAIN:",lg.laserAvDx,lg.laserAvSx,lg.laserDiDx, lg.laserDiSx, lg.laserFr)
                #ERRORE FATALE
                lg.avvio = 0
            '''
            # RICEVO FOTORESISTENZA E DISTANZA DA ESP
            # INVIO VELOCITA' DELLE COPPIE A ESP
            lg.luce, lg.distanza = esp.SetMot(lg.velDxTemp,lg.velSxTemp,foto=1,distanza=1)

        
            if(lg.avvio==0):
                cprint("Robot fermato da AVVIO", 'red', attrs=['bold'])
                time.sleep(5)
                lg.velDxTemp = 0
                lg.velSxTemp = 0
                lg.ultimoC = time.time()
                
            elif(lg.luce == esp.errore or lg.luce==esp.noEsp): #esp errore (spento x lack o errore fatale comunicazione)
                if(lg.contaErrori!=-1): #se sta ancora contando gli errori scrivi l'errore
                    if(lg.contaErrori == 0 or lg.contaErrori==6):
                        if(lg.luce == esp.errore): 
                            cprint("erroreESP n:" + str( lg.contaErrori), 'red')
                        if(lg.luce==esp.noEsp):
                            cprint("spentoESP n:" + str( lg.contaErrori), 'red')
                    lg.contaErrori+=1
                    
                if(lg.contaErrori>6): #se gli errori superano i 6 setta il lack 
                    lg.contaErrori = -1
                    lg.lack = 1                    
            else:
                #LOGICA 
                if(lg.lack): #se è il primo ciclo (quindi ti trovi sul ceeckpoint)
                    #RESETTO TUTTI I DATI RELATIVI ALLA LOGICA
                    lg.Resetta()
                    self.correggiZ()
                    lg.gradiZ, lg.gradiY, lg.gradiX = sn.giroscopio(0)
                    lg.gradiVolutiZ = lg.gradiZ #sistemo giroscopio
                    print("riarmo", lg.gradiVolutiZ )
                    print("setDiam:",esp.SetDiametro(68)) #se "1" il settaggio è andato a buon fine
                    dritto.Resetta()
                    gira.Resetta()
                    centra.Resetta()
                    allinea.Resetta()
                    finecorsa.Resetta()
                    
                else:
                    lg.contaErrori = 0 #se rietro e ci sono stati meno di 6 errori dall'esp li riporto a 0
                
                


                if(lg.stato != lg.oldStato or lg.azione!= lg.oldAzione): #se c'è stata una modifica nei flag princiali
                    cprint( str(lg.stato) +str(lg.azione) + " t:" + str(round(time.time()-lg.ultimoC,3)) , 'yellow')
                    lg.ultimoC = time.time() #setto tempo dell'ultimo cambio

                #SPERIMENTALE
                if((time.time()-lg.ultimoC)>10): #se i flag non cambiano da più di 10 secondi 
                    cprint("SUPERATO TEMPO DI CAMBIO DI 10 SECONDI",'magenta')
                    #dovrei dare errore e fare lack
                    
                lg.oldStato = lg.stato #aggiorno stati precedenti di controllo
                lg.oldAzione = lg.azione.copy()


                
                if(lg.stato == 0 and lg.Libero(0)): #se lo stato impostato è 0 e il programma non sta eseguendo nessun'altro programma
                    mappa.scrivi(mappa.X,mappa.Y,mappa.G) #scrivi la mappa alla visulizzazione
                    pidG.avvio = 0 #non voglio il pid giroscopio
                    lg.Resettino() #resetto variabili apposite logica (prx, pry, nodestra ecc)
                    lg.stato = 1 #aumento lo stato 
                elif(lg.stato == 1 and lg.Libero(0)): 
                    lg.ColoreCasella() #eseguo il calcolo per trovare il colore della casella (risultato in lg.casella)
                    mappa.colore[mappa.X,mappa.Y]=lg.casella #setto il colore sulla mappa
                    if(lg.casella==lg.nero): #se è nera
                        print("Casella Nera", lg.luce, lg.casella)
                        pidG.avvio = 1 #vado indietro dritto con il PID
                        dritto.Avvia(0,-1,30) #indietro a livello 0 con velocità -1g/s di 30cm
                        lg.noAvanti = 1 #setto che non può andare avanti e a destra nella decisione
                        lg.noDestra = 1
                        lg.stato = 2 #aumento lo stato
                        mappa.gr = -4 #setto i gradi di modifica della mappa (-4 = indietro)
                    else:
                        lg.stato = 2 #altrimenti aumento seplicemnte lo stato

                elif(lg.stato == 2 and lg.Libero(0)):
                    pidG.avvio = 1
                    centra.Avvia(0) #avvio il CentraY nella casella 
                    lg.stato = 3 

                elif(lg.stato == 3 and lg.Libero(0)):
                    mappa.gr = 0 
                    if(lg.nuoveMisure):
                        lg.nuoveMisure=0
                        if(lg.laserAvDx < 15 or lg.laserDiDx<15): #se almeno uno conferma il muro 
                            print("MDX",lg.laserAvDx,lg.laserDiDx)
                            mappa.muri[mappa.X,mappa.Y,mappa.modG(mappa.G+2)]=1 #setto il muro nella mappa
                            lg.pmdx = 1 # setto presenza muro a destra 
                        if(lg.laserAvSx < 15 or lg.laserDiSx<15):
                            print("MSX",lg.laserAvSx,lg.laserDiSx)
                            mappa.muri[mappa.X,mappa.Y,mappa.modG(mappa.G+0)]=1
                            lg.pmsx = 1
                        if(lg.laserFr < 15):
                            print("MFR",lg.laserFr)
                            lg.pmfr = 1
                            mappa.muri[mappa.X,mappa.Y,mappa.modG(mappa.G+1)]=1
                        if(lg.noDestra == 0 and lg.pmdx==0): #se posso andare a destra e non c'è il muro
                            print("giro Dx") 
                            gira.Avvia(0,1,90) #giro 90° in senso orario a livello 0 con una velocità di 1g/s
                            mappa.gr = 1 #setto i gradi di modifica della mappa 
                            if(lg.pmsx): #se c'è il muro a sinistra prima della rotazione
                                lg.flAllinea = 1 #poi allineati indietro con i finecorsa
                                lg.stato = 4
                            else:
                                lg.stato = 5
                        elif(lg.pmfr or lg.noAvanti or lg.noDestra): #se non posso andare avanti o davanti è bloccato (pmfr) o non ho potuto andare a destra
                            if(lg.pmsx==0): #se è libero a sinistra
                                print("giro Sx",lg.pmdx) 
                                gira.Avvia(0,1,-90) #giro di -90
                                mappa.gr = -1 
                                if(lg.pmdx): 
                                    lg.flAllinea = 1
                                    lg.stato = 4
                                else:
                                    lg.stato = 5
                            else: #altrimenti giro di 180
                                print("gira 180")
                                gira.Avvia(0,1,90) #giro di 90 alla volta per permettere il riconoscimento nel vicolo cieco
                                lg.stato = 3.5 #stato intermedio per finire i 90 che mancano
                                mappa.gr = 2
                                if(lg.pmfr):
                                    lg.flAllinea = 1
                        else:
                            lg.stato = 5
                elif(lg.stato == 3.5 and lg.Libero(0)):
                    gira.Avvia(0,1,90) #ultimo 90 del 180
                    if(lg.flAllinea):
                        lg.stato = 4
                    else:
                        lg.stato = 5
                        
                elif(lg.stato == 4 and lg.Libero(0)): #Allinea
                    pidG.avvio = 0 #non pid giroscopioo
                    allinea.Avvia(0)  #avvio l'allinea a livello 0
                    lg.flAllinea = 1 #setto che è stato sicuramente fatto l'allinea
                    lg.stato = 5
                elif(lg.stato == 5 and lg.Libero(0)):
                    #controllo Vittime
                    lg.stato = 6
                elif(lg.stato == 6 and lg.Libero(0)):
                    pidG.avvio = 1 #avanti con pid
                    if(lg.flAllinea):
                        dritto.Avvia(0,1,35) #se ho fatto l'allinea mi trovo 5cm più indietro (vado avanti di 35cm)
                    else:
                        dritto.Avvia(0,1,30) #altrimenti normali 30
                    lg.stato = 7
                elif(lg.stato == 7): #AVANTI NON BLOCCANTE
                    #controllare inclinazione
                    if(sn.inclinazione()==sn.inc_errore): #se sono in caso di errore di inclinazione (sopra o sotto i limiti)
                        cprint("errore inc",'grey')
                        if(lg.gradiY>0): #ribaltamento indietro
                            lg.velDx = lg.velSx = -1.5 
                        else: #ribaltamento in avanti
                            lg.velDx = lg.velSx = 1.5
                    elif(sn.inclinazione()!=sn.inc_piano): #se sono in salita o discesa normale 
                        cprint("salita o discesa",'grey')
                        lg.velDx = lg.velSx = 0.6 #vado più piano
                    elif(lg.laserFr<=7 and lg.azione[0]!=finecorsa.nome): #se ho un muro vicino d'avanti e non sto facndo i finecorsa
                        print("Fermo per laser FR" ,lg.laserFr)
                        lg.azione[0] = lg.libero  # fermo qualsiasi programma dichiarando il livello 0 libero
                        lg.stato = 8
                    elif( sn.ReadEndRun(sn.f_avdx) and lg.azione[0]!=finecorsa.nome): #premuto finecorsa avanti di Dx
                        pidG.avvio = 1
                        finecorsa.AvviaAvDx(0)
                    elif( sn.ReadEndRun(sn.f_avsx) and lg.azione[0]!=finecorsa.nome):
                        pidG.avvio = 1
                        finecorsa.AvviaAvSx(0)
                    elif( sn.ReadEndRun(sn.f_avfr) and lg.azione[0]!=finecorsa.nome): #premuto finecrosa avanti centrale
                        pidG.avvio = 1
                        dritto.Avvia(0,-0.8,5)  #vado indietro di 5 cm (DA RIGUARDARE)
                        lg.stato = 6
                    elif(lg.azione[0]==finecorsa.nome):
                        lg.flAllinea = 0 #imposto che non sono allineato indietro il prossimo avanti 30
                        lg.stato = 6 # ritorno allo stato 6 per ricominciare l'avanti 
                    elif(lg.laserFr<=20 and lg.Libero(0)): # continuo ad avanzare per avvicinarmi giusto al muro avanti
                        print("Continua per laser FR" ,lg.laserFr)
                        lg.velDx = lg.velSx = 0.4 #avanzo letamente per debug
                    elif(lg.Libero(0)): #se ha finito di fare qualsiasi cosa al livello 0
                        lg.stato = 8
                        print("esci per stato 8",lg.laserFr)

                elif(lg.stato ==8 and lg.Libero(0)): #Stato finale
                    #controllo vittime
                    lg.velDx = lg.velSx = 0 #fermo i motori per sicurezza
                    lg.stato = 0 #ricomincio il cilo
                    if(mappa.gr!=-4): #cacoli mappa
                        mappa.mg=mappa.modG(mappa.gr+mappa.G+1)
                        mappa.G=mappa.modG(mappa.G+mappa.gr)
                        mappa.X=mappa.modX(mappa.X,mappa.mg)
                        mappa.Y=mappa.modY(mappa.Y,mappa.mg)
                        print("norm",mappa.mg ,mappa.gr, "gxy:",mappa.G ,mappa.X,mappa.Y)
                    else:
                        mappa.mg=mappa.modG(mappa.G+3)
                        mappa.X=mappa.modX(mappa.X,mappa.mg)
                        mappa.Y=mappa.modY(mappa.Y,mappa.mg)
                        print("ne" , mappa.mg, "gxy:",mappa.G ,mappa.X,mappa.Y)
                    
                    
                
                
                #FINE LOGICA ####################################################################################################################
                    
                #DRITTO
                if(dritto.avvio): #se è stato attivato (non usare .avvio per disattivarlo)
                    if(dritto.stato==0): #se lo stato del dritto è lo 0
                        lg.azione[dritto.livello]=dritto.nome #imposto l'azione generale a livello (.livello) impostato a dritto (.nome)
                        esp.ResetDistanza() #resetto distanza esp
                        lg.distanza=0  
                        dritto.Resettino() #resetto le variabili secondarie del dritto, quelle che non sono di stato
                        if(dritto.velocita>0): #Calcolo componenti retta finale di rallentamento
                            dritto.qRetta = 0.2 #finirò con una velocità di 0.2 se sto andando in avanti (velocità>0)
                        else:
                            dritto.qRetta = -0.2
                        dritto.mRetta = (dritto.velocita-dritto.qRetta)/dritto.distanzaRetta #il coefficente angolare è dipendente dal'intercetta (q), la velocità voluta e la grandezzza della retta finale
                        dritto.stato = 1 #aumento lo stato
                    elif(dritto.stato == 1 and lg.azione[dritto.livello] == dritto.nome): #se lo stato è a 1 e l'azione appartiene ancora a dritto 
                        if(dritto.velocita<0): #calcolo distaza percorsa relativamnte alla velocità impostata (avanti o indietro)
                            dritto.distanzaPercorsa = -lg.distanza
                        else:
                            dritto.distanzaPercorsa = lg.distanza
                        dritto.distanzaMancante = dritto.distanzaVoluta-dritto.distanzaPercorsa #calcolo distanza che mi manca per finire il percorso 
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
                    elif(lg.azione[dritto.livello] != dritto.nome): #se l'azione al livello voluto non è più il dritto (ma .avvio è ancora valido)
                                                                    # vuol dire che il dritto è stato bloccato o sostituito da un'altra azione
                        dritto.errore = dritto.distanzaVoluta-dritto.distanzaPercorsa #calcolo errore
                        print(dritto.nome," bloccato e:",dritto.errore) 
                        dritto.avvio = 0 #disattivo l'avvio
                #FINE DRITTO
                        
                #GIRA 
                if(gira.avvio):
                   
                    if(gira.stato==0):
                        gira.Resettino() 
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
                    elif(gira.stato == 1 and lg.azione[gira.livello] == gira.nome): #se l'azione al livello voluto è ancora del gira
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
                            if( gira.gradiMancanti <= gira.gradiMaxRetta and gira.gradiMancanti>=gira.gradiMinRetta): #se i gradi sono dentro all'azione della retta 
                                gira.velocitaMod = (gira.gradiMancanti * 0.0224) + 0.08#0.033 0.4  #calcolo velocità da retta
                            else:
                                gira.velocitaMod = 1.2 #altrimenti velocità massima
    
                            lg.velDx = gira.velocitaMod*gira.versoGira*-1.0 #imposto le coppie 
                            lg.velSx = lg.velDx* -1
                        else:
                            lg.velDx = lg.velSx = 0 #mi fermo
                            lg.azione[gira.livello] = lg.libero #imposto che ho finito il gira
                            gira.errore = gira.gradiVoluti - gira.gradiAdesso - (gira.kGiri*360) + gira.gradiIniziali  #calcolo errore
                            print(gira.nome," e:",gira.errore)
                            gira.avvio = 0 #disabilito il gira
                    elif(lg.azione[gira.livello] != gira.nome): #se è stato bloccato (non ha finito in autonomia)
                        gira.errore = gira.gradiVoluti - gira.gradiAdesso - (gira.kGiri*360) + gira.gradiIniziali 
                        print(gira.nome," bloccato e:",gira.errore)
                        gira.avvio = 0
                        
                            
                #FINE GIRA 
            
                
                #FINECORSA
                if(finecorsa.avvio):
                    if(finecorsa.stato==0):
                        lg.azione[finecorsa.livello]=finecorsa.nome
                        if(finecorsa.posizione == 0 or finecorsa.posizione == 1): #se il finecorsa selezionato è lo 0 o 1 (possibilità di aggiungerne altri)
                            dritto.Avvia(finecorsa.livello+1,-0.6,5) #indietro di 5 cm, avvio il Dritto al livello sottostante (livello+1), altrimenti avrei sovrascitto la funzioen chiamante (fiencorsa)
                            finecorsa.stato=1
                    elif(lg.azione[finecorsa.livello+1] == lg.libero and finecorsa.stato>0 and lg.azione[finecorsa.livello]==finecorsa.nome):
                        # se lo stato non è lo 0 e l'azione al livello sottostante è vuota (libero) e l'azione al livello voluto è ancora il finecorsa
                        if(finecorsa.stato==1):
                            if(finecorsa.posizione==0): #se è il finecorsa 0 (Dx)
                                gira.Avvia(finecorsa.livello+1, 0.8,-35) #gira di -35 gradi 
                                
                                #gira -35
                            if(finecorsa.posizione==1):
                                gira.Avvia(finecorsa.livello+1,0.8, 35) #se è il finecorsa 1 (Sx)
                                #gira 35
                            finecorsa.stato=2
                        elif(finecorsa.stato==2):
                            dritto.Avvia(finecorsa.livello+1,0.6,16) # vai avati di 16cm
                            finecorsa.stato=3
                        elif(finecorsa.stato==3):
                            if(finecorsa.posizione==0):
                                gira.Avvia(finecorsa.livello+1,0.8,35) #rimettersi dritto, gira di -35 gradi 
                            else:
                                gira.Avvia(finecorsa.livello+1,0.8,-35) #rimettersi dritto, gira di -35 gradi 
                            finecorsa.stato = 4
                        elif(finecorsa.stato == 4):
                            dritto.Avvia(finecorsa.livello+1,-0.6,12) # vai indietro di 12cm
                            finecorsa.stato=5
                        elif(finecorsa.stato==5):
                            lg.azione[finecorsa.livello]=lg.libero
                            finecorsa.avvio=0
                            
                    elif(lg.azione[finecorsa.livello] != finecorsa.nome):
                        print(finecorsa.nome," bloccato st:",finecorsa.stato)
                        finecorsa.avvio = 0
                #FINE FINECORSA
                

                #ALLINEA
                if(allinea.avvio):
                
                    if(allinea.stato==0):
                        lg.azione[allinea.livello]=allinea.nome
                        allinea.tempoTotale = 5 #imposto tempo di allineamento massimo a 5s
                        allinea.tempoInizio = time.time() #imposto tempo iniziale
                        allinea.stato=1
                    elif(allinea.stato==1):
                        allinea.tempoMancante = allinea.tempoTotale - (time.time()-allinea.tempoInizio) #calcolo tempo che manca
                        lg.velDx = lg.velSx = -0.5 #vado indietro
                        if(sn.ReadEndRun(sn.f_didx)==1 and allinea.finecorsaDx==0):  #se è premuto il finecorsa dietro destra e non è ancora stato premuto
                            allinea.finecorsaDx = 1 #imposto che è stato premuto questo finecorso
                            allinea.tempoTotale = 2 #imposto tempo di allinamento ( 2s dal primo finecorsa che si preme)
                            allinea.tempoInizio = time.time() #reimposto tempo iniziale
                        if(sn.ReadEndRun(sn.f_disx)==1 and allinea.finecorsaSx==0): 
                            allinea.finecorsaSx = 1
                            allinea.tempoTotale = 2
                            allinea.tempoInizio = time.time()
                            
                        if(allinea.finecorsaDx and allinea.finecorsaSx and allinea.uscita==0): #se entrabi i finecorsa sono premuti e non è ancora impostata l'uscita per finecorsa
                            lg.velDx = lg.velSx = 0 #fermo
                            allinea.uscita = 1 #imposto l'uscita per finecorsa
                        elif(allinea.uscita or allinea.tempoMancante<=0 ): # se è impostata l'uscita o è finito il tempo
                            lg.velDx = lg.velSx = 0
                            self.correggiZ() #correggi valore giroscopio
                            lg.azione[allinea.livello]=lg.libero
                            allinea.avvio=0
                            
                    elif(lg.azione[allinea.livello] != allinea.nome):
                        print(allinea.nome," bloccato st:",allinea.stato)
                        allinea.avvio = 0
                #FINE ALLINEA
                


                #CENTRA Y
                if(centra.avvio):
                    if(centra.stato == 0):
                        lg.azione[centra.livello]=centra.nome
                        centra.stato = 1
                        centra.verso = 0
                    elif(centra.stato == 1):
                        # A DESTRA 
                        if(lg.laserAvDx>=20 and lg.laserDiDx<20): #se non c'è un muro davanti e c'è dietro 
                            lg.velDx = lg.velSx = 0.8 #vai avanti
                            centra.verso = 1 # 1--> avanti
                        elif(lg.laserAvDx<20 and lg.laserDiDx>=20): #se c'è un muro davanti e non c'è dietro
                            lg.velDx = lg.velSx = -0.8
                            centra.verso = -1 #-1 --> dietro
                        elif(centra.verso!=0): # se c'è stato un movimento 
                            centra.stato = 3 
                        else: #se non c'è stato un movimento
                            centra.stato = 2
                    elif(centra.stato == 2):
                        if(lg.laserAvSx>=20 and lg.laserDiSx<20):
                            lg.velDx = lg.velSx = 0.8
                            centra.verso = 1
                        elif(lg.laserAvSx<20 and lg.laserDiSx>=20):
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
                        lg.azione[centra.livello]=lg.libero
                        centra.avvio = 0
                        
                    elif(lg.azione[centra.livello] != centra.nome):
                        print(centra.nome," bloccato st:",centra.stato)
                        centra.avvio = 0

                #FINE CENTRA Y
                '''

                #RILEVA VITTIMA
                
                if(vittime.avvio):
                    if(vittime.stato==0):
                        if(sn.temperatura_dx()>35):
                            vittime.posizione = 0
                            vittime.stato = 1
                            #self.map_vittime[X,Y,0]=1
                            #self.map_vittime[X,Y,1]= self.modG(G+1)
                        if(sn.temperatura_sx()>35):
                            vittime.posizione = 1
                            vittime.stato = 1
                            #self.map_vittime[X,Y,0]=1
                            #self.map_vittime[X,Y,1]= self.modG(G-1)
                        if(vittime.stato == 1):
                            vittime.tempoInizio = time.time()
                            lg.azione[vittime.livello]=vittime.nome
                            if(vittime.posizione==0):
                                vittime.led = 2
                            else:
                                vittime.led = 0
                            vittime.statoLed =0
                            vittime.contLed=0
                            #vittime_led_st=0 che minchia di variabili sono?
                            #vittime_led_scr=0
                    elif(vittime.stato==1):
                        lg.velDx= lg.velSx= 0
                        
                        if(#vittime_led_scr==0):
                            #vittime_led_scr=1
                            if(#vittime_led_st==0):
                                self.LED(vittime_led,(255,255,255))
                            else:
                                self.LED(vittime_led,(0,0,255))
                             

                        if((time.time()-vittime.tempoInizio)>=0.5):
                            #vittime_led_scr=0
                            #vittime_led_st = not(vittime_led_st)
                            vittime.contLed +=1
                            vittime.tempoInizio = time.time()

                        if(vittime.contLed>6):
                            lg.azione[vittime.stato] = lg.libero
                            vittime.avvio = 0
                            self.LED(vittime_led,(0,0,0))

                '''
                #FINE RILEVA VITTIMA
                        

            
        
    
