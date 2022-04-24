#POGETTA MATTEO - BERLATO MATTEO
#SALBEGHI 2019-2021

#Programma per avviare tutti i processi del robot
#Avviare da terminale:
'''
. start
cd
cd Desktop/KODI
sudo -E env PATH=${PATH} python3 avvio.py
'''

import multiprocessing
import threading
import queue as Queue

import shutil
import os
import sys

DPRIMA = "/home/pi/Desktop/KODIM/FOTO"
DSECONDA =  "/home/pi/Desktop/KODIM/FOTOOLD"
if __name__ == "__main__":

    if(len(sys.argv)!= 4):
        print("devi passare tre parametri!! riconoscimento telecamere - rilascio kit - temperaturaMIN")
        exit()
    
    _, AttRiconoscimento, AttRilascio, tMinVitt = sys.argv
    import logicaLAL as MainLogic #importo tutti i file del robot necessari
    import riconoscimento6 as VisualRecognition #telecamere
    import disegnamappa2 as DrawingMaze
    #import visualizzazione #visualizzazione mappa
    #import usbcam # lettura telecamere
   
    _exit = multiprocessing.Event() #neccesaria per il multiprocessing
        
    listafoto = os.listdir( DPRIMA)
    numfoto = len(listafoto)
    if(numfoto>0):
        for ind in range(numfoto):
            nom = DPRIMA + '/' + listafoto[ind]
            shutil.move(nom,DSECONDA)
        
        
        
        
        
    dxframe=320 #dimensione voluta per il frame
    dyframe=240

    indiceDx = 2# 2 indice usb della telecamera di dx
    indiceSx = 0 #scambiare nel caso fossero invertite
    
    ric_dx = riconoscimento.ric(dxframe,dyframe,indiceDx, 'Dx') #inizializzo la libreria del riconoscimento, una per telecamera
    ric_sx = riconoscimento.ric(dxframe,dyframe,indiceSx, 'Sx') #come paramentri passo e dimensioni del frame, l'indice della telec e il nome
        
    q_ric_dx = multiprocessing.Queue(5) #queue per il riconoscimento
    q_ric_sx = multiprocessing.Queue(5) #invio trovati da riconoscimento a logica 
    q_avv_dx = multiprocessing.Queue(5) #queue attivazione telecamere
    q_avv_sx = multiprocessing.Queue(5) #invio comando da logia a riconoscimento
    
    q_mappa = multiprocessing.Queue(20)  #queue per il trasferimento della mappa da logica a visualizzazione     
    
    p_ric_dx = multiprocessing.Process(target=ric_dx.main, args=(q_ric_dx,q_avv_dx,)) #multiprocesso delle telecamere, la funzione da eseguire è il main in riconoscimento
    p_ric_sx = multiprocessing.Process(target=ric_sx.main, args=(q_ric_sx,q_avv_sx,)) # e come argomenti le queue di trasferimento

    dismappa = disegnamappa.disegna()
    p_visualizzazione = multiprocessing.Process(target=dismappa.Visualizza, args=(q_mappa,)) #multiprocesso della visualizzazione della mappatura, la funzione da eseguire è

    #p_visualizzazione = multiprocessing.Process(target=visualizzazione.main, args=(q_mappa,)) #multiprocesso della visualizzazione della mappatura, la funzione da eseguire è

    logi=logica.logica() #inizializzazione logica
    # main in visualizzazione e come argomento la queue della mappa
    try:
        #inizio dei processo
        p_visualizzazione.start()
        p_ric_dx.start()
        p_ric_sx.start()
        
        
        logi.main(q_ric_dx,q_avv_dx,q_ric_sx,q_avv_sx,q_mappa,int(AttRiconoscimento), int(AttRilascio), int(tMinVitt)) #richiama funzione bloccante (while True) in logica, come argomenti tutte le queue necessarie

        ric_dx.chiudi()
        ric_sx.chiudi()
        dismappa.chiudi()
        p_ric_dx.join()
        p_ric_sx.join()
        p_visualizzazione.join()
        #logi.clean()
        
        print("FINE")


    except KeyboardInterrupt: # termine dei processi in caso di interruzione da tastiera
        #p_ric_dx.join()
        #p_ric_sx.join()
        ric_dx.chiudi()
        ric_sx.chiudi()
        dismappa.chiudi()
        p_ric_dx.join()
        p_ric_sx.join()
        p_visualizzazione.join()
        #p_ric_dx.terminate()
        #p_ric_sx.terminate()
        #p_visualizzazione.terminate()
        
        logi.clean()
        
        print("FINE da KeyboardInterrupt")
