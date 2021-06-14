import cv2 
import numpy as np
import time
from termcolor import colored, cprint
import threading
import queue as Queue
import multiprocessing
import random

def nothing(x):
        pass 


class ric:
    def __init__(self,dx, dy , pos, name):
        self.name = "cam:"+name
        self.pos = pos
        self.cap = cv2.VideoCapture(pos)
        self.setcap(3,dx)
        self.setcap(4,dy)
        
        #cv2.namedWindow('setting')
        #cv2.createTrackbar('A', 'setting',1,100,nothing)
        #cv2.setTrackbarPos('A','setting', 11)
        print("BRI",self.getcap(cv2.CAP_PROP_BRIGHTNESS))
        print("CONT",self.getcap(cv2.CAP_PROP_CONTRAST))
        print("SAT",self.getcap(cv2.CAP_PROP_SATURATION))
        print("HUE",self.getcap(cv2.CAP_PROP_HUE))
        print("GAIN",self.getcap(cv2.CAP_PROP_GAIN))
        print("EXP",self.getcap(cv2.CAP_PROP_EXPOSURE))
        print("--------------")
        
        self.dxframe = dx
        self.dyframe = dy
            
        self.frame = self.framenero = np.zeros((self.dyframe, self.dxframe,3),np.uint8)
                
        self.low_red= (0,160,120)  #0,130
        self.up_red=  (15,255,230)

        self.low_red1= (170,160,120)  #0,130
        self.up_red1=  (180,255,255)

        self.low_green=(40 ,90,35)
        self.up_green= (100,240,200)

        self.low_yellow=(15 ,45,145) #10,100,150
        self.up_yellow=(50,255,255)

        self.low_black=(0 ,0,0)
        self.up_black=(255,255,100)
        
        self.low_white=(0 ,0,210)
        self.up_white=(255,50,255)
        
        self.DIM=(320, 240)
        self.K=np.array([[130.84597074945754, 0.0, 158.87725690869175], [0.0, 174.01572305427413, 125.79931582980298], [0.0, 0.0, 1.0]])
        self.D=np.array([[-0.032875249750133714], [-0.024042032778969283], [0.014076396079331421], [-5.659071911557526e-05]])
        balance=0.6
        dim1 = (320,240)
        dim2=None
        dim3=None
        if not dim2:
            dim2 = dim1
        if not dim3:
            dim3 = dim1
        scaled_K = self.K * dim1[0] / self.DIM[0] 
        scaled_K[2][2] = 1.0
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, self.D, dim2, np.eye(3), balance=balance)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, self.D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
        print("FINE INIZIALIZZAZIONE RICONOSCIMENTO",self.name)
       
    def setcap(self, prop, val):
        self.cap.set(prop, val)
      
    def getcap(self, prop):
        return self.cap.get(prop)
    
    def undistort(self, img):
        undistorted_img = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return undistorted_img
        
    def definisci(self,temp, in1, in2):
        temp = cv2.erode(temp,None,iterations=in1)
        temp = cv2.dilate(temp,None,iterations=in2)
        return temp
    
    def rotateImage(self,image, angle):
        (h, w) = image.shape[:2]
        center = (w / 2, h / 2)
        rot_mat = cv2.getRotationMatrix2D(center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, (h,w))
        return result
    
    def overlay(self,front_image, back_image, position=(0, 0)):
        x_offset, y_offset = position
        y_of_im, x_of_im, a = front_image.shape
        y_of_ba, x_of_ba, a = back_image.shape
        #print(y_of_im, x_of_im)
        dimDY = int((y_of_ba-y_of_im)/2)
        dimDX = int((x_of_ba-x_of_im)/2)
        back_image[y_offset+dimDY:y_offset + y_of_im+dimDY, x_offset+dimDX:x_offset + x_of_im+dimDX] = front_image
        return back_image
   
    def main(self,qRiconosciemnto,qAvvio):
        self.attivo = 1
        self.avvioRiconoscimento = 0
        print("start camera")
        startTime = time.time()
        frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        frame_sg = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        cErrori = 0
        clahefilter = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(16,16))
        fps = 30
        try:
            while(self.attivo):
                
                #A = cv2.getTrackbarPos('A','setting')
                #if(A%2==0):
                #    A+=1
                    
                try:
                    self.avvioRiconoscimento = qAvvio.get_nowait()
                    print(self.name," ricevuto:",self.avvioRiconoscimento)
                except Queue.Empty:
                    pass
                
                if(self.avvioRiconoscimento==0):
                    fps=30
                    
                
                if(fps<10): #26
                    self.cap.grab()
                    #print("seconda lettura")
                ret, framein = self.cap.read()
                if(ret!=True):
                    cErrori+=1
                    print("Errore lettura telecamera",self.name)
                    try:
                        self.cap.open(self.pos)
                    except:
                        self.cap = cv2.VideoCapture(self.pos)
                    time.sleep(3)
                    if(cErrori>20):
                        break
                else:
                    cErrori = 0
                    self.frame = self.undistort(framein)
                    self.frame = self.frame[60:210,:]    
                    self.dyframe, self.dxframe, a = self.frame.shape
                    
                    if(self.avvioRiconoscimento == 0):
                        time.sleep(0.03)
                        fps = 30
                        #print("no ric")
                    else:
                        
                        
                        frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                        frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                        
                        blurredframe = cv2.GaussianBlur(frame_gray, (7, 7), 0)
                        mask_black=cv2.adaptiveThreshold(blurredframe, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 19, 19) #11 11
                        mask_black=self.definisci(mask_black,0,1)
                        
                        cv2.imshow("mask_black  "+self.name,mask_black)
                        
                        
                        black = self.trova_lettere(mask_black)
                        LH = black[0]
                        LS = black[1]
                        LU = black[2]
                        #cv2.imshow(self.name + " 2",self.frame)
                        trovato = None
                        if(LH != 0):
                            trovato = 'H'
                        elif(LS != 0):
                            trovato = 'S'
                        elif(LU != 0):
                            trovato = 'U'
                        else:
                            mask_red0=cv2.inRange(frame_hsv, self.low_red, self.up_red)
                            mask_red1=cv2.inRange(frame_hsv, self.low_red1, self.up_red1)
                            mask_red = cv2.bitwise_or(mask_red0,mask_red1)
                            mask_red=self.definisci(mask_red,1,4)
                            red = self.trova_colori(mask_red,(0,0,255))
                            if(red != 0):
                                trovato = 'R'
                            else:
                                mask_green=cv2.inRange(frame_hsv, self.low_green, self.up_green)
                                mask_green=self.definisci(mask_green,1,4)
                                green = self.trova_colori(mask_green,(0,255,0))
                                if(green != 0):
                                    trovato = 'G'
                                else:
                                    mask_yellow=cv2.inRange(frame_hsv, self.low_yellow, self.up_yellow)
                                    mask_yellow=self.definisci(mask_yellow,1,4)
                                    yellow = self.trova_colori(mask_yellow,(0,255,255))
                                    if(yellow != 0):
                                        trovato = 'Y'
                                    else:
                                        trovato = 'N'
                                    
                        try:
                            qRiconosciemnto.put_nowait(trovato)
                        except Queue.Full:
                            try:
                                qRiconosciemnto.get_nowait()
                            except Queue.Empty:
                                None
                            try:
                                qRiconosciemnto.put_nowait(trovato)
                            except Queue.Full:
                                None
    
                    endTime = time.time()
                    deltaTime = endTime-startTime
                    startTime = endTime
                    fps = round(1/deltaTime,1)
                    cv2.putText(self.frame, str(fps), (20, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2, cv2.LINE_AA)
                    #frameout = cv2.cv2.resize(self.frame,(self.dxframe*3,self.dyframe*3),interpolation = cv2.INTER_AREA)
                    #cv2.imshow(self.name,frameout)
                    cv2.imshow(self.name,self.frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except KeyboardInterrupt:
            None
            self.chiudi()

    def chiudi(self):
        self.cap.release()
        self.attivo = 0
        print("chiuso riconosciemento camera",self.name)


    def trova_colori(self,mask,colore):
        risultato=0
        contorni, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for  contorno in contorni:
            listaX=[]  #lista delle x dei vertici
            listaY=[] #lista delle y dei vertici 
            lati=len(contorno) #numero di bordi
            for indice in range(lati):
                x,y=contorno[indice].ravel() 
                listaX.append(x) #aggiungo alle liste le coordinate
                listaY.append(y)
            minX = min(listaX) #prendo valori salienti
            maxX = max(listaX)
            minY = min(listaY)
            maxY = max(listaY)
            recta=np.array([[minX,minY],[minX,maxY],[maxX,maxY],[maxX,minY]])
            cB, cG, cR = colore
            cv2.drawContours(self.frame, [contorno], 0, (cB*0.6,cG*0.6,cR*0.6), 2)
            dx=abs(maxX-minX)
            dy=abs(maxY-minY)
            if(dx!=0 and dy!=0):
                if(dx<dy):
                    dRap=dx/dy
                else:
                    dRap=dy/dx
                percentuale = int(100 * dRap)
                perX= int(100*dx/self.dxframe)
                perY= int(100*dy/self.dyframe)
                verifica = minX>1 and maxY<(self.dyframe-10) #4 20
                if(verifica == False):
                    cv2.fillPoly(self.frame, pts = [contorno], color = (20,20,20))
                if(percentuale>40 and perX> 5 and perX<35 and perY>8 and perY<60 and verifica): #40 10 90 10 90
                    cv2.drawContours (self.frame, [contorno], 0 , colore, 3)
                    risultato+=1
                    break
        return risultato

    def limita(self, x , min_, max_):
        if(x>max_):
            x=max_
        elif(x<min_):
            x=min_
        return x

    def trova_lettere(self, imput):
        
        contorni, _ = cv2.findContours(imput, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        risultato = [0,0,0]
        for contorno in contorni:
            cApprossimato= cv2.approxPolyDP(contorno, 0.008*cv2.arcLength(contorno, True), True)
            cB, cG, cR = (255,255,0)
            #cv2.drawContours(self.frame, [contorno], 0, (cB*0.6,cG*0.6,cR*0.6), 2)
            cv2.drawContours(self.frame, [cApprossimato], 0, (cB*0.9,cG*0.9,cR*0.9),1)
            lati=len(cApprossimato)
            if(lati>5):
                listaX=[]  #lista delle x dei vertici
                listaY=[] #lista delle y dei vertici
                for indice in range(lati):
                    x,y=cApprossimato[indice].ravel() 
                    listaX.append(x) #aggiungo alle liste le coordinate
                    listaY.append(y)
                minX = min(listaX) #prendo valori salienti
                maxX = max(listaX)
                minY = min(listaY)
                maxY = max(listaY)
                recta=np.array([[minX,minY],[minX,maxY],[maxX,maxY],[maxX,minY]])
                dx=abs(maxX-minX)
                dy=abs(maxY-minY)
                if(dx!=0 and dy!=0):
                    if(dx<dy):
                        dRap=dx/dy
                    else:
                        dRap=dy/dx
                    percentuale = int(100 * dRap) #percentuale dimensioni rettangolo
                    perX= int(100*dx/self.dxframe) #perc rettangolo ripetto a immagine
                    perY= int(100*dy/self.dyframe)
                    centroX = minX+int(dx/2) #coordinate centro teorico
                    centroY = minY+int(dy/2)
                    verifica = minY>1 and maxY<(self.dyframe-10) #4 20
                    if(verifica == False):
                        cv2.fillPoly(self.frame, pts = [contorno], color = (20,20,20))
                    #print(percentuale,perX,perY)
                    
                    if(percentuale>40 and perX> 2 and perX<40 and perY>17 and perY<60 and verifica): #conrolli 40 10  95 10 95
                        distanza=[]
                        for index in range(lati):
                            index2=index-1
                            if(index2<0):
                                index2=lati-1
                            x,y=cApprossimato[index].ravel()
                            x1,y1=cApprossimato[index2].ravel()
                            distanza.append(pow( pow(x-x1 , 2)+pow(y-y1, 2) ,1/2))
                            
                        lMaxP = [[0,0],[0,0]] #[[x1,y1],[x2,y2]] lato massimo primo
                        lMaxS = [[0,0],[0,0]] #[[x1,y1],[x2,y2]] mlato massimo secondo
                        #calcolo lato con distanza massima 
                        distanzaMax=max(distanza) #dis massima registrata
                        posizione= distanza.index(distanzaMax)  #posizione nella lista
                        distanza[posizione]=-1 #cancello distanza 
                        lMaxP[0][0], lMaxP[0][1] = cApprossimato[posizione].ravel() #coordinate primo punto seg1 xpd0, ypd0
                        precedente=posizione-1 #calcolo posizione punto precendente
                        if(precedente==-1):
                            precedente=lati-1
                        lMaxP[1][0], lMaxP[1][1] = cApprossimato[precedente].ravel() #coordinate secondo punto seg1 xpd1, ypd1

                        #calcolo secondo lato con distanza massima 
                        distanzaMax=max(distanza) #dis massima registrata
                        posizione= distanza.index(distanzaMax)  #posizione nella lista
                        distanza[posizione]=-1 #cancello distanza 
                        lMaxS[0][0], lMaxS[0][1] = cApprossimato[posizione].ravel() #coordinate primo punto seg1 xpd0, ypd0
                        precedente=posizione-1 #calcolo posizione punto precendente
                        if(precedente==-1):
                            precedente=lati-1
                        lMaxS[1][0], lMaxS[1][1] = cApprossimato[precedente].ravel() #coordinate secondo punto seg1 xpd1, ypd1

                        percMediaOrtLatiMax = 100*((abs(lMaxP[1][1]-lMaxP[0][1])+abs(lMaxS[1][1]-lMaxS[0][1]))/2)
                        
                        #orizzontale centrale
                        lOrr = [[[0, 0], [0, 0]], [[0, 0], [0, 0]], [[0, 0], [0, 0]]]
                        #         x0 y0   x1 y1  ....
                        #           P0      P1       P0      P1  ....
                        #           Alta               Centrale          Bassa
                        lOrr[1][0][0] = self.limita(int(minX-0.12*dx), 0, self.dxframe) 
                        lOrr[1][1][0] = self.limita(int(maxX+0.12*dx), 0, self.dxframe)
                        lOrr[1][0][1] = lOrr[1][1][1] = self.limita(int(dy/2 +minY), 0, self.dyframe) #y comune alle linee orrizonatli
                        
                        #orizzontale su
                        lOrr[0][0][0] = self.limita(int(minX-0.18*dx), 0, self.dxframe) 
                        lOrr[0][1][0] = self.limita(int(maxX+0.18*dx), 0, self.dxframe)
                        lOrr[0][0][1] = lOrr[0][1][1]  = self.limita(int(dy/4 +minY), 0, self.dyframe) #y comune alle linee orrizonatli
                        
                        #orizzontale giù
                        lOrr[2][0][0] = self.limita(int(minX-0.18*dx), 0, self.dxframe) 
                        lOrr[2][1][0] = self.limita(int(maxX+0.18*dx), 0, self.dxframe)
                        lOrr[2][0][1] = lOrr[2][1][1] = self.limita(int((dy/4)*3 +minY), 0, self.dyframe) #y comune alle linee orrizonatli
                        
                        #verticale
                        lVer= [[0,0],[0,0]] #[[x1,y1],[x2,y2]]
                        incrementoX = ( centroX - (self.dxframe/2))*((perX *1.7)/(self.dxframe/2))
                        lVer[0][0]  = self.limita(int(dx/2 + minX + incrementoX), 0, self.dxframe) #x comune alle linee verticali
                        lVer[1][0]  = self.limita(int(dx/2 + minX - incrementoX ), 0, self.dxframe) #x comune alle linee verticali
                        lVer[0][1] = self.limita(int(minY-0.1*dy), 0, self.dyframe)
                        lVer[1][1] = self.limita(int(maxY+0.1*dy), 0, self.dyframe)
                                               
                        cv2.line(self.frame,tuple(lMaxP[0]),  tuple(lMaxP[1]),    (0,0,255),2) #due lati più lunghi 
                        cv2.line(self.frame,tuple(lMaxS[0]),  tuple(lMaxS[1]),    (0,0,255),2)                    
                        cv2.line(self.frame,tuple(lVer[0]),   tuple(lVer[1]),     (255,255,255),1) #line trasversali
                        cv2.line(self.frame,tuple(lOrr[0][0]),tuple(lOrr[0][1]),   (255,255,255),1)
                        cv2.line(self.frame,tuple(lOrr[1][0]),tuple(lOrr[1][1]),(255,255,255),1)
                        cv2.line(self.frame,tuple(lOrr[2][0]),tuple(lOrr[2][1]),(255,255,255),1)
                        
                        
                        if((lVer[0][1]-lVer[1][1])==0):
                            lVer[0][1] = lVer[1][1]+1
                            
                        coefficenteV = (lVer[0][0]-lVer[1][0])/(lVer[0][1]-lVer[1][1])
                        intercettaV = ((-(lVer[0][0]-lVer[1][0])*lVer[0][1]+(lVer[0][1]-lVer[1][1])*lVer[0][0]))/(lVer[0][1]-lVer[1][1])
                        
                        pTaOrr=[[[0, 0], [0, 0]], [[0, 0], [0, 0]], [[0, 0], [0, 0]]] #punti talio orrizzontali
                        nPuOrr=[[0, 0, 0], [0, 0, 0], [0, 0, 0]] #numero punti orrizzontali 
                        
                        pTaOrr[1][0][0] = int((lOrr[1][1][0]-lOrr[1][0][0])*0.48 +lOrr[1][0][0]) 
                        pTaOrr[1][1][0] = int((lOrr[1][1][0]-lOrr[1][0][0])*0.52 +lOrr[1][0][0])
                        pTaOrr[1][0][1] = lOrr[1][0][1]
                        pTaOrr[1][1][1] = lOrr[1][0][1]
                        
                        intercettaTaglioA = pTaOrr[1][0][0] - coefficenteV * pTaOrr[1][0][1]
                        intercettaTaglioB = pTaOrr[1][1][0] - coefficenteV * pTaOrr[1][1][1]
                        
                        pTaOrr[0][0][0] = int(coefficenteV * lOrr[0][0][1] +intercettaTaglioA)
                        pTaOrr[0][1][0] = int(coefficenteV * lOrr[0][0][1] +intercettaTaglioB)
                        pTaOrr[0][0][1] = lOrr[0][0][1]
                        pTaOrr[0][1][1] = lOrr[0][0][1]
                        
                        pTaOrr[2][0][0] = int(coefficenteV * lOrr[2][0][1] +intercettaTaglioA)
                        pTaOrr[2][1][0] = int(coefficenteV * lOrr[2][0][1] +intercettaTaglioB)
                        pTaOrr[2][0][1] = lOrr[2][0][1]
                        pTaOrr[2][1][1] = lOrr[2][0][1]
                        
                        cv2.circle(self.frame,tuple(pTaOrr[1][0]),3,(0,255,0),-1)
                        cv2.circle(self.frame,tuple(pTaOrr[1][1]),3,(0,0,255),-1)
                        
                        for nlinea in range(3):
                            old_pix=0
                            for indiceX in range(lOrr[nlinea][0][0],lOrr[nlinea][1][0]):
                                pix = imput[lOrr[nlinea][0][1],indiceX]
                                if((old_pix!=pix and old_pix==0) or (old_pix!=pix and old_pix!=0)):
                                    
                                    if(indiceX< pTaOrr[nlinea][0][0]):
                                        nPuOrr[nlinea][0]+=1
                                        cv2.circle(self.frame,(indiceX,lOrr[nlinea][0][1]),3,(0,140,50),-1)
                                    elif(indiceX< pTaOrr[nlinea][1][0]):
                                        nPuOrr[nlinea][1]+=1
                                        cv2.circle(self.frame,(indiceX,lOrr[nlinea][0][1]),3,(170,255,0),-1)
                                    else:
                                        nPuOrr[nlinea][2]+=1
                                        cv2.circle(self.frame,(indiceX,lOrr[nlinea][0][1]),3,(0,255,50),-1)
                                old_pix=pix
                            
                        #verticale
                                
                        pTaVer=[[0, 0], [0, 0]] #punti talio orrizzontali
                        nPuVer=[0, 0, 0] #numero punti orrizzontali
                        
                        pTaVer[0][1]=int((lVer[1][1]-lVer[0][1])*0.333 +lVer[0][1])
                        pTaVer[1][1]=int((lVer[1][1]-lVer[0][1])*0.666 +lVer[0][1])
                        pTaVer[0][0]=centroX
                        pTaVer[1][0]=centroX
                        
                        for indicey in range(lVer[0][1],lVer[1][1]):
                            indicex = self.limita(int(coefficenteV * indicey + intercettaV), 0, self.dxframe-1) 
                            pix = imput[indicey,indicex]
                            if((old_pix!=pix and old_pix==0)or (old_pix!=pix and old_pix!=0)):
                                
                                if(indicey<pTaVer[0][1]):
                                    nPuVer[0]+=1
                                    cv2.circle(self.frame,(indicex,indicey),3,(130,0,120),-1)
                                elif(indicey<pTaVer[1][1]):
                                    nPuVer[1]+=1
                                    cv2.circle(self.frame,(indicex,indicey),3,(200,0,200),-1)
                                else:
                                    nPuVer[2]+=1
                                    cv2.circle(self.frame,(indicex,indicey),3,(240,0,255),-1)
                            old_pix=pix
                            
                        lettera=None
                        offset = 20
                        rectb=np.array([[minX-offset,minY-offset],[minX-offset,maxY+offset],[maxX+offset,maxY+offset],[maxX+offset,minY-offset]])
                        if(percMediaOrtLatiMax>75
                            and nPuVer[0] == 0 and (nPuVer[1]+nPuVer[2])==2 and nPuVer[1]>=1 
                            and nPuOrr[0] == [2,0,2]
                            and (nPuOrr[1][0]+nPuOrr[1][1]+nPuOrr[1][2])>=2
                            and ((nPuOrr[2][0] + nPuOrr[2][1]) == 2 or nPuOrr[2][0]==2) and ((nPuOrr[2][2] + nPuOrr[2][1]) == 2 or nPuOrr[2][2]==2)):  #H
                            risultato[0]+=1
                            cv2.drawContours(self.frame, [rectb], 0, (255,100,100), 2 )
                            #print("HHH")
                            #print("perc:",percentulae,"   perx:",perx ,"   pery:",pery)
                            break
                        elif (nPuVer[0] == 2 and (nPuVer[1]+nPuVer[2])==4 and nPuVer[1]>=1 
                            and nPuOrr[0][0] == 2 and nPuOrr[0][1] ==0
                            and (nPuOrr[1][0]+nPuOrr[1][1]+nPuOrr[1][2])==2
                            and nPuOrr[2][1] ==0 and nPuOrr[2][2] == 2):  #S
                            risultato[1]+=1
                            #print("SSS")
                            cv2.drawContours(self.frame, [rectb], 0, (255,255,100), 2)
                            #print("perc:",percentulae,"   perx:",perx ,"   pery:",pery)
                            break
                        elif (percMediaOrtLatiMax>50
                            and nPuVer == [0,0,2]
                            and nPuOrr[0] == [2,0,2]
                            and nPuOrr[1] == [2,0,2]
                            and ((nPuOrr[2][0] + nPuOrr[2][1]) == 2 or nPuOrr[2][0]==2) and ((nPuOrr[2][2] + nPuOrr[2][1]) == 2 or nPuOrr[2][2]==2)):#U
                            risultato[2]+=1
                            #print("UUU")
                            cv2.drawContours(self.frame, [rectb], 0, (255,100,255), 2)
                            #print("perc:",percentulae,"   perx:",perx ,"   pery:",pery)+brea
                            break
                        
        
           
                            


        return risultato 


if __name__ == '__main__':
    import RPi.GPIO as GPIO

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(17,GPIO.OUT)

    GPIO.output(17,GPIO.HIGH)
    dxframe=320 #dimensione voluta per il frame
    dyframe=240
    t1 = ric(dxframe,dyframe,0, 'Dx')
    t = ric(dxframe,dyframe,2, 'Sx')
    q_ric_dx = multiprocessing.Queue(5) #queue per il riconoscimento
    q_avv_dx = multiprocessing.Queue(1) #queue attivazione telecamere
    q_avv_dx.put(1)
    try:
        t1.main(q_ric_dx,q_avv_dx)
    except KeyboardInterrupt: # termine dei processi in caso di interruzione da tastiera
        None
        #p_ric_dx.join()
        #p_ric_sx.join()
        
    t1.chiudi()
    GPIO.output(17,GPIO.LOW)
    GPIO.cleanup()