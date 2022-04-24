import cv2 
import numpy as np
import time
from termcolor import colored, cprint
import threading
import queue as Queue
import multiprocessing
import RPi.GPIO as GPIO
class ric:
    def __init__(self,dx, dy , pos, name):
        self.Avvia = 1
        self.name = "cam:"+name
        self.pos = pos
        self.cap = cv2.VideoCapture(pos)
        self.set(3,dx)
        self.set(4,dy)
        
        '''
        self.set(cv2.CAP_PROP_BRIGHTNESS,0.6)
        self.set(cv2.CAP_PROP_CONTRAST,0.6)
        self.set(cv2.CAP_PROP_SATURATION,0.7)
        self.set(cv2.CAP_PROP_HUE,0.5)
        self.set(cv2.CAP_PROP_GAIN,0)
        '''
        print("BRI",self.get(cv2.CAP_PROP_BRIGHTNESS))
        print("CONT",self.get(cv2.CAP_PROP_CONTRAST))
        print("SAT",self.get(cv2.CAP_PROP_SATURATION))
        print("HUE",self.get(cv2.CAP_PROP_HUE))
        print("GAIN",self.get(cv2.CAP_PROP_GAIN))
        print("EXP",self.get(cv2.CAP_PROP_EXPOSURE))
        print("--------------")
        
        self.dxframe = dy
        self.dyframe = dx
        
        self.frame = np.array((self.dxframe, self.dyframe,3),np.uint8)
        
        self.low_red= (0,140,100)  #0,130
        self.up_red=  (15,255,250)

        self.low_red1= (170,140,100)  #0,130
        self.up_red1=  (180,255,250)

        self.low_green=(40 ,30,50)
        self.up_green= (115,255,250)

        self.low_yellow=(15 ,60,120) #10,100,150
        self.up_yellow=(50,255,250)

        self.low_black=(0 ,0,0)
        self.up_black=(255,255,100)
        
        ''' ULTIMI PRESI CON BERLA
        self.low_red= (0,180,120)  #0,130
        self.up_red=  (15,255,255)

        self.low_red1= (170,180,120)  #0,130
        self.up_red1=  (180,255,255)

        self.low_green=(40 ,65,35)
        self.up_green= (100,255,255)

        self.low_yellow=(15 ,80,150) #10,100,150
        self.up_yellow=(50,255,255)

        self.low_black=(0 ,0,0)
        self.up_black=(255,255,100)
        '''
        
        '''
        self.dxframe = dy
        self.dyframe = dx

        self.low_red= (0,120,100)  #0,130
        self.up_red=  (15,255,255)

        self.low_red1= (170,120,100)  #0,130
        self.up_red1=  (180,255,255)

        self.low_green=(40 ,120,20)
        self.up_green= (80,255,255)

        self.low_yellow=(22 ,120,130) #10,100,150
        self.up_yellow=(35,255,255)

        self.low_black=(0 ,0,0)
        self.up_black=(255,255,100)

        '''
        '''
        
        self.low_red= (0,120,100)  #0,130
        self.up_red=  (15,255,255)

        self.low_red1= (170,120,100)  #0,130
        self.up_red1=  (180,255,255)

        self.low_green=(40 ,75,20)
        self.up_green= (80,255,255)

        self.low_yellow=(10 ,70,130) #10,100,150
        self.up_yellow=(35,255,255)

        self.low_black=(0 ,0,0)
        self.up_black=(255,255,100)
        '''
        self.avvio = 0
        print("INIT RIC",self.name)
    def set(self, prop, val):
        self.cap.set(prop, val)
      
    def get(self, prop):
        return self.cap.get(prop)
    
    def _reader(self):
        while (self.Avvia):
            #print("_reader",self.name)
            try:
                self.avvio = self.q_avv.get_nowait()
            except Queue.Empty:
                pass
            if(self.avvio):
                ret, frame = self.cap.read()
                if not ret:
                    break
                if not self.q_cam.empty():
                    try:
                        self.q_cam.get_nowait()
                    except Queue.Empty:
                        pass
                self.q_cam.put(frame)
            else:
                time.sleep(0.08)
            cv2.imshow(self.name,self.frame)
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
   
    def main(self,q_ric,q_avv):
        self.q_cam = Queue.Queue()
        self.q_avv = q_avv
        self.t_cam = threading.Thread(target=self._reader)
        self.t_cam.daemon = True
        self.t_cam.start()
        self.Avvia = 1
        print("start camera")
        try:
            #frame = np.zeros((100,100,3), np.uint8)
            self.result = np.zeros((400,400,3), np.uint8)
            while(self.Avvia):


                self.frame = frame = cv2.rotate(self.q_cam.get(), cv2.ROTATE_90_CLOCKWISE)
                frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
                mask_red0=cv2.inRange(frame_hsv, self.low_red, self.up_red)
                mask_red1=cv2.inRange(frame_hsv, self.low_red1, self.up_red1)
                mask_green=cv2.inRange(frame_hsv, self.low_green, self.up_green)
                mask_yellow=cv2.inRange(frame_hsv, self.low_yellow, self.up_yellow)
                mask_black=cv2.inRange(frame_hsv, self.low_black, self.up_black)
                mask_red = cv2.bitwise_or(mask_red0,mask_red1)

                mask_red=self.definisci(mask_red,1,4)
                mask_green=self.definisci(mask_green,1,4)
                mask_yellow=self.definisci(mask_yellow,1,4)
                mask_black=self.definisci(mask_black,1,2)

                red = self.trova_colori(mask_red,(0,0,255))
                green = self.trova_colori(mask_green,(0,255,0))
                yellow = self.trova_colori(mask_yellow,(0,255,255))
                black = self.trova_lettere(mask_black)

                LH = black[0]
                LS = black[1]
                LU = black[2]

                trovato = None
                if(LH != 0):
                    trovato = 'H'
                elif(LS != 0):
                    trovato = 'S'
                elif(LU != 0):
                    trovato = 'U'
                elif(red != 0):
                    trovato = 'R'
                elif(green != 0):
                    trovato = 'G'
                elif(yellow != 0):
                    trovato = 'Y'
                else:
                    trovato = 'N'

                if not q_ric.empty():
                    try:
                        q_ric.get_nowait()
                    except Queue.Empty:
                        pass
                try:
                    q_ric.put_nowait(trovato)
                except Queue.Full:
                    pass
                #print("Camera RGY",red,green,yellow)
                #print(frame)
                #cv2.imshow(self.name,self.frame)
                '''
                cv2.imshow("RED"+self.name,mask_red)
                cv2.imshow("GREEN"+self.name,mask_green)
                cv2.imshow("YELLOW"+self.name,mask_yellow)
                cv2.imshow("BLACK"+self.name,mask_black)
                '''
                #cv2.imshow("RESULT"+self.name,self.result)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            None
        self.chiudi()

    def chiudi(self):
        self.cap.release()
        self.Avvia = 0
        #self.t_cam.terminate()
        print("chiuso ric")


    def trova_colori(self,mask,colore):
        risultato=0
        clr=1
        n_col=1
        rit_x=0
        rit_y=0
        rit_dx=0
        rit_dy=0
        trovati =[]
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            '''
            appro = cv2.approxPolyDP(cnt, 0.08*cv2.arcLength(cnt, True), True) #0.025
            #print("ap0",appro,"   .  " ,appro[0])
            lati=len(appro) #numero di bordi
            listax=[]  #lista delle x dei vertici
            listay=[] #lista delle y dei vertici
            for indice in range(lati):
                x,y=appro[indice].ravel() 
                listax.append(x) #aggiungo alle liste le coordinate
                listay.append(y)
            '''
            listax=[]  #lista delle x dei vertici
            listay=[] #lista delle y dei vertici
            
            lati=len(cnt) #numero di bordi
            for indice in range(lati):
                x,y=cnt[indice].ravel() 
                listax.append(x) #aggiungo alle liste le coordinate
                listay.append(y)
            min_x = min(listax) #prendo valori salienti
            max_x = max(listax)
            min_y = min(listay)
            max_y = max(listay)
            
            recta=np.array([[min_x,min_y],[min_x,max_y],[max_x,max_y],[max_x,min_y]])
            a, b, c = colore
            #cv2.drawContours(self.frame, [recta], 0, (a*0.2,b*0.2,c*0.2), 2)
            cv2.drawContours(self.frame, [cnt], 0, (a*0.2,b*0.2,c*0.2), 2)
            dx=abs(max_x-min_x)
            dy=abs(max_y-min_y)

                    
            #cv2.putText(self.frame, str(area), (230, 50),  0.8, (255, 255, 255), 2, cv.LINE_AA)

            
            if(dx!=0 and dy!=0):
                if(dx<dy):
                    rap=dx/dy
                else:
                    rap=dy/dx
                percentulae = int(100 * rap)
                perx= int(100*dx/self.dxframe)
                pery= int(100*dy/self.dyframe)
                xc=min_x+int(dx/2) #coordinate centro teorico
                yc=min_y+int(dy/2) 
                #if(percentulae>60 and perx> 10 and perx<75 and pery>10 and pery<75 and lati==4):
                #cv2.putText(self.frame, str(round(percentulae,0))+"-"+str(perx)+"-"+str(pery), (xc-5, yc),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,50,50), 2, cv2.LINE_AA)
                if(percentulae>60 and perx> 20 and perx<70 and pery>10 and pery<70):
                    #area = cv2.contourArea(cnt)
                    '''
                    aa, bb, cc = self.frame.shape
                    #print(aa,bb)
                    mask = np.zeros((aa,bb), dtype=np.uint8)
                    cv2.fillPoly(mask, [cnt], (255))

                    listHSV = [[]]*3
                    cv2.imshow("mask",mask)
                    #print(min_x,max_x,min_y,max_y)
                    for a in range(min_x,max_x,8):
                        for b in range(min_y,max_y,8):
                            #print(a,b)
                            if(mask[b,a]==255):
                                H, S, V = self.frame[b,a]
                                listHSV[0].append(H)
                                listHSV[1].append(S)
                                listHSV[2].append(V)
                                cv2.circle(self.frame,(a,b),1,(0,0,0),-1)

                    media = [0]*3
                    varianza = [0]*3
                    num = len(listHSV[0])
                    #print("ok")
                    for f in range(3):
                        media[f] = np.mean(listHSV[f])
                        #print("media",media[f])
                    for i in range(num):
                        for f in range(3):
                            varianza[f] += (listHSV[f][i]-media[f])**2
                    for f in range(3):
                        varianza[f] = round((((varianza[f]/(num-1))**(1/2))/abs(media[f]))*100)
                        media[f] = round(media[f],1)
                        
                        
                    print(varianza,num,media)
                    '''

                    ''''

                    if((0,0,255)==colore):
                        pts2 = np.float32([[175,175],[175,225],[225,225],[225,175]])
                    
                    
                        M = cv2.getPerspectiveTransform(appro.astype(np.float32),pts2)
                    
                        self.result = cv2.warpPerspective(self.frame,M,(400,400))
                        
                        for a in range(3):
                            for b in range(3):
                                M[a,b]=round(M[a,b],3)
                        cv2.putText(self.result, str(M[0,:]), (0, 40),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
                        cv2.putText(self.result, str(M[1,:]), (0, 60),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
                        cv2.putText(self.result, str(M[2,:]), (0, 80),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
                    
                        #print()
                    
                    else:
                        self.result = np.zeros((400,400,3))
                    '''
                    #cv2.circle(self.frame,(xc,yc),6,(255,255,255),-1)
                    #cv2.drawContours (self.frame, [appro], 0 , colore, 3)
                    cv2.drawContours (self.frame, [cnt], 0 , colore, 3)
                    
                    #cv2.putText(self.frame, str(int(varianza[0]))+"-"+str(int(varianza[1]))+"-"+str(int(varianza[2])), (xc-30, yc),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)
                    risultato+=1
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
        for cnt in contorni:
            appro= cv2.approxPolyDP(cnt, 0.008*cv2.arcLength(cnt, True), True) #0.007
            lati=len(appro)
            if(lati>10):
                listax=[]  #lista delle x dei vertici
                listay=[] #lista delle y dei vertici
                for indice in range(lati):
                    x,y=appro[indice].ravel() 
                    listax.append(x) #aggiungo alle liste le coordinate
                    listay.append(y)
                min_x = min(listax) #prendo valori salienti
                max_x = max(listax)
                min_y = min(listay)
                max_y = max(listay)
                recta=np.array([[min_x,min_y],[min_x,max_y],[max_x,max_y],[max_x,min_y]])
                #cv2.drawContours(frame_mod, [recta], 0, (255,100,0), 2)
                dx=abs(max_x-min_x)
                dy=abs(max_y-min_y)
                if(dx!=0 and dy!=0):
                    if(dx<dy):
                        rap=dx/dy
                    else:
                        rap=dy/dx
                    percentulae = int(100 * rap) #percentuale dimensioni rettangolo
                    perx= int(100*dx/self.dxframe) #perc rettangolo ripetto a immagine
                    pery= int(100*dy/self.dyframe)
                    #print("perc:",percentulae,"   perx:",perx ,"   pery:",pery)
                    if(percentulae>50 and perx> 10 and perx<80 and pery>10 and pery<80 ): #conrolli
                        #and perx> 50 and perx<98 and pery>50 and pery<98
                        #cv2.drawContours(frame_mod, [recta], 0, (0,100,255), 3)
                        distanza=[]
                        for index in range(lati):
                            index2=index-1
                            if(index2<0):
                                index2=lati-1
                            x,y=appro[index].ravel()
                            x1,y1=appro[index2].ravel()
                            dist=pow( pow(x-x1 , 2)+pow(y-y1, 2) ,1/2)
                            distanza.append(dist)
                            
                        #calcolo lato con distanza massima 
                        dist_max=max(distanza) #dis massima registrata
                        pos= distanza.index(dist_max)  #posizione nella lista
                        distanza[pos]=-1 #cancello distanza 
                        xpd0, ypd0 = appro[pos].ravel() #coordinate primo punto seg1
                        posb=pos-1 #calcolo posizione punto precendente
                        if(posb==-1):
                            posb=lati-1
                        xpd1, ypd1 = appro[posb].ravel() #coordinate secondo punto seg1

                        #calcolo secondo lato con distanza massima 
                        dist_max=max(distanza) #dis massima registrata
                        pos= distanza.index(dist_max)  #posizione nella lista
                        distanza[pos]=-1 #cancello distanza 
                        xpd2, ypd2 = appro[pos].ravel() #coordinate primo punto seg2
                        posb=pos-1 #calcolo posizione punto precendente
                        if(posb==-1):
                            posb=lati-1
                        xpd3, ypd3 = appro[posb].ravel() #coordinate secondo punto seg2

                        media_latimax=(abs(ypd1-ypd0)+abs(ypd2-ypd3))/2
                        perc_latimax=100 *  media_latimax/dy
                        ####print(perc_latimax)
                        
                        #calcolo coordinate linee trasversali
                        xlo0 = self.limita(int(min_x-0.1*dx), 0, self.dxframe) 
                        xlo1 = self.limita(int(max_x+0.1*dx), 0, self.dxframe)
                        ylo  = self.limita(int(dy/2 +min_y), 0, self.dyframe) #y comune alle linee orrizonatli
                        xlv  = self.limita(int(dx/2 + min_x), 0, self.dxframe) #x comune alle linee verticali
                        ylv0 = self.limita(int(min_y-0.1*dy), 0, self.dyframe)
                        ylv1 = self.limita(int(max_y+0.1*dy), 0, self.dyframe)
                        
                        #Disegno sul secondo frame di debug lettere
                        #recta=np.array([[minx,miny],[minx,maxy],[maxx,maxy],[maxx,miny]])
                        #cv2.drawContours(frame_mod2, [recta], 0, (255,255,0), 2)
                        #cv2.circle(frame_mod2,(minx,miny),6,(0,255,0),-1)
                        #cv2.circle(frame_mod2,(maxx,maxy),6,(0,255,0),-1)
                        #Disegno sul frame principale
                        cv2.line(self.frame,(xpd0,ypd0),(xpd1,ypd1),(0,0,255),2)
                        cv2.line(self.frame,(xpd2,ypd2),(xpd3,ypd3),(0,0,255),2)                    
                        cv2.line(self.frame,(xlv,ylv0),(xlv,ylv1),(255,0,255),2)
                        cv2.line(self.frame,(xlo0,ylo),(xlo1,ylo),(255,0,255),2)

                        #scorro le linee trasversali e conto i passaggi da nero a bianco 
                        old_pix=0
                        np_orr=0 #numero passaggi linea orrizontale
                        np_orr0=0 #numero passaggi linea orrizontale prima della meta
                        np_orr1=0 #numero passaggi linea orrizontale dopo della meta
                        mezzo = (xlo0+xlo1)/2
                        for indice in range(xlo0,xlo1):
                            pix = imput[ylo,indice]
                            if((old_pix!=pix and old_pix==0) or (old_pix!=pix and old_pix!=0)):
                                if(indice< mezzo):
                                    cv2.circle(self.frame,(indice,ylo),6,(0,255,255),-1)
                                    np_orr0+=1
                                else:
                                    cv2.circle(self.frame,(indice,ylo),6,(0,100,255),-1)
                                    np_orr1+=1
                            np_orr+=1
                            old_pix=pix
                            
                        old_pix=0
                        per_ver = min_y+( (dy/3) *2);
                        fl_ver=-1
                        np_ver=0 #numero passaggi linea verticale
                        for indice in range(ylv0,ylv1):
                            pix = imput[indice,xlv]
                            if(old_pix!=pix and old_pix==0):
                                np_ver+=1
                                cv2.circle(self.frame,(xlv,indice),6,(255,255,0),-1)
                                if(indice>per_ver):
                                    fl_ver=indice
                                    cv2.circle(self.frame,(xlv,indice),8,(0,255,0),-1)
                            old_pix=pix
                            
                        lettera=None
                        #controlli lettere
                        
                        if(perc_latimax>75 and fl_ver==-1 and np_ver==1):  #H
                            risultato[0]+=1
                            cv2.drawContours(self.frame, [recta], 0, (255,100,100), 2 )
                        elif (np_orr0==1 and np_orr0==1 and np_ver==3 and fl_ver!=-1):   #S
                            risultato[1]+=1
                            cv2.drawContours(self.frame, [recta], 0, (255,255,100), 2)
                        elif (np_orr0==2 and np_orr0==2 and np_ver==1 and fl_ver!=-1 and perc_latimax>50):  #U
                            risultato[2]+=1
                            cv2.drawContours(self.frame, [recta], 0, (255,100,255), 2)
                        
                            


        return risultato 


if __name__ == '__main__':
    
    dxframe=320 #dimensione voluta per il frame
    dyframe=240
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(17,GPIO.OUT)
    GPIO.output(17,GPIO.HIGH) 
    ric_dx = ric(dxframe,dyframe,0, 'Dx')
    ric_sx = ric(dxframe,dyframe,2, 'Sx')
    q_ric_dx = multiprocessing.Queue(5) #queue per il riconoscimento
    q_avv_dx = multiprocessing.Queue(1) #queue attivazione telecamere
    q_avv_dx.put(1)
    q_ric_sx = multiprocessing.Queue(5) #queue per il riconoscimento
    q_avv_sx = multiprocessing.Queue(1) #queue attivazione telecamere
    q_avv_sx.put(1)
    p_ric_dx = multiprocessing.Process(target=ric_dx.main, args=(q_ric_dx,q_avv_dx,)) #multiprocesso delle telecamere, la funzione da eseguire è il main in riconoscimento
    p_ric_sx = multiprocessing.Process(target=ric_sx.main, args=(q_ric_sx,q_avv_sx,))
    
    p_ric_dx.start()
    p_ric_sx.start()
        
    try:
        while(1):
            None
            
    except KeyboardInterrupt: # termine dei processi in caso di interruzione da tastiera
        #p_ric_dx.join()
        #p_ric_sx.join()
        GPIO.cleanup()
        ric_dx.chiudi()
        ric_sx.chiudi()
        p_ric_dx.join()
        p_ric_sx.join()