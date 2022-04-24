
from os import fspath
import multiprocessing
from pygame.time import Clock
from classemappa import libcasella
import pygame
import pygame.gfxdraw
#import threading
from multiprocessing import  queues
import time
import copy
class disegna():

    def __init__(self) :
        None
        #tY = multiprocessing.Process(target=self.Visualizza,args=())
        '''
        self.tY = threading.Thread(target=self.Visualizza, args=())
        
        self.avvio = False
        self.tY.start()
        '''
    def chiudi(self):
        self.continua = 0
        #self.tY.join()
        
    def AggiornaMod(self, li):
        self.XYG = li
        self.aggiorna=1
        self.avvio = True

    def Aggiorna(self, x, y, g):
        self.aggiorna=1
        self.avvio = True
        if(x<self.nx and x>=0 and y<self.ny and y>=0):
            self.XYG=None
            self.X = x
            self.Y = y
            self.G = g%360
            
            self.posX = round(x*self.casdx + (self.casdx/2))
            self.posY = round(y*self.casdy + (self.casdy/2))
        else:
            print("errore aggiornamento visualizzazioe mappa")

    def BloccaVisualizzazione(self):
        self.aggiorna=0

    def Keyboard(self):
        while True:
            for event in pygame.event.get():  
                if event.type == pygame.KEYUP:  
                    break 
    def Visualizza(self,queue):
        
        
        
        try:
            ricevq = [None,None]
            while(ricevq[0]!=0):
                try:
                    ricevq = queue.get()
                except queues.Empty:
                    None
                time.sleep(0.5)
            mappa = ricevq[1]
            nx = len(mappa)
            ny = len(mappa[0])
          
            
            
            self.finito = 0
            self.nx = nx
            self.ny = ny
            self.mappa = mappa
            self.oldmappa = [ [ libcasella(x,y,4) for y in range(ny)] for x in range(nx) ]

            self.fps = 25
            self.bg = [255,255,255] 
            self.bglivello = 190
            
            
            self.visFramex = 1000
            self.visFramey = 1000
            if(True or (self.nx<15 and self.ny<15)):
                self.visFramex2 = None
                
            else:
                self.visFramex2 = self.visFramex*2
                self.visSize2 =[self.visFramex2, self.visFramey]
            
            self.visSize =[self.visFramex, self.visFramey]
            
            #self.visFrameMin = min([self.visFramex,self.visFramey])
            self.visx = 10
            self.visy = 10

            if(self.visx>self.nx):
                self.visx=self.nx
            if(self.visy>self.ny):
                self.visy = self.ny

            self.casdx = round(self.visFramex/self.visx)
            self.casdy = round(self.visFramey/self.visy)
            self.casSize = [self.casdx, self.casdy]
            self.casmin = min(self.casSize)

            self.relFramex = nx*self.casdx
            self.relFramey = ny*self.casdy
            self.relSize =[self.relFramex, self.relFramey]
            self.relFrameMin = min(self.relSize)

            self.tol = round(3*0.0125*self.casmin)#round(0.0025 * self.relFrameMin) #800->2
            self.lin = round(3*0.0125*self.casmin)#round(2 * 0.00125 * self.relFrameMin) #800->2
              
            
            self.XYG=None

            self.robotsize = round(self.casmin*0.7)
            self.imerrore = pygame.transform.scale(pygame.image.load("/home/pi/Desktop/KODIMAP/errore.png"),(int(self.visFramex/15),int(self.visFramey/15)))
            self.argento = pygame.transform.scale(pygame.image.load("/home/pi/Desktop/KODIMAP/argento1.jpg"),(self.casdx,self.casdy))
            self.robot = pygame.transform.scale(pygame.image.load("/home/pi/Desktop/KODIMAP/robR0NA.png"),(self.robotsize,self.robotsize))
            self.icon = pygame.transform.scale(pygame.image.load("/home/pi/Desktop/KODIMAP/robR0NA.png"),(32,32))
            self.aggiorna=0
            
            self.Aggiorna(1,1,0)
            
            
            
            pygame.init()       
            pygame.font.init()
            myfont = pygame.font.SysFont('Arial', round(0.35*self.casmin) ,bold=True)
            fH  = myfont.render('H', True, (0, 0, 0))
            fS  = myfont.render('S', True, (0, 0, 0))
            fU  = myfont.render('U', True, (0, 0, 0))
            myfont1 = pygame.font.SysFont('Arial', round(0.1875*self.casmin),bold=False)
            fO  = myfont1.render('(0,0)', True, (0, 0, 0))
            self.clock = pygame.time.Clock()  
            self.screen = pygame.Surface(self.relSize, pygame.SRCALPHA)
            if(self.visFramex2!=None):
                self.screenTOT = pygame.Surface(self.visSize, pygame.SRCALPHA)
                self.disp = pygame.display.set_mode(self.visSize2)
            else:
                self.disp = pygame.display.set_mode(self.visSize)
            #self.screen = pygame.display.set_mode(self.size) 
            #self.disp = pygame.display.set_mode(self.visSize2) 
            pygame.display.set_caption("MAPPA")
            pygame.display.set_icon(self.icon)
            #self.avvio = True
            self.screen.fill(self.bg)

            rectRobotRealeOld = pygame.Rect(0,0,0,0)
            rectPosScri  = pygame.Rect(0,0,0,0)
            self.continua = True
            minespx = 9999
            minespy = 9999
            maxespx = 0
            maxespy = 0
            minespxold = -1
            minespyold = -1
            maxespxold = -1
            maxespyold = -1
            crop = [0,0,0,0]
            scrCrop = pygame.Surface(self.visSize, pygame.SRCALPHA)
            
            
            ricevq =[None,None]
            while(ricevq[0]!=1):
                try:
                    ricevq = queue.get()
                except queues.Empty:
                    None
                time.sleep(0.5)
            self.AggiornaMod(ricevq[1])
            
            
            while self.continua:
                
                ricevq =[None,None]
                ctry = 1
                ctrz = 0
                while (ctry):
                    try:
                        ricevq = queue.get_nowait()
                        if(ricevq[0]==0):
                            self.mappa = ricevq[1]
                        elif(ricevq[0]== 1):
                            self.AggiornaMod(ricevq[1])
                        ctrz = 1
                    except queues.Empty:
                        if(ctrz):
                            ctry = 0
                        else:
                            time.sleep(0.2)
                        #None
                
                
                for event in pygame.event.get():  
                    if event.type == pygame.QUIT:  
                        print("QUIT PYGAME")
                        self.avvio =  False 
                        self.continua = False
                        break
                
                if(self.avvio):
                    
                    if(self.XYG!=None):
                        self.X=self.XYG[0]
                        self.Y=self.ny-self.XYG[1]-1#self.XYG[1]
                        self.G=(360-self.XYG[2])%360
                        #print("selfG:0",self.G,self.XYG[2])

                    
                    self.posX = round(self.X*self.casdx + (self.casdx/2))
                    self.posY = round(self.Y*self.casdy + (self.casdy/2))
                    if(self.aggiorna  ):
                        robot = pygame.transform.rotozoom(self.robot,self.G,1)
                        rectRobot = robot.get_rect()   
                        cx = round(self.posX- (rectRobot.w /2) )
                        cy = round(self.posY -(rectRobot.h /2))
                            
                        rectRobotReale = pygame.Rect(cx, cy, rectRobot.w, rectRobot.h)
                    
                        for vx in range(self.nx):
                            for vy in range (self.ny):
                                x = vx
                                #y=vy
                                y = self.ny-vy-1
                                posx = vx*self.casdx
                                posy = vy*self.casdy
                                rectCasella = pygame.Rect(posx, posy, self.casdx, self.casdy)
                                rectCasellaPlus = rectCasella.inflate(2,2)
                                self.screen.set_clip(rectCasella)
                                if(not(self.mappa[x][y]==self.oldmappa[x][y]) or 
                                    pygame.Rect.colliderect(rectRobotReale,rectCasellaPlus) or
                                    pygame.Rect.colliderect(rectRobotRealeOld,rectCasellaPlus) or
                                    pygame.Rect.colliderect(rectPosScri,rectCasellaPlus) or
                                    (vx == 0 and vy == 0) or
                                    self.XYG[3]==1
                                    ):                       
                                    
                                    self.oldmappa[x][y] = copy.deepcopy(self.mappa[x][y])
                                
                                    pygame.draw.rect(self.screen, self.bg, rectCasella)

                                    
                                    diegnackp = 0
                                    if(self.mappa[x][y].colore != 0):

                                        colore = (255,0,0)
                                        if(self.mappa[x][y].coloreVis !=0):
                                            if(self.mappa[x][y].coloreVis ==5):
                                                colore = (200,255,200)
                                            if(self.mappa[x][y].coloreVis ==6):
                                                colore = (255,200,200)
                                        elif(self.mappa[x][y].colore == 1):
                                            colore = (192,192,192)
                                            diegnackp = 1 
                                        elif(self.mappa[x][y].colore == 2):
                                            colore = (50,50,50)
                                        elif(self.mappa[x][y].colore == None):
                                            colore = (200,200,255)
                                    

                                        pygame.draw.rect(self.screen, colore,rectCasella.inflate(-round(self.tol*2),-round(self.tol*2)))
                                        
                                    if(diegnackp):
                                        self.screen.set_clip(rectCasella.inflate(-round(self.tol*1),-round(self.tol*1)))
                                        pygame.Surface.blit(self.screen, self.argento,(posx,posy))
                                        self.screen.set_clip(rectCasella)
                                    
                                    if(self.mappa[x][y].rampa!=None):
                                        tipo = self.mappa[x][y].rampa
                                        rcol = 0
                                        
                                        if(tipo == 2 or tipo == 0):
                                            acol = (255-self.bglivello)/self.casdx
                                            if(tipo == 0):
                                                rcol = self.bglivello
                                            else:
                                                rcol = 255
                                            for rampx in range(self.casdx):
                                                if(tipo == 0):
                                                    rcol += acol
                                                else:
                                                    rcol -= acol
                                                pygame.draw.line(self.screen,(rcol,rcol,rcol),(rampx+posx,posy+0*self.casdy),(rampx+posx,posy+self.casdy),1)
                                        if(tipo == 1 or tipo == 3):
                                            acol = (255-self.bglivello)/self.casdy
                                            if(tipo == 3):
                                                rcol = self.bglivello
                                            else:
                                                rcol = 255
                                            for rampy in range(self.casdy):
                                                if(tipo == 3):
                                                    rcol += acol
                                                else:
                                                    rcol -= acol
                                                pygame.draw.line(self.screen,(rcol,rcol,rcol),(posx+0*self.casdx,posy+rampy),(posx+self.casdx,posy+rampy),1)
                                
                                
                                    if(self.mappa[x][y].colore != None):
                                        if(x<minespx):
                                            minespx = x
                                        if(vy<minespy):
                                            minespy = vy
                                        if(x>maxespx):
                                            maxespx = x
                                        if(vy>maxespy):
                                            maxespy = vy

                                    if(x==0 and y==0):
                                        pygame.Surface.blit(self.screen, fO,(5,self.relFramey-25))
                            
                                    if(self.XYG[3]==1):
                                        scr  = myfont1.render("D:"+str(self.mappa[x][y].distanza) + "P:" +str(self.mappa[x][y].precedente), True, (0, 0, 0))
                                        pygame.Surface.blit(self.screen, scr,(posx+self.casdx*0.2,posy+self.casdy*0.2))

                                    rectCasellaMin = rectCasella.inflate(-self.tol*2,-self.tol*2)
                                    self.screen.set_clip(rectCasellaMin)

                                    for ind in range(4):
                                        coloreV = None
                                        lettera = None
                                        vittima = self.mappa[x][y].vittime[ind]
                                        if(vittima != None):
                                            posizione = ind
                                            if(vittima == 'R' ):
                                                coloreV = (255,0,0)
                                            if(vittima == 'Y' ):
                                                coloreV = (255,255,0)
                                            if(vittima == 'G' ):
                                                coloreV = (0,255,0)
                                            if(vittima == 'C' ):
                                                coloreV = (225,0,230)
                                            if(vittima == 'H' ):
                                                lettera = fH
                                            if(vittima == 'S' ):
                                                lettera = fS
                                            if(vittima == 'U' ):
                                                lettera = fU

                                            rag = round(0.1875 * self.casmin) #15
                                            lin = round(0.0125 * self.casmin) #1

                                            if(posizione == 0):
                                                if(coloreV!=None):
                                                    x1 = round(posx +self.casdx)
                                                    y1 = round(posy+self.casdy*0.5)
                                                    pygame.draw.circle(self.screen,coloreV,(x1, y1),rag)
                                                    pygame.draw.circle(self.screen,(0,0,0),(x1, y1),rag,lin)
                                                if(lettera!=None):
                                                    x1 = round(posx +self.casdx*0.7)
                                                    y1 = round(posy +self.casdy*0.3)
                                                    pygame.Surface.blit(self.screen, lettera,(x1,y1))
                            
                                            if(posizione == 1):
                                                if(coloreV!=None):
                                                    x1 = round(posx +self.casdx*0.5)
                                                    y1 = round(posy)
                                                    pygame.draw.circle(self.screen,coloreV,(x1, y1),rag)
                                                    pygame.draw.circle(self.screen,(0,0,0),(x1, y1),rag,lin)
                                                if(lettera!=None):
                                                    x1 = round(posx +self.casdx*0.4)
                                                    y1 = round(posy+ self.casdy*0.05)
                                                    r = round(0.1875 * self.casdx)
                                                    pygame.Surface.blit(self.screen, lettera,(x1,y1))

                                            if(posizione == 2):
                                                if(coloreV!=None):
                                                    x1 = round(posx)
                                                    y1 = round(posy+self.casdx*0.5)
                                                    pygame.draw.circle(self.screen,coloreV,(x1 , y1 ),rag)
                                                    pygame.draw.circle(self.screen,(0,0,0),(x1 , y1),rag,lin)
                                                if(lettera!=None):
                                                    x1 = round(posx +self.casdx*0.1)
                                                    y1 = round(posy +self.casdy*0.3)
                                                    pygame.Surface.blit(self.screen, lettera,(x1,y1))

                                            if(posizione == 3):
                                                if(coloreV!=None):
                                                    x1 = round(posx + self.casdx*0.5)
                                                    y1 = round(posy+self.casdy)
                                                    pygame.draw.circle(self.screen,coloreV,(x1, y1),rag)
                                                    pygame.draw.circle(self.screen,(0,0,0),(x1, y1),rag,lin)
                                                if(lettera!=None):
                                                    x1 = round(posx +self.casdx*0.4)
                                                    y1 = round(posy +self.casdy*0.6)
                                                    pygame.Surface.blit(self.screen, lettera,(x1,y1))
                        
                                    self.screen.set_clip(rectCasella)
                                    if(self.mappa[x][y].muri[2]==1):
                                        col  = (0,0,255)
                                        col = (0,0,0)
                                        x1 = x2 =  round(posx+self.tol)
                                        y1 = round(posy)#+self.tol)
                                        y2 = round(posy+self.casdy)#-self.tol)
                                        pygame.draw.line(self.screen,col,(x1,y1),(x2,y2),self.lin)
                                    if(self.mappa[x][y].muri[3]==1):
                                        col = (0,0,0)
                                        x1 = round(posx)#+self.tol)
                                        x2 = round(posx+self.casdx)#-self.tol) 
                                        y1 = y2 = round(posy+self.casdy-self.tol)
                                        pygame.draw.line(self.screen,col,(x1,y1),(x2,posy+self.casdy-self.tol),self.lin)
                                    if(self.mappa[x][y].muri[0]==1):
                                        col  = (0,255,0)
                                        col = (0,0,0)
                                        x1 = x2 = round(posx+self.casdx-self.tol)
                                        y1 = round(posy)#+self.tol)
                                        y2 = round(posy+self.casdy)#-self.tol) 
                                        pygame.draw.line(self.screen,col,(x1,y1),(x2,y2),self.lin)
                                    if(self.mappa[x][y].muri[1]==1):
                                        col  = (255,0,0)
                                        col = (0,0,0)
                                        x1 = round(posx)#+self.tol)
                                        x2 = round(posx+self.casdx)#-self.tol) 
                                        y1 = y2 = round(posy+self.tol)
                                        pygame.draw.line(self.screen,col,(x1,y2),(x2,y2),self.lin)
                                
                                
                        self.screen.set_clip()
                        for x in range(0,self.nx+1):
                            for y in range (0,self.ny+1):
                                posx = round(x*self.casdx)
                                posy = round(y*self.casdy)
                                rag = round(self.tol+(self.lin/2))#round(8 * 0.0125 * self.casmin) #5
                                #pygame.draw.circle(self.screen,(0,0,0),(posx,posy),6)
                                pygame.gfxdraw.filled_circle(self.screen,posx,posy,rag,(0,0,0))
                                #pygame.gfxdraw.aacircle(self.screen,int(posx),int(posy),6,(0,0,0))
                                
                        
                        pygame.draw.rect(self.screen, (0,0,0),[0,0,self.relFramex+2,self.relFramey+2],3)
                    
                        #fPS  = myfont1.render(str(int(self.clock.get_fps())), True, (0, 0, 0))
                        
                        #pygame.Surface.blit(self.screen, fPS,(5,5))
                        
                        

                        posRob  = myfont.render("("+str(round(self.XYG[0]))+","+str(round(self.XYG[1]))+")", True, (0, 0, 0))
                        rectPosScri = posRob.get_rect()
                        rectPosScri.top =cy-30
                        rectPosScri.left =cx+30
                        self.screen.fill((255,255,255),rectPosScri)
                        #print(rectPosScri) 
                        pygame.Surface.blit(self.screen, robot,(cx,cy))
                        pygame.Surface.blit(self.screen, posRob,(cx+30,cy-30))  
                        rectRobotRealeOld = copy.copy(rectRobotReale)

                    posinx = (self.X -(self.visx/2))
                    if(posinx<0):
                        posinx=0
                    if((posinx+self.visx)>self.nx):
                        posinx=self.nx-self.visx

                    posiny = (self.Y -(self.visy/2))
                    if(posiny<0):
                        posiny=0
                    if((posiny+self.visy)>self.ny):
                        posiny=self.ny-self.visy

                    inix = posinx  * self.casdx
                    iniy = posiny * self.casdy
                    #screenT = 
                    #pygame.transform.rotozoom(self.screen,0,1)
                    #pygame.Surface.blit(self.disp, screenT,(-100,-100))
                    if(self.finito and False):
                        self.screen = pygame.transform.scale(self.screen,(self.visFramex,self.visFramey))
                        pygame.Surface.blit(self.disp, self.screen,(0,0)) 
                     
                        #time.sleep(20)
                        self.avvio=False
                    else:
                        if(self.visFramex2!=None):
                            #pygame.Surface.blit(self.disp, self.screen,(0,0),(inix,iniy,self.visFramex,self.visFramey))
      
                            inespx = minespx
                            inespy = minespy
                            numespx = (maxespx-minespx +1)
                            numespy = (maxespy-minespy +1)
                            if((numespx>(self.visx/2 ) or numespy >(self.visy/2 )) and minespx!=9999):

                                if(minespx!= minespxold or 
                                    minespy!= minespyold or 
                                    maxespx!= maxespxold or 
                                    maxespy!= maxespyold 
                                    ):
                                    minespxold = minespx
                                    minespyold = minespy
                                    maxespyold = maxespy
                                    maxespxold = maxespx



                                
                                    if(numespx<numespy):
                                        diff = numespy-numespx
                                        numespx = numespy
                                        inespx -= diff/2
                                        if(inespx<0):
                                            inespx = 0
                                        if((inespx + numespx)>self.nx ):
                                            inespx = self.nx - numespx

                                    if(numespy<numespx):
                                        
                                        diff = numespx-numespy
                                        numespy = numespx
                                        inespy -= diff/2
                                        if(inespy<0):
                                            inespy = 0
                                        if((inespy + numespy)>self.ny ):
                                            inespy = self.ny - numespy
                                    
                                    enespx = numespx*self.casdx # + self.casdx
                                    enespy = numespy*self.casdy #+ self.casdy
                                    crop = [inespx*self.casdx,inespy*self.casdy,enespx,enespy]
                                    #print(inespx,inespy,numespx,numespy,crop)

                                    #scrCrop = pygame.transform.chop(self.screen,crop)
                                    scrCrop = pygame.Surface([enespx,enespy], pygame.SRCALPHA)
                                pygame.Surface.blit(scrCrop, self.screen,(0,0),crop) 
                                pygame.transform.scale(scrCrop,(self.visFramex,self.visFramey),self.screenTOT)
                                pygame.Surface.blit(self.disp, self.screenTOT,(self.visFramex,0)) 
                            else:  
                                pygame.transform.scale(self.screen,(self.visFramex,self.visFramey),self.screenTOT)
                                pygame.Surface.blit(self.disp, self.screenTOT,(self.visFramex,0)) 
                        
                        pygame.Surface.blit(self.disp, self.screen,(0,0),(inix,iniy,self.visFramex,self.visFramey))
                        if(self.XYG[3]==2):
                            pygame.Surface.blit(self.disp, self.imerrore,(10,10))
                    '''
                    pixels = pygame.surfarray.pixels2d(self.disp)
                    pixels ^= 2 ** 32 - 1
                    del pixels
                    '''
                pygame.display.update()  
                self.clock.tick(self.fps) 

            pygame.quit() 
        except KeyboardInterrupt :
            pygame.quit()

