
import numpy as np
import cv2
import time
from multiprocessing import  queues

def main(q_mappa):
    
    print("ok vis")
    image = np.zeros((910,910), np.uint8) #500500
    #image = cv2.resize(image, (910,910))
    #cv2.imshow("MAPPA",image)
    xim, yim = image.shape
    rob=[0,0,0,0,0]
    for i in range(5):
        name= "/home/pi/Desktop/KODI/mappa/robR"+str(i)+".png"
        rob[i]=cv2.imread(name,0)
    
    while True:
        start=time.time()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #var_lettura = q_mappa.get()
        esegui = 0
        try:
            var_lettura = q_mappa.get(False)
            esegui = 1
        except queues.Empty:
            #print("eccezione queue")
            esegui = 0
        
          
        if(esegui):
            image = cv2.resize(image, (500,500))
            xim, yim = image.shape
            
            
            mod = var_lettura[0]
            
            xa=var_lettura[1]
            ya=var_lettura[2]
            za=1
            prx=var_lettura[3]
            pry=var_lettura[4]
            prg=var_lettura[5]

            m_muri =var_lettura[6]
            m_vittime =var_lettura[7]
            m_colore=var_lettura[8]
            
            prx0=prx
            pry0=pry
            
            dim1 = xim/xa
            dim2 = yim/ya
            
            spx=4
            spy=4
            po1=prx-spx
            po2=prx+spx+1
            po3=pry-spy
            po4=pry+spy+1
            if(po1<0):
                po2=spx+spy+1
                po1=0
            if(po3<0):
                po4=spx+spy+1
                po3=0
            if(po2>xa):
                po1=xa-spx+spy+1
                po2=xa
            if(po4>ya):
                po3=ya-spx+spy+1
                po4=ya
                
            xa = dx=abs(po2-po1)
            ya = dy=abs(po4-po3)
            dim1 = xim/xa
            dim2 = yim/ya
            min_x=po1
            max_x=po2
            min_y=po3
            max_y=po4
                
            if(dim1<dim2):
                dim3=dim1
            else:
                dim3=dim2
            for x in range(xim):
                for y in range(yim):
                    image[x,y]=255
                  
            #print("vis stA")
            #x1=1*dim1

            
            for nx in range(min_x,max_x):
                for ny in range(min_y,max_y):

                        
                    tip_vittime=m_vittime[nx,ny,0]
                    pos_vittime=m_vittime[nx,ny,1]
                    col=m_colore[nx,ny]

                    rex = nx-min_x
                    rey = ny-min_y
                    x1=rex*dim1
                    y1=rey*dim2
                    x2=rex*dim1+dim1
                    y2=rey*dim2+dim2
                        
                    if(col==0):
                        x1=rex*dim1 + dim1/2
                        y1=rey*dim2+ dim2/2
                        cv2.circle(image,(int(x1),int(y1)),int(dim1/20),0,-1)
                    else:
                        
                        
                        if(col==2):
                            cv2.rectangle(image,(int(x1),int(y1)),(int(x2),int(y2)),200,-1)
                        elif (col==3):
                            cv2.rectangle(image,(int(x1),int(y1)),(int(x2),int(y2)),100,-1)
                        vittima=0
                        quad=0
                        if(tip_vittime==1):
                            if(pos_vittime==1):
                                x1+=dim1/2
                                gr=0
                            elif(pos_vittime==2):
                                y1+=dim2/2
                                x1+=dim1
                                gr=90
                            elif(pos_vittime==3):
                                y1+=dim2
                                x1+=dim1/2
                                gr=180
                            elif(pos_vittime==0):
                                y1+=dim2/2
                                gr=-90
                            cv2.ellipse(image,(int(x1),int(y1)),(int(dim1/6.5),int(dim2/6.5)),0,gr,gr+180,100,-1)
                        
                        elif(tip_vittime==2):
                            vittima='H'
                        elif(tip_vittime==3):
                            vittima='S'
                        elif(tip_vittime==4):
                            vittima='U'
                        elif(tip_vittime==5):
                            vittima='R'
                            quad=1
                        elif(tip_vittime==6):
                            vittima='V'
                            quad=1
                        elif(tip_vittime==7):
                            vittima='G'
                            quad=1
                        if(vittima!=0):
                            if(pos_vittime==0):
                                x1=x1+dim1/2 -dim1*0.08
                                y1= y1+dim2*0.3
                                
                            if(pos_vittime==1):
                                x1=x1 +dim1-dim1*0.3
                                y1= y1+dim2/2 +dim2*0.08
                            if(pos_vittime==2):
                                x1=x1+dim1/2 -dim1*0.08
                                y1= y1+dim2-dim2*0.08
                            if(pos_vittime==3):
                                x1=x1+dim1*0.08
                                y1= y1+dim2/2 +dim2*0.08
                            xt1=x1
                            yt1=y1
                            if(quad==1):
                                x1=x1-dim1*0.02
                                y1=y1+dim2*0.03
                                x2=x1+dim1*0.22
                                y2=y1-dim2*0.25
                                cv2.rectangle(image,(int(x1),int(y1)),(int(x2),int(y2)),200,-1)
                            
                            cv2.putText(image, vittima, (int(xt1), int( yt1)),cv2.FONT_HERSHEY_SIMPLEX, dim2/125, 0, 2 , cv2.LINE_AA)
                            
                            
                        
            
            
            for nx in range(min_x,max_x):
                for ny in range(min_y,max_y):
                    rex = nx-min_x
                    rey = ny-min_y
                    if(m_muri[nx,ny,1]==1):
                        x1=rex*dim1
                        y1=rey*dim2
                        x2=rex*dim1+dim1
                        y2=rey*dim2
                        cv2.line(image,(int(x1),int(y1)),(int(x2),int(y2)),0,int(dim3/15))
                    if(m_muri[nx,ny,2]==1):
                        x1=rex*dim1+dim1
                        y1=rey*dim2
                        x2=rex*dim1+dim1
                        y2=rey*dim2+dim2
                        cv2.line(image,(int(x1),int(y1)),(int(x2),int(y2)),0,int(dim3/15))
                    if(m_muri[nx,ny,3]==1):
                        x1=rex*dim1
                        y1=rey*dim2+dim2
                        x2=rex*dim1+dim1
                        y2=rey*dim2+dim2
                        cv2.line(image,(int(x1),int(y1)),(int(x2),int(y2)),0,int(dim3/15))
                    if(m_muri[nx,ny,0]==1):
                        x1=rex*dim1
                        y1=rey*dim2
                        x2=rex*dim1
                        y2=rey*dim2+dim2
                        cv2.line(image,(int(x1),int(y1)),(int(x2),int(y2)),0,int(dim3/15))
            
            for x in range(min_x,max_x):
                rx = x-min_x
                if(dim1/500>=1):
                    cv2.line(image,(int(rrx*dim1),0),(int(x*dim1),yim),0,int(dim1/500))

            for y in range(min_y,max_y):
                ry=y-min_y
                if(dim2/500>=1):
                    cv2.line(image,(0,int(ry*dim2)),(xim-1, int(ry*dim2)),0,int(dim2/500))

            vacca=cv2.resize(rob[prg],(int(dim3/1.2),int(dim3/1.2)))
            xvac, yvac = vacca.shape
            x1=(prx-min_x)*dim1 +dim1/2 - xvac/2
            y1=(pry-min_y)*dim2 +dim2/2 - yvac/2
            
            
           

            
            for ix in range(yvac):
                for iy in range(xvac):
                    vaccc=vacca[iy,ix]
                    if(vaccc!=255):
                        i1=int(iy+y1)
                        i2=int(ix+x1)
                        if(i1<yim and i2<xim):
                            image[i1,i2]    = vaccc
            
            cv2.rectangle(image,(20,20),(90,50),255,-1)
                            
            cv2.putText(image, "(" +str(prx0)+":"+str(pry0)+")", (20,40),cv2.FONT_HERSHEY_SIMPLEX,0.6, 0, 2 , cv2.LINE_AA)
            #print(time.time()-start)       
            

            image = cv2.resize(image, (910,910))
        cv2.imshow("MAPPA",image)
