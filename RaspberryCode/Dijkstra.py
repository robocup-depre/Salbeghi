#from pygame.constants import CONTROLLER_AXIS_INVALID
import classemappa
#import disegnamappa
import disegnamappa2 as disegnamappa
import time
import random
import copy

if __name__ == '__main__':
    base = classemappa.mappa()
    mappa = classemappa.mappa()
    base.leggiTXT("mappa0.txt")
    mappa.Vuoto(base.nx,base.ny)

    #d = disegnamappa.disegna()
    #d.Definisci(base.nx,base.ny,mappa.map)
    d = disegnamappa.disegna(base.nx,base.ny,mappa.map)
    




    inX = int(base.nx/2)
    inY = int(base.ny/2)
    inG = 0

    robo = [inX,inY,inG,0]
    d.AggiornaMod(robo)
    time.sleep(1)
    #d.AggiornaMod(robo)
    stepST = 20
    speed = 0.01


    def sempG(val):
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

    def limit04(inv):
        if(inv>3):
            out=inv-4
        elif(inv<0):
            out=inv+4
        else:
            out = inv
        return out

    def limita180(val):
        while(val>180):
            val-=360
        while(val<-180):
            val+=360
        return val

    def EseguiDijkstra(ix, iy ,mappa):
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


    def TrovaDirezione(mappa, atx, aty, inx = None, iny = None ):
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

    x = 0
    y = 1
    g = 2
    mod = 3
    
    esplor = 0
    while True:
        dira = 1
        mappa.map[robo[x]][robo[y]] = base.map[robo[x]][robo[y]]
        mappa.map[robo[x]][robo[y]].esplorata +=1
        esplor+=1
        esci = 0
        dRelEsci = 0
        numEsplorate = 0
        if(mappa.map[robo[x]][robo[y]].colore==2):
            gira = 0
            dira = -1
        else:
            inespolorate = {}
            numInesplorate = 0
            dRelesciInesp = None
            for dRelativa in range(4):
                G = sempG(robo[g])
                dAssoluta = limit04(G + dRelativa)
                colleg = not(mappa.map[robo[x]][robo[y]].muri[dAssoluta])
                mox = mappa.modX(robo[x],dAssoluta)
                moy = mappa.modY(robo[y],dAssoluta)
                '''
                if(colleg):
                    if(mappa.map[mox][moy].esplorata==0):
                        #print("ho scelto inesporata mano dx")
                        nContorni=0
                        for dAssC in range(4):
                            mox1 = mappa.modX(mox,dAssC)
                            moy1 = mappa.modY(moy,dAssC)
                            if(mox1 != robo[x] and moy1 != robo[y]):
                                try:
                                    if(mappa.map[mox1][moy1].esplorata>0):
                                        nContorni+=1
                                        
                                    
                                    for dAssC2 in range(4):
                                        mox2 = mappa.modX(mox1,dAssC2)
                                        moy2 = mappa.modY(moy1,dAssC2)
                                        if(mox2 != robo[x] and moy2 != robo[y] and 
                                            mappa.map[mox2][moy2].esplorata>0):
                                            nContorni+=1
                                    

                                except:
                                    None    
                        #print("nContorni",nContorni,"diraz:",dRelativa)                    
                        inespolorate[dRelativa] = nContorni
                        # {0:2,1:3}
                        #print(inespolorate)
                        numInesplorate +=1
                        if(numInesplorate==1):
                            dRelesciInesp = dRelativa
                    else:
                        numEsplorate+=1
            

                '''
                numInesplorate = 0
                if(colleg):
                    if(mappa.map[mappa.modX(robo[x],dAssoluta)][mappa.modY(robo[y],dAssoluta)].esplorata==0):
                        #print("ho scelto inesporata mano dx")
                        esci = 1
                        dRelEsci = dRelativa
                        break
                    else:
                        numEsplorate+=1
                        #if(dRelativa!=3):
                            #dRelEsci = dRelativa
                
                
                
                    
            #print(esci, dRelEsci,numEsplorate)
            gira = None

            if(numInesplorate==1):
                dRelEsci = dRelesciInesp
                #gira = 1
                esci = 1
            elif(numInesplorate>1):
                dRelEsci = max(inespolorate,key=inespolorate.get)
                #gira = 1
                esci = 1
                

            mappa.Pulisci()
            if(esci):
                gira = limita180((5-dRelEsci)*90)
            else:
                if(numEsplorate == 1):
                    gira = 180
                '''
                if(numEsplorate == 2):
                    gira = limita180((5-dRelEsci)*90)
                '''
                if(gira==None):
                    tempd = EseguiDijkstra(robo[x],robo[y],mappa)
                    dAssoluta,temp = TrovaDirezione(mappa,robo[x],robo[y])
                    #print("EseguiDijkstra + TrovaDirezione" ,tempd+temp,temp)
                    if(dAssoluta==None):
                        if(robo[x] == inX and robo[y] == inY):
                            #print("ABBIAMO FINITO TUTTO IL LABIRINTO")
                            print("esplor:",esplor,"%=+",((esplor-(mappa.nx *mappa.ny))/(mappa.nx *mappa.ny))*100)
                            d.finito = 1
                            break
                            
                        else:
                            dAssoluta,temp = TrovaDirezione(mappa,robo[x],robo[y],inX,inY)
                            #print(" + TrovaDirezione-> +" ,temp)
                            if(dAssoluta!=None):
                        
                                G = sempG(robo[g])
                                gira = limita180((5-dAssoluta+G)*90)
                                #print("da dijkstra FINALE G:", G,"dAssoluta:", dAssoluta,"gira:", gira)
                    else:
                        
                        G = sempG(robo[g])
                        gira = limita180((5-dAssoluta+G)*90)
                        #print("da dijkstra G:", G,"dAssoluta:", dAssoluta,"gira:", gira)
                        robo[mod]=1
                        #time.sleep(1)
                        #d.Keyboard()
                        robo[mod]=2
            '''
            print("fintio",EseguiDijkstra(0,0,base))
            robo[mod]=1
            break
            '''

        step = gira/stepST
        if(step!=0):
            for a in range(stepST):
                robo[g]=(robo[g]+step)%360
                #g-=0.9
                #d.Aggiorna(x,y,g)
                time.sleep(1*speed)
            

        G = sempG(robo[g])
        direz = limit04(G + 1)


        step = 1/stepST  * dira
        stepx = 0
        stepy = 0
        if(direz==0):
            stepx = step
            stepy = 0
        if(direz==1):
            stepx = 0
            stepy = -step
        if(direz==2):
            stepx = -step
            stepy = 0
        if(direz==3):
            stepx = 0
            stepy = step
        
        for a in range(stepST):
            robo[0]+=stepx
            robo[1]-=stepy
            time.sleep(5*speed)



        robo[0] = round(robo[0])
        robo[1] = round(robo[1])
        robo[2] = round(robo[2])%360

            
    #robo[3]=0
    

