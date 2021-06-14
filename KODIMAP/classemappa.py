
import copy



class libcasella():
    def __init__(self, x, y,col=0):
        self.x = x
        self.y = y
        self.z = 0
        self.distanza = 99999999
        self.esplorata = 0
        self.precedente = None
        self.vittime =  [None for _ in range(4)]
        self.muri = [None for _ in range(4)]
        self.coloreVis = 0
        self.rampa = None
        if(col==0):
            self.colore = None
        else:
            self.colore = col

    def Pulisci(self):
        self.distanza = 99999999
        self.precedente = None
        self.coloreVis = 0
    def dif(self,obj):
        return self != obj

    def __eq__(self, other):
        #print("CONTR")
        if(self.colore == other.colore
            and self.muri == other.muri
            and self.vittime == other.vittime
            and self.distanza == other.distanza
            and self.precedente == other.precedente
            and self.coloreVis == other.coloreVis
            and self.rampa == other.rampa
            and self.z == other.z
            and self.esplorata == other.esplorata):
            return True

        return False


class mappa():
    



    nx = 0
    ny = 0
    map = None
    '''
    def __eq__(self, other):
        if self.__class__ == other.__class__:
            fields = [field.name for field in self._meta.fields]
            for field in fields:
                if not getattr(self, field) == getattr(other, field):
                    return False
            return True
    '''
    def __eq__(self, other):
        if (isinstance(other, mappa)):
                if(self.map == other.map):
                    return True
           
        return False

    def Pulisci(self):
        for x in range(self.nx):
            for y in range(self.ny):
                self.map[x][y].Pulisci()
    def dif(self,obj):
        return self.map != obj.map

    def modX(self,X,num,inverti=1):
        if(num==0):
            X+= (1 *inverti)
        if(num==2):
            X-= (1 *inverti)
        return X

    def modY(self,Y,num,inverti=1):
        if(num==1):
            Y+= (1 *inverti)
        if(num==3):
            Y-= (1 *inverti)
        return Y

    def modXY(self,X,Y,num):
        return self.modX(X,num),self.modY(Y,num)

    def Vuoto(self, nx, ny):
        self.nx = nx
        self.ny = ny
        self.map =  [ [ libcasella(x,y) for y in range(ny)] for x in range(nx) ]

    def leggiTXT(self,percorso):
        file = open(percorso, "r")
        testo = file.readlines()
        file.close()
        ncolonne =len(testo[0])
        self.nx = nx = int((ncolonne-2)/2)
        nrighe = len(testo)
        self.ny = ny = int((nrighe-2)/2)
        print("nx:",nx,"ny:",ny)
        self.map =  [ [libcasella(x,y) for y in range(ny)] for x in range(nx) ]
        i = 0
        simbolo = ['R','G','V','H','S','U','T']
        for y in range(ny):
            righa = nrighe -3 - (y*2)
            for x in range(nx):
                i+=1
                colonna = (x*2)+1
                casella = testo[righa][colonna]
                if(casella == ' '):
                    self.map[x][y].colore = 0
                elif (casella == '$'):
                    self.map[x][y].colore = 1
                elif (casella == '@'):
                    self.map[x][y].colore = 2
                else:
                    
                    asc = ord(casella) -65
                    uscita = 0
                    for i in range(7):
                        if(asc>=0):
                            if(asc<4):
                                self.map[x][y].vittime[asc]= simbolo[i]
                                self.map[x][y].colore = 0
                                uscita = 1
                                break
                            else:
                                asc -= 4
                        else:
                            break
                        

                    if(uscita==0):
                        print("ERRORE LETTURA MAPPA TXT")
                        exit
                

                # regola destra 0-DX 1-SU' 2-SX 3-GIU'
                '''
                if(testo[righa][colonna+1] == "#"):
                    self.map[x][y].muri[0]=1
                elif(testo[righa][colonna+1] == " "):
                    self.map[x][y].muri[0]=0

                if(testo[righa-1][colonna] == "#"):
                    self.map[x][y].muri[1]=1
                elif(testo[righa-1][colonna] == " "):
                    self.map[x][y].muri[1]=0

                if(testo[righa][colonna-1] == "#"):
                    self.map[x][y].muri[2]=1
                elif(testo[righa][colonna-1] == " "):
                    self.map[x][y].muri[2]=0

                if(testo[righa+1][colonna] == "#"):
                    self.map[x][y].muri[3]=
                elif(testo[righa+1][colonna] == " "):
                    self.map[x][y].muri[3]=0
                '''
                for ind in range(4):
                    carattere = testo[self.modY(righa,ind,-1)][self.modX(colonna,ind)]
                    if(carattere=='#'):
                        self.map[x][y].muri[ind]=1
                    elif(carattere==' '):
                        self.map[x][y].muri[ind]=0
                