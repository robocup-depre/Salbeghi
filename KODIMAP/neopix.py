import board
import neopixel

class Neopix():
    def __init__(self):
  
        try:
            self.Nled = 2
            #D21
            self.pixels = neopixel.NeoPixel(board.D18,self.Nled, brightness=0.5, auto_write=True)#D18 #21
            for ind in range(self.Nled):
                self.pixels[ind] = (0, 0, 0)
            self.errore = 0
        except:
            print("errore Neopix")
            self.errore = 1
        #self.errore = 1

    def Led(self,i,color):
        if(self.errore == 0):
            if(i>=0 and i<self.Nled):
                self.pixels[i]=color
            else:
                for ind in range(self.Nled):
                    self.pixels[ind] = color

    def SpegniLed(self):
        if(self.errore == 0):
            for ind in range(self.Nled):
                self.pixels[ind] = (0, 0, 0)

    def VisualizzaTrovato(self,ledpos,trovato):
        if(trovato=='N'):
            self.Led(ledpos,(0,0,0)) #SPENTO
        if(trovato=='H'):
            self.Led(ledpos,(0,0,255)) #BLU
        if(trovato=='S'):
            self.Led(ledpos,(255,127,0)) # ARANCIONE
        if(trovato=='U'):
            self.Led(ledpos,(255,0,255)) # VIOLA
        if(trovato=='R'):
            self.Led(ledpos,(255,0,0)) #ROSSO
        if(trovato=='G'):
            self.Led(ledpos,(0,255,0)) #VERDE
        if(trovato=='Y'):
            self.Led(ledpos,(255,255,0)) #GIALLO
        if(trovato=='C'):
            self.Led(ledpos,(255,200,220)) #ROSA

