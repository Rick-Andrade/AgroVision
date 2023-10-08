import cv2
import serial
import numpy as np

# Ajusta a resolução
def make_1080p():
    video.set(3, 1920)
    video.set(4, 1080)

def make_720p():
    video.set(3, 1280)
    video.set(4, 720)

def make_480p():
    video.set(3, 640)
    video.set(4, 480)

def change_res(width, height):
    video.set(3, width)
    video.set(4, height)

def bicos_bolinha(src,nbicos,bicos_status, ser):
    bicos = np.zeros([int(src.shape[0] + 70),src.shape[1],3],dtype = np.uint8)
    bicos[:src.shape[0],:src.shape[1]] = src
    for i in range(0,nbicos):
        if bicos_status[i] == True:
            cv2.circle(bicos, (int((i*(src.shape[1]/nbicos)) + (src.shape[1]/(2*nbicos))),int(src.shape[0] + 35)), 25, (0,255,0), -1)
            
        else:
            cv2.circle(bicos, (int((i*(src.shape[1]/nbicos)) + (src.shape[1]/(2*nbicos))),int(src.shape[0] + 35)), 25, (0,0,255), -1)
            ser.write(b'Tem bico aberto ae')
    return bicos

ser = serial.Serial('COM3', 115200)
ser.write(b'Serial aberta')

#abre a camera
video = cv2.VideoCapture("videotest1.mp4")

make_720p()

nbicos = 3

# parametros para correcao de distorcao
mtx = np.array([[1.26036733e+03, 0.00000000e+00, 6.25372946e+02],[0.00000000e+00, 1.25723215e+03, 3.35408859e+02],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[0.03902417, -0.00412822, -0.00374443,  0.00342203, -0.22578201]])

e1  =  cv2.getTickCount()
if __name__ == "__main__":
    while(video.isOpened()):

        conect, image = video.read()
        
        #correcao de distorcao da camera
        h,  w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        recons = cv2.undistort(image, mtx, dist, None, newcameramtx)
        #cv2.imshow('Recons', recons)

        # Realiza o corte na regiao de interesse
        h,  w = recons.shape[:2]
        R = 2
        Yi = h-(h/R)
        dst = image[int(Yi):h,0:w]
        #cv2.imshow('Roi', dst)

        # Segmentação 
        #seg_final = segmentation(dst)
        (canalAzul, canalVerde, canalVermelho) = cv2.split(dst)
        RB = cv2.add(canalAzul,canalVermelho)
        seg_final = cv2.subtract(2*canalVerde,RB)
        #cv2.imshow("Segmentação Final", seg_final)
        
        #binarização
        ret1,binary = cv2.threshold(seg_final,20,255,cv2.THRESH_BINARY)
        #cv2.imshow("binarização", binary)

        # Erosao
        element_E = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        erosao = cv2.erode(binary, element_E)
        #cv2.imshow("Erosão",erosao)

        # Dilatacao
        element_D = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        dilata = cv2.dilate(erosao, element_D)
        #cv2.imshow("Dilatacao",dilata)

        bicos_status = []
        for i in range(0,nbicos):
            bicos_status.append(False)

        # Deteccao de bits brancos
        for y in range(int(dilata.shape[0]*0.9),dilata.shape[0]): #10 por cento da imagem em y
            for i in range(0,nbicos):
                status = False
                for x in range(int(i*(dilata.shape[1]/nbicos)), int((i+1)*(dilata.shape[1]/nbicos))):
                    cor = dilata[y][x]
                    if cor == 255 and status == False:
                        bicos_status[i] = True
        #print("fps:", fps)           
        for i in range(0,nbicos):
            if bicos_status[i] == True:
                print("Bico ", i, "on")
            else:
                print("Bico ", i, "off")

        
        color_dilata= cv2.cvtColor(dilata,cv2.COLOR_GRAY2BGR)
        cv2.line(color_dilata,(0,int(dilata.shape[0]*0.9)),(dilata.shape[1],int(dilata.shape[0]*0.9)),(0,0,255),2)
        simulacao = bicos_bolinha(color_dilata,nbicos,bicos_status,ser)        
        
        #cv2.imshow("linha",color_dilata)
        cv2.imshow("Simulacao", simulacao)

        #regula fps e fecha a aplicacao caso a tecla "q" seja pressionada
        if cv2.waitKey(1) == ord('q'):
            break
        e2  =  cv2.getTickCount () 
        t  =  ( e2 - e1 )/cv2.getTickFrequency() 
        print(t)
 
    video.release()
    cv2.destroyAllWindows()
