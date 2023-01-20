
# -*- coding: utf-8 -*-
import os
import cv2 as cv
import time
import sys
import itertools
import multiprocessing 

class bcolors:#字體顏色
    OK = '\033[92m' #GREEN
    WARNING = '\033[93m' #YELLOW
    FAIL = '\033[91m' #RED
    RESET = '\033[0m' #RESET COLOR

def map_quality(path , pathlist, Sensitive):#讀入地圖路徑進行該地圖評分(可調整fragment Sensitive偵測敏感度) 

    img_1 = cv.imread(path+'/'+pathlist)
    img = cv.cvtColor(img_1, cv.COLOR_BGR2GRAY)
#    cv.imshow('1',img)
#    cv.waitKey(0)
#    cv.destroyAllWindows()

    for i in range(img.shape[0]-1):
        for j in range(img.shape[1]-1):
            if(img[i][j]>=0 and img[i][j]<=100):
                img[i][j]=0
            elif(img[i][j]>100 and img[i][j]<235):
                img[i][j]=150
            elif(img[i][j]>235 and img[i][j]<=255):
                img[i][j]=255
            else:
                img[i][j]=255
    
    
    for i in range(img.shape[0]-1):
        for j in range(img.shape[1]-1):
            if(img[i][j]==0):
                for a in range(i-1,i+1,1):
                    for b in range(j-1,j+1,1): 
                        img[a][b]=0
                        
                    
    
    cv.imwrite(path+'/map_out/'+ pathlist , img)
    img = cv.cvtColor(img,cv.COLOR_GRAY2BGR)

    fragment=0
    white=0
    
    for i in range(1,img.shape[0]-1):
        for j in range(1,img.shape[1]-1):
            
            if(img[i][j][0]==255):
                white+=1
                black_flag=0
                gray_flag=0
                white_flag=0
                for a in range(i-2,i+2):
                    for b in range(j-2,j+2):                    
                        if(img[a][b][0]==0):
                            black_flag=1
                        elif(img[a][b][0]==150):
                            gray_flag=1
                        elif(img[a][b][0]==255):
                            white_flag+=1
                            
                if(black_flag!=1 and gray_flag==1 and white_flag<Sensitive):                  
                    cv.circle(img, (j, i), 2, (0, 255, 0), 2)
                    fragment+=1
                    
                else:
                    continue

    sys.stdout.flush()
    sys.stdout.write(bcolors.OK + '\rDone!                ' + bcolors.RESET)
    print('\nMap Area: {} \nFragment Quantity: {} \nMapping Defect Score: {} %\n '.format(white, fragment, float(fragment/(white/100.0))))
    
    cv.imwrite(path+'/map_out/out_'+ pathlist , img)

def loading():#loading 動畫

    for c in itertools.cycle(['.     ', '..    ', '...   ', '....  ', '..... ', '......']):
        sys.stdout.flush()
        sys.stdout.write(bcolors.WARNING + '\rprocessing ' + c+ bcolors.RESET)       
        time.sleep(0.4)
    
def finish():#結束動畫
    list = ['_(┐ / ε:)_ ', '_  _(┐ / ε:)', ':)_  _(┐ / ε', ' ε:)_  _(┐ /', ' / ε:)_  _(┐', '(┐ / ε:)_  _']
    for i in range(len(list)*3):
        sys.stdout.flush()
        sys.stdout.write('\r ' + list[i%5])       
        time.sleep(0.4)
    sys.stdout.flush()
    sys.stdout.write('\r         ∠( ᐛ 」∠)＿         \n')



if __name__ == '__main__':
    path=input(r'please input map path : ')
    
    pathlist = os.listdir(path)
#    if os.path.isdir(path+'/map_out'):
#        print(bcolors.FAIL + "\nPlease remove file map_out\n" + bcolors.RESET)
#        quit()
    pathlist.sort(key=lambda x:int(x.split('.')[0]))
    print(pathlist)
    if not os.path.isdir(path+'/map_out'):
        os.mkdir(path+'/map_out')
    
    Sensitive = 5 #更改（1<Sensitive<14）可調整辨識fragment 偵測敏感度
    done=0
   
    for i in range(0, len(pathlist)):
        done+=1
#        sys.stdout.write('\r{} ({}/{}) \n'.format(path+'/'+pathlist[i], done, len(pathlist)))
        sys.stdout.write('\r{} \n'.format(path+'/'+pathlist[i]))
        a=multiprocessing.Process(target = map_quality, args=(path, pathlist[i], Sensitive))
        b=multiprocessing.Process(target = loading)
        a.start()
        b.start()
        a.join()
        b.terminate()
        b.join()

    finish()
