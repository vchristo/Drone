#! /usr/bin/env python

#pgclock.py
#analog and digital clock example
import csv
import StringIO
import socket
import os, sys, pygame
from pygame.locals import *
import datetime
import time

class item:

    def __init__(self,imagename,colorkey,left,top):
        self.img = pygame.image.load(imagename).convert()
        if colorkey == -1:
            ckey = self.img.get_at((0,0))
            self.img.set_colorkey(ckey, RLEACCEL)
        self.rect = self.img.get_rect()
        self.left = left
        self.top = top
        self.width = self.rect.width
        self.height = self.rect.height
        self.center = self.rect.center

    def draw(self):
        screen.blit(self.img,(self.left, self.top))
		
    def setaxis(self,axis):
        self.axis = axis
    def setaxi(self,axis1):
        self.axis1 = axis1
        
    def setaxis_bar(self,axis_bar):
        self.axis_bar = axis_bar
        
    def setaxattitude(self,axis_at):
        self.axis_at = axis_at
        
        
    def setaxi_ligh_gps(self,axis_lg):
        self.axis_lg = axis_lg
        
    def setaxi_ligh_armed(self,axis_la):
        self.axis_la = axis_la
        
    def setaxi_temp(self,axis_t):
        self.axis_t = axis_t
        
    def drawrot(self,axis,angle):
        #Create new rotated image: preserve original
        self.newimg = pygame.transform.rotate(self.img,angle).convert()
        self.newrect = self.newimg.get_rect()
        #Now center the new rectangle to the rotation axis
        self.newrect.left = axis[0]-(self.newrect.w/2)
        self.newrect.top = axis[1]-(self.newrect.h/2)
        screen.blit(self.newimg,(self.newrect.left, self.newrect.top))
	


#setup screen size and background image
size = width, height = 800, 480
screen = pygame.display.set_mode(size)
#pygame.display.set_mode(size, FULLSCREEN)
pygame.init()
alt = 87.23

#load clock face as background        
bg = item("face.xcf",0,0,0)
bg.setaxis((800-78,78))  #compass
bg.setaxis_bar((800-235,78))
bg.setaxi((800-391,78))  #altimetro
bg.setaxattitude((800-391,329))
bg.setaxi_ligh_armed((110,255))
bg.setaxi_ligh_gps((189,429))
bg.setaxi_temp((253,78))
#load and place clock hands
#the hand images rotate around their own central axis because
#almost one half of the image is set to transparent
#longhand = item("clockhand-long.bmp",-1,90,23)
attitude = item("attitude_pointer.xcf",-1,200,400)
altShort = item("alt_pointer_smal.xcf",-1,800-468,156)
altLong = item("alt_pointer_big.xcf",-1,800-468,156)
temppoint = item("termometer_pointer.xcf",-1,331,156)
barhand = item("barometer_pointer.xcf",-1,800-312,156)
secondhand = item("plane.xcf",-1,800-156,156)
ligth_armed = item("green_light.xcf",-1,190,330)
ligth_gps = item("green_light.xcf",-1,190,410)

#setup font
black = 0,0,0
white = 255,255,255
green = 0,255,0
yelow = 248,206,11
red = 255,0,0
texto = "Missao -29.234678, -50.2445454"
font = pygame.font.Font(None, 20)
font1 = pygame.font.Font("DS-DIGIB.TTF", 20)
alt = 0
alt1 = 0
sqmin= 20
sqmax = 160

TCP_IP = '10.1.1.73'
TCP_PORT = 8888
BUFFER_SIZE = 2000
MESSAGE = "T"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
#send way point message
MESSAGE1 = "T138.705063,-9.355146,60" 
s.send(MESSAGE1)
pygame.time.delay(50)
#to do keyboar input way point
while 1:

    s.send(MESSAGE)
    pygame.time.delay(50)
    data = s.recv(BUFFER_SIZE)
    if not data:
        tp = "20.0"
        a = "80.0"
        t = "128"
        y = "128"
        p = "128"
        r = "128"
        cp = "20"
        pas = "1000"
        s.close()
    else:    
            #print "received data:", data
            f = StringIO.StringIO(data)
            reader = csv.reader(f, delimiter=',')  
            for row in reader:
                print '\t'.join(row) 
                tp = row[0]
                a = row[1]
                t = row[2]
                y = row[3]
                p = row[4]
                r = row[5]
                cp = row[6]
                pas = row[7]
                volt1 = row[8]
                volt2 = row[9]
                gplat = row[10]
                gplon = row[11]
                gpalt = row[12]
                gpf3d = row[13]
                gpgm = row[14]
                gpnm = row[15]
                gpns = row[16]
                gpnb = row[17]
                aty = row[18]
                atr = row[19]
                atp = row[20]  	
    atYaw = float(aty)
    atRol = float(atr)
    atPit = float(atp)
    alt = float(a)
    hpas = 3*(int(pas) - 1000)
    yaw = -((int(y)*160/255))
    throtler =  -((int(t)*160/255))
    roll = -((int(r)*160/255))
    pitch =  -((int(p)*160/255))
    temperature=3*float(tp)
    compass = float(cp)
    latitude = float(gplat)
    longitude = float(gplon)
    gpsaltitude = int(gpalt)
    fix3d = int(gpf3d)
    gpsmode = int(gpgm)
    navmode = int(gpnm)
    nsat = int(gpns)
    nav_bearing = int(gpnb)
    bg.setaxattitude((800-391,329+atPit*4.5))
#    attitude = item("attitude_pointer.xcf",-1,200+atPit,400)
    b=((int(alt * 100))/1000)
    c=((float(alt * 100)/100) - (b * 10 ))
# transformar a parte acima em uma funcao 
	
    #redraw the background to clear the screen
    bg.draw()
	
    #get time

    dt=str(datetime.datetime.today())
    hr = float(dt[11:13])
    min = float(dt[14:16])
    sec = float(dt[17:19])
    time = dt[11:19]
    #get angles for clock hands .. +1 is for pixel correction no in use
    second = -360.0/60*sec +1
    minute = -360.0/60*min +1     
    hour = hr % 12
    hour1 = -360.0/12*hour +1
    #get rotation offset of hour based on minutes
    offset = 360.0/12/60*min
    hour = hour1-offset

    
    #GPS
    fontimg = font1.render("LAT         ",1,yelow)
    screen.blit(fontimg, (20,310))
    fontimg = font1.render(gplat,1,green)
    screen.blit(fontimg, (52,310))
    
    fontimg = font1.render("LON         ",1,yelow)
    screen.blit(fontimg, (20,330))
    fontimg = font1.render(gplon,1,green)
    screen.blit(fontimg, (52,330))
    
    fontimg = font1.render("3D-FIX         ",1,yelow)
    screen.blit(fontimg, (20,350))
    if not fix3d:
       fontimg = font1.render(" NOT",1,red)
    else:
       fontimg = font1.render(" YES",1,green)   
    screen.blit(fontimg, (82,350))

    fontimg = font1.render("Number of Sat         ",1,yelow)
    screen.blit(fontimg, (20,370))
    fontimg = font1.render(gpns,1,green)
    screen.blit(fontimg, (140,370))    
    fontimg = font1.render("GPS ALTITUDE         ",1,yelow)
    screen.blit(fontimg, (20,390))
    fontimg = font1.render(gpalt,1,green)
    screen.blit(fontimg, (135,390)) 
    #Batery state
    fontimg = font1.render("batery        v",1,yelow)
    screen.blit(fontimg, (580,436))
    fontimg = font1.render(volt1,1,red)
    screen.blit(fontimg, (640,436))
    fontimg = font1.render("batery        v",1,yelow)
    screen.blit(fontimg, (580,416))
    fontimg = font1.render(volt2,1,red)
    screen.blit(fontimg, (640,416))
    fontimg = font1.render(cp,1,yelow)
    screen.blit(fontimg, (687,158))
    fontimg = font1.render(pas,1,yelow)
    screen.blit(fontimg, (535,160))
    fontimg = font1.render("hPa",1,yelow)
    screen.blit(fontimg, (575,160))
    fontimg = font1.render(str(alt),1,yelow)
    screen.blit(fontimg, (375,160))
    fontimg = font1.render("m",1,yelow)
    screen.blit(fontimg, (425,160))
    fontimg = font1.render(tp,1,yelow)
    screen.blit(fontimg, (235,158))
    fontimg = font1.render("C",1,yelow)
    screen.blit(fontimg, (275,158))
    if (second%5):
        ligth_armed.drawrot(bg.axis_la,1)
        #ligth_gps.drawrot(bg.axis_lg,1)
    attitude.drawrot(bg.axis_at,atRol)
    altShort.drawrot(bg.axis1,-3.6*b)
    altLong.drawrot(bg.axis1,-c*3.6)
    barhand.drawrot(bg.axis_bar,-hpas)
    temppoint.drawrot(bg.axis_t,-temperature)
    #longhand.drawrot(bg.axis,minute)
    secondhand.drawrot(bg.axis,-compass) #compass
    pygame.draw.rect(screen, green, (15, 180, 18, throtler))
    pygame.draw.rect(screen, green, (49, 180, 18, yaw))
    pygame.draw.rect(screen, green, (83, 180, 18, pitch))
    pygame.draw.rect(screen, green, (123, 180, 18, roll))
    pygame.display.update() 
    pygame.time.delay(100) #process every half second
s.close()
