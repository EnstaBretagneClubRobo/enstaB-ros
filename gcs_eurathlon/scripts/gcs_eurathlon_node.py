#!/usr/bin/env python
# -*- coding: utf-8 -*-

##UI
from Tkinter import *
from tkMessageBox import *
from tkFileDialog import *
import led
import os
import atexit
##
##SSH Connection
import thread
import threading
import pxssh
##
##ROS
import rospy
import time
#os.system('xset r off')

@atexit.register
def autorepeat():
    os.system('xset r on')
    ssh.close()

fenetre  = Tk()

label = Label(fenetre,text="Ground Station ROS Eurathlon")
label.pack()


def center(toplevel):
    toplevel.update_idletasks()
    w = toplevel.winfo_screenwidth()
    h = toplevel.winfo_screenheight()
    size = tuple(int(_) for _ in toplevel.geometry().split('+')[0].split('x'))
    x = w/2 - size[0]/2
    y = h/2 - size[1]/2
    toplevel.geometry("%dx%d+%d+%d" % (size + (x, y)))


def callback():
    if askyesno('Sure','try'):
       showwarning('t2','well')
    else:
       showinfo('t3','ss')
       showerror('t4','rr')

POINT_DOWN  = 0
POINT_UP    = 1
POINT_RIGHT = 2
POINT_LEFT  = 3

STATUS_OFF   = 1
STATUS_ON    = 2
STATUS_WARN  = 3
STATUS_ALARM = 4

IS_CONNECTED = 0
connectedSSH = 0

ssh = pxssh.pxssh()



def connection():
    try:                 
        ssh.login (entreIP.get(), entreLog.get(), entreMdp.get())
        ssh.sendline ('uptime')   # run a command
        ssh.prompt()             # match the prompt
        print ssh.before          # print everything before the prompt.
        ssh.sendline ('df')
        ssh.prompt()
        print ssh.before
        bConnect.config(relief = SUNKEN,text='Disconnection',bg='green')
        bConnect.pack()
    except Exception, e:
       showerror( "SSH session failed on login." ,str(e))  
       
    

################## Connection SSH #########################



def connect():
    global connectedSSH,ssh
    if connectedSSH==1:
       ssh.close()
       ssh = pxssh.pxssh()
       bConnect.config(relief = RAISED,text='Connection',bg='grey')
       bConnect.pack()
       showinfo("Disconnection","You have disconnect from the robot")
       connectedSSH = 0
    else:
        print "connection"
        connection()
        connectedSSH = 1
    valueMdp.set("")
    entreMdp.pack()
    

   


p = PanedWindow(fenetre, orient=HORIZONTAL)
p.pack(side=TOP, expand=Y, fill=BOTH, pady=2, padx=2)
frame11 = Frame(width=150, height=70, bg='grey', colormap="new")
valueLog = StringVar()
valueLog.set("elessog")
entreLog = Entry(frame11,textvariable=valueLog,width=30)
entreLog.pack()
p.add(frame11)
frame12 = Frame(width=150, height=70, bg='grey', colormap="new")
valueIP = StringVar()
valueIP.set("localhost")
entreIP = Entry(frame12,textvariable=valueIP,width=30)
entreIP.pack()
p.add(frame12)
frame13 = Frame(width=150, height=70, bg='grey', colormap="new")
valueMdp = StringVar()
valueMdp.set("")
entreMdp = Entry(frame13,textvariable=valueMdp,width=30)
entreMdp.pack()
p.add(frame13)
frame14 = Frame(width=150, height=70, bg='grey', colormap="new")
bConnect = Button(frame14,text="Connection",command=connect,anchor="n")
bConnect.pack()
p.add(frame14)
p.pack()
###########################################################

################# Send simple command #####################
def sendCommand():
    ssh.sendline (entreCmd.get())   # run a command
    ssh.prompt()             # match the prompt
    print ssh.before 
    valueCmd.set("")
    entreCmd.pack()

p2 = PanedWindow(fenetre, orient=HORIZONTAL)
p2.pack(side=TOP, expand=Y, fill=BOTH, pady=2, padx=2)
frame21 = Frame(width=300, height=70, bg='grey', colormap="new")
valueCmd = StringVar()
valueCmd.set("")
entreCmd = Entry(frame21,textvariable=valueCmd,width=60)
entreCmd.pack()
p2.add(frame21)
frame22 = Frame(width=150, height=70, bg='grey', colormap="new")
bCmd = Button(frame22,text="Send Command",command=sendCommand,anchor="n")
bCmd.pack()
p2.add(frame22)
p2.pack()
############################################################

def toggleButton(buttonClicked,buttons): 
    for but in buttons:
        if but is not buttonClicked:
            but.config(relief = RAISED)
            but.pack()
        else:
            but.config(relief = SUNKEN)
            but.pack()

################## Panel Status Launch Algorithme ##########
p3 = PanedWindow(fenetre, orient=HORIZONTAL)
p3.pack(side=TOP, expand=Y, fill=BOTH, pady=2, padx=2)
frame31 = Frame(width=550, height=260, bg='grey', colormap="new")
frame32 = Frame(width=50, height=260, bg='grey', colormap="new")
p3.add(frame31)
#####Left
p31 = PanedWindow(frame31, orient=HORIZONTAL)
p31.pack(side=TOP, expand=Y, fill=BOTH, pady=1, padx=1)
#CCNY
def ccnyLaunch():
    toggleButton(bCmdCCNY_L,ccnyBut)

def ccnyRestart():
    toggleButton(bCmdCCNY_R,ccnyBut)

def ccnyStop():
    toggleButton(bCmdCCNY_S,ccnyBut)


frame311 = Frame(width=180, height=260, bg='grey', colormap="new")
p31.add(frame311)
labelCCNY= Label(frame311,text = "Command 3D Mapping: ",background='grey', anchor=CENTER,pady=3,padx=3)
labelCCNY.pack()
bCmdCCNY_L= Button(frame311,text="Launch ",command=ccnyLaunch,anchor=CENTER)
bCmdCCNY_L.pack()
bCmdCCNY_R= Button(frame311,text="Restart ",command=ccnyRestart,anchor=CENTER)
bCmdCCNY_R.pack()
bCmdCCNY_S= Button(frame311,text="STOP ",command=ccnyStop,anchor=CENTER)
bCmdCCNY_S.pack()
ccnyBut = [bCmdCCNY_L, bCmdCCNY_R, bCmdCCNY_S]

#Hector
def hectorLaunch():
    toggleButton(bCmdHector_L,hectorBut)

def hectorRestart():
    toggleButton(bCmdHector_R,hectorBut)

def hectorStop():
    toggleButton(bCmdHector_S,hectorBut)
    

labelHector= Label(frame311,text = "Command 2D Mapping: ",background='grey', anchor=CENTER,pady=3,padx=3)
labelHector.pack()
bCmdHector_L= Button(frame311,text="Launch ",command=hectorLaunch,anchor=CENTER)
bCmdHector_L.pack()
bCmdHector_R= Button(frame311,text="Restart ",command=hectorRestart,anchor=CENTER)
bCmdHector_R.pack()
bCmdHector_S= Button(frame311,text="STOP ",command=hectorStop,anchor=CENTER)
bCmdHector_S.pack()
hectorBut = [bCmdHector_L, bCmdHector_R, bCmdHector_S]
##### Miscellaneous
frame312 = Frame(width=180, height=260, bg='grey', colormap="new")
p31.add(frame312)
#Cam Recording
#Laser Recording

#Deplacement
def launchPath():
    toggleButton(bCmdPath_L,DrivButton)

def restartPath():
    toggleButton(bCmdPath_R,DrivButton)

def stopPath():
    toggleButton(bCmdPath_S,DrivButton)

def launchPot():
    toggleButton(bCmdPot_L,DrivButton)

def restartPot():
    toggleButton(bCmdPot_R,DrivButton)

def stopPot():
    toggleButton(bCmdPot_S,DrivButton)

frame313 = Frame(width=180, height=260, bg='grey', colormap="new")
p31.add(frame313)
labelDriv= Label(frame313,text = "Command Inside autonomous",background='grey', anchor=CENTER,pady=3,padx=3)
labelDriv.pack()
labelPath= Label(frame313,text = "Pathfinding mode: ",background='grey', anchor=CENTER,pady=3,padx=3)
labelPath.pack()
bCmdPath_L= Button(frame313,text="Launch ",command=launchPath,anchor=CENTER)
bCmdPath_L.pack()
bCmdPath_R= Button(frame313,text="Restart ",command=restartPath,anchor=CENTER)
bCmdPath_R.pack()
bCmdPath_S= Button(frame313,text="STOP ",command=stopPath,anchor=CENTER)
bCmdPath_S.pack()
labelPot= Label(frame313,text = "Potentiel Field mode: ",background='grey', anchor=CENTER,pady=3,padx=3)
labelPot.pack()
bCmdPot_L= Button(frame313,text="Launch ",command=launchPot,anchor=CENTER)
bCmdPot_L.pack()
bCmdPot_R= Button(frame313,text="Restart ",command=restartPot,anchor=CENTER)
bCmdPot_R.pack()
bCmdPot_S= Button(frame313,text="STOP ",command=stopPot,anchor=CENTER)
bCmdPot_S.pack()
DrivButton = [bCmdPath_L, bCmdPath_R, bCmdPath_S, bCmdPot_L, bCmdPot_R, bCmdPot_S]


p3.add(frame32)
#####Right
label1 = Label(frame32,text = "Statut Driver Hokuyo: ",background='grey', anchor=W)
label1.pack()
t1 = led.LED(frame32,status=STATUS_WARN);
t1.frame.pack()
label2 = Label(frame32,text = "Statut Driver Kinect: ",background='grey', anchor=W)
label2.pack()
t2 = led.LED(frame32,status=STATUS_WARN);
t2.frame.pack()
label3 = Label(frame32,text = "Statut SLAM 2D: ",background='grey', anchor=W)
label3.pack()
t3 = led.LED(frame32,status=STATUS_WARN);
t3.frame.pack()
label4 = Label(frame32,text = "Statut SLAM 3D: ",background='grey', anchor=W)
label4.pack()
t4 = led.LED(frame32,status=STATUS_WARN);
t4.frame.pack()
label5 = Label(frame32,text = "Alarm algorithme drift: ",background='grey', anchor=W)
label5.pack()
t5 = led.LED(frame32,status=STATUS_WARN);
t5.frame.pack()
##############################################################



################### Panel Driving command ####################
topControl = None

def closeTeleOp():
    topControl.destroy()
    os.system('xset r on')
    print "end control"

def spawnControl():
    print "control"
    global topControl
    os.system('xset r off')
    topControl = Toplevel()
    topControl.title("Control Robot")
    labelframe = LabelFrame(topControl, text="Commands ", padx=5, pady=5)
    labelframe.pack(fill="both", expand="yes")
    labelframe.pack(padx=10, pady=10)
    up = Label(labelframe,text="UP")
    up.grid(row=0,column=1)
    left = Label(labelframe,text="Left")
    left.grid(row=1,column=0)
    right = Label(labelframe,text="Right")
    right.grid(row=1,column=2)
    down = Label(labelframe,text="Down")
    down.grid(row=2,column=1)
    Button(topControl,text='Close TeleOp',bg='orange',command=closeTeleOp).pack()
    labelframe.bind("<KeyPress>",keydown)
    labelframe.bind("<KeyRelease>",keyup)
    labelframe.pack()
    labelframe.focus_set()
    center(topControl)

def sendGPS():
    print "1-GPS"

topGPS = None
textGPS = None

def sendWayPoint():
    print "Waypoint"
    topGPS.destroy()

def sendMGPS():
    global topGPS,textGPS
    topGPS = Toplevel()
    topGPS.title("Enter GPS coordonates")
    textGPS = Text(topGPS,width= 80,height= 15)
    textGPS.pack()
    Button(topGPS,text='Send WayPoint',bg='pink',command=sendWayPoint).pack()
    print "M-GPS"

p4 = PanedWindow(fenetre, orient=HORIZONTAL)
p4.pack(side=TOP, expand=Y, fill=BOTH, pady=2, padx=2)
frame42 = Frame(width=50, height=70, bg='grey', colormap="new")
frame41 = Frame(width=550, height=70, bg='grey', colormap="new")
#
p4.add(frame41)
labelGPS = Label(frame41,text="Send GPS waypoint",bg='grey')
labelGPS.pack()
valueLong = StringVar()
valueLong.set("Longitude")
entreLong = Entry(frame41,textvariable=valueLong,width=30)
entreLong.pack()
valueLat = StringVar()
valueLat.set("Lattitude")
entreLat = Entry(frame41,textvariable=valueLat,width=30)
entreLat.pack()
button1GPS = Button(frame41,text='Send GPS coord',background = 'yellow',command=sendGPS)
button1GPS.pack()

buttonMGPS = Button(frame41,text='Send Multiple GPS waypoint',background = 'cyan',command=sendMGPS)
buttonMGPS.pack()
#
p4.add(frame42)
buttonTeleOp = Button(frame42,text='Take Control !',background = 'red',command=spawnControl)
buttonTeleOp.pack()


##############################################################


###################     OPI Detection     ####################
p5 = PanedWindow(fenetre, orient=HORIZONTAL)
p5.pack(side=TOP, expand=Y, fill=BOTH, pady=2, padx=2)
frame52 = Frame(width=50, height=70, bg='grey', colormap="new")
frame51 = Frame(width=50, height=70, bg='grey', colormap="new")
#
p5.add(frame51)
#
p5.add(frame52)
labelOPI= Label(frame52,text = "OPI Detection Alarm: ",background='grey', anchor=W)
labelOPI.pack()
ledOPI = led.LED(frame52,status=STATUS_WARN);
ledOPI.frame.pack()
##############################################################
def keyup(e):
    print 'up',e.char

def keydown(e):
    print 'down',e.char




#exit button
button = Button(fenetre,text="Fermer",command=fenetre.quit)
button.pack()





fenetre.mainloop()
os.system('xset r off')
