#!/usr/bin/env python
from Tkinter import *
import rospy
from colour_detection.srv import *

def sel():
   i = 1
   for va in var:
      try:
        set_threshold = rospy.ServiceProxy("/colour_detect/set_thresholds", Threshold)
        resp1 = set_threshold(i,va.get())
        i=i+1
      except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        exit(0)
   selection ="true"
   label.config(text = selection)

root = Tk()
var1 = DoubleVar()
var2 = DoubleVar()
var3 = DoubleVar()
var4 = DoubleVar()
var5 = DoubleVar()
var6 = DoubleVar()
var7 = DoubleVar()
var8 = DoubleVar()

var = [var1,var2,var3,var4,var5,var6,var7,var8]

scale1 = Scale( root, variable = var1 , label = "seuilBh",from_=0, to=179 )
scale1.pack(anchor=N,side = LEFT)
scale2 = Scale( root, variable = var2 , label = "seuilBs",from_=0, to=250)
scale2.pack(anchor=N,side = LEFT)


scale3 = Scale( root, variable = var3 , label = "seuilBv",from_=0, to=250)
scale3.pack(anchor=N,side = LEFT)

scale4 = Scale( root, variable = var4 , label = "seuilHh",from_=0, to=179)
scale4.pack(anchor=N,side = LEFT)
scale5 = Scale( root, variable = var5 , label = "seuilHs",from_=0, to=250)
scale5.pack(anchor=N,side = LEFT)
scale6 = Scale( root, variable = var6 , label = "seuilHv",from_=0, to=250)
scale6.pack(anchor=N,side = LEFT)

scale7 = Scale( root, variable = var7 , label = "nbIter",from_=0, to=250)
scale7.pack(anchor=N,side = LEFT)
scale8 = Scale( root, variable = var8 , label = "nbIterD",from_=0, to=250)
scale8.pack(anchor=N,side = LEFT)

button = Button(root, text="Send Scale Value", command=sel)
button.pack(anchor=CENTER)

label = Label(root)
label.pack()

root.mainloop()
