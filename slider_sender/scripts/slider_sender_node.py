#!/usr/bin/env python
from Tkinter import *
import rospy


def sel():
   selection = "Value = " + str(var.get())
   label.config(text = selection)

root = Tk()
var = DoubleVar()

scale2 = Scale( root, variable = var )
scale2.pack(anchor=N,side = LEFT)
scale1 = Scale( root, variable = var )
scale1.pack(anchor=N,side = LEFT)

#scale3 = Scale( root, variable = var )
#scale3.place(in_= scale2, x=+8,anchor=N)

#scale4 = Scale( root, variable = var )
#scale4.pack(anchor=W)
#scale5 = Scale( root, variable = var )
#scale5.pack(anchor=CENTER)
#scale6 = Scale( root, variable = var )
#scale6.pack(anchor=E)

#scale7 = Scale( root, variable = var )
#scale7.pack(anchor=SW)
#scale8 = Scale( root, variable = var )
#scale8.pack(anchor=SE)

button = Button(root, text="Send Scale Value", command=sel)
button.pack(anchor=CENTER)

label = Label(root)
label.pack()

root.mainloop()
