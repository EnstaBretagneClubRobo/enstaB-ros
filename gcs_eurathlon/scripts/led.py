from Tkinter import *

STATUS_OFF   = 1
STATUS_ON    = 2
STATUS_WARN  = 3
STATUS_ALARM = 4

class LED:
    def __init__(self, master=None, width=25, height=25, 
                 appearance=FLAT,
                 status=STATUS_ON, bd=1, 
                 bg=None, outline="",
                 takefocus=0):
        # preserve attributes
        self.master       = master
        self.shape        = 2 #ROUND
        self.alarmColor   = '#ff4422'
        self.warningColor = '#ffcc00'
        self.onColor = '#00FF33'
        self.offColor = '#656565'
        self.status       = status

        if not bg:
            bg = '#545454'

        ## Base frame to contain light
        self.frame=Frame(master, relief=appearance, bg=bg, bd=bd, 
                         takefocus=takefocus)

        basesize = width
        d = center = int(basesize/2)

        r = int((basesize-2)/2)
        self.canvas=Canvas(self.frame, width=width, height=width, 
                               highlightthickness=0, bg=bg, bd=0)
        if bd > 0:
            self.border=self.canvas.create_oval(center-r, center-r, 
                                                    center+r, center+r)
            r = r - bd
        self.light=self.canvas.create_oval(center-r-1, center-r-1, 
                               center+r, center+r, fill='#00FF33',
                               outline=outline)
        
        self.canvas.pack(side=TOP, fill=X, expand=NO)
        self.update()

    def turnon(self):
        self.status = STATUS_ON
        self.update()

    def turnoff(self):
        self.status = STATUS_OFF
        self.update()

    def update(self):

        if self.status == STATUS_ON:
            self.canvas.itemconfig(self.light, fill=self.onColor)
        elif self.status == STATUS_OFF:
            self.canvas.itemconfig(self.light, fill=self.offColor)
        elif self.status == STATUS_WARN:
            self.canvas.itemconfig(self.light, fill=self.warningColor)
        else :
            self.canvas.itemconfig(self.light, fill=self.alarmColor)

        self.canvas.update_idletasks()


