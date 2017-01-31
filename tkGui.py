#!/usr/bin/python
from Tkinter import *

m1 = PanedWindow(height=500, width=500)
m1.pack(fill=BOTH, expand=1)

m2 = PanedWindow(m1, orient=VERTICAL)
m1.add(m2)

tall = Label(m1, text="tall pane")
m1.add(tall)

top = Label(m2, text="Lat:")
top.pack(side=LEFT)
tLat = Entry(m2, bd=5)
tLat.pack(side=RIGHT)

bottom = Label(m2, text="bottom pane")
m2.add(bottom)

mainloop()
