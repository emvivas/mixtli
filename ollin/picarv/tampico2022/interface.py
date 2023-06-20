#Program: Autonomous Line Follower PiCarV
#Developer: Emiliano Vivas Rodríguez
#Contact: a01424732@tec.mx
#Since: 2022/10/10


import cv2
import imutils
from PIL import Image, ImageTk
from tkinter import Tk, Frame, Label, Button, Canvas
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.animation as animation
from matplotlib.figure import Figure
import random
import matplotlib

def imageFrameConfiguration():
    global visualBlur
    rtf, img = capture.read()
    if rtf:
        originalVideo = imutils.resize(img, width=510, height=360)
        video = cv2.cvtColor(originalVideo, cv2.COLOR_BGR2RGB)
        videoBlur = cv2.blur(video, (20, 20))
        visualization = Image.fromarray(video)
        visualizationBlur = Image.fromarray(videoBlur)
        visual = ImageTk.PhotoImage(image=visualization)
        visualBlur = ImageTk.PhotoImage(image=visualizationBlur)
        originalImageLabel.configure(image=visual)
        originalImageLabel.image = visual
        originalImageLabel.after(10, imageFrameConfiguration)

def imageThresholdConfiguration():
    global visualBlur
    if visualBlur is not None:
        imageThresholdLabel.configure(image=visualBlur)
        imageThresholdLabel.image = visualBlur
        imageThresholdLabel.after(10, imageThresholdConfiguration)

matplotlib.use('TkAgg')
capture = cv2.VideoCapture(0)
visualBlur = None
cv2.destroyAllWindows()
graphicInterface = Tk()
graphicInterface.title("Autonomous Mechatronics Grand Prix. SunFounder Smart Video Car Kit V2.0 PiCar-V Robot Control Panel")
#graphicInterface.iconbitmap("./amgp-logo.ico")
graphicInterface.resizable(width=False, height=False)
graphicInterface.geometry('{}x{}'.format(1120, 630)) #16:9
window = Frame(graphicInterface, bg="black")
window.grid(column=0, row=0)
window.columnconfigure(0, weight=1)
window.rowconfigure(0, weight=1)
banner = Frame(master=window, width=1120, height=50, bg="black")
banner.grid(column=0, row=0, columnspan=5)
tecLogo = Image.open("./tec-logo.webp")
tecLogo.thumbnail((175, 90))
canvas = Canvas(banner, bg="black", width=180, height=50, borderwidth=0, bd=0, highlightthickness=0, relief='ridge')
tecLogo = ImageTk.PhotoImage(tecLogo)
canvas.create_image(90, 25, image=tecLogo)
canvas.grid(column=0, row=0, padx=10)

competitionLogo = Image.open("./amgp-logo.webp")
competitionLogo.thumbnail((45, 45))
canvas = Canvas(banner, bg="black", width=50, height=50, borderwidth=0, bd=0, highlightthickness=0, relief='ridge')
competitionLogo = ImageTk.PhotoImage(competitionLogo)
canvas.create_image(25, 25, image=competitionLogo)
canvas.grid(column=1, row=0, padx=10)

title = Label(banner, text = "Autonomous Mechatronics Grand Prix. Tecnológico de Monterrey campus Cuernavaca", height=3)
title.config(fg="white", bg="black", font=("Arial", 9))
title.grid(column=2, row=0)

originalImage = Frame(window, width=510, height=359, bg="white")
originalImage.grid(column=0, row=1, columnspan=2, rowspan=2)
originalImageLabel = Label(originalImage, bg="black")
originalImageLabel.place(x=0, y=0)

imageThreshold = Frame(window, width=510, height=359, bg="white")
imageThreshold.grid(column=2, row=1, columnspan=2, rowspan=2)
imageThresholdLabel = Label(imageThreshold, bg="black")
imageThresholdLabel.place(x=0, y=0)

selectionPanel = Frame(window, width=100, height=580, bg="black")
selectionPanel.grid(column=4, row=1, rowspan=4)

startButton = Button(selectionPanel, text="Start\n mission", width=8, height=6, font=("Arial", 9), command = None)
startButton.grid(column=0, row=0, padx=10, pady=7)

stopButton = Button(selectionPanel, text="Abort\n mission", width=8, height=6, font=("Arial", 9), command = None)
stopButton.grid(column=0, row=1, padx=10, pady=7)

modeButton = Button(selectionPanel, text="Change\n mode", width=8, height=6, font=("Arial", 9), command = None)
modeButton.grid(column=0, row=2, padx=10, pady=7)

testButton = Button(selectionPanel, text="Test\n operativity", width=8, height=6, font=("Arial", 9), command = None)
testButton.grid(column=0, row=3, padx=10, pady=7)

colorSelectedFrame = Frame(selectionPanel, width=80, height=120, bg="black")
colorSelectedFrame.grid(column=0, row=4, padx=10, pady=7)

borderColor = Frame(colorSelectedFrame, width=80, height=60, bg="red")
borderColor.grid(column=0, row=0)
borderColorLabel = Label(borderColor, text = "Border", width=10, height=3)
borderColorLabel.config(fg="white", bg="red", font=("Arial", 9))
borderColorLabel.grid(column=0, row=0)

lineColor = Frame(colorSelectedFrame, width=80, height=60, bg="orange")
lineColor.grid(column=0, row=1)
lineColorLabel = Label(lineColor, text = "Line", width=10, height=3)
lineColorLabel.config(fg="white", bg="orange", font=("Arial", 9))
lineColorLabel.grid(column=0, row=0)

velocityGraphicFrame = Frame(window, width=255, height=220, bg="white")
velocityGraphicFrame.grid(column=0, row=3, rowspan=2)
directionGraphicFrame = Frame(window, width=255, height=220, bg="white")
directionGraphicFrame.grid(column=1, row=3, rowspan=2)
CPUUsageGraphicFrame = Frame(window, width=255, height=170, bg="white")
CPUUsageGraphicFrame.grid(column=2, row=3)
RAMUsageGraphicFrame = Frame(window, width=255, height=170, bg="white")
RAMUsageGraphicFrame.grid(column=3, row=3)

outputFrame = Frame(window, width=510, height=50, bg="black")
outputFrame.grid(column=2, row=4, columnspan=2)
consoleOutputLabel=Label(outputFrame, text ="Starting system monitoring...", height=3)
consoleOutputLabel.config(fg="white", bg="black", font=("Arial", 9))
consoleOutputLabel.grid(column=0, row=0)

imageFrameConfiguration()
imageThresholdConfiguration()

scanner = 1
velocityGraphic = Figure(figsize=(2.55, 2.2), dpi=100)
axVelocity = velocityGraphic.add_subplot(1, 1, 1)
xsVelocity = []
ysVelocity = []
def animateVelocity(index, xs, ys):
    global scanner
    temp_c = random.randint(0, 100)
    xs.append(str(scanner))
    scanner += 1
    ys.append(temp_c)
    xs = xs[-50:]
    ys = ys[-50:]
    axVelocity.clear()
    axVelocity.set_xticklabels([])
    axVelocity.plot(xs, ys, color="red", label="Velocity")
    axVelocity.legend()

directionGraphic = Figure(figsize=(2.56, 2.2), dpi=100)
axDirection = directionGraphic.add_subplot(1, 1, 1)
xsDirection = []
ysDirection = []
def animateDirection(index, xs, ys):
    temp_c = random.randint(0, 180)
    xs.append(str(scanner))
    ys.append(temp_c)
    xs = xs[-50:]
    ys = ys[-50:]
    axDirection.clear()
    axDirection.set_xticklabels([])
    axDirection.plot(xs, ys, color="blue", label="Direction")
    axDirection.legend()

CPUUsageGraphic = Figure(figsize=(2.55, 1.75), dpi=100)
axCPU = CPUUsageGraphic.add_subplot(1, 1, 1)
xsCPU = []
ysCPU = []
def animateCPU(index, xs, ys):
    temp_c = random.randint(0, 100)
    xs.append(str(scanner))
    ys.append(temp_c)
    xs = xs[-50:]
    ys = ys[-50:]
    axCPU.clear()
    axCPU.set_xticklabels([])
    axCPU.plot(xs, ys, color="red", label="CPU usage")
    axCPU.legend()

RAMUsageGraphic = Figure(figsize=(2.56, 1.75), dpi=100)
axRAM = RAMUsageGraphic.add_subplot(1, 1, 1)
xsRAM = []
ysRAM = []
def animateRAM(index, xs, ys):
    temp_c = random.randint(0, 100)
    xs.append(str(scanner))
    ys.append(temp_c)
    xs = xs[-50:]
    ys = ys[-50:]
    axRAM.clear()
    axRAM.set_xticklabels([])
    axRAM.plot(xs, ys, color="blue", label="RAM usage")
    axRAM.legend()

velocityCanvas = FigureCanvasTkAgg(velocityGraphic, velocityGraphicFrame)
velocityCanvas.draw()
velocityCanvas._tkcanvas.grid(column=0, row=0)
velocityAnimation = animation.FuncAnimation(velocityGraphic, animateVelocity, fargs=(xsVelocity, ysVelocity), interval=2000)

directionCanvas = FigureCanvasTkAgg(directionGraphic, directionGraphicFrame)
directionCanvas.draw()
directionCanvas._tkcanvas.grid(column=0, row=0)
directionAnimation = animation.FuncAnimation(directionGraphic, animateDirection, fargs=(xsDirection, ysDirection), interval=2000)

CPUCanvas = FigureCanvasTkAgg(CPUUsageGraphic, CPUUsageGraphicFrame)
CPUCanvas.draw()
CPUCanvas._tkcanvas.grid(column=0, row=0)
CPUAnimation = animation.FuncAnimation(CPUUsageGraphic, animateCPU, fargs=(xsCPU, ysCPU), interval=2000)

RAMCanvas = FigureCanvasTkAgg(RAMUsageGraphic, RAMUsageGraphicFrame)
RAMCanvas.draw()
RAMCanvas._tkcanvas.grid(column=0, row=0)
RAMAnimation = animation.FuncAnimation(RAMUsageGraphic, animateRAM, fargs=(xsRAM, ysRAM), interval=2000)

graphicInterface.mainloop()