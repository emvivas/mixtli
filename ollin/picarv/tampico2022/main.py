#Program: Autonomous SunFounder Picar-V Line Follower Tampico 2022
#Version: 1.0
#Developer: Emiliano Vivas Rodríguez
#Contact: a01424732@tec.mx
#Since: 2022/10/10


import picar, os, psutil, time, cv2, numpy#, imutils, matplotlib, matplotlib.animation as animation
from picar import front_wheels, back_wheels
from picar.SunFounder_PCA9685 import Servo
#from PIL import Image, ImageTk
#from tkinter import Tk, Frame, Label, Button, Canvas
#from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
#from matplotlib.figure import Figure
#matplotlib.use('TkAgg')

def thresholding(image, HSVValues, mode):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.dilate(hsv, (5, 5), iterations = 1)
    #hsv = cv2.morphologyEx(hsv, cv2.MORPH_CLOSE, (3, 3))
    lowerWhite = numpy.array([HSVValues[0], HSVValues[1], HSVValues[2]])
    upperWhite = numpy.array([HSVValues[3], HSVValues[4], HSVValues[5]])
    mask = cv2.inRange(hsv, lowerWhite, upperWhite)
    """
    if mode:
        contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if(len(contours) > 0):
            biggest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(biggest)
            xCenter = x + w//2
            yCenter = y + h//2
            cv2.drawContours(img, biggest, -1, (255, 0, 255), 5)
            cv2.circle(img, (xCenter, yCenter), 10, (0, 255, 0), cv2.FILLED)
    """
    return mask

def getSensorOutput(imageThreshold):
    global sensors
    images = numpy.hsplit(imageThreshold, sensors)
    totalPixels = img.shape[1]//sensors * img.shape[0]
    sensorsOutput = []
    for index, image in enumerate(images):
        pixelCount = cv2.countNonZero(image)
        if pixelCount > threshold*totalPixels:
            sensorsOutput.append(1)
        else:
            sensorsOutput.append(0)
    return sensorsOutput

def sendCommands(sensorsOutput, sensorsOutputBorder):
    global rotation, velocity, panServoRotation, recovery
    bw.speed = forwardSpeed
    velocity = forwardSpeed
    if sensorsOutput == [0, 0, 0, 0, 0]:
        bw.speed = 60
        velocity = 60
        
        
        if sensorsOutputBorder == [0, 0, 0, 0, 0]:
            fw.turn(rotationDegrees[rotation])
            pan_servo.write(rotationDegrees[panServoRotation])
            if panServoRotation==0:
                recovery = 1
            elif panServoRotation==8:
                recovery = -1
            panServoRotation += recovery
            time.sleep(0.25)
            return 0;
        elif sensorsOutputBorder == [0, 0, 0, 0, 1]:
            bw.speed = 80
            velocity = 80
            rotation = 2
        elif sensorsOutputBorder == [0, 0, 0, 1, 0]:
            rotation = 1
        elif sensorsOutputBorder == [0, 0, 0, 1, 1]:
            bw.speed = 80
            velocity = 80
            rotation = 1
        elif sensorsOutputBorder == [0, 0, 1, 0, 0]:
            rotation = 0 if rotation<4 else 8
        elif sensorsOutputBorder == [0, 0, 1, 0, 1]:
            rotation = 0
        elif sensorsOutputBorder == [0, 0, 1, 1, 0]:
            rotation = 0
        elif sensorsOutputBorder == [0, 0, 1, 1, 1]:
            bw.speed = 80
            velocity = 80
            rotation = 0
        elif sensorsOutputBorder == [0, 1, 0, 0, 0]:
            rotation = 7
        elif sensorsOutputBorder == [0, 1, 0, 0, 1]:
            bw.speed = 80
            velocity = 80
            rotation = 2
        elif sensorsOutputBorder == [0, 1, 0, 1, 0]:
            rotation = 0 if rotation<4 else 8
        elif sensorsOutputBorder == [0, 1, 0, 1, 1]:
            rotation = 1
        elif sensorsOutputBorder == [0, 1, 1, 0, 0]:
            rotation = 8
        elif sensorsOutputBorder == [0, 1, 1, 0, 1]:
            rotation = 8
        elif sensorsOutputBorder == [0, 1, 1, 1, 0]:
            rotation = 0 if rotation<4 else 8
        elif sensorsOutputBorder == [0, 1, 1, 1, 1]:
            rotation = 0
        elif sensorsOutputBorder == [1, 0, 0, 0, 0]:
            bw.speed = 80
            velocity = 80
            rotation = 6
        elif sensorsOutputBorder == [1, 0, 0, 0, 1]:
            rotation = 0 if rotation<4 else 8
        elif sensorsOutputBorder == [1, 0, 0, 1, 0]:
            bw.speed = 80
            velocity = 80
            rotation = 6
        elif sensorsOutputBorder == [1, 0, 0, 1, 1]:
            rotation = 5
        elif sensorsOutputBorder == [1, 0, 1, 0, 0]:
            rotation = 8
        elif sensorsOutputBorder == [1, 0, 1, 0, 1]:
            rotation = 0 if rotation<4 else 8
        elif sensorsOutputBorder == [1, 0, 1, 1, 0]:
            rotation = 8
        elif sensorsOutputBorder == [1, 0, 1, 1, 1]:
            bw.speed = 80
            velocity = 80
            rotation = 6
        elif sensorsOutputBorder == [1, 1, 0, 0, 0]:
            bw.speed = 80
            velocity = 80
            rotation = 7
        elif sensorsOutputBorder == [1, 1, 0, 0, 1]:
            rotation = 8
        elif sensorsOutputBorder == [1, 1, 0, 1, 0]:
            rotation = 7
        elif sensorsOutputBorder == [1, 1, 0, 1, 1]:
            rotation = 0 if rotation<4 else 8
        elif sensorsOutputBorder == [1, 1, 1, 0, 0]:
            bw.speed = 80
            velocity = 80
            rotation = 8
        elif sensorsOutputBorder == [1, 1, 1, 0, 1]:
            bw.speed = 80
            velocity = 80
            rotation = 2
        elif sensorsOutputBorder == [1, 1, 1, 1, 0]:
            rotation = 8
        elif sensorsOutputBorder == [1, 1, 1, 1, 1]:
            rotation = 0 if rotation<4 else 8
        
        
    elif sensorsOutput == [0, 0, 0, 0, 1]:
        rotation = 8
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 0, 0, 1, 0]:
        rotation = 7
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 0, 0, 1, 1]:
        rotation = 8
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 0, 1, 0, 0]:
        rotation = 4
    elif sensorsOutput == [0, 0, 1, 0, 1]:
        rotation = 6
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 0, 1, 1, 0]:
        rotation = 4#
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 0, 1, 1, 1]:
        rotation = 6
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 1, 0, 0, 0]:
        rotation = 1
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 1, 0, 0, 1]:
        rotation = 5
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 1, 0, 1, 0]:
        rotation = 4
    elif sensorsOutput == [0, 1, 0, 1, 1]:
        rotation = 5
    elif sensorsOutput == [0, 1, 1, 0, 0]:
        rotation = 4#
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [0, 1, 1, 0, 1]:
        rotation = 5
    elif sensorsOutput == [0, 1, 1, 1, 0]:
        rotation = 4
    elif sensorsOutput == [0, 1, 1, 1, 1]:
        rotation = 5
    elif sensorsOutput == [1, 0, 0, 0, 0]:
        rotation = 0
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [1, 0, 0, 0, 1]:
        rotation = 4
    elif sensorsOutput == [1, 0, 0, 1, 0]:
        rotation = 3
    elif sensorsOutput == [1, 0, 0, 1, 1]:
        rotation = 5
    elif sensorsOutput == [1, 0, 1, 0, 0]:
        rotation = 2
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [1, 0, 1, 0, 1]:
        rotation = 4
    elif sensorsOutput == [1, 0, 1, 1, 0]:
        rotation = 3
    elif sensorsOutput == [1, 0, 1, 1, 1]:
        rotation = 5
    elif sensorsOutput == [1, 1, 0, 0, 0]:
        rotation = 0
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [1, 1, 0, 0, 1]:
        rotation = 3
    elif sensorsOutput == [1, 1, 0, 1, 0]:
        rotation = 3
    elif sensorsOutput == [1, 1, 0, 1, 1]:
        rotation = 4
    elif sensorsOutput == [1, 1, 1, 0, 0]:
        rotation = 2
        bw.speed = spinSpeed
        velocity = spinSpeed
    elif sensorsOutput == [1, 1, 1, 0, 1]:
        rotation = 3
    elif sensorsOutput == [1, 1, 1, 1, 0]:
        rotation = 3
    elif sensorsOutput == [1, 1, 1, 1, 1]:
        rotation = 4
    fw.turn(rotationDegrees[rotation])
    pan_servo.write(cameraRotation[rotation])
    
def empty(a):
    pass

def colorPicker(title):
    HSVValues = None
    cv2.namedWindow(title)
    cv2.resizeWindow(title, 640, 240)
    cv2.createTrackbar("Hue minimum", title, 0, 179, empty)
    cv2.createTrackbar("Hue maximum", title, 179, 179, empty)
    cv2.createTrackbar("Saturation minimum", title, 0, 255, empty)
    cv2.createTrackbar("Saturation maximum", title, 255, 255, empty)
    cv2.createTrackbar("Value minimum", title, 0, 255, empty)
    cv2.createTrackbar("Value maximum", title, 255, 255, empty)
    while True:
        rtf, img = capture.read()
        img = cv2.resize(img, (width, height))
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        hueMinimum = cv2.getTrackbarPos("Hue minimum", title)
        hueMaximum = cv2.getTrackbarPos("Hue maximum", title)
        saturationMinimum = cv2.getTrackbarPos("Saturation minimum", title)
        saturationMaximum = cv2.getTrackbarPos("Saturation maximum", title)
        valueMinimum = cv2.getTrackbarPos("Value minimum", title)
        valueMaximum = cv2.getTrackbarPos("Value maximum", title)
        lower = numpy.array([hueMinimum, saturationMinimum, valueMinimum])
        upper = numpy.array([hueMaximum, saturationMaximum, valueMaximum])
        mask = cv2.inRange(imgHSV, lower, upper)
        result = cv2.bitwise_and(img, img, mask=mask)
        valuesHSV = [hueMinimum, saturationMinimum, valueMinimum, hueMaximum, saturationMaximum, valueMaximum]
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        hStack = numpy.hstack([img, mask, result])
        cv2.imshow('Horizontal stacking', hStack)
        if cv2.waitKey(1) & 0xFF == ord('\u001B'):
            break
    cv2.destroyAllWindows()
    return valuesHSV

capture = cv2.VideoCapture(0)
HSVValuesLine = None
HSVValuesBorder = None
sensors = 5
threshold = 0.05
width, height = 480, 360
rotationDegrees = numpy.linspace(start=0, stop=180, num=sensors*2-1)
cameraRotation = numpy.linspace(start=115, stop=65, num=sensors*2-1)
rotation = 4
forwardSpeed = 100
spinSpeed = 85
img = None
imageThreshold = None
panServoRotation = 0
recovery = 1
picar.setup()
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
pan_servo = Servo.Servo(1)
tilt_servo = Servo.Servo(2)
fw.offset = 10
pan_servo.offset = 10
tilt_servo.offset = 10
fw.turn(90)
pan_servo.write(90)
tilt_servo.write(0)
bw.speed = 0
velocity = 0
HSVValuesLine = colorPicker("Line HSV color configuration")
HSVValuesBorder = colorPicker("Border HSV color configuration")
bw.backward()

while True:
    rtf, img = capture.read()
    if rtf: 
        imageThreshold = thresholding(img, HSVValuesLine, False)
        imageThresholdBorder = thresholding(img, HSVValuesBorder, False)
        sensorsOutput = getSensorOutput(imageThreshold)
        sensorsOutputBorder = getSensorOutput(imageThresholdBorder)
        sendCommands(sensorsOutput, sensorsOutputBorder)
        #cv2.imshow("Image", img)
        #cv2.imshow("Threshold line image", imageThreshold)
        #cv2.imshow("Threshold border image", imageThresholdBorder)
        if cv2.waitKey(1) & 0xFF == ord('\u001B'):
            break
        time.sleep(0.01)

pan_servo.write(90)
tilt_servo.write(0)
bw.speed = 0
bw.stop()
capture.release()
cv2.destroyAllWindows()

"""
def imageFrameConfiguration():
    global capture, imageThreshold, img
    rtf, img = capture.read()
    if rtf:
        video = imutils.resize(img, width=510, height=360)
        imageThreshold2 = thresholding(video, HSVValuesLine, False)
        imageThreshold = thresholding(img, HSVValuesLine, True)
        imageThresholdBorder = thresholding(img, HSVValuesBorder, True)
        
        sensorsOutput = getSensorOutput(imageThreshold)
        sensorsOutputBorder = getSensorOutput(imageThresholdBorder)
        sendCommands(sensorsOutput, sensorsOutputBorder)
        
        video = imutils.resize(img, width=510, height=360)
        visualization = Image.fromarray(cv2.cvtColor(video, cv2.COLOR_BGR2RGB))
        visual = ImageTk.PhotoImage(image=visualization)
        originalImageLabel.configure(image=visual)
        originalImageLabel.image = visual
        
        visualizationImageThreshold = Image.fromarray(imageThreshold2)
        visualImageThreshold = ImageTk.PhotoImage(image=visualizationImageThreshold)
        imageThresholdLabel.configure(image=visualImageThreshold)
        imageThresholdLabel.image = visualImageThreshold
        originalImageLabel.after(15, imageFrameConfiguration)

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

scanner = 1
velocityGraphic = Figure(figsize=(2.55, 2.2), dpi=100)
axVelocity = velocityGraphic.add_subplot(1, 1, 1)
xsVelocity = []
ysVelocity = []
def animateVelocity(index, xs, ys):
    global scanner
    xs.append(str(scanner))
    scanner += 1
    ys.append(velocity)
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
    xs.append(str(scanner))
    ys.append(rotationDegrees[rotation])
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
    xs.append(str(scanner))
    ys.append(psutil.cpu_percent())
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
    xs.append(str(scanner))
    ys.append(psutil.virtual_memory()[2])
    xs = xs[-50:]
    ys = ys[-50:]
    axRAM.clear()
    axRAM.set_xticklabels([])
    axRAM.plot(xs, ys, color="blue", label="RAM usage")
    axRAM.legend()

velocityCanvas = FigureCanvasTkAgg(velocityGraphic, velocityGraphicFrame)
velocityCanvas.draw()
velocityCanvas._tkcanvas.grid(column=0, row=0)
#velocityAnimation = animation.FuncAnimation(velocityGraphic, animateVelocity, fargs=(xsVelocity, ysVelocity), interval=10000)

directionCanvas = FigureCanvasTkAgg(directionGraphic, directionGraphicFrame)
directionCanvas.draw()
directionCanvas._tkcanvas.grid(column=0, row=0)
#directionAnimation = animation.FuncAnimation(directionGraphic, animateDirection, fargs=(xsDirection, ysDirection), interval=10000)

CPUCanvas = FigureCanvasTkAgg(CPUUsageGraphic, CPUUsageGraphicFrame)
CPUCanvas.draw()
CPUCanvas._tkcanvas.grid(column=0, row=0)
#CPUAnimation = animation.FuncAnimation(CPUUsageGraphic, animateCPU, fargs=(xsCPU, ysCPU), interval=10000)

RAMCanvas = FigureCanvasTkAgg(RAMUsageGraphic, RAMUsageGraphicFrame)
RAMCanvas.draw()
RAMCanvas._tkcanvas.grid(column=0, row=0)
#RAMAnimation = animation.FuncAnimation(RAMUsageGraphic, animateRAM, fargs=(xsRAM, ysRAM), interval=10000)

#imageFrameConfiguration()
#graphicInterface.mainloop()
"""
