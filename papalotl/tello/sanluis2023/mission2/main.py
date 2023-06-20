#Program: Autonomous DJI Tello Drone Line Follower Mission 2 San Luis 2023
#Version: 1.0
#Developer: Emiliano Vivas Rodríguez
#Contact: a01424732@tec.mx
#Since: 2023/05/20


import cv2, numpy, sys, time
from djitellopy import Tello

def nothing(x):
   pass

def colorPicker(title):
    cv2.namedWindow(title)
    cv2.resizeWindow(title, 640, 240)
    cv2.createTrackbar("Hue min.", title, 0, 179, nothing)
    cv2.createTrackbar("Hue max.", title, 179, 179, nothing)
    cv2.createTrackbar("Saturation min.", title, 0, 255, nothing)
    cv2.createTrackbar("Saturation max.", title, 255, 255, nothing)
    cv2.createTrackbar("Value min.", title, 0, 255, nothing)
    cv2.createTrackbar("Value max.", title, 255, 255, nothing)
    while True:
        image = CAPTURE.frame
        #rtf, image = CAPTURE.read()
        image = cv2.resize(image, (WIDTH, HEIGHT))
        HSVImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        minimumHue = cv2.getTrackbarPos("Hue min.", title)
        maximumHue = cv2.getTrackbarPos("Hue max.", title)
        minimumSaturation = cv2.getTrackbarPos("Saturation min.", title)
        saturationMaximum = cv2.getTrackbarPos("Saturation max.", title)
        minimumValue = cv2.getTrackbarPos("Value min.", title)
        maximumValue = cv2.getTrackbarPos("Value max.", title)
        lowerColor = numpy.array([minimumHue, minimumSaturation, minimumValue])
        upperColor = numpy.array([maximumHue, saturationMaximum, maximumValue])
        mask = cv2.inRange(HSVImage, lowerColor, upperColor)
        result = cv2.bitwise_and(image, image, mask=mask)
        HSVValues = [minimumHue, minimumSaturation, minimumValue, maximumHue, saturationMaximum, maximumValue]
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        hStack = numpy.hstack([image, mask, result])
        cv2.imshow("TEC Autonomous Mechatronics Grand Prix. DJI Tello Drone Control Panel", hStack)
        if cv2.waitKey(1) & 0xFF == ord('\u001B'):
            break
    cv2.destroyAllWindows()
    return HSVValues

def thresholding(color):
    global frame
    lowerColor=numpy.array([color[0],color[1],color[2]])
    upperColor=numpy.array([color[3],color[4],color[5]])
    HSVImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(HSVImage, lowerColor, upperColor)
    kernel = numpy.ones((7, 7), numpy.uint8)
    sharpenKernel = numpy.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(mask, -1, sharpenKernel)
    _, thresh = cv2.threshold(sharpen, 50, 255, cv2.THRESH_BINARY)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    blur = cv2.medianBlur(closing, 5)
    dilation = cv2.dilate(blur, kernel)
    erosion = cv2.erode(dilation, kernel, iterations=2)
    return erosion

def getSensorOutput(threshold):
    global frame, selectedOption, selectedFigure
    centroid, selectedFigureCentroid = None, None
    if cv2.countNonZero(threshold) > THRESHOLD_PERCENTAGE*frame.shape[0]*frame.shape[1]:
        MOMENT = cv2.moments(threshold)
        if MOMENT["m00"] != 0:
            figure = None
            figureDetection = {"triangle":0, "parallelogram":0, "hexagon":0, "circle":0, "other":0}
            borders = []
            centroid = {'x': int(MOMENT["m10"] / MOMENT["m00"]), 'y': int(MOMENT["m01"] / MOMENT["m00"])}
            contours, hierarchy = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            borderObjects = sorted(contours, key = cv2.contourArea, reverse=True)[:3]

            for borderObject in borderObjects:
                epsilon = FIGURE_ACCURACY * cv2.arcLength(borderObject, True)
                approx = cv2.approxPolyDP(borderObject, epsilon = epsilon, closed=True)
                cv2.drawContours(frame, approx, -1, (255, 0, 255), 10)
                x, y, w, h = cv2.boundingRect(approx)
                match len(approx):
                    case 3:
                        figure = "Triangle"
                        figureDetection["triangle"]+=1
                    case 4:
                        figure = "Parallelogram"
                        figureDetection["parallelogram"]+=1
                    case 6:
                        figure = "Hexagon"
                        figureDetection["hexagon"]+=1
                    case _:
                        if len(approx) > 6:
                            figure = "Circle"
                            figureDetection["circle"]+=1
                        else:
                            figure = "Other"
                            figureDetection["other"]+=1

                cv2.putText(frame, figure, (x, y-5), 1, 1, (0, 255, 0), 1)
                borders.append({"figure":figure, 'x':x, 'y':y, 'w':w, 'h':h})

            figures = [border for border in borders if border["figure"]=="Parallelogram"]

            if selectedOption:
                match figureDetection["parallelogram"]:
                    case 1:
                        selectedFigure = figures[0]
                        selectedOption = None
                    case 2:
                        figures = sorted(figures, key=lambda border: border[selectedOption["coordinate"]], reverse=False)[:2]
                        selectedFigure = figures[selectedOption["direction"]]
                    case 3:
                        figures = sorted(figures, key=lambda border: border[selectedOption["coordinate"]], reverse=False)[1:3]
                        selectedFigure = figures[selectedOption["direction"]]
            else:
                if sum(figureDetection.values())==3 and figureDetection["parallelogram"]>=2 and figureDetection["other"]==0:
                    if figureDetection["parallelogram"]==3:
                        selectedOption = PARALLELOGRAM_OPTION
                        figures = sorted(figures, key=lambda border: border[selectedOption["coordinate"]], reverse=False)[1:3]
                    elif figureDetection["parallelogram"]==2:
                        if figureDetection["circle"]:
                            selectedOption = CIRCLE_OPTION
                        elif figureDetection["hexagon"]:
                            selectedOption = HEXAGON_OPTION
                        elif figureDetection["triangle"]:
                            selectedOption = TRIANGLE_OPTION
                        figures = sorted(figures, key=lambda border: border[selectedOption["coordinate"]], reverse=False)[:2]
                    selectedFigure = figures[selectedOption["direction"]]
                else:
                    print("Recovery")
                    selectedOption = None
                    selectedFigure = None
            
            if selectedFigure:
                selectedFigureCentroid = {'y': int(selectedFigure['y']+selectedFigure['h']/2), 'x': int(selectedFigure['x']+selectedFigure['w']/2)}
                cv2.rectangle(frame, (selectedFigure['x'], selectedFigure['y']), (selectedFigure['x'] + selectedFigure['w'], selectedFigure['y'] + selectedFigure['h']), (255, 255, 255), 3)
                cv2.circle(frame,(selectedFigureCentroid['x'],selectedFigureCentroid['y']),7,(255,255,255),-1)
    return {"centroid":centroid, "selectedFigureCentroid":selectedFigureCentroid}

def getParameters(point):
    global frame
    velocity = {}
    coordinates = point.keys()
    for index, coordinate in enumerate(coordinates):
        high, middle, low = SENSORS, 0, 0
        while low <= high:
            auxiliarMiddle = (high + low) // 2
            if auxiliarMiddle != middle:
                middle = auxiliarMiddle
            else:
                break
            middleFrame = frame.shape[index] * middle / SENSORS
            if point[coordinate] > frame.shape[index] * low / SENSORS and point[coordinate] < middleFrame:
                high = middle
            elif point[coordinate] > middleFrame and point[coordinate] < frame.shape[index] * high / SENSORS:
                low = middle
            else:
                break
        velocity.update({coordinate: VELOCITY[middle] if index == 0 else -VELOCITY[middle]})
    return velocity

def sendCommands(windowSensorOutput):
    global velocity, recovery, successful
    if windowSensorOutput["selectedFigureCentroid"]:
        parameters = getParameters(windowSensorOutput["selectedFigureCentroid"])
        velocity['x'] = parameters['x']
        velocity['y'] = parameters['y']
        if velocity['x']==0 and velocity['y']==0:
            velocity['z'] = 50
            
    else:
        print("Noooo")
    print(velocity)
    TELLO.send_rc_control(velocity['x'], velocity['z'], velocity['y'], velocity['w'])

if __name__ == '__main__':
    SENSORS, WIDTH, HEIGHT, MINIMUM_VELOCITY, MAXIMUM_VELOCITY, THRESHOLD_PERCENTAGE = 5, 480, 360, -10, 10, 0.1
    VELOCITY = list(map(lambda angle: round(angle), numpy.linspace(start=MINIMUM_VELOCITY, stop=MAXIMUM_VELOCITY, num=SENSORS)))

    FIGURE_ACCURACY, GO_LEFT_UP, GO_RIGHT_DOWN = 0.03, 0, 1
    PARALLELOGRAM_OPTION, CIRCLE_OPTION, HEXAGON_OPTION, TRIANGLE_OPTION = {"coordinate": 'x', "direction": GO_RIGHT_DOWN}, {"coordinate": 'x', "direction": GO_LEFT_UP}, {"coordinate": 'y', "direction": GO_LEFT_UP}, {"coordinate": 'y', "direction": GO_RIGHT_DOWN} #0 - (Left, Up) | 1 - (Right, Down)
    CENTER = {'x': WIDTH//2, 'y': HEIGHT//2}
    
    TELLO = Tello()
    TELLO.connect()
    TELLO.send_rc_control(0, 0, 0, 0)
    print("Drone battery: ", TELLO.get_battery(), "%", sep='\u0000')
    TELLO.streamon()
    CAPTURE = TELLO.get_frame_read()
    
    #CAPTURE = cv2.VideoCapture(0)
    recovery, successful, frame = False, False, None
    velocity = {'w': 0, 'x': 0, 'y': 0, 'z': 0}
    selectedOption = None
    selectedFigure = None
    windowColor = colorPicker("TEC Mixtli Team | Line HSV Color Configuration")
    
    TELLO.takeoff()
    
    while(True):
        image = CAPTURE.frame
        #rtf, image = CAPTURE.read()
        frame = cv2.resize(image, (WIDTH, HEIGHT))
        windowThreshold = thresholding(windowColor)
        windowSensorOutput = getSensorOutput(windowThreshold)
        sendCommands(windowSensorOutput)
        """
        if windowSensorOutput["centroid"]:
            cv2.circle(frame,(windowSensorOutput["centroid"]['x'],windowSensorOutput["centroid"]['y']),7,(0,255,0),-1)
        """
        """
        if lineSensorOutput["centroid"]:
            cv2.circle(frame,(lineSensorOutput["centroid"]['x'],lineSensorOutput["centroid"]['y']),7,(0,0,255),-1)
            cv2.line(frame, (lineSensorOutput["centroid"]['x'], lineSensorOutput["centroid"]['y']), (lineSensorOutput["topPoint"]['x'], lineSensorOutput["topPoint"]['y']), (0, 255, 255), 2)
            cv2.circle(frame, (lineSensorOutput["topPoint"]['x'], lineSensorOutput["topPoint"]['y']), 7, (255,255,0), -1)
        """
        cv2.circle(frame, (CENTER['x'], CENTER['y']), 3, (255,255,0), -1)
        cv2.imshow("TEC Mixtli Team | Frame Thresholding", frame)
        hStack = numpy.hstack([windowThreshold])
        cv2.imshow("TEC Autonomous Mechatronics Grand Prix. DJI Tello Drone Computer Vision", hStack)
        if (cv2.waitKey(1) & 0xFF == ord('\u001B')) or successful:
            
            TELLO.send_rc_control(0, 0, 0, 0)
            TELLO.land()
            
            break
        elif (cv2.waitKey(1) & 0xFF == ord('\u0009')):    
            TELLO.emergency()
            break
        time.sleep(1)
    """
    TELLO.end()
    """
    TELLO.land()
    cv2.destroyAllWindows()
    sys.exit(0)