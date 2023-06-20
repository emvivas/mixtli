#Program: Autonomous SunFounder Picar-V Line Follower New Tampico 2022
#Version: 1.0
#Developer: Emiliano Vivas Rodríguez
#Contact: a01424732@tec.mx
#Since: 2023/05/20


import cv2, time, numpy, sys

import picar
from picar import front_wheels, back_wheels


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
        rtf, image = CAPTURE.read()
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
        cv2.imshow("TEC Autonomous Mechatronics Grand Prix. SunFounder Smart Video Car Kit V2.0 PiCar-V Robot Control Panel", hStack)
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
    dilation = cv2.dilate(blur, kernel, iterations=2)
    return dilation

def getSensorOutput(threshold):
    global frame
    centroid, middlePoint = None, None
    if cv2.countNonZero(threshold) > THRESHOLD_PERCENTAGE*frame.shape[0]*frame.shape[1]:
        MOMENT = cv2.moments(threshold)
        if MOMENT["m00"] != 0:
            borders = []
            centroid = {'x': int(MOMENT["m10"] / MOMENT["m00"]), 'y': int(MOMENT["m01"] / MOMENT["m00"])}
            contours, hierarchy = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            borderObjects = sorted(contours, key = cv2.contourArea, reverse=True)[:2]
            for borderObject in borderObjects:
                cv2.drawContours(frame, borderObject, -1, (255, 0, 255), 5)
                x, y, w, h = cv2.boundingRect(borderObject)
                borders.append({'x':x, 'y':y, 'w':w, 'h':h})
            if len(borders)==2:
                middlePoint = {'x': int((borders[0]['x']+borders[0]['w'] + borders[1]['x'])/2), 'y': int((borders[0]['y']+borders[0]['h']+borders[1]['y']+borders[1]['h'])/2)}
                cv2.line(frame, (borders[0]['x']+borders[0]['w'], borders[0]['y']+borders[0]['h']), (borders[1]['x'], borders[1]['y']+borders[1]['h']), (0, 255, 255), 2)
    return {"centroid":centroid, "middlePoint":middlePoint}

def getParameters(point):
    global frame
    high, middle, low = SENSORS, 0, 0
    while low <= high:
        auxiliarMiddle = (high + low) // 2
        if auxiliarMiddle != middle:
            middle = auxiliarMiddle
        else:
            break
        middleFrame = frame.shape[1] * middle / SENSORS
        if point['x'] > frame.shape[1] * low / SENSORS and point['x'] < middleFrame:
            high = middle
        elif point['x'] > middleFrame and point['x'] < frame.shape[1] * high / SENSORS:
            low = middle
        else:
            break
    return {"velocity":VELOCITY[middle], "steering":STEERING_ANGLES[middle]}

def sendCommands(borderSensorOutput, goalSensorOuput):
    global velocity, steering, recovery, goal, successful, frame
    if goalSensorOuput["centroid"]:
        recovery, goal = False, False
        parameters = getParameters(goalSensorOuput["centroid"])
        velocity = parameters["velocity"]
        steering = parameters["steering"]
    elif borderSensorOutput["middlePoint"]:
        recovery = False
        parameters = getParameters(borderSensorOutput["middlePoint"])
        velocity = parameters["velocity"]
        steering = parameters["steering"]
    elif borderSensorOutput["centroid"] and not goal:
        if not recovery:
            recovery, velocity = True, MINIMUM_VELOCITY
            if borderSensorOutput["centroid"]['x'] >= frame.shape[1] / 3:
                steering = MINIMUM_STEERING_ANGLE
            elif borderSensorOutput["centroid"]['x'] >= frame.shape[1] * 2 / 3:
                steering = MAXIMUM_STEERING_ANGLE if steering < LINEAL_ANGLE_STEERING else MINIMUM_STEERING_ANGLE
            else:
                steering = MAXIMUM_STEERING_ANGLE
    elif goal:
        successful = True
    else:
        if not recovery:
            recovery, velocity = True, MINIMUM_VELOCITY
            steering = MAXIMUM_STEERING_ANGLE if steering < LINEAL_ANGLE_STEERING else MINIMUM_STEERING_ANGLE
    
    #print(steering, velocity)
    fw.turn(steering)
    bw.speed = velocity


if __name__ == '__main__':
    SENSORS, WIDTH, HEIGHT, MINIMUM_VELOCITY, MAXIMUM_VELOCITY, LINEAL_ANGLE_STEERING, MINIMUM_STEERING_ANGLE, MAXIMUM_STEERING_ANGLE, THRESHOLD_PERCENTAGE = 180, 480, 360, 90, 100, 90, 0, 180, 0.075
    STEERING_ANGLES = list(map(lambda angle: round(angle), numpy.linspace(start=MINIMUM_STEERING_ANGLE, stop=MAXIMUM_STEERING_ANGLE, num=SENSORS)))
    VELOCITY = list(map(lambda velocity: round(velocity), numpy.linspace(start=MINIMUM_VELOCITY, stop=MAXIMUM_VELOCITY, num=SENSORS//2)))
    VELOCITY = VELOCITY + [VELOCITY[-1]] + list(reversed(VELOCITY))
    CAPTURE = cv2.VideoCapture(0)
    velocity, steering, recovery, goal, successful, frame = MAXIMUM_VELOCITY, LINEAL_ANGLE_STEERING, False, False, False, None
    borderColor = colorPicker("TEC Mixtli Team | Border HSV Color Configuration")
    goalColor = colorPicker("TEC Mixtli Team | Goal HSV Color Configuration")

    picar.setup()
    bw = back_wheels.Back_Wheels()
    fw = front_wheels.Front_Wheels()
    fw.offset = 10
    fw.turn(90)
    
    bw.speed = 0
    bw.backward()

    while(True):
        ret, frame = CAPTURE.read()
        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        borderThreshold = thresholding(borderColor)
        goalThreshold = thresholding(goalColor)
        borderSensorOutput = getSensorOutput(borderThreshold)
        goalSensorOuput = getSensorOutput(goalThreshold)
        sendCommands(borderSensorOutput, goalSensorOuput)
        """
        if goalSensorOuput["centroid"]:
            cv2.circle(frame,(goalSensorOuput["centroid"]['x'],goalSensorOuput["centroid"]['y']),7,(0,255,0),-1)
        if goalSensorOuput["middlePoint"]:
            cv2.circle(frame,(goalSensorOuput["middlePoint"]['x'],goalSensorOuput["middlePoint"]['y']),7,(0,255,0),-1)
        if borderSensorOutput["centroid"]:
            cv2.circle(frame,(borderSensorOutput["centroid"]['x'],borderSensorOutput["centroid"]['y']),7,(0,0,255),-1)
        if borderSensorOutput["middlePoint"]:
            cv2.circle(frame,(borderSensorOutput["middlePoint"]['x'],borderSensorOutput["middlePoint"]['y']),7,(0,0,255),-1)
        cv2.imshow("TEC Mixtli Team | Frame Thresholding", frame)
        """
        hStack = numpy.hstack([borderThreshold, goalThreshold])
        cv2.imshow("TEC Autonomous Mechatronics Grand Prix. SunFounder Smart Video Car Kit V2.0 PiCar-V Robot Computer Vision", hStack)
        if (cv2.waitKey(1) & 0xFF == ord('\u001B')) or successful:
            break
        time.sleep(0.01)
    if successful:
        time.sleep(0.5)
    
    bw.speed = 0
    bw.stop()

    cv2.destroyAllWindows()
    CAPTURE.release()
    sys.exit(0)
