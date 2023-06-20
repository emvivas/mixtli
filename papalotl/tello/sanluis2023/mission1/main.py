#Program: Autonomous DJI Tello Drone Line Follower Mission 1 San Luis 2023
#Version: 1.0
#Developer: Emiliano Vivas Rodríguez
#Contact: a01424732@tec.mx
#Since: 2023/05/20  


import cv2, time, numpy, sys, threading
from djitellopy import Tello


class Cronometer(threading.Thread):

    def __init__(self):
        super().__init__()
        self._seconds = 0
        self._end = False

    def run(self):
            while not self._end:
                time.sleep(1)
                self._seconds+=1

    def getSeconds(self):
        return self._seconds

    def end(self):
        self._end = True


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
    dilation = cv2.dilate(blur, kernel, iterations=2)
    return dilation

def getSensorOutput(threshold):
    global frame
    centroid, topPoint = None, None
    if cv2.countNonZero(threshold) > THRESHOLD_PERCENTAGE*frame.shape[0]*frame.shape[1]:
        MOMENT = cv2.moments(threshold)
        if MOMENT["m00"] != 0:
            centroid = {'x': int(MOMENT["m10"] / MOMENT["m00"]), 'y': int(MOMENT["m01"] / MOMENT["m00"])}
            contours, hierarchy = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            borderObject = max(contours, key = cv2.contourArea)
            cv2.drawContours(frame, borderObject, -1, (255, 0, 255), 5)
            x, y, w, h = cv2.boundingRect(borderObject)
            topPoint = {'x': centroid['x'], 'y': centroid['y']+h//2}
    return {"centroid":centroid, "topPoint":topPoint}

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
    return {"velocity":VELOCITY[middle], "steering":STEERING_ANGLES[middle], "direction": DIRECTION_ANGLES[middle]}

def sendCommands(lineSensorOutput, goalSensorOutput):
    global velocity, steering, direction, recovery, successful
    if goalSensorOutput["centroid"] and CRONOMETER.getSeconds() > 10:
        successful = True
        return
    elif lineSensorOutput["centroid"]:
        recovery = False
        parameters = getParameters(lineSensorOutput["topPoint"])
        velocity = parameters["velocity"]
        steering = parameters["steering"]
        direction = parameters["direction"]
    else:
        if not recovery:
            recovery, velocity, direction, direction = True, 0, 0, 0
            steering = MAXIMUM_STEERING_ANGLE if direction < LINEAL_ANGLE_STEERING else MINIMUM_STEERING_ANGLE
    #print(steering, velocity)
    print("Drone battery: ", TELLO.get_battery(), "%", sep='\u0000')
    TELLO.send_rc_control(direction, velocity, 0, steering)
    time.sleep(1)

if __name__ == '__main__':
    SENSORS, WIDTH, HEIGHT, MINIMUM_VELOCITY, MAXIMUM_VELOCITY, LINEAL_ANGLE_STEERING, MINIMUM_STEERING_ANGLE, MAXIMUM_STEERING_ANGLE, LINEAL_ANGLE_DIRECTION, MINIMUM_DIRECTION_ANGLE, MAXIMUM_DIRECTION_ANGLE, THRESHOLD_PERCENTAGE = 15, 480, 360, 10, 15, 0, -7, 7, 0, -15, 15, 0.05
    STEERING_ANGLES = list(map(lambda angle: round(angle), numpy.linspace(start=MINIMUM_STEERING_ANGLE, stop=MAXIMUM_STEERING_ANGLE, num=SENSORS)))
    DIRECTION_ANGLES = list(map(lambda angle: round(angle), numpy.linspace(start=MINIMUM_DIRECTION_ANGLE, stop=MAXIMUM_DIRECTION_ANGLE, num=SENSORS)))
    VELOCITY = list(map(lambda velocity: round(velocity), numpy.linspace(start=MINIMUM_VELOCITY, stop=MAXIMUM_VELOCITY, num=SENSORS//2)))
    VELOCITY = VELOCITY + [VELOCITY[-1]] + list(reversed(VELOCITY))
    CRONOMETER = Cronometer()

    TELLO = Tello()
    TELLO.connect()
    time.sleep(0.2)
    TELLO.streamon()
    time.sleep(0.2)
    CAPTURE = TELLO.get_frame_read()

    #CAPTURE = cv2.VideoCapture(0)
    velocity, steering, direction, recovery, successful, frame = MAXIMUM_VELOCITY, LINEAL_ANGLE_STEERING, LINEAL_ANGLE_DIRECTION, False, False, None
    lineColor = colorPicker("TEC Mixtli Team | Line HSV Color Configuration")
    goalColor = colorPicker("TEC Mixtli Team | Goal HSV Color Configuration")
    CRONOMETER.start()
    image = CAPTURE.frame
    time.sleep(3)
    if image.any():
        TELLO.takeoff()
        time.sleep(0.2)
        TELLO.send_rc_control(0, 0, 0, 0)

        while True:
            image = CAPTURE.frame
            #rtf, image = CAPTURE.read()
            frame = cv2.resize(image, (WIDTH, HEIGHT))
            lineThreshold = thresholding(lineColor)
            goalThreshold = thresholding(goalColor)
            lineSensorOutput = getSensorOutput(lineThreshold)
            goalSensorOutput = getSensorOutput(goalThreshold)
            sendCommands(lineSensorOutput, goalSensorOutput)
            if goalSensorOutput["centroid"]:
                cv2.circle(frame,(goalSensorOutput["centroid"]['x'],goalSensorOutput["centroid"]['y']),7,(0,255,0),-1)
            if lineSensorOutput["centroid"]:
                cv2.circle(frame,(lineSensorOutput["centroid"]['x'],lineSensorOutput["centroid"]['y']),7,(0,0,255),-1)
                cv2.line(frame, (lineSensorOutput["centroid"]['x'], lineSensorOutput["centroid"]['y']), (lineSensorOutput["topPoint"]['x'], lineSensorOutput["topPoint"]['y']), (0, 255, 255), 2)
                cv2.circle(frame, (lineSensorOutput["topPoint"]['x'], lineSensorOutput["topPoint"]['y']), 7, (255,255,0), -1)
            cv2.imshow("TEC Mixtli Team | Frame Thresholding", frame)
            hStack = numpy.hstack([lineThreshold, goalThreshold])
            cv2.imshow("TEC Autonomous Mechatronics Grand Prix. DJI Tello Drone Computer Vision", hStack)
            if (cv2.waitKey(1) & 0xFF == ord('\u001B')) or successful:
                TELLO.send_rc_control(0, 0, 0, 0)
                time.sleep(0.2)
                TELLO.land()
                break
            elif (cv2.waitKey(1) & 0xFF == ord('\u0009')):
                TELLO.emergency()
                break
    CRONOMETER.end()
    CRONOMETER.join()

    #TELLO.end()

    cv2.destroyAllWindows()
    sys.exit(0)