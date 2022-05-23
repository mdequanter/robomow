import cv2
import numpy as np

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver



def rescale_frame(frame, percent=50):
    width = int(frame.shape[1] * percent / 100)
    height = int(frame.shape[0] * percent / 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

cap = cv2.VideoCapture("grass.mp4")
#cap = cv2.VideoCapture(3)


def detectObstacleInGrass(frame) :
    percentage = 0
    # Make a copy to draw bounding box
    frame_copy = frame.copy()
    blur = cv2.GaussianBlur(frame_copy, (7, 7), cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV_FULL)
    # Threshold of blue in HSV space
    lower_grass = np.array([42, 0, 0])
    upper_grass = np.array([106, 255, 255])
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_grass, upper_grass)
    result = cv2.bitwise_not(mask)
    ratioGrass = (cv2.countNonZero(result) / (frame.size / 12))*100
    frame_copy = cv2.copyMakeBorder(frame_copy, 2, 2, 2, 2, cv2.BORDER_CONSTANT, None, value=0)
    if (ratioGrass > 5.00) :
        resultFrame = cv2.putText(frame_copy, "OBSTACLE ",(50,50), cv2.FONT_HERSHEY_SIMPLEX,1, (0,0,255), 2, cv2.LINE_AA)
    else :
        resultFrame = frame_copy

    return resultFrame, ratioGrass




while (True):
    _, frame = cap.read()
    # It converts the BGR color space of image to HSV color space
    frame = rescale_frame(frame, 30)

    h, w, c = frame.shape

    cropped_topLeft = frame[0:int(h/2), 0:int(w/2)]
    cropped_bottomLeft = frame[int(h/2):h, 0:int(w/2)]
    cropped_topRight = frame[0:int(h/2), int(w/2):w]
    cropped_bottomRight = frame[int(h/2):h, int(w/2):w]

    topLeft, ratioGrassTL =  detectObstacleInGrass(cropped_topLeft)
    topRight, ratioGrassTR =  detectObstacleInGrass(cropped_topRight)
    bottomLeft, ratioGrassBT =  detectObstacleInGrass(cropped_bottomLeft)
    bottomRight, ratioGrassBR =  detectObstacleInGrass(cropped_bottomRight)

    imgStack = stackImages(1,([topLeft,topRight],[bottomLeft,bottomRight]))
    cv2.imshow("Stacked Images", imgStack)

    cv2.waitKey(0)

cv2.destroyAllWindows()
cap.release()