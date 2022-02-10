from imutils import paths
import numpy as np
import imutils
import cv2

KNOWN_DISTANCE = 24.0
KNOWN_WIDTH = 11.0

def find_marker(image):
    #Converts image to grayscale, blurs it, and detects edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)
    #find contours in edged image
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key = cv2.contourArea)
    return cv2.minAreaRect(c)

def distance_to_camera(knownWidth, focalLength, perWidth):
    return (knownWidth * focalLength) / perWidth

image = cv2.imread("images/test.jpg")
marker = find_marker(image)
focalLength = (marker[1][0] * KNOWN_DISTANCE)/KNOWN_WIDTH

for imagePath in sorted(paths.list_images("images")):
    image = cv2.imread(imagePath)
    marker = find_marker(image)
    inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])

    #draw bounding box
    box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
    box = np.int0(box)
    cv2.drawContours(image, [box], -1, (0,255,0), 2)
    cv2.putText(image, "%.2fft" % (inches/12),
    (image.shape[1] -200, image.shape[0] -20), cv2.FONT_HERSHEY_SIMPLEX,
    2.0, (0,255,0), 3)
    cv2.imshow("image", image)
    cv2.waitKey(0)
