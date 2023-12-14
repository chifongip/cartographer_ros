import cv2
import numpy as np

img = cv2.imread("C:/Users/oscar.ip/Desktop/maxbot/maps/corridor/mymap.pgm")

# imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

kernel = np.ones((3, 3), np.uint8)
opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

cv2.imshow("img", opening)


cv2.waitKey(0)
 

cv2.destroyAllWindows()