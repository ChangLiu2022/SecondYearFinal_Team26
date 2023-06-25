import numpy as np
import cv2
import math

image = np.zeros((480,640,3), np.uint8)

while True:
    print('?:')
    if(input() == "0"):
        break
    print('x:')
    x = int(input(),16)
    print('y:')
    y = int(input(),16)

    print(str(x)+", "+str(y))
    image = cv2.circle(image, (x,y), radius=5, color=(0, 255, 255), thickness=-1)

cv2.imwrite("output_images/test.jpeg", image)

# show the result
cv2.imshow("result", image)
cv2.waitKey(0)
cv2.destroyAllWindows()