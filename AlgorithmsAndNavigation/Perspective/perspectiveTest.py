import numpy as np
import cv2
import math

# read input
img = cv2.imread("images/image1.jpeg")
img2 = cv2.imread("images/image.jpeg")
print(img)
print(img.shape[:2])
hh, ww = [x * 2 for x in img.shape[:2]]
print(hh)
print(ww)

# specify input coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,
input = np.float32([[512, 634], [1490, 644], [1914,878], [108,832]])

# get top and left dimensions and set to output dimensions of red rectangle
width = round(math.hypot(input[0,0]-input[1,0], input[0,1]-input[1,1]))
height = round(math.hypot(input[0,0]-input[3,0], input[0,1]-input[3,1]))
print("width:",width, "height:",height)

# set upper left coordinates for output rectangle
x = input[0,0]
y = input[0,1]

# specify output coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,
output = np.float32([[x,y], [x+width-1,y], [x+width-1,y+height-1], [x,y+height-1]])

# compute perspective matrix
matrix = cv2.getPerspectiveTransform(input,output)
print(matrix)

# do perspective transformation setting area outside input to black
# Note that output size is the same as the input image size
imgOutput = cv2.warpPerspective(img, matrix, (ww,hh), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

# save the warped output
cv2.imwrite("output_images/output_"+"image"+".jpeg", imgOutput)

imgOutput2 = cv2.warpPerspective(img2, matrix, (ww,hh), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

# save the warped output
cv2.imwrite("output_images/output_"+"image2"+".jpeg", imgOutput2)

# show the result
cv2.imshow("result", imgOutput2)
cv2.waitKey(0)
cv2.destroyAllWindows()