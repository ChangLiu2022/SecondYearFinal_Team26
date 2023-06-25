import numpy as np
import cv2
import math

# read input
img = cv2.imread("images/image5.jpeg")
img2 = cv2.imread("images/image6.jpeg")
img3 = cv2.imread("images/image8.jpeg")
print(img)
print(img.shape[:2])
hh = 480
ww = 640
print(hh)
print(ww)

# specify input coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,
#input = np.float32([[512, 634], [1490, 644], [1914,878], [108,832]])
input = np.float32([[185, 407], [386, 407], [230,369], [400,369]])

# get top and left dimensions and set to output dimensions of red rectangle
width = round(math.hypot(input[0,0]-input[1,0], input[0,1]-input[1,1]))
height = round(math.hypot(input[0,0]-input[3,0], input[0,1]-input[3,1]))
print("width:",width, "height:",height)

# set bottom middle
x = round((input[2,0]+input[3,0])/2)+2000
y = round((input[2,1]+input[3,1])/2)+2000

x= round(ww/2)
y= hh-100
# specify output coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,
output = np.float32([[x-round(width/2)+1,y-round(height/2)+1], [x+round(width/2)-1,y-round(height/2)+1], [x+round(width/2)-1,y], [x-round(width/2)+1,y]])

# compute perspective matrix
matrix = cv2.getPerspectiveTransform(input,output)
print(matrix)

# do perspective transformation setting area outside input to black
# Note that output size is the same as the input image size
imgOutput = cv2.warpPerspective(img, matrix, (ww,hh), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

# save the warped output
cv2.imwrite("output_images/output_"+"image5"+".jpeg", imgOutput)

imgOutput2 = cv2.warpPerspective(img2, matrix, (ww,hh), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

# save the warped output
cv2.imwrite("output_images/output_"+"image6"+".jpeg", imgOutput2)

imgOutput3 = cv2.warpPerspective(img3, matrix, (ww,hh), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))

# save the warped output
cv2.imwrite("output_images/output_"+"image8"+".jpeg", imgOutput3)

# show the result
cv2.imshow("result", imgOutput3)
cv2.waitKey(0)
cv2.destroyAllWindows()