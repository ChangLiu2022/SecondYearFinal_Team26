import cv2
import numpy as np

img0 = cv2.imread("output_images/output_image.jpeg")
img2 = cv2.imread("output_images/output_image2.jpeg")
img6 = cv2.imread("output_images/output_image6.jpeg")
img8 = cv2.imread("output_images/output_image8.jpeg")

blank_image = np.zeros((5*img0.shape[:2][0],2*img0.shape[:2][1]+1,3), np.uint8)

print(blank_image.shape)
blank_image[blank_image.shape[0]-img0.shape[0]:blank_image.shape[0], blank_image.shape[1]-img0.shape[1]:blank_image.shape[1]] = img0

dy1 = 0.1
dx1 = 0.05

blank_image[blank_image.shape[0]-round(dy1*img2.shape[0])-img2.shape[0]:blank_image.shape[0]-round(dy1*img2.shape[0]), blank_image.shape[1]-round(dx1*img2.shape[1])-img2.shape[1]:blank_image.shape[1]-round(dx1*img2.shape[1])] = img2

dy1 = dy1 + 0.1
dx1 = dx1 + 0

blank_image[blank_image.shape[0]-round(dy1*img2.shape[0])-img2.shape[0]:blank_image.shape[0]-round(dy1*img2.shape[0]), blank_image.shape[1]-round(dx1*img2.shape[1])-img2.shape[1]:blank_image.shape[1]-round(dx1*img2.shape[1])] = img6

dy1 = dy1 + 0.1
dx1 = dx1 + 0

blank_image[blank_image.shape[0]-round(dy1*img2.shape[0])-img2.shape[0]:blank_image.shape[0]-round(dy1*img2.shape[0]), blank_image.shape[1]-round(dx1*img2.shape[1])-img2.shape[1]:blank_image.shape[1]-round(dx1*img2.shape[1])] = img8



cv2.imwrite("output_images/STACK_output_"+"image"+".jpeg", blank_image)

# show the result
cv2.imshow("result", blank_image)
cv2.waitKey(0)
cv2.destroyAllWindows()