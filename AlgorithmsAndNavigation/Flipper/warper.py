import numpy as np
from matplotlib import pyplot as plt
import cv2
import math
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
# algorithm from nvidia https://docs.nvidia.com/vpi/algo_persp_warp.html#:~:text=Perspective%20Warp%20algorithm%20allows%20for,wall%2C%20but%20looking%20from%20below.
# warping using equation here https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html#gaf73673a7e8e18ec6963e3774e6a94b87
# matrix from perspectivetest2 from opencv

# def warpPoint(xu,xv,matrix):
#     xmatrix = np.matrix([[xu],
#                         [xv],
#                         [1]])
#     ymatrix = np.matmul(matrix,xmatrix)
#     yuu = ymatrix[0,0]/ymatrix[2,0]
#     yvv = ymatrix[1,0]/ymatrix[2,0]
    
#     #return yuu, yvv
#     return np.array([[yuu,yvv]])

def warpPoint(xu,xv,matrix):

    yuu = 640-(matrix[0,0]*xu+matrix[0,1]*xv+matrix[0,2])/(matrix[2,0]*xu+matrix[2,1]*xv+matrix[2,2])
    yvv = 480-(matrix[1,0]*xu+matrix[1,1]*xv+matrix[1,2])/(matrix[2,0]*xu+matrix[2,1]*xv+matrix[2,2])
    
    #return yuu, yvv
    return np.array([[yuu,yvv]])

# assume matrix is coded as
# np.matrix([[x1,y1],
#           [x2,y2],
#           [x3,y3],
#           ...
#           [xn,yn]])

def getMatrix():
    hh = 480
    ww = 640
    print(hh)
    print(ww)

    # specify input coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,
    # input = np.float32([[512, 634], [1490, 644], [1914,878], [108,832]])
    input = np.float32([[450, 110], [239, 105], [177,40], [538,45]])

    # get top and left dimensions and set to output dimensions of red rectangle
    width = round(math.hypot(input[0,0]-input[1,0], input[0,1]-input[1,1]))
    height = round(math.hypot(input[0,0]-input[3,0], input[0,1]-input[3,1]))
    print("width:",width, "height:",height)

    x= round(ww/2)
    y= hh/2

    # specify output coordinates for corners of red quadrilateral in order TL, TR, BR, BL as x,
    output = np.float32([[x-round(width/2)+1,y-round(height/2)+1], [x+round(width/2)-1,y-round(height/2)+1], [x+round(width/2)-1,y], [x-round(width/2)+1,y]])

    # compute perspective matrix
    matrix = cv2.getPerspectiveTransform(input,output)
    return matrix

def warpRoutine(input):
    i = 0
    xu = 0
    xv = 0
    warpmatrix = getMatrix()
    
    target = np.array([[0,0]])
    for i in range(input.shape[0]):
        xu = input[i,0]
        xv = input[i,1]
        newrow = warpPoint(xu,xv,warpmatrix)
        target = np.vstack([target, newrow])

    target = target[1:(target.shape[0]-1),:]
    return target

def imageGen(target,dx,dy):
    x, y = target.T
    plt.scatter(x,y)
    plt.savefig('plt_output_images/plt_warped.png')
    plt.show()
    #clusterer(x,y,0.4, True)
    xxx,yyy=imagecomb(target,dx,dy)
    #plt.scatter(xxx.T,yyy.T)
    #plt.savefig('plt_output_images/plt_warped_biased.png')
    #plt.show()
    return xxx,yyy

def imagecomb(target, dx,dy):
    x = (target[:,0]+dx).T
    y = (target[:,1]+dy).T
    return x,y

def clusterer(ax,ay,eps=0.2,warped = False):
    X = [i for i in zip(ax,ay)]
    X = StandardScaler().fit_transform(X)

    """
    Compute the DBSCAN
    """
    db = DBSCAN(eps, min_samples=1).fit(X)
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_clusters_

    """
    Plot the clusters
    """
    d= dict(zip(set(labels),['red','green','blue','yellow','purple','grey','pink','red','green','blue','yellow','purple']))
    d[-1] = "black"
    if(not warped):
        plt.xlim([0, 640])
        plt.ylim([0, 480])
    plt.scatter(ax,ay,color=[d[i] for i in labels])
    plt.show()

def imageRoutine(input,dx,dy):
    return imageGen(warpRoutine(input),dx,dy)
    
# a = np.array([[1,1],
#           [2,2],
#           [3,3],
#           [9,8]])
# imageGen(a)