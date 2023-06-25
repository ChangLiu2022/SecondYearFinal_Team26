import math

def getAngle(x_1,x_mid,x_2,xb = 640, fov=1.22173): #in radian where fov = 70 degrees
    x_left = min(x_1,x_2)
    x_right = max(x_1,x_2)
    phi = (x_mid - x_left) * (fov/xb)
    theta = (x_right - x_mid) * (fov/xb)
    return phi, theta


def angle2distance(phi, theta, a, b, alpha = 1.5708):#alpha, a, b are known parameters that were given when putting the beacons down
    nominator = a*b*math.sin(phi+alpha)-pow(a,2)*math.sin(phi)
    denominator = math.sqrt(pow(a,2)*pow((math.sin(phi)),2)+pow(b,2)*pow((math.sin(theta)),2)+2*a*b*math.sin(theta)*math.sin(phi)*math.cos(alpha+phi-theta))
    print(phi)
    print(theta)
    print(nominator)
    print(denominator)
    c = nominator/denominator

    beta = math.atan((b*math.sin(theta)*math.sin(alpha+phi)-a*math.sin(phi)*math.sin(theta))/(b*math.sin(theta)*math.cos(alpha+phi)+a*math.sin(phi)*math.cos(theta)))
    amb = alpha-beta
    d = b*math.sin(amb)/math.sin(phi)

    return d,c,beta

def getXY(c,beta,theta,a):
    dtheta = 1.5708 - beta - theta
    x = math.sin(dtheta) * c + a
    y = math.cos(dtheta) * c
    return x, y

def locationRoutine(a, b, x_1, x_mid, x_2): #main function, feed in coords of beacons and set a,b, return x y location
    phi, theta = getAngle(x_1, x_mid, x_2, 640, 1.22173)
    _,c,beta = angle2distance(phi, theta, a, b, 1.5708)
    return getXY(c,beta,theta,a)
