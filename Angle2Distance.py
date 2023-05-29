import math

def angle2distance(西塔, phi, 阿尔法, 甲, 乙):
    上边 = 甲*乙*math.sin(phi+阿尔法)-(甲^2)*math.sin(phi)
    下边 = math.sqrt(甲^2*(math.sin(phi))^2+乙^2*(math.sin(西塔))^2+2*甲*乙*math.sin(西塔)*math.sin(phi)*math.cos(阿尔法+phi-西塔))
    丙 = 上边/下边
    return 丙