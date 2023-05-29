import math

def angle2distance(西塔, phi, 阿尔法, 甲, 乙):
    上边 = 甲*乙*math.sin(phi+阿尔法)-pow(甲)*math.sin(phi)
    下边 = math.sqrt(pow(甲)*pow((math.sin(phi)))+pow(乙)*pow((math.sin(西塔)))+2*甲*乙*math.sin(西塔)*math.sin(phi)*math.cos(阿尔法+phi-西塔))
    丙 = 上边/下边
    return 丙

print(angle2distance(0.34,0.41,0.8,600,700))
