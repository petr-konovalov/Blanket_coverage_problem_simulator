import simulator_base_classes
from simulator_base_classes import *
import simulator_base_constants
from simulator_base_constants import *

def dot(U, V):
    return U[0]*V[0]+U[1]*V[1]

def mul(U, V):
    return U[0]*V[1]-U[1]*V[0]
            
def calcN(objs, RID):
    res = []
    for Id, obj in enumerate(objs):
        if obj.isLive() and obj.inN(objs[RID].getPos(), RVis):
            res.append(Id)
    return res
    
#Вернёт координаты случайно точки круга радиуса R с центров в (OX, OY) (равномерное распределение)
def getRndCrcPnt(R, OX = 0, OY = 0, OZ = 0):
    X = rdm.uniform(-R, R)
    Y = rdm.uniform(-R, R)
    Z = rdm.uniform(-R, R)
    #Генерируем случайную точку внутри квадрата пока не попадём в круг
    while X ** 2 + Y ** 2 + Z ** 2 > R ** 2:
        X = rdm.uniform(-R, R)
        Y = rdm.uniform(-R, R)
        Z = rdm.uniform(-R, R)
        
    return np.array([X + OX, Y + OY, Z + OZ])

def getRndDir(coef = 0.5):
    a = (np.random.rand() - 0.5) * mh.pi * coef
    return np.array([mh.cos(a), mh.sin(a)])

def getRobotsDist(objs, pos):
    res = 100000
    for obj in objs:
        if obj.isLive():
         res = min(res, obj.getDist(pos))
    return res

def tupleIntMul(scalar, Tuple):
    return tuple([scalar * x for x in Tuple])

def tupleSum(tuple1, tuple2):
    l = min(len(tuple1), len(tuple2))    
    return tuple([tuple1[j] + tuple2[j] for j in range(0, l)])

def rotateVec(U, ang):
    sa = mh.sin(ang)
    ca = mh.cos(ang)
    return [U[0]*ca-U[1]*sa, U[0]*sa+U[1]*ca]