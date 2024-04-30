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

def trunc(arg, left, right):
    if arg < left:
        return left
    elif arg > right:
        return right
    else:
        return arg

def normir(v):
    vnorm = la.norm(v)
    if vnorm == 0:
        return v
    else:
        return v / vnorm

def getPointToSegmentProjection(pnt, A, B):
    v = B - A
    nv2 = dot(v, v)
    if nv2 == 0:
        return A
    return A + v * trunc(dot(pnt - A, v) / nv2, 0, 1)

def inRange(arg, left, right):
    if left > right:
        left, right = right, left
    return left <= arg and arg <= right

def pntInRect(pnt, A, B):
    return inRange(pnt[0], A[0], B[0]) and inRange(pnt[1], A[1], B[1])

def measureVisibleObjects(objs, objectsDescriptor, agentId, rCnt, RVis, generationCount = DEFAULT_ANCHOR_GENERATION_COUNT):
    res = []
    for i, agent in enumerate(objs):
        p = agent.getPos() - objs[agentId].getPos()
        if i != agentId and la.norm(p) <= RVis:
            res.append(p)
    for a in objectsDescriptor:
        for a1, a2 in zip(a, np.roll(a, len(a[0]))):
            proj = getPointToSegmentProjection(objs[agentId].getPos(), a1, a2) - objs[agentId].getPos()
            nProj = la.norm(proj)
            if nProj <= RVis:
                v = normir(a2 - a1) * RVis / generationCount
                for p in [proj + t * v for t in range(-generationCount, generationCount + 1)]:
                    if pntInRect(p + objs[agentId].getPos(), a1, a2) and la.norm(p) <= RVis:
                        res.append(p)
    return res

def measureVisibleObjectsSimple(objs, agentId, rCnt, RVis):
    res = []
    for i, agent in enumerate(objs):
        p = agent.getPos() - objs[agentId].getPos()
        if i != agentId and la.norm(p) <= RVis:
            res.append(p)
    return res