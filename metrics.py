import numpy as np
import numpy.linalg as la
import math as mh

def between(l, r, arg):
    return l <= arg and arg < r

def getSegmentY(pL, pR, x):
    return (pR[1] - pL[1]) / (pR[0]-pL[0]) * (x - pL[0]) + pL[1]
    
def pointInWorkArea(objectsDescriptor, pnt):
    cnt = 0
    for angles in objectsDescriptor:
        for l in range(0, len(angles)):
            pntL = angles[l]
            pntR = angles[0 if l + 1 == len(angles) else l + 1]
            if pntR[0] < pntL[0]:
                pntL, pntR = pntR, pntL
            if between(pntL[0], pntR[0], pnt[0]) and getSegmentY(pntL, pntR, pnt[0]) > pnt[1]:
                cnt += 1
    return cnt % 2 == 1

def robotAreaCount(objs, pnt, RVis):
    cnt = 0
    for obj in objs:
        if la.norm(obj.getPos()[:2] - pnt) < RVis:
            cnt += 1
    return cnt
    
def pointInRobotArea(objs, pnt, RVis):
    for obj in objs:
        if la.norm(obj.getPos()[:2] - pnt) < RVis:
            return True
    return False

def calculateMetrics(objs, rCnt, objectsDescriptor, RVis, running, leftBound = -1000, rightBound = 1000, downBound = -1000, upBound = 1000, step = 3):
    workAreaCnt = 0
    robotAreaCnt = 0
    areaCounts = []
    for x in range(leftBound, rightBound, step):
        for y in range(downBound, upBound, step):
            if pointInWorkArea(objectsDescriptor, np.array([x, y])):
                workAreaCnt += 1
                sensorsAreaCnt = robotAreaCount(objs[:rCnt], np.array([x, y]), RVis)
                if sensorsAreaCnt > 0:
                    robotAreaCnt += 1
                    areaCounts.append(sensorsAreaCnt)
    sensorCoverageUniformity = np.std(areaCounts)
    uniformity = 0
    for j in range(0, rCnt):
        meanDist = 0
        nCnt = 0
        for k in range(0, rCnt):
            if j != k:
                curDist = la.norm(objs[j].getPos() - objs[k].getPos()) / 100
                if curDist < RVis / 100:
                    meanDist += curDist
                    nCnt += 1
        if nCnt > 0:
            meanDist /= nCnt
        U = 0
        for k in range(0, rCnt):
            if j != k:
                curDist = la.norm(objs[j].getPos() - objs[k].getPos()) / 100
                if curDist < RVis / 100:
                    U += (curDist - meanDist) ** 2
        if nCnt > 0:
            U = mh.sqrt(U / nCnt) 
        uniformity += U
    uniformity /= rCnt
    return robotAreaCnt, workAreaCnt, sensorCoverageUniformity, uniformity

def checkTerminateCondition(running, maxrunning, sumV, rCnt, maxV):
    return running == maxrunning or running >= 0 and running < maxrunning and sumV / rCnt < maxV * 0.01

