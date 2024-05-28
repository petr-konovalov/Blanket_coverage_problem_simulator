import simulator_base_constants
from simulator_base_constants import *
import simulator_base_functions
from simulator_base_functions import *
from math import pi, sin, cos

class Algorithm:
    def applySpeedConstraints(self, speed):
        absV = la.norm(speed)
        if absV > maxV:
            return speed / absV * maxV
        return speed
    def calcSpeed(self, visibleObjects):
        pass
    
class DebugAlgorithm(Algorithm):
    def hideDebug(self, sc, O, pos):
        pass
    def drawDebug(self, sc, O, pos):
        pass

class DiscreteTimeSpeedConstraintsAlgorithm(Algorithm):
    def __init__(self, e = 0.52, Olim = 10, Slim = 10):
        self.e = e
        self.Olim = Olim
        self.Slim = Slim
        self.Ocnt = 0
        self.Scnt = 0
        self.state = 'ordinary'
        self.prevSpeed = np.zeros(3)
        
    def applySpeedConstraints(self, speed):
        speed = super().applySpeedConstraints(speed)
        if la.norm(self.prevSpeed) < self.e:
            self.Scnt += 1
        if self.Scnt >= self.Slim:
            speed = np.zeros(3)
            self.state = 'stop'
        if la.norm(speed + self.prevSpeed) < self.e:
            self.Ocnt += 1
        if self.Ocnt >= self.Olim:
            speed = speed / 2
            self.state = 'stop'
        self.prevSpeed = speed
        return speed
    
class InitialContextRequiredAlgorithm(Algorithm):
    def setup(self, objects, agentsCount, obstacles):
        pass
    
class DynamicContextRequiredAlgorithm(Algorithm):
    def update(self, objects, agentsCount, selfId):
        pass
    
class CommunicationRequiredAlgorithm(Algorithm):
    def getResponse(self):
        pass
    def receiveData(self, availableAgents):
        self.receivedData = [agent.getResponse() for agent in availableAgents]

class SWARM(DiscreteTimeSpeedConstraintsAlgorithm, CommunicationRequiredAlgorithm):
    def __init__(self):
        super().__init__()
        self.K = (mh.sqrt(3) * RVis * 0.5) ** 3
        self.w1 = 0.4
        self.w2 = 0.4
        self.w3 = 1 - self.w1 - self.w2
        
    def getResponse(self):
        return self.prevSpeed
    
    def calcSpeed(self, visibleObjects):
        Fsep = np.zeros(3)
        Fcoh = np.zeros(3)
        Fobst = np.zeros(3)
        Falig = np.mean(self.receivedData, axis = 0)
        agentCnt = 0
        wallCnt = 0
        if self.state == 'ordinary':
            for vec, label in zip(visibleObjects['Positions'], visibleObjects['Labels']):
                if label == 'Agent':
                    Fsep += self.K * vec / la.norm(vec) ** 3
                    Fcoh += vec
                    agentCnt += 1
                elif label == 'Wall':
                    Fobst += self.K * vec / la.norm(vec) ** 2
                    wallCnt += 1
            if agentCnt > 0:     
                Fsep /= agentCnt
                Fcoh /= agentCnt
            if wallCnt > 0:
                Fobst /= wallCnt
        return self.applySpeedConstraints(-(self.w1 * (Fsep + Fobst) + self.w2 * Fcoh + self.w3 * Falig))

class VFASF(DynamicContextRequiredAlgorithm):
    def __init__(self):
        self.kap = 15/2500
        self.Fcentr = 0.005/2500
        self.gamma = 7.75/50
        self.speed = np.zeros(3)
        
    def update(self, objects, agentsCount, selfId):
        self.position = objects[selfId].getPos()
        
    def applySpeedConstraints(self, speed):
        absV = la.norm(speed)
        if absV < maxV * 0.01:
            speed = np.zeros(3)
        elif absV > maxV:
            speed = speed / absV * maxV
        return speed
        
    def calcSpeed(self, visibleObjects):
        force = -self.Fcentr * self.position
        N = [[la.norm(vec)] + list(vec) for vec in visibleObjects['Positions']]
        N.sort()
        for k, vec in enumerate(N):
            check = True
            for v in N[:k]:
                if dot(vec[1:], v[1:]) / (vec[0] * v[0]) >= 0.5:
                    check = False
                    break
            if check:
                force += self.kap * (vec[0] - RVis/mh.sqrt(3)) * np.array(vec[1:]) / vec[0]
        self.speed = self.applySpeedConstraints((self.speed + force)/(1 + self.gamma))
        return self.speed

class SA(Algorithm):
    def __init__(self, sectorCount = 6, speedCoef = 12):
        self.secCnt = sectorCount
        self.speedCoef = speedCoef
        self.secBisectors = [np.array([cos(a), sin(a), 0]) for a in [(k+0.5)*pi/sectorCount*2 for k in range(0, sectorCount)]] 

    def getSecId(self, v):
        ang = mh.atan2(v[1], v[0])
        return int(ang / pi * self.secCnt * 0.5 + self.secCnt) % self.secCnt
        
    def calcSpeed(self, visibleObjects):
        secNearest = np.array([RVis * d for d in self.secBisectors])
        for v in visibleObjects['Positions']:
            secId = self.getSecId(v)
            if la.norm(secNearest[secId, :]) > la.norm(v):
                secNearest[secId, :] = v
        meanNearest = sum([la.norm(S) for S in secNearest]) / self.secCnt
        newV = 0
        for S in secNearest:
            Snorm = la.norm(S)
            newV += -S / Snorm * (1 - Snorm / meanNearest)
        return self.applySpeedConstraints(self.speedCoef * newV)

class CSA(SA):
    def __init__(self, sectorCount = 6, speedCoef = 20):
        super().__init__(sectorCount, speedCoef)

    def getLeftRightSecs(self, secId):
        angLeft = secId * 2 * pi / self.secCnt
        return angLeft, (secId + self.secCnt - 1) % self.secCnt, angLeft + 2 * pi / self.secCnt, (secId + 1) % self.secCnt

    def calcReflections(self, v, vLen, angLeft, secLeftId, angRight, secRightId):
        vecLeft = np.array([cos(angLeft), sin(angLeft), 0])
        vecRight = np.array([cos(angRight), sin(angRight), 0])
        dL = abs(mul(vecLeft, v))
        dR = abs(mul(vecRight, v))
        leftReflection = (self.secBisectors[secLeftId] * RVis * dL + vecLeft * vLen * dR) / (dR + dL)
        rightReflection = (self.secBisectors[secRightId] * RVis * dR + vecRight * vLen * dL) / (dR + dL)
        leftReflection = normir(leftReflection) * max(vLen, min(la.norm(leftReflection), RVis))
        rightReflection = normir(rightReflection) * max(vLen, min(la.norm(rightReflection), RVis))
        return leftReflection, rightReflection

    def calcShadow(self, v, vLen, secId):
        cD = mul(self.secBisectors[secId], v)
        rD = RVis - vLen
        if max(rD, 0) < abs(cD):
            v = v + np.array([self.secBisectors[secId][1], 
                              -self.secBisectors[secId][0], 0]) * cD / abs(cD) * (abs(cD) - rD)
            v = v / la.norm(v) * vLen
        return v

    def updateContinuousNearest(self, v, w, secId, weightsSumBySec, secNearest):
        if weightsSumBySec[secId] == 0:
            secNearest[secId, :] = np.zeros(3)
        weightsSumBySec[secId] += w 
        secNearest[secId, :] += v * w            

    def calcShadowAndUpdate(self, v, vLen, secId, weightsSumBySec, secNearest):
        if vLen > 0: 
            self.updateContinuousNearest(self.calcShadow(v, vLen, secId), (RVis - vLen) ** 2, secId, weightsSumBySec, secNearest)

    def calcContinuousNearest(self, weightsSumBySec, secNearest):
        nonZeroWeights = weightsSumBySec > 0
        secNearest[nonZeroWeights] = (secNearest[nonZeroWeights].T / weightsSumBySec[nonZeroWeights]).T
        
    def calcSpeed(self, visibleObjects):
        weightsSumBySec = np.zeros([self.secCnt])
        secNearest = np.array([s * RVis for s in self.secBisectors])
        for v in visibleObjects['Positions']:
            vLen = la.norm(v)
            secId = self.getSecId(v)
            angLeft, secLeftId, angRight, secRightId = self.getLeftRightSecs(secId)
            leftRef, rightRef = self.calcReflections(v, vLen, angLeft, secLeftId, angRight, secRightId)
            self.calcShadowAndUpdate(v, vLen, secId, weightsSumBySec, secNearest)
            self.calcShadowAndUpdate(leftRef, la.norm(leftRef), secLeftId, weightsSumBySec, secNearest)
            self.calcShadowAndUpdate(rightRef, la.norm(rightRef), secRightId, weightsSumBySec, secNearest)
        self.calcContinuousNearest(weightsSumBySec, secNearest)
        meanNearest = sum([la.norm(S) for S in secNearest]) / self.secCnt
        newV = 0
        for S in secNearest:
            Snorm = la.norm(S)
            if Snorm > 0:
                newV += -S / Snorm * (1 - Snorm / meanNearest)
        return self.applySpeedConstraints(self.speedCoef * newV)

class DSSA(DiscreteTimeSpeedConstraintsAlgorithm, InitialContextRequiredAlgorithm):
    def __init__(self, A):
        super().__init__()
        self.A = A
        
    def setup(self, objects, agentsCount, obstacles):
        self.mu = agentsCount * 3.14 * RVis ** 2 / self.A

    def partialForce(self, D, vec, dst):
        return -(D / (self.mu ** 2)) * (RVis - dst) * vec / dst
            
    def calcSpeed(self, visibleObjects):
        newV = np.zeros(3)
        D = 0
        if self.state == 'ordinary':
            D = len(visibleObjects['Positions'])
            for vec in visibleObjects['Positions']:
                newV += self.partialForce(D, vec, la.norm(vec)) 
        return self.applySpeedConstraints(newV)

class SODA(DSSA):
    def partialForce(self, D, vec, dst):
        return -(D / (self.mu * (D if D > self.mu else self.mu))) * (RVis - dst) * vec / dst

class SSND(DSSA, CommunicationRequiredAlgorithm):
    def __init__(self, A, alpha = 1):
        super().__init__(A = A)
        self.alpha = alpha
        self.M = 0
        self.F = np.zeros(3)
        self.Elig = 0

    def getResponse(self):
        return {'F': self.F, 'Eligibility': self.Elig}

    def calcEligibility(self):
        self.Elig = -maxrunning if self.state != 'ordinary' else (self.alpha * 
                         (len([1 for data in self.receivedData if la.norm(self.F) >= la.norm(data['F'])]) - self.M) - 
                     abs(self.mu - len(self.receivedData))) 
        
    def calcSpeed(self, visibleObjects):
        moving = self.Elig >= max([data['Eligibility'] for data in self.receivedData])
        self.calcEligibility()
        self.F = super().calcSpeed(visibleObjects)
        if moving:
            self.M += 1
            self.F = self.applySpeedConstraints(self.F)
            return self.F
        else:
            return np.zeros(3)

class ESF(DebugAlgorithm): #Empty sector follower
    def __init__(self, minEmptySectorDegrees = 20, agentSectorDegrees = 120, wallSectorDegrees = 16, baseAlgorithm = SA()):
        self.minEmptySec = minEmptySectorDegrees / 180.0 * mh.pi
        self.agentSecHalf = 0.5 * agentSectorDegrees / 180.0 * mh.pi
        self.wallSecHalf = 0.5 * wallSectorDegrees / 180.0 * mh.pi
        self.baseAlgorithm = baseAlgorithm
        self.target = 0
        self.maxSector = (0, 0)

    def hideDebug(self, sc, O, pos):
        targetDir = np.array([cos(self.__debugOldTarget), sin(self.__debugOldTarget), 0])
        leftSecBound = np.array([cos(self.__debugOldMaxSector[0]), sin(self.__debugOldMaxSector[0]), 0])
        rightSecBound = np.array([cos(self.__debugOldMaxSector[1]), sin(self.__debugOldMaxSector[1]), 0])
        pygame.draw.line(sc, (255, 255, 255), tuple((pos*scale+O)[:2]), tuple(((pos+RVis/3*targetDir)*scale+O)[:2]), 5)
        pygame.draw.line(sc, (255, 255, 255), tuple((pos*scale+O)[:2]), tuple(((pos+RVis/3*leftSecBound)*scale+O)[:2]), 5)
        pygame.draw.line(sc, (255, 255, 255), tuple((pos*scale+O)[:2]), tuple(((pos+RVis/3*rightSecBound)*scale+O)[:2]), 5)
        
    def drawDebug(self, sc, O, pos):
        self.__debugOldTarget = self.target
        self.__debugOldMaxSector = self.maxSector
        if self.maxSector[1] - self.maxSector[0] > self.minEmptySec:
            targetDir = np.array([cos(self.target), sin(self.target), 0])
            leftSecBound = np.array([cos(self.maxSector[0]), sin(self.maxSector[0]), 0])
            rightSecBound = np.array([cos(self.maxSector[1]), sin(self.maxSector[1]), 0])
            pygame.draw.line(sc, (255, 165,   0), tuple((pos*scale+O)[:2]), tuple(((pos+RVis/3*targetDir)*scale+O)[:2]), 3)
            pygame.draw.line(sc, (  0, 128,   0), tuple((pos*scale+O)[:2]), tuple(((pos+RVis/3*leftSecBound)*scale+O)[:2]), 3)
            pygame.draw.line(sc, (  0, 128,   0), tuple((pos*scale+O)[:2]), tuple(((pos+RVis/3*rightSecBound)*scale+O)[:2]), 3)

    def __checkSegIntersect(self, l1, r1, l2, r2):
        return l2 <= l1 and l1 <= r2 or l2 <= r1 and r1 <= r2
        
    def calcAngleBraces(self, visibleObjects):
        angleBraces = []
        for pos, label in zip(visibleObjects['Positions'], visibleObjects['Labels']):
            angle = mh.atan2(pos[1], pos[0])
            if label == 'Wall':
                l = angle-self.wallSecHalf
                r = angle+self.wallSecHalf
            elif label == 'Agent':
                l = angle-self.agentSecHalf
                r = angle+self.agentSecHalf
            if self.__checkSegIntersect(l-2*mh.pi, r-2*mh.pi, -2*mh.pi, 2*mh.pi):
                angleBraces.append((l-2*mh.pi, 0))
                angleBraces.append((r-2*mh.pi, 1))
            angleBraces.append((l, 0))
            angleBraces.append((r, 1))
            if self.__checkSegIntersect(l+2*mh.pi, r+2*mh.pi, -2*mh.pi, 2*mh.pi):
                angleBraces.append((l+2*mh.pi, 0))
                angleBraces.append((r+2*mh.pi, 1))
        angleBraces.sort()
        return angleBraces

    def checkDirection(self, angleBraces):
        if self.target < -mh.pi:
            self.target += 2 * mh.pi
        elif self.target >  mh.pi:
            self.target -= 2 * mh.pi
        depth = 0
        for k, b in enumerate(angleBraces[:-1]):
            depth += 1 if b[1] == 0 else -1
            if b[0] <= self.target and self.target <= angleBraces[k+1][0]:
                if depth == 0 and angleBraces[k+1][0] - b[0] > self.minEmptySec:
                    # self.target = (angleBraces[k+1][0] + b[0]) * 0.5
                    self.maxSector = (b[0], angleBraces[k+1][0])
                    return True
                else:
                    return False
        return True

    def findLargestSector(self, angleBraces):
        maxSector = (0, 0)
        depth = 0
        for k, b in enumerate(angleBraces[:-1]):
            depth += 1 if b[1] == 0 else -1
            if depth == 0:
                angleWidth = angleBraces[k+1][0] - b[0]
                if maxSector[1] - maxSector[0] < angleWidth:
                    maxSector = (b[0], b[0] + angleWidth)        
        return maxSector
    
    def calcSpeed(self, visibleObjects):
        angleBraces = self.calcAngleBraces(visibleObjects)
        nearestAgentDist = RVis
        for v, label in zip(visibleObjects['Positions'], visibleObjects['Labels']):
            vlen = la.norm(v)
            if label == 'Agent' and vlen < nearestAgentDist:
                nearestAgentDist = vlen
        if self.checkDirection(angleBraces):
            return np.array([cos(self.target), sin(self.target), 0]) * max(min((1 - nearestAgentDist/RVis) * 10, 1) * maxV, maxV/10)
        maxSector = self.findLargestSector(angleBraces)
        self.maxSector = maxSector
        if maxSector[1] - maxSector[0] > self.minEmptySec:
            self.target = (maxSector[1] + maxSector[0]) * 0.5
            return np.array([cos(self.target), sin(self.target), 0]) * max(min((1 - nearestAgentDist/RVis) * 10, 1) * maxV, maxV/10)
        else:
            return self.baseAlgorithm.calcSpeed(visibleObjects)

class SAOP(ESF): #Sectors Attracts Obstacles Pushes
    def __init__(self, speedCoef = 20, minEmptySectorDegrees = 20, agentSectorDegrees = 120, wallSectorDegrees = 30, baseAlgorithm = SA()):
        super().__init__(minEmptySectorDegrees, agentSectorDegrees, wallSectorDegrees, baseAlgorithm)
        self.state = 0
        self.speedCoef = speedCoef
        
    def calcSectorDirections(self, angleBraces):
        res = {}
        depth = 0
        angleWidthSum = 0
        dirPerSec = 10
        for k, b in enumerate(angleBraces[:-1]):
            depth += 1 if b[1] == 0 else -1
            if depth == 0:
                angleWidth = angleBraces[k+1][0] - b[0]
                if angleWidth > self.minEmptySec:
                    for j in range(1, dirPerSec - 1):
                        dir = (angleBraces[k+1][0] * j + b[0] * (dirPerSec - j))/dirPerSec
                        if dir > mh.pi:
                            dir -= 2 * mh.pi
                        elif dir < -mh.pi:
                            dir += 2 * mh.pi
                        res[dir] = angleWidth
        return res

    def calcSectorSum(self, angleBraces):
        res = {}
        depth = 0
        angleWidthSum = 0
        for k, b in enumerate(angleBraces[:-1]):
            depth += 1 if b[1] == 0 else -1
            if depth == 0:
                angleWidth = angleBraces[k+1][0] - b[0]
                if angleWidth > self.minEmptySec:
                    dir = (angleBraces[k+1][0] + b[0])/2
                    if dir > mh.pi:
                        dir -= 2 * mh.pi
                    elif dir < -mh.pi:
                        dir += 2 * mh.pi
                    if not dir in res:
                        res[dir] = angleWidth
                        angleWidthSum += angleWidth
        return angleWidthSum
        
    def calcSpeed(self, visibleObjects):
        # secDirs = self.calcSectorDirections(self.calcAngleBraces(visibleObjects))
        # newV = np.zeros(3)
        # for dir, w in secDirs.items():
        #     v = np.array([cos(dir), sin(dir), 0])
        #     W = mh.sqrt(w / mh.pi)
        #     newV += v * W
        # newV = normir(newV) * maxV * 0.75
        
        nearestAgentDist = RVis
        nearestAgent = np.zeros(3)
        alpha = 0.99
        agentCnt = 0
        newV = np.zeros(3)
        for v, l in zip(visibleObjects['Positions'], visibleObjects['Labels']):
            if l == 'Agent':
                vlen = la.norm(v)
                if vlen < nearestAgentDist:
                    nearestAgentDist = vlen
                    nearestAgent = v
                    agentCnt += 1
        check = True
        for v, l in zip(visibleObjects['Positions'], visibleObjects['Labels']):
            if l == 'Agent':
                if dot(v, nearestAgent) < 0:
                    check = False
                    break
        if self.state == 2 and self.calcSectorSum(self.calcAngleBraces(visibleObjects)) > self.minEmptySec and agentCnt >= 1 and check and nearestAgentDist < RVis * alpha:
            self.state = 0
        if self.state < 2:
            near = {'Positions': [v for v, l in zip(visibleObjects['Positions'], visibleObjects['Labels'])
                                       if la.norm(v) < RVis * alpha], 
                     'Labels': [l for v, l in zip(visibleObjects['Positions'], visibleObjects['Labels'])
                                   if la.norm(v) < RVis * alpha]}
            angleBraces = self.calcAngleBraces(near)
            if self.state == 1:
                if self.maxSector[1] - self.maxSector[0] > 0 and self.checkDirection(angleBraces):
                    return np.array([cos(self.target), sin(self.target), 0]) * maxV * (0.5 if agentCnt <= 1 else min((1 - nearestAgentDist/RVis) * 10, 1))
                else:
                    self.state = 2
            elif self.state == 0:
                maxSec = self.findLargestSector(angleBraces)
                self.maxSector = maxSec
                if maxSec[1] - maxSec[0] > self.minEmptySec:
                    self.state = 1
                    self.target = (maxSec[1] + maxSec[0]) * 0.5
                    return np.array([cos(self.target), sin(self.target), 0]) * maxV * (0.5 if agentCnt <= 1 else min((1 - nearestAgentDist/RVis) * 10, 1))
                else:
                    self.state = 2
        elif self.state == 2:
            self.maxSector = (0, 0)
            self.target = 0
            newV = np.zeros(3)
            wallSum = np.zeros(3)
            wallWeight = 0
            wallMinLen = RVis
            wallMaxLen = 0
            wallLargest = np.zeros(3)
            wallNearest = np.zeros(3)
            nearestAgentDist = RVis
            meanDist = 0
            cnt = 0
            N = [[la.norm(vec)] + list(vec) for vec, l in zip(visibleObjects['Positions'], visibleObjects['Labels']) if l == 'Agent']
            N.sort()
            for k, vec in enumerate(N):
                coef = 1
                for v in N[:k]:
                    curDot = dot(vec[1:], v[1:]) / (vec[0] * v[0])
                    coef = min(coef, (0.5 - max(curDot - 0.5, 0))/0.5)
                    # if dot(vec[1:], v[1:]) / (vec[0] * v[0]) >= 0.5:
                    #     check = False
                    #     break
                meanDist += coef * vec[0]#la.norm(vec)
                cnt += coef
            if cnt > 0:
                meanDist /= cnt
            for k, vec in enumerate(N):
                coef = 1
                for v in N[:k]:
                    curDot = dot(vec[1:], v[1:]) / (vec[0] * v[0])
                    coef = min(coef, (0.5 - max(curDot - 0.5, 0))/0.5)
                newV += coef * (vec[0]/meanDist - 1.6 + 0.7 * max((vec[0]-0.8*RVis)/(0.2*RVis), 0)) * np.array(vec[1:]) / vec[0]
            # if len(visibleObjects['Positions']) > 0:
            #     meanAgentDist = np.mean([la.norm(v) for v, l in zip(visibleObjects['Positions'], visibleObjects['Labels'])])
            # else:
            #     meanAgentDist = RVis
        # for v, label in zip(visibleObjects['Positions'], visibleObjects['Labels']):
        #     vLen = la.norm(v)
        #     # newV -= v / vLen * w
        #     if label == 'Agent':
        #         newV -= v / vLen * (1 - vLen/meanAgentDist)
        #         nearestAgentDist = min(nearestAgentDist, vLen)
        #     if label == 'Wall':
        #         if vLen > wallMaxLen:
        #             wallMaxLen = vLen
        #             wallLargest = v
        #         if vLen < wallMinLen:
        #             wallMinLen = vLen
        #             wallNearest = v
            # elif label == 'Wall' and vLen < RVis / mh.sqrt(2):
            #     wallSum -= v / vLen * w
        # if wallMinLen < RVis:
        #     newV -= wallNearest / wallMinLen * (1 - wallMinLen / RVis)
        # if wallMaxLen > 0:
        #     newV += wallLargest / wallMaxLen * (1 - wallMaxLen / RVis)
            
        # maxSec = self.findLargestSector(self.calcAngleBraces(visibleObjects))
        # if maxSec[1] - maxSec[0] > self.minEmptySec:
        #     dir = (maxSec[1] + maxSec[0]) * 0.5
        #     return np.array([cos(dir), sin(dir), 0]) * min((1 - nearestAgentDist/RVis) * 10, 1)  * maxV
        # else:
            walls = {'Positions': [v for v, l in zip(visibleObjects['Positions'], visibleObjects['Labels'])
                                       if l == 'Wall' and la.norm(v) < RVis / mh.sqrt(2)], 
                     'Labels': [l for v, l in zip(visibleObjects['Positions'], visibleObjects['Labels'])
                                   if l == 'Wall' and la.norm(v) < RVis / mh.sqrt(2)]}
            secDirs = self.calcSectorDirections(self.calcAngleBraces(walls))
            if len(secDirs) > 0:
                maxDot = 0
                mV = np.zeros(3)
                vlen = la.norm(newV)
                for dir in secDirs:
                    v = np.array([cos(dir), sin(dir), 0])
                    curDot = dot(newV, v)
                    if curDot > maxDot:# and abs(mul(newV, v)) < la.norm(newV):
                        mV = v * curDot
                newV = mV
            # if la.norm(wallSum) > 0:
            #     newV += wallSum / la.norm(wallSum) * wallWeight 
            return self.applySpeedConstraints(newV * self.speedCoef)
        return np.zeros(3)

