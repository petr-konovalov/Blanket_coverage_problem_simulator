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
    def getName(self):
        pass

class DiscreteTimeSpeedConstraintsAlgorithm(Algorithm):
    def __init__(self, e = 0.52, Olim = 10, Slim = 10):
        self.e = e
        self.Olim = Olim
        self.Slim = Slim
        self.Ocnt = 0
        self.Scnt = 0
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

class SWARMAlgorithm(DiscreteTimeSpeedConstraintsAlgorithm, CommunicationRequiredAlgorithm):
    def __init__(self):
        super().__init__()
        self.K = (mh.sqrt(3) * RVis * 0.5) ** 3
        self.w1 = 0.4
        self.w2 = 0.4
        self.w3 = 1 - self.w1 - self.w2
        self.state = 'ordinary'
        
    def getResponse(self):
        return self.prevSpeed

    def getName(self):
        return 'SWARM'
    
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

class VFASFAlgorithm(DynamicContextRequiredAlgorithm):
    def __init__(self):
        self.kap = 15/2500
        self.Fcentr = 0.005/2500
        self.gamma = 7.75/50
        self.speed = np.zeros(3)

    def getName(self):
        return 'VFASF'
        
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
        N = [[la.norm(vec), vec] for vec in visibleObjects['Positions']]
        print(N)
        N.sort()
        for k, dst, vec in enumerate(N):
            check = True
            for d, v in N[:k]:
                if dot(vec, v) / (dst * d) >= 0.5:
                    check = False
                    break
            if check:
                force += self.kap * (dst - RVis/mh.sqrt(3)) * vec / dst
        self.speed = self.applySpeedConstraints((self.speed + force)/(1 + self.gamma))
        return self.speed

class CSAAlgorithm(Algorithm):
    def __init__(self, sectorCount = 6, speedCoef = 20):
        self.secCnt = sectorCount
        self.speedCoef = speedCoef
    
    def getName(self):
        return 'CSA'
        
    def calcSpeed(self, visibleObjects):
        wSecSum = np.zeros([self.secCnt])
        secDir = [normir(np.array([cos(k*pi/self.secCnt*2)+cos((k+1)*pi/self.secCnt*2),
                                      sin(k*pi/self.secCnt*2)+sin((k+1)*pi/self.secCnt*2), 0])) for k in range(0, self.secCnt)]
        secNearest = np.array([s * RVis for s in secDir])
        for vec in visibleObjects['Positions']:
            nvec = la.norm(vec)
            ang = mh.atan2(mul(np.array([1, 0]), vec), dot(np.array([1, 0]), vec))
            angId = int(ang / pi * self.secCnt*0.5+self.secCnt)%self.secCnt
            angLeftId = (angId + self.secCnt - 1) % self.secCnt
            angRightId = (angId + 1) % self.secCnt
            angLeft = angId * 2 * pi / self.secCnt
            angRight = angLeft + 2 * pi / self.secCnt
            vecLeft = np.array([cos(angLeft), sin(angLeft), 0])
            vecRight = np.array([cos(angRight), sin(angRight), 0])
            dL = abs(mul(vecLeft, vec))
            dR = abs(mul(vecRight, vec))
            leftProj = (secDir[angLeftId] * RVis * dL + vecLeft * nvec * dR) / (dR + dL)
            rightProj = (secDir[angRightId] * RVis * dR + vecRight * nvec * dL) / (dR + dL)
            leftProj = normir(leftProj) * max(nvec, min(la.norm(leftProj), RVis))
            rightProj = normir(rightProj) * max(nvec, min(la.norm(rightProj), RVis))
            if nvec > 0: 
                cD = mul(secDir[angId], vec)
                rD = RVis - nvec
                if max(rD, 0.01) < abs(cD):
                    vec = vec + np.array([secDir[angId][1], 
                                          -secDir[angId][0], 0]) * cD / abs(cD) * (abs(cD) - rD)
                    vec = vec / la.norm(vec) * nvec
                if wSecSum[angId] == 0:
                    secNearest[angId, :] = np.zeros(3)
                w = (RVis - nvec) ** 2
                wSecSum[angId] += w 
                secNearest[angId, :] += vec * w
            nLeftProj = la.norm(leftProj)
            if nLeftProj > 0:
                cD = mul(secDir[angLeftId], leftProj)
                rD = RVis - nLeftProj
                if max(rD, 0.01) < abs(cD):
                    leftProj = leftProj + np.array([secDir[angLeftId][1], 
                                                    -secDir[angLeftId][0], 0]) * cD / abs(cD) * (abs(cD) - rD)
                    leftProj = leftProj / la.norm(leftProj) * nLeftProj
                if wSecSum[angLeftId] == 0:
                    secNearest[angLeftId, :] = np.zeros(3)
                w = (RVis - la.norm(leftProj)) ** 2
                wSecSum[angLeftId] += w
                secNearest[angLeftId, :] += leftProj * w
            nRightProj = la.norm(rightProj)
            if nRightProj > 0:
                cD = mul(secDir[angRightId], rightProj)
                rD = RVis - nRightProj
                if max(rD, 0.01) < abs(cD):
                    rightProj = rightProj + np.array([secDir[angRightId][1], 
                                                      -secDir[angRightId][0], 0]) * cD / abs(cD) * (abs(cD) - rD)
                    rightProj = rightProj / la.norm(rightProj) * nRightProj
                if wSecSum[angRightId] == 0:
                    secNearest[angRightId, :] = np.zeros(3)
                w = (RVis - la.norm(rightProj)) ** 2
                wSecSum[angRightId] += w
                secNearest[angRightId, :] += rightProj * w
        z = wSecSum > 0
        secNearest[z] = (secNearest[z].T / wSecSum[z]).T
        meanNearest = sum([la.norm(S) for S in secNearest]) / self.secCnt
        return self.applySpeedConstraints(self.speedCoef * 
            sum([-S / la.norm(S) * (1 - la.norm(S) / meanNearest) for S in secNearest  if la.norm(S) > 0]))

class SAAlgorithm(Algorithm):
    def __init__(self, sectorCount = 6, speedCoef = 12):
        self.secCnt = sectorCount
        self.speedCoef = 12
        
    def getName(self):
        return 'SA'
        
    def calcSpeed(self, visibleObjects):
        secNearest = np.array([RVis * normir(np.array([cos(k*pi/self.secCnt*2)+cos((k+1)*pi/self.secCnt*2),
                                                       sin(k*pi/self.secCnt*2)+sin((k+1)*pi/self.secCnt*2), 0])) 
                               for k in range(0, self.secCnt)])
        for v in visibleObjects['Positions']:
            ang = mh.atan2(mul(np.array([1, 0]), v), dot(np.array([1, 0]), v))
            angId = int(ang / pi * self.secCnt*0.5+self.secCnt)%self.secCnt
            if la.norm(secNearest[angId, :]) > la.norm(v):
                secNearest[angId, :] = v
        meanNearest = sum([la.norm(S) for S in secNearest]) / self.secCnt
        return self.applySpeedConstraints(self.speedCoef * 
            sum([-S / la.norm(S) * (1 - la.norm(S) / meanNearest) for S in secNearest]))

class DSSAAlgorithm(DiscreteTimeSpeedConstraintsAlgorithm, InitialContextRequiredAlgorithm):
    def __init__(self, A):
        super().__init__()
        self.A = A
        self.state = 'ordinary'
        
    def setup(self, objects, agentsCount, obstacles):
        self.mu = agentsCount * 3.14 * RVis ** 2 / self.A
    
    def getName(self):
        return 'DSSA'

    def partialForce(self, D, vec, dst):
        return -(D / (self.mu ** 2)) * (RVis - dst) * vec / dst
            
    def calcSpeed(self, visibleObjects):
        newV = np.array([0.0, 0.0, 0.0])
        D = 0
        if self.state == 'ordinary':
            D = len(visibleObjects['Positions'])
            for vec in visibleObjects['Positions']:
                newV += self.partialForce(D, vec, la.norm(vec)) 
        return self.applySpeedConstraints(newV)

class SODAAlgorithm(DSSAAlgorithm):
    def getName(self):
        return 'SODA'
    
    def partialForce(self, D, vec, dst):
        return -(D / (self.mu * (D if D > self.mu else self.mu))) * (RVis - dst) * vec / dst

class SSNDAlgorithm(DSSAAlgorithm):
    def __init__(self, A, alpha = 1):
        super().__init__(A)
        self.alpha = alpha
        
    def getName(self):
        return 'SSND'
        
    def algorithmSetup(self, rCnt):
        super().algorithmSetup(rCnt)
        self.M = np.zeros(rCnt)
        self.F = np.zeros([rCnt, 3])
        self.Elig = np.zeros([rCnt])

    def calcEligibility(self, objs, agentId, rCnt):
        self.F[agentId] = 0
        self.Elig[agentId] = 0
        if objs[agentId].getState() == 'ordinary':
            D = 0
            for k in range(0, rCnt):
                dst = la.norm(objs[k].getPos() - objs[agentId].getPos())
                if 0 < dst and dst < RVis:
                    D += 1
            for k in range(0, rCnt + anchorCnt):
                vec = objs[k].getPos() - objs[agentId].getPos()
                dst = la.norm(vec)
                if dst > 0 and dst < RVis:
                    self.F[agentId] += self.partialForce(5 if k >= rCnt else 1, D, vec, dst)
            Nlf = 0
            for k in range(0, rCnt): 
                vec = objs[k].getPos() - objs[agentId].getPos()
                dst = la.norm(vec)
                if dst > 0 and dst < RVis:
                    if la.norm(self.F[agentId]) >= la.norm(self.F[k]):
                        Nlf += 1
            Felig = Nlf - self.M[agentId]
            self.Elig[agentId] = (self.alpha * Felig - abs(self.mu - D)) 
        
    def calcSpeed(self, objs, agentId, rCnt):
        newV = np.array([0.0, 0.0, 0.0])
        mElig = -maxrunning
        for k in range(0, rCnt): 
            dst = la.norm(objs[k].getPos() - objs[agentId].getPos())
            if dst > 0 and dst < RVis:
                mElig = max(mElig, self.Elig[k])
        if self.Elig[agentId] >= mElig:
            newV = self.F[agentId]
            self.M[agentId] += 1
            if la.norm(objs[agentId].getPos() - self.prevPos[agentId]) < self.e:
                self.Scnt[agentId] += 1
            if self.Scnt[agentId] >= self.Slim:
                newV = np.array([0.0, 0.0, 0.0])
                objs[agentId].setState('stop')
            nextPos = objs[agentId].getPos() + newV
            if la.norm(nextPos - self.prevPos[agentId]) < self.e:
                self.Ocnt[agentId] += 1
            if self.Ocnt[agentId] >= self.Olim:
                newV = newV / 2
                objs[agentId].setState('stop')
            self.prevPos[agentId] = objs[agentId].getPos()
        else:
            newV = np.array([0.0, 0.0, 0.0])
        absV = la.norm(newV)
        if absV > maxV:
            newV = newV / absV * maxV
        return newV