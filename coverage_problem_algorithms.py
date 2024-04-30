import simulator_base_constants
from simulator_base_constants import *
import simulator_base_functions
from simulator_base_functions import *
from math import pi, sin, cos

class Algorithm:
    def algorithmSetup(self, rCnt):
        pass
    def calcSpeed(self, visibleObjects):
        pass
    def getName(self):
        pass

class SWARMAlgorithm(Algorithm):
    def __init__(self):
        self.K = (mh.sqrt(3) * RVis * 0.5) ** 3
        self.w1 = 0.4
        self.w2 = 0.4
        self.w3 = 1 - self.w1 - self.w2
        self.e = 0.26*100/50
        self.Olim = 10
        self.Slim = 10

    def getName(self):
        return 'SWARM'

    def algorithmSetup(self, rCnt):
        self.Ocnt = np.zeros(rCnt)
        self.Scnt = np.zeros(rCnt)
        self.prevF = np.zeros([rCnt, 3])
        self.prevPos = np.zeros([rCnt, 3])
    
    def calcSpeed(self, objs, agentId, rCnt):
        N = calcN(objs, agentId) #Visible robots set
        Fsep = np.array([0.0, 0.0, 0.0])
        Fcoh = np.array([0.0, 0.0, 0.0])
        Falig = np.array([0.0, 0.0, 0.0])
        Fobst = np.array([0.0, 0.0, 0.0])
        cnt = 0
        if objs[agentId].getState() == 'ordinary':
            for k in range(0, rCnt):
                vec = objs[agentId].getPos() - objs[k].getPos()
                dst = la.norm(vec)
                if dst > 0 and dst < RVis:
                    Fsep += self.K * vec / dst ** 3
                    Fcoh += vec
                    Falig += self.prevF[k]
                    cnt += 1
            if cnt > 0:     
                Fsep /= cnt
                Fcoh /= cnt
                Falig /= cnt
            cnt = 0
            for k in range(rCnt, rCnt + anchorCnt):
                vec = objs[agentId].getPos() - objs[k].getPos()
                dst = la.norm(vec)
                if dst > 0 and dst < RVis:
                    Fobst += self.K * vec / dst ** 2
                    cnt += 1
            if cnt > 0:
                Fobst /= cnt
        force = (self.w1 * (Fsep + Fobst) + self.w2 * Fcoh + self.w3 * Falig)
        if la.norm(objs[agentId].getPos() - self.prevPos[agentId]) < self.e:
            self.Scnt[agentId] += 1
        if self.Scnt[agentId] >= self.Slim:
            force = np.array([0.0, 0.0, 0.0])
            objs[agentId].setState('stop')                            
        nextPos = objs[agentId].getPos() + force
        if la.norm(nextPos - self.prevPos[agentId]) < self.e:
            self.Ocnt[agentId] += 1
        if self.Ocnt[agentId] >= self.Olim:
            force = force / 2
            objs[agentId].setState('stop')
        self.prevPos[agentId] = objs[agentId].getPos()
        self.prevF[agentId] = force
        newV = force 
        absV = la.norm(newV)
        if absV > maxV:
            newV = newV / absV * maxV  
        return newV

class VFASFAlgorithm(Algorithm):
    def __init__(self):
        self.kap = 15/2500
        self.Fcentr = 0.005/2500
        self.gamma = 7.75/50

    def getName(self):
        return 'VFASF'

    def algorithmSetup(self, rCnt):
        self.newV = np.zeros([rCnt, 3])
        
    def calcSpeed(self,objs, agentId, rCnt):
        force = -self.Fcentr * objs[agentId].getPos()
        D = 0
        if objs[agentId].getState() == 'ordinary':
            N = []
            for k in range(0, rCnt + anchorCnt):
                vec = (objs[k].getPos() - objs[agentId].getPos())
                dst = la.norm(vec)
                if dst > 0 and dst < RVis:
                    N.append([dst, k])
            N.sort()
            for k, n in enumerate(N):
                check = True
                vec = objs[n[1]].getPos() - objs[agentId].getPos()
                dst = la.norm(vec)
                for p in N[:k]:
                    if dot(vec, objs[p[1]].getPos() - objs[agentId].getPos())/(dst * la.norm(objs[p[1]].getPos() - objs[agentId].getPos())) >= 0.5:
                        check = False
                        break
                if check:
                    force += self.kap * (dst - RVis/mh.sqrt(3)) * vec / dst
        self.newV[agentId] = (self.newV[agentId] + force)/(1 + self.gamma)
        vnorm = la.norm(self.newV[agentId])
        if la.norm(self.newV[agentId]) < maxV * 0.01:
            self.newV[agentId] = np.zeros(3)
        absV = la.norm(self.newV[agentId])
        if absV > maxV:
            self.newV[agentId] = self.newV[agentId] / absV * maxV
        return self.newV[agentId]

class CSAAlgorithm(Algorithm):
    def __init__(self, sectorCount = 6):
        self.sectorCount = sectorCount
    
    def getName(self):
        return 'CSA'
        
    def calcSpeed(self, objs, agentId, rCnt):
        self.sumInSector = np.zeros([self.sectorCount, 3])
        self.cntInSector = np.zeros([self.sectorCount, 1])
        self.wSumInSector = np.zeros([self.sectorCount])
        self.sectorDir = [np.array([cos(k*pi/self.sectorCount*2)+cos((k+1)*pi/self.sectorCount*2),
                                    sin(k*pi/self.sectorCount*2)+sin((k+1)*pi/self.sectorCount*2), 0]) for k in range(0, self.sectorCount)]
        self.sectorDir = [s/la.norm(s) for s in self.sectorDir]
        self.nearestInSector = np.array([s * RVis for s in self.sectorDir])
        for k in range(0, rCnt + anchorCnt):
            vec = objs[k].getPos() - objs[agentId].getPos()
            nvec = la.norm(vec)
            if k != agentId and nvec < RVis:
                ang = mh.atan2(mul(np.array([1, 0]), vec), dot(np.array([1, 0]), vec))
                angId = int(ang / pi * self.sectorCount*0.5+self.sectorCount)%self.sectorCount
                angLeftId = (angId + self.sectorCount - 1) % self.sectorCount
                angRightId = (angId + 1) % self.sectorCount
                angLeft = angId * 2 * pi / self.sectorCount
                angRight = angLeft + 2 * pi / self.sectorCount
                vecLeft = np.array([cos(angLeft), sin(angLeft), 0])
                vecRight = np.array([cos(angRight), sin(angRight), 0])
                dL = abs(mul(vecLeft, vec))
                dR = abs(mul(vecRight, vec))
                leftProj = (self.sectorDir[angLeftId] * RVis * dL + vecLeft * nvec * dR) / (dR + dL)
                rightProj = (self.sectorDir[angRightId] * RVis * dR + vecRight * nvec * dL) / (dR + dL)
                leftProj = leftProj / la.norm(leftProj) * max(nvec, la.norm(leftProj))
                rightProj = rightProj / la.norm(rightProj) * max(nvec, la.norm(rightProj))
                self.sumInSector[angId] += vec
                self.cntInSector[angId] += 1    
                if nvec > 0: 
                    cD = mul(self.sectorDir[angId], vec)
                    rD = RVis - nvec
                    if rD < abs(cD):
                        vec = vec + np.array([self.sectorDir[angId][1], 
                                              -self.sectorDir[angId][0], 0]) * cD / abs(cD) * (abs(cD) - rD)
                        vec = vec / la.norm(vec) * nvec
                    if self.wSumInSector[angId] == 0:
                        self.nearestInSector[angId, :] = np.zeros(3)
                    w = (RVis - nvec) ** 2
                    self.wSumInSector[angId] += w 
                    self.nearestInSector[angId, :] += vec * w
                nLeftProj = la.norm(leftProj)
                if nLeftProj > 0:
                    cD = mul(self.sectorDir[angLeftId], leftProj)
                    rD = RVis - nLeftProj
                    if rD < abs(cD):
                        leftProj = leftProj + np.array([self.sectorDir[angLeftId][1], 
                                                        -self.sectorDir[angLeftId][0], 0]) * cD / abs(cD) * (abs(cD) - rD)
                        leftProj = leftProj / la.norm(leftProj) * nLeftProj
                    if self.wSumInSector[angLeftId] == 0:
                        self.nearestInSector[angLeftId, :] = np.zeros(3)
                    w = (RVis - la.norm(leftProj)) ** 2
                    self.wSumInSector[angLeftId] += w
                    self.nearestInSector[angLeftId, :] += leftProj * w
                nRightProj = la.norm(rightProj)
                if nRightProj > 0:
                    cD = mul(self.sectorDir[angRightId], rightProj)
                    rD = RVis - nRightProj
                    if rD < abs(cD):
                        rightProj = rightProj + np.array([self.sectorDir[angRightId][1], 
                                                          -self.sectorDir[angRightId][0], 0]) * cD / abs(cD) * (abs(cD) - rD)
                        rightProj = rightProj / la.norm(rightProj) * nRightProj
                    if self.wSumInSector[angRightId] == 0:
                        self.nearestInSector[angRightId, :] = np.zeros(3)
                    w = (RVis - la.norm(rightProj)) ** 2
                    self.wSumInSector[angRightId] += w
                    self.nearestInSector[angRightId, :] += rightProj * w
        nSum = np.array([0.0, 0.0, 0.0])
        nRevSum = np.array([0.0, 0.0, 0.0])
        nCnt = 0
        for k in range(0, self.sectorCount):
            if la.norm(self.nearestInSector[k]) > 0:
                if self.wSumInSector[k] > 0:
                    self.nearestInSector[k] /= self.wSumInSector[k]
                nSum += la.norm(self.nearestInSector[k])
                nCnt += 1
        for k in range(0, self.sectorCount):
            if la.norm(self.nearestInSector[k]) > 0:                            
                nRevSum += -self.nearestInSector[k] / la.norm(self.nearestInSector[k]) * (nSum/nCnt - la.norm(self.nearestInSector[k]))/(nSum/nCnt)
        newV = nRevSum * 20 
        absV = la.norm(newV)
        if absV > maxV:
            newV = newV / absV * maxV
        return newV

class SAAlgorithm(Algorithm):
    def __init__(self, sectorCount = 6):
        self.secCnt = sectorCount
        
    def getName(self):
        return 'SA'
        
    def calcSpeed(self, visibleObjects):
        sectorDir = [normir(np.array([cos(k*pi/self.secCnt*2)+cos((k+1)*pi/self.secCnt*2),
                                      sin(k*pi/self.secCnt*2)+sin((k+1)*pi/self.secCnt*2), 0])) for k in range(0, self.secCnt)]
        secNearest = np.array([s * RVis for s in sectorDir])
        for v in visibleObjects:
            ang = mh.atan2(mul(np.array([1, 0]), v), dot(np.array([1, 0]), v))
            angId = int(ang / pi * self.secCnt*0.5+self.secCnt)%self.secCnt
            if la.norm(secNearest[angId, :]) > la.norm(v):
                secNearest[angId, :] = v
        meanNearest = sum([la.norm(S) for S in secNearest]) / self.secCnt
        newV = 12 * sum([-S / la.norm(S) * (1 - la.norm(S) / meanNearest) for S in secNearest])
        absV = la.norm(newV)
        if absV > maxV:
            newV = newV / absV * maxV
        return newV

class DSSAAlgorithm(Algorithm):
    def __init__(self, A):
        self.e = 0.26*100/50
        self.Olim = 10
        self.Slim = 10
        self.A = A
        
    def getName(self):
        return 'DSSA'
        
    def algorithmSetup(self, rCnt):
        self.Ocnt = np.zeros(rCnt)
        self.Scnt = np.zeros(rCnt)
        self.prevPos = np.zeros([rCnt, 3])
        self.mu = rCnt * 3.14 * (RVis) ** 2 / self.A

    def partialForce(self, C, D, vec, dst):
        return -C * (D / (self.mu ** 2)) * (RVis - dst) * vec / dst
        
    def calcSpeed(self, objs, agentId, rCnt):
        newV = np.array([0.0, 0.0, 0.0])
        D = 0
        if objs[agentId].getState() == 'ordinary':
            for k in range(0, rCnt):
                dst = la.norm(objs[k].getPos() - objs[agentId].getPos())
                if dst < RVis:
                    D += 1
            for k in range(0, rCnt + anchorCnt):
                vec = (objs[k].getPos() - objs[agentId].getPos())
                dst = la.norm(vec)
                if dst > 0 and dst < RVis:
                    newV += self.partialForce(5 if k >= rCnt else 1, D, vec, dst) 
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
            absV = la.norm(newV)
            if absV > maxV:
                newV = newV / absV * maxV
        return newV

class SODAAlgorithm(DSSAAlgorithm):
    def getName(self):
        return 'SODA'
    
    def partialForce(self, C, D, vec, dst):
        return -C * (D / (self.mu * (D if D > self.mu else self.mu))) * (RVis - dst) * vec / dst

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
        