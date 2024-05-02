import simulator_base_constants
from simulator_base_constants import *
import simulator_base_functions
from simulator_base_functions import *
from math import pi, sin, cos

class Algorithm:
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
        for vec in visibleObjects:
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
        newV = self.speedCoef * sum([-S / la.norm(S) * (1 - la.norm(S) / meanNearest) for S in secNearest  if la.norm(S) > 0])
        absV = la.norm(newV)
        if absV > maxV:
            newV = newV / absV * maxV
        return newV

class SAAlgorithm(Algorithm):
    def __init__(self, sectorCount = 6, speedCoef = 12):
        self.secCnt = sectorCount
        self.speedCoef = 12
        
    def getName(self):
        return 'SA'
        
    def calcSpeed(self, visibleObjects):
        secNearest = np.array([RVis * normir(np.array([cos(k*pi/self.secCnt*2)+cos((k+1)*pi/self.secCnt*2),
                                      sin(k*pi/self.secCnt*2)+sin((k+1)*pi/self.secCnt*2), 0])) for k in range(0, self.secCnt)])
        for v in visibleObjects:
            ang = mh.atan2(mul(np.array([1, 0]), v), dot(np.array([1, 0]), v))
            angId = int(ang / pi * self.secCnt*0.5+self.secCnt)%self.secCnt
            if la.norm(secNearest[angId, :]) > la.norm(v):
                secNearest[angId, :] = v
        meanNearest = sum([la.norm(S) for S in secNearest]) / self.secCnt
        newV = self.speedCoef * sum([-S / la.norm(S) * (1 - la.norm(S) / meanNearest) for S in secNearest])
        absV = la.norm(newV)
        if absV > maxV:
            newV = newV / absV * maxV
        return newV

class DSSAAlgorithm(Algorithm):
    def __init__(self, rCnt, A):
        self.e = 0.26*100/50
        self.Olim = 10
        self.Slim = 10
        self.state = 'ordinary'
        self.Ocnt = 0
        self.Scnt = 0
        self.prevSpeed = np.zeros(3)
        self.A = A
        self.mu = rCnt * 3.14 * RVis ** 2 / A
    
    def getName(self):
        return 'DSSA'

    def partialForce(self, D, vec, dst):
        return -(D / (self.mu ** 2)) * (RVis - dst) * vec / dst
        
    def calcSpeed(self, visibleObjects):
        newV = np.array([0.0, 0.0, 0.0])
        D = 0
        if self.state == 'ordinary':
            D = len(visibleObjects)
            for vec in visibleObjects:
                newV += self.partialForce(D, vec, la.norm(vec)) 
            absV = la.norm(newV)
            if absV > maxV:
                newV = newV / absV * maxV
            if la.norm(self.prevSpeed) < self.e:
                self.Scnt += 1
            if self.Scnt >= self.Slim:
                newV = np.zeros(3)
                self.state = 'stop'
            if la.norm(newV + self.prevSpeed) < self.e:
                self.Ocnt += 1
            if self.Ocnt >= self.Olim:
                newV = newV / 2
                self.state = 'stop'
            self.prevSpeed = newV
        return newV

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