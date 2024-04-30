import simulator_base_constants
from simulator_base_constants import *
import simulator_base_functions
from simulator_base_functions import *

FPS = 200
SForFrame = 1 / FPS
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BRAWN = (101, 67, 33)
GRAY = (125, 125, 125)
LIGHT_BLUE = (64, 128, 255)
GREEN = (0, 128, 0)
RED = (255, 0, 0)
LIME = (0, 255, 0)
YELLOW = (225, 225, 0)
ORANGE = (255, 165, 0)
PINK = (230, 50, 230)
BLUE = (0, 0, 150)
BGCOLOR = WHITE#(210, 210, 210)#BLUE
MINROBOTCOLOR = (100, 0, 250)
MAXROBOTCOLOR = (100, 250, 0)
MINCOLORGOF = (100, 0, 0)
MAXCOLORGOF = (255, 0, 0)
MINCOLORGOB = (100, 100, 0)
MAXCOLORGOB = (255, 255, 0)
ROBOTCOLOR = (200, 200, 200)
VELCOL = (0, 0, 0)
DIRCOLOR = ORANGE
WALLCOLOR = (200, 0, 0)
LINECHARTCOLOR = GRAY

class VisualObj:
    def draw(self):
        pass

class ActiveObj(VisualObj):
    def action(self):
        pass
    
class GeomObj(ActiveObj):
    def getDist(self, oPos):
        pass
    def getXDiff(self, oPos):
        pass
    def getYDiff(self, oPos):
        pass
    def getYDist(self, oPos):
        return abs(self.getYDiff(oPos))
    def isAbove(self, oPos):
        return self.getYDiff(oPos) > 0
    
class Robot(GeomObj):
    def __init__(self, pos, V = np.array([0, 0, 0]), Rvis = 10, r = 12, color = ROBOTCOLOR, Dir = np.array([1, 0]), maxAngVel = 14, maxAcc = 0.08, maxAx = 0.05, maxAy = 0.05, Vm = 0.5, state = 'ordinary'): #maxAngVel = 14, maxAcc = 0.08
        self._dir = Dir
        self._live = True
        self._pos = pos
        self._maxAngVel = maxAngVel / 180 * mh.pi
        self._maxAcc = maxAcc
        self._V = V
        self._Vm = Vm
        self._maxAx = maxAx
        self._maxAy = maxAy
#         if la.norm(V) > 1e-2: #Костыль
#             self._V = V
#         else:
#             self._V = Dir
        self._r = r
        self._color = color
        self._state = state
    def setState(self, newState):
        self._state = newState
    def getState(self):
        return self._state
    def getColor(self):
        return self._color
    def setColor(self, color):
        self._color = color
    def rise(self):
        self._live = True
    def kill(self):
        self._live = False
    def isLive(self):
        return self._live
    def teleportation(self, tPos):
        self._pos = tPos
    def getPos(self, distortion = 0.1): #5.1
        if distortion < 1:
            return self._pos
        else:
            return self.getDistoredPos(distortion)
    def getDistoredPos(self, R = 7):
        return self._pos + getRndCrcPnt(R)
    def getDir(self):
        return self._dir
    def getOrtDir(self):
        return np.array([self._dir[1], -self._dir[0]])
    def setDir(self, nDir):
        self._dir = nDir / la.norm(nDir)
    def getDist(self, oPos):
        return la.norm(self._pos - oPos)
    def getXDiff(self, oPos):
        return self._pos[0] - oPos[0]
    def getYDiff(self, oPos):
        return self._pos[1] - oPos[1]
    def getZDiff(self, oPos):
        return self._pos[2] - oPos[2]
    def getKDiff(self, oPos, k):
        return self._pos[k] - oPos[k]
    def getKDist(self, oPos, k):
        return abs(self.getKDiff(oPos, k))
    def getDirDiff(self, oPos, Dir):
        u = self._pos - oPos
        return u[0] * Dir[0] + u[1] * Dir[1]
    def getRDirDiff(self, oPos):
        return self.getDirDiff(oPos, self.getDir())
    def getOrtRDirDiff(self, oPos):
        return self.getDirDiff(oPos, self.getOrtDir())
    def isDirAbove(self, oPos, oDir):
        return self.getDirDiff(oPos, oDir) < 0
    def isKDirAbove(self, oPos, k):
        return self.getKDiff(oPos, k) < 0
    def inRank(self, oPos, buildWidth):
        return abs(self.getXDiff(oPos)) < buildWidth
    def setV(self, V):
        self._V = V
#         disturb = 0.15
# #         A = calcA(objs, np.array([min(self._maxAx, abs(V[0]-self._V[0])), min(self._maxAy, abs(V[1]-self._V[1]))]), V, self._V)
#         A = calcA(np.array([self._maxAx, self._maxAy]), V, self._V)
#         lV = la.norm(self._V)
#         th = mh.atan2(self._V[1], self._V[0])
#         if self._V[0] >= self._Vm:
#             uv = sat(self._maxAcc, A[0]*mh.cos(th)+A[1]*mh.sin(th))
#             uth = sat(self._maxAngVel, (A[1]*mh.cos(th)-A[0]*mh.sin(th))/V[0])
#         else:
# #             uv = sat(abs(V[0]-self._V[0]), mh.copysign(self._maxAcc, V[0]-self._V[0]))
# #             uth = sat(abs(self._V[1]), mh.copysign(self._maxAngVel, -self._V[1]))
#             uv = mh.copysign(self._maxAcc, V[0]-self._V[0])
#             uth = mh.copysign(self._maxAngVel, -self._V[1])
#         uvdis = rdm.uniform(-self._maxAcc*disturb/2, self._maxAcc*disturb/2)
#         uthdis = rdm.uniform(-self._maxAngVel*disturb/2, self._maxAngVel*disturb/2)
#         self._V = (lV+uv+uvdis)*np.array([mh.cos(th+uth+uthdis), mh.sin(th+uth+uthdis)])

    def getV(self):
        return self._V
    def getY(self, X):
        return self._pos[1]
    def inN(self, RPos, RVis):
        return self.getDist(RPos) < RVis
    def isWall(self):
        return False
    
    def draw(self, sc, O):
        #DispPos = self._pos*scale + O
        #pygame.draw.circle(sc, self._color, (int(DispPos[1]), int(DispPos[2])), int(self._r*scale))
        DispPosXY = self._pos*scale + O#Oxy
        pygame.draw.circle(sc, self._color, (int(DispPosXY[0]), int(DispPosXY[1])), int(self._r*scale))
        if self._state != 'wall':
        #if self._state == 'Red':
            pygame.draw.circle(sc, self._color, (int(DispPosXY[0]), int(DispPosXY[1])), int(RVis*scale), width = 1)
#         DispPosYZ = self._pos*scale + Oyz
#         pygame.draw.circle(sc, self._color, (int(DispPosYZ[1]), int(DispPosYZ[2])), int(self._r*scale))
#         DispPosXZ = self._pos*scale + Oxz
#         pygame.draw.circle(sc, self._color, (int(DispPosXZ[0]), int(DispPosXZ[2])), int(self._r*scale))
        
    def hide(self, sc, O):
#         DispPos = self._pos*scale + O
#         pygame.draw.circle(sc, BGCOLOR, (int(DispPos[1]), int(DispPos[2])), int(3+self._r*scale))
        DispPosXY = self._pos*scale + O#Oxy
        pygame.draw.circle(sc, BGCOLOR, (int(DispPosXY[0]), int(DispPosXY[1])), int(3+self._r*scale))
        if self._state != 'wall':
        #if self._state == 'Red':
            pygame.draw.circle(sc, BGCOLOR, (int(DispPosXY[0]), int(DispPosXY[1])), int(RVis*scale), width = 3)
#         DispPosYZ = self._pos*scale + Oyz
#         pygame.draw.circle(sc, BGCOLOR, (int(DispPosYZ[1]), int(DispPosYZ[2])), int(3+self._r*scale))
#         DispPosXZ = self._pos*scale + Oxz
#         pygame.draw.circle(sc, BGCOLOR, (int(DispPosXZ[0]), int(DispPosXZ[2])), int(3+self._r*scale))
    def action(self):
        self._pos = self._pos + self._V
    
class Scene(ActiveObj):
    def __init__(self, objs, GlobalCenter = np.array([0, 0])):
        self._objects = objs
        self._O = GlobalCenter
    def draw(self, sc):
        for obj in self._objects:
            if obj.isLive():
                obj.draw(sc, self._O)
    def action(self):
        for obj in self._objects:
            if obj.isLive():
                obj.action()  
                
class LineChart:
    def __init__(self, sc, GlobalCenter = np.array([0, 0]), FirstPnt = np.array([0, 0]), color = LINECHARTCOLOR, width = 3, Scale = np.array([1, 1])):
        self._sc = sc
        self._O = GlobalCenter
        self._prevPnt = FirstPnt
        self._color = color
        self._width = width
        self._S = Scale
    def addPnt(self, nextPnt):
        pygame.draw.line(sc, self._color, tuple(self._S*self._prevPnt+self._O), tuple(self._S*nextPnt+self._O), self._width)
        self._prevPnt = nextPnt
        