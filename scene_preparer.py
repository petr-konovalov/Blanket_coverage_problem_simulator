import simulator_base_classes
from simulator_base_classes import *
import simulator_base_constants
from simulator_base_constants import *
from math import ceil

def spawnAgents(objs, spawn_point, rCnt, SpawnR = 100, botDist = 1):
    for j in range(0, rCnt):
        position = spawn_point + getRndCrcPnt(SpawnR)
        position[2] = 0
        attemptNumber = 0
        while getRobotsDist(objs, position) < botDist:
            position = spawn_point + getRndCrcPnt(SpawnR)
            position[2] = 0
            attemptNumber = attemptNumber + 1
            if attemptNumber == spawnAttemtsCount:
                attemptNumber = 0
                SpawnR = SpawnR * spawnGrowthSpeed
        objs[j].teleportation(position)

def drawSceneWalls(sc, O, objectsDescriptor):
    for angles in objectsDescriptor:
        for l in range(0, len(angles)):
            pntL = angles[l] * scale + O
            pntR = angles[0 if l + 1 == len(angles) else l + 1] * scale + O
            pygame.draw.line(sc, WALLCOLOR, (pntL[0], pntL[1]), (pntR[0], pntR[1]), 10)
    
def generateSceneByDescriptor(objectsDescriptor, objs, rCnt):
    edgesCount = sum([len(angles) for angles in objectsDescriptor])
    pointPerEdge = int(ceil(anchorCnt/edgesCount))
    for angles in objectsDescriptor:
        for j in range(0, len(angles) - 1):
            for k in range(0, pointPerEdge):
                objs.append(Robot((angles[j]*k+angles[j+1]*(pointPerEdge-k))/pointPerEdge, color = WALLCOLOR, state = 'wall'))
        for k in range(0, pointPerEdge):
            objs.append(Robot((angles[-1]*k+angles[0]*(pointPerEdge-k))/pointPerEdge, color = WALLCOLOR, state = 'wall'))
    return pointPerEdge * edgesCount

class SceneGenerator:
    def generate(self, objs):
        pass
    def getSpawnPoint(self):
        pass
    def getName(self):
        pass

class OSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = [[np.array([-500, -500, 0]), np.array([-500,  500, 0]), np.array([ 500,  500, 0]), np.array([ 500, -500, 0])],
                             [np.array([-150, -150, 0]), np.array([-150,  150, 0]), np.array([ 150,  150, 0]), np.array([ 150, -150, 0])]]
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([-300, 0, 0])
    def getName(self):
        return 'O'
    
class ButterflySceneGenerator(SceneGenerator):
    def generate(self, objs, cW = 150):
        objectsDescriptor = [[np.array([-600, -400, 0]), np.array([-600,  400, 0]), np.array([   0, cW/2, 0]),
                              np.array([ 600,  400, 0]), np.array([ 600, -400, 0]), np.array([   0,-cW/2, 0])]]
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([-300, 0, 0])
    def getName(self):
        return 'Butterfly'

class TwoBoxesSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = [[np.array([-500, -500, 0]), np.array([-500,  500, 0]), np.array([ 500,  500, 0]), np.array([ 500, -500, 0])],
                             [np.array([-300, -300, 0]), np.array([-300, -200, 0]), np.array([-200, -200, 0]), np.array([-200, -300, 0])],
                             [np.array([ 300,  300, 0]), np.array([ 300,  200, 0]), np.array([ 200,  200, 0]), np.array([ 200,  300, 0])]]
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([0, 0, 0])
    def getName(self):
        return 'TwoBoxes'

class SimpleSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = [[np.array([-500, -500, 0]), np.array([-500,  500, 0]), np.array([ 500,  500, 0]), np.array([ 500, -500, 0])]]
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([0, 0, 0])
    def getName(self):
        return 'Simple'

class ZigZagSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = [[np.array([-700,  500, 0]), np.array([-300,  500, 0]), np.array([ 300, -100, 0]), np.array([ 300, 500, 0]),
                              np.array([ 700,  500, 0]), np.array([ 700, -500, 0]), np.array([ 300, -500, 0]), np.array([-300, 100, 0]),
                              np.array([-300, -500, 0]), np.array([-700, -500, 0])]]
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([-500, 0, 0])
    def getName(self):
        return 'ZigZag'
        