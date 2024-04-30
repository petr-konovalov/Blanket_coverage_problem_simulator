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

def drawSceneWalls(sc, O, objectsDescriptor, color = WALLCOLOR):
    for angles in objectsDescriptor:
        for l in range(0, len(angles)):
            pntL = angles[l] * scale + O
            pntR = angles[0 if l + 1 == len(angles) else l + 1] * scale + O
            pygame.draw.line(sc, color, tuple(pntL[:2]), tuple(pntR[:2]), 10)            

def generateSceneByDescriptor(objectsDescriptor, objs, rCnt):
    correctedAnchorCount = 0
    perimeter = sum([sum([la.norm(a2 - a1) for a1, a2 in zip(a, np.roll(a, len(a[0])))]) for a in objectsDescriptor])
    for angles in objectsDescriptor:
        for a1, a2 in zip(angles, np.roll(angles, len(angles[0]))):
            pointPerEdge = int(ceil(anchorCnt / perimeter * la.norm(a2 - a1)))
            correctedAnchorCount += pointPerEdge
            for k in range(0, pointPerEdge):
                objs.append(Robot((a2*k+a1*(pointPerEdge-k))/pointPerEdge, color = WALLCOLOR, state = 'wall'))
    return correctedAnchorCount

class SceneGenerator:
    def generate(self, objs):
        pass
    def getSpawnPoint(self):
        pass
    def getName(self):
        pass

class OSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = np.array([[[-500, -500, 0], [-500,  500, 0], [ 500,  500, 0], [ 500, -500, 0]],
                                      [[-150, -150, 0], [-150,  150, 0], [ 150,  150, 0], [ 150, -150, 0]]])
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([-300, 0, 0])
    def getName(self):
        return 'O'
    
class ButterflySceneGenerator(SceneGenerator):
    def generate(self, objs, cW = 150):
        objectsDescriptor = np.array([[[-600, -400, 0], [-600,  400, 0], [   0, cW/2, 0],
                                       [ 600,  400, 0], [ 600, -400, 0], [   0,-cW/2, 0]]])
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([-300, 0, 0])
    def getName(self):
        return 'Butterfly'

class TwoBoxesSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = np.array([[[-500, -500, 0], [-500,  500, 0], [ 500,  500, 0], [ 500, -500, 0]],
                                      [[-300, -300, 0], [-300, -200, 0], [-200, -200, 0], [-200, -300, 0]],
                                      [[ 300,  300, 0], [ 300,  200, 0], [ 200,  200, 0], [ 200,  300, 0]]])
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([0, 0, 0])
    def getName(self):
        return 'TwoBoxes'

class SimpleSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = np.array([[[-500, -500, 0], [-500,  500, 0], [ 500,  500, 0], [ 500, -500, 0]]])
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([0, 0, 0])
    def getName(self):
        return 'Simple'

class ZigZagSceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = np.array([[[-700,  500, 0], [-300,  500, 0], [ 300, -100, 0], [ 300, 500, 0],
                                       [ 700,  500, 0], [ 700, -500, 0], [ 300, -500, 0], [-300, 100, 0],
                                       [-300, -500, 0], [-700, -500, 0]]])
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([-500, 0, 0])
    def getName(self):
        return 'ZigZag'
    
class Maze1SceneGenerator(SceneGenerator):
    def generate(self, objs):
        objectsDescriptor = [np.array([[-672, -672, 0], [-116, -684, 0], [-114, -216, 0], [-64, -216, 0], [-68, -680, 0], [658, -690, 0], [654, -292, 0], [134, -286, 0], [134, -228, 0], [950, -242, 0], [940, -306, 0], [714, -302, 0], [706, -688, 0], [2070, -690, 0], [2070, 672, 0], [1560, 644, 0], [1522, -330, 0], [1446, -324, 0], [1466, 638, 0], [224, 642, 0], [198, 24, 0], [140, 28, 0], [126, 628, 0], [-656, 648, 0], [-664, 126, 0], [-48, 112, 0], [-54, 54, 0], [-674, 52, 0]]),
 np.array([[526, 36, 0], [526, 144, 0], [1144, 130, 0], [1130, 38, 0]])]
        return objectsDescriptor, generateSceneByDescriptor(objectsDescriptor, objs, len(objs))
    def getSpawnPoint(self):
        return np.array([-400, -300, 0])
    def getName(self):
        return 'Maze1'
        
