import sys
import simulator_base_constants
import simulator_base_classes
import simulator_base_functions
import os
import metrics
import copy
from metrics import *
from scene_preparer import spawnAgents, drawSceneWalls
from simulator_base_constants import *
from simulator_base_classes import *
from simulator_base_functions import *
from coverage_problem_algorithms import *

def initPygame(width, height):
    pygame.init()
    clock = pygame.time.Clock()
    sc = pygame.display.set_mode((width , height))
    sc.fill(BGCOLOR)
    return sc, clock

def algorithmSetup(algorithm, objs, rCnt, obstacles):
    if issubclass(type(algorithm), InitialContextRequiredAlgorithm):
        for obj in objs[:rCnt]:
            obj.getAlgorithm().setup(objs, rCnt, obstacles)

def initScene(rCnt, sceneGenerator, algorithm, O):
    objs = [Robot(
                  np.array([0, 0, 0]), 
                  color = tupleIntMul(1/rCnt, tupleSum(tupleIntMul(j, MINROBOTCOLOR), tupleIntMul((rCnt - j), MAXROBOTCOLOR))), 
                  Dir = np.array([1, 0]), 
                  r = 15, 
                  algorithm = copy.deepcopy(algorithm)
                 )
            for j in range(0, rCnt)]
    obstacles, anchorCnt = sceneGenerator.generate(objs)
    spawnAgents(objs, sceneGenerator.getSpawnPoint(), rCnt)
    algorithmSetup(algorithm, objs, rCnt, obstacles)
    return Scene(objs, O), objs, obstacles, anchorCnt

def runSimulator(sceneGenerator, rCnt, algorithm, metricsWriter,
                 width = DEFAULT_WIDTH, 
                 height = DEFAULT_HEIGHT, 
                 O = DEFAULT_O,
                 anchor_generation_count = DEFAULT_ANCHOR_GENERATION_COUNT):
    if drawing:
        #При визуализации запускаемся только один раз
        if saving:
            dirPath = "Experiment as scene " + sceneGenerator.getName() + " with algorithm " + type(algorithm).__name__ + " robots count " +str(rCnt)
            os.makedirs(dirPath)
        sc, clock = initPygame(width, height)
    
    #Robots placement at the beggining of the corridor
    sm, objs, obstacles, anchorCnt = initScene(rCnt, sceneGenerator, algorithm, O)
    
    running = 1
    energy = 0
    startPoints = [obj.getPos() for obj in objs[:rCnt]]
    while running >= 0 and running <= maxrunning:
        #Прорисовка роботов
        if drawing:
            StartTime = time.time()
            for i in pygame.event.get():
                if i.type == pygame.QUIT: running = -10
            for j in range(0, rCnt + (anchorCnt if anchorDrawing else 0)):
                if objs[j].isLive():
                    objs[j].draw(sc, O)
            if issubclass(type(algorithm), DebugAlgorithm):
                for j in range(0, rCnt):
                    objs[j].getAlgorithm().drawDebug(sc, O, objs[j].getPos())
            if wallsDrawing:
                drawSceneWalls(sc, O, obstacles)
            pygame.display.update()
        if issubclass(type(algorithm), CommunicationRequiredAlgorithm):
            for j in range(0, rCnt):
                if objs[j].isLive():
                    objs[j].getAlgorithm().receiveData(communicationAvailableAgents(objs, j, rCnt, RVis))
        #Все перемещения происходят здесь
        sumV = 0
        for j in range(0, rCnt):
            if objs[j].isLive():
                if issubclass(type(algorithm), DynamicContextRequiredAlgorithm):
                    objs[j].getAlgorithm().update(objs, rCnt, j)
                newV = objs[j].calcSpeed(
                    topologicalOptimizationFilter(
                        measureVisibleObjects(objs, obstacles, j, rCnt, RVis, anchor_generation_count)
                    )
                )
                sumV += la.norm(newV)
                energy += la.norm(newV) / 100
        #Проверка завершения эксперимента и сбор метрик
        if checkTerminateCondition(running, maxrunning, sumV, rCnt, maxV):
            robotAreaCnt, workAreaCnt, sensorCoverageUniformity, uniformity = calculateMetrics(objs, rCnt, obstacles, RVis, running,
                                                                                              leftBound = -int(O[0] / scale),
                                                                                              rightBound = int((width - O[0]) / scale),
                                                                                              downBound = -int(O[1] / scale),
                                                                                              upBound = int((height - O[1]) / scale))
            metricsWriter(rCnt, running * 0.02, energy, np.mean([la.norm(objs[k].getPos() - startPoints[k]) for k in range(0, rCnt)])/100, robotAreaCnt / workAreaCnt * 100, uniformity, sensorCoverageUniformity)
            running = maxrunning
        #Стирание роботов и прорисовка границ объектов
        if drawing:
            if saving:
                pygame.image.save(sc, dirPath + "/Frame " + str(running).zfill(6) + ".png")
            for j in range(0, rCnt):
                if objs[j].isLive():
                    objs[j].hide(sc, O)
            if issubclass(type(algorithm), DebugAlgorithm):
                for j in range(0, rCnt):
                    objs[j].getAlgorithm().hideDebug(sc, O, objs[j].getPos())
        sm.action()
        
        running = running + 1
        if drawing:
            SleepTime = SForFrame + StartTime - time.time()
            if SleepTime > 0:
                time.sleep(SleepTime)
    if drawing:
        pygame.quit()