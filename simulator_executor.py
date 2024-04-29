import sys
import simulator_base_constants
import simulator_base_classes
import simulator_base_functions
import os
import metrics
from metrics import *
from scene_preparer import spawnAgents, drawSceneWalls
from simulator_base_constants import *
from simulator_base_classes import *
from simulator_base_functions import *

def runSimulator(sceneGenerator, algorithm, metricsWriter, width = DEFAULT_WIDTH, height = DEFAULT_HEIGHT, O = DEFAULT_O):
    if drawing:
        #При визуализации запускаемся только один раз
        simulationCount = 1
        if saving:
            dirPath = "Experiment as scene " + sceneGenerator.getName() + " with algorithm " + algorithm.getName() + " robots count " +str(rCnts[0])
            os.makedirs(dirPath)
    else:
        simulationCount = len(rCnts)
    
    #Simulation initialization
    if drawing:
        pygame.init()
        clock = pygame.time.Clock()
        sc = pygame.display.set_mode((width , height))
        sc.fill(BGCOLOR)
    
    while simulationCount > 0:
        simulationCount = simulationCount - 1
        rCnt = rCnts[simulationCount]
        #Robots placement at the beggining of the corridor
        objs = [Robot(np.array([0, 0, 0]), color = tupleIntMul(1/rCnt, tupleSum(tupleIntMul(j, MINROBOTCOLOR), tupleIntMul((rCnt - j), MAXROBOTCOLOR))), Dir = np.array([1, 0]), r = 20) for j in range(0, rCnt)]
        objectsDescriptor, anchorCnt = sceneGenerator.generate(objs)
        spawnAgents(objs, sceneGenerator.getSpawnPoint(), rCnt)
        algorithm.algorithmSetup(rCnt)
        
        sm = Scene(objs, O)
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
                pygame.display.update()
            #Костыль для SSND
            if hasattr(algorithm, 'calcEligibility'):
                for j in range(0, rCnt):
                    if objs[j].isLive():
                        algorithm.calcEligibility(objs, j, rCnt)
            #Все перемещения происходят здесь
            sumV = 0
            for j in range(0, rCnt):
                if objs[j].isLive():
                    newV = algorithm.calcSpeed(objs, j, rCnt)
                    objs[j].setV(newV)                            
                    sumV += la.norm(newV)
                    energy += la.norm(newV) / 100
            #Проверка завершения эксперимента и сбор метрик
            if checkTerminateCondition(running, maxrunning, sumV, rCnt, maxV):
                robotAreaCnt, workAreaCnt, sensorCoverageUniformity, uniformity = calculateMetrics(objs, rCnt, objectsDescriptor, RVis, running)
                metricsWriter(rCnt, running * 0.02, energy, np.mean([la.norm(objs[k].getPos() - startPoints[k]) for k in range(0, rCnt)])/100, robotAreaCnt / workAreaCnt * 100, uniformity, sensorCoverageUniformity)
                running = maxrunning
            #Стирание роботов и прорисовка границ объектов
            if drawing:
                if saving:
                    pygame.image.save(sc, dirPath + "/Frame " + str(running).zfill(6) + ".png")
                for j in range(0, rCnt):
                    if objs[j].isLive():
                        objs[j].hide(sc, O)
                if wallsDrawing:
                    drawSceneWalls(sc, O, objectsDescriptor)
    
            sm.action()
            
            running = running + 1
            if drawing:
                SleepTime = SForFrame + StartTime - time.time()
                if SleepTime > 0:
                    time.sleep(SleepTime)
        if drawing:
            pygame.quit()