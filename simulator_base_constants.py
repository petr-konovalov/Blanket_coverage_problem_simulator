import pygame
import numpy as np
import numpy.linalg as la
import random as rdm
import math as mh
from math import pi, sin, cos
import time
import matplotlib.pyplot as plt
import pandas as pd

WIN_WIDTH = 604
WIN_HEIGHT = 404

#задаются константы
anchorCnt = 240
eC = 100
rCnts = [35]+[50]*eC+[45]*eC+[40]*eC+[35]*eC+[30]*eC+[25]*eC+[20]*eC+[15]*eC+[10]*eC #robots count
RVis = 200 #vision radius
maxV = 8 #maximum speed
O = np.array([WIN_WIDTH * 0.5, WIN_HEIGHT * 0.5, 0])

scale = 0.5
spawnAttemtsCount = 100
spawnGrowthSpeed = 1.1

drawing = True
saving = False
anchorDrawing = False
maxrunning = 2400#2400 # 1 секунда == 50 кадров