import pygame
import numpy as np
import numpy.linalg as la
import random as rdm
import math as mh
from math import pi, sin, cos
import time
import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_WIDTH = 704
DEFAULT_HEIGHT = 704
DEFAULT_O = np.array([DEFAULT_WIDTH * 0.5, DEFAULT_HEIGHT * 0.5, 0])
DEFAULT_ANCHOR_GENERATION_COUNT = 8

#задаются константы
anchorCnt = 1
eC = 100
RVis = 200 #vision radius
maxV = 8 #maximum speed

scale = 0.5
spawnAttemtsCount = 100
spawnGrowthSpeed = 1.1

topologicalOptimization = 18
drawing = True
saving = False
anchorDrawing = True
wallsDrawing = True
maxrunning = 10000#2400 # 1 секунда == 50 кадров