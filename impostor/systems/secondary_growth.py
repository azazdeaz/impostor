import random
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform._rotation import Rotation

import impostor.components as comp
from impostor.plant import Entity, Plant
from impostor.utils import NormalDistribution


