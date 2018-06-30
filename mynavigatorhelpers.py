'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *

### This function optimizes the given path and returns a new path
### source: the current position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the Floyd-Warshall algorithm
### world: pointer to the world
def shortcutPath(source, dest, path, world, agent):
    ### YOUR CODE GOES BELOW HERE ###
    old = len(path)
    for node2 in reversed(path):
        for node1 in path:
            if path.index(node2) <= path.index(node1) + 1:
                break
            if clearShot(node1, node2, world.getLinesWithoutBorders(), world.getPoints(), agent):
                while path.index(node2) != path.index(node1) + 1:
                    print len(path), path.index(node1), path.index(node2)
                    del path[path.index(node1) + 1]
    ### YOUR CODE GOES BELOW HERE ###
    return path


### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def mySmooth(nav):
    ### YOUR CODE GOES BELOW HERE ###
    end = nav.agent.moveTarget
    start = nav.agent.position
    if nav.agent.moveTarget:
        if clearShot(start, end, nav.world.getLinesWithoutBorders(), nav.world.getPoints(), nav.agent):
            nav.agent.moveToTarget(end)
            return True
    ### YOUR CODE GOES ABOVE HERE ###
    return False

def clearShot(p1, p2, worldLines, worldPoints, agent):
    bisect = (p1, p2) #bisect between points
    for point in worldPoints:
        if minimumDistance(bisect, point) <= agent.getMaxRadius() + 1:
            return False
    ray = rayTraceWorldNoEndPoints(p1, p2, worldLines)
    if ray is not None and ray != p1 and ray != p2:
        return False
    return True


