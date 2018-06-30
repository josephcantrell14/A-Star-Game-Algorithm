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
from mycreatepathnetwork import *
from mynavigatorhelpers import *
from Queue import PriorityQueue


###############################
### AStarNavigator
###
### Creates a path node network and implements the FloydWarshall all-pairs shortest-path algorithm to create a path to the given destination.

class AStarNavigator(NavMeshNavigator):

    def __init__(self):
        NavMeshNavigator.__init__(self)


    ### Create the pathnode network and pre-compute all shortest paths along the network.
    ### self: the navigator object
    ### world: the world object
    def createPathNetwork(self, world):
        self.pathnodes, self.pathnetwork, self.navmesh = myCreatePathNetwork(world, self.agent)
        return None

    ### Finds the shortest path from the source to the destination using A*.
    ### self: the navigator object
    ### source: the place the agent is starting from (i.e., it's current location)
    ### dest: the place the agent is told to go to
    def computePath(self, source, dest):
        ### Make sure the next and dist matricies exist
        if self.agent != None and self.world != None:
            self.source = source
            self.destination = dest
            ### Step 1: If the agent has a clear path from the source to dest, then go straight there.
            ###   Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
            ###   Tell the agent to move to dest
            ### Step 2: If there is an obstacle, create the path that will move around the obstacles.
            ###   Find the pathnodes closest to source and destination.
            ###   Create the path by traversing the self.next matrix until the pathnode closes to the destination is reached
            ###   Store the path by calling self.setPath()
            ###   Tell the agent to move to the first node in the path (and pop the first node off the path)
            if clearShot(source, dest, self.world.getLines(), self.world.getPoints(), self.agent):
                self.agent.moveToTarget(dest)
            else:
                start = findClosestUnobstructed(source, self.pathnodes, self.world.getLinesWithoutBorders())
                end = findClosestUnobstructed(dest, self.pathnodes, self.world.getLinesWithoutBorders())
                if start != None and end != None:
                    newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates())
                    closedlist = []
                    path, closedlist = astar(start, end, newnetwork)
                    if path is not None and len(path) > 0:
                        path = shortcutPath(source, dest, path, self.world, self.agent)
                        self.setPath(path)
                        if self.path is not None and len(self.path) > 0:
                            first = self.path.pop(0)
                            self.agent.moveToTarget(first)
        return None

    ### Called when the agent gets to a node in the path.
    ### self: the navigator object
    def checkpoint(self):
        myCheckpoint(self)
        return None

    ### This function gets called by the agent to figure out if some shortcutes can be taken when traversing the path.
    ### This function should update the path and return True if the path was updated.
    def smooth(self):
        return mySmooth(self)

    def update(self, delta):
        myUpdate(self, delta)


def unobstructedNetwork(network, worldLines):
    newnetwork = []
    for l in network:
        hit = rayTraceWorld(l[0], l[1], worldLines)
        if hit == None:
            newnetwork.append(l)
    return newnetwork




def astar(init, goal, network):
    path = []
    closed = []
    ### YOUR CODE GOES BELOW HERE ###
    open = PriorityQueue()
    openList = []
    parent = {}
    cost = {}
    openList.append(init)
    parent[init] = init
    open.put((0, init))
    cost[init] = 0
    current = ("cost", "node")
    while current[1] != goal and not open.empty():
        current = open.get()
        currentParent = parent[current[1]]
        closed.append(current[1])
        cost[current[1]] = cost[currentParent] - heuristic(currentParent, goal) + distance(currentParent, current[1]) + heuristic(current[1], goal)
        successors = getSuccessors(current[1], network)
        for x in successors:
            xCost = cost[current[1]] - heuristic(current[1], goal) + distance(current[1], x) + heuristic(x, goal)
            if x == goal:
                parent[x] = current[1]
                current = (xCost, x)
                break
            if x not in openList:
                parent[x] = current[1]
                cost[x] = xCost
                open.put((xCost, x))
                openList.append(x)
            elif x in closed and xCost < cost[x]:
                parent[x] = current[1]
                cost[x] = xCost
    # goal state, generate the path network from current
    current = current[1]
    while current != init:
        path.append(current)
        current = parent[current]
    path.append(init)
    path.reverse()
    ### YOUR CODE GOES ABOVE HERE ###
    return path, closed

def heuristic(init, goal):
    return distance(init, goal)

def getSuccessors(node, network):
    successors = []
    for line in network:
        if node == line[0]:
            successors.append(line[1])
        elif node == line[1]:
            successors.append(line[0])
    return successors




def myUpdate(nav, delta):
    ### YOUR CODE GOES BELOW HERE ###
    worldPoints = nav.world.getPoints()
    worldLines = nav.world.getLinesWithoutBorders()
    if nav.agent.moveTarget:
        bisect = (nav.agent.position, nav.agent.moveTarget)
        for point in worldPoints:
            if minimumDistance(bisect, point) <= nav.agent.maxradius + 1:
                nav.computePath(nav.agent.position, nav.pathnodes[-1])
        for line in worldLines:
            if nav.agent.moveTarget:
                if minimumDistance(line, nav.agent.moveTarget) <= nav.agent.maxradius + 1:
                    nav.computePath(nav.agent.position, nav.pathnodes[-1])
                if minimumDistance(line, nav.agent.position) <= 1 + nav.agent.maxradius:
                    nav.agent.stopMoving()
        ray = None
        if nav.agent.moveTarget:
            ray = rayTraceWorldNoEndPoints(nav.agent.position, nav.agent.moveTarget, nav.world.getLinesWithoutBorders())
        if ray and ray != nav.agent.position:
            nav.computePath(nav.agent.position, nav.pathnodes[-1])
        if len(nav.pathnodes) == 0:
            nav.agent.stopMoving()
            print "stop"
    ### YOUR CODE GOES ABOVE HERE ###
    return None



def myCheckpoint(nav):
    ### YOUR CODE GOES BELOW HERE ###

    ### YOUR CODE GOES ABOVE HERE ###
    return None


### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
    ### YOUR CODE GOES BELOW HERE ###
    bisect = (p1, p2) #bisect between points
    for point in worldPoints:
        if minimumDistance(bisect, point) <= agent.getMaxRadius() + 1:
            return False
    ray = rayTraceWorldNoEndPoints(p1, p2, worldLines)
    if ray is not None and ray != p1 and ray != p2:
        return False
    return True
    ### YOUR CODE GOES ABOVE HERE ###
    return False

