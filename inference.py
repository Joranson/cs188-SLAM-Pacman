# inference.py
# ------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


import itertools
import util
import random
import game
import slam
from game import Directions


class InferenceModule:
    """
    An inference module tracks a belief distribution over walls and
    Pacman's location.
    This is an abstract class, which you should not modify.
    """

    ############################################
    # Useful methods for all inference modules #
    ############################################

    def __init__(self):
        pass

    ######################################
    # Methods that need to be overridden #
    ######################################

    def initialize(self):
        "Sets the belief state to some initial configuration."
        pass

    def observe(self, observation):
        """
        Updates beliefs based on the given distance observation and gameState.
        This combines together observe and elapseTime from the previous particle
        filtering project. You may, of course, make your own elapseTime helper
        function if you wish.
        """
        pass

    def getWallBeliefDistribution(self):
        """
        Returns the agent's current belief state about the distribution
        of walls conditioned on all evidence so far.
        """
        pass
    
    def getPositionDistribution(self):
        """
        Returns the agent's current belief state about the distribution
        of its own position conditioned on all evidence so far.
        """
        pass
    
class SLAMParticleFilter(InferenceModule):
    """
    Particle filtering inference module for use in SLAM.
    """
    
    "*** YOU MAY ADD WHATEVER HELPER METHODS YOU WANT TO THIS CLASS ***"    
    
    def __init__(self, startPos, layoutWidth, layoutHeight, wallPrior, legalPositions, numParticles=500):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        self.startPos = list(startPos)
        self.legalPositions = legalPositions
        self.layoutHeight = layoutHeight
        self.layoutWidth = layoutWidth
        self.wallPrior = wallPrior
        self.numParticles = numParticles
        self.permanant = util.Counter()

        self.particles = []
        random.shuffle(legalPositions)
        myList = itertools.cycle(legalPositions)
        self.particles = [myList.next() for i in range(0, numParticles)]
        self.particless = self.particles

        self.pos= util.Counter()
        self.wall = util.Counter()
        for p in legalPositions:
            self.pos[p] = 0.001
            self.wall[p] = 0.1
        self.pos[startPos] = 1.0
        self.wall[startPos] = 0.001
        self.permanant = util.Counter()
        self.pos.normalize()
        self.wall.normalize()

    
    def initialize(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # pass
        print 'initializinggggggg'
        self.particles = []
        random.shuffle(legalPositions)
        myList = itertools.cycle(legalPositions)
        self.particles = [myList.next() for i in range(0, numParticles)]

        
    def inRange(self, direction, pos):
        if direction==Directions.NORTH:
            if (pos[1]+1) in range(self.layoutHeight):
                return True
            return False
        elif direction==Directions.SOUTH:
            if (pos[1]-1) in range(self.layoutHeight):
                return True
            return False
        elif direction==Directions.WEST:
            if (pos[0]-1) in range(self.layoutWidth):
                return True
            return False
        elif direction==Directions.EAST:
            if (pos[0]+1) in range(self.layoutWidth):
                return True
            return False
        else:
            return True

    def updateDist(self, counter, sonarVal, myrange, p):
        new = util.Counter()
        for key in counter:
            val = counter[key]
            if key >= abs(sonarVal) and key+p in range(myrange):
                new[key] = val
        new.normalize()
        return new

    def observe(self, prevAction, ranges):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        ## iterate through all possible pacman positions, vague
        for p in self.legalPositions:
            center = self.pos[p]
            if prevAction==Directions.NORTH:
                if self.inRange(Directions.NORTH, p):
                    newPos=list(p)
                    newPos[1]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.9
                else:
                    self.pos[p] += center * 0.9

                if self.inRange(Directions.SOUTH, p):
                    newPos=list(p)
                    newPos[1]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center*0.025

                if self.inRange(Directions.WEST, p):
                    newPos=list(p)
                    newPos[0]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center*0.025

                if self.inRange(Directions.EAST, p):
                    newPos=list(p)
                    newPos[0]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center*0.025

                self.pos[p] += center*0.025

            elif prevAction==Directions.SOUTH:
                if self.inRange(Directions.NORTH, p):
                    newPos=list(p)
                    newPos[1]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.SOUTH, p):
                    newPos=list(p)
                    newPos[1]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.9
                else:
                    self.pos[p] += center * 0.9

                if self.inRange(Directions.WEST, p):
                    newPos=list(p)
                    newPos[0]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center*0.025

                if self.inRange(Directions.EAST, p):
                    newPos=list(p)
                    newPos[0]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center*0.025

                self.pos[p] += center*0.025

            elif prevAction==Directions.WEST:
                if self.inRange(Directions.NORTH, p):
                    newPos=list(p)
                    newPos[1]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.SOUTH, p):
                    newPos=list(p)
                    newPos[1]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.WEST, p):
                    newPos=list(p)
                    newPos[0]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.9
                else:
                    self.pos[p] += center * 0.9

                if self.inRange(Directions.EAST, p):
                    newPos=list(p)
                    newPos[0]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center*0.025

                self.pos[p] += center*0.025

            elif prevAction==Directions.EAST:
                if self.inRange(Directions.NORTH, p):
                    newPos=list(p)
                    newPos[1]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.SOUTH, p):
                    newPos=list(p)
                    newPos[1]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.WEST, p):
                    newPos=list(p)
                    newPos[0]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.EAST, p):
                    newPos=list(p)
                    newPos[0]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += self.pos[p] * 0.9
                else:
                    self.pos[p] += center * 0.9

                self.pos[p] += center*0.025

            else:
                if self.inRange(Directions.NORTH, p):
                    newPos=list(p)
                    newPos[1]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.SOUTH, p):
                    newPos=list(p)
                    newPos[1]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.WEST, p):
                    newPos=list(p)
                    newPos[0]-=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if self.inRange(Directions.EAST, p):
                    newPos=list(p)
                    newPos[0]+=1
                    newPos=tuple(newPos)
                    self.pos[newPos] += center * 0.025
                else:
                    self.pos[p] += center * 0.025

                self.pos[p] += center*0.9
        

        ## update walls based on range, further degrade possibilities if pacman position collides with wall position
        for p in self.legalPositions:
            if (p[0]-ranges[3] in range(self.layoutWidth)) and (p[0]+ranges[1] in range(self.layoutWidth)) and (p[1]+ranges[0] in range(self.layoutHeight)) and (p[1]-ranges[2] in range(self.layoutHeight)):
                for i in range(4):
                    dists = slam.getObservationDistribution(ranges[i])
                    if i == 0:
                        # for j in range(ranges[i]):
                        #     self.wall[(p[0],p[1]+j)] -=
                        dist = self.updateDist(dists, ranges[i], self.layoutHeight, p[1])
                        for key in dist:
                            val = dist[key]
                            # print dists
                            # print dist
                            # print p, i, ranges[i],val
                            wallpos = (p[0],p[1]+key)
                            if val == 1:
                                # print dist
                                # print p, i, ranges[i],val
                                self.permanant[wallpos] += 0.5*self.pos[wallpos]
                            else:
                                self.wall[wallpos] += self.wall[wallpos]*(self.wallPrior/float(1-self.wallPrior)) * (val/float(1-val))
                    elif i == 1:
                        dist = self.updateDist(dists, ranges[i], self.layoutWidth, p[0])
                        for key in dist:
                            val = dist[key]
                            # print dists
                            # print dist
                            # print p, i, ranges[i],val
                            wallpos = (p[0]+key,p[1])
                            if val == 1:
                                # print dist
                                # print p, i, ranges[i],val
                                self.permanant[wallpos] += 0.5*self.pos[wallpos]
                            else:
                                self.wall[wallpos] += self.wall[wallpos]*(self.wallPrior/float(1-self.wallPrior)) * (val/float(1-val))
                    elif i == 2:
                        dist = self.updateDist(dists, -ranges[i], self.layoutHeight, p[1])
                        for key in dist:
                            val = dist[key]
                            # print dists
                            # print dist
                            # print p, i, ranges[i],val
                            wallpos = (p[0],p[1]-key)
                            if val == 1:
                                # print dist
                                # print p, i, ranges[i],val
                                self.permanant[wallpos] += 0.5*self.pos[wallpos]
                            else:
                                self.wall[wallpos] += self.wall[wallpos]*(self.wallPrior/float(1-self.wallPrior)) * (val/float(1-val))
                    elif i == 3:
                        dist = self.updateDist(dists, -ranges[i], self.layoutWidth, p[0])
                        for key in dist:
                            val = dist[key]
                            # print dists
                            # print dist
                            # print p, i, ranges[i],val
                            wallpos = (p[0]-key,p[0])
                            if val == 1:
                                # print dist
                                # print p, i, ranges[i],val
                                self.permanant[wallpos] += 0.5*self.pos[wallpos]
                            else:
                                self.wall[wallpos] += self.wall[wallpos]*(self.wallPrior/float(1-self.wallPrior)) * (val/float(1-val))
            else:
                self.pos[p] = 0
            self.wall.normalize()
            
            self.pos.normalize()

        # self.particless = []
        # if self.wall.totalCount() == 0:
        #     self.initialize()
        # else: 
        #     for i in range(self.numParticles):
        #         # self.particles.append(util.sample(self.pos))
        #         p = util.sample(self.wall)
        #         if p != 0:
        #             self.particless.append(p)



        # new = []
        # for i in range(self.numParticles):
        #     newPosDist = self.pos
        #     p = util.sample(newPosDist)
        #     if p != 0:
        #         new.append(p)
        # self.legalPositions = new

            

            # self.wall[p] *= (self.wallPrior/(1-self.wallPrior)) * p(m[i][j] | zt, xt)/(1-p(m[i][j] | zt, xt)

    
    def getWallBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # beliefs = util.Counter()
        # for pos in self.legalPositions:
        #     beliefs[pos] = 0.1
        # beliefs[(3,4)]=0
        # print beliefs
        for key in self.permanant:
            self.wall[key]=self.permanant[key]

        return self.wall
    
    def getPositionBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # beliefs = util.Counter()
        # beliefs[self.startPos] = 0.5
        # beliefs[4,4]=0.2
        # beliefs.normalize()
        # print beliefs
        return self.pos