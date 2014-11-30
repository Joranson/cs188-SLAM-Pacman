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

        self.pos= util.Counter()
        self.wall = util.Counter()
        for p in legalPositions:
            self.pos[p] = 0
            self.wall[p] = 1
        self.pos[startPos] = 1
        self.wall[startPos] = 0
        self.time = 0

    
    def initialize(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        pass
        
    def inRange(direction, pos):
        if direction==Directions.NORTH:
            if (pos[1]+1) in range(self.layoutHeight+1):
                return True
            return False
        elif direction==Directions.SOUTH:
            if (pos[1]-1) in range(self.layoutHeight+1):
                return True
            return False
        elif direction==Directions.WEST:
            if (pos[0]-1) in range(self.layoutWidth+1):
                return True
            return False
        elif direction==Directions.EAST:
            if (pos[0]+1) in range(self.layoutWidth+1):
                return True
            return False
        else:
            return True


    def observe(self, prevAction, ranges):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # pass
        if self.time==0:
            print prevAction
        self.time += 1
        from game import Directions

        ## iterate through all possible pacman positions, vague
        for p in self.legalPositions:
            center = self.pos[p]
            if prevAction==Directions.NORTH:
                if inRange(Directions.NORTH, p):
                    newPos=p
                    newPos[1]+=1
                    self.pos[newPos] = self.pos[p] * 0.9
                else:
                    self.pos[p] += center * 0.9

                if inRange(Directions.SOUTH, p):
                    newPos=p
                    newPos[1]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center*0.025

                if inRange(Directions.WEST, p):
                    newPos=p
                    newPos[0]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center*0.025

                if inRange(Directions.EAST, p):
                    newPos=p
                    newPos[0]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center*0.025

                self.pos[p] += center*0.025

            elif prevAction==Directions.SOUTH:
                if inRange(Directions.NORTH, p):
                    newPos=p
                    newPos[1]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.SOUTH, p):
                    newPos=p
                    newPos[1]-=1
                    self.pos[newPos] = self.pos[p] * 0.9
                else:
                    self.pos[p] += center * 0.9

                if inRange(Directions.WEST, p):
                    newPos=p
                    newPos[0]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center*0.025

                if inRange(Directions.EAST, p):
                    newPos=p
                    newPos[0]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center*0.025

                self.pos[p] += center*0.025

            elif prevAction==Directions.WEST:
                if inRange(Directions.NORTH, p):
                    newPos=p
                    newPos[1]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.SOUTH, p):
                    newPos=p
                    newPos[1]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.WEST, p):
                    newPos=p
                    newPos[0]-=1
                    self.pos[newPos] = self.pos[p] * 0.9
                else:
                    self.pos[p] += center * 0.9

                if inRange(Directions.EAST, p):
                    newPos=p
                    newPos[0]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center*0.025

                self.pos[p] += center*0.025

            elif prevAction==Directions.EAST:
                if inRange(Directions.NORTH, p):
                    newPos=p
                    newPos[1]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.SOUTH, p):
                    newPos=p
                    newPos[1]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.WEST, p):
                    newPos=p
                    newPos[0]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.EAST, p):
                    newPos=p
                    newPos[0]+=1
                    self.pos[newPos] = self.pos[p] * 0.9
                else:
                    self.pos[p] += center * 0.9

                self.pos[p] += center*0.025

            else:
                if inRange(Directions.NORTH, p):
                    newPos=p
                    newPos[1]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.SOUTH, p):
                    newPos=p
                    newPos[1]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.WEST, p):
                    newPos=p
                    newPos[0]-=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                if inRange(Directions.EAST, p):
                    newPos=p
                    newPos[0]+=1
                    self.pos[newPos] = center * 0.025
                else:
                    self.pos[p] += center * 0.025

                self.pos[p] += center*0.9
        self.pos.normalize()

        ## update walls based on range, further degrade possibilities if pacman position collides with wall position
        for p in self.legalPositions:
            center = self.pos[p]
            for i range(p[0]-ranges[3]+1, self.startPos[0]+ranges[1])

        for i in range(self.startPos[0]-ranges[3]+1, self.startPos[0]+ranges[1]):
            for j in range(self.startPos[1]-ranges[2]+1, self.startPos[1]+ranges[0]):
                self.wall[(i,j)]=0
       
            # self.wall[p] *= (self.wallPrior/(1-self.wallPrior)) * p(m[i][j] | zt, xt)/(1-p(m[i][j] | zt, xt)
    
    def getWallBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # beliefs = util.Counter()
        # for pos in self.legalPositions:
        #     beliefs[pos] = 0.1
        # beliefs[(3,4)]=0
        # print beliefs
        return self.wall
    
    def getPositionBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # beliefs = util.Counter()
        # beliefs[self.startPos] = 0.5
        # beliefs[4,4]=0.2
        # beliefs.normalize()
        # print beliefs
        return self.pos
