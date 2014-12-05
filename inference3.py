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

import copy, math, random
import functions
import particle

# Shortcuts to common functions:
multiply = functions.multiply
add = functions.add
determinant = functions.determinant
inverse = functions.inverse
transpose = functions.transpose
multVector = functions.multVector
multScalar = functions.multScalar

# The 2x2 identity matrix:
Identity = ((1, 0),
            (0, 1))
# This global constant defines how accurate the sensors are. It is the
# covariance matrix for the sensors in range and bearing.
# TODO: tune this.
SensorCovariance = ((101, 0),
                        (  0, 0.101))


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



    # The value of delta for the gaussian motion noise (S is for x and y, Sr is for theta):
    S = 10
    Sr = math.pi/6.0

   
    
    def __init__(self, startPos, layoutWidth, layoutHeight, wallPrior, legalPositions, numParticles=500):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        self.numParticles=numParticles
        self.legalPositions = legalPositions
        self.particles = []
        for i in range(numParticles):
            self.particles.append(particle.Particle(startPos, layoutHeight, layoutWidth, wallPrior))


    def observe(self, prevAction, ranges):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        if prevAction==Directions.NORTH:
            offset = (0,1)
        elif prevAction==Directions.SOUTH:
            offset = (0,-1)
        elif prevAction==Directions.WEST:
            offset = (-1,0)
        elif prevAction==Directions.EAST:
            offset = (0,1)
        else:
            offset = (0,0)
        self.updateEach(offset, ranges)
        return self.resampleParticles()

    def updateEach(self, offset, ranges):
        for p in self.particles:
            last = p.path[-1]
            new = copy.deepcopy(last)
            # Add motion noise:
            new = list(new)
            new[0] += offset[0] + random.gauss(0, self.S)         ################
            new[1] += offset[1] + random.gauss(0, self.S)
            new = tuple(new)
            p.path.append(new)
            # Update this particle:
            self.updateParticle(p, ranges)

    def updateParticle(self, particle, ranges):
        pacman_pos = (particle.path[-1][0], particle.path[-1][1])
        for i in range(4):
            offset = ranges[i]
            if i==0:
                landmarkOffset = (0, offset)
            elif i==1:
                landmarkOffset = (offset, 0)
            elif i==2:
                landmarkOffset = (0, -offset)
            elif i==3:
                landmarkOffset = (-offset, 0)
            estimatedPosition = (pacman_pos[0] + landmarkOffset[0], pacman_pos[1] + landmarkOffset[1])

            if estimatedPosition not in particle.walls:
                # Jacobian = functions.bearingRangeJacobian(*landmarkOffset)
                xyUncertainty = SensorCovariance         ####################
                particle.walls[estimatedPosition] = xyUncertainty
                particle.importance = 1
            else:
                oldPosition = (estimatedPosition[0], estimatedPosition[1])                ##########
                oldUncertainty = particle.walls[estimatedPosition]
                projectedOffset = (oldPosition[0] - pacman_pos[0], oldPosition[1] - pacman_pos[1])
                # forecast = list(functions.rTheta(*projectedOffset))
                # forecast[1] = forecast[1] - robotPose[2]
                landmarkCorrection = (landmarkOffset[0] - projectedOffset[0], landmarkOffset[1] - projectedOffset[1])
                # Jacobian = functions.bearingRangeJacobian(*projectedOffset)
                updatedUncertainty = add(oldUncertainty, SensorCovariance)    #########
                KalmanGain = multiply(oldUncertainty, inverse(updatedUncertainty))
                correctionVector = multVector(KalmanGain, landmarkCorrection)                   ############# landmarkCorrection not a rTheata form
                newLandmarkPosition = (oldPosition[0] + correctionVector[0], oldPosition[1] + correctionVector[1])
                newUncertainty = multiply(add(Identity, multScalar(KalmanGain, -1)), oldUncertainty)
                partOfExponent = multVector(inverse(updatedUncertainty), landmarkCorrection)
                importanceFactor = determinant(
                                     multScalar(
                                       updatedUncertainty,
                                       2*math.pi))**-0.5 *\
                                   math.exp(-0.5 *\
                                            (landmarkCorrection[0] * partOfExponent[0] +\
                                             landmarkCorrection[1] * partOfExponent[1]))
                estimatedPosition = list(estimatedPosition)
                estimatedPosition[0]= newLandmarkPosition[0]
                estimatedPosition[1]= newLandmarkPosition[1]
                estimatedPosition = tuple(estimatedPosition)
                particle.walls[estimatedPosition] = newUncertainty
                particle.importance = importanceFactor

    def resampleParticles(self):
        N = len(self.particles)
        ssss = sum(p.importance for p in self.particles)
        newparticles = []
        for i in range(N):
            choice = random.uniform(0, ssss)
            j = 0
            particle = self.particles[j]
            while choice > particle.importance:
                choice -= particle.importance
                j += 1
                particle = self.particles[j]
            newparticles.append(copy.deepcopy(particle))
        return newparticles


        
    
    def getWallBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # beliefs = util.Counter()
        # for pacman_pos in self.legalPositions:
        #     beliefs[pos] = 0.1
        # beliefs[(3,4)]=0
        # print beliefs
        self.walls=util.Counter()
        N = len(self.particles)
        for i in range(N):
            for legalPosition in self.legalPositions:
                self.walls[legalPosition]+=self.particles[i].walls[legalPosition]
        for legalPosition in self.legalPositions:
            self.walls[legalPosition] /= float(self.numParticles)
        return self.walls
    
    def getPositionBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        # beliefs = util.Counter()
        # beliefs[self.startPos] = 0.5
        # beliefs[4,4]=0.2
        # beliefs.normalize()
        # print beliefs
        self.pos=util.Counter()
        N = len(self.particles)
        for i in range(N):
            self.pos[self.particles[i].path[-1]]+=self.particles[i].importance
        self.pos.normalize()
        print self.pos
        return self.pos
