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
        self.startPos = startPos
        self.legalPositions = legalPositions
        print startPos
        # print legalPositions
        print wallPrior
        print layoutHeight
    
    def initialize(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        pass
                
    def observe(self, prevAction, ranges):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        print prevAction
        pass
    
    def getWallBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        beliefs = util.Counter()
        for pos in self.legalPositions:
            beliefs[pos] = 1
        # print beliefs
        return beliefs
    
    def getPositionBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        beliefs = util.Counter()
        beliefs[self.startPos] = 1
        beliefs.normalize()
        print beliefs
        return beliefs