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

class Particle():
    def __init__(self, startPos, h, w, prior):
        self.path = [startPos]
        self.walls = util.Counter()
        self.importance = 0
        self.ratio = 1
        for i in range(w):
            for j in range(h):
                self.walls[(i,j)]=prior
        self.walls[startPos]=0
    
class SLAMParticleFilter(InferenceModule):
    """
    Particle filtering inference module for use in SLAM.
    """
    
    "*** YOU MAY ADD WHATEVER HELPER METHODS YOU WANT TO THIS CLASS ***"    

   
    
    def __init__(self, startPos, layoutWidth, layoutHeight, wallPrior, legalPositions, numParticles=100):
        # "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        """
        -For __init__, we build a self.particles list which contains all 
         the particles, each particle is represented as a Particle Class object. 
        -The initial state of the self.particles: all particles are put in self.startPos
        """
        self.numParticles=numParticles
        self.legalPositions = legalPositions
        self.particles = []
        self.wallPrior=wallPrior
        self.layoutHeight = layoutHeight
        self.layoutWidth = layoutWidth
        for i in range(numParticles):
            self.particles.append(Particle(startPos, layoutHeight, layoutWidth, wallPrior))
        self.resam = 0


    def rand(self, i, new):
        """
        -parameters: i indicates the direction pacman received, new is the current position of a particle
        -This is a helper function used in updateEach() function. Because the probabiltiy of pacman moving
         into its prevAction direction 90%, we generate random number according to the probability to 
         determine where each particle is moving toward. 
        -return value is the new position the particle is at.
        """
        r = random.randint(1,1000)
        if i==0:
            if r<=900:
                new[1]+=1
            elif r<=925:
                new[1]-=1
            elif r<=950:
                new[0]+=1
            elif r<=975:
                new[0]-=1
        elif i==1:
            if r<=900:
                new[0]+=1
            elif r<=925:
                new[0]-=1
            elif r<=950:
                new[1]+=1
            elif r<=975:
                new[1]-=1
        elif i==2:
            if r<=900:
                new[1]-=1
            elif r<=925:
                new[1]+=1
            elif r<=950:
                new[0]+=1
            elif r<=975:
                new[0]-=1
        elif i==3:
            if r<=900:
                new[0]-=1
            elif r<=925:
                new[0]+=1
            elif r<=950:
                new[1]+=1
            elif r<=975:
                new[1]-=1
        else:
            if r<=900:
                pass
            elif r<=925:
                new[1]-=1
            elif r<=950:
                new[0]+=1
            elif r<=975:
                new[0]-=1
            else:
                new[1]+=1

        return new


    def observe(self, prevAction, ranges):
        """
        -this function is called at every time stamp. we first call updateEach() function to update each 
         particles and then we call resampleParticles() to resample the particles. i.e. particle filtering.
        """
        if prevAction==Directions.NORTH:
            i=0
        elif prevAction==Directions.EAST:
            i=1
        elif prevAction==Directions.SOUTH:
            i=2
        elif prevAction==Directions.WEST:
            i=3
        else:
            i=4
        self.updateEach(ranges, i)
        self.resampleParticles()
        self.resam += 1

    def updateEach(self, ranges, i):
        """
        -parameters: ranges is the range measurement, i indicates the direction pacman is supposed to go according
         to prevAction
        -in this funciton, we loop over all the particles in self.particles list, we call self.rand() to genterate
         the next position of the particle and we append the new position to each particle's path list.
        -After obtaining the new position, we call updateParticle() to do further localization and mapping.
        """
        for p in self.particles:
            # print p.path
            last = p.path[-1]
            new = copy.deepcopy(last)
            # Add motion noise:
            reserved = new
            new = list(new)
            ## randomnized
            new = self.rand(i, new)                                            
            new = tuple(new)
            if (new[1] not in range(self.layoutHeight)) or (new[0] not in range(self.layoutWidth)):
                new = reserved
            p.path.append(new)
            # Update this particle:
            self.updateParticle(p, ranges)

    def ratio(self, particle):
        """
        -parameters: this function takes in one particle
        -this helper function returns the ratio of 1-(all the position pacman has been to/all the positions pacman can be in).
        -the function is called in updateParticle because we decide to lower particle's ability to decrease his preception of
         walls over time. (at time step 1000, if pacman detects a wall, it will only add a tiny bit of weight to the wall position) 
        """
        rt = 1-len(set(particle.path))/(self.layoutWidth*self.layoutHeight-round(self.wallPrior*self.layoutWidth*self.layoutHeight, 0))
        if rt<=0:
            rt=0.01
        return rt



    def updateParticle(self, particle, ranges):
        """
        -parameters: particle is the particle that is being updated, ranges is the range measurement
        -First, we decrease the wall probability of all the position that are inside our range. 
        -Second, we increase the wall probability and importance(resampling criteria, more on that later) of 
         all the position that are at our range. 
        -If the particle we are updating is out of range, we set its importance(will explain in 
         resampleParticles() function) to 0.
        -We times all the weight with self.ratio(), helper function defined above.
        """
        pacman_pos = (particle.path[-1][0], particle.path[-1][1])
        for i in range(4):
            for ii in range(ranges[i]):
                if i==0:
                    particle.walls[pacman_pos[0], pacman_pos[1]+ii]-=self.ratio(particle)*0.3
                    if particle.walls[pacman_pos[0], pacman_pos[1]+ii] < 0:
                        particle.walls[pacman_pos[0], pacman_pos[1]+ii] = 0
                elif i==1:
                    particle.walls[pacman_pos[0]+ii, pacman_pos[1]]-=self.ratio(particle)*0.3
                    if particle.walls[pacman_pos[0]+ii, pacman_pos[1]] < 0:
                        particle.walls[pacman_pos[0]+ii, pacman_pos[1]] = 0
                elif i==2:
                    particle.walls[pacman_pos[0], pacman_pos[1]-ii]-=self.ratio(particle)*0.3
                    if particle.walls[pacman_pos[0], pacman_pos[1]-ii] < 0:
                        particle.walls[pacman_pos[0], pacman_pos[1]-ii] = 0
                else:
                    particle.walls[pacman_pos[0]-ii, pacman_pos[1]]-=self.ratio(particle)*0.3
                    if particle.walls[pacman_pos[0]-ii, pacman_pos[1]] < 0:
                        particle.walls[pacman_pos[0]-ii, pacman_pos[1]] = 0

            if i==0:
                wallpos = (pacman_pos[0],pacman_pos[1]+ranges[i])
            elif i==1:
                wallpos = (pacman_pos[0]+ranges[i],pacman_pos[1])
            elif i==2:
                wallpos = (pacman_pos[0],pacman_pos[1]-ranges[i])
            else:
                wallpos = (pacman_pos[0]-ranges[i],pacman_pos[1])

            particle.walls[wallpos] += 0.1*self.ratio(particle)
            if particle.walls[wallpos] > 1:
                particle.walls[wallpos] = 1


            if i==0:
                if (pacman_pos[1]+ranges[0]) in range(self.layoutHeight):
                    particle.importance += 0.25
                else:
                    particle.importance = 0
                    break
            if i==1:
                if (pacman_pos[0]+ranges[1]) in range(self.layoutWidth):
                    particle.importance += 0.25
                else:
                    particle.importance = 0
                    break
            if i==2:
                if (pacman_pos[1]-ranges[2]) in range(self.layoutHeight):
                    particle.importance += 0.25
                else:
                    particle.importance = 0
                    break
            if i==3:
                if (pacman_pos[0]-ranges[3]) in range(self.layoutWidth):
                    particle.importance += 0.25
                else:
                    particle.importance = 0
                    break


    def resampleParticles(self):
        """
        -We resample all our particles in this function. 
        -All particles has either a self.importance value of 0 or 1. If they have importance of 1, we add it to the newparticles list
         we then throw out the particles with self.importance = 0, and duplicate good particles until we have numParticles particles in
         our newparticles list.
        -If all our particles appears to have a self.importance of 0, we initialize the particles list by setting all the particles'
         importance to 1.
        """
        N = len(self.particles)
        newparticles = []
        cop = []
        for i in range(N):
            destiny = random.random()
            particle = self.particles[i]
            if destiny <= particle.importance:
                cop.append(copy.deepcopy(particle))
                newparticles.append(copy.deepcopy(particle))
        while len(newparticles)<N:
            indicator = False
            for i in range(len(cop)):
                if len(newparticles) >= N:
                    indicator = True
                    break
                destiny = random.random()
                newparticles.append(copy.deepcopy(cop[i]))            ## need to add new selected particles
            if len(cop)==0:
                newparticles = copy.deepcopy(self.particles)
                for p in newparticles:
                    p.importance = 1
            if indicator:
                break
        self.particles = newparticles

    
    def getWallBeliefDistribution(self):
        """
        We took all our Particle objects, build a new global counter every time, and add every particles' map and beliefs into 
        self.walls and divide the value by numParticles to a distribution of walls.
        """        
        self.walls=util.Counter()
        N = len(self.particles)
        for i in range(N):
            for legalPosition in self.legalPositions:
                self.walls[legalPosition]+=self.particles[i].walls[legalPosition]
        for legalPosition in self.legalPositions:
            self.walls[legalPosition] /= float(self.numParticles)
        return self.walls
    
    def getPositionBeliefDistribution(self):
        """
        We build a new pos Counter, setting all outside pos value of outside wall to 0 and all particles last position to particles'
        importance. We also muliply all weight by (1-probabilty of wall at that pos) using self.wall Counter we built in
        getWallBeliefDistribution() function.
        """
        pos=util.Counter()
        N = len(self.particles)
        for i in range(N):
            pos[self.particles[i].path[-1]] += (self.particles[i].importance)*(1-self.walls[self.particles[i].path[-1]])
        for i in range(self.layoutWidth):
            pos[(i,0)]=0
            pos[(i,self.layoutHeight-1)]=0
        for j in range(self.layoutHeight):
            pos[(0,j)]=0
            pos[(self.layoutWidth-1, j)]=0
        pos.normalize()
        return pos