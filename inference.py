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
import particle

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

   
    
    def __init__(self, startPos, layoutWidth, layoutHeight, wallPrior, legalPositions, numParticles=100):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        self.numParticles=numParticles
        self.legalPositions = legalPositions
        self.particles = []
        self.wallPrior=wallPrior
        self.layoutHeight = layoutHeight
        self.layoutWidth = layoutWidth
        for i in range(numParticles):
            self.particles.append(particle.Particle(startPos, layoutHeight, layoutWidth, wallPrior))
        self.resam = 0

    # def initialize(self, particle):
    #     for i in range(self.numParticles):


    def rand(self, i, new):
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
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
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
        # print "sample number: ", self.resam
        self.resam += 1

    def updateEach(self, ranges, i):
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

    # def updateDist(self, counter, wallOffset, i, pacman_pos):
    #     myrange = self.layoutHeight if (i==0) or (i==2) else self.layoutWidth
    #     sonarVal = wallOffset[1] if (i==0) or (i==2) else wallOffset[0]
    #     p = pacman_pos[1] if (i==0) or (i==2) else pacman_pos[0]
    #     new = util.Counter()
    #     for key in counter:
    #         val = counter[key]
    #         if key >= abs(sonarVal) and key+p in range(myrange):
    #             new[key] = val
    #     new.normalize()
    #     return new


    def updateParticle(self, particle, ranges):
        pacman_pos = (particle.path[-1][0], particle.path[-1][1])
        for i in range(4):
            # dists = slam.getObservationDistribution(ranges[i])
            # if i==0:
            #     wallOffset = (0, ranges[i])
            # elif i==1:
            #     wallOffset = (ranges[i], 0)
            # elif i==2:
            #     wallOffset = (0, -ranges[i])
            # elif i==3:
            #     wallOffset = (-ranges[i], 0)
            # dist = self.updateDist(dists, wallOffset, i, pacman_pos)

            for ii in range(ranges[i]):
                if i==0:
                    particle.walls[pacman_pos[0], pacman_pos[1]+ii]-=0.3
                    if particle.walls[pacman_pos[0], pacman_pos[1]+ii] < 0:
                        particle.walls[pacman_pos[0], pacman_pos[1]+ii] = 0
                elif i==1:
                    particle.walls[pacman_pos[0]+ii, pacman_pos[1]]-=0.3
                    if particle.walls[pacman_pos[0]+ii, pacman_pos[1]] < 0:
                        particle.walls[pacman_pos[0]+ii, pacman_pos[1]] = 0
                elif i==2:
                    particle.walls[pacman_pos[0], pacman_pos[1]-ii]-=0.3
                    if particle.walls[pacman_pos[0], pacman_pos[1]-ii] < 0:
                        particle.walls[pacman_pos[0], pacman_pos[1]-ii] = 0
                else:
                    particle.walls[pacman_pos[0]-ii, pacman_pos[1]]-=0.3
                    if particle.walls[pacman_pos[0]-ii, pacman_pos[1]] < 0:
                        particle.walls[pacman_pos[0]-ii, pacman_pos[1]] = 0

            # for key in dist:
            #     val = dist[key]
            #     if i==0:
            #         wallpos = (pacman_pos[0],pacman_pos[1]+key)
            #     elif i==1:
            #         wallpos = (pacman_pos[0]+key,pacman_pos[1])
            #     elif i==2:
            #         wallpos = (pacman_pos[0],pacman_pos[1]-key)
            #     else:
            #         wallpos = (pacman_pos[0]-key,pacman_pos[1])
            #     if val == 1:
            #         particle.walls[wallpos]=1
            #     else:
            #         particle.walls[wallpos] *= (float(1-self.wallPrior)/self.wallPrior) * (val/float(1-val))
            #         # particle.walls[wallpos] = val
            #         # particle.importance += 0.25                      ## need to change!!!!!

            #     if particle.walls[wallpos]>=1:
            #         particle.walls[wallpos]=1
            #     if (particle.walls[wallpos]==1) and (wallpos in particle.path):
            #         particle.importance = 0

            if i==0:
                wallpos = (pacman_pos[0],pacman_pos[1]+ranges[i])
            elif i==1:
                wallpos = (pacman_pos[0]+ranges[i],pacman_pos[1])
            elif i==2:
                wallpos = (pacman_pos[0],pacman_pos[1]-ranges[i])
            else:
                wallpos = (pacman_pos[0]-ranges[i],pacman_pos[1])
            # print "here"

            particle.walls[wallpos] += 0.1
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
        
        # if particle.walls[particle.path[-2]]==1:
        #     # print "reached here!", particle.path[-2]
        #     # print particle.walls
        #     particle.importance = 0.5


    def resampleParticles(self):
        # print "Before resampling: "
        # self.printParticleImportance()
        N = len(self.particles)
        # ssss = sum(p.importance for p in self.particles)
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
            # print "len cop:", len(cop)
            for i in range(len(cop)):
                if len(newparticles) >= N:
                    indicator = True
                    break
                destiny = random.random()
                newparticles.append(copy.deepcopy(cop[i]))            ## need to use new particles
            if len(cop)==0:
                newparticles = copy.deepcopy(self.particles)
                for p in newparticles:
                    p.importance = 1
            if indicator:
                break
        self.particles = newparticles

    
    def getWallBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        walls=util.Counter()
        N = len(self.particles)
        for i in range(N):
            for legalPosition in self.legalPositions:
                walls[legalPosition]+=self.particles[i].walls[legalPosition]
        for legalPosition in self.legalPositions:
            walls[legalPosition] /= float(self.numParticles)
        return walls
    
    def getPositionBeliefDistribution(self):
        "*** YOU OVERWRITE THIS METHOD HOWEVER YOU WANT ***"
        pos=util.Counter()
        N = len(self.particles)
        for i in range(N):
            pos[self.particles[i].path[-1]] += self.particles[i].importance
        for i in range(self.layoutWidth):
            pos[(i,0)]=0
            pos[(i,self.layoutHeight-1)]=0
        for j in range(self.layoutHeight):
            pos[(0,j)]=0
            pos[(self.layoutWidth-1, j)]=0
        pos.normalize()
        # print pos
        return pos

    # def printParticleImportance(self):
        # rt = []
        # for p in self.particles:
        #     rt.append(p.importance)
        # print rt 
