Partners: Qiyin Wu (edX ID: Joranson) 
          Jiahui Huang (edX ID: csrocks)

Layout&Agents that runs well: 1) oneHunt PatrolSLAMAgent/AutoSLAMAgent
               				  2) message PatrolSLAMAgent/AutoSLAMAgent
               				  3) smallHunt PatrolSLAMAgent/AutoSLAMAgent
               				  4) openHunt PatrolSLAMAgent/AutoSLAMAgent


Layout that runs worst: bigHunt. Our pacman localization is based on ranges that Pacman
                        perceives each step and the walls that are pretty sure. Because
                        it is fairly difficult for pacman to know exactly where
                        he is based on range measurements when pacman is surrounded
                        totally by innner walls (none of the range are from outer walls). 


How to fix/Improvement: we could use decay/expontial/other mathematical formulas to update (we 
                        are using numbers and formulas generated from intuition and experience here). 
                        Also, pacman's ability to increase/decrease wall distributio should be 
                        expressed in a mathematical formula (i.e. exponential growth/decay, etc) so
                        that it won't be adding random walls to the map after everything on the 
                        map has converged.


Outside Source:         we read a project called simpleSLAM written by researchers in Harvey Mudd 
				                College and use the basic particle filtering idea in fastslam.py to 
                        implement our inference.py


General Ideas:          1) we consider every particle as a Particle Class Object.
                            Fields inside Particle object:
                              path: a list recording Pacman's path, we can get Pacman's
                                            current position by calling path[-1], initialized to
                                            be [startPos]. Each particle is sure where is its own
                                            Pacman.
                              walls: each particle is assigned a map, using particle.walls
                                            to represent the probability of walls at each position.
                              importance: using this field to represent the correctness 
                                            of this particle based on Pacman's current position 
                                            and wall distribution within that specific map. If 
                                            the importance is 0, it means this particle needs to
                                            be resampled. Importance can only be 0, 1 or above 1
                                            (1 or above 1 are treated the same way)
                           
                        2) -> observe() function gets called every time step. we 
                                call updateEach() and resampleParticles() in observe()
                                to update each particle
                          -> updateEach() loops over all the particles in the particle
                                list and update its position using the prevAction.
                                in the end of updateEach(), we call updateParticle()
                          -> updateParticle() function further localizes the particle
                                using range measurements to increase/decrease wall 
                                distribution and position distribution. 
                          -> resampleParticle() where we use what we updated to 
                                resample all the particles

* Specific function descriptions are written in inference.py


