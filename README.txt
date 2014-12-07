Partners: 1) Qiyin Wu (edX ID: Joranson) 
          2) Jiahui Huang (edX ID: csrocks)

Layout&Agents that runs well: 1) oneHunt PatrolSLAMAgent/AutoSLAMAgent
               				  2) message PatrolSLAMAgent/AutoSLAMAgent
               				  3) smallHunt PatrolSLAMAgent/AutoSLAMAgent
               				  4) openHunt PatrolSLAMAgent/AutoSLAMAgent


Layout that runs worst: bigHunt. Our pacman localization is based on positions
                        of outside walls and because of complexity of bigHunt,
                        it is fairly difficult for pacman to know exactly where
                        he is based on range measurements when pacman is surrounded
                        by inside walls. 


how to fix/Improvement: we should use decay/expontial/other mathematical formulas to update (we 
                        are using numbers and formulas generated from intuition and experience here). 
                        Also, pacman's ability to increase/decrease wall distributio should be 
                        expressed in a mathematical formula (i.e. exponential growth/decay, etc) so
                        that it won't be adding random walls to the map after everything on the map has converged.


Outside Source: we read a project called simpleSLAM written by researchers in Harvey Mudd 
				College and use the idea of in fastslam.py to implement our slam.py

                General Ideas:
                1) we consider every particle as a Particle Class Object.
                    Inside Particle object:
                    
                2) -> observe() function gets called every time step. we call updateEach() and resampleParticles() in observe() to update each particle
                      -> updateEach() loops over all the particles in the particle list and update its position using the prevAction.
                       in the end of updateEach(), we call updateParticle()
                          -> updateParticle() function further localizes the particle using range measurements to increase/decrease wall 
                           distribution and position distribution. 
                      -> resampleParticle() where we use what we updated to resample all the particles

                3) specific function descriptions are written in slam.py


