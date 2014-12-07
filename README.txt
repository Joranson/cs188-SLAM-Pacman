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


how to fix/Improvement: We have high certainty that information we get from our starPos
						is 


Outside Source: we read a project called simpleSLAM written by researchers in Harvey Mudd College
                and use the idea of in fastslam.py to implement our slam.py
                General Ideas:
                1) we consider every particle as a Particle Class Object.
                2) in our observe() function, we update each particle and then do resampling
                   on the self.particles list.
                3) specific function descriptions are written in slam.py


