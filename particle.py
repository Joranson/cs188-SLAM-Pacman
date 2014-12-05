import util
class Particle():
	def __init__(self, startPos, h, w, prior):
		self.path = [startPos]
		self.walls = util.Counter()
		self.importance = 0
		for i in range(w):
			for j in range(h):
				self.walls[i,j]=prior
		self.walls[startPos]=0
