import numpy as np
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
import sys

class Controller:
    def __init__(self, target_betti_file):
        pass

def __init__(
		self,
		betti_start_thresh,
		betti_end_thresh,
		betti_thresh_spacing,
		target_betti_file,
		target_points
	):

		# Previous points in the simulation...
		self.prev_pos = np.copy(target_points)

		# Compute pariwise differenes for points in the target...
		self.target_dists = squareform(pdist(target_points, "euclidean"))

		# Define s for the kernel function as average pairwise difference of 
		# target points plus one std deviation...
		self.s = np.median(self.target_dists) #+ np.std(self.target_dists)
		logr.debug("Kernel scale parameter 's' : {}".format(self.s))

		# Threshold for the loss...
		self.loss_thresh = 0#4.0e5
		self.num_correction_tries = int(target_points.shape[0] * 0.5)

		# TODO: Mabye have some state variables for when betti numbers are 
		# generated
		
		self.betti_start_thresh 	= betti_start_thresh
		self.betti_end_thresh 		= betti_end_thresh
		self.betti_thresh_spacing 	= betti_thresh_spacing

		# Read the betti file corresponding to the desired topology...
		target_num_times, \
		target_start_thresh, \
		target_end_thresh, \
		target_thresh_spacing, \
		target_betti_array = betti_generator.read_betti_file(target_betti_file)
		self.target_thresholds = [
			i 
			for i in 
			range(target_start_thresh,target_end_thresh+1,target_thresh_spacing)
		]
		self.target_times = [
			i 
			for i in 
			range(
				target_start_thresh, 
				target_start_thresh + target_thresh_spacing*target_num_times,
				target_thresh_spacing
			)
		]
		self.target_betti_array = target_betti_array.reshape((
			len(self.target_thresholds),
			len(self.target_times),
			len(self.target_times)	
		))

		# Need to check if the current spacing and the target spacing are
		# consistent...
		if betti_thresh_spacing != target_thresh_spacing:
			print("The current algorithm only works when the threshold parameters for the two betti-0 functions have the same spacing. Please re-generate the betti-0 functions so this is the case.")
			sys.exit()

    def loss(self, x_prime_dists):
		# Compute the kernel...
		k = np.where(
			x_prime_dists > self.s, 
			0*x_prime_dists,
			np.ones_like(x_prime_dists)
		)

		# Regularization...
		tau = np.sum(
			k * (x_prime_dists - self.target_dists) * \
				(x_prime_dists - self.target_dists)
		)

		return tau

	def compare_topology(self, current_betti_array):
		# Average percent difference...
		a = current_betti_array
		b = self.target_betti_array

		return np.average(np.abs(a - b) / ((a + b) / 2.0))

    # Callback for each time step in the boid simulation which should 
	# enforce control...
	#
	def control_boids(self, positions):
		# # Find the betti numbers for the flock positions...

		# betti_array, _, thresholds = betti_generator.get_betti_array(
		# 	self.betti_start_thresh,
		# 	self.betti_end_thresh,
		# 	self.betti_thresh_spacing,
		# 	2,
		# 	[positions] # Turn this into a list
		# )

		# # Need to see if the current topology is 'correct' and enforce a control
		# # on the positions if necessary to generate the desired topology...

		# tda_diff = self.compare_topology(betti_array)
		# logr.debug("TDA percent difference {}".format(tda_diff))
		dists = squareform(pdist(positions, "euclidean"))
		loss = self.loss(dists)
		prev_loss = loss
		correction_try = 0

		logr.debug("Current loss : {}".format(loss))

		update_positions = np.random.choice(
			positions.shape[0], self.num_correction_tries, replace=False
		)
		while loss > self.loss_thresh and \
			correction_try < self.num_correction_tries:

			# Randomly sample a point to replace using its position from the
			# previous time step...

			update_pos = update_positions[correction_try]
			boid_pos_pre = positions[update_pos]
			positions[update_pos] = np.copy(self.prev_pos[update_pos])

			# Recalculate the loss...
			recalc_dists = squareform(pdist(positions, "euclidean"))
			loss = self.loss(recalc_dists)

			if loss >= prev_loss:
				# Refuse the update since it made the loss go up...
				positions[update_pos] = boid_pos_pre
			else:
				logr.debug("Loss after correction : {}".format(loss))

			# Update the number of corrections tried...
			correction_try = correction_try + 1

		self.prev_pos = np.copy(positions)
		return flock
