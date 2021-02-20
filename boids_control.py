#-------------------------------------------------------------------------------
# System Imports
#-------------------------------------------------------------------------------

import argparse
from datetime import datetime, timezone
import numpy as np
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
import sys

#-------------------------------------------------------------------------------
# Local Imports
#-------------------------------------------------------------------------------

import betti_generator
import betti0_erosion_distance
import boids_simulation
import logger

################################################################################
################################################################################
# Class responsible for implementing control on a boid simulation
#
class BoidController:

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

	###########################################################################
	###########################################################################
	#
	#
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


	###########################################################################
	###########################################################################
	#
	#
	def compare_topology(self, current_betti_array):

		# Average percent difference...
		a = current_betti_array
		b = self.target_betti_array

		return np.average(np.abs(a - b) / ((a + b) / 2.0))


	###########################################################################
	###########################################################################
	# Callback for each time step in the boid simulation which should 
	# enforce control...
	#
	def control_boids(self, flock):

		# # Find the betti numbers for the flock positions...

		# betti_array, _, thresholds = betti_generator.get_betti_array(
		# 	self.betti_start_thresh,
		# 	self.betti_end_thresh,
		# 	self.betti_thresh_spacing,
		# 	2,
		# 	[flock.position] # Turn this into a list
		# )

		# # Need to see if the current topology is 'correct' and enforce a control
		# # on the positions if necessary to generate the desired topology...

		# tda_diff = self.compare_topology(betti_array)
		# logr.debug("TDA percent difference {}".format(tda_diff))

		dists = squareform(pdist(flock.position, "euclidean"))
		loss = self.loss(dists)
		prev_loss = loss
		correction_try = 0

		logr.debug("Current loss : {}".format(loss))

		update_positions = np.random.choice(
			flock.position.shape[0], self.num_correction_tries, replace=False
		)
		while loss > self.loss_thresh and \
			correction_try < self.num_correction_tries:

			# Randomly sample a point to replace using its position from the
			# previous time step...

			update_pos = update_positions[correction_try]
			boid_pos_pre = flock.position[update_pos]
			flock.position[update_pos] = np.copy(self.prev_pos[update_pos])

			# Recalculate the loss...
			recalc_dists = squareform(pdist(flock.position, "euclidean"))
			loss = self.loss(recalc_dists)

			if loss >= prev_loss:
				# Refuse the update since it made the loss go up...
				flock.position[update_pos] = boid_pos_pre
			else:
				logr.debug("Loss after correction : {}".format(loss))

			# Update the number of corrections tried...
			correction_try = correction_try + 1

		# 	# Find the most likely culprit causing the difference. Lets say its
		# 	# the boid with the max average pairwise distances...

		# 	dists = squareform(pdist(flock.position, "euclidean"))
		# 	avg_dists = np.average(dists, axis=1)
		# 	rogue_boid = np.argmax(avg_dists)
		# 	logr.debug("Rogue boid {} : {}".format(
		# 		rogue_boid, flock.position[rogue_boid]
		# 	))

		# 	# Find the center of mass and average of pairwise distances from it.
		# 	# Then find the angle from the CoM to the rogue point. Update the
		# 	# rogue point's position using the r,theta just calculated...

		# 	center_of_mass = np.average(flock.position, axis=0)
		# 	logr.debug("Center of mass center_of_mass : {}".format(
		# 		center_of_mass
		# 	))

		# 	avg_dist_com = np.average(
		# 		np.linalg.norm(
		# 			np.repeat(
		# 				center_of_mass, 
		# 				flock.position.shape[0]
		# 			).reshape((-1, 2), order="F") - flock.position,
		# 			axis=1
		# 		)
		# 	)
		# 	logr.debug(avg_dist_com)
			
		# 	theta = np.arctan2(
		# 		flock.position[rogue_boid, 1] - center_of_mass[1],
		# 		flock.position[rogue_boid, 0] - center_of_mass[0]
		# 	)
		# 	logr.debug(np.rad2deg(theta))

		# 	# Update the x and y position...
		# 	flock.position[rogue_boid,0] = avg_dist_com * np.cos(theta)
		# 	flock.position[rogue_boid,1] = avg_dist_com * np.sin(theta)
		# 	flock.position[rogue_boid] = flock.position[rogue_boid] + center_of_mass

		# 	logr.debug("Corrected rogue boid {} : {}".format(
		# 		rogue_boid, flock.position[rogue_boid]
		# 	))

		# 	# Recalculate the TDA difference...
		# 	betti_array, _, thresholds = betti_generator.get_betti_array(
		# 		self.betti_start_thresh,
		# 		self.betti_end_thresh,
		# 		self.betti_thresh_spacing,
		# 		2,
		# 		[flock.position] # Turn this into a list
		# 	)
		# 	tda_diff = self.compare_topology(betti_array)
		# 	logr.debug("Recalculated TDA percent difference {}".format(tda_diff))

		# 	break

		self.prev_pos = np.copy(flock.position)
		return flock


################################################################################
################################################################################
# Read initial positions file which is a DMS file with a single time step...
#
def read_initial_positions(fname):
	return np.loadtxt(fname, skiprows=1)

################################################################################
################################################################################
# Gets the command line arguments...
#
def parse_cmd_line(as_dict=False):
	"""
	Description:
		Parses the command line arguments for the program
	Parameters:
		as_dict - bool
			Returns the args as a dict. Default=False
	Returns:
		The command line arguments as a dictionary or a Namespace object and the
		parser used to parse the command line.
	"""

	defaults = {
		"log_level" 			: "INFO",
		"initial_pos_fname"		: None,
		"separation_force"		: 0,
		"separation_radius"		: 50,
		"alignment_force"		: 0,
		"alignment_radius"		: 100,
		"cohesion_force"		: 0,
		"cohesion_radius"		: 150,
		"dms_fname"				: datetime.now(timezone.utc).strftime(
			"%Y-%m-%d_%H:%M:%S%z") + "_dms-out.txt",
		"nframes"				: 999,
		"animate"				: False,
		"betti_start_thresh"	: 0,
		"betti_end_thresh"		: 50,
		"betti_thresh_spacing"	: 5
	}

	# Add sub parser
	parser = argparse.ArgumentParser(
		description="""Boid simulation implementing control""",
		formatter_class=argparse.ArgumentDefaultsHelpFormatter
	)
	parser.add_argument("num_points", type=int,
		help="""Number of boids in the simulation"""
	)
	parser.add_argument("target_betti_fname", type=str,
		help="""Betti fname describing the initial topology"""
	)
	parser.add_argument("initial_pos_fname", type=str,
		help="""
		File of initial positions of boids. DMS file with single time step
		"""
	)
	parser.add_argument("-s", "--separation_force", type=float,
		default=defaults["separation_force"],
		help=""""""
	)
	parser.add_argument("--separation_radius", type=float,
		default=defaults["separation_radius"],
		help=""""""
	)
	parser.add_argument("-a", "--alignment_force", type=float,
		default=defaults["alignment_force"],
		help=""""""
	)
	parser.add_argument("--alignment_radius", type=float,
		default=defaults["alignment_radius"],
		help=""""""
	)
	parser.add_argument("-c", "--cohesion_force", type=float,
		default=defaults["cohesion_force"],
		help=""""""
	)
	parser.add_argument("--cohesion_radius", type=float,
		default=defaults["cohesion_radius"],
		help=""""""
	)
	parser.add_argument("-f", "--dms_fname", type=str,
		default=defaults["dms_fname"],
		help=""""""
	)
	parser.add_argument("--nframes", type=int,
		default=defaults["nframes"],
		help=""""""
	)
	parser.add_argument("--animate", action="store_true",
		default=defaults["animate"],
		help=""""""
	)
	parser.add_argument("--betti_start_thresh", type=int,
		default=defaults["betti_start_thresh"],
		help=""""""
	)
	parser.add_argument("--betti_end_thresh", type=int,
		default=defaults["betti_end_thresh"],
		help=""""""
	)
	parser.add_argument("--betti_thresh_spacing", type=int,
		default=defaults["betti_thresh_spacing"],
		help=""""""
	)
	parser.add_argument("--no_control", action="store_true",
		help=""""""
	)
	parser.add_argument("--log_level", 
		choices=["CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"],
		default=defaults["log_level"],
		help="""Log level of the program."""
	)
	
	args = parser.parse_args()
	if as_dict:
		args = vars(args)

	return args, parser

################################################################################
################################################################################
# Main...
#
def main():
	
	# TODO: Very much dislike the use of global but doing it here to save time
	global logr

	# Get the command line arguments...

	args, _ = parse_cmd_line(as_dict=True)
	
	# Get instance of the logger...

	logr = logger.get_logger(__name__, level=args["log_level"])

	# Run the simulation...

	init_pos = read_initial_positions(args["initial_pos_fname"])

	boid_ctrl = BoidController(
		args["betti_start_thresh"],
		args["betti_end_thresh"],
		args["betti_thresh_spacing"],
		args["target_betti_fname"],
		init_pos
	)
	boids_simulation.run_sim(
		num_boids=args["num_points"],
		sep_in=args["separation_force"],
		ali_in=args["alignment_force"],
		coh_in=args["cohesion_force"],
		dms_filename=args["dms_fname"],
		seprad_in=args["separation_radius"],
		alirad_in=args["alignment_radius"],
		cohrad_in=args["cohesion_radius"],
		initial_positions=init_pos,
		control_callback=boid_ctrl.control_boids if 
			not args["no_control"] else None,
		nframes=args["nframes"],
		animate=args["animate"]
	)
	

################################################################################
################################################################################
# Entry point...
#
if __name__ == "__main__":
	main()

