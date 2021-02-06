#-------------------------------------------------------------------------------
# System Imports
#-------------------------------------------------------------------------------

import argparse
from datetime import datetime, timezone
import sys

#-------------------------------------------------------------------------------
# Local Imports
#-------------------------------------------------------------------------------

import boids_simulation
import logger


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
		"log_level" 		: "INFO",
		"separation_force"	: 0,
		"separation_radius"	: 50,
		"alignment_force"	: 0,
		"alignment_radius"	: 100,
		"cohesion_force"	: 0,
		"cohesion_radius"	: 150,
		"dms_fname"			: datetime.now(timezone.utc).strftime(
			"%Y-%m-%d_%H:%M:%S%z") + "_dms-out.txt",
		"nframes"			: 999,
		"animate"			: False
	}

	# Add sub parser
	parser = argparse.ArgumentParser(
		description="""Boid simulation implementing control""",
		formatter_class=argparse.ArgumentDefaultsHelpFormatter
	)
	parser.add_argument("num_points", type=int,
		help="""Number of boids in the simulation"""
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
	
	# Get the command line arguments...

	args, _ = parse_cmd_line(as_dict=True)
	
	# Run the simulation...

	boids_simulation.run_sim(
		num_boids=args["num_points"],
		sep_in=args["separation_force"],
		ali_in=args["alignment_force"],
		coh_in=args["cohesion_force"],
		dms_filename=args["dms_fname"],
		seprad_in=args["separation_radius"],
		alirad_in=args["alignment_radius"],
		cohrad_in=args["cohesion_radius"],
		nframes=args["nframes"],
		animate=args["animate"]
	)
	

################################################################################
################################################################################
# Entry point for testing...
#
if __name__ == "__main__":
	main()




