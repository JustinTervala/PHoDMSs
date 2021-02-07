import dionysus as d
import numpy as np
from scipy.spatial.distance import squareform
from scipy.spatial.distance import pdist
import sys

def get_betti_count(dgms,thresh):
	counter = 0
	for i in dgms:
		if i.birth <= thresh <= i.death:
			counter += 1
	return counter


def get_dgms(dists):
	f = d.fill_rips(dists, 1, np.amax(dists)+1)
	m = d.homology_persistence(f)
	dgms = d.init_diagrams(m,f)
	return dgms[0]
 

def get_dist(points):
	n = len(points)
	xpts = np.zeros((n,2))
	ypts = np.zeros((n,2))	
	points = np.array(points)

	xpts[:,0] = points[:,0]
	ypts[:,0] = points[:,1]

	xdists = pdist(xpts)
	ydists = pdist(ypts)
	
	dists = np.sqrt(np.square(xdists)+np.square(ydists))
	return dists	


def get_points(file, numpoints):
	points = []
	for i in range(numpoints):
		line = file.readline()
		sline = line.split(' ')
		fline = list(map(float, sline))
		points.append(fline)
	return points


def get_betti_array(
	start_thresh,
	end_thresh,
	spacing,
	time_samples,
	points
):
	num_times = len(points)
	thresholds = [i for i in range(start_thresh, end_thresh+1, spacing)]
	# times = [i for i in range(start_thresh,start_thresh+1+num_times*spacing,spacing)] 

	useddists = [get_dist(points[i]) for i in range(0,num_times,int(num_times/(time_samples-1)))]
	useddists = np.array(useddists)
	bettiarray = np.zeros((len(thresholds),len(useddists),len(useddists)))
	n = len(useddists)
	
	for i in range(n):
		dgms = get_dgms(useddists[i])
		ba = [get_betti_count(dgms,thresh) for thresh in thresholds]
		ba = np.array(ba)
		bettiarray[:,i,n-i-1] = ba

	for diag in range(n):
		for row in range(n-diag-1):
			newuseddists = np.minimum(useddists[row],useddists[row+1])
			dgms = get_dgms(newuseddists)
			useddists[row] = newuseddists
			ba = [get_betti_count(dgms,thresh) for thresh in thresholds]
			ba = np.array(ba)
			bettiarray[:,row,n-diag-row-2]=ba
	
	return bettiarray, n, thresholds


def write_betti_array(
	bettiarray,
	betti_file,
	start_thresh,
	end_thresh,
	spacing
):	
	infostring = str(n) + ' ' + str(start_thresh) + ' ' + str(end_thresh) + ' ' + str(spacing) 
	with open(betti_file, 'w') as outfile:
		if infostring:
			outfile.write(infostring + '\n')
		for data_slice in bettiarray:
			np.savetxt(outfile, data_slice, fmt='%d')


def read_betti_file(fname):

	with open(fname, 'r') as f1:
		firstline1 = f1.readline()
		info1 = firstline1.split()
		num_times = int(info1[0])
		start_thresh = int(info1[1])
		end_thresh = int(info1[2])
		spacing = int(info1[3])
	
	betti_array = np.loadtxt(fname, skiprows=1)

	return num_times, start_thresh, end_thresh, spacing, betti_array

if __name__ == '__main__':

	if len(sys.argv) == 7:
		position_file = str(sys.argv[1])
		betti_file = str(sys.argv[2])
		start_thresh = int(sys.argv[3])
		end_thresh = int(sys.argv[4])
		spacing = int(sys.argv[5])
		time_samples = int(sys.argv[6])
	else:
		print('Please pass arguments:')
		print('position filename, output filename, starting Rips threshold, ending Rips threshold, spacing, time_samples')
		exit()

	points = []
	num_times = -1
	for line in open(position_file).readlines():
		num_times += 1
	
	myfile = open(position_file)
	line1 = myfile.readline()
	num_points =  int(line1)
	num_times = int(num_times/num_points)
	for i in range(num_times):
		points.append(get_points(myfile,num_points))
	myfile.close()
	
	bettiarray, n, _ = get_betti_array(
		start_thresh,
		end_thresh,
		spacing,
		time_samples,
		points
	)

	write_betti_array(
		bettiarray, 
		betti_file,
		start_thresh,
		end_thresh,
		spacing
	)


