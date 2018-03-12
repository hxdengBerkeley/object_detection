import numpy as np 
import scipy as sp 
from numpy import linalg as LA
import sklearn.neighbors
import sklearn.metrics
import argparse

DOWN_SAMPLING = 200
NUM_FEATURES = 28

def feature_extraction(point_cloud):
	#point_cloud = np.loadtxt(filename)
	new_ind = np.random.choice(point_cloud.shape[0], DOWN_SAMPLING)
	new_ind.sort()
	point_cloud = point_cloud[new_ind]
	features = np.zeros((NUM_FEATURES))
	mod_cck = point_cloud.shape[0]
	
	# Histogram features
	features[0] = np.max(point_cloud[:,3])
	features[1] = np.mean(point_cloud[:,3])
	features[2] = np.var(point_cloud[:,3])
	l = np.max(point_cloud[:,0]) - np.min(point_cloud[:,0])
	w = np.max(point_cloud[:,1]) - np.min(point_cloud[:,1])
	h = np.max(point_cloud[:,2]) - np.min(point_cloud[:,2])
	features[3] = l * w * h

	lalonde_features = np.zeros((DOWN_SAMPLING,3))
	# Lalonde features are computed here
	point_cloud = point_cloud[:,:3]
	# For each point in a component obtain the 20 nearest neighbors within radius 0.5m 
	# perform eigen decomposition and normalize eigenvalues
	# Calculate L1 = d1, L2 = d1-d2, L3 = d2-d3, were d1>d2>d3 are eigen values
	tree = sklearn.neighbors.KDTree(point_cloud)
	for i in range(mod_cck):
		#print("Procssing point number {}".format(i))
		dist, ind = tree.query([point_cloud[i]], k=21)     
		dist = dist[0,1:]
		ind = ind[0,1:]
		ind = ind[(dist <= 0.5)[0]]
		if len(ind) > 0:
			ind = ind[0]
			cck = (point_cloud[ind, :]).transpose()
			cck[0,:] = cck[0,:] - np.mean(cck[0,:])
			cck[1,:] = cck[1,:] - np.mean(cck[1,:])
			cck[2,:] = cck[2,:] - np.mean(cck[2,:])
			cck_trans = (cck).transpose()
			cov_mat = np.matmul(cck , cck_trans)/len(ind)
			e_vals, e_vecs = LA.eigh(cov_mat)
			d_mat = np.array([e_vals[2], e_vals[1], e_vals[0]])
			dnorm  = d_mat/np.sum(d_mat)
			l1 = dnorm[0]
			l2 = dnorm[0] - dnorm[1]
			l3 = dnorm[1] - dnorm[2]
			lalonde_features[i] = [l1,l2,l3]
		else:
			lalonde_features[i] = [0,0,0]

	# Calculate the normalized histogram with 4 bins for each Lalonde feature
	features[4] = sum(lalonde_features[:,0] <= 0.25)/mod_cck
	features[5] = sum((lalonde_features[:,0] > 0.25) &  (lalonde_features[:,0] <= 0.5))/mod_cck
	features[6] = sum((lalonde_features[:,0] > 0.5) &  (lalonde_features[:,0] <= 0.75))/mod_cck
	features[7] = sum((lalonde_features[:,0] > 0.75) &  (lalonde_features[:,0] <= 1))/mod_cck

	features[8] = sum(lalonde_features[:,1] <= 0.25)/mod_cck
	features[9] = sum((lalonde_features[:,1] > 0.25) &  (lalonde_features[:,1] <= 0.5))/mod_cck
	features[10] = sum((lalonde_features[:,1] > 0.5) &  (lalonde_features[:,1] <= 0.75))/mod_cck
	features[11] = sum((lalonde_features[:,1] > 0.75) &  (lalonde_features[:,1] <= 1))/mod_cck

	features[12] = sum(lalonde_features[:,2] <= 0.25)/mod_cck
	features[13] = sum((lalonde_features[:,2] > 0.25) &  (lalonde_features[:,2] <= 0.5))/mod_cck
	features[14] = sum((lalonde_features[:,2] > 0.5) &  (lalonde_features[:,2] <= 0.75))/mod_cck
	features[15] = sum((lalonde_features[:,2] > 0.75) &  (lalonde_features[:,2] <= 1))/mod_cck

	# Anguelov features are computed here
	dist = sklearn.metrics.pairwise.pairwise_distances(point_cloud[:,:2])

	# For each point, obtain a right cylindrical region on top of it with radius 0.1 and height 2 m
	# treat the point is in the lower axes of the cylinder.
	# Split the right circular region into 3 parts and calculate the sum of points in each part which 
	# are the three Anguelov features A1, A2 and A3 
	anguelov_features = np.zeros((DOWN_SAMPLING,3))
	for pt in range(mod_cck):
		indexes_circ = np.where(dist[pt,:] < 0.1)
		z_values = point_cloud[indexes_circ[0],2]
		height_diff = point_cloud[pt,2] - z_values
		a1 = sum((height_diff >= 0) & (height_diff <= 0.6667))/len(height_diff)
		a2 = sum((height_diff > 0.6667) & (height_diff <= 1.3334))/len(height_diff)
		a3 = sum((height_diff > 1.3334) & (height_diff <= 2))/len(height_diff)
		anguelov_features[pt] = [a1, a2, a3]


	# Calculate the normalized histogram with 4 bins for each Anguelov feature
	features[16] = sum(anguelov_features[:,0] <= 0.25)/mod_cck
	features[17] = sum((anguelov_features[:,0] > 0.25) &  (anguelov_features[:,0] <= 0.5))/mod_cck
	features[18] = sum((anguelov_features[:,0] > 0.5) &  (anguelov_features[:,0] <= 0.75))/mod_cck
	features[19] = sum((anguelov_features[:,0] > 0.75) &  (anguelov_features[:,0] <= 1))/mod_cck

	features[20] = sum(anguelov_features[:,1] <= 0.25)/mod_cck
	features[21] = sum((anguelov_features[:,1] > 0.25) &  (anguelov_features[:,1] <= 0.5))/mod_cck
	features[22] = sum((anguelov_features[:,1] > 0.5) &  (anguelov_features[:,1] <= 0.75))/mod_cck
	features[23] = sum((anguelov_features[:,1] > 0.75) &  (anguelov_features[:,1] <= 1))/mod_cck

	features[24] = sum(anguelov_features[:,2] <= 0.25)/mod_cck
	features[25] = sum((anguelov_features[:,2] > 0.25) &  (anguelov_features[:,2] <= 0.5))/mod_cck
	features[26] = sum((anguelov_features[:,2] > 0.5) &  (anguelov_features[:,2] <= 0.75))/mod_cck
	features[27] = sum((anguelov_features[:,2] > 0.75) &  (anguelov_features[:,2] <= 1))/mod_cck

	return features


# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('source', help='list of frames to process')
#     args = parser.parse_args()
    
#     f = feature_extraction(args.source)
#     print(f)
#     return


# if __name__ == '__main__':
#     main()








