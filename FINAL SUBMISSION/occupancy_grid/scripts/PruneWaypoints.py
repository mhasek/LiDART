import numpy as np
from numpy.linalg import *
import scipy as sp
from scipy import ndimage
from scipy import io
import matplotlib.pyplot as plt
import pdb
from scipy.signal import savgol_filter
from scipy import interpolate

def pathInterp(path,d):
	Interp = path[0,:].reshape(1,2)
	p0 = path[0,:]

	for i in np.arange(1,len(path)):
		p1 = path[i,:]
		dist = norm(p1-p0)
		pointCount = dist/d
		unitvec = (p1-p0)/pointCount
		
		x = np.linspace(p0[0]+unitvec[0],p1[0],int(pointCount))
		y = np.linspace(p0[1]+unitvec[0],p1[1],int(pointCount))
		tempPath = np.hstack((x.reshape(-1,1), y.reshape(-1,1)))
		Interp = np.vstack((Interp, tempPath));
		p0 = p1;

	Interp[-1] = Interp[0]
	return Interp


def dist2line(pt, v1, v2):

	pt = np.hstack((pt, np.zeros((len(pt),1))))
	v1 = np.hstack((v1, np.zeros((1,1))))
	v2 = np.hstack((v2, np.zeros((1,1))))
	a = v1 - v2
	b = pt - v2


	alpha = np.sum(a*b,axis=1)/np.sum(a*a)

	proj = alpha.reshape(-1,1)*a

	check = (norm(proj,axis=1) < norm(a)) & (alpha>0)

	d = norm(np.cross(a,b,axis = 1),axis=1)/norm(a)


	if np.sum(check) == 0:
		return 0
	else:
		return np.min(d[check])


def PruneWayPoints(map,res,path,d):
	(r,c) = np.where(map==0)
	r = res[0] * r 
	c = res[1] * c

	id = 1
	sparsePath = path[0,:]
	lastpts = path[0,:]

	while id < len(path)-1:

		count = 0;
		for j in np.arange(id,len(path),1):

			x = np.append(lastpts[1],path[j,1])*20

			y = np.append(lastpts[0],path[j,0])*20

			num = np.ceil(max(abs(np.diff(x)),abs(np.diff(y))))	

			xq = np.int32(np.linspace(x[0], x[1],np.int32(num*3)))
			yq = np.int32(np.linspace(y[0], y[1],np.int32(num*3)))

			val = map[yq,xq]

			dist = dist2line(np.array((r,c)).T,lastpts.reshape(1,2),path[j,:].reshape(1,2))
			
			cutswall = ~(np.sum(val == 255) == len(val))
			drivable = ~cutswall and (dist > d)
			
			if drivable:
				id  = j

			elif cutswall:
				break

		lastpts = path[id,:];
		id = id + 1
		print id
		sparsePath = np.vstack((sparsePath, lastpts))

	sparsePath[0,:] = lastpts
	return sparsePath



if __name__ == "__main__":
	map = plt.imread('/home/mhasek/Documents/ESE680/TrajGen/racemap.pgm')
	res = np.array([0.05,0.05])
	d = 0.6
	path = io.loadmat("1209rrt.mat")

	sparsePath = PruneWayPoints(map, res, path['path'], d)


	tck,u = interpolate.splprep(sparsePath.T,k=1,s=0)
	unew = np.arange(0, 1.01, 0.01)
	out = interpolate.splev(unew, tck)

	sparsePath = PruneWayPoints(map, res, np.hstack((out[0].reshape(-1,1),out[1].reshape(-1,1))), 0.4)

	sparsePath = pathInterp(sparsePath,0.3)

	tck,u = interpolate.splprep(sparsePath.T,k=2,s=1)
	unew = np.arange(0, 1.01, 0.01)
	out = interpolate.splev(unew, tck)

	path = np.hstack((out[0].reshape(-1,1),out[1].reshape(-1,1)))

