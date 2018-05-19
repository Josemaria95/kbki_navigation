import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from functions_APF import repulsive_walls, attractive_potential

def main():
	# Obstacles
	#qobstacles = np.array ([[10.0, 10.0],[10.0, 14.0], [10.0, 9.0], [13.0, 17.0]])
	qobstacles = np.array([[5., 15.],[10., 15.], [10., 12.,], [11., 9.]])
	#qobstacles = np.array ([[5.,15.]])
	# Goal
	qgoal = np.array([18., 18.])
	# Grid
	xx = np.arange (0,20,1)
	yy = np.arange (0,22,1)
	x, y = np.meshgrid(xx,yy)
	# Parameters for repulsive field 
	rho = 3.0
	eta = 2.0
	Umax = 20
	# Parameters for attractive field
	zeta = 0.5
	dgoal = 2				


	# Algorithm
	U_total = np.zeros(x.shape)
	F_total = np.zeros((x.shape[0], x.shape[1], 2))
	for i in range(x.shape[0]):
		for j in range (x.shape[1]):
			U_repulsive, F_repulsive = repulsive_walls (np.array([x[i,j], y[i,j]]), qobstacles, rho, eta, Umax)
			U_attractive, F_attractive = attractive_potential (np.array([x[i,j], y[i,j]]), qgoal, dgoal, zeta)
			U_total [i,j] = U_repulsive + U_attractive
			#U_total [i,j] = U_attractive
			F_total [i,j,:] = F_repulsive + F_attractive
			#F_total [i,j,:] = F_attractive

	plt.title('Fuerzas de Navegacion')
	plt.imshow(U_total)
	plt.quiver(x,y, F_total [:,:,0], F_total[:,:,1])
	#plt.show()

	# Point Start
	point_start = np.array([0.,0.])
	trajectory = point_start
	max_iter = 10000
	for i in range(max_iter):
		U_repulsive, F_repulsive = repulsive_walls (point_start, qobstacles, rho, eta, Umax)
		U_attractive, F_attractive = attractive_potential (point_start, qgoal, dgoal, zeta)
		point_start = point_start + 0.1 * (F_repulsive + F_attractive)
		#point_start = point_start + 0.1 * (F_attractive)
		print (point_start)
		trajectory = np.vstack ([trajectory, point_start])
		if (np.linalg.norm(point_start - qgoal) < 0.1):
			break;
	print i
	plt.axis([xx[0],xx[-1],yy[0],yy[-1]])
	plt.plot(trajectory[:,0], trajectory[:,1], 'k')
	plt.plot(trajectory[:,0], trajectory[:,1], 'ko')
	plt.plot(trajectory[:,0], trajectory[:,1], 'ro')


	fig = plt.figure()
	ax = fig.gca(projection='3d')
	surf = ax.plot_surface(x, y, U_total, cmap = cm.coolwarm, rstride=1, cstride=1,
                           linewidth=0, antialiased=False)

	plt.title('Campo Potencial de Navegacion')
	plt.show()


if __name__=='__main__':
    main()