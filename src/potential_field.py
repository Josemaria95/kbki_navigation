#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
from functions_APF import repulsive_walls, attractive_potential
from geometry_msgs.msg import Twist , Pose
from nav_msgs.msg import Odometry
import rospy


class Value_kbki(): 												# Clase donde defino las variables que voy a tomar del Kobuki (posicion y orientacion)

	def __init__(self):

		self.kbki_orientation_x = 0.0
		self.kbki_orientation_y = 0.0
		self.kbki_orientation_z = 0.0
		self.kbki_orientation_w = 1.0
		self.kbki_position_x = 0.0
		self.kbki_position_y = 0.0

	def callback(self,msg):
		# Position values
		self.kbki_position_x = msg.pose.pose.position.x
		self.kbki_position_y = msg.pose.pose.position.y
		# Orientation value
		self.kbki_orientation_x = msg.pose.pose.orientation.x
		self.kbki_orientation_y = msg.pose.pose.orientation.y
		self.kbki_orientation_z = msg.pose.pose.orientation.z
		self.kbki_orientation_w = msg.pose.pose.orientation.w

# Logs
frep = open("/home/jm95/Documents/TESIS/test_APF/data_txt/frep_test.txt",'w')
fattr = open("/home/jm95/Documents/TESIS/test_APF/data_txt/fattr_test.txt",'w')
posicion_actual = open("/home/jm95/Documents/TESIS/test_APF/data_txt/posicion_actual_test.txt",'w')
#fvelocidad_cmd = open("/home/jm95/Documents/TESIS/test_controlpolar/data_txt/velocidad_cmd10.txt",'w')


# Kobuki class
kbki = Value_kbki() # Nombre que voy a tomar de la clase

# Init node ROS	 
rospy.init_node('test_kobuki', anonymous=True) 	# Creo nodo en ROS
rospy.Subscriber("/odom", Odometry, kbki.callback) 	# Subscripcion del topico "/odom"

# Obstacles
qobstacles = np.array ([[1.,1.],[2., 1.],[3., 3.]]) # Defino los obstaculos 

# Goal
qgoal = np.array([4.0, 4.0]) # Punto a donde se quiere llegar

# Parameters for repulsive field 
rho = 1.0 # Q*
eta = 0.5 # Ganancia del gradiente repulsivo 0.5
Umax = 1.0

# Parameters for attractive field
zeta = 0.5 # Ganancia del gradiente atractivo
dgoal = 1  # Distancia a la que el robot empieza a disminuir las velocidad

# Parameters for linear and angular velocity
delta_tiempo = 1.0

max_iter = 1000

#pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10) # Publicar velocidades (linear y angular) en el topico
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publicar velocidades (linear y angular) en el topico


freq = 5     # Frecuencia (Hz)
T = 1.0/freq # Periodo
rate = rospy.Rate(freq)

# Tiempo inicial
tinit = rospy.get_time()

for i in range(max_iter):
	#print("Inicio interaccion N: ",i)
	
	# Point Start
	point_start = np.array([kbki.kbki_position_x , kbki.kbki_position_y])	# Inicializar el punto de inicio de acuerdo a la odometria

	theta = 2 * np.arccos(kbki.kbki_orientation_w) * np.sign(kbki.kbki_orientation_z)  # Obtencion del angulo de rotacion del cuaternion
	# Matriz de rotacion
	matrix_rotation = np.array([[np.cos(theta), -np.sin(theta), 0.], [np.sin(theta), np.cos(theta), 0.], [0. , 0., 1.]])
	matrix_rotation = matrix_rotation.transpose()

	# Artificial Potential Field
	U_repulsive, F_repulsive = repulsive_walls(point_start, qobstacles, rho, eta, Umax)
	U_attractive, F_attractive = attractive_potential(point_start,qgoal,dgoal,zeta)

	#print ("F_rep: ", F_repulsive, "F_attr: ",F_attractive)

	# Logs
	t = rospy.get_time()-tinit
	fattr.write(str(t) + " " + str(F_attractive[0]) + " " + 
		str(F_attractive[1]) + "\n")
	frep.write(str(t) + " " + str(F_repulsive[0]) + " " + 
		str(F_repulsive[1]) + "\n")
	posicion_actual.write(str(t) + " " + str(point_start[0]) + " " +
		str(point_start[1]) + "\n")
	
	# Navigation Force in body frame
	F_total = F_attractive + F_repulsive
	#print("F_total: ", F_total)
	
	F_total_I = np.array([F_total[0], F_total[1], 1]).T 	# Fuerza de navegacion en un array
	#print ("F_total_I", F_total_I)
	F_total_kbki = np.dot(matrix_rotation, F_total_I)		# Fuerza de Navegacion con respecto al plano del kobuki
	#print ("F_total_kbki", F_total_kbki)

	# Magnitud and direction for F_navigation
	delta_theta =  1.0*np.arctan2(F_total_kbki[1], F_total_kbki[0])  # Direccion
	delta_x = np.sqrt(F_total_kbki[0]**2 + F_total_kbki[1]**2)		# Magnitud

	#print "Theta: ", np.rad2deg(theta), "Delta theta: ", np.rad2deg(delta_theta), "delta_x: ", delta_x

	# Linear and angular velocity
	velo_angular = delta_theta / delta_tiempo
	velo_lineal = delta_x / delta_tiempo

	# Message type
	gts = Twist()

	# Create message type
	# geometry_msgs/linear
	gts.linear.x = velo_lineal
	gts.linear.y = 0.0
	gts.linear.z = 0.0

	#print("Velocidad lineal:  ", velo_lineal)
	# geometry_msgs/angular
	gts.angular.x = 0.0
	gts.angular.y = 0.0	
	gts.angular.z = velo_angular
	#print("Velocidad angular:  ",velo_angular)

	if (np.linalg.norm(np.abs(point_start - qgoal)) < 0.05):
		print ("--------------------- Finish -----------------")
		break;

	print ("v: ", velo_lineal, "w: ",velo_angular)
	#print ("ang:", posicion_actual[2], "error:", errornorm, "v:",
	#		velo_lineal,"w: ",velo_angular)

	# Publisher node ROS
	pub.publish(gts)
	rate.sleep()

frep.close()
fattr.close()
posicion_actual.close()

rospy.spin()
