#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist , Pose
from functions_APF import repulsive_walls, attractive_potential, normalizeAngle, getControlPolar
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
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

# Kobuki class
kbki = Value_kbki() # Nombre que voy a tomar de la clase

# Init node ROS	 
rospy.init_node('test_kobuki', anonymous=True) 	# Creo nodo en ROS
rospy.Subscriber("/odom", Odometry, kbki.callback) 	# Subscripcion del topico "/odom"

# Obstacles (x,y) 
#qobstacles = np.array ([[1.,2.],[3., 1.],[4., 3.]]) # Defino los obstaculos 

# Goal
xgoal = 4.0
ygoal = 4.0
thgoal = -45
#p_final =  np.array([4.0, 4.0]) # Punto a donde se quiere llegar
qgoal = np.array([xgoal,ygoal,np.deg2rad(thgoal)])

# Parameters for repulsive field 
rho = 1.0 # Q*
eta = 2.0 # Ganancia del gradiente repulsivo
Umax = 1.0

# Parameters for attractive field
zeta = 0.5 # Ganancia del gradiente atractivo
dgoal = 1  # Distancia a la que el robot empieza a disminuir las velocidad

# Parametros del controlador polar lazo cerrado
# parametros = [krho, kbeta,kalpha]
krho =  0.6
kbeta = -0.6
dalpha = 0.05 #0.2
kalpha = (2.0/np.pi) * krho - (5.0/3.0) * kbeta + dalpha
parametros = np.array([krho, kbeta, kalpha])

max_iter = 1000

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

freq = 5     # Frecuencia (Hz)
T = 1.0/freq # Periodo
rate = rospy.Rate(freq)

# Tiempo inicial
tinit = rospy.get_time()

for i in range(max_iter):
	#print("Inicio interaccion N: ",i)
	
	# Point Start
	#point_start = np.array([kbki.kbki_position_x , kbki.kbki_position_y])	# Inicializar el punto de inicio de acuerdo a la odometria
	xi = kbki.kbki_position_x
	yi = kbki.kbki_position_y
	thi = 2 * np.arccos(kbki.kbki_orientation_w) * np.sign(kbki.kbki_orientation_z)  # Obtencion del angulo de rotacion del cuaternion
	
	# Posicion actual
	posicion_actual = np.array([xi,yi,thi])

	# Artificial Potential Field
	U_repulsive, F_repulsive = repulsive_walls(np.array([xi,yi]), qobstacles, rho, eta, Umax)
	U_attractive, F_attractive = attractive_potential(np.array([xi,yi]),np.array([xgoal,ygoal]),dgoal,zeta)

	#print ("F_rep: ", F_repulsive, "F_attr: ",F_attractive)
	
	# Navigation Force in body frame
	F_total = F_attractive + F_repulsive
	#print("F_total: ", F_total)
	
	# Magnitud and direction for F_navigation
	direction_Ftotal =  1.0*np.arctan2(F_total[1], F_total[0])  # Direccion
	magnitud_Ftotal = np.sqrt(F_total[0]**2 + F_total[1]**2)		# Magnitud

	# Posiciones temporales deseadas
	xdes = posicion_actual[0] + (magnitud_Ftotal * np.cos(direction_Ftotal))
	ydes = posicion_actual[1] + (magnitud_Ftotal * np.sin(direction_Ftotal))
	thdes = direction_Ftotal
	#print ("thdes: ", thdes)
	posicion_deseada = np.array([xdes, ydes,thdes])

	# Control Polar
	#control_polar = getControlPolar(posicion_actual,qgoal,parametros)
	[velo_lineal, velo_angular]= getControlPolar(posicion_actual,posicion_deseada,parametros)
	"""
	if (np.linalg.norm(posicion_actual[0:2]-qgoal[0:2]) < 0.1):
		posicion_deseada[2] = np.deg2rad(thgoal)
		[velo_lineal, velo_angular]= getControlPolar(posicion_actual,posicion_deseada,parametros)
		print ("****************************************************")
	else:
		[velo_lineal, velo_angular]= getControlPolar(posicion_actual,posicion_deseada,parametros)
	"""

	if (np.linalg.norm(posicion_actual[0:2] - qgoal[0:2]) < 0.01):
		kp = 0.05
		velo_angular = kp * (posicion_actual[2] - np.deg2rad(thgoal))
		velo_lineal = 0.0
		print("================================================")


	#print (posicion_deseada)
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


	#errornorm = np.linalg.norm(posicion_actual- qgoal)
	#print (posicion_actual[2])
	posicion_actual[2] = normalizeAngle(posicion_actual[2])
	error = posicion_actual - qgoal
	
	#print ("error: ", error)
	W = np.array([[1., 0., 0.],[0., 1., 0.],[0., 0., 10.0]])
	#W = np.eye(3)
	errornorm = error.T.dot(W).dot(error)
	
	if (errornorm < 0.05):
		print ("--------------------- Finish -----------------")
		break;


	#print ("velo_lineal", velo_lineal,"velo_angular: ",velo_angular,'\n')
	#print ("ang:", np.rad2deg(posicion_actual[2]), "error:", errornorm, "v:",
	#		velo_lineal,"w: ",velo_angular)

	pub.publish(gts)
	rate.sleep()

frep.close()
fattr.close()
odometria.close()

rospy.spin()