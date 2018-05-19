#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from functions_APF import normalizeAngle, getControlPolar
from geometry_msgs.msg import Twist , Pose
from nav_msgs.msg import Odometry
import rospy

class Value_kbki():

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
"""
def normalizeAngle(angle):
    # Setea el angulo en el rango de [pi,-pi)
    angulo = np.mod(angle + np.pi, 2*np.pi) - np.pi

    return angulo


def getControlPolar (X, Xdes,parametros):

    # Posicion y orientacion actuales
    x  = X[0]
    y  = X[1]
    th = X[2]

    # Posicion y orientacion del goal
    xdes  = Xdes[0]
    ydes  = Xdes[1]
    thdes = Xdes[2]

    # Parametros
    rho   = np.sqrt((xdes - x)**2 + (ydes - y)**2)
    lamda = np.arctan2(ydes - y, xdes - x)
    alpha = lamda - th 
    #alpha = np.arctan2(ydes - y, xdes - x) - th
    alpha = normalizeAngle(alpha)

    #print("alpha", np.rad2deg(alpha))

    if (np.abs(alpha) <= (np.pi/2)):
        #beta  = -np.arctan2(ydes - y,xdes - x) + thdes
        beta = thdes - lamda
        #beta  = normalizeAngle(beta)
        krho2 = parametros[0]

    else:
        #alpha = alpha - np.pi
        alpha = lamda - th - np.pi
        alpha = normalizeAngle(alpha)
        #beta = -np.arctan2(ydes - y,xdes - x) + thdes
        beta = thdes - lamda - np.pi 
        #beta = beta - np.pi
        beta = normalizeAngle(beta)
        krho2 = -parametros[0]

    vu = krho2 * rho;
    omega = parametros[2] * alpha + parametros[1] * beta;

    return (vu, omega)

"""
# Logs
fposicion_actual = open("/home/jm95/Documents/TESIS/test_controlpolar/data_txt/posicion_actualtesis.txt",'w')
fposicion_deseada = open("/home/jm95/Documents/TESIS/test_controlpolar/data_txt/posicion_deseadatesis.txt",'w')
fvelocidad_cmd = open("/home/jm95/Documents/TESIS/test_controlpolar/data_txt/velocidad_cmdtesis.txt",'w')

# Kobuki class
kbki = Value_kbki()

# Init node ROS	 
rospy.init_node('test_kobuki', anonymous=True)
rospy.Subscriber("/odom", Odometry, kbki.callback)

# Goal (x,y)
p_final = np.array([3.0,3.0])
qgoal = np.array([3.0,3.0,np.deg2rad(90)])

# Parametros del controlador polar lazo cerrado
# parametros = [krho, kbeta,kalpha]
krho =  0.2
kbeta = -0.2
dalpha = 0.05 #0.2
kalpha = (2.0/np.pi) * krho - (5.0/3.0) * kbeta + dalpha
parametros = np.array([krho, kbeta, kalpha])

max_iter = 1000

#pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

freq = 5     # Frecuencia (Hz)
T = 1.0/freq # Periodo
rate = rospy.Rate(freq)

# Tiempo inicial
tinit = rospy.get_time()

for i in range(max_iter):

	# Punto de inicio(x,y)
	#point_start = np.array([kbki.kbki_position_x , kbki.kbki_position_y])

	# Angulo actual (theta)
	theta_actual = 2 * np.arccos(kbki.kbki_orientation_w) * np.sign(kbki.kbki_orientation_z)

	#print("th_actual: ",np.rad2deg(theta_actual),'\n')

	# Posicion actual
	posicion_actual = np.array([kbki.kbki_position_x, kbki.kbki_position_y,theta_actual])

	# Logs
	t = rospy.get_time()-tinit
	fposicion_actual.write(str(t) + " " + str(posicion_actual[0]) + " " + 
		str(posicion_actual[1]) + " " + str(posicion_actual[2]) + "\n")
	fposicion_deseada.write(str(t) + " " + str(qgoal[0]) + " " + str(qgoal[1]) +
		" " + str(qgoal[2]) + "\n")

	# Control Polar
	#control_polar = getControlPolar(posicion_actual,qgoal,parametros)
	[velo_lineal, velo_angular]= getControlPolar(posicion_actual,qgoal,parametros)

	"""
	rho = control_polar[0]
	alpha = control_polar[1]
	beta = control_polar[2]
	"""

	#print("rho: ",rho,"alpha: ",alpha,"beta: ",beta,'\n')

	# Velocidades Lineal y angular 
	#velo_lineal = krho * rho
	#velo_angular = kalpha * alpha + kbeta * beta
	"""
	absVel = np.abs(velo_lineal)
	if (absVel > 1e-6):
		velo_lineal = velo_lineal/absVel * 2;
		velo_angular = velo_angular/absVel * 2;
	"""

	# Log velocidad
	fvelocidad_cmd.write(str(t) + " " + str(velo_lineal) + " " + str(velo_angular) + "\n")

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
	error = posicion_actual - qgoal
	W = np.array([[1., 0., 0.],[0., 1., 0.],[0., 0., 30.0]])
	#W = np.eye(3)
	errornorm = error.T.dot(W).dot(error)
	
	if (errornorm < 0.001):
		print ("--------------------- Finish -----------------")
		break;
	

	#print ("velo_lineal", velo_lineal,"velo_angular: ",velo_angular,'\n')
	print ("ang:", np.rad2deg(posicion_actual[2]), "error:", errornorm, "v:",
			velo_lineal,"w: ",velo_angular)

	pub.publish(gts)
	rate.sleep()


	"""
	if (np.linalg.norm(np.abs(point_start - p_final)) < 0.05):
		print ("--------------------- Finish -----------------")
		break;
	"""

fposicion_deseada.close()
fposicion_actual.close()
fvelocidad_cmd.close()

rospy.spin()

