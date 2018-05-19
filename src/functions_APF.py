from __future__ import print_function
import numpy as np

def attractive_potential (qc,qgoal,dgoal,zeta=1):
    dist = np.linalg.norm(qc - qgoal)
    if (dist <= dgoal):
        U_attractive = 0.5 * zeta * dist * dist
        F_attractive = -zeta * (qc - qgoal)
    else:
        U_attractive = dgoal * zeta * dist - 0.5 * zeta * dgoal ** 2
        F_attractive = -dgoal * zeta * (qc - qgoal) / dist
    
    return U_attractive, F_attractive

def repulsive_potential (qc, qobstacles, rho, eta=1, Umax=1):
    dist = np.linalg.norm(qc - qobstacles)
    if (dist <= rho):
        if (dist == 0):
            U_repulsive = 100
            F_repulsive = np.zeros(2,)
        else:
            U_repulsive = 0.5 * eta * (1/dist - 1/rho) ** 2
            F_repulsive = eta * (1/dist - 1/rho) * (1/dist ** 2) * (qc - qobstacles)/dist
            
        if (U_repulsive > Umax):
            U_repulsive = Umax
    else:
        U_repulsive = 0
        F_repulsive = np.zeros(2,)
    
    return U_repulsive, F_repulsive


def repulsive_walls (qc, qobstacles, rho, eta=1, Umax=1):
    U_total = 0
    F_total = np.zeros(2,)
    for i in range(len(qobstacles)):
        U_repulsive, F_repulsive = repulsive_potential(qc, qobstacles[i,:], rho, eta, Umax)
        U_total += U_repulsive
        F_total += F_repulsive
    return U_total, F_total

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




