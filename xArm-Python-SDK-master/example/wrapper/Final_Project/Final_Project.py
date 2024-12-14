#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2121, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
UFactory Lite6 trajectory planning in task-space

Autores: 
    -> Miguel Meireles Teixeira, n.º2021217493
    -> Ângelo da Rocha Rodrigues, n.º2021236348

Entrega: 5 de Janeiro de 2025
"""

# ============ Importar as Bibliotecas ============
import os
import sys
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sympy as sp

# Expressões Matemáticas:
import math
from sympy import tensorproduct,shape, DotProduct, Matrix, pprint, Inverse, Subs
from numpy import eye, round
from math import sqrt, cos, sin, pi
from MGH_DH import MGH_DH


sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI # Importar a API do Robô

ip = 162.163

# ============ Conexão ao Robô ============

# Configuração do ip do Robô:
#if len(sys.argv) >= 2:
#    ip = sys.argv[1]
#else:
#    try:
#        from configparser import ConfigParser
#        parser = ConfigParser()
#        parser.read('../robot.conf')
#        ip = parser.get('xArm', 'ip')
#    except:
#        ip = input('Please input the xArm ip address:')
#        if not ip:
#            print('input error, exit')
#            sys.exit(1)



# ============ Variáveis ============

# Utiizando o modelo Modificado de DH:
Roll_x = pi/2
Pitch_y = -pi/2
Yaw_z = pi/2

alpha_velocity = pi/2
alpha = 0
iterationTime = 0.1

C = [100, 100, 100] # Inicialização do Centro
r = 20  # Incialização do raio

Lg = 61.5

Kp = 0.9
Ki = 0.04


# ============ Setup do Robô ============

UFactory_Lite = XArmAPI(ip)
#UFactory_Lite.motion_enable(enable=True)
#UFactory_Lite.set_mode(0)                     # after 4 -> for set_velocity() mode control 
#UFactory_Lite.set_state(state=0)
#UFactory_Lite.move_gohome(wait=True)          # Going to rest position 



# ============ Setup do Robô ============



# ============ Utilização dos Métodos ============

# Ciclo para escolher o Modo de Operação:
metodo = "0"

#while (metodo != "1" and metodo !="2"):
 #   os.system('cls' if os.name == 'nt' else 'clear') # Limpa o Terminal

 #   print("===================================")
 #   print("| [1] - Controlo em Malha Fechada |")
 #  print("| [2] - Por Definir               |")
  #  print("===================================")

#    metodo = input("Opção: ")


# Ciclo para escolher os valores de C e r:
while not (C[0] and C[1] and C[2]):
    os.system('cls' if os.name == 'nt' else 'clear')  # Limpa o terminal

    print("Introduza valores numéricos Centro e o Raio")
    C[0] = input("Cx: ")
    C[1] = input("Cy: ")
    C[2] = input("Cz: ")
    r = input("Raio: ")

print(f"\nOs valores são: C = [{C[0]}, {C[1]}, {C[2]}] e r = {r}")

# Conversão dos valores de C e r para double:
C = [float(C[0]), float(C[1]), float(C[2])]
r = float(r)


##############   Parâmetros DH do Robô      #####################

# Offset's
offset1=0
offset2=-pi/2
offset3=-pi/2
offset4=0
offset5=0
offset6=0

# D's
d1=243.3
d2=0
d3=0
d4=227.6
d5=0
d6=61.5

# A's
a1=0
a2=200
a3=87
a4=0
a5=0
a6=0

# Alphas 
alpha1=-pi/2
alpha2=pi
alpha3=pi/2
alpha4=pi/2
alpha5=-pi/2
alpha6=0

t1, t2, t3, t4, t5, t6 = sp.symbols('t1 t2 t3 t4 t5 t6') 


# Matriz DH do Robô 
DH_Matrix = sp.Array([[t1+offset1, d1, a1, alpha1],
                      [t2+offset2, d2, a2, alpha2],
                      [t3+offset3, d3, a3, alpha3],
                      [t4+offset4, d4, a4, alpha4],
                      [t5+offset5, d5, a5, alpha5],
                      [t6+offset6, d6, a6, alpha6]])


# Confirm the Transformation Matrices
[Transformation_Matrices,T_final] = MGH_DH(DH_Matrix)
T_final = sp.nsimplify(T_final, tolerance=1e-5)


# Simplify das matrizes
Transformation_Matrices[0] = sp.nsimplify(Transformation_Matrices[0],tolerance=1e-5)  # T01
Transformation_Matrices[1] = sp.nsimplify(Transformation_Matrices[1],tolerance=1e-5)  # T12
Transformation_Matrices[2] = sp.nsimplify(Transformation_Matrices[2],tolerance=1e-5)  # T23
Transformation_Matrices[3] = sp.nsimplify(Transformation_Matrices[3],tolerance=1e-5)  # T34
Transformation_Matrices[4] = sp.nsimplify(Transformation_Matrices[4],tolerance=1e-5)  # T45
Transformation_Matrices[5] = sp.nsimplify(Transformation_Matrices[5],tolerance=1e-5)  # T56


# Transformation Matrices
T_01_sym = Matrix(Transformation_Matrices[0])
T_02_sym = sp.Mul(T_01_sym, Matrix(Transformation_Matrices[1]), evaluate=False)
T_03_sym = sp.Mul(T_02_sym, Matrix(Transformation_Matrices[2]), evaluate=False)
T_04_sym = sp.Mul(T_03_sym, Matrix(Transformation_Matrices[3]), evaluate=False)
T_05_sym = sp.Mul(T_04_sym, Matrix(Transformation_Matrices[4]), evaluate=False)


# Simplified Transformation Matrices
#T_01_sym = sp.simplify(T_01_sym)
#T_02_sym = sp.simplify(T_02_sym)
#T_03_sym = sp.simplify(T_03_sym)
#T_04_sym = sp.simplify(T_04_sym)
#T_05_sym = sp.simplify(T_05_sym)


# Jacobiano

P_0G = T_final[0:3,3]
#print(P_0G.shape)
#pprint(P_0G)



# Jacobiano de velocidades lineares
Jac_v = sp.Array([
    [sp.diff(P_0G[0], t1), sp.diff(P_0G[0], t2), sp.diff(P_0G[0], t3), sp.diff(P_0G[0], t4), sp.diff(P_0G[0], t5), sp.diff(P_0G[0], t6)],
    [sp.diff(P_0G[1], t1), sp.diff(P_0G[1], t2), sp.diff(P_0G[1], t3), sp.diff(P_0G[1], t4), sp.diff(P_0G[1], t5), sp.diff(P_0G[1], t6)],
    [sp.diff(P_0G[2], t1), sp.diff(P_0G[2], t2), sp.diff(P_0G[2], t3), sp.diff(P_0G[2], t4), sp.diff(P_0G[2], t5), sp.diff(P_0G[2], t6)]
])
#pprint(Jac_v)


# Jacobiano de velocidades angulares
Jac_w = sp.Array([
    [0, T_01_sym[0, 2], T_02_sym[0, 2], T_03_sym[0, 2], T_04_sym[0, 2], T_05_sym[0, 2]],
    [0, T_01_sym[1, 2], T_02_sym[1, 2], T_03_sym[1, 2], T_04_sym[1, 2], T_05_sym[1, 2]],
    [1 ,T_01_sym[2, 2], T_02_sym[2, 2], T_03_sym[2, 2], T_04_sym[2, 2], T_05_sym[2, 2]]
])

#pprint(Jac_w.shape)
#pprint(Jac_v.shape)

#pprint(Jac_w)

# Jacobiano Completo
J0R = sp.Array([[Jac_v[0,0], Jac_v[0,1], Jac_v[0,2], Jac_v[0,3], Jac_v[0,4], Jac_v[0,5]], 
                [Jac_v[1,0], Jac_v[1,1], Jac_v[1,2], Jac_v[1,3], Jac_v[1,4], Jac_v[1,5]],
                [Jac_v[2,0], Jac_v[2,1], Jac_v[2,2], Jac_v[2,3], Jac_v[2,4], Jac_v[2,5]], 
                [Jac_w[0,0], Jac_w[0,1], Jac_w[0,2], Jac_w[0,3], Jac_w[0,4], Jac_w[0,5]],
                [Jac_w[1,0], Jac_w[1,1], Jac_w[1,2], Jac_w[1,3], Jac_w[1,4], Jac_w[1,5]],
                [Jac_w[2,0], Jac_w[2,1], Jac_w[2,2], Jac_w[2,3], Jac_w[2,4], Jac_w[2,5]],
                ])

J0R = sp.nsimplify(J0R, tolerance = 1e-5)

#pprint(J0R.shape)
#pprint(J0R)


# Defining the cartisian velocities
cartisian_velocities = np.array([           0,
                 -r*sin(alpha)*alpha_velocity,
 r*(cos(alpha)**2-sin(alpha)**2)*alpha_velocity,
                                            0,
                                            0,
                                            0])



# Definição da posição inicial

p_x = C[0]                               #  Compensação por um dos robôs não ter Gripper
p_y = C[1] + r * cos(alpha)
p_z = C[2] + r * cos(alpha) * sin(alpha)

# Get the joint angles for initial position
Pos_ini_angles=UFactory_Lite.get_inverse_kinematics([p_x, p_y, p_z, Roll_x, Pitch_y, Yaw_z],input_is_radian=True, return_is_radian=True)
config_rads = Pos_ini_angles

config_rads = UFactory_Lite.set_tool_position([p_x, p_y, p_z, Roll_x, Pitch_y, Yaw_z], speed = alpha_velocity, wait = True, is_radian = True)
UFactory_Lite.set_mode(4) # modo de velocidades


###########################   Start Section  ######################


T_0G = UFactory_Lite.get_forward_kinematics(Pos_ini_angles,input_is_radian=True, return_is_radian=True)

pprint(T_0G)

py_g_i = T_0G[2]
pz_g_i = T_0G[3]

#alpha_i = alpha + alpha_velocity * iterationTime
alpha_i = alpha

# Variables for PI controller
integrative_error_vy = 0
integrative_error_vz = 0

N_voltas = 1

# Se o Método escolhido for 1

while alpha_i < N_voltas*2*pi:

    #J0R_red_subs = eval(subs(J0R,[t1 t2 t3 t4 t5 t6],config_rads(1:6)));
    J0R_red_subs = Subs(J0R,[t1, t2, t3, t4, t5, t6], config_rads)
    cartisian_velocities = np.array([           0,
                    -r*sin(alpha_i)*alpha_velocity,
    r*(cos(alpha_i)**2-sin(alpha_i)**2)*alpha_velocity,
                                                0,
                                                0,
                                                0])
    
    T_0G = UFactory_Lite.get_forward_kinematics(config_rads, input_is_radian=True, return_is_radian=True)

    # Real position values

    py_g_r = T_0G[1]
    pz_g_r = T_0G[2]


    error_y = py_g_i - py_g_r
    error_z = pz_g_i - pz_g_r

    vel = prop_vel = Inverse(J0R_red_subs) * cartisian_velocities
    # velocidade de compensação para parte proporcional
    vyy = (py_g_i - py_g_r)/iterationTime
    vzz = (pz_g_i - pz_g_r)/iterationTime

    # velocidade de compensação para parte integrativa
    integrative_error_vy = integrative_error_vy + vyy
    integrative_error_vz = integrative_error_vz + vzz


     # velocidade de erro proporcional
    prop_vel = Inverse(J0R_red_subs) * sp.Array([0, vyy, vzz, 0, 0, 0])~
    
    # veocidrade de erro integrativo
    vel_integrative = Inverse(J0R_red_subs) * sp.Array([0, integrative_error_vy, integrative_error_vz, 0, 0, 0])


    
    # aqui que mandamos as velocidades

    vel_config = vel + Kp * prop_vel + Ki * vel_integrative 

    
    
    aux_config = config_rads + Kp * np.array([vel[0], vel[1],vel[2], vel[3], vel[4],vel[5]]) * iterationTime + Ki * np.array([vel_integrative[0], vel_integrative[1],vel_integrative[2], vel_integrative[3], vel_integrative[4],vel_integrative[5]]) * iterationTime
    
    py_g_i = py_g_i + cartisian_velocities[1] * iterationTime
    pz_g_i = pz_g_i + cartisian_velocities[2] * iterationTime

    alpha_i = alpha_i + alpha_velocity * iterationTime
    

    #UFactory_Lite.plot(config_rads, 'view', 'y')

    UFactory_Lite.vc_set_joint_velocity(vel_config,is_radian=True)

    config_rads = aux_config
  




# Se o Método escolhido for 2


while alpha_i < N_voltas*2*pi:

    #J0R_red_subs = eval(subs(J0R,[t1 t2 t3 t4 t5 t6],config_rads(1:6)));
    J0R_red_subs = Subs(J0R,[t1, t2, t3, t4, t5, t6], config_rads)
    cartisian_velocities = np.array([           0,
                        -r*sin(alpha_i)*alpha_velocity,
    r*(cos(alpha_i)**2-sin(alpha_i)**2)*alpha_velocity,
                                                0,
                                                0,
                                                0])
    

    vel = Inverse(J0R_red_subs) * cartisian_velocities

    


    aux_config = config_rads + (np.array([vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]]) * iterationTime)
    
    alpha = alpha + alpha_velocity * iterationTime

    #UFactory_Lite6.plot(config_rads,'view','y')

    UFactory_Lite.vc_set_joint_velocity(vel, is_radian=True)

    config_rads = aux_config
      


# ============ Fim dos Métodos ============

#UFactory_Lite.move_gohome(wait=True)
#UFactory_Lite.disconnect()





