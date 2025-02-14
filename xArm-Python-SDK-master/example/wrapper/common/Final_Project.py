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

# =============================== Importar as Bibliotecas ===============================
import os
import sys
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sympy as sp

#  ==> Expressões Matemáticas:
import math
from sympy import tensorproduct, shape, DotProduct, Matrix, pprint, Subs
from numpy import eye, round
from math import sqrt, cos, sin, pi
from MGH_DH import MGH_DH

# ==> Funções auxiliares (functions.py):
import functions
from functions import clc
from functions import opcoes_menu
from functions import constantes_infinito
from functions import constantes_elipse
from functions import constantes_circunferencia
from functions import Compute_cartesian_velocity
from functions import Compute_Inical_Position
from functions import Compute_Roll_Pitch_Yaw
from functions import Compute_Position
from functions import Compute_PI_Velocity_Errors
from functions import Final_Plot
from functions import Final_Plot_2

# ==> API do Robô:
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI 





# =============================== Setup do Robô ===============================
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)

UFactory_Lite = XArmAPI(ip)
UFactory_Lite.set_simulation_robot(on_off=True) # Simulação do Robô
UFactory_Lite.motion_enable(enable=True)
UFactory_Lite.set_mode(0)                     
UFactory_Lite.set_state(state=0)
UFactory_Lite.move_gohome(wait=True)    # Ir para a Posição Inicial





# =============================== Variáveis ===============================
# ==> Inicialização das Constantes das Equações: 
C = ["Cx", "Cy", "Cz"] 
r = "r"  
r_a = "r_a" 
r_b = "r_b" 

# ==> Constantes do Controlador:
Kp = 1     # Ganho Porporcional
Ki = 0.05  # Ganho Integral

# ==> Variáveis Simbólicas:
t1, t2, t3, t4, t5, t6 = sp.symbols('t1 t2 t3 t4 t5 t6') 

alpha_velocity = pi/2
alpha = 0
iterationTime = float(0.1)





# =============================== Menu de Opções ===============================
# ==> Menu de Opções:
opcao, metodo = opcoes_menu()

# -> Equação do Infinito:
if (opcao=="1" or opcao=="2"):                                  
    C[0], C[1], C[2], r = constantes_infinito(C, r)

    C = [float(C[0]), float(C[1]), float(C[2])] # Conversão para float
    r = float(r)

# -> Equação da Elipse:
elif (opcao=="3" or opcao=="4"):
    C[0], C[1], C[2], r_a, r_b = constantes_elipse(C, r_a, r_b)

    C = [float(C[0]), float(C[1]), float(C[2])] # Conversão para float
    r_a = float(r_a)
    r_b = float(r_b)

# -> Equação da Circunferência:
elif (opcao=="5" or opcao=="6"):
    C[0], C[1], C[2], r = constantes_circunferencia(C, r)

    C = [float(C[0]), float(C[1]), float(C[2])] #
    r = float(r)    


# ==> Cálculo do [R, P, Y] baseado na Equação:
Roll_x, Pitch_y, Yaw_z = Compute_Roll_Pitch_Yaw(opcao)





# =============================== Cálculo da Matriz DH ===============================
# ==> Theta Offsets:
offset1 = 0
offset2 = -pi/2
offset3 = -pi/2
offset4 = 0
offset5 = 0
offset6 = 0

# ==> d's:
d1 = 243.3
d2 = 0
d3 = 0
d4 = 227.6
d5 = 0
d6 = 61.5

# ==> a's
a1 = 0
a2 = 200
a3 = 87
a4 = 0
a5 = 0
a6 = 0

# ==> alphas 
alpha1 = -pi/2
alpha2 = pi
alpha3 = pi/2
alpha4 = pi/2
alpha5 = -pi/2
alpha6 = 0

# ==> Matriz DH do Robô 
DH_Matrix = sp.Array([[t1+offset1, d1, a1, alpha1],
                      [t2+offset2, d2, a2, alpha2],
                      [t3+offset3, d3, a3, alpha3],
                      [t4+offset4, d4, a4, alpha4],
                      [t5+offset5, d5, a5, alpha5],
                      [t6+offset6, d6, a6, alpha6]])





# =============================== Cálculo das Transformações Simbólicas ===============================
# ==> Cálculo das Transformações Simbólicas:
[Transformation_Matrices, T_final] = MGH_DH(DH_Matrix)

# ==> Simplificação das Matrizes:
T_final = sp.nsimplify(T_final, tolerance=1e-5)                                       # T_final
Transformation_Matrices[0] = sp.nsimplify(Transformation_Matrices[0],tolerance=1e-5)  # T01
Transformation_Matrices[1] = sp.nsimplify(Transformation_Matrices[1],tolerance=1e-5)  # T12
Transformation_Matrices[2] = sp.nsimplify(Transformation_Matrices[2],tolerance=1e-5)  # T23
Transformation_Matrices[3] = sp.nsimplify(Transformation_Matrices[3],tolerance=1e-5)  # T34
Transformation_Matrices[4] = sp.nsimplify(Transformation_Matrices[4],tolerance=1e-5)  # T45
Transformation_Matrices[5] = sp.nsimplify(Transformation_Matrices[5],tolerance=1e-5)  # T56


# ==> Cálculo das Transformações T_0i:
T_01_sym = Matrix(Transformation_Matrices[0])
T_02_sym = sp.Mul(T_01_sym, Matrix(Transformation_Matrices[1]), evaluate=False)
T_03_sym = sp.Mul(T_02_sym, Matrix(Transformation_Matrices[2]), evaluate=False)
T_04_sym = sp.Mul(T_03_sym, Matrix(Transformation_Matrices[3]), evaluate=False)
T_05_sym = sp.Mul(T_04_sym, Matrix(Transformation_Matrices[4]), evaluate=False)





# =============================== Cálculo do Jacobiano Simbólico ===============================
P_0G = T_final[0:3,3] # Posição do Gripper 

# ==> Jacobiano de Velocidades Lineares
Jac_v = sp.Array([
    [sp.diff(P_0G[0], t1), sp.diff(P_0G[0], t2), sp.diff(P_0G[0], t3), sp.diff(P_0G[0], t4), sp.diff(P_0G[0], t5), sp.diff(P_0G[0], t6)],
    [sp.diff(P_0G[1], t1), sp.diff(P_0G[1], t2), sp.diff(P_0G[1], t3), sp.diff(P_0G[1], t4), sp.diff(P_0G[1], t5), sp.diff(P_0G[1], t6)],
    [sp.diff(P_0G[2], t1), sp.diff(P_0G[2], t2), sp.diff(P_0G[2], t3), sp.diff(P_0G[2], t4), sp.diff(P_0G[2], t5), sp.diff(P_0G[2], t6)]
])

#  ==> Jacobiano de Velocidades Angulares
Jac_w = sp.Array([
    [0, T_01_sym[0, 2], T_02_sym[0, 2], T_03_sym[0, 2], T_04_sym[0, 2], T_05_sym[0, 2]],
    [0, T_01_sym[1, 2], T_02_sym[1, 2], T_03_sym[1, 2], T_04_sym[1, 2], T_05_sym[1, 2]],
    [1 ,T_01_sym[2, 2], T_02_sym[2, 2], T_03_sym[2, 2], T_04_sym[2, 2], T_05_sym[2, 2]]
])

# ==> Jacobiano Completo
J0R = sp.Array([[Jac_v[0,0], Jac_v[0,1], Jac_v[0,2], Jac_v[0,3], Jac_v[0,4], Jac_v[0,5]], 
                [Jac_v[1,0], Jac_v[1,1], Jac_v[1,2], Jac_v[1,3], Jac_v[1,4], Jac_v[1,5]],
                [Jac_v[2,0], Jac_v[2,1], Jac_v[2,2], Jac_v[2,3], Jac_v[2,4], Jac_v[2,5]], 
                [Jac_w[0,0], Jac_w[0,1], Jac_w[0,2], Jac_w[0,3], Jac_w[0,4], Jac_w[0,5]],
                [Jac_w[1,0], Jac_w[1,1], Jac_w[1,2], Jac_w[1,3], Jac_w[1,4], Jac_w[1,5]],
                [Jac_w[2,0], Jac_w[2,1], Jac_w[2,2], Jac_w[2,3], Jac_w[2,4], Jac_w[2,5]],
                ])

J0R = sp.nsimplify(J0R, tolerance = 1e-5) # Simplificação do Jacobiano





# =============================== Definição das velocidades Cartesianas ===============================
# ==> Cálculo das velocidades cartesianas:
cartesian_velocities = Compute_cartesian_velocity(opcao, r, r_a, r_b, alpha, alpha_velocity)





# =============================== Definição da Posição Inicial ===============================
# ==> Cálculo da Posição Inical do Gripper:
p_x, p_y, p_z = Compute_Inical_Position(opcao, C, r, r_a, r_b, alpha)

# ==> Cálculo da Cinemática Inversa para a Posição Inicial do Gripper:
Pos_ini_angles = UFactory_Lite.get_inverse_kinematics([p_x, p_y, p_z, Roll_x, Pitch_y, Yaw_z], input_is_radian=True, return_is_radian=True)
config_rads = Pos_ini_angles[1]
config_rads = config_rads[:6] # Ângulas das 6 Juntas

# ==> Coloca o Robô na posição Inicial:
UFactory_Lite.set_position(p_x, p_y, p_z, Roll_x, Pitch_y, Yaw_z, speed=200, wait=True, is_radian=True)
UFactory_Lite.set_mode(4) # Modo de Velocidades (Aceita Velocidades)





# =============================== Definição da Posição Ideal ===============================
# ==> Cálculo da Transformação T_0G para a Posição Inicial do Gripper:
T_0G_aux = UFactory_Lite.get_forward_kinematics(Pos_ini_angles[1], input_is_radian=True, return_is_radian=True)
T_0G = T_0G_aux[1]

# ==> Cálculo das Posições ideiais:
p1_g_i, p2_g_i = Compute_Position(opcao, T_0G)

alpha_i = alpha # Inicialização do Alpha_Ideal





# =============================== Inícios dos Métodos ===============================
N_voltas = 4 # Número de Voltas

# Variables for PI controller
integrative_error_v1 = 0
integrative_error_v2 = 0

# ==> Erro:
error_1_array_plot = [] 
error_2_array_plot = []

# ==> Posições no Plano:
p1_g_r_array_plot = [] 
p2_g_r_array_plot = []

# ==> Posições das Juntas:
config_rads_array_plot_1 = [] # Posição Junta 1
config_rads_array_plot_2 = [] # Posição Junta 2
config_rads_array_plot_3 = [] # Posição Junta 3
config_rads_array_plot_4 = [] # Posição Junta 4
config_rads_array_plot_5 = [] # Posição Junta 5
config_rads_array_plot_6 = [] # Posição Junta 6

# ==> Velocidades das Juntas:
vel_config_array_plot_1 = [] # Velocidades Junta 1
vel_config_array_plot_2 = [] # Velocidades Junta 2
vel_config_array_plot_3 = [] # Velocidades Junta 3
vel_config_array_plot_4 = [] # Velocidades Junta 4
vel_config_array_plot_5 = [] # Velocidades Junta 5
vel_config_array_plot_6 = [] # Velocidades Junta 6

# ==> Velocidades Cartesianas:
cartesian_velocities_array_plot_1 = [] # Velocidades Cartesianas
cartesian_velocities_array_plot_2 = [] # Velocidades Cartesianas

UFactory_Lite.set_mode(4)
UFactory_Lite.set_state(state=0)





# =============================== Método 1 ===============================
if metodo == "1":
    while alpha_i < N_voltas*2*pi:
        startTime = time.monotonic() # Início do Relógio

        # ==> Cálculo do Jacobiano Simplificado:
        J0R_red_subs = J0R.subs([(t1,config_rads[0]), (t2,config_rads[1]), (t3,config_rads[2]), (t4,config_rads[3]), (t5,config_rads[4]), (t6,config_rads[5])])
        
        J0R_red_subs = Matrix(J0R_red_subs)
        J0R_red_subs = np.array(J0R_red_subs.evalf(), dtype=float) # Conversão para Numpy Array

        config_rads_array_plot_1.append(config_rads[0])
        config_rads_array_plot_2.append(config_rads[1])
        config_rads_array_plot_3.append(config_rads[2])
        config_rads_array_plot_4.append(config_rads[3])
        config_rads_array_plot_5.append(config_rads[4])
        config_rads_array_plot_6.append(config_rads[5])

        # ==> Cálculo das Equações da Velocidade Cartesiana:
        cartisian_velocities = Compute_cartesian_velocity(opcao, r, r_a, r_b, alpha_i, alpha_velocity)

        # -> Plano Z
        if (opcao == "1" or opcao == "3" or opcao == "5"): 
            cartesian_velocities_array_plot_1.append(cartisian_velocities[0])
            cartesian_velocities_array_plot_2.append(cartisian_velocities[1])

        # -> Plano X
        elif(opcao == "2" or opcao == "4" or opcao == "6"): 
            cartesian_velocities_array_plot_1.append(cartisian_velocities[1])
            cartesian_velocities_array_plot_2.append(cartisian_velocities[2])
        



        # ==> Cálculo das Posições Reais:
        T_0G_aux = UFactory_Lite.get_forward_kinematics(config_rads, input_is_radian=True, return_is_radian=True)
        T_0G = T_0G_aux[1]

        p1_g_r, p2_g_r = Compute_Position(opcao, T_0G)
        p1_g_r_array_plot.append(p1_g_r)
        p2_g_r_array_plot.append(p2_g_r)

        # ==> Cálculo do Erro entre Posições Ideais e Reais:
        error_1 = p1_g_i - p1_g_r
        error_2 = p2_g_i - p2_g_r
        error_1_array_plot.append(error_1)
        error_2_array_plot.append(error_2)

        
        # ==> Cálculo das Velocidades das Juntas:
        vel = np.linalg.inv(J0R_red_subs) @ cartisian_velocities

        # ==> Cálculo do Erro da Velocidade Proporcional:
        v1 = (p1_g_i - p1_g_r)/iterationTime
        v2 = (p2_g_i - p2_g_r)/iterationTime

        # ==> Cálculo do Erro da Velocidade Integrativo:
        integrative_error_v1 = integrative_error_v1 + v1
        integrative_error_v2 = integrative_error_v2 + v2

        # ==> Cálculo das Vecolidades das Juntas:
        prop_vel, vel_integrative = Compute_PI_Velocity_Errors(opcao, v1, v2, integrative_error_v1, integrative_error_v2, J0R_red_subs)
        
        # ==> Cálculo das Velocidades das Juntas compensada com PI:
        vel_config = vel.T + Kp * prop_vel.T + Ki * vel_integrative.T 

        vel_config_array_plot_1.append(vel_config[0])
        vel_config_array_plot_2.append(vel_config[1])
        vel_config_array_plot_3.append(vel_config[2])
        vel_config_array_plot_4.append(vel_config[3])
        vel_config_array_plot_5.append(vel_config[4])
        vel_config_array_plot_6.append(vel_config[5])

        # ==> Cálculo das Posições das Juntas compensada com PI:
        aux_config = config_rads + vel_config.T * iterationTime

        
        if (opcao == "1" or opcao == "3" or opcao == "5"): # -> Plano Z:
            p1_g_i = p1_g_i + cartisian_velocities[0] * iterationTime
            p2_g_i = p2_g_i + cartisian_velocities[1] * iterationTime
        
        elif(opcao == "2" or opcao == "4" or opcao == "6"): # -> Plano X:
            p1_g_i = p1_g_i + cartisian_velocities[1] * iterationTime
            p2_g_i = p2_g_i + cartisian_velocities[2] * iterationTime


        # ==> Atualização do Alpha_ideal:
        alpha_i = alpha_i + alpha_velocity * iterationTime
            
        UFactory_Lite.vc_set_joint_velocity(vel_config, is_radian=True) # ====> Atualização das Velocidades das Juntas
    
        # ==> Cálculo do Tempo de Delay():
        finalTime = time.monotonic() # Fim do Relógio
        
        pprint(finalTime-startTime)

        if (finalTime - startTime) < iterationTime:
            time.sleep(iterationTime-(finalTime-startTime))
    
        # ==> Atualização das Posições das Juntas:
        config_rads = aux_config


else:
    # Se o Método escolhido for 2

    #UFactory_Lite.set_mode(4) # modo de velocidades
    while alpha < N_voltas*2*pi:
        startTime = time.monotonic()

        #J0R_red_subs = eval(subs(J0R,[t1 t2 t3 t4 t5 t6],config_rads(1:6)));
        J0R_red_subs = J0R.subs([(t1,config_rads[0]), (t2,config_rads[1]), (t3,config_rads[2]), (t4,config_rads[3]), (t5,config_rads[4]), (t6,config_rads[5])])
        
        config_rads_array_plot_1.append(config_rads[0])
        config_rads_array_plot_2.append(config_rads[1])
        config_rads_array_plot_3.append(config_rads[2])
        config_rads_array_plot_4.append(config_rads[3])
        config_rads_array_plot_5.append(config_rads[4])
        config_rads_array_plot_6.append(config_rads[5])


        cartisian_velocities = Compute_cartesian_velocity(opcao,r,r_a,r_b,alpha,alpha_velocity)

        # -> Plano Z
        if (opcao == "1" or opcao == "3" or opcao == "5"): 
            cartesian_velocities_array_plot_1.append(cartisian_velocities[0])
            cartesian_velocities_array_plot_2.append(cartisian_velocities[1])

        # -> Plano X
        elif(opcao == "2" or opcao == "4" or opcao == "6"):
            cartesian_velocities_array_plot_1.append(cartisian_velocities[1])
            cartesian_velocities_array_plot_2.append(cartisian_velocities[2])

        # ==> Cálculo das Posições Reais:
        T_0G_aux = UFactory_Lite.get_forward_kinematics(config_rads, input_is_radian=True, return_is_radian=True)
        T_0G = T_0G_aux[1]

        p1_g_r, p2_g_r = Compute_Position(opcao, T_0G)
        p1_g_r_array_plot.append(p1_g_r)
        p2_g_r_array_plot.append(p2_g_r)
        
        J0R_red_subs = Matrix(J0R_red_subs)
        J0R_red_subs = np.array(J0R_red_subs.evalf(), dtype=float)

        vel = np.linalg.inv(J0R_red_subs) @ cartisian_velocities

        vel_config_array_plot_1.append(vel[0])
        vel_config_array_plot_2.append(vel[1])
        vel_config_array_plot_3.append(vel[2])
        vel_config_array_plot_4.append(vel[3])
        vel_config_array_plot_5.append(vel[4])
        vel_config_array_plot_6.append(vel[5])
        


        aux_config = config_rads + vel.T * iterationTime
        
        alpha = alpha + alpha_velocity * iterationTime



        finalTime = time.monotonic()
        #pprint(finalTime - startTime)

        if (finalTime - startTime) < iterationTime:
            time.sleep(iterationTime-(finalTime-startTime))

        UFactory_Lite.vc_set_joint_velocity(vel, is_radian=True)

        config_rads = aux_config
        


# ============ Fim dos Métodos ============
UFactory_Lite.vc_set_joint_velocity([0,0,0,0,0,0], is_radian=True)
time.sleep(3)
UFactory_Lite.set_mode(mode=0)
UFactory_Lite.set_state(state=0)
UFactory_Lite.move_gohome(wait=True)
UFactory_Lite.disconnect()

pprint(error_1_array_plot)

if metodo == "1":
    Final_Plot(opcao, C, r, r_a, r_b, error_1_array_plot, error_2_array_plot, p1_g_r_array_plot, p2_g_r_array_plot, config_rads_array_plot_1, config_rads_array_plot_2, config_rads_array_plot_3, config_rads_array_plot_4, config_rads_array_plot_5, config_rads_array_plot_6, vel_config_array_plot_1, vel_config_array_plot_2, vel_config_array_plot_3, vel_config_array_plot_4, vel_config_array_plot_5, vel_config_array_plot_6, cartesian_velocities_array_plot_1, cartesian_velocities_array_plot_2, iterationTime)

elif metodo == "2":
    Final_Plot_2(opcao, C, r, r_a, r_b, p1_g_r_array_plot, p2_g_r_array_plot, config_rads_array_plot_1, config_rads_array_plot_2, config_rads_array_plot_3, config_rads_array_plot_4, config_rads_array_plot_5, config_rads_array_plot_6, vel_config_array_plot_1, vel_config_array_plot_2, vel_config_array_plot_3, vel_config_array_plot_4, vel_config_array_plot_5, vel_config_array_plot_6, cartesian_velocities_array_plot_1, cartesian_velocities_array_plot_2, iterationTime)