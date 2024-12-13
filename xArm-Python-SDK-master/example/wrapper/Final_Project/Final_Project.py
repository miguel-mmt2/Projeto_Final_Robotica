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
from numpy import eye, matrix
from math import sqrt, cos, sin, pi




from MGH_DH import MGH_DH


sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI # Importar a API do Robô



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

C = [100, 100, 100] # Inicialização do Centro
r = 20  # Incialização do raio

Lg = 61.5


# ============ Setup do Robô ============

#UFactory_Lite = XArmAPI(ip)
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
T_final = sp.simplify(T_final)

print(Transformation_Matrices)
    

# Se o Método escolhido for 1


# Se o Método escolhido for 2


# ============ Fim dos Métodos ============

#UFactory_Lite.move_gohome(wait=True)
#UFactory_Lite.disconnect()





