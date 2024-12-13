<<<<<<< HEAD
# Importar as principais bibliotecas:
import numpy as np
from numpy import  matrix, array
import sympy as sp


from sympy import simplify, cos, sin, Array, Matrix, eye
=======
import sympy as sp
from sympy import simplify, cos, sin, Array
>>>>>>> 076f78e1165617b0a8664f4042f465c12ac4932d

def MGH_DH(DH_Matrix):
    N_Links = DH_Matrix.shape[0]  # Número de Links
    print("Número de Links:", N_Links)

    # Inicializar a matriz de transformação acumulada como identidade simbólica
    Tf = sp.eye(4)

    # Lista para armazenar as transformações individuais
    T_aux = [None] * N_Links

    for i in range(N_Links):
        # Extrair os parâmetros DH da linha i
        theta, d, a, alpha = DH_Matrix[i, :]

        # Matriz de transformação individual
        T_aux[i] = sp.Matrix([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),            cos(alpha),            d],
            [0,           0,                     0,                     1]
        ])
<<<<<<< HEAD
        Tf = Tf * Matrix(T_aux[i])
        
    return T_aux,Tf
=======

        # Atualizar a matriz acumulada
        Tf = Tf * T_aux[i]

>>>>>>> 076f78e1165617b0a8664f4042f465c12ac4932d

    return T_aux, Tf