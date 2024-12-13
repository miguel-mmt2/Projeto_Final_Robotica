# Importar as principais bibliotecas:
import numpy as np
from numpy import  eye, matrix, array
import sympy as sp


from sympy import simplify, cos, sin, Array

def MGH_DH(DH_Matrix):
    N_Links = DH_Matrix.shape[0]  # Número de Links
    Tf = eye(4)

    T_aux = np.zeros()

    for i in range(N_Links):
        theta, d, a, alpha = DH_Matrix[i, :]
        T_aux[i] = Array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),            cos(alpha),            d],
            [0,           0,                     0,                     1]
        ])
        Tf = Tf * T_aux[i]

    return T_aux, simplify(Tf)

