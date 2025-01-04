function [T_aux,Tf] = MGH_DH(DH_Matrix)
N_Links=size(DH_Matrix,1); % Número de Links
Tf=eye(4);
T_aux = sym(zeros(4,4,N_Links));

for i=1:N_Links
    T_aux(:,:,i) =      [cos(DH_Matrix(i,1)) -sin(DH_Matrix(i,1))*cos(DH_Matrix(i,4)) sin(DH_Matrix(i,1))*sin(DH_Matrix(i,4))  DH_Matrix(i,3)*cos(DH_Matrix(i,1));
                          sin(DH_Matrix(i,1)) cos(DH_Matrix(i,1))*cos(DH_Matrix(i,4)) -cos(DH_Matrix(i,1))*sin(DH_Matrix(i,4)) DH_Matrix(i,3)*sin(DH_Matrix(i,1));
                          0                   sin(DH_Matrix(i,4))                      cos(DH_Matrix(i,4))                     DH_Matrix(i,2);
                          0                   0                                         0                                      1                                    ];
    Tf=Tf*T_aux(:,:,i);
end