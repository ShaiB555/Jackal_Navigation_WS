import numpy as np

#CALCULATES ANGLE ALPHA MINUS ANGLE BETA TAKING SMALLEST POSSIBLE ANGLE
def angle_diff(alpha,beta):
    z1=np.exp(complex(0,alpha))
    z2=np.exp(complex(0,beta))
    z2_bar=np.conjugate(z2)
    dz=z2_bar*z1
    d_theta= np.angle(dz)
    return d_theta