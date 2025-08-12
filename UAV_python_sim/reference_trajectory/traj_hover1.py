import numpy as np

XRef=np.zeros((4000,12))
XRef[:,2]=1
XRef[:,8]=0.03
print(XRef)

print(XRef.shape)
print(type(XRef))