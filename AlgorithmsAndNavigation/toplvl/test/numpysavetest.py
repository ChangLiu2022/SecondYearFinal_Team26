import numpy as np

a = np.array([[0,0],[1,1],[2,2],[3,3]])
np.save('toplvl/test/points_warped.npy', a)
b = np.load('toplvl/test/points_warped.npy')
print(b)