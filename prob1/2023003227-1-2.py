import numpy as np

# 2-A
M = np.arange(2, 27)
print(M)
print()

# 2-B
M = M.reshape(5, 5)
print(M)
print()

# 2-C
M[1:4, 1:4].fill(0)
print(M)
print()

# 2-D
M = M @ M
print(M)
print()

# 2-E
print(np.sqrt((M[0] ** 2).sum()))
print()