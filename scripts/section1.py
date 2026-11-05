#!/usr/bin/env python3
# add import and helper functions her
import numpy as np

if __name__ == "__main__":
    np.random.seed(42)
    x = np.random.normal(size=(4, 10))
    a = x[:,None,:]
    b = x[None,:,:]
    c = np.square(a - b)
    ans = np.sum(c, axis=2)    
    print(ans)

	
