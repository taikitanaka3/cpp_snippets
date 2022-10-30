# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

def main():

    X = [1.0,2.0,3.0,4.0,5.0]
    Y = [1.1, 2.1, 2.8, 4.3, 5.1]

    A = np.array([X,np.ones(len(X))])
    A = A.T
    a,b = np.linalg.lstsq(A,Y)[0]

    plt.plot(X,Y,"ro")
    plt.plot(X,(np.multiply(a,X)+b),"g--")
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()