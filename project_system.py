import math
import numpy as np
import numpy.linalg as npl
import control
import scipy.signal as scis
import scipy.integrate as scii
import matplotlib.pyplot as plt


def sys_state(mass,damp,l=0.5):
    #foundimetal elements
    g = 9.8
    I1 = 1/12*mass[1]*l**2
    I2 = 1/12*mass[3]*l**2

    DD = np.array([[mass[0]+mass[1]+mass[2]+mass[3],mass[1]*0.5*l+mass[2]*l+mass[3]*l,mass[3]*0.5*l],[mass[1]*0.5*l+mass[2]*l+4*l, mass[1]*(0.5*l)**2+I1+mass[2]*l**2+mass[3]*l**2 , mass[3]*0.5*l],[mass[3]*0.5*l,-1*mass[3]*0.5*l*l,mass[3]*(0.5*l)**2+I2]])
    DC = np.array([[damp[0],0,0],[0,damp[1],0],[0,0,damp[2]]])
    DG = np.array([[0,0,0],[0,-1*(mass[1]*0.5*l+mass[2]*l+mass[3]*l)*g,0],[0,0,-1*mass[3]*g*0.5*l]])
    DH = np.array([[1],[0],[0]])
    #system
    upA = np.column_stack([np.zeros((3,3)),np.eye(3)])
    downA = np.column_stack([-1*np.matmul(npl.inv(DD),DG),-1*np.matmul(npl.inv(DD),DC)])
    A = np.row_stack([upA,downA])
    B = np.row_stack([np.zeros((3,1)),-1*np.matmul(npl.inv(DD),DH)])
    C = np.array([[1,1,1,0,0,0]])
    D = [[0]]
    return A,B,C,D


def lqr_sys(A,B,C,D,in_q,r=1,num_q=6):
    #parameter
    sys = control.StateSpace(A,B,C,D)
    Q = np.zeros((num_q,num_q))
    R = [r]
    for i in range(num_q):
        Q[i,i]=in_q[i]

    K,solu_R, Eig_sys = control.lqr(sys,Q,r)
    kr = -1/np.matmul(np.matmul(C,npl.inv((A-np.matmul(B,K)))),B)[0]

    pole,eig_v =  npl.eig(A - B * K)
    pole_real = [a.real for a in pole]
    minpol = -3 + min(pole_real)
    repol=[]

    for i in range(num_q):
        repol.append(minpol+i+1)
    repol = np.array(repol)
    L = scis.place_poles(A.transpose(),C.transpose(),repol).gain_matrix
    L = L.transpose()

    upCA = np.column_stack(((A-np.matmul(B,K)),(np.matmul(B,K))))
    downCA = np.column_stack((np.zeros((num_q,num_q)),(A-np.matmul(L,C))))
    CA = np.row_stack((upCA,downCA))
    CB = np.row_stack((B*kr,np.zeros((num_q,1))))
    CC = np.column_stack((C, np.zeros((1,num_q))))
    CD = np.array([0])

    return CA,CB,CC,CD


def sys_simulation(A,B,t_inteval,num_t,ini_position,ref=[],control=False):

    t0, t1 = t_inteval[0], t_inteval[1]
    t = np.linspace(t0, t1, num_t)
    x0 = ini_position
    sol_x = np.zeros((len(t), len(x0)))  # array for solution
    sol_x[0,:] = x0

    if control == True and ref!=[]:

        def sysC_fun(inx,t, A, B,ref):
            x = np.array([inx]).transpose()
            X = np.matmul(A, x) + B*ref
            #out = np.array([X[0],X[1],X[2],X[3],X[4],X[5],X[6],X[7],X[8],X[9],X[10],X[11]]).reshape(12)
            out = X.transpose().reshape(len(X))
            return out
        sol = scii.odeint(sysC_fun, x0, t, args=(A, B,ref))

    else:
        def sys_fun(inx, t, A, B):
            u = 1
            x = np.array([inx]).transpose()
            X = np.matmul(A, x) + B * u
            #out = np.array([X[0],X[1],X[2],X[3],X[4],X[5]]).reshape(6)
            out = X.transpose().reshape(len(X))
            return out
        sol = scii.odeint(sys_fun,x0, t,args=(A,B))

    '''''    
    for i in range(1, t.size):

        x[i, :] = r.integrate(t[i])
        if not r.successful():
            raise RuntimeError("Could not integrate")
    '''''
    return sol,t


if __name__=="__main__":
    print('hi')
    m1 = 10
    m2 = 0.5
    m3 = 0.25
    m4 = 0.5
    m = [m1,m2,m3,m4]
    b1 = 2
    b2 = 0.04
    b3 = 0.03
    b = [b1,b2,b3]
    l = 0.25
    A,B,C,D = sys_state(m,b,l)

    x0 = [0,0,0,0,0,0]
    t_in = [0,3]
    num_t = 10000
    x,t = sys_simulation(A,B,t_in,num_t,x0)
    #print(x)
    '''''
    plt.plot(t, x[:,0])
    plt.plot(t, x[:, 1])
    plt.plot(t, x[:, 2])
    plt.plot(t, x[:, 3])
    plt.plot(t, x[:, 4])
    plt.plot(t, x[:, 5])
    plt.show()
    '''''
    print('hi2')
    q1 = 1000
    q2 = 2000
    q3 = 4000
    q4 = 0
    q5 = 0
    q6 = 0
    r1 = 1
    in_q = [q1,q2,q3,q4,q5,q6]
    CA,CB,CC,CD = lqr_sys(A,B,C,D,in_q,r1)
    ref = 15
    e0 = [0.5,0.1,0.1,-0.5,-0.1,-0.1]
    cx0 = x0+e0
    t_in = [0,5]
    x, t = sys_simulation(CA, CB, t_in, num_t, cx0,ref,control=True)

    plt.plot(t, x[:,0])
    plt.plot(t, x[:, 1])
    plt.plot(t, x[:, 2])
    plt.plot(t, x[:, 3])
    plt.plot(t, x[:, 4])
    plt.plot(t, x[:, 5])
    plt.show()



