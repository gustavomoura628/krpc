import krpc
conn = krpc.connect(name='Drone')
vessel = conn.space_center.active_vessel

import numpy as np


while True:
    print("Inertia:")
    I = np.array(vessel.inertia_tensor).reshape(3,3)
    print(I)
    print("Inverted I: ")
    print(np.linalg.inv(I))

    I = np.array(vessel.inertia_tensor).reshape(3,3)
    I_inv = np.linalg.inv(I)
    a_max = []
    alpha_max = []
    engines = []
    for engine in vessel.parts.engines:
        print()
        position = np.array([engine.part.center_of_mass(vessel.reference_frame)])
        direction = np.array([engine.part.direction(vessel.reference_frame)])
        f_i = direction * engine.max_thrust
        a_max_i = (vessel.mass ** -1) * f_i

        #line = conn.drawing.add_line(tuple(position.reshape(3)),tuple((position + a_max_i).reshape(3)),vessel.reference_frame)
        #line.color = [1,0,0]

        a_max.append(a_max_i[0])
        print(f"f_i = {f_i}")
        print(f"a_max_i = {a_max_i}")


        tau_i = np.cross(f_i,position) 


        print(f"tau_i = {tau_i}")
        alpha_max_i = np.matmul(tau_i, I_inv)
        #alpha_max_i = (vessel.mass ** -1) * tau_i

        #line = conn.drawing.add_line(tuple(position.reshape(3)),tuple((position + alpha_max_i).reshape(3)),vessel.reference_frame)
        #line.color = [0,0,1]

        alpha_max.append(alpha_max_i[0])
        print(f"alpha_max_i = {alpha_max_i}")
        engines.append(engine)

    a_max = np.array(a_max)
    alpha_max = np.array(alpha_max)

    print("A_MAX = ",a_max)
    print("ALPHA_MAX = ",alpha_max)
    #activation = [1,1,0,0]
    #print(f"{activation} * alpha_max = {np.matmul(activation, alpha_max)}")

    def optimize(x, a_T, alpha_T, a_max, alpha_max):
        DEBUG_PRINT = False
        if(DEBUG_PRINT): print()
        if(DEBUG_PRINT): print("OPTIMIZING!")
        if(DEBUG_PRINT): print(f"x = {x}")
        if(DEBUG_PRINT): print(f"a_max = {a_max}")
        a = np.matmul(x, a_max)
        if(DEBUG_PRINT): print(f"x*a_max = {a}")
        alpha = np.matmul(x, alpha_max)
        return np.linalg.norm(a - a_T) + 2*np.linalg.norm(alpha - alpha_T) + np.dot(x,x)

    import time
    from scipy.optimize import minimize
    from scipy.optimize import Bounds

    a_T = np.array([0,9.9,0])
    alpha_T = np.array([0,0,0])
    x_0 = [0.5] * len(engines)
    optimize(x_0, a_T, alpha_T, a_max, alpha_max)



    bounds = Bounds([0]*len(engines),[1]*len(engines))
    res = minimize(optimize, x_0, (a_T, alpha_T, a_max, alpha_max), method='nelder-mead',
                options={'xatol': 1e-8, 'disp': True}, bounds=bounds)
    #res = minimize(optimize, x_0, (a_T, alpha_T, a_max, alpha_max), method='SLSQP', options={'disp': True}, bounds=bounds)

    print(f"res = {res}")
    print(f"res.x = {res.x}")

    x = res.x
    a = np.matmul(x, a_max)
    alpha = np.matmul(x, alpha_max)
    print(f"a_max = {a_max}")
    print(f"alpha_max = {alpha_max}")
    print(f"a = {a}")
    print(f"alpha = {alpha}")

    for i, engine in enumerate(engines):
        engine.thrust_limit = res.x[i]

    #liney = conn.drawing.add_line([0,-0.5,0],[0,0.5,0],vessel.reference_frame)
    #linex = conn.drawing.add_line([-0.5,0,0],[0.5,0,0],vessel.reference_frame)
    #linez = conn.drawing.add_line([0,0,-0.5],[0,0,0.5],vessel.reference_frame)
    #liney.color = [0,1,0]
    #linex.color = [0,1,0]
    #linez.color = [0,1,0]