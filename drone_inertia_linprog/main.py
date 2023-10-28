import krpc
conn = krpc.connect(name='Drone')
vessel = conn.space_center.active_vessel

import numpy as np
import time
def clamp(x,min,max):
    if(x < min): return min
    if(x > max): return max
    return x

i = 0
while True:
    I = np.array(vessel.inertia_tensor).reshape(3,3)
    I_inv = np.linalg.inv(I)

    a_max = np.empty((3,0))
    alpha_max = np.empty((3,0))
    engines = []
    cost = np.empty((0))

    for engine in vessel.parts.engines:
        position = np.array([engine.part.center_of_mass(vessel.reference_frame)])
        direction = np.array([engine.part.direction(vessel.reference_frame)])

        f_i = direction * engine.max_thrust
        a_max_i = (vessel.mass ** -1) * f_i
        a_max = np.concatenate((a_max,a_max_i.reshape(3,1)),axis=1)

        tau_i = np.cross(f_i,position) 
        alpha_max_i = np.matmul(tau_i, I_inv)
        alpha_max = np.concatenate((alpha_max,alpha_max_i.reshape(3,1)),axis=1)

        cost = np.concatenate((cost,[engine.specific_impulse]))
        engines.append(engine)

    A = np.concatenate((a_max,alpha_max),axis=0)

    altitude = vessel.flight().surface_altitude
    target_altitude = 15
    error = target_altitude - altitude
    print("altitude = ",altitude)

    orbit_velocity = vessel.flight(vessel.orbit.body.reference_frame).velocity
    print("body = ",vessel.orbit.body.name)
    print("orbit velocity = ",orbit_velocity)
    velocity = conn.space_center.transform_direction(orbit_velocity,vessel.orbit.body.reference_frame,vessel.surface_reference_frame)
    print("velocity = ",velocity)
    vertical_speed = velocity[0]

    print("vertical speed = ",vertical_speed)
    print("error = ",error)
    #pid_acc = clamp(error,-0.5,0.5) - clamp(vertical_speed,-1,1)
    #pid_acc = error - vertical_speed
    pid_acc = clamp(error,-1,1) - clamp(vertical_speed,-1,1)

    velocity_cancelation = np.array([0, -velocity[1], -velocity[2]])

    a_T_srf = np.array([9.81,0,0]) + velocity_cancelation + np.array([pid_acc,0,0])
    a_T = np.array(conn.space_center.transform_position(tuple(a_T_srf),vessel.surface_reference_frame, vessel.reference_frame ))
    
    alpha_T = np.array([0,0,0])
    b = np.concatenate((a_T,alpha_T))

    from scipy.optimize import linprog

    # Define the bounds for x (0 to 1) as a list of tuples.
    x_bounds = [(0, 1) for _ in range(len(cost))]


    # Solve the linear program using the 'linprog' function.
    start_time = time.time()
    #res = linprog(cost, A_eq=A, b_eq=b, bounds=x_bounds, method='simplex')
    res = linprog(cost, A_eq=A, b_eq=b, bounds=x_bounds)
    linprog_time = time.time() - start_time
    print("Solve time = ",linprog_time)

    # Check if a solution was found.
    if res.success:
        x_solution = res.x
        min_cost = res.fun
        #print("Solution found:")
        #print("x:", x_solution)
        #print("Minimized cost:", min_cost)
    else:
        print("No feasible solution found.")
        continue

    #from scipy.optimize import minimize
    #from scipy.optimize import Bounds

    #def optimize_linprog_0(x, b, A):
    #    DEBUG_PRINT = False
    #    a = np.matmul(A, x)
    #    return np.linalg.norm(a - b) + np.dot(x,x)

    #x_0 = [0.5] * len(engines)
    #bounds = Bounds([0]*len(engines),[1]*len(engines))
    #time_start = time.time()
    #res = minimize(optimize_linprog_0, x_0, (b, A), options={'disp': False}, bounds=bounds)
    #optimize_time = time.time() - time_start


    x = res.x
    a = np.matmul(a_max, x)
    alpha = np.matmul(alpha_max, x)
    print("a:\n",a)
    print("alpha:\n",alpha)

    for i, engine in enumerate(engines):
        engine.thrust_limit = res.x[i]