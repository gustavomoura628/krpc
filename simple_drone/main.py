import krpc
conn = krpc.connect(name='Drone')
vessel = conn.space_center.active_vessel

import time
import math
import numpy as np
def clamp(x, min, max):
    if x > max:
        return max
    elif x < min:
        return min
    else:
        return x

class TorquePID:
    kp = 0.1
    ki = 0.01
    kd = -0.1
    i = 0
    last_error = 0
    decay = 0.9
    engine = None
    vessel = None
    last_time = None

    def __init__(self, engine, vessel):
        self.vessel = vessel
        self.engine = engine
        relative_position = np.array(self.engine.part.position(self.vessel.surface_reference_frame))
        print(f"Added engine {relative_position}")

        self.last_time = conn.space_center.ut

    def position(self):
        relative_position = np.array(self.engine.part.position(self.vessel.surface_reference_frame))
        print(f"self.engine position = {relative_position}")

    def update(self):
        DEBUG_PRINT = True
        if(DEBUG_PRINT): print()
        relative_position = np.array(self.engine.part.position(self.vessel.surface_reference_frame))
        if(DEBUG_PRINT): print(f"Relative Position = {relative_position}")
        if(DEBUG_PRINT): print(f"Direction = {self.vessel.direction(self.vessel.surface_reference_frame)}")
        torque = np.cross(relative_position,self.engine.part.direction(self.vessel.surface_reference_frame))
        if(DEBUG_PRINT): print(f"Torque = {torque}")
        corrective_torque_direction = np.cross(self.vessel.direction(self.vessel.surface_reference_frame),[1,0,0])

        error = np.dot(torque, corrective_torque_direction)
        if(DEBUG_PRINT): print(f"Error = {error}")
        p = error
        self.i = self.i*self.decay + error
        current_time = conn.space_center.ut
        dt = current_time - self.last_time
        if(DEBUG_PRINT): print(f"time = {current_time}")
        if(DEBUG_PRINT): print(f"dt = {dt}")
        self.last_time = current_time
        #d = (self.last_error - error) / dt
        d = np.dot(self.vessel.angular_velocity(self.vessel.surface_reference_frame), torque)
        self.last_error = error

        if(DEBUG_PRINT): print(f"initial pid= P: {p}, I: {self.i}, D: {d}")
        if(DEBUG_PRINT): print(f"constants pid= P: {self.kp*p}, I: {self.ki * self.i}, D: {self.kd * d}")
        magnitude = self.kp*p + self.ki*self.i + self.kd*d
        print(f"Magnitude = {magnitude}")
        up_magnitude = 0.5 * np.dot(self.engine.part.direction(self.vessel.surface_reference_frame), [1,0,0])
        print(f"Up Magnitude = {up_magnitude}")
        up_magnitude = clamp(up_magnitude, 0, 1)
        self.engine.thrust_limit = clamp(magnitude+up_magnitude, 0, 1)


pid_engines = []
# Disable engines
for engine in vessel.parts.engines:
    engine.thrust_limit = 0
    pid_engines.append(TorquePID(engine,vessel))
vessel.control.throttle = 1

for engine in pid_engines:
    engine.position()
    
while True:
    for pid in pid_engines:
        pid.update()

#while True:
#    for engine in vessel.parts.engines:
#        relative_position = np.array(engine.part.position(vessel.surface_reference_frame))
#        print(f"Relative Position = {relative_position}")
#        print(f"Direction = {vessel.direction(vessel.surface_reference_frame)}")
#        torque = np.cross(relative_position,vessel.direction(vessel.surface_reference_frame))
#        print(f"Torque = {torque}")
#        corrective_torque_direction = np.cross(vessel.direction(vessel.surface_reference_frame),[1,0,0])
#
#        magnitude = np.dot(torque, corrective_torque_direction)
#        print(f"Magnitude = {magnitude}")
#        engine.thrust_limit = clamp(magnitude+.5, 0, 1)
#        print()