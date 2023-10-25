import krpc
conn = krpc.connect(name='Hello World')
vessel = conn.space_center.active_vessel

import time
import math
import numpy as np

print(vessel.name)
print(vessel.biome)
print(vessel.thrust)
print(vessel.surface_reference_frame)
print(vessel.position(vessel.surface_reference_frame))

# Disable engines
for engine in vessel.parts.engines:
    engine.thrust_limit = 0

vessel.control.throttle = 1

front_engine = None
rear_engine = None
left_engine = None
right_engine = None

# Iterate through the vessel's engines
for engine in vessel.parts.engines:
    # Get the position of the engine part relative to the vessel's center of mass
    relative_position = engine.part.position(vessel.reference_frame)
    print(f"Engine on part {engine.part.name}: Relative Position = {relative_position}")
    if(relative_position[2] > 0.01):
        front_engine = engine
    elif(relative_position[2] < -0.01):
        rear_engine = engine
    elif(relative_position[0] > 0.01):
         right_engine = engine
    elif(relative_position[0] < -0.01):
         left_engine = engine

class ControlDrone:
    vessel
    front_engine
    rear_engine
    left_engine
    right_engine

    def __init__(self, vessel, front_engine, rear_engine, left_engine, right_engine):
        self.vessel = vessel
        self.front_engine = front_engine
        self.rear_engine = rear_engine
        self.left_engine = left_engine
        self.right_engine = right_engine

    def get_altitude(self):
        return vessel.flight().surface_altitude
    
    def get_yaw(self):
        yaw = math.atan2(vessel.flight().direction[1], vessel.flight().direction[2])
        return yaw

    def get_roll(self):
        roll = math.atan2(vessel.flight().direction[1], vessel.flight().direction[0])
        return roll


test_control = ControlDrone(vessel, front_engine, rear_engine, left_engine, right_engine)

while True:
    vessel_height = vessel.flight().surface_altitude
    print(f"Vessel height: {vessel_height} meters")
    print(f"Yaw = {test_control.get_yaw()}")
    print(f"Roll = {test_control.get_roll()}")
    print(f"dir normal = {vessel.flight(vessel.surface_reference_frame).direction}")
    print(f"dir vessel = {vessel.flight(vessel.reference_frame).direction}")
    time.sleep(1)