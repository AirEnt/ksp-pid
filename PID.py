import krpc
import time
import math
import scipy as sp

# Flight plan below. (lat, long, altitude, speed) The alitude and speed is the approach speed/alt of each waypoint.
Flight_Plan = [(-5.63, -83.80, 3000, 200),
               (-2.15, -106.13, 5000, 300),
               (2.46, -160.22, 5000, 300),
               (18.55, -164.88, 5000, 300),
               (15.82, -184.31, 5000, 300),
               (4.83, -181.14, 5000, 300),
               (31.90, -174.55, 5000, 300),
               (34.45, -166.03, 5000, 300),
               (40.87, -146.69, 5000, 300),
               (12.57, -77.87, 5000, 300),
               (-0.048, -74.72, 3000, 250)
                ]
# it flies west to the ocean wehere is then crosses over to get to the desert, then flies to places that look interesting.
# It returns to KSC. 
(lat, long, alt, speed) = Flight_Plan[1] # The starting way point
distance = 50000
way_point = 1

class Local:
    def __init__(self, lat , long, speed, alt):
        self.latitude = lat
        self.longitude = long
        self.speed = speed
        self.altitude = alt


class PID:

    def __init__(self, proportional, intergral, derivative, omax, omin):
        self.Kp = proportional
        self.Ki = intergral
        self.Kd = derivative
        self.output_max = omax
        self.output_min = omin
        self.P = 0
        self.I = 0
        self.D = 0
        self.error = [0, 0, 0, 0, 0]
        self.delta_time = 0
        self.previous_time = 0
        self.Output = 0

    def step(self, control_variable, set_point, current_time):

        self.delta_time = current_time - self.previous_time

        for value in range(len(self.error) - 1, 0, -1):
            self.error[value] = self.error[value - 1]
        self.error[0] = set_point - control_variable

        # Proportional
        self.P = self.error[0]

        # Derivative
        self.D = self.Kd * (self.error[0]-self.error[1])

        # Integral
        self.I += self.error[0] * self.Ki

        if self.I > 1:
            self.I = 1
        if self.I < -1:
            self.I = -1

        # Combine Terms
        self.Output = (self.Kp * self.P) + self.I + self.D

        if self.Output > self.output_max:
            self.Output = self.output_max
        if self.Output < self.output_min:
            self.Output = self.output_min

        self.previous_time = current_time
        return self.Output


class PIDV:

    def __init__(self, proportional, intergral, derivative, omax, omin):
        self.Kp = proportional
        self.Ki = intergral
        self.Kd = derivative
        self.output_max = omax
        self.output_min = omin
        self.P = 0
        self.I = 0
        self.D = 0
        self.error = [0, 0, 0, 0, 0]
        self.delta_time = 0
        self.previous_time = 0
        self.Output = 0

    def step(self, control_variable, set_point, current_time):

        self.delta_time = current_time - self.previous_time

        for value in range(len(self.error) - 1, 0, -1):
            self.error[value] = self.error[value - 1]
        self.error[0] = set_point - control_variable

        # Proportional
        self.P = self.error[0]

        # Derivative
        self.D = self.error[0] - self.error[1]

        # Integral
        self.I += self.error[0] * self.delta_time

        if self.I > 10:
            self.I = 10
        if self.I < -10:
            self.I = -10

        # Combine Terms
        self.Output += self.Kp * self.P  + self.Ki * self.I + self.Kd * self.D
        if self.Output > self.output_max:
            self.Output = self.output_max
        if self.Output < self.output_min:
            self.Output = self.output_min

        self.previous_time = current_time
        return self.Output


class MAC:

    def __init__(self, pos, neg):
        self.worker = 0
        self.verticalSpeed = 0
        self.desired_VerticalSpeed = 0
        self.delta_correct = 0
        self.delta_time = 0
        self.positiveVS = pos #s etup
        self.negativeVS = neg # setup
        self.CV = [0, 0, 0]
        self.output = 0
        self.deltaU = 0
        self.preTime = 0

    def step(self, control_variable, set_point, time, correct):
        self.CV.append(control_variable)
        del self.CV[0]

        self.delta_time = time - self.preTime
        self.preTime = time

        self.worker = self.delta_correct
        self.verticalSpeed = (self.CV[-1] - self.CV[-2])/0.04

        self.delta_correct = correct - self.worker

        self.desired_VerticalSpeed= 0.1*(set_point - control_variable)
        if self.desired_VerticalSpeed > self.positiveVS:
            self.desired_VerticalSpeed = self.positiveVS
        if self.desired_VerticalSpeed < self.negativeVS:
            self.desired_VerticalSpeed = self.negativeVS

        self.output += 0.03 * (self.desired_VerticalSpeed - self.verticalSpeed)  - 0.001 * self.delta_correct

        if self.output > 20:
            self.output = 20
        if self.output < -20:
            self.output = -20
        return self.output


magnatude = PID(3, 0.1, 0, 30, -30)     # PID for heading vector
pitchcommand = MAC(50, -50)

def control_loop(roll, pitch, speed):
    vessel.control.throttle = throttleControl.step(vessel.flight().true_air_speed, speed, time.time())
    vessel.control.roll = rollControl.step(vessel.flight().roll, roll, time.time())
    vessel.control.pitch = pitchControl.step(vessel.flight().pitch, pitch, time.time())
    vessel.control.yaw = 0.1 * vessel.control.roll


def heading_vector(heading):
    angel = heading - vessel.flight().heading

    if angel > 180:
        angel -= 360
    if angel < -180:
        angel += 360

    return (magnatude.step(vessel.flight().roll, angel, time.time()))


def get_heading(lat_1, long_1, lat_2, long_2):
    degToRad = (math.pi)/180
    lat_1 *= degToRad
    lat_2 *= degToRad
    long_1 *= degToRad
    long_2 *= degToRad

    d_long = long_2 - long_1
    d_lat = lat_2 - lat_1
    radius = 600000
    bering = math.atan2( math.sin(d_long) * math.cos(lat_2), math.cos(lat_1) * math.sin(lat_2) - math.sin(lat_1) * math.cos(lat_2) * math.cos(d_long))
    c = math.acos(math.sin(lat_1) * math.sin(lat_2) + math.cos(lat_1) * math.cos(lat_2) * math.cos(d_long)) * radius
    bering /= degToRad
    if bering < 0:
        bering += 360
    if bering > 360:
        bering -= 360
    return (bering, c)


conn = krpc.connect()                       # CONNECT TO KSP SERVER
vessel = conn.space_center.active_vessel

throttleControl = PIDV(0.001, 0, 0.4, 1, 0)
rollControl = PID(0.015, 0.00005, 0.4, 1, -1)
pitchControl = PID(0.05, 0.001, 0.04, 1, -1)
yawControl = PID(0, 0, 0.2, 1, -1)

print("Latitude = {0}; Longitude = {1}; Altitude = {2}; Speed = {3}; Number = {4}".format(lat, long, alt, speed, way_point))
while True:

    if distance < 15000:
        way_point += 1
        (lat, long, alt, speed) = Flight_Plan[way_point]
        print("Latitude = {0}; Longitude = {1}; Altitude = {2}; Speed = {3}; Number = {4}".format(lat, long, alt, speed, way_point))

    (head, distance) = get_heading(vessel.flight().latitude, vessel.flight().longitude, lat, long)

    turn = heading_vector(head)
    pitch = pitchcommand.step(vessel.flight().mean_altitude, alt, time.time(), vessel.flight().pitch)

    control_loop(turn, pitch, speed)
    time.sleep(0.04)