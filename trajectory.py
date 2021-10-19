from math import sin, cos, radians
from matplotlib import pyplot as plt

"""
# Position is calculated in each axis via:
x = x_0 + v_0 * t + .5 * a_0 * t^2

# Wind force is calculated using
F = A * P * Cd

Where:
    A is Surface Area
    P is pressure
    Cd is coeffiecent of drag

# Pressure
P = .00256 * V * V

Where:
    V is velocity in mph
    P is pressure in Pounds per Square Foot (psf)

# Area is computed as a conserative cross-section of the largest part of the
# X1 drone: 0.015 m^2

# P is converted from Pounds per Square Foot to Newtons per Square Meter using
# the conversion factor of 47.8803.
# (source: https://www.google.com/search?q=Pounds+per+Square+Foot+to+Newtons+per+Square+Meter)

# Acceleration due to wind is applied in the worst case scenario, where the force
# is perfectly applied in the direction of the initial velocity of the drone.
# There is no wind resistance applied in the opposite direction (very conservative)

# Cd of 1.05 was assumed as a very conservative value based on the coefficient
# of drag of a cube. The surface area is also very conservative (a flat plane),
# considering the actual X1 drone is much more aerodynamic (closer to a Cd of
# .6).
# Source: https://en.wikipedia.org/wiki/Drag_coefficient
"""

class Projectile:
    def __init__(self, x0, y0, v, angle, mass, wind_force):
        """
        x0 and y0 are initial coordinates of the projectile
        v is the initial velocity
        angle is the angle of shooting in degrees
        """
        # current x and y coordinates of the projectile
        self.x    = x0
        self.y    = y0

        # current value of velocity components
        self.vx  = v*cos(radians(angle))
        self.vy  = v*sin(radians(angle))

        # acceleration by x and y axes
        self.ax   = wind_force / mass
        self.ay   = -9.8

        # start time
        self.time = 0

        self.conversion = 1.0

        # these list will contain discrete set of projectile coordinates
        self.xarr = [self.x * self.conversion]
        self.yarr = [self.y * self.conversion]

    def updateVx(self, dt):
        self.vx = self.vx + self.ax*dt
        return self.vx

    def updateVy(self, dt):
        self.vy = self.vy + self.ay*dt
        return self.vy

    def updateX(self, dt):
        self.x = self.x + 0.5*(self.vx + self.updateVx(dt))*dt
        return self.x

    def updateY(self, dt):
        self.y = self.y + 0.5*(self.vy + self.updateVy(dt))*dt
        return self.y

    def step(self, dt):
        self.xarr.append(self.updateX(dt) * self.conversion)
        self.yarr.append(self.updateY(dt) * self.conversion)
        self.time = self.time + dt

def startSimulation(x0, y0, velocity, angle, mass = 0.0, wind_force = 0.0):
    """
    Returns a tuple with sequential pairs of x and y coordinates
    """
    projectile = Projectile(x0, y0, velocity, angle, mass, wind_force)
    dt = 0.05 # time step
    t = 0 # initial time
    projectile.step(dt)

    ###### THE  INTEGRATION ######
    while projectile.y >= 0:
        projectile.step(dt)
        t = t + dt
    ##############################

    return (projectile.xarr, projectile.yarr)

class WindForceModel():
    # Conversion from Imperial to Metric
    psqft_to_nsqm = 47.8803
    mps_to_mph = 2.23694

    def __init__(self, cd, h, w, mass):
        self.area = h * w
        self.mass = mass
        self.cd = cd

    def getWindForce(self, velocity) -> float:
        velocity_mph = WindForceModel.mps_to_mph * velocity

        # Force in Newtons due to wind (constant acceleration)
        P = .00256 * velocity_mph * velocity_mph * WindForceModel.psqft_to_nsqm
        return self.area * P * self.cd

def main():
    # Conversion from Imperial to Metric
    ft_to_m = 0.3048

    # Initial Conditions
    x0 = 0
    y0 = 400 * ft_to_m
    velocity = 7

    # Physical Model (h x w x d) in m
    h = .10
    w = .15
    mass = 1.25 # kg

    # Coefficient of drag
    cd = 1.05 # for a cube (https://en.wikipedia.org/wiki/Drag_coefficient)

    # Wind Model
    wind = WindForceModel(cd, h, w, mass)

    x = []
    y = []

    knots_to_ms = 0.514444
    windspeeds = [
        0.0 * knots_to_ms,
        5.0 * knots_to_ms,
        10.0 * knots_to_ms,
        20.0 * knots_to_ms,
    ]
    heights = [
        400.0 * ft_to_m,
        200.0 * ft_to_m,
    ]
    velocities = [
        14,
        7,
        3
    ]

    for height in heights:
        for windspeed in windspeeds:
            F = wind.getWindForce(windspeed)
            X, Y = startSimulation(x0, height, velocity, 0, mass, F)
            x.append(X)
            y.append(Y)

    plt.plot(
        x[0], y[0], 'g-', x[1], y[1], 'b-', x[2], y[2], 'y-', x[3], y[3], 'r-',
        x[4], y[4], 'g-', x[5], y[5], 'b-', x[6], y[6], 'y-', x[7], y[7], 'r-',
        [0, max(x[3])], [0, 0], 'k-' # ground
        )
    plt.legend([
        '0 knots (400ft)', '5 knots (400ft)', '10 knots (400ft)', '20 knots (400ft)',
        '0 knots (200ft)', '5 knots (200ft)', '10 knots (200ft)', '20 knots (200ft)'
    ])
    plt.xlabel('X coordinate (m)')
    plt.ylabel('Y coordinate (m)')
    plt.title(f'Distance Traveled (m) After Motor Shutdown at {round(velocity, 2)} m/s')
    plt.show()

    x = []
    y = []
    for height in heights:
        for velocity in velocities:
            X, Y = startSimulation(x0, height, velocity, 0, mass, 0)
            x.append(X)
            y.append(Y)

    plt.plot(
        x[0], y[0], 'r-', x[1], y[1], 'b-', x[2], y[2], 'g-',
        x[3], y[3], 'r--', x[4], y[4], 'b--', x[5], y[5], 'g--',
        [0, max(x[0])], [0, 0], 'k-' # ground
        )
    plt.legend([
        '14 m/s (400ft)', '7 m/s (400ft)', '3 m/s (400ft)',
        '14 m/s (200ft)', '7 m/s (200ft)', '3 m/s (200ft)'
    ])
    plt.xlabel('X coordinate (m)')
    plt.ylabel('Y coordinate (m)')
    plt.title(f'Distance Traveled (m) After Motor Shutdown')
    plt.show()

def tables():
    knots_to_ms = 0.514444
    ft_to_m = 0.3048
    ms_to_knots = 1.0 / knots_to_ms
    m_to_ft = 1.0 / ft_to_m

    v0 = 7 # m/s
    a0 = 0 # degrees

    # Wind Speeds (Knots)
    windspeeds = [
        1 * knots_to_ms,
        5 * knots_to_ms,
        10 * knots_to_ms,
        # 20 * knots_to_ms,
    ]

    # Initial Heights (Ft)
    heights = [
        400 * ft_to_m,
        300 * ft_to_m,
        200 * ft_to_m,
        100 * ft_to_m,
    ]

    speeds = [
        14,
        7,
        3
    ]

    # Physical Model (h x w x d) in m
    h = .10
    w = .15
    mass = 1.25 # kg
    cd = 1.05 # for a cube (https://en.wikipedia.org/wiki/Drag_coefficient)

    # Wind Model
    wind = WindForceModel(cd, h, w, mass)

    # Height Table (5 knots)
    print("Freefall Initial Height (ft), Initial Velocity (m/s), Windspeed (knots), Distance Traveled (ft), Safe Distance (+50 ft)")
    for speed in speeds:
        for height in heights:
            w0 = 5.0 * knots_to_ms
            v0 = speed
            F = wind.getWindForce(w0)
            x, y = startSimulation(0, height, v0, a0, mass, F)
            print(f"{round(height * m_to_ft, 2)}, {v0}, {round(w0 * ms_to_knots, 2)}, {round(x[-1] * m_to_ft, 2)}, {round(x[-1] * m_to_ft + 50.0, 2)}")

    print()

    # Max Travel Distances (400 ft)
    print("Freefall Initial Height (ft), Initial Velocity (m/s), Windspeed (knots), Distance Traveled (ft), Safe Distance (+50 ft)")
    for speed in speeds:
        for height in heights:
            for windspeed in windspeeds:
                y0 = height
                v0 = speed
                F = wind.getWindForce(windspeed)
                x, y = startSimulation(0, y0, v0, a0, mass, F)
                print(f"{round(y0 * m_to_ft, 2)}, {v0}, {round(windspeed * ms_to_knots, 2)}, {round(x[-1] * m_to_ft, 2)}, {round(x[-1] * m_to_ft + 50.0, 2)}")

if __name__ == '__main__':
    tables()
    main()


