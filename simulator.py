import time
import math
import signal
import numpy as np
import matplotlib.pyplot as plt
from pprint import pformat, pprint

# Teensy 3.2 at 96Mhz -> Bus at 48Mhz
F_BUS = 48e6


class Base(object):
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)


class Stepper(Base):
    position = 0
    speed = 0
    accel = 0
    jerk = 0
    maxSpeed = 20000
    minSpeed = 100
    maxAccel = 40000
    direction = 1
    target = 0
    deviation = 0
    delta = (1, 1)
    
    def step(self):
        if self.error() == 0:
            return
        self.position += self.direction
    
    def setTargetRel(self, pos):
        self.target += pos
        e = self.error()
        if e > 0:
            self.direction = 1
        elif e < 0:
            self.direction = -1
    
    def setTargetAbs(self, pos):
        self.setTargetRel(pos - self.position)
    
    def error(self):
        return self.target - self.position
        
    
class Controller(Base):
    active = 0
    motors = [
        Stepper(),
        Stepper(),
        Stepper()
    ]
    active_motors = []
    action_queue = []
    profile = None
    done = True
    
    def start(self, point):
        self.done = False
        self.current_action = point
        self.active = len(point)
        for i, p in enumerate(point):
            self.motors[i].setTargetAbs(p)
        
        self.active_motors = sorted(
            self.motors, key=lambda m: -abs(m.error()))
        
        lead_motor = self.active_motors[0]
        e = abs(lead_motor.error())
        lead_motor.deviation = e
        for i in range(1, self.active):
            m = self.active_motors[i]
            m.deviation = 0
            m.delta = (2*abs(m.error()), 2*e)
        
        return self.profile.start(self)
    
    def onStep(self):
        lead_motor = self.active_motors[0]
        lead_motor.step()
        for i in range(1, self.active):
            m = self.active_motors[i]
            m.deviation += m.delta[0]
            if m.deviation >= lead_motor.deviation:
                m.step()
                m.deviation -= m.delta[1]
        
        if self.error() == 0:
            self.onComplete()
    
    def error(self):
        return sum(abs(m.error()) for m in self.active_motors)
    
    def onSpeedUpdate(self):
        return self.profile.update(self)

    def onComplete(self):
        self.done = True


class TrapMotionProfile(Base):
    speedMax = 20000
    speedMin = 1000
    accelMax = 8000
    accelEnd = 0
    decelStart = 0
    distance = 0
    dtMin = 0
    dtMax = 0
    dtAccel = 0
    currentDelay = 0
    
    def start(self, controller):
        e = abs(controller.active_motors[0].error())
        self.distance = e
        dv = self.speedMax-self.speedMin
        dt = dv/float(self.accelMax)
        d = dv*dt/2.0
        self.accelEnd = min(self.distance/2, d)
        self.decelStart = self.distance-self.accelEnd
        self.dtMin = F_BUS/self.speedMin
        self.dtMax = F_BUS/self.speedMax
        self.dtAccel = math.sqrt(2*self.accelMax)
        return self.dtMin
    
    def update(self, controller):
        e = abs(controller.active_motors[0].error())
        pos = abs(self.distance-e)
        if pos < self.accelEnd:
            return F_BUS/(self.dtAccel*math.sqrt(pos)+self.speedMin)
        elif pos < self.decelStart:
            return self.dtMax
        else:
            return F_BUS/(self.dtAccel*math.sqrt(e)+self.speedMin)


class SCurveMotionProfile(Base):
    speedMax = 60000
    speedMin = 100
    accelMax = 80000
    accelAvg = float(40000)#accelMax/2.0
    accelEnd = 0
    decelStart = 0
    distance = 0
    dtMin = 0
    dtMax = 0
    dtAccel = 0
    currentDelay = 0
    lastSpeed = 0
    
    def start(self, controller):
        e = abs(controller.active_motors[0].error())
        d = self.distance = e
        da = self.accelMax-self.accelAvg
        assert self.accelAvg >= self.accelMax/2.0, (
            'Invalid accel params. accelAvg must be >= 1/2 accelMax')
        
        dv = self.speedMax-self.speedMin
        t3 = dv/self.accelAvg
        
        # Determine max velocity we can reach
        d = dv*t3/2.0
        if d > self.distance/2.0:
            print('d is less than 1/2 distance')
            # sqrt(2*a*d)
            dv = math.sqrt(self.accelAvg*self.distance)
            t3 = dv/self.accelAvg  # Recalculate
        
        a2 = self.accelMax*self.accelMax
        
        self.jerkMax = a2*self.accelAvg/(dv*da)
        t1 = self.accelMax/self.jerkMax
        t2 = t3 - t1
        v1 = self.jerkMax*t1*t1/2.0  # speed at t1
        v2 = dv-v1  # speed at t2
        
        dt = max(0, t2-t1)
        d3 = dv*dv/(2*self.accelAvg)  # distance at t3
        d1 = min(d3, self.jerkMax*t1*t1*t1/6.0)  # distance at t1
        d2 = min(d3, d1+self.accelMax*dt*dt/2.0 + v1*dt)  # distance at t2
        self.stage1, self.stage2, self.stage3 = d1, d2, d3
        self.dv, self.v1, self.v2 = dv, v1, v2
        self.t1, self.t2, self.t3 = t1, t2, t3
        
        
        if self.accelAvg > self.accelMax/2.0:
            # TODO: Changing the ratio jacks it up
            # and this hack is not correct
            self.dv += self.speedMin#self.accelMax/self.accelAvg
        #dv = v2-v1
        #d = dv*dt/2.0
        #dt = dv/float(self.accelMax)
        self.dtMin = F_BUS/self.speedMin
        self.dtMax = F_BUS/dv
        self.jerkRoot = math.pow(6/self.jerkMax, 1/3.0)
        pprint(locals())
        pprint(self.__dict__)
        return self.dtMin
    
    def update(self, controller):
        e = abs(controller.active_motors[0].error())
        pos = abs(self.distance-e)
        
        # Accel stages
        if pos <= self.stage1:  # Stage 1
            t = self.jerkRoot * math.pow(pos, 1/3.0)
            v = self.jerkMax/2.0*t*t
        elif pos <= self.stage2:  # Stage 2
            p = pos-self.stage1
            # Good ole quadratic formula saving the day
            v1 = self.v1
            a = self.accelMax
            t = max(0, (math.sqrt(v1*v1+2*a*p)-v1)/a)
            v = self.accelMax*t+self.v1
        elif pos <= self.stage3:  # Stage 3
            t = self.t3-self.jerkRoot *math.pow(pos, 1/3.0)
            v = self.dv-self.jerkMax/2.0*t*t
            
        # Decel stages
        elif e <= self.stage1:  # Stage 7
            t = self.jerkRoot *math.pow(e, 1/3.0)
            v = self.jerkMax/2.0*t*t
        elif e <= self.stage2:  # Stage 6
            p = e-self.stage1
            # Good ole quadratic formula saving the day
            v1 = self.v1
            a = self.accelMax
            t = max(0, (math.sqrt(v1*v1+2*a*p)-v1)/a)
            v = self.accelMax*t+self.v1
        elif e <= self.stage3:  # Stage 5
            t = self.t3-self.jerkRoot*math.pow(e, 1/3.0)
            v = self.dv-self.jerkMax/2.0*t*t
        
        # Constant
        else:  # Stage 4
            # Between accel and decel 
            return self.lastSpeed
        self.lastSpeed = F_BUS/min(self.speedMax, max(self.speedMin, v))
        return self.lastSpeed


class SCurveTransition(Base):
    accelAvg = float(20000)
    accelMax = accelAvg*2
    distance = 0
    jerkMax = 0
    jerkRoot = 0
    end = 0
    reversed = False
    
    def prepare(self, distance, startSpeed, endSpeed):
        da = self.accelMax-self.accelAvg
        
        # We always calculate using low to high
        if startSpeed > endSpeed:
            self.reversed = True
            startSpeed, endSpeed = endSpeed, startSpeed
            
        dv = self.dv = endSpeed-startSpeed
        if dv == 0:
            self.end = 0
            return 0
        
        t3 = self.t3 = dv/self.accelAvg
        
        # Determine max velocity we can reach
        d = dv*t3/2.0
        if d > distance/2.0:
            # sqrt(2*a*d)
            dv = self.dv = math.sqrt(self.accelAvg*distance)
            if dv == 0:
                self.end = 0
                return 0
            t3 = self.t3 = dv/self.accelAvg  # Recalculate
            
        endSpeed = self.dv+startSpeed
        self.endSpeed = endSpeed
        self.startSpeed = startSpeed
            
        a2 = self.accelMax*self.accelMax
        
        self.jerkMax = a2*self.accelAvg/(dv*da)
        t1 = self.t1 = self.accelMax/self.jerkMax
        t2 = self.t2 = t3 - t1
        v1 = self.v1 = self.jerkMax*t1*t1/2.0  # speed at t1
        v2 = self.v2 = dv-v1  # speed at t2
        
        dt = max(0, t2-t1)
        d3 = self.d3 = dv*dv/(2*self.accelAvg)#+t3*startSpeed  # distance at t3
        # distance at t1
        d1 = self.d1 = min(d3, self.jerkMax*t1*t1*t1/6.0)#+t1*startSpeed)  
        # distance at t2
        d2 = self.d2 = min(d3, d1+self.accelMax*dt*dt/2.0 + v1*dt)  
        self.jerkRoot = math.pow(6/self.jerkMax, 1/3.0)
        self.end = d3
        self.distance = distance
        
        #pprint(locals())
        pprint(self.__dict__)
        
        return endSpeed
    
    def update(self, pos):
        # Accel stages
        if pos <= self.d1:  # Stage 1
            t = self.jerkRoot * math.pow(pos, 1/3.0)
            return self.jerkMax/2.0*t*t+self.startSpeed
        elif pos <= self.d2:  # Stage 2
            p = pos-self.d1
            # Good ole quadratic formula saving the day
            v1 = self.v1
            a = self.accelMax
            t = max(0, (math.sqrt(v1*v1+2*a*p)-v1)/a)
            return self.accelMax*t+self.v1+self.startSpeed
        elif pos <= self.d3:  # Stage 3
            t = self.t3-self.jerkRoot * math.pow(pos, 1/3.0)
            return self.endSpeed-self.jerkMax/2.0*t*t
        return 0  # Unchanged


class TransitionMotionProfile(Base):
    
    accel = SCurveTransition()
    decel = SCurveTransition()
    startSpeed = 0
    finalSpeed = 0
    speedMin = 1
    speedMax = 40000
    lastSpeed = 0
    
    def start(self, controller):
        e = abs(controller.active_motors[0].error())
        self.distance = e
        startSpeed = max(self.speedMin, self.lastSpeed)
        
        d = e
        
        # Determine the max speed that still allows slowing down
        # to the final speed
        dv = math.sqrt(self.accel.accelAvg*e)
        
        if controller.action_queue:
            # TODO: Determine what speed
            #errors = [0, 0, 0]
            #for i in range(len(controller.action_queue)):
            #    
            #d = controller.action_queue[-1][0]/2
            #p0 = controller.current_action
            #p1 = controller.action_queue[0]
            #cos_theta = (p0[0]*p1[0]+p0[1]*p1[1])/(
            #        math.sqrt(p0[0]*p0[0]+p0[1]*p0[1])*
            #        math.sqrt(p1[0]*p1[0]+p1[1]*p1[1]))
            #finalSpeed = midSpeed#self.speedMin*40#midSpeed * cos_theta
            finalSpeed = self.speedMax/2
        else:
            finalSpeed = self.speedMin
        
        # dv = cruiseSpeed - finalSpeed
        # dv = cruiseSpeed - startSpeed
        
        # Max speed we can reage
        speed = min(self.speedMax, min(dv+finalSpeed, dv+startSpeed))
                
        #: TODO: If dv is the same we can just copy values over
        self.cruiseSpeed = self.accel.prepare(d, startSpeed, speed)
        self.finalSpeed = self.decel.prepare(d, speed, finalSpeed)
        self.lastSpeed = startSpeed
        #print("Clock: {}".format(controller.clock))
        #print("Position: {}".format([m.position for m in controller.motors]))
        return F_BUS/self.lastSpeed
    
    def update(self, controller):
        e = abs(controller.active_motors[0].error())
        pos = abs(self.distance-e)
        v = 0 
        if pos <= self.accel.end:
            if self.accel.reversed:
                pos = self.accel.end-pos
            v = self.accel.update(pos)
        elif e <= self.decel.end:
            if not self.decel.reversed:
                e = self.decel.end-e
            v = self.decel.update(e)
        if v == 0:  # Constant speed
            #print(((pos, e), 'stage4'))
            return F_BUS/self.lastSpeed 
        self.lastSpeed = min(self.speedMax, max(self.speedMin, v))
        return F_BUS/self.lastSpeed


class Simulation(object):
    """ Simulates the TeensyStep controller
    
    """
    speedUpdateRate = 5000
    profile = None
    
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
            
    def take_snapshot(self, clk, sim, controller, t1_expiry):
        for axis, m in enumerate(controller.motors):
            # Log simulation data
            last = sim[axis][-1]
            t = (clk-last[0])/self.simulation_speed
            if t > 10:
                v = (m.position-last[1])/t
                a = (v-last[2])/t
                j = (a-last[3])/t
                sim[axis].append((clk, m.position, v, a, j, t1_expiry))
    
    def run(self, path, time_limit=10, sample_smoothing=1, 
            simulation_speed=10.0):
        """
        Run a simulation of a move from 0 to target steps
        
        Parameters
        ----------
        path: list
            List of points to move to
        time_limit: float
            Seconds to wait before stopping the simulation if target is not hit
        sample_smoothing: unsigned int
            Smooth samples over multiple points
        simulation_speed: float
            Scales the timers to "speed up" the simulation
        
        """
        # t, p, v, a, j
        start_time = time.time()
        controller = self.controller
        simulation_speed = self.simulation_speed = float(simulation_speed)
        
        # Setup simulated timers
        t1_expiry = None  # PIT timer
        
        # FTM timer
        t2_expiry = F_BUS/1000000*self.speedUpdateRate/simulation_speed  
        
        # Simulation watchdog timer
        t3_expiry = F_BUS/(2*simulation_speed)  
        
        # Set initial values clk, pos, v, a, j, period
        p, v, a, j, = 0, 0, 0, 0
        sim = [[(-t2_expiry/3, p, v, a, j, 0), (0, p, v, a, j, 0)]
               for m in controller.motors]
        
        # Run until complete
        #print("Timers: PIT={}, FTM={}".format(t1_expiry, t2_expiry))
        
        # Run simulation
        clk, i, t1, t2, t3 = 0, 0, 0, 0, 0
        while True:
            clk += 1
            t1 += 1
            t2 += 1
            t3 += 1
            # Handle next point
            if controller.done:
                queue = controller.action_queue
                if not queue:
                    break  # No more moves
                controller.clock = clk
                t1_expiry = controller.start(queue.pop(0))/simulation_speed
                t1, t2 = 0, 0  # clear timers
            if t1_expiry and t1 >= t1_expiry:  # PIT timer
                t1 = 0  # clear timer
                controller.onStep()
            if t1_expiry and t2 >= t2_expiry:  # FTM timer
                t2 = 0  # clear timer
                t1_expiry = controller.onSpeedUpdate()/simulation_speed
                i += 1
                # If we log too much we get jittery data
                if i >= sample_smoothing:
                    i = 0
                    self.take_snapshot(clk, sim, controller, 
                                       t1_expiry*simulation_speed)
                    
            # In case nothing happens abort
            if t3 >= t3_expiry:
                t3 = 0
                if time.time()-start_time > time_limit:
                    print("Aborted: {} (error {})".format(
                        clk, [m.error() for m in controller.motors]))
                    break
            
        print("Complete: real={}s simulated={}s".format(
            round(time.time()-start_time, 2), 
            round(clk/float(F_BUS), 2)
            ))
        return sim


def main():
    # Simulate the given motion profile
    
    path = []
    
    #for i in range(1, 3):
    #    path.append(
    #        (i*50000, i*25000, 0)
    #    )
    
    path.extend([
        (0, 10000, 1000), 
        (5000, 5000, 500), 
        (20000, 5000, 500), 
        (5000, 10000, 500), 
        (0, 0, 0), 
    ])
    
    
    controller = Controller(
        #profile=TrapMotionProfile(),
        profile=SCurveMotionProfile(),
        #profile=TransitionMotionProfile(),
        action_queue=path
    )
    sim = Simulation(controller=controller)
    data = sim.run(path=controller.action_queue, 
                   time_limit=30, sample_smoothing=10,
                   simulation_speed=10)
    
    # Plot the data
    time_scale = F_BUS/float(sim.simulation_speed)
    t = np.array([s[0] for s in data[0]])/time_scale  # times are all the same
    fig, plots = plt.subplots(6, 1)
    plots[0].set_title("Simulation")
    for i, (label, color) in enumerate((
              ('Position', 'red'),
              ('Velocity', 'blue'),
              ('Accel', 'green'),
              ('Jerk', 'purple'),
              ('Cycles', 'orange')
            )):
        p = plots[i]
        for axis_data in data:
            ydata = [s[i+1] for s in axis_data]
            p.plot(t, ydata, label=label+str(i+1))
        p.set_ylabel(label)
        p.grid()
    plots[-2].set_xlabel('Time (s)')
    p = plots[-1]
    p.scatter([s[1] for s in data[0]], [s[1] for s in data[1]],
              marker='.', s=4)
    p.set_ylabel("X - Y")
    plt.show()


if __name__ == '__main__':
    # Keep ctrl c
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()
