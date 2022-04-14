import math

class PIDController:
    def __init__(self, Kp=0.0, Kd=0.0, Ki=0.0):
        self.Kp = float(Kp)
        self.Kd = float(Kd)
        self.Ki = float(Ki)
        self.err_last = 0.0
        self.err_int = 0.0
    
    def apply(self, err):
        self.err_int += err
        u = self.Kp * err + self.Kd * (err-self.err_last) + self.Ki * self.err_int
        self.err_last = err
        return u

    def reset(self):
        self.err_int = 0.0
        self.err_last = 0.0

class NonholomonicController:
    def __init__(self, kr=0.0, ka=0.0, kb=0.0):
        self.k_rho = float(kr)
        self.k_alpha = float(ka)
        self.k_beta = float(kb)

    def apply(self, dx, dy, theta):
        rho = math.sqrt(dx*dx) # + dy*dy)
        alpha = - theta + math.atan2(dy, dx)
        beta = - theta - alpha
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        return v, w