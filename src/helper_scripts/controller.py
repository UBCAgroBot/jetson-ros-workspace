import time


class PID_controller:
    """ Use the PID controller to control wheels according to intersection point.
    """
    previous = 0.0
    integral = 0.0

    def __init__(self, k_p=0.1, k_i=0.0, k_d=0.0, use_p=True, use_i=True, use_d=True):
        """
        Parameters
        ----------
        - k_p : float, constant for P (proportional) section of the PID. Default is 0.1
        - k_i : float, constant for I (integral) section of the PID. Default is 0.0
        - k_d : float, constant for D (derivative) section of the PID. Default is 0.0
        - use_p, use_i, use_d : boolean, PID controller can work as P, PI etc.
                             Turn these to False to not use that part of the PID.
                             Default is True. It is usually a good practice to use P.
        """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.use_p = use_p
        self.use_i = use_i
        self.use_d = use_d
        self.lasttime = time.time()

    def move(self, current, target=0.0):
        """ Determines the wheel angle for the next step using the PID controller.
        Use on regular intervals for best performance.

        Parameters
        ----------
        current : int or float

        target : int or float, default is 0.0

        Returns
        ----------
        float, angle to set for the front wheels (if constants are set correctly when initializing the controller)

        """
        output = 0.0
        error = target - current
        dt = time.time() - self.lasttime
        self.integral += error * dt
        derivative = float(error - self.previous) / float(dt)
        self.previous = error
        if self.use_p:
            output += self.k_p * error
        if self.use_i:
            output += self.k_i * self.integral
        if self.use_d:
            output += self.k_d * derivative
        self.lasttime = time.time()
        return output
