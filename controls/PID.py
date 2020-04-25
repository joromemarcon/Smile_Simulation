import time

class PID:
    def __init__(self,kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.cumError = 0

    def P(self, error):
        '''
        Proportion (Present):
            In order to reach our 'goal' multiply error with proportional gain (kp)

        error: Error of the control system
        '''
        return (self.kp * error)

    def I(self, error, etime):
        '''
        Integration (Past):
        SP: Set Point, the gaol value 
        PV: Previous value of the system
        '''
        self.cumError += error * etime
        return (self.ki * self.cumError)

    def D(self, error, prv_error, etime):
        '''
        Derivative (Future):
        error: Error of the control system
        '''
        rateError = (error - prv_error)/etime
        return (self.kd * rateError)

    def pid_controller(self, error, prv_error, etime):
        '''
        pid_controller: This is what is being called to produce our actions
        error: current error of the system 
        prv_error: previous error of the system
        etime: elapsed time of the comand
        '''
        p = self.P(error)
        i = self.I(error,etime)
        d = self.D(error, prv_error, etime)
        pid = p + i + d
        return (pid)
    

        


