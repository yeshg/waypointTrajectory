class PIDController:
    def __init__(self, kp, ki, kd):
        self.previous_error = 0
        self.error_sum = 0
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def get_u(self,set_point,current_value):
        # calculate the errors
        error = set_point - current_value
        # accumulate the errors
        self.error_sum += error
        # calculate the rate of change on the errors
        delta_error = error - self.previous_error
        # save the current error as the previous error for the next iteration
        self.previous_error = error
        # pid equation...
        output = self.Kp*error + self.Ki*self.error_sum + self.Kd*delta_error

        return output