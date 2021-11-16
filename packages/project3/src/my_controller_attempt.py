import numpy as np
from time import sleep

class LaneController:

    #This is my attempt at a PD controller.

    def __init__(self, parameters):
        self.parameters = parameters
        self.prev_d_err = 1.0
        self.prev_phi_err = 1.0
        self.d_deriv = 0.0
        self.phi_deriv = 0.0
        self.d_I = 0.0
        self.phi_I = 0.0
        d_err_array = [0, 0, 0, 0]
        self.omega_array = [0, 0, 0, 0]

    def update_parameters(self, parameters):
        self.parameters = parameters

    def compute_control_action(self, d_err, phi_err, dt, wheels_cmd_exec):

        self.reset_if_needed(d_err, phi_err, wheels_cmd_exec)
        
        d_err = d_err + 0.02

        #if d_err < 0.1 and d_err > -0.1:
        #    d_err = 0
        if phi_err < 0.1 and phi_err > -0.1:
            phi_err = 0

        #d_err_array.apend(d_err)
        #d_err_array = d_err_array[1:]
        #d_err = sum(d_err_array) / len(d_err_array)
        

        if dt is not None:
            self.d_deriv = (d_err - self.prev_d_err) / dt
            self.phi_deriv = (phi_err - self.prev_phi_err) / dt
            self.d_I += d_err * dt
            self.phi_I += phi_err * dt
        
        if self.d_I > 0.3:
            self.d_I = 0.3       
        if self.d_I < -0.3:
            self.d_I = -0.3
        if self.phi_I > 1.2:
            self.phi_I = 1.2
        if self.phi_I < -1.2:
            self.d_I = -1.2

        

        omega = (
            self.parameters["~k_d"].value * d_err
            + self.parameters["~k_theta"].value * phi_err
            + self.d_I * self.parameters["~k_Id"].value
            + self.phi_I * self.parameters["~k_Iphi"].value
            + self.d_deriv * self.parameters["~k_Dd"].value
            + self.phi_deriv * self.parameters["~k_Dphi"].value
        )

        
        if omega > 5:
            omega = 5
        if omega < -5:
            omega = -5
        #if phi_err > -0.3 and phi_err < 0.3:
        #    omega = 0

        self.prev_d_err = d_err
        self.prev_phi_err = phi_err
         
        v = self.parameters["~v_bar"].value - np.abs(d_err)

        return v, omega



    def reset_if_needed(self, d_err, phi_err, wheels_cmd_exec):
        if np.sign(d_err) != np.sign(self.prev_d_err):
            self.d_I = 0
        if np.sign(phi_err) != np.sign(self.prev_phi_err):
            self.phi_I = 0
        if wheels_cmd_exec[0] == 0 and wheels_cmd_exec[1] == 0:
            self.d_I = 0
            self.phi_I = 0

