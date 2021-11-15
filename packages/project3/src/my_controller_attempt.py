import numpy as np


class LaneController:

    #This is my attempt at a PD controller.

    def __init__(self, parameters):
        self.parameters = parameters
        self.prev_d_err = 1.0
        self.prev_phi_err = 1.0
        self.d_deriv = 0
        self.phi_deriv = 0

    def update_parameters(self, parameters):
        self.parameters = parameters

    def compute_control_action(self, d_err, phi_err, dt, wheels_cmd_exec):

        self.reset_if_needed(d_err, phi_err, wheels_cmd_exec)
        
        
        if np.abs(d_err) < 0.02:
            d_err = 0
        

        if dt is not None:
            self.d_deriv = (d_err - self.prev_d_err) / dt
            self.phi_deriv = (phi_err - self.prev_phi_err) / dt
        
        omega = (
            self.parameters["~k_d"].value * d_err
            + self.parameters["~k_theta"].value * phi_err
            + self.d_deriv * self.parameters["~k_Dd"].value
            + self.phi_deriv * self.parameters["~k_Dphi"].value
        )


        self.prev_d_err = d_err
        self.prev_phi_err = phi_err
         
        v = self.parameters["~v_bar"].value

        
        return v, omega



    def reset_if_needed(self, d_err, phi_err, wheels_cmd_exec):
        if np.sign(d_err) != np.sign(self.prev_d_err):
            self.d_I = 0
        if np.sign(phi_err) != np.sign(self.prev_phi_err):
            self.phi_I = 0
        if wheels_cmd_exec[0] == 0 and wheels_cmd_exec[1] == 0:
            self.d_I = 0
            self.phi_I = 0

