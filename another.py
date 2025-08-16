import numpy as np

a1 = 6
a2 = 8  

def forward_kinematics(beta , gamma):
        """Calculate end effector position given joint angles"""
        beta = beta - 90

        beta_rad = np.radians(beta)
        gamma_rad = np.radians(gamma)
        
        x1 = a1 * np.cos(beta_rad)
        y1 = a1 * np.sin(beta_rad)
        
        x2 = x1 + a2 * np.cos(beta_rad + gamma_rad)
        y2 = y1 + a2 * np.sin(beta_rad + gamma_rad)
        
        # return (x1, y1), (x2, y2)
        print(f"x1 = {x1}")
        print(f"y1 = {y1}")
        print(f"x2 = {x2}")
        print(f"y2 = {y2}")

forward_kinematics(135, 135)