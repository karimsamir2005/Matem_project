import numpy as np
import math
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class fuzzy:
    def __init__(self):
        self.ex=ctrl.Antecedent(np.arange(-1.5,1.51,0.01), 'error_in_x')
        self.vx = ctrl.Consequent(np.arange(-0.5,0.51, 0.01), 'speed_in_x')

        self.ey=ctrl.Antecedent(np.arange(-1.5,1.51,0.01), 'error_in_y')
        self.vy = ctrl.Consequent(np.arange(-0.5,0.51, 0.01), 'speed_in_y')

        self.ez=ctrl.Antecedent(np.arange(-math.pi,math.pi+0.01,0.01), 'error_in_w')
        self.wz = ctrl.Consequent(np.arange(-1.2,1.21, 0.01), 'rotation_about_z')

        self.ex['high_negative_deviation']=fuzz.trapmf(self.ex.universe, [-1.5,-1.5,-1,-0.5])
        self.ex['low_negative_deviation']=fuzz.trimf(self.ex.universe, [-1,-0.50,0])
        self.ex['zero_deviation']=fuzz.gaussmf(self.ex.universe, 0,0.10)
        self.ex['low_positive_deviation']=fuzz.trimf(self.ex.universe, [0,0.5,1])
        self.ex['high_positive_deviation']=fuzz.trapmf(self.ex.universe, [0.5,1,1.5,1.5])

        self.vx['high_negative_velocity']=fuzz.zmf(self.vx.universe,-0.35,-0.1)
        self.vx['low_negative_velocity']=fuzz.trimf(self.vx.universe, [-0.4,-0.2,0])
        self.vx['zero_deviation_velocity']=fuzz.gaussmf(self.vx.universe,0,0.03)
        self.vx['low_positive_velocity']=fuzz.trimf(self.vx.universe, [0,0.2,0.40])
        self.vx['high_positive_velocity']=fuzz.smf(self.vx.universe,0.1,0.35)

        rule1_x=ctrl.Rule(self.ex['high_negative_deviation'],self.vx['high_negative_velocity'])
        rule2_x=ctrl.Rule(self.ex['low_negative_deviation'],self.vx['low_negative_velocity'])
        rule3_x=ctrl.Rule(self.ex['zero_deviation'],self.vx['zero_deviation_velocity'])
        rule4_x=ctrl.Rule(self.ex['low_positive_deviation'],self.vx['low_positive_velocity'])
        rule5_x=ctrl.Rule(self.ex['high_positive_deviation'],self.vx['high_positive_velocity'])

        #------------------------------------------------------------------------------

        self.ey['high_negative_deviation']=fuzz.trapmf(self.ey.universe, [-1.5,-1.5,-1,-0.5])
        self.ey['low_negative_deviation']=fuzz.trimf(self.ey.universe, [-1,-0.5,0])
        self.ey['zero_deviation']=fuzz.gaussmf(self.ey.universe,0,0.10)
        self.ey['low_positive_deviation']=fuzz.trimf(self.ey.universe, [0,0.5,1])
        self.ey['high_positive_deviation']=fuzz.trapmf(self.ey.universe, [0.5,1,1.5,1.5])

        self.vy['high_negative_velocity']=fuzz.zmf(self.vy.universe,-0.35,-0.1)
        self.vy['low_negative_velocity']=fuzz.trimf(self.vy.universe, [-0.4,-0.2,0])
        self.vy['zero_deviation_velocity']=fuzz.gaussmf(self.vy.universe,0,0.05)
        self.vy['low_positive_velocity']=fuzz.trimf(self.vy.universe, [0,0.2,0.4])
        self.vy['high_positive_velocity']=fuzz.smf(self.vy.universe,0.1,0.35)

        rule1_y=ctrl.Rule(self.ey['high_negative_deviation'],self.vy['high_negative_velocity'])
        rule2_y=ctrl.Rule(self.ey['low_negative_deviation'],self.vy['low_negative_velocity'])
        rule3_y=ctrl.Rule(self.ey['zero_deviation'],self.vy['zero_deviation_velocity'])
        rule4_y=ctrl.Rule(self.ey['low_positive_deviation'],self.vy['low_positive_velocity'])
        rule5_y=ctrl.Rule(self.ey['high_positive_deviation'],self.vy['high_positive_velocity'])


        #----------------------------------------------------------------
        #why do we need the paramters in the z axis the motion is in 2D??

        self.ez['high_negative_deviation']=fuzz.trapmf(self.ez.universe, [-math.pi,-math.pi,-2.0,-0.8])
        self.ez['low_negative_deviation']=fuzz.trimf(self.ez.universe, [-2,-0.8,0])
        self.ez['zero_deviation']=fuzz.gaussmf(self.ez.universe,0,0.25)
        self.ez['low_positive_deviation']=fuzz.trimf(self.ez.universe, [0,0.8,2.0])
        self.ez['high_positive_deviation']=fuzz.trapmf(self.ez.universe, [0.8,2,math.pi,math.pi])

        self.wz['high_negative_velocity']=fuzz.zmf(self.wz.universe,-1.2,-0.75)
        self.wz['low_negative_velocity']=fuzz.trimf(self.wz.universe, [-0.8,-0.4,0])
        self.wz['zero_deviation_velocity']=fuzz.gaussmf(self.wz.universe,0,0.2)
        self.wz['low_positive_velocity']=fuzz.trimf(self.wz.universe, [0,0.4,0.8])
        self.wz['high_positive_velocity']=fuzz.smf(self.wz.universe,0.75,1.2)

        rule1_z=ctrl.Rule(self.ez['high_negative_deviation'],self.wz['high_negative_velocity'])
        rule2_z=ctrl.Rule(self.ez['low_negative_deviation'],self.wz['low_negative_velocity'])
        rule3_z=ctrl.Rule(self.ez['zero_deviation'],self.wz['zero_deviation_velocity'])
        rule4_z=ctrl.Rule(self.ez['low_positive_deviation'],self.wz['low_positive_velocity'])
        rule5_z=ctrl.Rule(self.ez['high_positive_deviation'],self.wz['high_positive_velocity'])


        self.all_rules=[rule1_x,rule2_x,rule3_x,rule4_x,rule5_x,
                                    rule1_y,rule2_y,rule3_y,rule4_y,rule5_y,
                                    rule1_z,rule2_z,rule3_z,rule4_z,rule5_z]
        self.control_system = ctrl.ControlSystem(self.all_rules)
        self.base_control = ctrl.ControlSystemSimulation(self.control_system)   



    def fuzzy_logic_calc(self,V_x:float,V_y:float,W_z:float):
        self.base_control.input['error_in_x'] = V_x
        self.base_control.input['error_in_y'] = V_y
        self.base_control.input['error_in_w'] = W_z
        #base_control.compute()
        #return [base_control.output['speed_in_x'],base_control.output['speed_in_y'],base_control.output['rotation_about_z']]
        try:
            self.base_control.compute()
            return [
            self.base_control.output['speed_in_x'],
            self.base_control.output['speed_in_y'],
            self.base_control.output['rotation_about_z']
            ]
        except Exception as e:
            return [0.0, 0.0, 0.0]

# list=fuzzy_logic_calc(0.2,0.1,1)1,1.2,
# print("Vx =",list[0])
# print("Vy =", list[1])
# print("self.wz =", list[2])