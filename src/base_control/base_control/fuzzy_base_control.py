import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def fuzzy_logic_calc(V_x:float,V_y:float,W_z:float):
        
    ex=ctrl.Antecedent(np.arange(-0.1,0.450,0.01), 'error_in_x')
    vx = ctrl.Consequent(np.arange(-0.4,0.4, 0.01), 'speed_in_x')

    ey=ctrl.Antecedent(np.arange(-0.21,0.210,0.01), 'error_in_y')
    vy = ctrl.Consequent(np.arange(-0.2,0.2, 0.01), 'speed_in_y')

    ez=ctrl.Antecedent(np.arange(-45,45,0.01), 'error_in_w')
    wz = ctrl.Consequent(np.arange(-1.2,1.2, 0.01), 'rotation_about_z')


    ex['high_negative_deviation']=fuzz.trapmf(ex.universe, [-0.100,-0.100,-0.050,0])
    ex['low_negative_deviation']=fuzz.trimf(ex.universe, [-0.100,-0.050,0])
    ex['zero_deviation']=fuzz.trimf(ex.universe, [-0.20,0,0.20])
    ex['low_positive_deviation']=fuzz.trimf(ex.universe, [0,0.100,0.250])
    ex['high_positive_deviation']=fuzz.trapmf(ex.universe, [0.150,0.300,0.450,0.450])

    vx['high_negative_velocity']=fuzz.trapmf(vx.universe, [-0.4,-0.4,-0.35,-0.25])
    vx['low_negative_velocity']=fuzz.trimf(vx.universe, [-0.2,-0.1,0])
    vx['zero_deviation_velocity']=fuzz.trimf(vx.universe, [-0.05,0,0.05])
    vx['low_positive_velocity']=fuzz.trimf(vx.universe, [0,0.300,0.40])
    vx['high_positive_velocity']=fuzz.trapmf(vx.universe, [0.250,0.350,0.40,0.40])

    rule1_x=ctrl.Rule(ex['high_negative_deviation'],vx['high_negative_velocity'])
    rule2_x=ctrl.Rule(ex['low_negative_deviation'],vx['low_negative_velocity'])
    rule3_x=ctrl.Rule(ex['zero_deviation'],vx['zero_deviation_velocity'])
    rule4_x=ctrl.Rule(ex['low_positive_deviation'],vx['low_positive_velocity'])
    rule5_x=ctrl.Rule(ex['high_positive_deviation'],vx['high_positive_velocity'])

    #------------------------------------------------------------------------------

    ey['high_negative_deviation']=fuzz.trapmf(ey.universe, [-0.210,-0.210,-0.100,0])
    ey['low_negative_deviation']=fuzz.trimf(ey.universe, [-0.210,-0.100,0])
    ey['zero_deviation']=fuzz.trimf(ey.universe, [-0.50,0,0.50])
    ey['low_positive_deviation']=fuzz.trimf(ey.universe, [0,0.100,0.210])
    ey['high_positive_deviation']=fuzz.trapmf(ey.universe, [0.100,0.150,0.210,0.210])

    vy['high_negative_velocity']=fuzz.trapmf(vy.universe, [-0.2,-0.2,-0.175,-0.1])
    vy['low_negative_velocity']=fuzz.trimf(vy.universe, [-0.175,-0.1,0])
    vy['zero_deviation_velocity']=fuzz.trimf(vy.universe, [-0.05,0,0.05])
    vy['low_positive_velocity']=fuzz.trimf(vy.universe, [0,0.1,0.1750])
    vy['high_positive_velocity']=fuzz.trapmf(vy.universe, [0.1,0.175,0.2,0.2])

    rule1_y=ctrl.Rule(ey['high_negative_deviation'],vy['high_negative_velocity'])
    rule2_y=ctrl.Rule(ey['low_negative_deviation'],vy['low_negative_velocity'])
    rule3_y=ctrl.Rule(ey['zero_deviation'],vy['zero_deviation_velocity'])
    rule4_y=ctrl.Rule(ey['low_positive_deviation'],vy['low_positive_velocity'])
    rule5_y=ctrl.Rule(ey['high_positive_deviation'],vy['high_positive_velocity'])


    #----------------------------------------------------------------

    ez['high_negative_deviation']=fuzz.trapmf(ez.universe, [-45,-45,-20,0])
    ez['low_negative_deviation']=fuzz.trimf(ez.universe, [-45,-20,0])
    ez['zero_deviation']=fuzz.trimf(ez.universe, [-5,0,5])
    ez['low_positive_deviation']=fuzz.trimf(ez.universe, [0,20,45])
    ez['high_positive_deviation']=fuzz.trapmf(ez.universe, [20,30,45,45])

    wz['high_negative_velocity']=fuzz.trapmf(wz.universe, [-1.2,-1.2,-1,-0.75])
    wz['low_negative_velocity']=fuzz.trimf(wz.universe, [-0.175,-0.1,0])
    wz['zero_deviation_velocity']=fuzz.trimf(wz.universe, [-0.05,0,0.05])
    wz['low_positive_velocity']=fuzz.trimf(wz.universe, [0,0.1,0.175])
    wz['high_positive_velocity']=fuzz.trapmf(wz.universe, [0.75,1,1.2,1.2])

    rule1_z=ctrl.Rule(ez['high_negative_deviation'],wz['high_negative_velocity'])
    rule2_z=ctrl.Rule(ez['low_negative_deviation'],wz['low_negative_velocity'])
    rule3_z=ctrl.Rule(ez['zero_deviation'],wz['zero_deviation_velocity'])
    rule4_z=ctrl.Rule(ez['low_positive_deviation'],wz['low_positive_velocity'])
    rule5_z=ctrl.Rule(ez['high_positive_deviation'],wz['high_positive_velocity'])


    all_rules=ctrl.ControlSystem([rule1_x,rule2_x,rule3_x,rule4_x,rule5_x,
                                rule1_y,rule2_y,rule3_y,rule4_y,rule5_y,
                                rule1_z,rule2_z,rule3_z,rule4_z,rule5_z,])


    base_control=ctrl.ControlSystemSimulation(all_rules)

    base_control.input['error_in_x'] = V_x
    base_control.input['error_in_y'] = V_y
    base_control.input['error_in_w'] = W_z

    base_control.compute()
    return [base_control.output['speed_in_x'],base_control.output['speed_in_y'],base_control.output['rotation_about_z']]

# list=fuzzy_logic_calc(0.2,0.1,1)
# print("Vx =",list[0])
# print("Vy =", list[1])
# print("Wz =", list[2])