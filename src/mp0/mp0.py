from typing import Tuple, List 

import numpy as np 
from scipy.integrate import ode 

from verse import BaseAgent, Scenario
from verse.analysis.utils import wrap_to_pi 
from verse.analysis.analysis_tree import TraceType, AnalysisTree 
from verse.parser import ControllerIR
from vehicle_controller import VehicleMode, PedestrianMode
from verse.analysis import AnalysisTreeNode, AnalysisTree, AnalysisTreeNodeType

import copy 

refine_profile = {
    'R1': [0],
    'R2': [3,3,3,0],
    'R3': [3,3,3,0]
}

def tree_safe(tree: AnalysisTree):
    for node in tree.nodes:
        if node.assert_hits is not None:
            return False 
    return True

def verify_refine(scenario: Scenario, time_horizon, time_step):
    refine_depth = 10
    init_car = scenario.init_dict['car']
    init_ped = scenario.init_dict['pedestrian']
    partition_depth = 0
    if init_ped[1][0] - init_ped[0][0]>0.1:
        exp = 'R3'
    elif init_car[1][3] - init_car[0][3] > 0.1:
        exp = 'R2'
    else:
        exp = 'R1'
    res_list = []
    init_queue = []
    if init_car[1][3]-init_car[0][3] > 0.05:
        car_v_init_range = np.linspace(init_car[0][3], init_car[1][3], 33)
    else:
        car_v_init_range = [init_car[0][3], init_car[1][3]]
    if init_car[1][0]-init_car[0][0] > 0.1:
        car_x_init_range = np.linspace(init_car[0][0], init_car[1][0], 5)
    else:
        car_x_init_range = [init_car[0][0], init_car[1][0]]
    for i in range(len(car_x_init_range)-1):
        for j in range(len(car_v_init_range)-1):
            tmp = copy.deepcopy(init_car)
            tmp[0][0] = car_x_init_range[i]
            tmp[1][0] = car_x_init_range[i+1]
            tmp[0][3] = car_v_init_range[j]
            tmp[1][3] = car_v_init_range[j+1]
            init_queue.append((tmp, init_ped, partition_depth))
    # init_queue = [(init_car, init_ped, partition_depth)]
    safe = []
    unsafe = []
    while init_queue!=[] and partition_depth < refine_depth:
        car_init, ped_init, partition_depth = init_queue.pop(0)
        print(f"######## {partition_depth}, car x, {car_init[0][0]}, {car_init[1][0]}, car v, {car_init[0][3]}, {car_init[1][3]}, ped x, {ped_init[0][0]}, {ped_init[1][0]}, ped y, {ped_init[0][1]}, {ped_init[1][1]}")
        scenario.set_init_single('car', car_init, (VehicleMode.Normal,))
        scenario.set_init_single('pedestrian', ped_init, (PedestrianMode.Normal,))
        traces = scenario.verify(time_horizon, time_step)
        if not tree_safe(traces):
            # Partition car and pedestrian initial state
            idx = refine_profile[exp][partition_depth%len(refine_profile[exp])]
            if car_init[1][idx] - car_init[0][idx] < 0.01:
                print(f"Stop refine car state {idx}")
                init_queue.append((car_init, ped_init, partition_depth+1))
            elif partition_depth >= refine_depth:
                print('Threshold Reached. Scenario is UNSAFE.')
                res_list.append(traces)
                unsafe.append({'car':car_init, 'pedestrian': ped_init})
                break
            car_v_init = (car_init[0][idx] + car_init[1][idx])/2
            car_init1 = copy.deepcopy(car_init)
            car_init1[1][idx] = car_v_init 
            init_queue.append((car_init1, ped_init, partition_depth+1))
            car_init2 = copy.deepcopy(car_init)
            car_init2[0][idx] = car_v_init 
            init_queue.append((car_init2, ped_init, partition_depth+1))
        else:
            safe.append({'car': car_init, 'pedestrian': ped_init})
            res_list.append(traces)
    com_traces = combine_tree(res_list)
    
    return com_traces, safe, unsafe

class PedestrianAgent(BaseAgent):
    def __init__(
        self, 
        id, 
    ):
        self.decision_logic: ControllerIR = ControllerIR.empty()
        self.id = id 

    @staticmethod
    def dynamic(t, state):
        x, y, theta, v = state
        x_dot = 0
        y_dot = v
        theta_dot = 0
        v_dot = 0
        return [x_dot, y_dot, theta_dot, v_dot]    

    def TC_simulate(
        self, mode: List[str], init, time_bound, time_step, lane_map = None
    ) -> TraceType:
        time_bound = float(time_bound)
        num_points = int(np.ceil(time_bound / time_step))
        trace = np.zeros((num_points + 1, 1 + len(init)))
        trace[1:, 0] = [round(i * time_step, 10) for i in range(num_points)]
        trace[0, 1:] = init
        for i in range(num_points):
            r = ode(self.dynamic)
            r.set_initial_value(init)
            res: np.ndarray = r.integrate(r.t + time_step)
            init = res.flatten()
            if init[3] < 0:
                init[3] = 0
            trace[i + 1, 0] = time_step * (i + 1)
            trace[i + 1, 1:] = init
        return trace

class VehicleAgent(BaseAgent):
    def __init__(
        self, 
        id, 
        code = None,
        file_name = None, 
        accel_brake = 5,
        accel_notbrake = 5,
        accel_hardbrake = 20,
        speed = 10
    ):
        super().__init__(
            id, code, file_name
        )
        self.accel_brake = accel_brake
        self.accel_notbrake = accel_notbrake
        self.accel_hardbrake = accel_hardbrake
        self.speed = speed
        self.vmax = 20
         
    @staticmethod
    def dynamic(t, state, u):
        x, y, theta, v = state
        delta, a = u
        x_dot = v * np.cos(theta + delta)
        y_dot = v * np.sin(theta + delta)
        theta_dot = v / 1.75 * np.sin(delta)
        v_dot = a
        return [x_dot, y_dot, theta_dot, v_dot]
    
    def action_handler(self, mode: List[str], state) -> Tuple[float, float]:
        x, y, theta, v = state
        vehicle_mode,  = mode
        vehicle_pos = np.array([x, y])
        a = 0
        lane_width = 3
        d = -y
        if vehicle_mode == "Normal" or vehicle_mode == "Stop":
            pass
        elif vehicle_mode == "SwitchLeft":
            d += lane_width
        elif vehicle_mode == "SwitchRight":
            d -= lane_width
        elif vehicle_mode == "Brake":
            a = max(-self.accel_brake, -v)
            # a = -50
        elif vehicle_mode == "HardBrake":
            a = max(-self.accel_hardbrake, -v)
            # a = -50
        elif vehicle_mode == "Accel":
            a = min(self.accel_notbrake, self.speed-v)
        else:
            raise ValueError(f"Invalid mode: {vehicle_mode}")

        heading = 0
        psi = wrap_to_pi(heading - theta)
        steering = psi + np.arctan2(0.45 * d, v)
        steering = np.clip(steering, -0.61, 0.61)
        return steering, a

    def TC_simulate(
        self, mode: List[str], init, time_bound, time_step, lane_map = None
    ) -> TraceType:
        time_bound = float(time_bound)
        num_points = int(np.ceil(time_bound / time_step))
        trace = np.zeros((num_points + 1, 1 + len(init)))
        trace[1:, 0] = [round(i * time_step, 10) for i in range(num_points)]
        trace[0, 1:] = init
        for i in range(num_points):
            steering, a = self.action_handler(mode, init)
            r = ode(self.dynamic)
            r.set_initial_value(init).set_f_params([steering, a])
            res: np.ndarray = r.integrate(r.t + time_step)
            init = res.flatten()
            if init[3] < 0:
                init[3] = 0
            trace[i + 1, 0] = time_step * (i + 1)
            trace[i + 1, 1:] = init
        return trace

def dist(pnt1, pnt2):
    return np.linalg.norm(
        np.array(pnt1) - np.array(pnt2)
    )

def get_extreme(rect1, rect2):
    lb11 = rect1[0]
    lb12 = rect1[1]
    ub11 = rect1[2]
    ub12 = rect1[3]

    lb21 = rect2[0]
    lb22 = rect2[1]
    ub21 = rect2[2]
    ub22 = rect2[3]

    # Using rect 2 as reference
    left = lb21 > ub11 
    right = ub21 < lb11 
    bottom = lb22 > ub12
    top = ub22 < lb12

    if top and left: 
        dist_min = dist((ub11, lb12),(lb21, ub22))
        dist_max = dist((lb11, ub12),(ub21, lb22))
    elif bottom and left:
        dist_min = dist((ub11, ub12),(lb21, lb22))
        dist_max = dist((lb11, lb12),(ub21, ub22))
    elif top and right:
        dist_min = dist((lb11, lb12), (ub21, ub22))
        dist_max = dist((ub11, ub12), (lb21, lb22))
    elif bottom and right:
        dist_min = dist((lb11, ub12),(ub21, lb22))
        dist_max = dist((ub11, lb12),(lb21, ub22))
    elif left:
        dist_min = lb21 - ub11 
        dist_max = np.sqrt((lb21 - ub11)**2 + max((ub22-lb12)**2, (ub12-lb22)**2))
    elif right: 
        dist_min = lb11 - ub21 
        dist_max = np.sqrt((lb21 - ub11)**2 + max((ub22-lb12)**2, (ub12-lb22)**2))
    elif top: 
        dist_min = lb12 - ub22
        dist_max = np.sqrt((ub12 - lb22)**2 + max((ub21-lb11)**2, (ub11-lb21)**2))
    elif bottom: 
        dist_min = lb22 - ub12 
        dist_max = np.sqrt((ub22 - lb12)**2 + max((ub21-lb11)**2, (ub11-lb21)**2)) 
    else: 
        dist_min = 0 
        dist_max = max(
            dist((lb11, lb12), (ub21, ub22)),
            dist((lb11, ub12), (ub21, lb22)),
            dist((ub11, lb12), (lb21, ub12)),
            dist((ub11, ub12), (lb21, lb22))
        )
    return dist_min, dist_max

class VehiclePedestrianSensor:
    def __init__(self):
        self.sensor_distance = 60

    # The baseline sensor is omniscient. Each agent can get the state of all other agents
    def sense(self, agent: BaseAgent, state_dict, lane_map):
        len_dict = {}
        cont = {}
        disc = {}
        len_dict = {"others": len(state_dict) - 1}
        tmp = np.array(list(state_dict.values())[0][0])
        if tmp.ndim < 2:
            if agent.id == 'car':
                len_dict['others'] = 1 
                cont['ego.x'] = state_dict['car'][0][1]
                cont['ego.y'] = state_dict['car'][0][2]
                cont['ego.theta'] = state_dict['car'][0][3]
                cont['ego.v'] = state_dict['car'][0][4]
                disc['ego.agent_mode'] = state_dict['car'][1][0]
                dist = np.sqrt(
                    (state_dict['car'][0][1]-state_dict['pedestrian'][0][1])**2+\
                    (state_dict['car'][0][2]-state_dict['pedestrian'][0][2])**2
                )
                # cont['ego.dist'] = dist
                if dist < self.sensor_distance:
                    cont['other.dist'] = dist
                    # cont['other.x'] = state_dict['pedestrian'][0][1]
                    # cont['other.y'] = state_dict['pedestrian'][0][2]
                    # cont['other.v'] = state_dict['pedestrian'][0][4]
                else:
                    cont['other.dist'] = 1000
                    # cont['other.x'] = 1000
                    # cont['other.y'] = 1000
                    # cont['other.v'] = 1000
        else:
            if agent.id == 'car':
                len_dict['others'] = 1 
                dist_min, dist_max = get_extreme(
                    (state_dict['car'][0][0][1],state_dict['car'][0][0][2],state_dict['car'][0][1][1],state_dict['car'][0][1][2]),
                    (state_dict['pedestrian'][0][0][1],state_dict['pedestrian'][0][0][2],state_dict['pedestrian'][0][1][1],state_dict['pedestrian'][0][1][2]),
                )
                cont['ego.x'] = [
                    state_dict['car'][0][0][1], state_dict['car'][0][1][1]
                ]
                cont['ego.y'] = [
                    state_dict['car'][0][0][2], state_dict['car'][0][1][2]
                ]
                cont['ego.theta'] = [
                    state_dict['car'][0][0][3], state_dict['car'][0][1][3]
                ]
                cont['ego.v'] = [
                    state_dict['car'][0][0][4], state_dict['car'][0][1][4]
                ]
                cont['other.dist'] = [
                    dist_min, dist_max
                ]
                disc['ego.agent_mode'] = state_dict['car'][1][0]
                if dist_min<self.sensor_distance:
                    cont['other.dist'] = [
                        dist_min, dist_max
                    ]
                    # cont['other.x'] = [
                    #     state_dict['pedestrian'][0][0][1], state_dict['pedestrian'][0][1][1]
                    # ]
                    # cont['other.y'] = [
                    #     state_dict['pedestrian'][0][0][2], state_dict['pedestrian'][0][1][2]
                    # ]
                    # cont['other.v'] = [
                    #     state_dict['pedestrian'][0][0][4], state_dict['pedestrian'][0][1][4]
                    # ]
                else:
                    cont['other.dist'] = [
                        1000, 1000
                    ]
                    # cont['other.x'] = [
                    #     1000, 1000
                    # ]
                    # cont['other.y'] = [
                    #     1000, 1000
                    # ]
                    # cont['other.v'] = [
                    #     1000, 1000
                    # ]


        return cont, disc, len_dict

def sample_init(scenario: Scenario, num_sample=50):
    """
    TODO:   given the initial set,
            generate multiple initial points located in the initial set
            as the input of multiple simulation.
            note that output should be formatted correctly and every point should be in inital set.
            refer the following sample code to write your code. 
    """
    init_dict = scenario.init_dict
    print(init_dict)
    ############## Your Code Start Here ##############
    sample_dict_list = []

    np.random.seed(2023)
    for i in range(num_sample):
        sample_dict={}
        for agent in init_dict:
            point = np.random.uniform(init_dict[agent][0], init_dict[agent][1]).tolist()
            sample_dict[agent] = point
        sample_dict_list.append(sample_dict)
    ############## Your Code End Here ##############
    print(sample_dict_list)

    return sample_dict_list

def eval_velocity(tree_list: List[AnalysisTree]):
    agent_id = 'car'
    velo_list = []
    unsafe_init = []
    for tree in tree_list:
        assert agent_id in tree.root.init
        leaves = list(filter(lambda node: node.child == [], tree.nodes))
        unsafe = list(filter(lambda node: node.assert_hits != None, leaves))
        if len(unsafe) != 0:
            print(f"unsafety detected in tree with init {tree.root.init}")
            unsafe_init.append(tree.root.init)
        else:
            safe = np.array(list(filter(lambda node: node.assert_hits == None, leaves)))
            init_x = tree.root.init[agent_id][0]
            last_xs = np.array([node.trace[agent_id][-1][1] for node in safe])
            time = round(safe[0].trace[agent_id][-1][0], 3)
            velos = (last_xs-init_x)/time
            max_velo = np.max(velos)
            velo_list.append(max_velo)
            print(f"max avg velocoty {max_velo} in tree with init {tree.root.init}")
    if len(tree_list) == len(velo_list):
        print(f"No unsafety detected! Overall average velocity is {sum(velo_list)/len(velo_list)}.")
        return {sum(velo_list)/len(velo_list)}, 0, []
    else:
        print(f"Unsafety detected! Please update your DL.")
        return {sum(velo_list)/len(tree_list)}, float(len(unsafe_init))/float(len(tree_list)), unsafe_init

def combine_tree(tree_list: List[AnalysisTree]):
    combined_trace={}
    for tree in tree_list:
        for node in tree.nodes:
            for agent_id in node.agent:
                traces = node.trace
                trace = np.array(traces[agent_id])
                if agent_id not in combined_trace:
                    combined_trace[agent_id]={}
                for i in range (0, len(trace), 2):
                    step = round(trace[i][0], 3)
                    if step not in combined_trace[agent_id]:
                        combined_trace[agent_id][step]=[trace[i], trace[i+1]]
                    else:
                        lower = np.min([combined_trace[agent_id][step][0],trace[i]], 0)
                        upper = np.max([combined_trace[agent_id][step][1],trace[i+1]], 0)
                        combined_trace[agent_id][step]=[lower, upper]

    final_trace = {agent_id:np.array(list(combined_trace[agent_id].values())).flatten().reshape((-1, trace[i].size)).tolist() for agent_id in combined_trace}
    root = AnalysisTreeNode(final_trace,None,None,None,None, node.agent, None,None,[],0,10,AnalysisTreeNodeType.REACH_TUBE,0)
    return AnalysisTree(root)
