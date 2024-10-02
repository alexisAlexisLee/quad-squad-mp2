from wsgiref.validate import PartialIteratorWrapper
from mp0 import VehicleAgent, PedestrianAgent, VehiclePedestrianSensor, verify_refine, eval_velocity, sample_init
from verse import Scenario, ScenarioConfig
from vehicle_controller import VehicleMode, PedestrianMode

from verse.plotter.plotter2D import *
from verse.plotter.plotter3D_new import *
import plotly.graph_objects as go
import copy

if __name__ == "__main__":
    import os 
    script_dir = os.path.realpath(os.path.dirname(__file__))
    input_code_name = os.path.join(script_dir, "vehicle_controller.py")
    vehicle = VehicleAgent('car', file_name=input_code_name)
    pedestrian = PedestrianAgent('pedestrian')

    scenario = Scenario(ScenarioConfig(init_seg_length=1, parallel=False))

    scenario.add_agent(vehicle) 
    scenario.add_agent(pedestrian)
    scenario.set_sensor(VehiclePedestrianSensor())

    # # ----------- Different initial ranges -------------
    # # Uncomment this block to use R1
    init_car = [[-5,-5,0,8],[5,5,0,8]]
    init_pedestrian = [[175,-55,0,3],[175,-55,0,3]]
    # # -----------------------------------------

    # # Uncomment this block to use R2
    # init_car = [[-5,-5,0,5],[5,5,0,10]]
    # init_pedestrian = [[175,-55,0,3],[175,-55,0,3]]
    # # -----------------------------------------

    # # Uncomment this block to use R3
    # init_car = [[-5,-5,0,5],[5,5,0,10]]
    # init_pedestrian = [[165,-55,0,3],[175,-50,0,3]]
    # # -----------------------------------------

    scenario.set_init_single(
        'car', init_car,(VehicleMode.Normal,)
    )
    scenario.set_init_single(
        'pedestrian', init_pedestrian, (PedestrianMode.Normal,)
    )

    # # ----------- Simulate single: Uncomment this block to perform single simulation -------------
    # trace = scenario.simulate(50, 0.1)
    # fig = go.Figure()
    # fig = simulation_tree_3d(trace, fig,\
    #                           0,'time', 1,'x',2,'y')
    # fig.show()
    # -----------------------------------------

    # # ----------- Simulate multi: Uncomment this block to perform batch of simulations -------------
    init_dict_list= sample_init(scenario, num_sample=50)
    traces = scenario.simulate_multi(50, 0.1,\
         init_dict_list=init_dict_list)
    fig = go.Figure()
    for trace in traces:
        fig = simulation_tree_3d(trace, fig,\
                                  0,'time', 1,'x',2,'y')
    fig.show()
    avg_vel, unsafe_frac, unsafe_init = eval_velocity(traces)
    print(f"###### Average velocity {avg_vel}, Unsafe fraction {unsafe_frac}, Unsafe init {unsafe_init}")
    # # -----------------------------------------

    # # ------------- Verify refine: Uncomment this block to perform verification with refinement -------------
    traces, safe, unsafe = verify_refine(scenario, 50, 0.1)
    fig = go.Figure()
    fig = reachtube_tree_3d(traces, fig,\
                             0,'time', 1,'x',2,'y')
    fig.show()
    # # -----------------------------------------

    # # ----------- verify no refine: Uncomment this block to perform verification without refinement ----------
    # traces = scenario.verify(30, 1)
    # fig = go.Figure()
    # fig = reachtube_tree_3d(traces, fig,\
    #                          0,'time', 1,'x',2,'y')
    # fig.show()
    # # -----------------------------------------
