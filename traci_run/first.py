import os
import sys
import random
import optparse

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("environment variable SUMO_HOME not declared")

from sumolib import checkBinary     
import traci 

def generate_route_file():
    random.seed(42)
    N = 3000 ### Number of timesteps
    pWE = 1 / 10
    pEW = 1 / 10
    pNS = 1 / 40
    with open("first.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="car" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="5" maxSpeed="16.67" guiShape="passenger"/> 
        
        <route id="right" edges="1i 2o"/>
        <route id="left" edges="2i 1o"/>
        <route id="down" edges="4i 3o"/>
        """, file=routes)

        veh_num = 0 ### records each vehicle to get unique value
        for i in range(N): ###Each timestep, max 3 cars generated on each lane 
            if random.uniform(0, 1) < pWE:
                print('     <vehicle id="right_%i" type="car" route="right" depart="%i" color="1,0,0"/>' %(veh_num, i), file=routes)
                veh_num += 1
            # if random.uniform(0, 1) < pEW:
            #     print('     <vehicle id="left_%i" type="car" route="left" depart="%i" color="0,1,0"/>' %(veh_num, i), file=routes)
            #     veh_num += 1
            if random.uniform(0, 1) < pNS:
                print('     <vehicle id="down_%i" type="car" route="down" depart="%i" color="0,0,1"/>' %(veh_num, i), file=routes)
                veh_num += 1
        print("</routes>", file=routes)

def run():
    """executing TraCI control loop, traffic light control"""
    step = 0
    theta = 8
    Mu = 3
    Ki_1, Ki_2 = 0, 0
    Oi_1, Oi_2 = 0, 0
    ##starting by keeping lane red
    traci.trafficlight.setPhase("0", 0)
    while traci.simulation.getMinExpectedNumber() > 0:
        """bounding the simulation control on the basis of number of vehicles left"""
        traci.simulationStep() ##not sure why we are forcing traci to take a simulation step here
        if traci.trafficlight.getPhase("0") == 0: ##first check phase is red
            Ki_1 += traci.inductionloop.getLastStepVehicleNumber("det_1i")
            Ki_2 += traci.inductionloop.getLastStepVehicleNumber("det_2i")

            Oi_1 += traci.inductionloop.getLastStepVehicleNumber("det_4i")
            if Ki_1 >= theta:
                if Oi_1 == 0:
                    traci.trafficlight.setPhase("0", 1) ## setting to yellow
                    ### reset induction loop value or increment theta by 10
                    Ki_1 = 0
                if Oi_1 < Mu:
                    traci.trafficlight.setPhase("0", 0)
                else:
                    traci.trafficlight.setPhase("0", 1) ## setting to yellow
                    Ki_1 = 0
                    Oi_1 = 0
                    # Mu = Oi_1 + 3
            else:
                traci.trafficlight.setPhase("0", 0) ##Maintain red light
        step += 1
    traci.close()
    sys.stdout.flush()

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

#main etry point of scipt

if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    #first, generate route file
    generate_route_file()

    #start simulation
    traci.start([sumoBinary, "-c", "first.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    run()


# Pns, Pew = 0, 0
# random.seed(42)
# for i in range(20):
#     u = random.uniform(0, 1)
#     if u < 1 / 40 :
#         Pns += 1
#     if u < 1 / 11 :
#         Pew += 1
# print(Pns / 20, Pew / 20)