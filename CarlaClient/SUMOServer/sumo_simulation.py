import os
import sys
import subprocess

config_file_path = ''

def sumo_init():
    if 'SUMO_HOME' in os.environ:
        tools =os.path.join(os.environ['SUMO_HOME'], 'tools')
        print("设置的SUMO路径为" + tools)
        sys.path.append(tools)
        print('sumo path set done')
    else:  
        sys.exit("please declare environmentvariable 'SUMO_HOME'")
    # sumoProcess = subprocess.Popen([sumoBinary, "-c", "./test.sumocfg", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)




if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) != 1:
        print('Usage: python ' + sys.argv[0] + ' cfg_file')
        exit(-1)
    config_file_path = args[0]
    print("path: " + config_file_path)
    sumo_init()
    
    import traci
    import traci.constants as tc
    print("traci import done")
    PORT = 8813
    sumoBinary = 'sumo'
    sumoCmd = [sumoBinary, "-c", config_file_path]
    vehID = "0"
    traci.start(sumoCmd)
    traci.vehicle.subscribe(vehID, (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
    print(traci.vehicle.getSubscriptionResults(vehID))
    for step in range(3):
        print("step", step)
        traci.simulationStep()
        print(traci.vehicle.getSubscriptionResults(vehID))
    traci.close()
    # traci.vehicle.subscribe("0", (tc.VAR_SPEED, tc.VAR_ACCEL, tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_POSITION))
    # 一直执行仿真直到所有车辆到达目的地
    # traci.vehicle.moveToXY(vehID="0", edgeID="1[1][0]", lane=1, x=563.869048, y=456, keepRoute=0)
    i = 0
    # while traci.simulation.getMinExpectedNumber() > 0:
    #     traci.simulationStep()
        # if i < 100:
        #     for veh_id in traci.vehicle.getIDList():
        #         speed = traci.vehicle.getSpeed(veh_id)
        #         [x, y] = traci.vehicle.getPosition(veh_id)
        #         print(veh_id, ':', speed, ":[", x, ",", y, "]")

        #         i += 1
    
    # step = 0
    # while step < 1000:
    #     traci.simulationStep()
    #     # print(traci.vehicle.getSubscriptionResults(vehID))
    #     step += 1
    # traci.close()


# traci.start(["sumo", "-c", "my.sumocfg"]) 
# traci.vehicle.subscribe(vehID, (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
# print(traci.vehicle.getSubscriptionResults(vehID))
# for step in range(3):
#    print("step", step)
#    traci.simulationStep()
#    print(traci.vehicle.getSubscriptionResults(vehID))
# traci.close()