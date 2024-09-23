import rclpy
import sys
from mrs_main.agent.agent import Agent


def main(args=None):
    rclpy.init()
    intrest_exec = 0.2
    intrest_coord = 0.2
    if (len(sys.argv)>1):
        intrest_exec = float(sys.argv[1])
    if (len(sys.argv)>2):
        intrest_coord = float(sys.argv[2])
        
    agent = Agent(intrest_exec=intrest_exec, intrest_coord=intrest_coord)

    try:
        agent.start_agent()
    except KeyboardInterrupt:
        print("Shutting down agent...")
        agent.print_task_states()
        raise KeyboardInterrupt



if __name__ == '__main__':
    main()