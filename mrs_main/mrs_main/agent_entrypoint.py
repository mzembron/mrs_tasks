import rclpy
import sys
from mrs_main.agent.agent import Agent


def main(args=None):
    rclpy.init()
    agent_type = 1
    agent_name = 'tb1'
    if (len(sys.argv)>1):
        agent_type = int(sys.argv[1])
    if (len(sys.argv)>2):
        agent_name = str(sys.argv[2])
        
    agent = Agent(agent_type=agent_type, agent_name=agent_name)

    try:
        agent.start_agent()
    except KeyboardInterrupt:
        print("Shutting down agent...")
        agent.print_task_states()
        raise KeyboardInterrupt



if __name__ == '__main__':
    main()