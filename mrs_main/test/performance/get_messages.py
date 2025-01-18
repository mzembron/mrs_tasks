import rclpy
import sys
from mrs_main.agent.agent import Agent
import multiprocessing

def start_agent(agent_type):
    rclpy.init()
    agent = Agent(agent_type=agent_type)

    try:
        agent.start_agent()
    except KeyboardInterrupt:
        print(f"Shutting down agent {agent_type}...")
        agent.print_task_states()
        raise KeyboardInterrupt

def main(args=None):
    processes = []
    agent_type = 0
    for i in range(5):
        if i != 0:
            agent_type = 1
        p = multiprocessing.Process(target=start_agent, args=(agent_type,))
        processes.append(p)
        p.start()

    for p in processes:
        p.join()

if __name__ == '__main__':
    main()