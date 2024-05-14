import rclpy
from mrs_main.agent.agent import Agent


def main(args=None):
    rclpy.init()

    agent = Agent(agent_number='123')
    agent.start_agent()


if __name__ == '__main__':
    main()