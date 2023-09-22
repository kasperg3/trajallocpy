import sys

import numpy as np
import rclpy
import shapely
from rclpy.node import Node
from std_msgs.msg import String

sys.path.append('../trajectory-task-allocation')
from task_allocation import ACBBA, Agent, CoverageProblem


class RosAgent(Node):
    def __init__(self, id: int):
        self.name = "Agent" + str(id)
        super().__init__(self.name)
        # External communication
        self.bid_info_publisher = self.create_publisher(String, "bid_info", 10)
        self.bid_info_listener = self.create_subscription(String, "bid_info", callback=self.consensus, qos_profile=10)

        # Internal message buffer
        self.message_buffer = []

        # Bundle builder
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.bundle_builder)
        self.i = 0

    def consensus(self, message):
        """
        Deconflict the messages recieved by other agents ()
        TaskBidInfo: Sender ID, Task ID, Winnning Agent ID, Winning agent score, timestamp for last update
        The listener fills one of two buffers, the bundle builder then switches between these two, to avoid overwriting
        """
        # TODO do not listen to messages from the node itself
        # Update the recieved message: robot.receive_message(Y)
        # Perform consensus: robot.update_task()

        pass

    def bundle_builder(self):
        """Builds a bundle based on the buffer received by the listener and send a message with the tasks and bid list"""
        # TODO only build a bundle if there is new information in the buffer
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.bid_info_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        # Build the bundle using the newest state from the listener: robot.build_bundle()

        # After building, send the bid info: winning bids, winning agents, timestamps of when the bids were placed
        # self.bid_info_publisher(agent.send_message())


def main(coverage_problem: CoverageProblem.CoverageProblem, agents: list[Agent.agent], enable_plotting=False, args=None):
    # Load the problem
    robot_list = []
    # Instanciate the agents
    for agent in agents:
        robot_list.append(
            ACBBA.agent(
                id=agent.id,
                state=shapely.Point(agent.position),
                environment=coverage_problem.environment,
                tasks=np.array(coverage_problem.getTasks()),
                number_of_agents=len(agents),
                capacity=agent.capacity,
            )
        )

    rclpy.init(args=args)
    agent = RosAgent(id=1)
    rclpy.spin(agent)

    agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
