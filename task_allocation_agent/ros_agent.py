import sys
import threading

import geojson
import numpy as np
import rclpy
import shapely
import shapely.affinity
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

sys.path.append("../trajectory-task-allocation")
from task_allocation import ACBBA, Agent, CoverageProblem, Utility


class RosAgent(Node):
    def __init__(self, agent: Agent, tasks: list = []):
        self.name = "Agent" + str(agent.id)
        super().__init__(self.name)

        # Agent communication
        self.bid_info_publisher = self.create_publisher(String, "bid_info", 10)
        self.bid_info_listener = self.create_subscription(String, "bid_info", callback=self.listener, qos_profile=10)

        # Publish the tasklist, which initiates the consensus
        self.task_info_listener = self.create_subscription(String, "task_info", callback=self.task_listener, qos_profile=10)

        # Internal message buffer
        self.message_buffer = {}

        self.agent = ACBBA.agent(
            id=agent.id,
            state=shapely.Point(agent.position),
            tasks=np.array(tasks),
            capacity=agent.capacity,
        )
        # If tasks are predefined calculate bundle and communicate
        if tasks is not None:
            # Create a timer with a 0-second delay to start the callback immediately
            self.one_shot_timer = self.create_timer(0.0, self.bundle_builder)

    def listener(self, message):
        """
        Deconflict the messages recieved by other agents ()
        TaskBidInfo: Sender ID, Task ID, Winnning Agent ID, Winning agent score, timestamp for last update
        The listener fills one of two buffers, the bundle builder then switches between these two, to avoid overwriting
        """

        self.get_logger().info("test")
        # TODO do not listen to messages from the node itself
        # Update the recieved message: robot.receive_message(Y)
        # Perform consensus: robot.update_task()

        # TODO if the bundle is already building save the message to a buffer
        # The buffer should only save the most recent message from each agent
        if self.message_buffer.get(message.sender_id).build_time < message.build_time:
            self.message_buffer[message.sender_id] = message.bid_info
        if not bundle_is_building:
            self.bundle_builder()

    def task_listener(self, message):
        """Listens for tasks to generate a bundle

        Parameters
        ----------
        message : TaskList
            A complete tasklist containing both patial features, priority, time restrictions,
            simply all information needed to build a bundle
        """
        pass

    def bundle_builder(self):
        """Builds a bundle based on the buffer received by the listener and send a message with the tasks and bid list"""
        # TODO only build a bundle if there is new information in the buffer

        if self.one_shot_timer is not None:
            self.one_shot_timer.destroy()
            self.one_shot_timer = None

        # Build the bundle using the newest state from the listener: robot.build_bundle()
        self.agent.build_bundle()
        self.get_logger().info(str(self.agent.getBundle()))

        # After building, send the bid info: winning bids, winning agents, timestamps of when the bids were placed
        msg = String()
        self.bid_info_publisher.publish(msg)


def load_coverage_problem():
    files = Utility.getAllCoverageFiles("AC300")
    for file_name in files:
        with open(file_name) as json_file:
            features = geojson.load(json_file)["features"]

        geometries = {
            "obstacles": shapely.MultiPolygon(),
            "tasks": shapely.MultiLineString(),
            "boundary": shapely.Polygon(),
        }

        for feature in features:
            if feature["geometry"]:
                geometries[feature["id"]] = shapely.geometry.shape(feature["geometry"])
        number_of_tasks = len(list(geometries["tasks"].geoms))

        print(file_name, " Tasks: ", number_of_tasks)
        # Initialize coverage problem and the agents
        geometries["boundary"] = shapely.affinity.scale(geometries["boundary"], xfact=1.01, yfact=1.01)

        # Scale each polygon in the MultiPolygon
        scaled_polygons = []
        for polygon in geometries["obstacles"].geoms:
            scaled_polygon = shapely.affinity.scale(polygon, xfact=0.95, yfact=0.95, origin="centroid")
            scaled_polygons.append(scaled_polygon)

        # Create a new MultiPolygon with scaled polygons
        scaled_multi_polygon = shapely.geometry.MultiPolygon(scaled_polygons)

        cp = CoverageProblem.CoverageProblem(restricted_areas=scaled_multi_polygon, search_area=geometries["boundary"], tasks=geometries["tasks"])
        return cp


def main(
    args=None,
):
    seed = 1352323
    np.random.seed(seed)

    rclpy.init(args=args)
    cp = load_coverage_problem()

    tasks = cp.getTasks()
    n_agents = 3
    executor = MultiThreadedExecutor()
    for id in range(n_agents):
        node = RosAgent(Agent.agent(id, cp.generate_random_point_in_problem(), 1000), tasks)
        executor.add_node(node)

    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
