import sys
import threading
from typing import List

import geojson
import numpy as np
import rclpy
import shapely
import shapely.affinity
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from task_allocation_interfaces.msg import BidInfo, BidInfoList, TaskInfo

sys.path.append("../trajectory-task-allocation")
from task_allocation import ACBBA, Agent, CoverageProblem, Utility


def fromBidInfoMessage(bid: BidInfo):
    return ACBBA.BidInformation(y=bid.winning_score, z=bid.winning_agent, t=bid.timestamp, j=bid.task_id, k=bid.sender_id)


def fromBidInfoListMessage(msg: BidInfoList):
    bid_list = []
    for bid in msg.bids:
        bid_list.append(fromBidInfoMessage(bid))
    return bid_list


def toBidInfoMessage(bid: ACBBA.BidInformation):
    return BidInfo(winning_agent=bid.z, task_id=bid.j, timestamp=float(bid.t), sender_id=bid.k)


def toBidInfoListMessage(bids: List[ACBBA.BidInformation], agent_id) -> BidInfoList:
    bid_list = []
    for bid in bids:
        bid_list.append(toBidInfoMessage(bid))
    return BidInfoList(bids=bid_list, agent_id=agent_id)


class MessageBuffer:
    def __init__(self):
        self.buffer = {}
        self.lock = threading.Lock()

    def add_messages(self, messages: BidInfoList):
        for message in messages.bids:
            self.add_message(message)

    def add_message(self, message: BidInfo):
        with self.lock:
            if message.task_id in self.buffer:
                if message.timestamp > self.buffer[message.task_id].timestamp:
                    self.buffer[message.task_id] = message
            else:
                self.buffer[message.task_id] = message

    def is_empty(self):
        with self.lock:
            return len(self.buffer) == 0

    def get(self, task_id):
        with self.lock:
            return self.buffer.get(task_id, None)

    def get_all(self):
        with self.lock:
            result = list(self.buffer.values())
            self.buffer.clear()
            return result

    def __str__(self):
        with self.lock:
            return str(self.buffer)


class RosAgent(Node):
    def __init__(self, agent: Agent.config, tasks: list = []):
        self.name = "Agent" + str(agent.id)
        super().__init__(self.name)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Agent communication
        self.bid_info_publisher = self.create_publisher(BidInfoList, "bid_info", 10)
        self.bid_info_listener = self.create_subscription(BidInfoList, "bid_info", callback=self.listener, qos_profile=10)

        # Publish the tasklist, which initiates the consensus
        self.task_info_listener = self.create_subscription(TaskInfo, "task_info", callback=self.task_listener, qos_profile=10)

        # Internal message buffer
        # the message buffer is a hash table where only the most recent information is stored for each task
        self.incomming_buffer = MessageBuffer()
        self.outgoing_buffer = MessageBuffer()

        self.agent = ACBBA.agent(
            id=agent.id,
            state=shapely.Point(agent.position),
            tasks=np.array(tasks),
            capacity=agent.capacity,
        )
        # If tasks are predefined calculate bundle and communicate
        self.bundle_timer = self.create_timer(1, self.bundle_builder)

    def listener(self, messages: BidInfoList):
        """
        Deconflict the messages recieved by other agents ()
        TaskBidInfo: Sender ID, Task ID, Winnning Agent ID, Winning agent score, timestamp for last update
        The listener fills one of two buffers, the bundle builder then switches between these two, to avoid overwriting
        """
        # Do not listen to messages from self
        if messages.agent_id == self.get_name():
            return

        self.get_logger().debug("recieved message from agent: " + messages.agent_id)
        self.incomming_buffer.add_messages(messages)

    def task_listener(self, message):
        """Listens for tasks to generate a bundle

        Parameters
        ----------
        message : TaskList
            A complete tasklist containing both patial features, priority, time restrictions,
            simply all information needed to build a bundle
        """
        # TODO Update the tasklist and rebuild the bundle

        pass

    def bundle_builder(self):
        """Builds a bundle based on the buffer received by the listener and send a message with the tasks and bid list"""
        # Only build a bundle if there is new information in the buffer
        if self.incomming_buffer.is_empty() and len(self.agent.bundle) != 0:
            return
        # perform consensus to determine which messages has to be rebroadcasted
        incomming_messages = BidInfoList(bids=self.incomming_buffer.get_all(), agent_id=self.get_name())
        rebroadcasts = self.agent.update_task_async(fromBidInfoListMessage(incomming_messages))
        self.get_logger().debug("Number of rebroadcasts: " + str(len(rebroadcasts)))
        self.outgoing_buffer.add_messages(toBidInfoListMessage(rebroadcasts, self.get_name()))

        # Build the bundle using the newest state from the listener
        bids = self.agent.build_bundle()
        self.get_logger().info(str(self.agent.bundle))
        self.outgoing_buffer.add_messages(toBidInfoListMessage(bids, self.get_name()))

        # publish the new bids from the buffer, overwriting any potential rebroadcasts with new information
        if not self.outgoing_buffer.is_empty():
            outgoing_messages = self.outgoing_buffer.get_all()
            self.get_logger().debug("Number of outgoing messages: " + str(len(outgoing_messages)))
            self.bid_info_publisher.publish(BidInfoList(bids=outgoing_messages, agent_id=self.get_name()))


def load_coverage_problem() -> CoverageProblem.CoverageProblem:
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
    n_agents = 2
    executor = MultiThreadedExecutor()
    for id in range(n_agents):
        node = RosAgent(Agent.config(id, cp.generate_random_point_in_problem(), 1000), tasks)
        executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
