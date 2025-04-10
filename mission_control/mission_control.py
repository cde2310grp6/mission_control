import rclpy
from rclpy.node import Node

# service for control of exploration node
from custom_msg_srv.msg import MapExplored

# for control of casualty_location node
from custom_msg_srv.srv import StartCasualtyService

# for updating when all casualties found
from custom_msg_srv.msg import CasualtyLocateStatus

# for updating when all casualties saved
from custom_msg_srv.msg import CasualtySaveStatus


RESET = "\033[0m"
GREEN = "\033[92m"


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.get_logger().info("Mission Control Node Started")

        # service to set casualty_locate or casualty_save to start
        self.exploration_service = self.create_client(StartCasualtyService, 'casualty_state')
        self.set_casualty_service("STOPPED")

        # topic to tell when nav2_wfd explorer exploration complete
        # we assume that the exploration node will start by default when the launch file is run
        self.exploration_topic = self.create_subscription(MapExplored, 'exploration_complete', self.exploration_callback, 10)
        self.mapExplored = False

        # topic to tell when casualty_location is done
        self.casualty_locate = self.create_subscription(CasualtyLocateStatus, 'casualty_found', self.casualty_callback, 10)
        self.casualties_found = False

        # topic to tell when all casualty_save is complete
        self.casualty_save = self.create_subscription(CasualtySaveStatus, 'casualty_saved', self.casualty_save_callback, 10)
        self.casualties_saved = False

        # prevState to prevent excessive calling of the same state actions
        self.state = "explore"
        self.prevState = None


        # timer to run the FSM
        self.create_timer(1.0, self.run_fsm)


    def set_casualty_service(self, state):
        req = StartCasualtyService.Request()
        req.state = state

        if not self.exploration_service.service_is_ready():
            self.get_logger().warning('Explorer service not available. Retrying...')
            self.exploration_service.wait_for_service(timeout_sec=5.0)

        if self.exploration_service.service_is_ready():
            self.exploration_service.call_async(req)
        else:
            self.get_logger().error('Explorer service unavailable after timeout.')

    def exploration_callback(self, msg):
        self.mapExplored = msg.explore_complete

    def casualty_callback(self, msg):
        self.casualties_found = msg.all_casualties_found

    def casualty_save_callback(self, msg):
        self.casualties_saved = msg.all_casualties_saved


    # FSM: finite state machine
    def run_fsm(self):


        if self.state == 'explore':
            if self.prevState != 'explore':
                self.get_logger().info(f"{GREEN}exploring map...{RESET}")
                self.prevState = self.state
            # wait for exploration to finish
            if self.mapExplored:
                self.get_logger().info(f"{GREEN}exploration complete{RESET}")
                self.state = 'locate casualty'



        elif self.state == 'locate casualty':
            # ensure that the service is called only once
            if self.prevState != 'locate casualty':
                self.set_casualty_service("LOCATE")
                self.prevState = self.state

            if self.casualties_found:
                self.get_logger().info(f"{GREEN}all casualties found{RESET}")
                self.state = 'save casualty'



        elif self.state == 'save casualty':
            # ensure service only called once
            if self.prevState != 'save casualty':
                self.get_logger().info(f"{GREEN}all casualties found{RESET}")
                self.set_casualty_service("SAVE")
                self.prevState = self.state

            if self.casualties_saved:
                self.set_casualty_service("STOPPED")
                self.state = 'mission complete'



        elif self.state == 'mission complete':
            if self.prevState != 'mission complete':
                self.get_logger().info(f"{GREEN}Mission Completed{RESET}")
                self.prevState = self.state
                # stop the mission control node 

        
            

def main():
    rclpy.init()
    mission_control = MissionControl()

    try:
        rclpy.spin(mission_control)
    except KeyboardInterrupt:
        pass
    finally:
        mission_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()