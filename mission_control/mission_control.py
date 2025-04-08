import rclpy
from rclpy.node import Node

# service for control of exploration node
from custom_msg_srv.msg import MapExplored

# for control of casualty_location node
from custom_msg_srv.srv import StartCasualtyLocation

# for updating when all casualties found
from custom_msg_srv.msg import CasualtyLocateStatus


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.get_logger().info("Mission Control Node Started")

        # service to set casualty_locate or casualty_save to start
        self.exploration_service = self.create_client(StartCasualtyLocation, 'casualty_state')
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
        self.create_timer(1.0, self.run_fsm())


    def set_casualty_service(self, state):
        self.req = StartExploration.Request()
        self.req.state = state
        while not self.exploration_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('explorer not avail, waiting...')
        self.exploration_service.call_async(self.req)

    def exploration_callback(self, msg):
        self.mapExplored = msg.explore_complete

    def casualty_callback(self, msg):
        self.casualties_found = msg.all_casualties_found

    def casualty_save_callback(self, msg):
        self.casualties_saved = msg.all_casualties_saved



    # FSM: finite state machine
    def run_fsm(self):


        if self.state == 'explore':
            self.prevState = self.state
            # wait for exploration to finish
            if self.mapExplored:
                self.get_logger().info("exploration complete")
                self.state = 'locate casualty'



        elif self.state == 'locate casualty':
            # ensure that the service is called only once
            if self.prevState != 'locate casualty':
                self.set_casualty_service("LOCATE")
                self.prevState = self.state

            if self.casualties_found:
                self.get_logger().info("all casualties found")
                self.state = 'save casualty'



        elif self.state == 'save casualty':
            # ensure service only called once
            if self.prevState != 'save casualty':
                self.set_casualty_service("SAVE")
                self.prevState = self.state

            if self.casualties_saved:
                self.set_casualty_service("STOPPED")
                self.state = 'mission complete'



        elif self.state == 'mission complete':
            if self.prevState != 'mission complete':
                self.get_logger().info("Mission Completed")
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