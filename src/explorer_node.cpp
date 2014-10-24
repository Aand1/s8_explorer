#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_common_node/Node.h>

#include <s8_turner/TurnAction.h>


#define NODE_NAME		"s8_explorer_node"

#define ACTION_TURN		"/s8/turn"

class Explorer : public s8::Node {
	actionlib::SimpleActionClient<s8_turner::TurnAction> turn_action;


public:
	Explorer() : turn_action(ACTION_TURN, true) {
        ROS_INFO("Waiting for turn action server...");
        turn_action.waitForServer();
        ROS_INFO("Connected to turn action server!");

        turn(90);
	}

	void turn(int degrees) {
		s8_turner::TurnGoal goal;
		goal.degrees = degrees;
		turn_action.sendGoal(goal);


        bool finised_before_timeout = turn_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = turn_action.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            	ROS_INFO("Turn action succeeded. Degrees turn: %d", turn_action.getResult()->degrees);
            } else {
            	ROS_INFO("Turn action finished with unknown state: %s", state.toString().c_str());
            }
        } else {
            ROS_WARN("Turn action timed out.");
        }
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);
	Explorer explorer;
	ros::spin();
	return 0;
}