#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_common_node/Node.h>

#include <s8_turner/TurnAction.h>
#include <s8_msgs/IRDistances.h>

#define NODE_NAME		"s8_explorer_node"

#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR "front_distance_treshold_near"
#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR  "front_distance_treshold_far"
#define PARAM_NAME_FRONT_DISTANCE_STOP          "front_distance_stop"

#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR  0.10
#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR   0.4
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP           0.18


#define TOPIC_DISTANCES                             "/s8/ir_distances"
#define ACTION_TURN		                            "/s8/turn"

class Explorer : public s8::Node {
    ros::Subscriber distances_subscriber;
	actionlib::SimpleActionClient<s8_turner::TurnAction> turn_action;
    double front_distance_treshold_near;
    double front_distance_treshold_far;
    double front_distance_stop;


public:
	Explorer() : turn_action(ACTION_TURN, true) {
        init_params();
        print_params();
        distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_DISTANCES, 0, &Explorer::distances_callback, this);

        ROS_INFO("Waiting for turn action server...");
        turn_action.waitForServer();
        ROS_INFO("Connected to turn action server!");
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

private:
    void stop() {

    }

    void distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        double front_left = ir_distances->front_left;
        double front_right = ir_distances->front_right;

        auto inside_treshold = [this] (double value) {
            return value > front_distance_treshold_near && value < front_distance_treshold_far;
        };

        if(inside_treshold(front_left) || inside_treshold(front_right)) {
            //There is a wall in front of robot.
            ROS_INFO("Wall ahead! left: %.2lf, right: %.2lf", front_left, front_right);

            if(front_left <= front_distance_stop || front_right <= front_distance_stop) {
                ROS_INFO("Too close to wall! Stopping...");
                stop();
            }
        }
    }

    void init_params() {
        add_param(PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR, front_distance_treshold_near, PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR);
        add_param(PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR, front_distance_treshold_far, PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR);
        add_param(PARAM_NAME_FRONT_DISTANCE_STOP, front_distance_stop, PARAM_DEFAULT_FRONT_DISTANCE_STOP);
    }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);
	Explorer explorer;
	ros::spin();
	return 0;
}