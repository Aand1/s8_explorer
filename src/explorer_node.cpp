#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_common_node/Node.h>

#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>
#include <s8_msgs/IRDistances.h>

#define NODE_NAME       "s8_explorer_node"

#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR "front_distance_treshold_near"
#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR  "front_distance_treshold_far"
#define PARAM_NAME_FRONT_DISTANCE_STOP          "front_distance_stop"

#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR  0.10
#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR   0.4
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP           0.18


#define TOPIC_DISTANCES                             "/s8/ir_distances"
#define ACTION_TURN                                 "/s8/turn"
#define ACTION_STOP                                 "/s8_motor_controller/stop"
#define ACTION_FOLLOW_WALL                          "/s8/follow_wall"

typedef actionlib::SimpleActionClient<s8_wall_follower_controller::FollowWallAction> follow_wall_client;

class Explorer : public s8::Node {
    ros::Subscriber distances_subscriber;
    actionlib::SimpleActionClient<s8_turner::TurnAction> turn_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;
    follow_wall_client follow_wall_action;
    double front_distance_treshold_near;
    double front_distance_treshold_far;
    double front_distance_stop;
    bool turning;

public:
    Explorer() : turning(false), turn_action(ACTION_TURN, true), stop_action(ACTION_STOP, true), follow_wall_action(ACTION_FOLLOW_WALL, true) {
        init_params();
        print_params();
        distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_DISTANCES, 0, &Explorer::distances_callback, this);

        ROS_INFO("Waiting for turn action server...");
        turn_action.waitForServer();
        ROS_INFO("Connected to turn action server!");

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");

        ROS_INFO("Waiting for follow_wall action server...");
        follow_wall_action.waitForServer();
        ROS_INFO("Connected to follow_wall action server!");

        follow_wall(1);
        //turn(90);
    }

private:
    void follow_wall(int side) {
        ROS_INFO("Following %s wall...", side == -1 ? "left" : "right");

        s8_wall_follower_controller::FollowWallGoal goal;
        goal.wall_to_follow = side;
        follow_wall_action.sendGoal(goal, boost::bind(&Explorer::follow_wall_done_callback, this, _1, _2), follow_wall_client::SimpleActiveCallback(), follow_wall_client::SimpleFeedbackCallback());

        /*bool finised_before_timeout = follow_wall_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = follow_wall_action.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Follow wall action succeeded. Reason: %d", follow_wall_action.getResult()->reason);
            } else {
                ROS_INFO("Follow wall action finished with unknown state: %s", state.toString().c_str());
            }
        } else {
            ROS_WARN("Follow wall action timed out.");
        }*/
    }

    void follow_wall_done_callback(const actionlib::SimpleClientGoalState& state, const s8_wall_follower_controller::FollowWallResultConstPtr & result) {
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Follow wall action succeeded. Reason: %d", follow_wall_action.getResult()->reason);
        } else {
            ROS_INFO("Follow wall action finished with unknown state: %s", state.toString().c_str());
        }

        stop();
        ros::Duration duration(5);
        duration.sleep();
        turn(90);
        duration.sleep();
        follow_wall(1);
    }

    void turn(int degrees) {
        ROS_INFO("Turning %d...", degrees);

        turning = true;
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

        turning = false;
    }

    void stop() {
        ROS_INFO("Stopping...");

        s8_motor_controller::StopGoal goal;
        goal.stop = true;
        stop_action.sendGoal(goal);

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = stop_action.getState();
            ROS_INFO("Stop action finished. %s", state.toString().c_str());
        } else {
            ROS_WARN("Stop action timed out.");
        }
    }

    void distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        if(turning) {
            return;
        }

        double front_left = ir_distances->front_left;
        double front_right = ir_distances->front_right;

        auto front_inside_treshold = [this] (double value) {
            return value > front_distance_treshold_near && value < front_distance_treshold_far;
        };

        if(front_inside_treshold(front_left) || front_inside_treshold(front_right)) {
            //There is a wall in front of robot.
            ROS_INFO("Wall ahead! left: %.2lf, right: %.2lf", front_left, front_right);

            if(std::abs(front_left) <= front_distance_stop || std::abs(front_right) <= front_distance_stop) {
                ROS_INFO("Too close to wall! Stopping...");
                //TODO: Need to cancel wall follower if running.
                follow_wall_action.cancelGoal();
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
