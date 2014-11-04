#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_common_node/Node.h>

#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>
#include <s8_msgs/IRDistances.h>

#define NODE_NAME                                   "s8_explorer_node"

#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR     "front_distance_treshold_near"
#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR      "front_distance_treshold_far"
#define PARAM_NAME_FRONT_DISTANCE_STOP              "front_distance_stop"

#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR  0.10
#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR   0.4
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP           0.25


#define TOPIC_DISTANCES                             "/s8/ir_distances"
#define ACTION_TURN                                 "/s8/turn"
#define ACTION_STOP                                 "/s8_motor_controller/stop"
#define ACTION_FOLLOW_WALL                          "/s8/follow_wall"

#define WALL_FOLLOW_REASON_TIMEOUT                   1
#define WALL_FOLLOW_REASON_OUT_OF_RANGE              1 << 1
#define WALL_FOLLOW_REASON_PREEMPTED                 1 << 2

typedef actionlib::SimpleActionClient<s8_wall_follower_controller::FollowWallAction> follow_wall_client;

class StateManager {
public:
    enum State {
        STILL,
        STOPPING,
        STOPPING_TIMED_OUT,
        FOLLOWING_WALL,
        FOLLOWING_WALL_TIMED_OUT,
        FOLLOWING_WALL_OUT_OF_RANGE,
        FOLLOWING_WALL_PREEMPTED,
        TURNING,
        TURNING_TIMED_OUT,
    };

    State state;
    std::function<void(State, State)> on_state_changed;

public:
    StateManager(State state, std::function<void(State, State)> on_state_changed) : state(state), on_state_changed(on_state_changed) {}

    void set_state(State new_state) {
        State previous_state = state;
        state = new_state;
        on_state_changed(previous_state, new_state);
    }

    State get_state() {
        return state;
    }

    std::string state_to_string(State s) {
        switch(s) {
        case State::STILL:
            return "STILL";
        case State::STOPPING:
            return "STOPPING";
        case State::STOPPING_TIMED_OUT:
            return "STOPPING_TIMED_OUT";
        case State::FOLLOWING_WALL:
            return "FOLLOWING_WALL";
        case State::FOLLOWING_WALL_TIMED_OUT:
            return "FOLLOWING_WALL_TIMED_OUT";
        case State::FOLLOWING_WALL_OUT_OF_RANGE:
            return "FOLLOWING_WALL_OUT_OF_RANGE";
        case State::FOLLOWING_WALL_PREEMPTED:
            return "FOLLOWING_WALL_PREEMPTED";
        case State::TURNING:
            return "TURNING";
        case State::TURNING_TIMED_OUT:
            return "TURNING_TIMED_OUT";
        }

        return "UNKNOWN";
    }
};

class Explorer : public s8::Node {
    ros::Subscriber distances_subscriber;
    actionlib::SimpleActionClient<s8_turner::TurnAction> turn_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;
    follow_wall_client follow_wall_action;
    double front_distance_treshold_near;
    double front_distance_treshold_far;
    double front_distance_stop;
    StateManager state_manager;

public:
    Explorer() : turn_action(ACTION_TURN, true), stop_action(ACTION_STOP, true), follow_wall_action(ACTION_FOLLOW_WALL, true), state_manager(StateManager::State::STILL, std::bind(&Explorer::on_state_changed, this, std::placeholders::_1, std::placeholders::_2)) {
        init_params();
        print_params();
        distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_DISTANCES, 1, &Explorer::distances_callback, this);

        ROS_INFO("Waiting for turn action server...");
        turn_action.waitForServer();
        ROS_INFO("Connected to turn action server!");

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");

        ROS_INFO("Waiting for follow_wall action server...");
        follow_wall_action.waitForServer();
        ROS_INFO("Connected to follow_wall action server!");

        ROS_INFO("");

        follow_wall(1);
    }

private:
    void on_state_changed(StateManager::State previous_state, StateManager::State current_state) {
        typedef StateManager::State State;

        ROS_INFO("State transition: %s -> %s", state_manager.state_to_string(previous_state).c_str(), state_manager.state_to_string(current_state).c_str());

        ros::Duration duration(5);

        switch(current_state) {
        case State::FOLLOWING_WALL_PREEMPTED:
            //Wall following has been cancelled. Probably because something is in front of the robot (but it might have been cancelled by other reasons).

            stop();
            duration.sleep();
            turn(90);
            duration.sleep();
            //ROS_INFO("DONE");
            follow_wall(1);
            break;
        case State::FOLLOWING_WALL_OUT_OF_RANGE:
            //Wall following out of range. This means that there is no more wall to follow on this partical side (but there might be on the other side).

            stop();
            break;
        case State::FOLLOWING_WALL_TIMED_OUT:
            //Wall following timed out. This means that the robot has been following the wall for long time, but the is still more wall to follow.

            //Keep following wall since there is no other reason to do something else.
            follow_wall(1);
            break;
        case State::TURNING_TIMED_OUT:
            stop();
            break;
        case State::STOPPING_TIMED_OUT:
            break;
        case State::STILL:
            break;
        }
    }

    void follow_wall(int side) {
        state_manager.set_state(StateManager::State::FOLLOWING_WALL);

        s8_wall_follower_controller::FollowWallGoal goal;
        goal.wall_to_follow = side;
        follow_wall_action.sendGoal(goal, boost::bind(&Explorer::follow_wall_done_callback, this, _1, _2), follow_wall_client::SimpleActiveCallback(), follow_wall_client::SimpleFeedbackCallback());
    }

    void follow_wall_done_callback(const actionlib::SimpleClientGoalState& state, const s8_wall_follower_controller::FollowWallResultConstPtr & result) {
        int reason = follow_wall_action.getResult()->reason;

        switch(reason) {
        case WALL_FOLLOW_REASON_OUT_OF_RANGE:
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_OUT_OF_RANGE);
            break;
        case WALL_FOLLOW_REASON_PREEMPTED:
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_PREEMPTED);
            break;
        case WALL_FOLLOW_REASON_TIMEOUT:
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_TIMED_OUT);
            break;
        default:
            ROS_FATAL("Unknown wall following action goal reason: %d", reason);
        }
    }

    void turn(int degrees) {
        state_manager.set_state(StateManager::State::TURNING);

        s8_turner::TurnGoal goal;
        goal.degrees = degrees;
        turn_action.sendGoal(goal);

        bool finised_before_timeout = turn_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = turn_action.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                state_manager.set_state(StateManager::State::STILL);
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                //Time out from turner node.
                state_manager.set_state(StateManager::State::TURNING_TIMED_OUT);
            } else {
                ROS_WARN("Turn action finished with unknown state %s", state.toString().c_str());
                state_manager.set_state(StateManager::State::TURNING_TIMED_OUT);
            }
        } else {
            state_manager.set_state(StateManager::State::TURNING_TIMED_OUT);
        }
    }

    void stop() {
        state_manager.set_state(StateManager::State::STOPPING);

        s8_motor_controller::StopGoal goal;
        goal.stop = true;
        stop_action.sendGoal(goal);

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(30.0));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = stop_action.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                state_manager.set_state(StateManager::State::STILL);
            } else {
                ROS_INFO("Stop action finished with unknown state: %s", state.toString().c_str());
                state_manager.set_state(StateManager::State::STOPPING_TIMED_OUT);
            }
        } else {
            state_manager.set_state(StateManager::State::STOPPING_TIMED_OUT);
        }
    }

    void distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        if(!is_following_wall()) {
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
                ROS_INFO("Too close to wall!");
                //TODO: Need to cancel wall follower if running.
                follow_wall_action.cancelGoal();
            }
        }
    }

    bool is_following_wall() {
        return state_manager.get_state() == StateManager::State::FOLLOWING_WALL;
    }

    bool is_turning() {
        return state_manager.get_state() == StateManager::State::TURNING;
    }

    bool is_stopping() {
        return state_manager.get_state() == StateManager::State::STOPPING;
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
