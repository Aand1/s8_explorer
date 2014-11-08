#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <s8_common_node/Node.h>
#include <s8_wall_follower_controller/wall_follower_controller_node.h>

#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>
#include <s8_msgs/IRDistances.h>
#include <geometry_msgs/Twist.h>

#define NODE_NAME                                   "s8_explorer_node"

#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR     "front_distance_treshold_near"
#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR      "front_distance_treshold_far"
#define PARAM_NAME_FRONT_DISTANCE_STOP_MAX          "front_distance_stop_max"
#define PARAM_NAME_FRONT_DISTANCE_STOP_MIN          "front_distance_stop_min"
#define PARAM_NAME_SIDE_DISTANCE_TRESHOLD_NEAR      "side_distance_treshold_near"
#define PARAM_NAME_SIDE_DISTANCE_TRESHOLD_FAR       "side_distance_treshold_far"
#define PARAM_NAME_GO_STRAIGHT_VELOCITY             "go_straight_velocity"

#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR  0.10
#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR   0.4
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP_MAX       0.28
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP_MIN       0.19
#define PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_NEAR   0.04
#define PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_FAR    0.2
#define PARAM_DEFAULT_GO_STRAIGHT_VELOCITY          0.2


#define TOPIC_DISTANCES                             "/s8/ir_distances"
#define TOPIC_TWIST                                 "/s8/twist"
#define TOPIC_ACTUAL_TWIST                          "/s8/actual_twist"
#define ACTION_TURN                                 "/s8/turn"
#define ACTION_STOP                                 "/s8_motor_controller/stop"
#define ACTION_FOLLOW_WALL                          "/s8/follow_wall"

#define ACTION_STOP_TIMEOUT                         30.0
#define ACTION_TURN_TIMEOUT                         30.0

#define WALL_FOLLOW_REASON_TIMEOUT                  s8::wall_follower_controller_node::FollowWallFinishedReason::TIMEOUT
#define WALL_FOLLOW_REASON_OUT_OF_RANGE             s8::wall_follower_controller_node::FollowWallFinishedReason::OUT_OF_RANGE
#define WALL_FOLLOW_REASON_PREEMPTED                s8::wall_follower_controller_node::FollowWallFinishedReason::PREEMPTED

#define WALL_FOLLOW_SIDE_LEFT                       s8::wall_follower_controller_node::WallToFollow::LEFT
#define WALL_FOLLOW_SIDE_RIGHT                      s8::wall_follower_controller_node::WallToFollow::RIGHT

#define TURN_DEGREES_90                             90

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
        GOING_STRAIGHT
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
        case State::GOING_STRAIGHT:
            return "GOING_STRAIGHT";
        }

        return "UNKNOWN";
    }
};

class Explorer : public s8::Node {
    ros::Subscriber distances_subscriber;
    ros::Subscriber actual_twist_subscriber;
    ros::Publisher twist_publisher;
    actionlib::SimpleActionClient<s8_turner::TurnAction> turn_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;
    follow_wall_client follow_wall_action;
    int following_wall_side;
    double front_distance_treshold_near;
    double front_distance_treshold_far;
    double front_distance_stop_max;
    double front_distance_stop_min;
    double side_distance_treshold_near;
    double side_distance_treshold_far;
    double front_left;
    double front_right;
    double left_back;
    double left_front;
    double right_back;
    double right_front;
    double go_straight_velocity;
    StateManager state_manager;
    bool should_stop_go_straight;
    double actual_v;
    double actual_w;

public:
    Explorer() : actual_v(0.0), actual_w(0.0), should_stop_go_straight(false), turn_action(ACTION_TURN, true), stop_action(ACTION_STOP, true), follow_wall_action(ACTION_FOLLOW_WALL, true), state_manager(StateManager::State::STILL, std::bind(&Explorer::on_state_changed, this, std::placeholders::_1, std::placeholders::_2)), front_left(0.0), front_right(0.0), left_back(0.0), left_front(0.0), right_back(0.0), right_front(0.0), following_wall_side(0) {
        init_params();
        print_params();
        distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_DISTANCES, 1, &Explorer::distances_callback, this);
        actual_twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_ACTUAL_TWIST, 1, &Explorer::actual_twist_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);

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

        follow_wall(WALL_FOLLOW_SIDE_RIGHT);
    }

private:
    void on_state_changed(StateManager::State previous_state, StateManager::State current_state) {
        typedef StateManager::State State;

        std::string print_extra = "";

        if(state_manager.get_state() == StateManager::State::FOLLOWING_WALL) {
            print_extra += "(" + std::string(following_wall_side == WALL_FOLLOW_SIDE_RIGHT ? "right" : "left") + ")";
        }

        ROS_INFO("State transition: %s -> %s %s", state_manager.state_to_string(previous_state).c_str(), state_manager.state_to_string(current_state).c_str(), print_extra.c_str());

        if(current_state == State::FOLLOWING_WALL_PREEMPTED) {
            //Wall following has been cancelled. Probably because something is in front of the robot (but it might have been cancelled by other reasons).

            stop();
            turn(following_wall_side * TURN_DEGREES_90);
            follow_wall(following_wall_side);
        } else if(current_state == State::FOLLOWING_WALL_OUT_OF_RANGE) {
            //Wall following out of range. This means that there is no more wall to follow on this partical side (but there might be on the other side).

            stop();

            //If there are no walls to close, the robot needs to explore (just go straight) until a wall pops up.
            int follow_side = get_wall_to_follow();
            if(follow_side == 0) {
                //No walls in range. Just stop for now.
                //TODO: Should do something else in the future
                ROS_INFO("I dont know what to do, so I'm just going forward!");

                go_straight([this]() {
                    return !is_left_wall_present() && !is_right_wall_present() && !is_front_obstacle_too_close();
                });

                stop();

                ROS_INFO("get wall: %d", get_wall_to_follow());
            }

            follow_side = get_wall_to_follow();

            if(follow_side == 0) {
                follow_side = -following_wall_side;
                
                if(follow_side == 0) {
                    ROS_FATAL("Weird state");
                    follow_side = WALL_FOLLOW_SIDE_LEFT;
                }
            }

            if(is_front_obstacle_too_close()) {
                //There is a wall to follow but we have an object to the front. So turn away from the wall.
                turn(follow_side * TURN_DEGREES_90);
            }

            //Now there is a wall to follow, so follow it.
            follow_wall(follow_side);
        } else if(current_state == State::FOLLOWING_WALL_TIMED_OUT) {
            //Wall following timed out. This means that the robot has been following the wall for long time, but the is still more wall to follow.

            //Keep following wall since there is no other reason to do something else.
            follow_wall(following_wall_side);
        } else if(current_state == State::TURNING_TIMED_OUT) {
            stop();
        } else if(current_state == State::STOPPING_TIMED_OUT) {

        } else if(current_state == State::STILL) {

        }
    }

    void follow_wall(int side) {
        following_wall_side = side;
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

        bool finised_before_timeout = turn_action.waitForResult(ros::Duration(ACTION_TURN_TIMEOUT));

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

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(ACTION_STOP_TIMEOUT));

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
        front_left = ir_distances->front_left;
        front_right = ir_distances->front_right;
        left_back = ir_distances->left_back;
        left_front = ir_distances->left_front;
        right_back = ir_distances->right_back;
        right_front = ir_distances->right_front;

	    if(!is_following_wall() && !is_going_straight()) {
            return;
        }

        if(is_front_obstacle_too_close()) {
     
            //TODO: Need to cancel wall follower if running.
	        if(is_following_wall()) {
	            follow_wall_action.cancelGoal();
            } else if(is_going_straight()) {
                should_stop_go_straight = true;
            }
        }
    }

    void actual_twist_callback(const geometry_msgs::Twist::ConstPtr & actual_twist) {
        actual_v = actual_twist->linear.x;
        actual_w = actual_twist->angular.z;
    }

    int get_wall_to_follow() {
        //Check if there is a wall on the opposite side.
        if(following_wall_side == WALL_FOLLOW_SIDE_LEFT) {
            if(is_right_wall_present()) {
                //We used to follow left side and there is a right wall present. Follow that wall instead.
                return WALL_FOLLOW_SIDE_RIGHT;
            } else if(is_left_wall_present()) {
                //We used to follow left side, there is no right wall to follow but the left side seem to be present again. Lets follow it again.
                return WALL_FOLLOW_SIDE_LEFT;
            }
        } else if(following_wall_side == WALL_FOLLOW_SIDE_RIGHT) {
            if(is_left_wall_present()) {
                //We used to follow right side and there is a left wall present. Follow that wall instead.
                return WALL_FOLLOW_SIDE_LEFT;
            } else if(is_right_wall_present()) {
                //We used to follow right side, there is no left wall to follow but the right side seem to be present again. Lets follow it again.
                return WALL_FOLLOW_SIDE_RIGHT;
            }
        } else {
            if(is_right_wall_present()) {
                //We used to follow left side and there is a right wall present. Follow that wall instead.
                return WALL_FOLLOW_SIDE_RIGHT;
            } else if(is_left_wall_present()) {
                //We used to follow left side, there is no right wall to follow but the left side seem to be present again. Lets follow it again.
                return WALL_FOLLOW_SIDE_LEFT;
            }
        }

        return 0;
    }

    void go_straight(std::function<bool()> condition) {
        state_manager.set_state(StateManager::State::GOING_STRAIGHT);
        geometry_msgs::Twist twist;
        twist.linear.x = go_straight_velocity;

        should_stop_go_straight = false;

        ros::Rate loop_rate(10);
        while(condition() && ros::ok() && !should_stop_go_straight) {
            twist_publisher.publish(twist);
            loop_rate.sleep();
        }
    }

    bool is_inside_treshold(double value, double treshold_near, double treshold_far) {
        return value > treshold_near && value < treshold_far;
    }

    bool is_front_inside_treshold(double value) {
        return is_inside_treshold(value, front_distance_treshold_near, front_distance_treshold_far);
    }

    bool is_side_inside_treshold(double value) {
        return is_inside_treshold(value, side_distance_treshold_near, side_distance_treshold_far);
    }

    bool is_side_wall_present(double back, double front) {
        return is_side_inside_treshold(back) && is_side_inside_treshold(front);
    }

    bool is_left_wall_present() {
        return is_side_wall_present(left_back, left_front);
    }

    bool is_right_wall_present() {
        return is_side_wall_present(right_back, right_front);
    }

    bool is_front_obstacle_present() {
        auto is = is_front_inside_treshold(front_left) || is_front_inside_treshold(front_right);

        if(is) {
            ROS_INFO("Wall ahead! left: %.2lf, right: %.2lf", front_left, front_right);
        }

        return is;
    }

    bool is_front_obstacle_too_close() {
        if(is_front_obstacle_present()) {
            double treshold = get_speed_calculated_distance_stop();
            ROS_INFO("Front stop treshold: %.2lf", treshold);

            auto is = (std::abs(front_left) <= treshold || std::abs(front_right) <= treshold);
    
            if(is) {
                ROS_INFO("Too close to obstacle!");
            }

            return is;
        }

        return false;
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

    bool is_going_straight() {
        return state_manager.get_state() == StateManager::State::GOING_STRAIGHT;
    }

    double get_speed_calculated_distance_stop() {
        const double nearest = front_distance_stop_min;
        const double farest = front_distance_stop_max;
        const double diff = farest - nearest;
        const double max_speed = 0.2;
        return (diff * actual_v / max_speed) + nearest;
    }

    void init_params() {
        add_param(PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR, front_distance_treshold_near, PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR);
        add_param(PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR, front_distance_treshold_far, PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR);
        add_param(PARAM_NAME_FRONT_DISTANCE_STOP_MAX, front_distance_stop_max, PARAM_DEFAULT_FRONT_DISTANCE_STOP_MAX);
        add_param(PARAM_NAME_FRONT_DISTANCE_STOP_MIN, front_distance_stop_min, PARAM_DEFAULT_FRONT_DISTANCE_STOP_MIN);
        add_param(PARAM_NAME_SIDE_DISTANCE_TRESHOLD_NEAR, side_distance_treshold_near, PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_NEAR);
        add_param(PARAM_NAME_SIDE_DISTANCE_TRESHOLD_FAR, side_distance_treshold_far, PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_FAR);
        add_param(PARAM_NAME_GO_STRAIGHT_VELOCITY, go_straight_velocity, PARAM_DEFAULT_GO_STRAIGHT_VELOCITY);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    Explorer explorer;
    ros::spin();
    return 0;
}
