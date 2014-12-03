#include <ros/ros.h>

#include <s8_explorer/explorer_node.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <s8_common_node/Node.h>
#include <s8_utils/math.h>

#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>
#include <s8_msgs/IRDistances.h>
#include <geometry_msgs/Twist.h>
#include <s8_explorer/ExploreAction.h>
#include <s8_mapper/PlaceNode.h>
#include <s8_mapper/mapper_node.h>
#include <s8_msgs/DistPose.h>

#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR     "front_distance_treshold_near"
#define PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR      "front_distance_treshold_far"
#define PARAM_NAME_FRONT_DISTANCE_STOP_MAX          "front_distance_stop_max"
#define PARAM_NAME_FRONT_DISTANCE_STOP_MIN          "front_distance_stop_min"
#define PARAM_NAME_SIDE_DISTANCE_TRESHOLD_NEAR      "side_distance_treshold_near"
#define PARAM_NAME_SIDE_DISTANCE_TRESHOLD_FAR       "side_distance_treshold_far"
#define PARAM_NAME_GO_STRAIGHT_VELOCITY             "go_straight_velocity"
#define PARAM_NAME_THRESHOLD_TOLERANCE              "threshold_tolerance"

#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR  0.10
#define PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR   0.4
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP_MAX       0.23
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP_MIN       0.14
#define PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_NEAR   0.04
#define PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_FAR    0.2
#define PARAM_DEFAULT_GO_STRAIGHT_VELOCITY          0.15
#define PARAM_DEFAULT_THRESHOLD_TOLERANCE           0.15

#define ACTION_STOP_TIMEOUT                         30.0
#define ACTION_TURN_TIMEOUT                         30.0

#define TURN_DEGREES_90                             90

#define TOPO_NODE_FREE      1 << 1

using namespace s8::explorer_node;
using namespace s8::utils::math;
using s8::turner_node::to_string;
using s8::mapper_node::SERVICE_PLACE_NODE;

typedef actionlib::SimpleActionClient<s8_wall_follower_controller::FollowWallAction> follow_wall_client;
typedef s8::wall_follower_controller_node::FollowWallFinishedReason FollowWallFinishedReason;
typedef s8::turner_node::Direction RotateDirection;

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
    actionlib::SimpleActionServer<s8_explorer::ExploreAction> explore_action_server;
    follow_wall_client follow_wall_action;
    ros::ServiceClient place_node_client;
    FollowingWall following_wall;
    double front_distance_treshold_near;
    double front_distance_treshold_far;
    double front_distance_stop_max;
    double front_distance_stop_min;
    double side_distance_treshold_near;
    double side_distance_treshold_far;
    double threshold_tolerance;
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
    bool first;
    bool explore;
    bool preempted;
    bool merged_node;
    bool just_started;

public:
    Explorer() : merged_node(false), explore(false), preempted(false), first(false), actual_v(0.0), actual_w(0.0), should_stop_go_straight(false), turn_action(ACTION_TURN, true), stop_action(ACTION_STOP, true), follow_wall_action(ACTION_FOLLOW_WALL, true), state_manager(StateManager::State::STILL, std::bind(&Explorer::on_state_changed, this, std::placeholders::_1, std::placeholders::_2)), front_left(0.0), front_right(0.0), left_back(0.0), left_front(0.0), right_back(0.0), right_front(0.0), following_wall(FollowingWall::NONE), explore_action_server(nh, ACTION_EXPLORE, boost::bind(&Explorer::action_execute_explore_callback, this, _1), false) {
        init_params();
        print_params();

        distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_DISTANCES, 1, &Explorer::distances_callback, this);
        actual_twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_ACTUAL_TWIST, 1, &Explorer::actual_twist_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);

        place_node_client = nh.serviceClient<s8_mapper::PlaceNode>(SERVICE_PLACE_NODE, true);
        just_started = true;
        ROS_INFO("Waiting for turn action server...");
        turn_action.waitForServer();
        ROS_INFO("Connected to turn action server!");

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");

        ROS_INFO("Waiting for follow_wall action server...");
        follow_wall_action.waitForServer();
        ROS_INFO("Connected to follow_wall action server!");

        explore_action_server.registerPreemptCallback(boost::bind(&Explorer::explore_action_cancel_callback, this));
        explore_action_server.start();

        ROS_INFO("");
    }

private:
    void initial_move() {
        ROS_INFO("initial move");

        if(just_started){
            ROS_INFO("FIRST NODE");
	    place_node(0,0,TOPO_NODE_FREE, is_left_wall_present(), false, is_right_wall_present());
            just_started = false;
        }
        
        /*int dir = 1;
        while(ros::ok()) {
            turn(dir * TURN_DEGREES_90);
            ros::Duration duration(1.0);
            duration.sleep();
            dir *= -1;
        }*/

        if(is_right_wall_present()) {
            follow_wall(FollowingWall::RIGHT);
        } else if(is_left_wall_present()) {
            follow_wall(FollowingWall::LEFT);
        } else {
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_OUT_OF_RANGE);
        }
        /*
            go_straight([this,front_left, front_right]() {
                ros::spinOnce();
                return !is_left_wall_present() && !is_right_wall_present() && !is_front_obstacle_too_close() && !is_side_wall_over_threshold(left_back, left_front) && !is_side_wall_over_threshold(right_back, right_front);
            });
            stop();
            if(is_right_wall_present()) {
                follow_wall(FollowingWall::RIGHT);
            } else if(is_left_wall_present()) {
                follow_wall(FollowingWall::LEFT);
            } else {
                follow_wall(FollowingWall::NONE);
            }
        } */  
    }

    void on_state_changed(StateManager::State previous_state, StateManager::State current_state) {
        typedef StateManager::State State;

        if(!explore) {
            return;
        }

        std::string print_extra = "";

        if(state_manager.get_state() == StateManager::State::FOLLOWING_WALL) {
            print_extra += "(" + to_string(following_wall) + ")";
        }
        
        ROS_INFO("State transition: %s -> %s %s", state_manager.state_to_string(previous_state).c_str(), state_manager.state_to_string(current_state).c_str(), print_extra.c_str());

        if(current_state == State::FOLLOWING_WALL_PREEMPTED) {
            //Wall following has been cancelled. Probably because something is in front of the robot (but it might have been cancelled by other reasons).
            ROS_INFO("PREEMPTED");
            stop();
            if(following_wall == FollowingWall::NONE) {
                ROS_FATAL("no wall");
            }
            if (is_left_wall_present() || is_right_wall_present()){
                ROS_INFO("SEE A WALL");
                place_node(0,0,TOPO_NODE_FREE, is_left_wall_present(), is_front_obstacle_present(), is_right_wall_present());
                turn(RotateDirection(-following_wall) * TURN_DEGREES_90);
            }
            else{
                ROS_INFO("DONT SEE A WALL");
                place_node(0,0,TOPO_NODE_FREE, is_left_wall_present(), is_front_obstacle_present(), is_right_wall_present());
                turn(RotateDirection(following_wall) * TURN_DEGREES_90);
            }
            follow_wall(following_wall);
        } else if(current_state == State::FOLLOWING_WALL_OUT_OF_RANGE) {
            //Wall following out of range. This means that there is no more wall to follow on this partical side (but there might be on the other side).
            ROS_INFO("OUT OF RANGE");
            //stop();

            //If there are no walls to close, the robot needs to explore (just go straight) until a wall pops up.
            FollowingWall follow_side = get_wall_to_follow();
            if(follow_side == FollowingWall::NONE) {
                //No walls in range. Just stop for now.
                //TODO: Should do something else in the future
                ROS_INFO("I dont know what to do, so I'm just going forward!");

                go_straight([this]() {
                    return !is_left_wall_present() && !is_right_wall_present() && !is_front_obstacle_too_close() && !is_side_wall_over_threshold(left_back, left_front) && !is_side_wall_over_threshold(right_back, right_front);
                });

                //stop();

                //ROS_INFO("get wall: %d", get_wall_to_follow());
            }

            follow_side = get_wall_to_follow();
            //ROS_INFO("GET PREVIOUS WALL: %d", follow_side);

            if(follow_side == FollowingWall::NONE) {
                follow_side = FollowingWall(-following_wall);
                
                if(follow_side == FollowingWall::NONE) {
                    ROS_FATAL("Weird state");
                    follow_side = FollowingWall::LEFT;
                }
            }

            if(is_front_obstacle_too_close()) {
                //There is a wall to follow but we have an object to the front. So turn away from the wall.
                ROS_INFO("Stopping is_front_obstacle_too_close()");
                stop();
                place_node(0,0,TOPO_NODE_FREE, is_left_wall_present(), is_front_obstacle_present(), is_right_wall_present());
                turn(RotateDirection(-follow_side) * TURN_DEGREES_90);
            }

            //Now there is a wall to follow, so follow it.
            following_wall = follow_side;
            follow_wall(follow_side);
        } else if(current_state == State::FOLLOWING_WALL_TIMED_OUT) {
            //Wall following timed out. This means that the robot has been following the wall for long time, but the is still more wall to follow.

            //Keep following wall since there is no other reason to do something else.
            follow_wall(following_wall);
        } else if(current_state == State::TURNING_TIMED_OUT) {
            ROS_WARN("Turning action timed out");
            stop();
        } else if(current_state == State::STOPPING_TIMED_OUT) {

        } else if(current_state == State::STILL) {

        }
    }

    void action_execute_explore_callback(const s8_explorer::ExploreGoalConstPtr & explore_goal) {
        ROS_INFO("STARTED: Explore action started!");
        explore = true;
        preempted = false;
        first = true;
        merged_node = false;

        const int timeout = 60 * 5; // 5 min.
        const int rate_hz = 10;
        ros::Rate rate(rate_hz);
        int ticks = 0;

        while(ros::ok() && explore && ticks <= timeout * rate_hz && !merged_node) {
            rate.sleep();
            ticks++;
        }

        if(ticks >= timeout * rate_hz) {
            stop();
            s8_explorer::ExploreResult explore_action_result;
            explore_action_result.reason = ExploreFinishedReason::TIMEOUT;
            ROS_INFO("TIMEOUT: Explore action timed out.");
            explore_action_server.setAborted(explore_action_result);
        } else {
            if(preempted) {
                s8_explorer::ExploreResult explore_action_result;
                explore_action_result.reason = ExploreFinishedReason::PREEMPTED;
                ROS_INFO("PREEMPTED: Explore action preempted.");
                explore_action_server.setPreempted(explore_action_result);
            } else if(merged_node) {
                s8_explorer::ExploreResult explore_action_result;
                explore_action_result.reason = ExploreFinishedReason::REVISITED;
                ROS_INFO("PREEMPTED: Explore action preempted.");
                explore_action_server.setAborted(explore_action_result);
            } else {
                stop();
                s8_explorer::ExploreResult explore_action_result;
                explore_action_result.reason = ExploreFinishedReason::FAILED;
                ROS_INFO("FAILED: Explore action failed.");
                explore_action_server.setAborted(explore_action_result);
            }
        }
    }

    void explore_action_cancel_callback() {
        //TODO: Need to cancel wall follower if running.
        if(is_following_wall()) {
            follow_wall_action.cancelGoal();
        } else if(is_going_straight()) {
            should_stop_go_straight = true;
        }

        preempted = true;
        explore = false;
    }

    void follow_wall(FollowingWall wall) {
        following_wall = wall;

        if(following_wall == FollowingWall::NONE) {
            ROS_FATAL("follow_wall was called with NONE wall to follow.");
            return;
        }

        state_manager.set_state(StateManager::State::FOLLOWING_WALL);

        /*if(is_front_obstacle_too_close()) {
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_PREEMPTED);
            return;
        }*/

        s8_wall_follower_controller::FollowWallGoal goal;
        goal.wall_to_follow = wall;
        follow_wall_action.sendGoal(goal, boost::bind(&Explorer::follow_wall_done_callback, this, _1, _2), follow_wall_client::SimpleActiveCallback(), follow_wall_client::SimpleFeedbackCallback());
    }

    void follow_wall_done_callback(const actionlib::SimpleClientGoalState& state, const s8_wall_follower_controller::FollowWallResultConstPtr & result) {
        FollowWallFinishedReason reason = FollowWallFinishedReason(follow_wall_action.getResult()->reason);

        switch(reason) {
        case FollowWallFinishedReason::OUT_OF_RANGE:
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_OUT_OF_RANGE);
            break;
        case FollowWallFinishedReason::PREEMPTED:
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_PREEMPTED);
            break;
        case FollowWallFinishedReason::TIMEOUT:
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_TIMED_OUT);
            break;
        default:
            ROS_FATAL("Unknown wall following action goal reason: %d", reason);
        }
    }

    void turn(int degrees) {
        stop();
        ROS_INFO("Turning %s", to_string(RotateDirection(sign(degrees))).c_str());
        state_manager.set_state(StateManager::State::TURNING);

        s8_turner::TurnGoal goal;
        goal.degrees = degrees;
        turn_action.sendGoal(goal);

        bool finised_before_timeout = turn_action.waitForResult(ros::Duration(ACTION_TURN_TIMEOUT));
        ROS_INFO("Turn action state: %s", turn_action.getState().toString().c_str());
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

        actual_v = actual_w = 0.0; //TODO: safe to assume this?
    }

    void distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        front_left = ir_distances->front_left;
        front_right = ir_distances->front_right;
        left_back = ir_distances->left_back;
        left_front = ir_distances->left_front;
        right_back = ir_distances->right_back;
        right_front = ir_distances->right_front;

        if(first) {
            first = false;
            initial_move();
        }

        if(!is_following_wall() && !is_going_straight()) {
            return;
        }

        if(is_front_obstacle_too_close()) {
     
            //TODO: Need to cancel wall follower if running.
            if(is_following_wall()) {
                follow_wall_action.cancelGoal();
                ROS_INFO("Wall Following goal cancelled due to wall too close");
            } else if(is_going_straight()) {
                should_stop_go_straight = true;
            }
        }
    }

    void actual_twist_callback(const geometry_msgs::Twist::ConstPtr & actual_twist) {
        actual_v = actual_twist->linear.x;
        actual_w = actual_twist->angular.z;
    }

    FollowingWall get_wall_to_follow() {
        //Check if there is a wall on the opposite side.
        if(following_wall == FollowingWall::LEFT) {
            if(is_right_wall_present()) {
                //We used to follow left side and there is a right wall present. Follow that wall instead.
                ROS_INFO("FOLLOWING RIGHT");
                return FollowingWall::RIGHT;
            } else if(is_left_wall_present()) {
                //We used to follow left side, there is no right wall to follow but the left side seem to be present again. Lets follow it again.
                return FollowingWall::LEFT;
            }
        } else if(following_wall == FollowingWall::RIGHT) {
            if(is_left_wall_present()) {
                //We used to follow right side and there is a left wall present. Follow that wall instead.
                return FollowingWall::LEFT;
            } else if(is_right_wall_present()) {
                //We used to follow right side, there is no left wall to follow but the right side seem to be present again. Lets follow it again.
                return FollowingWall::RIGHT;
            }
        } else {
            if(is_right_wall_present()) {
                //We used to follow left side and there is a right wall present. Follow that wall instead.
                return FollowingWall::RIGHT;
            } else if(is_left_wall_present()) {
                //We used to follow left side, there is no right wall to follow but the left side seem to be present again. Lets follow it again.
                return FollowingWall::LEFT;
            }
        }

        return FollowingWall::NONE;
    }

    void go_straight(std::function<bool()> condition) {
        state_manager.set_state(StateManager::State::GOING_STRAIGHT);
        geometry_msgs::Twist twist;
        twist.linear.x = go_straight_velocity;

        should_stop_go_straight = false;

        ros::Rate loop_rate(25);
        while(condition() && ros::ok() && !should_stop_go_straight) {
            ros::spinOnce();
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

    bool is_side_wall_over_threshold(double back, double front){
    return is_side_wall_present(back*(1+threshold_tolerance), front*(1+threshold_tolerance));
    }


    bool is_front_obstacle_present() {
        auto is = is_front_inside_treshold(front_left) || is_front_inside_treshold(front_right);

        if(is) {
            //ROS_INFO("Wall ahead! left: %.2lf, right: %.2lf", front_left, front_right);
        }

        return is;
    }

    bool is_front_obstacle_too_close() {
        if(is_front_obstacle_present()) {
            double treshold = get_speed_calculated_distance_stop();
            //ROS_INFO("Front stop treshold: %.2lf", treshold);

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
        const double max_speed = 0.15;
        return (diff * actual_v / max_speed) + nearest;
    }

    void place_node(double x, double y, int value, bool left, bool forward, bool right) {
        s8_mapper::PlaceNode pn;
        pn.request.x = x;
        pn.request.y = y;
        pn.request.value = value;
        pn.request.isWallLeft = left;
        pn.request.isWallForward = forward;
        pn.request.isWallRight = right;
        if (left == true)
            ROS_INFO("left true");
        if (forward == true)
            ROS_INFO("forward true");
        if (right == true)
            ROS_INFO("right true");
        if(!place_node_client.call(pn)) {
            ROS_FATAL("Failed to call place node.");
        }

        //Check if the node was merged or not. 
        //merged_node should be set to true if it was.
    }

    void init_params() {
        add_param(PARAM_NAME_FRONT_DISTANCE_TRESHOLD_NEAR, front_distance_treshold_near, PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_NEAR);
        add_param(PARAM_NAME_FRONT_DISTANCE_TRESHOLD_FAR, front_distance_treshold_far, PARAM_DEFAULT_FRONT_DISTANCE_TRESHOLD_FAR);
        add_param(PARAM_NAME_FRONT_DISTANCE_STOP_MAX, front_distance_stop_max, PARAM_DEFAULT_FRONT_DISTANCE_STOP_MAX);
        add_param(PARAM_NAME_FRONT_DISTANCE_STOP_MIN, front_distance_stop_min, PARAM_DEFAULT_FRONT_DISTANCE_STOP_MIN);
        add_param(PARAM_NAME_SIDE_DISTANCE_TRESHOLD_NEAR, side_distance_treshold_near, PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_NEAR);
        add_param(PARAM_NAME_SIDE_DISTANCE_TRESHOLD_FAR, side_distance_treshold_far, PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_FAR);
        add_param(PARAM_NAME_GO_STRAIGHT_VELOCITY, go_straight_velocity, PARAM_DEFAULT_GO_STRAIGHT_VELOCITY);
        add_param(PARAM_NAME_THRESHOLD_TOLERANCE, threshold_tolerance, PARAM_DEFAULT_THRESHOLD_TOLERANCE); 
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    Explorer explorer;
    ros::spin();
    return 0;
}
