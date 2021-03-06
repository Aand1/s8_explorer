#include <ros/ros.h>

#include <s8_explorer/explorer_node.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <s8_common_node/Node.h>
#include <s8_utils/math.h>
#include <s8_pose/pose_node.h>
#include <geometry_msgs/Pose2D.h>

#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_wall_follower_controller/FollowWallAction.h>
#include <s8_msgs/IRDistances.h>
#include <geometry_msgs/Twist.h>
#include <s8_explorer/ExploreAction.h>
#include <s8_mapper/PlaceNode.h>
#include <s8_mapper/mapper_node.h>
#include <s8_msgs/DistPose.h>
#include <s8_msgs/isFrontWall.h>

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
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP_MAX       0.26
#define PARAM_DEFAULT_FRONT_DISTANCE_STOP_MIN       0.14
#define PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_NEAR   0.04
#define PARAM_DEFAULT_SIDE_DISTANCE_TRESHOLD_FAR    0.15
#define PARAM_DEFAULT_GO_STRAIGHT_VELOCITY          0.15
#define PARAM_DEFAULT_THRESHOLD_TOLERANCE           0.15

#define ACTION_STOP_TIMEOUT                         30.0
#define ACTION_TURN_TIMEOUT                         30.0

#define TURN_DEGREES_90                             90

#define TOPO_NODE_FREE      1 << 1

#define NODE_WALL_SIDE_TRESHOLD                     0.25

#define TOPO_EAST                   -1 * M_PI / 4
#define TOPO_NORTH                  1 * M_PI / 4
#define TOPO_WEST                   3 * M_PI / 4
#define TOPO_SOUTH                  -3 * M_PI / 4
#define TOPIC_POSE                  s8::pose_node::TOPIC_POSE_SIMPLE
#define TOPIC_IS_FRONT_WALL         "/s8/isFrontWall"

using namespace s8::explorer_node;
using namespace s8::utils::math;
using s8::turner_node::to_string;
using s8::mapper_node::SERVICE_PLACE_NODE;
using namespace s8::mapper_node;

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
    ros::Subscriber pose_subscriber;
    ros::Subscriber wall_subscriber;
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
    RobotPose robot_pose;
    StateManager state_manager;
    bool is_front_wall;
    double front_wall_distance;
    bool should_stop_go_straight;
    double actual_v;
    double actual_w;
    bool first;
    bool explore;
    bool preempted;
    bool merged_node;
    bool just_started;
    int is_front_wall_cnt;

public:
    Explorer() : is_front_wall_cnt(0), merged_node(false), explore(false), preempted(false), first(false), actual_v(0.0), actual_w(0.0), should_stop_go_straight(false), turn_action(ACTION_TURN, true), stop_action(ACTION_STOP, true), follow_wall_action(ACTION_FOLLOW_WALL, true), state_manager(StateManager::State::STILL, std::bind(&Explorer::on_state_changed, this, std::placeholders::_1, std::placeholders::_2)), front_left(0.0), front_right(0.0), left_back(0.0), left_front(0.0), right_back(0.0), right_front(0.0), following_wall(FollowingWall::NONE), explore_action_server(nh, ACTION_EXPLORE, boost::bind(&Explorer::action_execute_explore_callback, this, _1), false) {
        init_params();
        print_params();

        distances_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_DISTANCES, 1, &Explorer::distances_callback, this);
        actual_twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_ACTUAL_TWIST, 1, &Explorer::actual_twist_callback, this);
        twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_TWIST, 1);
        pose_subscriber = nh.subscribe<geometry_msgs::Pose2D>(TOPIC_POSE, 1, &Explorer::pose_callback, this);
        wall_subscriber = nh.subscribe<s8_msgs::isFrontWall>(TOPIC_IS_FRONT_WALL, 1, &Explorer::wall_callback, this);
        is_front_wall = false;
        front_wall_distance = 0.0;
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
            place_node(0,0,0,0,TOPO_NODE_FREE, is_left_wall_present(NODE_WALL_SIDE_TRESHOLD), false, is_right_wall_present(NODE_WALL_SIDE_TRESHOLD), true);
            just_started = false;
            if(merged_node) {
                return;
            }
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
        // Ugly workaround TODO make right turning direction
        
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
                place_node(0,0,0,0,TOPO_NODE_FREE, is_left_wall_present(NODE_WALL_SIDE_TRESHOLD), is_front_obstacle_present(), is_right_wall_present(NODE_WALL_SIDE_TRESHOLD), true);
                if(merged_node) {
                    ROS_INFO("MERGED NODE");
                    return;
                }
                ROS_INFO("SHOULD TURN");
                turn(RotateDirection(-following_wall) * TURN_DEGREES_90);
            }
            else{
                ROS_INFO("DONT SEE A WALL");
                place_node(0,0,0,0,TOPO_NODE_FREE, is_left_wall_present(NODE_WALL_SIDE_TRESHOLD), is_front_obstacle_present(), is_right_wall_present(NODE_WALL_SIDE_TRESHOLD), true);
                if(merged_node) {
                    return;
                }
                turn(RotateDirection(following_wall) * TURN_DEGREES_90);
            }
             ROS_INFO("HELLUUU");
            follow_wall(following_wall);
        } else if(current_state == State::FOLLOWING_WALL_OUT_OF_RANGE) {
            //Wall following out of range. This means that there is no more wall to follow on this partical side (but there might be on the other side).
            ROS_INFO("OUT OF RANGE");
            //stop();
            place_node(0,0,0.3,0,TOPO_NODE_FREE, is_left_wall_present(NODE_WALL_SIDE_TRESHOLD), false, is_right_wall_present(NODE_WALL_SIDE_TRESHOLD), false);
            ROS_INFO("PLACED A NODE");
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
                place_node(0,0,0,0,TOPO_NODE_FREE, is_left_wall_present(NODE_WALL_SIDE_TRESHOLD), is_front_obstacle_present(), is_right_wall_present(NODE_WALL_SIDE_TRESHOLD), true);
                if(merged_node) {
                    return;
                }
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

    void pose_callback(const geometry_msgs::Pose2D::ConstPtr & pose) {
        robot_pose.position.x = pose->x;
        robot_pose.position.y = pose->y;
        robot_pose.rotation = radians_to_degrees(pose->theta);
        
        // Make sure that rotation stays in the range [0, 360]
        if (robot_pose.rotation > 360){
            int toRange = floor(robot_pose.rotation/360);
            robot_pose.rotation = robot_pose.rotation - toRange*360; 
        }
        else if (robot_pose.rotation < 0){
            int toRange = ceil(robot_pose.rotation/360);
            robot_pose.rotation = robot_pose.rotation + (toRange+1)*360;    
        }
    }

    void wall_callback(const s8_msgs::isFrontWall::ConstPtr & wall){
        ROS_INFO("Wall: %s", wall->isFrontWall ? "true" : "false");

        if(is_turning()) {
            ROS_INFO("Turning and wall callback");
            is_front_wall_cnt = 0;
            is_front_wall = false;
            return;   
        }

        front_wall_distance = wall->distToFrontWall;

        if(wall->isFrontWall) {
            is_front_wall_cnt++;
        } else {
            is_front_wall_cnt = 0;
        }

        if(is_front_wall_cnt >= 3) {
            is_front_wall = true;
        } else {
            is_front_wall = false;
        }
    }

    void follow_wall(FollowingWall wall) {
        if(!explore) {
            ROS_INFO("Ingnoring wall following because we are no longer exploring");
            return;
        }

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
            if(!explore && preempted) {
                ROS_INFO("Explore action was preempted and wall following has been cancelled. Stopping...");
                stop();
            }
            break;
        case FollowWallFinishedReason::TIMEOUT:
            state_manager.set_state(StateManager::State::FOLLOWING_WALL_TIMED_OUT);
            break;
        default:
            ROS_FATAL("Unknown wall following action goal reason: %d", reason);
        }
    }

    void turn(int degrees) {
        if(!explore) {
            ROS_INFO("Ingnoring turn because we are no longer exploring");
            return;
        }

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
        int heading = get_current_heading();

        should_stop_go_straight = false;

        ros::Rate loop_rate(25);
        ROS_INFO("IM GOING STRAIGHT");
        while(condition() && ros::ok() && !should_stop_go_straight) {
            ros::spinOnce();
            int rotation = (int)robot_pose.rotation % 360;
            if (rotation < 0){
                rotation += 360;
            }
            ROS_INFO("ROBOT POSE %d", rotation );

            int diff = heading - rotation;
            if (diff <= -45 )
                diff += 360; 
            ROS_INFO("DIFF %d", diff);
            double alpha = 1.0/45.0;
            twist.angular.z = alpha*(double)diff;
            ROS_INFO("alpha: %lf, diff: %lf",alpha, (double)diff);
            twist_publisher.publish(twist);
            loop_rate.sleep();
        }

        if(!explore && preempted) {
            ROS_INFO("Explore action has been preempted and go straight has been cancelled. Stopping...");
            stop();
        }
    }

    bool is_inside_treshold(double value, double treshold_near, double treshold_far) {
        return value > treshold_near && value < treshold_far;
    }

    bool is_front_inside_treshold(double value, double treshold = -1) {
        return is_inside_treshold(value, front_distance_treshold_near, front_distance_treshold_far);
    }

    bool is_side_inside_treshold(double value, double treshold = -1) {
        if(treshold < 0) {
            treshold = side_distance_treshold_far;
        }
        return is_inside_treshold(value, side_distance_treshold_near, treshold);
    }

    bool is_side_wall_present(double back, double front, double treshold = -1) {
        return is_side_inside_treshold(back, treshold) && is_side_inside_treshold(front, treshold);
    }

    bool is_left_wall_present(double treshold = -1) {
        return is_side_wall_present(left_back, left_front, treshold);
    }

    bool is_right_wall_present(double treshold = -1) {
        return is_side_wall_present(right_back, right_front, treshold);
    }

    bool is_side_wall_over_threshold(double back, double front, double treshold = -1){
    return is_side_wall_present(back*(1+threshold_tolerance), front*(1+threshold_tolerance), treshold);
    }


    bool is_front_obstacle_present() {
        auto is = is_front_inside_treshold(front_left) || is_front_inside_treshold(front_right) || is_front_wall;

        if(is) {
            //ROS_INFO("Wall ahead! left: %.2lf, right: %.2lf", front_left, front_right);
        }

        return is;
    }

    bool is_front_obstacle_too_close() {
        if(is_front_obstacle_present()) {
            double treshold = get_speed_calculated_distance_stop();
            //ROS_INFO("Front stop treshold: %.2lf", treshold);

            auto is = (std::abs(front_left) <= treshold || std::abs(front_right) <= treshold || is_front_wall);
    
            if(is) {
                ROS_INFO("Too close to obstacle!");
            }
            if(is_front_wall){
                ROS_FATAL("CAMERA DETECTED WALL");
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

    int get_current_heading(){
        ROS_INFO("REAL HEADING: %lf", robot_pose.rotation);
        if (robot_pose.rotation >= 315 || robot_pose.rotation < 45){
            ROS_INFO("HEADING EAST");
            return 0;
        }
        else if (robot_pose.rotation >= 45 && robot_pose.rotation < 135){
            ROS_INFO("HEADING NORTH");
            return 90;
        }
        else if (robot_pose.rotation >= 135 && robot_pose.rotation < 225){
            ROS_INFO("HEADING WEST");
            return 180;
        }
        else{
            ROS_INFO("HEADING SOUTH");
            return 270;
        }
    }

    void place_node(double x, double y, double dist, double theta, int value, bool left, bool forward, bool right, bool isTurn) {
        s8_mapper::PlaceNode pn;
        pn.request.x = x;
        pn.request.y = y;
        pn.request.dist = dist;
        pn.request.theta = theta;
        pn.request.value = value;
        pn.request.isWallLeft = left;
        pn.request.isWallForward = forward;
        pn.request.isWallRight = right;
        pn.request.isTurn = isTurn;
        if (left == true)
            ROS_INFO("left true");
        if (forward == true)
            ROS_INFO("forward true");
        if (right == true)
            ROS_INFO("right true");
        if(!place_node_client.call(pn)) {
            ROS_FATAL("Failed to call place node.");
            return;
        }

        if(!pn.response.placed) {
            ROS_INFO("Node was merged. Been here before. Stop exploring.");
            merged_node = true;
            explore = false;
            if(is_following_wall()) {
                follow_wall_action.cancelGoal();
            } else if(is_going_straight()) {
                should_stop_go_straight = true;
            }
        }
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
    ros::init(argc, argv, s8::explorer_node::NODE_NAME);
    Explorer explorer;
    ros::spin();
    return 0;
}
