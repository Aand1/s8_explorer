#ifndef __EXPLORER_NODE_H
#define __EXPLORER_NODE_H

#include <string>
#include <s8_ir_sensors/ir_sensors_node.h>
#include <s8_motor_controller/motor_controller_node.h>
#include <s8_turner/turner_node.h>
#include <s8_wall_follower_controller/wall_follower_controller_node.h>

namespace s8 {
    namespace explorer_node {
        const std::string NODE_NAME =                       "s8_explorer_node";

        const std::string TOPIC_DISTANCES =                 s8::ir_sensors_node::TOPIC_IR_DISTANCES;
        const std::string TOPIC_TWIST =                     s8::motor_controller_node::TOPIC_TWIST;
        const std::string TOPIC_ACTUAL_TWIST =              s8::motor_controller_node::TOPIC_ACTUAL_TWIST;
        const std::string ACTION_STOP =                     s8::motor_controller_node::ACTION_STOP;
        const std::string ACTION_TURN =                     s8::turner_node::ACTION_TURN;
        const std::string ACTION_FOLLOW_WALL =              s8::wall_follower_controller_node::ACTION_FOLLOW_WALL;

        enum FollowingWall {
            LEFT = s8::wall_follower_controller_node::WallToFollow::LEFT,
            NONE = 0,
            RIGHT = s8::wall_follower_controller_node::WallToFollow::RIGHT
        };

        std::string to_string(FollowingWall following_wall) {
            switch(following_wall) {
                case FollowingWall::LEFT: return "LEFT";
                case FollowingWall::NONE: return "NONE";
                case FollowingWall::RIGHT: return "RIGHT";
            }

            return "UNKNOWN";
        }
    }
}

#endif
