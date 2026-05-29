#ifndef ESTRATEGA_HPP
#define ESTRATEGA_HPP

#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/string.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <memory>

// Estados del guerrero
enum Estado_e { 
    GRANJERO_e, 
    PREPARANDO_e, 
    ACECHADOR_e, 
    KAMIKAZE_e,
    DESCONOCIDO_e 
};

// Estructura de datos del enemigo
struct EnemyInfo_e {
    std::string id;
    float health;
    float battery;
    bool has_hammer;
    bool has_shield;
    float x, y, gamma; 
};

class ESTRATEGA : public rclcpp::Node
{
public:
    ESTRATEGA();
    ~ESTRATEGA();

private:
    // --- COMUNICACIONES ---
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_warrior_state_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_my_scene_;
    
    // Mapa de publishers para atacar
    std::map<std::string, rclcpp::Publisher<rosgame_msgs::msg::RosgameTwist>::SharedPtr> cmd_publisher;

    // --- VARIABLES ---
    std::string my_code_ = "";     
    Estado_e current_warrior_state_ = DESCONOCIDO_e; 
    bool linked_to_warrior_ = false;
    float my_pos_x = 0.0f, my_pos_y = 0.0f;

    std::map<std::string, EnemyInfo_e> enemies_data_; 
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> enemy_subscriptions_;

    rclcpp::TimerBase::SharedPtr timer_cmd_;
    rclcpp::TimerBase::SharedPtr timer_scanner_;

    // FUNCIONES
    void process_warrior_state(const std_msgs::msg::String::SharedPtr msg);
    void process_my_scene_info(const std_msgs::msg::String::SharedPtr msg);
    void process_enemy_data(const std_msgs::msg::String::SharedPtr msg, std::string enemy_id);

    void scan();
    void cmd_cycle(); 
    void send_fake_cmd(std::string target_code, float lin, float ang);
    float calculate_distance(float x1, float y1, float x2, float y2);
};

#endif // ESTRATEGA_HPP