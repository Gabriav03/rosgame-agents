#ifndef WARRIOR_HPP
#define WARRIOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/string.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "rosgame_msgs/msg/rosgame_point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <map> 

// CONSTANTES GLOBALES
const float MAX_LINEAR_SPEED = 0.8f;
const float MAX_ANGULAR_SPEED = 1.0f; // Aumentado para reacción rápida
const float BATTERY_THRESHOLD = 40.0f; 
const float WALL_THRESHOLD = 0.95f;
const float OPONENT_THRESHOLD = 5.0f;
const float ITEM_WALL_MARGIN = 0.3f; // Margen para evitar muros al buscar ítems
const float KAMIKAZE_CHARGE_DIST = 2.5f;
const float KAMIKAZE_AIM_TOLERANCE = 0.35f;

// CONSTANTES REACTIVAS (NUEVO)
const float SAFE_DISTANCE = 1.2f;      // Distancia mínima para considerar un camino "libre"
const float CRITICAL_DISTANCE = 0.55f; // Distancia de pánico (choque inminente)
const float ROBOT_WIDTH_ANGLE = 0.45f;  // Margen angular (radianes) para el ancho del robot

const std::string INTERNAL_STATE_TOPIC = "/warrior_internal/state";

// --- CLASE PID COMPARTIDA ---
class PIDController {
public:
    PIDController();
    void set_gains(double kp, double ki, double kd);
    double compute(double error, double dt);
    void reset();

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_error_, deriv_filter_state_;
    bool first_run_;
    
    double INTEGRAL_LIMIT = 0.3;
    double OUTPUT_LIMIT = 2.5;
    double FILTER_ALPHA = 0.6;
};

// --- STRUCT ENEMY INFO ---
struct EnemyInfo {
    std::string id;
    float health;
    float battery;
    bool has_hammer;
    bool has_shield;
    float x, y, gamma; 
    PIDController pid;
    rclcpp::Time last_control_time;
    float target_x, target_y;
    bool target_active;
    EnemyInfo() : health(100), battery(100), has_hammer(false), has_shield(false),
                  x(0), y(0), gamma(0), last_control_time(0,0, RCL_ROS_TIME), target_active(false) {}
};

enum Estado { DESCONOCIDO, GRANJERO, PREPARANDO, ACECHADOR, KAMIKAZE };

class Warrior : public rclcpp::Node
{
public:
    Warrior();
    ~Warrior();

    void process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void process_scene_info(const std_msgs::msg::String::SharedPtr msg);

private:
    void update_fsm();          
    void execute_navigation();  
    
    // NUEVA FUNCIÓN DE NAVEGACIÓN REACTIVA
    float find_safe_angle(float target_angle_local);
    bool check_path_clear(float angle, float dist_needed);

    void find_nearest_target(const std::vector<std::vector<float>>& targets); 
    void find_weakest_player();
    float calculate_target_score(const EnemyInfo& enemy);
    float distance_between(float x1, float y1, float x2, float y2);

    void publish_cmd(float v, float w);
    float normalize_angle(float angle);

    void scan_enemies();
    void process_enemy_data(const std_msgs::msg::String::SharedPtr msg, std::string enemy_id);
    void check_integrity(const rosgame_msgs::msg::RosgameTwist::SharedPtr msg);

    // --- VARIABLES ---
    rclcpp::Publisher<rosgame_msgs::msg::RosgameTwist>::SharedPtr pub_vel_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_scene_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Subscription<rosgame_msgs::msg::RosgameTwist>::SharedPtr sub_integrity_check_;

    float battery, pos_x, pos_y, gamma;
    bool hammer_enabled = false, shield_enabled = false;
    bool hammer_prev = false;
    float health;

    double target_x, target_y;
    bool target_set = false;
    float last_target_pos_x, last_target_pos_y;

    std::string code = "-1";
    std::string warrior_nick = "";

    std::vector<std::vector<float>> skills_pos;
    std::vector<std::vector<float>> chargers_pos;
    std::vector<std::vector<float>> players_pos;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    // YA NO USAMOS repulsive_force_x/y (Eliminadas)

    Estado current_state;
    PIDController pid_; 
    rclcpp::Time last_pid_time_;

    rclcpp::TimerBase::SharedPtr timer_scan_;

    std::map<std::string, EnemyInfo> enemies_data_; 
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> enemy_subscriptions_;

    float last_sent_v_ = 0.0f;
    float last_sent_w_ = 0.0f;
    bool under_cyberattack_ = false;
};

#endif