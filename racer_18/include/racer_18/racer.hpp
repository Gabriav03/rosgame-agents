#include "rclcpp/rclcpp.hpp"
#include "jsoncpp/json/json.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosgame_msgs/msg/rosgame_twist.hpp"
#include "rosgame_msgs/msg/rosgame_point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgame_bridge/srv/rosgame_register.hpp"
#include "rclcpp/time.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

// Estados
enum class States {
    DECIDIR_OBJETIVO,
    NAVEGANDO,
    ESTOY_CARGANDO,
    RECOVERY
};

enum class TipoObjetivo {
    MONEDA,
    BATERIA,
    NINGUNO
};

struct Force {
    float f_x;
    float f_y;
};

class Racer : public rclcpp::Node
{
public:
    Racer();
    ~Racer();
    void FSM_Control_Loop();

private:
    // Callbacks
    void process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void process_scene_info(const std_msgs::msg::String::SharedPtr msg);

    // Funciones principales
    void decidir_objetivo();
    void control_navegacion(); 
    void PublishVelocity(float vel_lineal, float vel_angular);
    
    bool check_if_stuck();       
    void reset_stuck_monitor();

    // Auxiliares
    Force calc_fuerza_atraccion();
    bool he_llegado_a_objetivo();
    bool necesito_cargar(); 
    float calcula_coste(const std::vector<float>& pos); 
    float GetMinInSector(const sensor_msgs::msg::LaserScan::SharedPtr msg, float angle_width_rad);
    bool check_obstacle_in_path(const std::vector<float>& target_pos);

    // Comunicacion
    rclcpp::Publisher<rosgame_msgs::msg::RosgameTwist>::SharedPtr pub1_;
    rclcpp::Publisher<rosgame_msgs::msg::RosgamePoint>::SharedPtr pub2_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
    rclcpp::Client<rosgame_bridge::srv::RosgameRegister>::SharedPtr client_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    std::string warrior_nick;
    std::string code;
    
    States current_state_;
    TipoObjetivo objetivo_actual;

    float battery;
    float pos_x, pos_y, gamma;
    float objetivo_x, objetivo_y;

    // Parametros de control
    float lin_vel_base;
    float umbral_distancia_meta; 
    
    float front_dist_;      
    float frontal_cone_rad_;

    bool objetivo_decidido;
    
    float pid_kp, pid_kd;
    float prev_error;
    float peso_atraccion; 
    float peso_repulsion_max;
    float error;
    float kp_cargador;

    Force fuerza_repulsion;
    Force f_att;
    
    std::vector<std::vector<float>> chargers_pos_array;
    std::vector<std::vector<float>> coins_pos_array;
    
    // --- NUEVO: Memoria de cargadores para no perderlos al girar ---
    std::vector<std::vector<float>> known_chargers_; 

    // --- VARIABLES WATCHDOG (ATASCO) ---
    rclcpp::Time last_stuck_check_time_;
    rclcpp::Time recovery_start_time_;
    float last_stuck_pos_x_;
    float last_stuck_pos_y_;

    bool target_locked_;
    std::vector<float> current_target_pos_;
};