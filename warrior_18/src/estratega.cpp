#include "warrior_18/estratega.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

ESTRATEGA::ESTRATEGA(): Node("robot_estratega")
{
    sub_warrior_state_ = this->create_subscription<std_msgs::msg::String>(
        "/warrior_internal/state", 10, std::bind(&ESTRATEGA::process_warrior_state, this, _1));

    // Ciclo de comandos (100ms)
    timer_cmd_ = this->create_wall_timer(100ms, std::bind(&ESTRATEGA::cmd_cycle, this));
    // Escaner de entorno (2s)
    timer_scanner_ = this->create_wall_timer(2s, std::bind(&ESTRATEGA::scan, this));

}

ESTRATEGA::~ESTRATEGA() {}

// SINCRONIZACION CON WARRIOR
void ESTRATEGA::process_warrior_state(const std_msgs::msg::String::SharedPtr msg)
{
    std::string data = msg->data;
    size_t delimiter = data.find("|");
    
    if (delimiter != std::string::npos) {
        std::string code_rec = data.substr(0, delimiter);
        std::string state_str = data.substr(delimiter + 1);

        if (!linked_to_warrior_ || my_code_ != code_rec) {
            my_code_ = code_rec;
            linked_to_warrior_ = true;
            sub_my_scene_ = this->create_subscription<std_msgs::msg::String>(
                "/" + my_code_ + "/scene_info", 10, 
                std::bind(&ESTRATEGA::process_my_scene_info, this, _1));
        }

        if (state_str == "GRANJERO") current_warrior_state_ = GRANJERO_e;
        else if (state_str == "PREPARANDO") current_warrior_state_ = PREPARANDO_e;
        else if (state_str == "ACECHADOR") current_warrior_state_ = ACECHADOR_e;
        else if (state_str == "KAMIKAZE") current_warrior_state_ = KAMIKAZE_e;
    }
}

// LECTURA RAPIDA DE JSON (MEMORIA DIRECTA)
void ESTRATEGA::process_my_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    // Configuracion estatica del builder para no recrearlo cada vez
    static const auto builder = []() {
        Json::CharReaderBuilder b;
        b["collectComments"] = false;
        return b;
    }();

    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value data;
    std::string errs;

    // Punteros directos al buffer del mensaje
    const char* begin = msg->data.c_str();
    const char* end = begin + msg->data.length();

    if (!reader->parse(begin, end, &data, &errs)) return;
    
    my_pos_x = data["Robot_Pose"]["x"].asFloat();
    my_pos_y = data["Robot_Pose"]["y"].asFloat();
}

void ESTRATEGA::scan()
{
    if (!linked_to_warrior_) return;
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    for (const auto& topic : topic_names_and_types) {
        std::string name = topic.first;
        if (name.find("/scene_info") != std::string::npos && 
            name.find("/Robot") == std::string::npos && 
            name.find(my_code_) == std::string::npos) 
        {
            size_t first = name.find("/");
            size_t second = name.find("/", first + 1);
            if (second != std::string::npos) {
                std::string enemy_id = name.substr(first + 1, second - first - 1);
                
                if (enemy_subscriptions_.find(enemy_id) == enemy_subscriptions_.end()) {
                    auto callback = [this, enemy_id](const std_msgs::msg::String::SharedPtr msg) {
                        this->process_enemy_data(msg, enemy_id);
                    };
                    enemy_subscriptions_[enemy_id] = this->create_subscription<std_msgs::msg::String>(name, 10, callback);
                    
                    EnemyInfo_e new_enemy;
                    new_enemy.id = enemy_id;
                    enemies_data_[enemy_id] = new_enemy;

                }
            }
        }
    }
}

// LECTURA RAPIDA DE JSON PARA ENEMIGOS
void ESTRATEGA::process_enemy_data(const std_msgs::msg::String::SharedPtr msg, std::string enemy_id)
{
    static const auto builder = []() {
        Json::CharReaderBuilder b;
        b["collectComments"] = false;
        return b;
    }();

    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value data;
    std::string errs;

    const char* begin = msg->data.c_str();
    const char* end = begin + msg->data.length();

    if (!reader->parse(begin, end, &data, &errs)) return;

    // Actualizamos directamente usando iterador para evitar busquedas repetidas
    auto it = enemies_data_.find(enemy_id);
    if (it != enemies_data_.end()) {
        it->second.health = data["Health"].asFloat();
        it->second.battery = data["Battery_Level"].asFloat();
        it->second.has_hammer = data["Skills"]["Hammer"].asBool();
        it->second.x = data["Robot_Pose"]["x"].asFloat();
        it->second.y = data["Robot_Pose"]["y"].asFloat();
        it->second.gamma = data["Robot_Pose"]["gamma"].asFloat(); 
    }
}

void ESTRATEGA::cmd_cycle() {
    if (!linked_to_warrior_ || enemies_data_.empty()) return;

    for (auto& [id, enemy] : enemies_data_) {
        if (id == my_code_ || enemy.health <= 0) continue;

        float dist = std::hypot(enemy.x - my_pos_x, enemy.y - my_pos_y);
        
        
        
        // 1. DEFENSA (GRANJERO / PREPARANDO)
        if (current_warrior_state_ == GRANJERO_e || current_warrior_state_ == PREPARANDO_e) {
            if (dist < 6.5) 
            {
                float angle_to_me = std::atan2(my_pos_y - enemy.y, my_pos_x - enemy.x);
                float angle_diff = angle_to_me - enemy.gamma;
                while(angle_diff > M_PI) angle_diff -= 2*M_PI; 
                while(angle_diff < -M_PI) angle_diff += 2*M_PI;

                if (std::abs(angle_diff) < 10.0) {
                    send_fake_cmd(id, -2.5, 1.4); 
                } else {
                    send_fake_cmd(id, 2.5, 0.0);
                }
            }
        }

        // 2. ATAQUE (KAMIKAZE)
        else if (current_warrior_state_ == KAMIKAZE_e && dist < 4.5) {
            send_fake_cmd(id, 0.03, 0.0); 
        }

        // 3. OPORTUNISMO
        else if (enemy.battery < 30.0) {
            send_fake_cmd(id, 0.5, 0.2);
        }
    }
}

void ESTRATEGA::send_fake_cmd(std::string target_code, float lin, float ang)
{
    if (cmd_publisher.find(target_code) == cmd_publisher.end()) {
        std::string topic = "/" + target_code + "/cmd_vel";
        cmd_publisher[target_code] = this->create_publisher<rosgame_msgs::msg::RosgameTwist>(topic, 10);
    }

    rosgame_msgs::msg::RosgameTwist msg;
    msg.code = target_code;
    msg.vel.linear.x = lin;
    msg.vel.angular.z = ang;

    // RAFAGA
    for(int i = 0; i < 3; i++) {
        cmd_publisher[target_code]->publish(msg);
    }
}

float ESTRATEGA::calculate_distance(float x1, float y1, float x2, float y2) {
    return std::hypot(x1 - x2, y1 - y2);
}