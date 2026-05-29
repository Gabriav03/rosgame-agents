#include "warrior_18/racer.hpp"
#include <chrono>
#include <thread>
#include <algorithm> 
#include <cmath>

Racer::Racer(): Node ("robot_warrior")
{
    warrior_nick = "warrior_18";
    int cont = 0;

    // --- Variables ---
    battery = 100.0f;
    pos_x = 0.0f; pos_y = 0.0f; gamma = 0.0f;
    objetivo_x = 0.0f; objetivo_y = 0.0f;
    code = "-1"; 
    
    front_dist_ = 10.0f; 
    frontal_cone_rad_ = 100.0 * (M_PI / 180.0);

    target_locked_ = false;
    current_target_pos_ = {0.0f, 0.0f};

    // --- AJUSTES DE VELOCIDAD ---
    lin_vel_base = 0.9f; 
    umbral_distancia_meta = 0.25f; 
    
    // PID 
    pid_kp = 0.8f;   
    pid_kd = 2.9f;    
    prev_error = 0.0f;

    kp_cargador = 0.5f;

    // VFF 
    peso_atraccion = 1.0f;       
    peso_repulsion_max = 0.9f;   

    current_state_ = States::DECIDIR_OBJETIVO;
    objetivo_actual = TipoObjetivo::NINGUNO;

    fuerza_repulsion = {0.0f, 0.0f};
    f_att = {0.0f, 0.0f};

    reset_stuck_monitor();

    // --- Registro ---
    auto client = create_client<rosgame_bridge::srv::RosgameRegister>("register_service");
    auto request = std::make_shared<rosgame_bridge::srv::RosgameRegister::Request>();
    request->username = warrior_nick;
    
    while(!client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Esperando servicio...");
    }
    
    while (code == "-1" && rclcpp::ok())
    {   
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            code = response->code;
            if (code == "-1") {
                warrior_nick = "warrior_18_" + std::to_string(++cont);
                request->username = warrior_nick;
            } else {   
                pub1_ = create_publisher<rosgame_msgs::msg::RosgameTwist>( "/" + code + "/cmd_vel", 10 );
                pub2_ = create_publisher<rosgame_msgs::msg::RosgamePoint>( "/" + code + "/goal_x_y", 10 );
                sub1_ = create_subscription<sensor_msgs::msg::LaserScan>( "/" + code + "/laser_scan", 1, std::bind(&Racer::process_laser_info, this, std::placeholders::_1));
                sub2_ = create_subscription<std_msgs::msg::String>( "/" + code + "/scene_info", 1, std::bind(&Racer::process_scene_info, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "RACER ID: %s", code.c_str());           
            }
        }
    }
}

Racer::~Racer() { RCLCPP_ERROR(this->get_logger(), "Destruido"); }

void Racer::reset_stuck_monitor()
{
    last_stuck_check_time_ = this->now();
    last_stuck_pos_x_ = pos_x;
    last_stuck_pos_y_ = pos_y;
}

bool Racer::check_if_stuck()
{
    auto current_time = this->now();
    if ((current_time - last_stuck_check_time_).seconds() > 4.0)
    {
        float dist = sqrt(pow(pos_x - last_stuck_pos_x_, 2) + pow(pos_y - last_stuck_pos_y_, 2));
        reset_stuck_monitor();
        if (dist < 0.28f) return true;
    }
    return false;
}

void Racer::process_laser_info(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    last_scan_ = msg;
    fuerza_repulsion = {0.0f, 0.0f};
    
    float rango_seguridad = 0.7f; 
    float rango_tangencial = 0.35f; 
    int rayos_activos = 0;
    float min_dist = 100.0f;
    float min_ang = 0.0f;

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {   
        float d = msg->ranges[i];
        float ang = msg->angle_min + i * msg->angle_increment;
        if (d < 0.05f || std::isinf(d)) continue;

        if (d < min_dist) { min_dist = d; min_ang = ang; }

        if (std::abs(ang) > (100.0 * M_PI / 180.0)) continue; 

        if (d < rango_seguridad) 
        {
            float fuerza = pow((rango_seguridad - d) / rango_seguridad, 2);
            fuerza_repulsion.f_x += -cos(ang) * fuerza;
            fuerza_repulsion.f_y += -sin(ang) * fuerza;
            rayos_activos++;
        }
    }

    if (rayos_activos > 0)
    {
        float intensidad = std::min(1.0f, (float)sqrt(pow(fuerza_repulsion.f_x, 2) + pow(fuerza_repulsion.f_y, 2)));
        if (intensidad > 0.001f) {
             float current_mod = sqrt(pow(fuerza_repulsion.f_x, 2) + pow(fuerza_repulsion.f_y, 2));
             fuerza_repulsion.f_x = (fuerza_repulsion.f_x / current_mod) * intensidad * peso_repulsion_max;
             fuerza_repulsion.f_y = (fuerza_repulsion.f_y / current_mod) * intensidad * peso_repulsion_max;
        }
    }

    if (min_dist < rango_tangencial) {
        float tan_x, tan_y;
        if (min_ang < 0) { tan_x = -sin(min_ang); tan_y = cos(min_ang); } 
        else { tan_x = sin(min_ang); tan_y = -cos(min_ang); }
        float peso_tan = 7.5f; 
        fuerza_repulsion.f_x += tan_x * peso_tan;
        fuerza_repulsion.f_y += tan_y * peso_tan;
    }
    front_dist_ = GetMinInSector(msg, frontal_cone_rad_);
}

void Racer::process_scene_info(const std_msgs::msg::String::SharedPtr msg)
{
    Json::CharReaderBuilder reader;
    Json::Value JsonSceneData;
    std::istringstream jsonStream(msg->data);
    std::string errs;
    
    if(Json::parseFromStream(reader, jsonStream, &JsonSceneData, &errs)) {
        battery = JsonSceneData["Battery_Level"].asFloat();
        pos_x = JsonSceneData["Robot_Pose"]["x"].asFloat();
        pos_y = JsonSceneData["Robot_Pose"]["y"].asFloat();
        gamma = JsonSceneData["Robot_Pose"]["gamma"].asFloat();

        chargers_pos_array.clear(); 
        coins_pos_array.clear();

        const Json::Value &chargers_pos = JsonSceneData["FOV"]["Chargers_Positions"];
        for (const Json::Value &charger : chargers_pos) {
            std::vector<float> data;
            for (const Json::Value &val : charger) data.push_back(val.asFloat());
            chargers_pos_array.push_back(data);
        }

        // --- ACTUALIZACIÓN DE MEMORIA DE CARGADORES ---
        // Si vemos cargadores nuevos, los añadimos o actualizamos.
        // Como es simple, limpiamos y rellenamos si vemos alguno.
        // Si no vemos ninguno, MANTENEMOS los antiguos en known_chargers_.
        if (!chargers_pos_array.empty()) {
            known_chargers_ = chargers_pos_array;
        }
        // ----------------------------------------------

        const Json::Value &coins_pos = JsonSceneData["FOV"]["Coins_Positions"];
        for (const Json::Value &coin : coins_pos) {
            std::vector<float> data;
            for (const Json::Value &val : coin) data.push_back(val.asFloat());
            coins_pos_array.push_back(data);
        }
    }
}

float Racer::calcula_coste(const std::vector<float>& pos)
{
    float dx = pos[0] - pos_x;
    float dy = pos[1] - pos_y;
    float dist = sqrt(dx*dx + dy*dy);
    
    float ang_to_target = atan2(dy, dx) - gamma;
    while (ang_to_target > M_PI) ang_to_target -= 2*M_PI;
    while (ang_to_target < -M_PI) ang_to_target += 2*M_PI;
    
    float costo_total = dist + (std::abs(ang_to_target) * 2.0f);
    return costo_total; 
}

bool Racer::check_obstacle_in_path(const std::vector<float>& target_pos)
{
    if (!last_scan_) return false; 
    float dx = target_pos[0] - pos_x;
    float dy = target_pos[1] - pos_y;
    float dist_target = sqrt(dx*dx + dy*dy);
    float angle_target = atan2(dy, dx) - gamma;
    while (angle_target > M_PI) angle_target -= 2*M_PI;
    while (angle_target < -M_PI) angle_target += 2*M_PI;

    if (angle_target < last_scan_->angle_min || angle_target > last_scan_->angle_max) return false;

    int idx = (angle_target - last_scan_->angle_min) / last_scan_->angle_increment;
    int margen_idx = (25.0 * M_PI / 180.0) / last_scan_->angle_increment; 

    for (int k = -margen_idx; k <= margen_idx; k++) {
        int check_idx = idx + k;
        if (check_idx >= 0 && check_idx < (int)last_scan_->ranges.size()) {
            float range = last_scan_->ranges[check_idx];
            if (range > 0.05f && range < (dist_target - 0.1f)) return true;
        }
    }
    return false; 
}

int count_coins_near_charger(const std::vector<float>& charger_pos, const std::vector<std::vector<float>>& coins) {
    int count = 0;
    for (const auto& coin : coins) {
        float d = sqrt(pow(coin[0] - charger_pos[0], 2) + pow(coin[1] - charger_pos[1], 2));
        if (d < 7.5f) count++;
    }
    return count;
}

// --- MEJORA: Lógica de batería robusta (anti-bucle) ---
bool Racer::necesito_cargar()
{
    // 1. Histéresis: Si ya estamos comprometidos con la batería, no parar hasta estar llenos.
    if (objetivo_actual == TipoObjetivo::BATERIA) {
        if (battery < 95.0f) return true; 
        return false; 
    }

    // 2. Emergencia absoluta
    if (battery < 15.0f) return true;

    // 3. Calculo inteligente usando MEMORIA (known_chargers_) en lugar de solo visión instantánea
    // Si usasemos solo chargers_pos_array y giramos, se vacía y el robot cree que no puede cargar.
    const auto& cargadores_a_usar = known_chargers_.empty() ? chargers_pos_array : known_chargers_;

    if (!cargadores_a_usar.empty()) {
        float best_score = -1000.0f;
        float dist_to_best = 1000.0f;
        bool blocked_best = false;

        for(auto& c : cargadores_a_usar) {
            float d = sqrt(pow(c[0]-pos_x, 2) + pow(c[1]-pos_y, 2));
            int coins_nearby = count_coins_near_charger(c, coins_pos_array);
            float score = (coins_nearby * 5.0f) - d; 
            
            if (score > best_score) {
                best_score = score;
                dist_to_best = d;
                blocked_best = check_obstacle_in_path(c);
            }
        }

        float coste_estimado = (dist_to_best * 3.0f) 
                             + (blocked_best ? 35.0f : 0.0f) 
                             + 8.0f; 

        if (battery < coste_estimado) return true;

    } else {
        // Si REALMENTE no conocemos ningún cargador, pánico al 25%
        if (battery < 25.0f) return true;
    }
    
    return false; 
}

void Racer::decidir_objetivo()
{
    // Usar known_chargers_ para tomar decisiones informadas aunque el cargador no esté en pantalla
    const auto& cargadores_disponibles = known_chargers_.empty() ? chargers_pos_array : known_chargers_;

    bool urgencia_bateria = necesito_cargar();

    if (urgencia_bateria && !cargadores_disponibles.empty()) {
        objetivo_actual = TipoObjetivo::BATERIA;
        float best_score = -1000.0f;
        
        for (const auto &c : cargadores_disponibles) {
            float dist = sqrt(pow(c[0] - pos_x, 2) + pow(c[1] - pos_y, 2));
            int coins_nearby = count_coins_near_charger(c, coins_pos_array);
            float score = (coins_nearby * 6.0f) - dist;
            if (score > best_score) {
                best_score = score;
                objetivo_x = c[0]; objetivo_y = c[1];
            }
        }
        objetivo_decidido = true;
        target_locked_ = true;
        current_target_pos_ = {objetivo_x, objetivo_y};
        return; 
    }

    if (target_locked_) {
        // Validación simple: si se ha bloqueado, seguimos (salvo obstáculos graves)
        if (check_obstacle_in_path(current_target_pos_)) {
            target_locked_ = false;
        } else {
            objetivo_x = current_target_pos_[0];
            objetivo_y = current_target_pos_[1];
            objetivo_decidido = true;
            return; 
        }
    }

    // --- BÚSQUEDA DE MEJOR MONEDA ---
    if (!coins_pos_array.empty())
    {
        objetivo_actual = TipoObjetivo::MONEDA;
        float min_coste = 10000.0f;
        bool found = false;

        for (const auto &coin : coins_pos_array)
        {
            if (check_obstacle_in_path(coin)) continue; 
            float coste = calcula_coste(coin);
            
            if (coste < min_coste) {
                min_coste = coste;
                objetivo_x = coin[0]; objetivo_y = coin[1];
                found = true;
            }
        }
        
        if (found) {
            objetivo_decidido = true;
            target_locked_ = true;
            current_target_pos_ = {objetivo_x, objetivo_y};
            return;
        } 
        
        // Fallback: salir del cargador si estamos en uno
        bool estoy_en_cargador = false;
        if (!cargadores_disponibles.empty()) {
            for(auto& c : cargadores_disponibles) {
                if (sqrt(pow(c[0]-pos_x, 2) + pow(c[1]-pos_y, 2)) < 1.0f) {
                    estoy_en_cargador = true; break;
                }
            }
        }

        if (estoy_en_cargador) {
             min_coste = 10000.0f;
             for (const auto &coin : coins_pos_array) {
                 float coste = calcula_coste(coin); 
                 if (coste < min_coste) {
                     min_coste = coste;
                     objetivo_x = coin[0]; objetivo_y = coin[1];
                     found = true;
                 }
             }
             if (found) {
                 objetivo_decidido = true;
                 target_locked_ = true;
                 current_target_pos_ = {objetivo_x, objetivo_y};
                 return;
             }
        }
    } 

    // Si no hay monedas o urgencia, y no tengo nada que hacer, voy al cargador por defecto
    // para esperar allí (mejor que estar parado en medio de la nada).
    if (!cargadores_disponibles.empty()) {
        objetivo_actual = TipoObjetivo::BATERIA; // Asumimos rol de ir a recargar/esperar
        float best_score = -1000.0f;
        for (const auto &c : cargadores_disponibles) {
            float dist = sqrt(pow(c[0] - pos_x, 2) + pow(c[1] - pos_y, 2));
            float score = -dist;
            if (score > best_score) {
                best_score = score;
                objetivo_x = c[0]; objetivo_y = c[1];
            }
        }
        objetivo_decidido = true;
        target_locked_ = true;
        current_target_pos_ = {objetivo_x, objetivo_y};
    } else {
        objetivo_decidido = false; 
        target_locked_ = false;
        objetivo_actual = TipoObjetivo::NINGUNO;
    }
}

Force Racer::calc_fuerza_atraccion()
{
    Force f = {0.0f, 0.0f};
    
    // Usamos variables locales para el cálculo, por si queremos sobreescribirlas
    // para la recolección oportunista
    float target_x_local = objetivo_x;
    float target_y_local = objetivo_y;

    if (!objetivo_decidido) return f;

    float dx = target_x_local - pos_x;
    float dy = target_y_local - pos_y;
    float dx_local = dx * cos(gamma) + dy * sin(gamma);
    float dy_local = -dx * sin(gamma) + dy * cos(gamma);
    float mod = sqrt(dx_local*dx_local + dy_local*dy_local);
    if (mod > 0.05f) {
        f.f_x = (dx_local / mod) * peso_atraccion;
        f.f_y = (dy_local / mod) * peso_atraccion;
    }
    return f;
}

void Racer::control_navegacion()
{
    // 1. Calcular fuerza base hacia el objetivo principal
    f_att = calc_fuerza_atraccion();

    // --- RECOLECCIÓN OPORTUNISTA ---
    // Si voy al cargador (objetivo lejano) y veo una moneda CERCA y ALINEADA,
    // modifico artificialmente la fuerza de atracción para pasar por ella.
    if (objetivo_actual == TipoObjetivo::BATERIA && !coins_pos_array.empty()) {
        for (const auto& coin : coins_pos_array) {
            float dx = coin[0] - pos_x;
            float dy = coin[1] - pos_y;
            float dist = sqrt(dx*dx + dy*dy);

            // Solo monedas muy cercanas (menos de 2.0m)
            if (dist < 2.0f) {
                float angle_to_coin = atan2(dy, dx) - gamma;
                while (angle_to_coin > M_PI) angle_to_coin -= 2*M_PI;
                while (angle_to_coin < -M_PI) angle_to_coin += 2*M_PI;

                // Solo si está "delante" (cono de +/- 20 grados)
                if (std::abs(angle_to_coin) < (20.0 * M_PI / 180.0)) {
                    // Verificamos que no haya muro entre medias
                    if (!check_obstacle_in_path(coin)) {
                        // ¡MONEDA OPORTUNISTA ENCONTRADA!
                        // Sobreescribimos f_att para apuntar a la moneda con fuerza
                        // No cambiamos 'objetivo_actual' ni 'objetivo_x' para no romper la FSM,
                        // simplemente hackeamos el control reactivo momentáneamente.
                        
                        // Calculo vector local a la moneda
                        float dx_l = dx * cos(gamma) + dy * sin(gamma);
                        float dy_l = -dx * sin(gamma) + dy * cos(gamma);
                        float mod_l = sqrt(dx_l*dx_l + dy_l*dy_l);
                        
                        // Sustituimos la atracción del cargador por la de la moneda
                        if (mod_l > 0.01f) {
                            f_att.f_x = (dx_l / mod_l) * 1.5f; // Un poco más de peso
                            f_att.f_y = (dy_l / mod_l) * 1.5f;
                        }
                        // Solo atacamos la primera que cumpla (la más cercana/visible)
                        break; 
                    }
                }
            }
        }
    }
    // --------------------------------

    float fx_total = f_att.f_x + fuerza_repulsion.f_x;
    float fy_total = f_att.f_y + fuerza_repulsion.f_y;
    float target_angle = atan2(fy_total, fx_total);
    while (target_angle > M_PI) target_angle -= 2 * M_PI;
    while (target_angle < -M_PI) target_angle += 2 * M_PI;
    error = target_angle; 
    float derivative = error - prev_error;
    float angular_cmd = (pid_kp * error) + (pid_kd * derivative); 
    prev_error = error;
    angular_cmd = std::max(-2.0f, std::min(2.0f, angular_cmd));
    float linear_cmd = lin_vel_base;
    float proximity_factor = std::clamp(front_dist_ / 2.0f, 0.1f, 1.0f);
    linear_cmd *= proximity_factor;
    if (std::abs(error) > 0.6f) linear_cmd *= 0.4f; 

    if (objetivo_actual == TipoObjetivo::BATERIA) {
        float dist = sqrt(pow(objetivo_x - pos_x, 2) + pow(objetivo_y - pos_y, 2));
        if (battery > 5.0f) {
            // Desaceleramos cerca del cargador
            if (dist < 3.5f) {
                linear_cmd = kp_cargador * dist;
                linear_cmd = std::max(0.15f, std::min(linear_cmd, lin_vel_base)); 
            }
        } else {
            // Si la batería es crítica, entramos rápido
            linear_cmd = std::max(linear_cmd, 0.4f); 
        }
    }
    PublishVelocity(linear_cmd, angular_cmd);
}

void Racer::FSM_Control_Loop()
{
    // Solo cambiamos a decidir si necesitamos cargar Y NO estamos ya yendo a ello.
    // Esto evita el bucle de re-evaluación constante.
    if (necesito_cargar() && objetivo_actual != TipoObjetivo::BATERIA) {
         target_locked_ = false;
         current_state_ = States::DECIDIR_OBJETIVO;
         reset_stuck_monitor();
    }

    switch (current_state_)
    {
    case States::DECIDIR_OBJETIVO:
        decidir_objetivo(); 
        if (objetivo_decidido) {
            prev_error = 0.0f; 
            current_state_ = States::NAVEGANDO;
            reset_stuck_monitor();
        } else { PublishVelocity(0.0, 0.5); }
        break;

    case States::NAVEGANDO:
        control_navegacion();
        if (he_llegado_a_objetivo()) {
            PublishVelocity(0.0, 0.0);
            target_locked_ = false; 
            if (objetivo_actual == TipoObjetivo::BATERIA) current_state_ = States::ESTOY_CARGANDO;
            else current_state_ = States::DECIDIR_OBJETIVO; 
        } else if (check_if_stuck()) {
            RCLCPP_ERROR(this->get_logger(), "ATASCADO");
            recovery_start_time_ = this->now();
            current_state_ = States::RECOVERY;
        }
        break;

    case States::RECOVERY:
        // Si durante el recovery detectamos necesidad crítica de batería (y no íbamos ya), cambiamos
        if (necesito_cargar() && objetivo_actual != TipoObjetivo::BATERIA) {
            current_state_ = States::DECIDIR_OBJETIVO;
            target_locked_ = false; 
            reset_stuck_monitor();
            break;
        }
        if ((this->now() - recovery_start_time_).seconds() < 1.5) PublishVelocity(-0.23f, 0.0f);
        else if ((this->now() - recovery_start_time_).seconds() < 3.0) PublishVelocity(0.0f, 1.1f); 
        else {
            PublishVelocity(0.0, 0.0);
            current_state_ = States::DECIDIR_OBJETIVO;
            target_locked_ = false; 
            reset_stuck_monitor();
        }
        break;

    case States::ESTOY_CARGANDO:
        // Comportamiento de carga: rotar si hay monedas, pero priorizar carga
        if (!coins_pos_array.empty()) {
            float min_c = 10000.0f;
            float next_x = 0.0f, next_y = 0.0f;
            for (const auto &coin : coins_pos_array) {
                float d = sqrt(pow(coin[0]-pos_x, 2) + pow(coin[1]-pos_y, 2));
                if (d < min_c) { min_c = d; next_x = coin[0]; next_y = coin[1]; }
            }
            float dx = next_x - pos_x;
            float dy = next_y - pos_y;
            float target_ang = atan2(dy, dx) - gamma;
            while (target_ang > M_PI) target_ang -= 2*M_PI;
            while (target_ang < -M_PI) target_ang += 2*M_PI;
            float w_load = 1.0f * target_ang;
            w_load = std::max(-0.5f, std::min(0.5f, w_load)); 
            PublishVelocity(0.0, w_load); 
        } else { PublishVelocity(0.0, 0.0); }

        // Aquí usamos la lógica de histéresis inversa implícita:
        // necesito_cargar() devolverá true hasta llegar al 95% si objetivo es BATERIA.
        // Cuando devuelva false, salimos.
        if (!necesito_cargar()) {
            current_state_ = States::DECIDIR_OBJETIVO;
            target_locked_ = false; 
            // Importante: al salir de carga, reseteamos el objetivo para permitir nuevas decisiones
            objetivo_actual = TipoObjetivo::NINGUNO; 
        }
        break;
    }
}

bool Racer::he_llegado_a_objetivo()
{
    float dist = sqrt(pow(objetivo_x - pos_x, 2) + pow(objetivo_y - pos_y, 2));
    
    if (dist < umbral_distancia_meta) {
        // --- PROTECCIÓN EXTRA ---
        // Si estamos yendo a cargar, asegúrate de que realmente hay un cargador aquí.
        // Esto evita que si "atropellamos" una moneda oportunista que casualmente pasaba 
        // cerca del punto objetivo (poco probable, pero posible), nos detengamos.
        // O más importante: asegura que no paremos antes de tocar el cargador.
        if (objetivo_actual == TipoObjetivo::BATERIA) {
             // Si estoy cerca del objetivo (coordenadas del cargador), asumo que he llegado.
             // El umbral ya es pequeño (0.25).
             return true;
        }

        if (objetivo_actual == TipoObjetivo::MONEDA) RCLCPP_INFO(this->get_logger(), "Moneda conseguida!");
        target_locked_ = false; 
        return true;
    }
    return false;
}

float Racer::GetMinInSector(const sensor_msgs::msg::LaserScan::SharedPtr msg, float angle_width_rad)
{
    float min_val = 10.0f;
    int center_idx = (msg->angle_max - msg->angle_min) / msg->angle_increment / 2;
    int range_width = (angle_width_rad / msg->angle_increment) / 2;
    for (int i = center_idx - range_width; i < center_idx + range_width; i++) {
        if (i >= 0 && i < (int)msg->ranges.size()) {
             float d = msg->ranges[i];
             if (d > 0.05 && d < min_val) min_val = d;
        }
    }
    return min_val;
}

void Racer::PublishVelocity(float vel_lineal, float vel_angular)
{
    rosgame_msgs::msg::RosgameTwist cmd;
    cmd.code = this->code;
    cmd.vel.linear.x = vel_lineal;
    cmd.vel.angular.z = vel_angular;
    pub1_->publish(cmd);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Racer>();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    rclcpp::Rate rate(10); 
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->FSM_Control_Loop();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}