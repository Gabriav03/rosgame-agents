// src/main_warrior.cpp
#include "warrior_18/warrior.hpp"
#include "warrior_18/estratega.hpp"
int main ( int argc, char * argv[] )
{
    rclcpp::init ( argc, argv );
    
    // Solo instanciamos a TU robot y TU estratega
    auto node = std::make_shared<Warrior>();
    auto node_est = std::make_shared<ESTRATEGA>();

    // Espera inicial para estabilizar ROS
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Usamos MultiThreadedExecutor para que ambos corran fluidos en paralelo
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node_est);
    
    executor.spin(); // Esto maneja los callbacks automáticamente
    
    rclcpp::shutdown();
    return 0;
}