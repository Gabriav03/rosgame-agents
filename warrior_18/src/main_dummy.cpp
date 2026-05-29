#include "warrior_18/warrior.hpp" // Usa la misma cabecera

int main ( int argc, char * argv[] )
{
    rclcpp::init ( argc, argv );
    
    // Aquí se instancia la clase Warrior, pero como compilaremos
    // este main junto con dummie.cpp, usará la lógica del dummy.
    auto node = std::make_shared<Warrior>();

    rclcpp::Rate rate(1);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}