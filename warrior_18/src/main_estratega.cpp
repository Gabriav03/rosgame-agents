//--------------------------------------------------------------------//
//University of Málaga
//MAPIR Research Group - Machine Perception and Intelligent Robotics
//--------------------------------------------------------------------//

#include "warrior_18/estratega.hpp"

int main ( int argc, char * argv[] )
{
    rclcpp::init ( argc, argv );
    
    auto node=std::make_shared<ESTRATEGA>();

    rclcpp::Rate rate(20);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}