#include "bot3_firmware/bot_three_interface.hpp"
#include<hardware_interface/types/hardware_interface_type_values.hpp>


namespace bot_one_frimware
{
BotOneInterface::BotOneInterface()
{

}

BotOneInterface::~BotOneInterface(){
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
           RCLCPP_FATAL_STREAM(rclcpp::get_logger("BotOneInterface"),"Something went wrong while closing the connection with the port " << port_);
        }
    }

}

CallbackReturn BotOneInterface::on_init(const hardware_interface::HardwareInfo & hardware_info){

    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if(result != CallbackReturn::SUCCESS){
        return result;
    }
    try{
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range &e){
       return CallbackReturn::FAILURE;

    }

    velocity_commands.reserve(info_.joints.size());
    position_states.reserve(info_.joints.size());
    velocity_states.reserve(info_.joints.size());
    last_run = rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> BotOneInterface::export_state_interfaces() {

        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION,
            &position_states[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY,
            &velocity_states[i]));
        }

        return state_interfaces;
    }

std::vector<hardware_interface::CommandInterface> BotOneInterface::export_command_interfaces(){

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i= 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,&velocity_commands[i]) );
    }
    
    return command_interfaces;
}


CallbackReturn BotOneInterface::on_activate(const rclcpp_lifecycle::State & previous_state) {

        RCLCPP_INFO(rclcpp::get_logger("BotOneInterface"),"starting robot hardware.........");
        velocity_commands = {0.0, 0.0, 0.0, 0.0};
        position_states = {0.0, 0.0, 0.0, 0.0};
        velocity_states = {0.0, 0.0, 0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("BotOneInterface"),"something went wrong while interacting with port "<< port_);
        return CallbackReturn::FAILURE;

        }

        RCLCPP_INFO(rclcpp::get_logger("BotOneInterface"),"hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }
CallbackReturn BotOneInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state) {

    RCLCPP_INFO(rclcpp::get_logger("BotOneInterface"),"stoping robot hardware.........");
    if(arduino_.IsOpen())
    {
        try{
            arduino_.Close();
        }
        catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BotOneInterface"),"something went wrong while closing the port "<< port_);
            return CallbackReturn::FAILURE;

        }
    }
}

hardware_interface::return_type BotOneInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

    if(arduino_.IsDataAvailable()){

        auto dt = (rclcpp::Clock().now() - last_run).seconds();

        std::string message;
        arduino_.ReadLine(message);
        std::stringstream ss(message);
        std::string res;
        int multiplier = 1;
        while(std::getline(ss, res, ',')){
            multiplier = res.at(2) == 'p' ? 1 : -1;
            if(res.at(0) == 'f'){
                if(res.at(1) == 'l'){
                velocity_states.at(0) = multiplier * std::stod(res.substr(3,res.size()));
                position_states.at(0) += velocity_states.at(0) * dt;
                }
                else if(res.at(1) == 'r'){
                velocity_states.at(1) = multiplier * std::stod(res.substr(3,res.size()));
                position_states.at(1) += velocity_states.at(1) * dt;                   
                }

            }
            else if(res.at(0) == 'r'){
                if(res.at(1) == 'l'){
                velocity_states.at(2) = multiplier * std::stod(res.substr(3,res.size()));
                position_states.at(2) += velocity_states.at(2) * dt;
                }
                else if(res.at(1) == 'r'){
                velocity_states.at(3) = multiplier * std::stod(res.substr(3,res.size()));
                position_states.at(3) += velocity_states.at(3) * dt;
                }

            }
        }
        last_run = rclcpp::Clock().now();
    }
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type BotOneInterface::write(const rclcpp::Time & time,const rclcpp::Duration & period) {

    std::stringstream message_stream;
    char front_left_wheel_sign = velocity_commands.at(0) >= 0 ? 'p' : 'n';
    char front_right_wheel_sign = velocity_commands.at(1) >= 0 ? 'p' : 'n';
    char rear_left_wheel_sign = velocity_commands.at(2) >= 0 ? 'p' : 'n';
    char rear_right_wheel_sign = velocity_commands.at(3) >= 0 ? 'p' : 'n';

    std::string compensate_zeros_front_left = "";
    std::string compensate_zeros_front_right = "";
    std::string compensate_zeros_rear_right = "";
    std::string compensate_zeros_rear_left = "";

    if(std::abs(velocity_commands.at(0)) < 10.0){
        compensate_zeros_front_left = "0";
    }
    else{
        compensate_zeros_front_left = "";
    }

    if(std::abs(velocity_commands.at(1)) < 10.0){
        compensate_zeros_front_right = "0";
    }
    else{
        compensate_zeros_front_right = "";
    }

    if(std::abs(velocity_commands.at(2)) < 10.0){
        compensate_zeros_rear_left = "0";
    }
    else{
        compensate_zeros_rear_left = "";
    }

    if(std::abs(velocity_commands.at(3)) < 10.0){
        compensate_zeros_rear_right = "0";
    }
    else{
        compensate_zeros_rear_right = "";
    }


    message_stream << std::fixed << std::setprecision(2) <<
    "fl" << front_left_wheel_sign << compensate_zeros_front_left << std::abs(velocity_commands.at(0)) <<
    ", fr" << front_right_wheel_sign << compensate_zeros_front_right << std::abs(velocity_commands.at(1)) << 
    ", rl"<< rear_left_wheel_sign << compensate_zeros_rear_left << std::abs(velocity_commands.at(2)) <<
    ", rr" << rear_right_wheel_sign << compensate_zeros_rear_right << std::abs(velocity_commands.at(3)); 

    try{
        arduino_.Write(message_stream.str());

    }
    catch(...){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("BotOneInterface"),"something went wrong while sending msg "<< message_stream.str() << "on the port " << port_);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


}

#include<pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bot_one_frimware::BotOneInterface, hardware_interface::SystemInterface)

