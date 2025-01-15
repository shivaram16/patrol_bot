#include "bot3_firmware/patrolbot_interface.hpp"
#include<hardware_interface/types/hardware_interface_type_values.hpp>


namespace bot3_firmware
{
PatrolbotInterface::PatrolbotInterface()
{

}

PatrolbotInterface::~PatrolbotInterface(){
    if(arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
           RCLCPP_FATAL_STREAM(rclcpp::get_logger("PatrolbotInterface"),"Something went wrong while closing the connection with the port " << port_);
        }
    }

}

CallbackReturn PatrolbotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info){

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

    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    last_run_ = rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> PatrolbotInterface::export_state_interfaces() {

        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION,
            &position_states_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY,
            &velocity_states_[i]));
        }

        return state_interfaces;
    }

std::vector<hardware_interface::CommandInterface> PatrolbotInterface::export_command_interfaces(){

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i= 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,&velocity_commands_[i]) );
    }
    
    return command_interfaces;
}


CallbackReturn PatrolbotInterface::on_activate(const rclcpp_lifecycle::State & previous_state) {

        RCLCPP_INFO(rclcpp::get_logger("PatrolbotInterface"),"starting robot hardware.........");
        velocity_commands_ = {0.0, 0.0, 0.0, 0.0};
        position_states_ = {0.0, 0.0, 0.0, 0.0};
        velocity_states_ = {0.0, 0.0, 0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("PatrolbotInterface"),"something went wrong while interacting with port "<< port_);
        return CallbackReturn::FAILURE;

        }

        RCLCPP_INFO(rclcpp::get_logger("PatrolbotInterface"),"hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }


CallbackReturn PatrolbotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state) {

    RCLCPP_INFO(rclcpp::get_logger("PatrolbotInterface"),"stoping robot hardware.........");
    if(arduino_.IsOpen())
    {
        try{
            arduino_.Close();
        }
        catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("PatrolbotInterface"),"something went wrong while closing the port "<< port_);
            return CallbackReturn::FAILURE;

        }
    }
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type PatrolbotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

    // RCLCPP_INFO(rclcpp::get_logger("PatrolbotInterface"), "reading");

    if (arduino_.IsDataAvailable()) {
        auto dt = (rclcpp::Clock().now() - last_run_).seconds();

        std::string message;
        arduino_.ReadLine(message);  // Read data from the serial port
        //RCLCPP_INFO(rclcpp::get_logger("PatrolbotInterface"), "Received message: %s", message.c_str());
        std::stringstream ss(message);
        std::string res;
        int multiplier = 1;  // Default multiplier

        // Split the message by commas
        while (std::getline(ss, res, ',')) {
            if (res.size() < 4) {
                // If the component has fewer than 4 characters, it's invalid
               // RCLCPP_WARN(rclcpp::get_logger("PatrolbotInterface"), "Skipping invalid data: %s", res.c_str());
                continue;
            }

            // Extract direction and value
            std::string direction = res.substr(0, 2);  // "fl", "fr", "rl", "rr"
            char sign = res.at(2);  // 'p' or 'n' (positive or negative)
            double value = std::stod(res.substr(3));  // The numeric value

            // Set the multiplier based on the sign
            multiplier = (sign == 'p') ? 1 : -1;

            // Store the value in the correct position based on the direction
            if (direction == "fl") {
                velocity_states_.at(0) = multiplier * value;
                position_states_.at(0) += velocity_states_.at(0) * dt;
            } else if (direction == "fr") {
                velocity_states_.at(1) = multiplier * value;
                position_states_.at(1) += velocity_states_.at(1) * dt;
            } else if (direction == "rl") {
                velocity_states_.at(2) = multiplier * value;
                position_states_.at(2) += velocity_states_.at(2) * dt;
            } else if (direction == "rr") {
                velocity_states_.at(3) = multiplier * value;
                position_states_.at(3) += velocity_states_.at(3) * dt;
            } else {
               // RCLCPP_WARN(rclcpp::get_logger("PatrolbotInterface"), "Unknown direction: %s", direction.c_str());
            }

            //RCLCPP_INFO(rclcpp::get_logger("PatrolbotInterface"), "Parsed data: %s", res.c_str());
        }

        last_run_ = rclcpp::Clock().now();
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PatrolbotInterface::write(const rclcpp::Time & time,const rclcpp::Duration & period) {

    std::stringstream message_stream;
    char front_left_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
    char front_right_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
    char rear_left_wheel_sign = velocity_commands_.at(2) >= 0 ? 'p' : 'n';
    char rear_right_wheel_sign = velocity_commands_.at(3) >= 0 ? 'p' : 'n';

    std::string compensate_zeros_front_left = "";
    std::string compensate_zeros_front_right = "";
    std::string compensate_zeros_rear_right = "";
    std::string compensate_zeros_rear_left = "";

    if(std::abs(velocity_commands_.at(0)) < 10.0){
        compensate_zeros_front_left = "0";
    }
    else{
        compensate_zeros_front_left = "";
    }

    if(std::abs(velocity_commands_.at(1)) < 10.0){
        compensate_zeros_front_right = "0";
    }
    else{
        compensate_zeros_front_right = "";
    }

    if(std::abs(velocity_commands_.at(2)) < 10.0){
        compensate_zeros_rear_left = "0";
    }
    else{
        compensate_zeros_rear_left = "";
    }

    if(std::abs(velocity_commands_.at(3)) < 10.0){
        compensate_zeros_rear_right = "0";
    }
    else{
        compensate_zeros_rear_right = "";
    }


    message_stream << std::fixed << std::setprecision(2) <<
    "Fl" << front_left_wheel_sign << compensate_zeros_front_left << std::abs(velocity_commands_.at(0)) <<
    ",Fr" << front_right_wheel_sign << compensate_zeros_front_right << std::abs(velocity_commands_.at(1)) << 
    ",Rl"<< rear_left_wheel_sign << compensate_zeros_rear_left << std::abs(velocity_commands_.at(2)) <<
    ",Rr" << rear_right_wheel_sign << compensate_zeros_rear_right << std::abs(velocity_commands_.at(3))<< ","; 

    try{
        arduino_.Write(message_stream.str());
        //RCLCPP_INFO_STREAM(rclcpp::get_logger("PatrolbotInterface"), "Message to Arduino: " << message_stream.str());    
        }
    catch(...){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("PatrolbotInterface"),"something went wrong while sending msg "<< message_stream.str() << "on the port " << port_);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}

#include<pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bot3_firmware::PatrolbotInterface,hardware_interface::SystemInterface)

