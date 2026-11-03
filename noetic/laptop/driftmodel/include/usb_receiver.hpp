#ifndef USB_RECEIVER_HPP
#define USB_RECEIVER_HPP

class USBReceiver {
public:
    USBReceiver();
    ~USBReceiver();
    
    void receiveData();
    void transmitData(float steer_, float throttle_);

    static float steerVal();
    static float throttleVal();  
    static float brakeVal();
    static bool flagVal_mode();  
    static bool flagVal_punishment(); 
        
private:
    std::string received_data;
    static float steer;
    static float throttle;
    static float brake;
    static bool mode_flag;
    static bool punishment_flag;
};

#endif  // USB_RECEIVER_HPP
