
#include "task_manager.hpp"
template <typename Message, typename Receiver>
class TaskLogger 
{
public:
    static TaskLogger() {
        msg_idx = 0;
    }
    void setIdx(uint num) {
        msg_idx = num;
    }

    Message sendMessage (Message msg) {
        Receiver.getMessage(msg); 
        msgs.push_back(msg);
        msg_idx++;       
    }
private:
    uint msg_idx;
    std::vector<Message> msgs;


};