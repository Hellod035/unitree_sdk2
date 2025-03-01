#include "mailbox.hpp"
#include "simple_writer.hpp"
#include <typeinfo>
#include <concepts>

template<typename T>
concept ProcessData = requires(T t){
    {t.timestamp} -> std::convertible_to<uint64_t>;
    {t.timeperiod} -> std::convertible_to<int64_t>;
    {t.messageName()}-> std::same_as<std::string>;
    {t.fields()} ->std::same_as<std::vector<ulog_cpp::Field>>;
};




template<ProcessData P, size_t NUM_BUFFER>
class logger
{
private:
    /* data */
    std::unique_ptr<ulog_cpp::SimpleWriter> writer_;
    uint16_t msg_data_id_;
    Mailbox<P, NUM_BUFFER> mb_process_data_;
    std::chrono::steady_clock::time_point last_tp_; 
    int64_t timeperiod_;
    bool calc_period_flag_ = false;

    static inline auto nano_now(void)
    {
        auto t = timing::now();
        return t.sec()*1000*1000+t.nsec()/1000;
    }




public:
    logger(){
        last_tp_ = std::chrono::steady_clock::now();
    }

    void setup_writer(void){
        std::string filename = "log_"+ std::to_string(nano_now())+".ulg";
        writer_ = std::make_unique<ulog_cpp::SimpleWriter>(filename, nano_now());
        writer_->writeInfo("sys_name", P::messageName());
        writer_->writeMessageFormat(P::messageName(), P::fields());
        writer_->headerComplete();
        msg_data_id_ = writer_->writeAddLoggedMessage(P::messageName());

    }

    bool send_data(P data){
        data.timestamp = nano_now();
        if(calc_period_flag_){
            data.timeperiod = timeperiod_;
            calc_period_flag_ = false;
        }else{
            data.timeperiod = -1;
        }
        auto p_tmp = mb_process_data_.take();
        if(p_tmp){
            *p_tmp = data;
            mb_process_data_.send(p_tmp);
            return true;
        }
        return false;
    }

    void calc_period(std::chrono::steady_clock::time_point tp){
        auto period = std::chrono::duration<int64_t,std::nano>(tp-last_tp_);
        timeperiod_ = period.count();
        last_tp_ = tp;
        calc_period_flag_ = true;
    }

    void log_data(){
        while(true){
            auto p_tmp = mb_process_data_.try_recv();
            if(!p_tmp) break;
            writer_->writeData(msg_data_id_, *p_tmp);
            mb_process_data_.drop(p_tmp);
        }
    }

    auto GetTimestamp() -> uint64_t{
        return nano_now();
    }


    ~logger(){
        //empty
    }
};


