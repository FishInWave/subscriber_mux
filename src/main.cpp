#include "subscriber_mux/subscriber_mux.hpp"

int main(int argc,char** argv){
    ros::init(argc,argv,"subscriber_mux");
    SubscriberMux sm;
    thread run_thread(&SubscriberMux::run,&sm);
    ros::MultiThreadedSpinner multispinner(14);
    multispinner.spin();

    run_thread.join();

    return 0;
}