#include "metainfo.cpp"
#include "mosquitto.h"
#include <iostream>

struct mosquitto *mosq;

std::string METAINFO_TOPIC = "metainfo";
std::string ROBOT_NAME_COMMANDS_TOPIC = "EDScorbot/commands";
std::string ROBOT_NAME_MOVED_TOPIC = "EDScorbot/moved";


void subscribe_all_topics()
{
  mosquitto_subscribe(mosq,NULL,METAINFO_TOPIC.c_str(),0);
  mosquitto_subscribe(mosq,NULL,ROBOT_NAME_COMMANDS_TOPIC.c_str(),0);
  mosquitto_subscribe(mosq,NULL,ROBOT_NAME_MOVED_TOPIC.c_str(),0);

  std::cout << "Subscribed on topics: " << std::endl
    << "  " + METAINFO_TOPIC<< std::endl
    << "  " + ROBOT_NAME_COMMANDS_TOPIC<< std::endl
    << "  " + ROBOT_NAME_MOVED_TOPIC<< std::endl;

}