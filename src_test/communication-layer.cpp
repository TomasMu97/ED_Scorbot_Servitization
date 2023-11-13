#include "topics.cpp"

int publish_message(std::string topic, const char *buf)
{
  char *payload = (char *)buf;
  int rc = mosquitto_publish(mosq, NULL, topic.c_str(), strlen(payload), payload, 1, false);
  return rc;

}

class CommunicationLayer
{
public:
  virtual void handle_metainfo_topic(const struct mosquitto_message *message) = 0;
  virtual void handle_robot_name_commands_topic(const struct mosquitto_message *message) = 0;
  virtual void handle_robot_name_moved_topic(const struct mosquitto_message *message) = 0;
};