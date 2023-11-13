#include <string>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

enum CommandsSignal {
  ARM_CHECK_STATUS = 3, 
  ARM_STATUS = 4, 
  ARM_CONNECT = 5, 
  ARM_CONNECTED = 6, 
  ARM_MOVE_TO_POINT = 7, 
  ARM_APPLY_TRAJECTORY = 8, 
  ARM_CANCEL_TRAJECTORY = 9, 
  ARM_CANCELED_TRAJECTORY = 10, 
  ARM_DISCONNECT = 11, 
  ARM_DISCONNECTED = 12, 
  ARM_HOME_SEARCHED = 13
};

enum MetaInfoSignal {
  ARM_GET_METAINFO = 1, 
  ARM_METAINFO = 2
};

class Client
{
public:
  std::string id;

  Client()
  {
    this->id = "";
  }

  json to_json()
  {
    json result;
    result["id"] = this->id;

    return result;
  }

  static Client from_json(json json_obj)
  {
    return from_json_string(json_obj.dump());
  }

  static Client from_json_string(std::string json_string)
  {
    json json_obj = json::parse(json_string);

    Client result = Client();
    if(!json_obj["id"].is_null())
    {
      result.id = json_obj["id"];
    }


    return result;
  }

  /**
   * Method to validate a client. Valid clients have some
   * content into their id attribute
   **/
  bool is_valid()
  {
      bool ret = (strcmp(this->id.c_str(), "") != 0);
      return ret;
  }

};

/**
 * A global variable meaning the owner of the arm.
 **/
static Client owner = Client();

/**
 * A global flag representing is the arm is in a error state or not.
 * This allows one to notify clients that an internal problem has
 * happened with the arm. The controller implementation should
 * consider mechanisms of detecting arm's errors to update this value,
 * as well as some efficient ways to recover from the failure
 **/
static bool error_state = false;

class Point
{
public:
  std::vector<double> coordinates;
  double velocity;

  Point()
  {
    this->coordinates = std::vector<double>();
    this->velocity = 0.0;
  }

  /**
   * Method to say if a point is empty (has coordinates) or not
   **/
  bool is_empty()
  {
      int size = coordinates.size();
      return coordinates.size() == 0;
  }

  json to_json()
  {
    json result;
    json coordinates_json = json::array();
    if(coordinates.size() > 0)
    {
      for(double item:coordinates)
      {
        coordinates_json.push_back(item);
      }
    }
    result["coordinates"] = coordinates_json;

    return result;
  }

  static Point from_json(json json_obj)
  {
    return from_json_string(json_obj.dump());
  }

  static Point from_json_string(std::string json_string)
  {
    json json_obj = json::parse(json_string);

    Point result = Point();
    json coordinates_json = json_obj["coordinates"];
    for (json::iterator it = coordinates_json.begin(); it != coordinates_json.end(); ++it)
    {
      double item = it.value();
      result.coordinates.push_back(item);
    }

    return result;
  }
};

class JointInfo
{
public:
  double minimum;
  double maximum;
  std::string name;

  JointInfo()
  {
    this->minimum = 0.0;
    this->maximum = 0.0;
    this->name = "";
  }

  json to_json()
  {
    json result;
    result["minimum"] = this->minimum;
    result["maximum"] = this->maximum;

    return result;
  }

  static JointInfo from_json(json json_obj)
  {
    return from_json_string(json_obj.dump());
  }

  static JointInfo from_json_string(std::string json_string)
  {
    json json_obj = json::parse(json_string);

    JointInfo result = JointInfo();
    if(!json_obj["minimum"].is_null())
    {
      result.minimum = json_obj["minimum"];
    }

    if(!json_obj["maximum"].is_null())
    {
      result.maximum = json_obj["maximum"];
    }


    return result;
  }
};

class Trajectory
{
public:
  std::vector<Point> points;

  Trajectory()
  {
    this->points = std::vector<Point>();
  }

  json to_json()
  {
    json result;
    json points_json = json::array();
    if(points.size() > 0)
    {
      for(Point item:points)
      {
        points_json.push_back(item.to_json());
      }
    }
    result["points"] = points_json;

    return result;
  }

  static Trajectory from_json(json json_obj)
  {
    return from_json_string(json_obj.dump());
  }

  static Trajectory from_json_string(std::string json_string)
  {
    json json_obj = json::parse(json_string);

    Trajectory result = Trajectory();
    json points_json = json_obj["points"];
    for (json::iterator it = points_json.begin(); it != points_json.end(); ++it)
    {
      Point item = Point::from_json(it.value());
      result.points.push_back(item);
    }

    return result;
  }
};

class MetaInfoObject
{
public:
  int signal;
  std::string name;
  std::vector<JointInfo> mi_joints;

  MetaInfoObject()
  {
    this->signal = 0;
    this->name = "";
    this->mi_joints = std::vector<JointInfo>();
  }

  json to_json()
  {
    json result;
    result["signal"] = this->signal;
    result["name"] = this->name;
    json joints_json = json::array();
    if(mi_joints.size() > 0)
    {
      for(JointInfo item:mi_joints)
      {
        joints_json.push_back(item.to_json());
      }
    }
    result["joints"] = joints_json;

    return result;
  }

  static MetaInfoObject from_json(json json_obj)
  {
    return from_json_string(json_obj.dump());
  }

  static MetaInfoObject from_json_string(std::string json_string)
  {
    json json_obj = json::parse(json_string);

    MetaInfoObject result = MetaInfoObject();
    if(!json_obj["signal"].is_null())
    {
      result.signal = json_obj["signal"];
    }

    if(!json_obj["name"].is_null())
    {
      result.name = json_obj["name"];
    }

    json joints_json = json_obj["joints"];
    for (json::iterator it = joints_json.begin(); it != joints_json.end(); ++it)
    {
      JointInfo item = JointInfo::from_json(it.value());
      result.mi_joints.push_back(item);
    }

    return result;
  }
};

class CommandObject
{
public:
  int signal;
  Client client;
  bool error;
  Point point;
  Trajectory trajectory;

  CommandObject()
  {
    this->signal = 0;
    this->client = Client();
    this->error = false;
    this->point = Point();
    this->trajectory = Trajectory();
  }

  json to_json()
  {
    json result;
    result["signal"] = this->signal;
    result["client"] = this->client.to_json();
    result["error"] = this->error;
    result["point"] = this->point.to_json();
    result["trajectory"] = this->trajectory.to_json();

    return result;
  }

  static CommandObject from_json(json json_obj)
  {
    return from_json_string(json_obj.dump());
  }

  static CommandObject from_json_string(std::string json_string)
  {
    json json_obj = json::parse(json_string);

    CommandObject result = CommandObject();
    if(!json_obj["signal"].is_null())
    {
      result.signal = json_obj["signal"];
    }

    result.client = Client::from_json(json_obj["client"]);
    if(!json_obj["error"].is_null())
    {
      result.error = json_obj["error"];
    }

    result.point = Point::from_json(json_obj["point"]);
    result.trajectory = Trajectory::from_json(json_obj["trajectory"]);

    return result;
  }
};

class MovedObject
{
public:
  Client client;
  bool error;
  //Trajectory content;
  Point content;

  MovedObject()
  {
    this->client = Client();
    this->error = false;
    //this->content = Trajectory();
    this->content = Point();
  }

  json to_json()
  {
    json result;
    result["client"] = this->client.to_json();
    result["error"] = this->error;
    result["content"] = this->content.to_json();
    //result["Trajectory_content"] = this->Trajectory_content.to_json();

    return result;
  }

  static MovedObject from_json(json json_obj)
  {
    return from_json_string(json_obj.dump());
  }

  static MovedObject from_json_string(std::string json_string)
  {
    json json_obj = json::parse(json_string);

    MovedObject result = MovedObject();
    result.client = Client::from_json(json_obj["client"]);
    if(!json_obj["error"].is_null())
    {
      result.error = json_obj["error"];
    }

    result.content = Point::from_json(json_obj["content"]);
    //result.Trajectory_content = Trajectory::from_json(json_obj["Trajectory_content"]);

    return result;
  }
};

