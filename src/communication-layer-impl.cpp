#include "EDScorbot.cpp"


#define DEFAULT_SLEEP 125000 //microseconds


int axi_address;
std::string config_file;
std::vector<std::string> joint_list;
MetaInfoObject mi;
pthread_t search_home_thread;
pthread_t move_to_point_thread;
pthread_t apply_trajectory_thread;

typedef struct{
	Point p;
	std::string config;
	int sleep;
} pthread_args,*ppthread_args;

static pthread_args args = {Point(),"",DEFAULT_SLEEP};

void update_pthread_args(ppthread_args args,std::string& config, int sleep, Point& p){
	
	args->config = config;
	args->p = p;
	args->sleep = sleep;

}

/**
 * A global variable maintaining the current point to move the arm
 * This point is initially empty and its value is updated when the user
 * requests via ARM_MOVE_TO_POINT signal. After the robotc arm is moved
 * its value is reset to an empty point again.
**/
Point current_point;

/**
 * A global variable maintaining the current trajectory to be executed
 * This trajectory is initially empty and its value is updated when the user
 * requests via ARM_APPLY_TRAJECTORY signal. After the trajectory is applied 
 * its value is reset to and empty trajectory again.
 **/
Trajectory current_trajectory;

/**
 * Global flags informing that a trajectory is being executed or cancelled
**/
bool executing_trajectory = false;
bool cancel_trajectory = false;

/** search_home_threaded_function - Description
 * Function to move the arm to home position. It must be executed into a
 * thread to avoid blocking the main process and disconnect from the broker.
 * Before calling this function all pre-conditions have already been validated
**/
void* search_home_threaded_function(void* arg){

  //Create output structure
	CommandObject output = CommandObject();
  output.signal = ARM_HOME_SEARCHED;
	output.client = owner;
	output.error = error_state;

	EDScorbot handler("/home/root/initial_config.json");

  handler.initJoints();
  //putenv("HOME_EXEC=1");

  //puts("J3");
  handler.searchHome(handler.j3,true);
  //puts("J2");
  handler.searchHome(handler.j2,true);
  //puts("J1");
  handler.searchHome(handler.j1,true);

  handler.searchHome(handler.j4,true);

  //puts("Waiting for PID to stabilize");
  usleep(15000000);
  //putenv("HOME_EXEC=0");
  EDScorbotJoint *joints[6] = {&handler.j1, &handler.j2, &handler.j3, &handler.j4, &handler.j5, &handler.j6};

  int i;
  for (i = 0; i < 4; i++)
  {
      handler.resetJPos(*joints[i]);
  }

	//publish message notifying that home has been reached
	publish_message(ROBOT_NAME_COMMANDS_TOPIC,output.to_json().dump().c_str());
	std::cout << "Home position reached" << std::endl;
	return NULL;

}

/** move_to_point_threaded_function - Description
 * Function to move the arm to a single point. It must be executed into a
 * thread to avoid blocking the main process and disconnect from the broker.
 * Before calling this function all pre-conditions have already been validated
 */
void* move_to_point_threaded_function(void* arg){

  //Lock-variables ON
  executing_trajectory = true;

  //Create output structure
	MovedObject output = MovedObject();
	output.client = owner;
	output.error = error_state;

  //Create robot instance
	EDScorbot handler("/home/root/initial_config.json");	//call the low level function to move to a single point considering 
	
	std::vector<double> coords = current_point.coordinates;
	int i;
	for(i = 0; i < 4 ; i++){
		int ref = angle_to_ref(i+1,coords[i]); //para pasar a RAD dandole DEG (lo mismo se cambia)
		handler.sendRef(ref,handler.joints[i]);
    std::cout << "ref: "  << ref << std::endl;

	}
  
  //int s =  (int) (current_point.coordinates.back());
  usleep(2000000);
  
  double angles[6];
	handler.readJoints(angles);
	Point reached;
	std::vector<double> reached_v;

	
	for(i = 0; i< 6;i++){
		reached_v.push_back(angles[i]);
	}

	reached.coordinates = reached_v;
	output.content = reached;

	//publish message notifying that the point has been reached
	publish_message(ROBOT_NAME_MOVED_TOPIC,output.to_json().dump().c_str());
	std::cout << "Arm moved to point " << output.content.to_json().dump().c_str() << std::endl;
  
  //Lock-variables OFF
  executing_trajectory = false;
  cancel_trajectory = false;

	//after publishing the message, the current_point must be re-instantiated with empty point
	current_point = Point();

	return NULL;
}

/** apply_trajectory_threaded_function - Description
 * Function to aply a trajectory. It must be executed into a
 * thread to avoid blocking the main process and disconnect from the broker.
 * Before calling this function all pre-conditions have already been validated
 */
void* apply_trajectory_threaded_function(void* arg){

	//points to be considered come from the global variable "current_trajectory"
	std::vector<Point> points = std::vector<Point>(current_trajectory.points);
	//ppthread_args traj_args = reinterpret_cast<ppthread_args>(arg);
	//std::string config = traj_args->config;
	//int sleep = traj_args->sleep;
  //Create output structure
  MovedObject output = MovedObject();
	output.client = owner;
	output.error = error_state;

  //Lock-variables ON
	executing_trajectory = true;
	EDScorbot handler("/home/root/initial_config.json");
	int i;
	int ref;


  //Moving robot to each point of the trajectory
  while (!points.empty() && !cancel_trajectory){

    Point p = points.front();

    std::cout << "Moving to point " << p.to_json().dump().c_str() << std::endl;
		//move_to_point(p,); 

		
		for(i = 0; i< 4;i++){
			ref = angle_to_ref(i+1,p.coordinates[i]);
			handler.sendRef(ref,handler.joints[i]);
		}

		int s =  (int) (p.coordinates.back());
		usleep(s*1000);

		double angles[6];
		handler.readJoints_angle(angles);
		Point reached;

		std::vector<double> reached_v;
	
		for(i = 0; i< 6;i++){
			reached_v.push_back(angles[i]);
		}
		reached.coordinates = reached_v;
		output.content = reached;

    //publish message notifying that the point has been reached
    int err = publish_message(ROBOT_NAME_MOVED_TOPIC,output.to_json().dump().c_str());
    std::cout << "Arm moved to point " << output.content.to_json().dump().c_str() << std::endl;

    
		points.erase(points.begin());
	}

  std::cout << "Fin bucle" << std::endl;

  //Lock-variables OFF
	executing_trajectory = false;
  cancel_trajectory = false;

	//clean the current trajectory variable
	current_trajectory = Trajectory();

	return NULL;
}

/** parse_arguments - Description
 * Robotanno needs some initial parameters to be defined, such as the base address for the registers, an initial configuration file
 * and others that are not required such as trajectory files or an ip address.
 */
void parse_arguments(int argc, char *argv[]){

  // Declare parser
 	argparse::ArgumentParser parser("mocked_server");
    parser.add_argument("-c", "--config_file").help("Optional. Configuration file in JSON format. This file can be used to configure each joint's controller parameters. Default is 'initial_config.json'").default_value(std::string("/home/root/initial_config.json"));
    parser.add_argument("-v", "--verbose").help("Increase verbosity of output").default_value(false).implicit_value(true);

    try
    {
        parser.parse_args(argc, argv);
    }
    catch (const std::runtime_error &err)
    {
        std::cerr << err.what() << std::endl;
        std::cerr << parser;
        std::exit(1);
    }

    std::string config_file = parser.get<std::string>("--config_file");
    bool verbose = parser.get<bool>("--verbose");

}

/** generate_metainfo - Description
 * Function that will assign metainfo values to metainfo clase variable "mi" and take the number of joints out
 * for the robot to use it as input for its joints.
**/
void generate_metainfo(void)
{
  //Generate information that will be sent via topic metainfo
  mi = initial_metainfo();
  mi.signal = ARM_METAINFO;
  mi.name = "EDScorbot";

  //Take number of joints and elaborate a list for the robot to recognize it
  joint_list.resize(mi.mi_joints.size());

	for (int Jidx = 0; Jidx < mi.mi_joints.size(); Jidx++){
		joint_list[Jidx]=mi.mi_joints[Jidx].name;
	} 

}

/** generate_metainfo - Description
 * Function that will read the mqtt message and extract the command number from it
**/
int extract_signal(std::string message)
{
    int result = 0;
    json json_obj = json::parse(message);

    result = json_obj["signal"];

    return result;
}


class CommunicationLayerImpl: public CommunicationLayer
{
public:
  void handle_metainfo_topic(const struct mosquitto_message *message)
  {
    MetaInfoObject::from_json_string((char *)message->payload);

    //I dunno if this is necessary 
    int sig = extract_signal((char *)message->payload);
    MetaInfoObject output = MetaInfoObject();
    MetaInfoObject receivedMetainfo = MetaInfoObject::from_json_string((char *)message->payload);
    switch (sig)
    {
      case ARM_GET_METAINFO: //user requested arm status
        output=mi;
		    std::cout << "Request metainfo received. "
				  << " Sending metainfo " 
				  << output.to_json().dump().c_str() << std::endl;
		    publish_message(METAINFO_TOPIC, output.to_json().dump().c_str());
		  break;
    }
  }
  void handle_robot_name_commands_topic(const struct mosquitto_message *message)
  {
    CommandObject::from_json_string((char *)message->payload);
    
    //obtains the signal/code
    int sig = extract_signal((char *)message->payload);

    //the answer
    CommandObject output = CommandObject();

    //parses the received command from the payload
    CommandObject receivedCommand = CommandObject::from_json_string((char *)message->payload);

    switch (sig)
    {
      case ARM_CHECK_STATUS: //user requested arm status
        output.signal=ARM_STATUS;
		    std::cout << "Request status received. Sending payload " << output.to_json().dump().c_str() << std::endl;
		    publish_message(ROBOT_NAME_COMMANDS_TOPIC, output.to_json().dump().c_str());
		  break;

      case ARM_CONNECT: //user wants to connect to the arm to become the owner
        std::cout << "Request to connect received: "<< receivedCommand.to_json().dump().c_str() << std::endl;

        if (!owner.is_valid())
        {
          owner = receivedCommand.client;
          output.signal = ARM_CONNECTED;
          output.client = owner;

          std::cout 	<< "Arm's owner is " << owner.id << std::endl << "client is "<< receivedCommand.client.id << std::endl;

          publish_message(ROBOT_NAME_COMMANDS_TOPIC, output.to_json().dump().c_str());

          int err = pthread_create(&search_home_thread, NULL, &search_home_threaded_function, NULL);
          pthread_detach(search_home_thread);

        }
        else
        {
          std::cout << "Connection refused. Arm is busy." << std::endl;
        }
      break;

      case ARM_MOVE_TO_POINT: //user requested to move the arm to a single point
        std::cout << "Move to point request received. " << std::endl;

        if (owner.is_valid())
        {
          std::cout << "Owner is valid" << std::endl;

          Client client = receivedCommand.client;
          if (owner.id == client.id) // only owner can do that
          {
            /*
            std::cout << "Owner is Client" << std::endl;
            MovedObject output = MovedObject();
            Point P_target = receivedCommand.point;
            output.client = owner;
            output.error = error_state;
            //sets the content of the answer - Point format
            output.content = robot.MvToPoint_Class(P_target);
            //publish message notifying that the point has been published
            publish_message(ROBOT_NAME_MOVED_TOPIC,output.to_json().dump().c_str());
            */
            if (!receivedCommand.point.is_empty() && !executing_trajectory && !cancel_trajectory){
              current_point = receivedCommand.point;
              int err = pthread_create(&move_to_point_thread, NULL, &move_to_point_threaded_function, NULL);
              pthread_detach(move_to_point_thread);

              // handle err?
            }

          }
          else
          {
            std::cout << "Owner is not client" << std::endl;
            // other client is trying to move the arm ==> ignore
          }
        }
        else
        {
          std::cout << "Owner is not valid" << std::endl;
          // arm has no owner ==> ignore
        }
        
      break;

      case ARM_APPLY_TRAJECTORY:  //user requested to apply a trajectory
        std::cout << "Apply trajectory request received. " << std::endl;
        if (owner.is_valid())
        {
          Client client = receivedCommand.client;
          if (owner.id == client.id) //only owner can do that
          {
            /*
            Trajectory T_target = receivedCommand.trajectory;
            MovedObject output = MovedObject();

            ////sets the content of the answer - Trayectory format
            //output.content = robot.Trajectory_Class(T_target);

            //sets the content of the answer - Point format
            for (int Numpoint = 0; Numpoint < T_target.points.size(); Numpoint++)
            {
              output.content = robot.MvToPoint_Class(T_target.points[Numpoint]);
              int err = publish_message(ROBOT_NAME_MOVED_TOPIC,output.to_json().dump().c_str());
              //std::cout << "Published:  " << output.to_json().dump().c_str() << err << std::endl;
            }
            */
           std::cout << "Empezar trayectoria" << std::endl;
            if (!executing_trajectory){
              current_trajectory = receivedCommand.trajectory;
				      int err = pthread_create(&apply_trajectory_thread, NULL, &apply_trajectory_threaded_function, NULL);
				      pthread_detach(apply_trajectory_thread); 
            }


          }
          else
          {
            std::cout << "Owner is not client" << std::endl;
            // other client is trying to move the arm ==> ignore
          }
        }
        else
        {
          std::cout << "Owner is not valid" << std::endl;
          // arm has no owner ==> ignore
        }
        
      break;

      //TODO
      case ARM_CANCEL_TRAJECTORY: //user requested to cancel trajectory execution
        std::cout << "Cancel trajectory received. " << std::endl;
        if (owner.is_valid())
        {
          Client client = receivedCommand.client;
          if (owner.id == client.id) // only owner can do that
          {
            cancel_trajectory = true;
            output.signal = ARM_CANCELED_TRAJECTORY;
            output.client = owner;
            output.error = error_state;

            publish_message(ROBOT_NAME_COMMANDS_TOPIC, output.to_json().dump().c_str());
            std::cout << "Trajectory cancelled. " << std::endl;
          }
        }
        else
        {
          // arm has no owner ==> ignore
        }
      break;

      case ARM_DISCONNECT: //user requested to disconnect from the arm
        Client client = receivedCommand.client;
        std::cout << "Request disconnect received: " << receivedCommand.to_json().dump().c_str() << std::endl;
        if (owner.id == client.id) // only owner can do that
        {
          owner = Client();
          output.signal = ARM_DISCONNECTED;
          output.client = client;

          publish_message(ROBOT_NAME_COMMANDS_TOPIC, output.to_json().dump().c_str());
          std::cout 	<< "Client disconnected " << output.to_json().dump().c_str() << std::endl;
        }

        int err = pthread_create(&search_home_thread, NULL, &search_home_threaded_function, NULL);
        pthread_detach(search_home_thread);

      break;
    }
  }
  void handle_robot_name_moved_topic(const struct mosquitto_message *message)
  {
    MovedObject::from_json_string((char *)message->payload);
    //TODO implement your business code
    //std::cout << "handle RbtAnno/moved topic" << std::endl;
  }
  
};

CommunicationLayerImpl impl;

void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
  if (std::strcmp(message->topic,METAINFO_TOPIC.c_str()) == 0)
  {
    impl.handle_metainfo_topic(message);
  }
  else if (std::strcmp(message->topic,ROBOT_NAME_COMMANDS_TOPIC.c_str()) == 0)
  {
    impl.handle_robot_name_commands_topic(message);
  }
  else if (std::strcmp(message->topic,ROBOT_NAME_MOVED_TOPIC.c_str()) == 0)
  {
    impl.handle_robot_name_moved_topic(message);
  }
  
}

//Moverse a un punto:
/*
{
	"signal":7,
	"client":{"id":"root"},
	"point":{"coordinates":[150,0,0,0,0,0,500]}
}
*/

//Ejecutar trayectoria:
/*
{
	"signal":8,
	"client":{"id":"root"},
	"trajectory":
	{
		"points":
		[
			{"coordinates":[150,0,0,0,0,0,500]},
			{"coordinates":[130,0,0,0,0,0,500]}
		]
	}
}
*/

//Test en robot
/*

void robot_test(void){

  mi = initial_metainfo();
  //std::vector<std::string> joint_list;
  joint_list.resize(mi.mi_joints.size());

	for (int Jidx = 0; Jidx < mi.mi_joints.size(); Jidx++){
		joint_list[Jidx]=mi.mi_joints[Jidx].name;
	} 


  std::cout << "Creando instancia robot..." << std::endl;
  Rbtanno robot = Rbtanno(axi_address, config_file);
  std::cout << "Configuring Joints..." << std::endl;
  robot.config_joints(joint_list);
  std::cout << "Joints configured!" << std::endl;
  
  //Primero movemos a un punto
  Point Punto_Objetivo;
  Punto_Objetivo.coordinates = {90,-60,30,-15,-5,0,2000};
  std::cout << "Punto_Objetivo_COMMAND:" << Punto_Objetivo.to_json().dump().c_str() << std::endl;
  Punto_Objetivo = robot.MvToPoint_Class(Punto_Objetivo);
  std::cout << "Punto_Objetivo_MOVED:" << Punto_Objetivo.to_json().dump().c_str() << std::endl;

  Punto_Objetivo.coordinates = {45,-45,45,-45,-45,0,2000};
  std::cout << "Punto_Objetivo_COMMAND:" << Punto_Objetivo.to_json().dump().c_str() << std::endl;
  Punto_Objetivo = robot.MvToPoint_Class(Punto_Objetivo);
  std::cout << "Punto_Objetivo_MOVED:" << Punto_Objetivo.to_json().dump().c_str() << std::endl;

  //Por ahora, el punto que se genera para enviarse por moved es el movimiento relativo en grados efectuado por el robot.

  
  //Luego movemos una trayectoria
  Trajectory Trayectoria_objetivo;
  Point Punto_uno;
  Point Punto_dos;
  Point Punto_tres;
  Punto_uno.coordinates ={-20,10,0,-20,0,0,2000};
  Punto_dos.coordinates ={-20,-40,0,-20,0,0,2000};
  Punto_tres.coordinates ={-20,10,0,-20,0,0,2000};
  Trayectoria_objetivo.points = {Punto_uno,Punto_dos,Punto_tres};

  std::cout << "Trayectoria_objetivo:" << Trayectoria_objetivo.to_json().dump().c_str() << std::endl;
  robot.Trajectory_Class(Trayectoria_objetivo);
  
  //Por ultimo nos vamos a home
  std::cout << "Searching home..." << std::endl;
  robot.home();
  std::cout << "Home searched" << std::endl;

}
*/