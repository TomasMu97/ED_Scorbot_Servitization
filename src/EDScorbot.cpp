#include "EDScorbot.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <chrono>
#include "devmem.hpp"
using namespace std;
using json = nlohmann::json;

static int j1_t, j2_t, j3_t, j4_t, j5_t, j6_t;

// EDScorbot::~EDScorbot()
// {
// #ifdef THREADED

//     stopRead();

// #endif
// }

// Constructor
// string config_path -> relative path to json configuration file
// Initializes configuration for SPID controllers of EDScorbot

EDScorbot::EDScorbot(string config_path)
{

    /*
    ***********************************
        Joint configuration
    ***********************************
    */
    // Open config file
    ifstream config_file(config_path, ios::in);
    // Parse with json library
    json config = json::parse(config_file);
    // Take what we need -- check initial_config.json to see data structure
    json motor_config = config["Motor Config"];

    // Automatic configuration for SPID controllers
    // We iterate through the json motor config dictionary
    for (const auto &[k, v] : motor_config.items())
    {
        // Check to which motor the key corresponds
        bool m1, m2, m3, m4, m5, m6;
        // Keys have format XX_XX_...._MJ, with J ={1,2,3,4,5,6}
        m1 = (k.find("M1") != std::string::npos);
        m2 = (k.find("M2") != std::string::npos);
        m3 = (k.find("M3") != std::string::npos);
        m4 = (k.find("M4") != std::string::npos);
        m5 = (k.find("M5") != std::string::npos);
        m6 = (k.find("M6") != std::string::npos);

        // Remove MJ from keys, internal keys for the controller are motor-agnostic
        string subk = k.substr(0, k.size() - 3);
        // Assign each key its corresponding value
        if (m1)
        {
            j1.controller[subk] = v;
        }

        if (m2)
        {
            j2.controller[subk] = v;
        }

        if (m3)
        {
            j3.controller[subk] = v;
        }

        if (m4)
        {
            j4.controller[subk] = v;
        }

        if (m5)
        {
            j5.controller[subk] = v;
        }

        if (m6)
        {
            j6.controller[subk] = v;
        }
    }
    // Create array to iterate over joints
    std::array<EDScorbotJoint *, 6> joints = {{&j1, &j2, &j3, &j4, &j5, &j6}};
    int i = 1;
    for (const auto &j : joints)
    { // Assign each joint its correct address -- 0x00,0x20,...,0xA0
        j->address = addresses[j->id];
#ifdef EDS_VERBOSE
        // print controller configuration if verbose

        cout << "J" << i << "\n";
        i = i + 1;
        for (const auto &[k, v] : j->controller)
        {
            cout << "Key: " << k << " Valor: " << v << "\n";
        }
#endif
    }

    /*
***********************************
    BRAM config for write/read
***********************************
*/

    int *bram_ptr = open_devmem();
    this->bram_ptr = bram_ptr;
};

void EDScorbot::sendAngle(double ang, EDScorbotJoint j){
    int ref = angle_to_ref(j.jnum,ang);
    sendRef(ref, j);
    return ;
}

int EDScorbot::sendRef(int ref, EDScorbotJoint j)
{
    uchar offset = 0x02;
    uchar address = j.address + offset;
    uchar b1 = ((ref >> 8) & 0xFF);
    uchar b2 = ref & 0xFF;
    sendCommand16(address, b1, b2, this->bram_ptr);
    return 0;
}

void EDScorbot::initJoints() // Equivalente a configurespid sin las referencias
{

    EDScorbotJoint *joints[6] = {&j1, &j2, &j3, &j4, &j5, &j6};
    int data, base;
    // podria desenrollarse el bucle
    for (int i = 0; i < 6; i++)
    {

        base = 0x00000000 | (i * JOINT_STEP) << 16;

        data = base | PI_FD_ENABLE_ADDR << 16 | 0x0f; //|0x00 << 8
        this->bram_ptr[0] = data;

        data = base | PI_FD_ENABLE_ADDR << 16 | 0x03; //|0x00 << 8
        this->bram_ptr[0] = data;

        data = base | PI_FD_ADDR << 16 | ((joints[i]->controller["PI_FD_bank3_18bits"] >> 8) & 0xFF) << 8 | (joints[i]->controller["PI_FD_bank3_18bits"] & 0xFF);
        this->bram_ptr[0] = data;
#ifdef EDS_VERBOSE
        printf("J%d PI_FD: %d\n", i + 1, data);
#endif
        data = base | PD_FD_ENABLE_ADDR << 16 | 0x0f; //|0x00 << 8
        this->bram_ptr[0] = data;

        data = base | PD_FD_ENABLE_ADDR << 16 | 0x03; //|0x00 << 8
        this->bram_ptr[0] = data;

        data = base | PD_FD_ADDR << 16 | ((joints[i]->controller["PD_FD_bank3_22bits"] >> 8) & 0xFF) << 8 | (joints[i]->controller["PD_FD_bank3_22bits"] & 0xFF);
        this->bram_ptr[0] = data;
#ifdef EDS_VERBOSE
        printf("J%d PD_FD: %d\n", i + 1, data);
#endif

        data = base | EI_FD_ENABLE_ADDR << 16 | 0x0f; //|0x00 << 8
        this->bram_ptr[0] = data;

        data = base | EI_FD_ENABLE_ADDR << 16 | 0x03; //|0x00 << 8
        this->bram_ptr[0] = data;

        data = base | EI_FD_ADDR << 16 | ((joints[i]->controller["EI_FD_bank3_18bits"] >> 8) & 0xFF) << 8 | (joints[i]->controller["EI_FD_bank3_18bits"] & 0xFF);
        this->bram_ptr[0] = data;

#ifdef EDS_VERBOSE
        printf("J%d EI_FD: %d\n", i + 1, data);
#endif
        data = base | SPIKE_EXPANSOR_ADDR << 16 | ((joints[i]->controller["spike_expansor"] >> 8) & 0xFF) << 8 | (joints[i]->controller["spike_expansor"] & 0xFF);
        this->bram_ptr[0] = data;

#ifdef EDS_VERBOSE
        printf("J%d spike expansor: %d\n", i + 1, data);

#endif
        this->sendRef(0, *joints[i]);

        // send ref = 0 to stay put
        // data = base | REF_ADDR << 16;
    }
};

#ifdef THREADED

void EDScorbot::read_threaded()
{
    assert(exec);
    while (exec)
    {
        j1_t = this->bram_ptr[1];
        j2_t = this->bram_ptr[2];
        j3_t = this->bram_ptr[3];
        j4_t = this->bram_ptr[4];
        j5_t = this->bram_ptr[5];
        j6_t = this->bram_ptr[6];
        this_thread::sleep_for(chrono::microseconds(1));
    }
}

void EDScorbot::readJoints()
{
    t2 = std::thread(&EDScorbot::read_threaded, this);
    this->t = &t2;
}

void EDScorbot::stopRead()
{
    this->exec = false;
    this->t.join();
}

#else

void EDScorbot::readJoints_counter(int *ret)
{
    // int base_address = 0x00;//To be defined
    // int offset = 0x20;

    // int j1, j2, j3, j4, j5, j6;

    // j1 = this->bram_ptr[1];
    // j2 = this->bram_ptr[2];
    // j3 = this->bram_ptr[3];
    // j4 = this->bram_ptr[4];
    // j5 = this->bram_ptr[5];
    // j6 = this->bram_ptr[6];

    ret[0] = this->bram_ptr[1];
    ret[1] = this->bram_ptr[2];
    ret[2] = this->bram_ptr[3];
    ret[3] = this->bram_ptr[4];
    ret[4] = this->bram_ptr[5];
    ret[5] = this->bram_ptr[6];

    // std::array<int, 6> ret = {j1, j2, j3, j4, j5, j6};
    // int reads[6]= {j1,j2,j3,j4,j5,j6};
    // ret[0] = j1;
    // ret[1] = j2;
    // ret[2] = j3;
    // ret[3] = j4;
    // ret[4] = j5;
    // ret[5] = j6;
    // return ret;
};


void EDScorbot::readJoints_angle(double *ret)
{
    // int base_address = 0x00;//To be defined
    // int offset = 0x20;

    // int j1, j2, j3, j4, j5, j6;

    // j1 = this->bram_ptr[1];
    // j2 = this->bram_ptr[2];
    // j3 = this->bram_ptr[3];
    // j4 = this->bram_ptr[4];
    // j5 = this->bram_ptr[5];
    // j6 = this->bram_ptr[6];

    int j[6];

    
    
    for(int i = 0; i < 6; i++)
    {
        j[i] = this->bram_ptr[i+1];
        if(i<4)
            ret[i] = count_to_angle(i+1,j[i]);

    }
    // std::array<int, 6> ret = {j1, j2, j3, j4, j5, j6};
    // int reads[6]= {j1,j2,j3,j4,j5,j6};
    // ret[0] = j1;
    // ret[1] = j2;
    // ret[2] = j3;
    // ret[3] = j4;
    // ret[4] = j5;
    // ret[5] = j6;
    // return ret;
};


void EDScorbot::readJoints(double* ret){

    readJoints_angle(ret);
}


#endif

// Un poco mas bonito
//  static std::map<std::string, int> polarities = {
//          {1, 1},
//          {2, -1},
//          {3, -1},
//          {4, -1},
//          {5, -1},
//          {6, -1}};

// Polarities per joint are: 1,-1,-1,-1,-1,-1
void EDScorbot::searchHome(EDScorbotJoint j, bool v=false)
{
    // Un poco mas rapido
    int pol = (j.jnum < 4 ? 1 : -1);

    int old_sj = 0x20000 / 4; // 32768
    int sj = 0x20000 / 4;     // 32768
    int addr_j = 0x02;        // 2
    int inc_j = -50 * pol;

    int data = 0x00000000 | 0xF7 << 16 | 0xff << 8 | 0xff; // enable counter reset when microswitch is hit
    this->bram_ptr[0] = data;

    // El joint a 50/-50
    sendRef(inc_j, j);
    usleep(2000000);
    sj = this->bram_ptr[j.jnum]; // lectura de posicion
    if (v)
        puts("while1");
    while (abs(sj - old_sj) != 0) //Buscar final de carrera
    {
        inc_j = inc_j - angle_to_ref(j.jnum,-20) * pol;
        sendRef(inc_j, j);
        usleep(2000000);
        old_sj = sj;
        sj = this->bram_ptr[j.jnum];
        std::cout << "sj: " << sj << ";   old_sj: " << old_sj << std::endl; //Controlar la diferencia de contadores no funciona con j4.
        if ((abs(sj - old_sj) < 0x5)) //Llegada a final de carrera
        //if (abs(sj - old_sj) < 0x5)
            break;
    }
    if (v)
        puts("configureinit");
    configureInitJoint(j); // En java y python la llamada es a SendFPGAReset_joint
    resetJPos(j);
    if (v)
        puts("switch");
    switch (j.jnum) //Salto grande a mas o menos el centro
    {
    case 1:
        inc_j = 300 * pol;
        sendRef(inc_j, j1);
        break;
    case 2:
        inc_j = 550 * pol;
        sendRef(inc_j, j2);
        break;
    case 3:
        inc_j = 300 * pol;
        sendRef(inc_j, j3);
        break;
    case 4:
        inc_j = 1000 * pol;
        sendRef(inc_j, j4);
        break;
    default:
        break;
    }
    usleep(10000000); //wait for 10 seconds
    sj = this->bram_ptr[j.jnum];
    old_sj = sj + 1000;
    if (v)
        puts("while2");
    while (abs(old_sj - sj) > 200) //Ni idea de qué hace esto
    {
        old_sj = sj;
        sj = this->bram_ptr[j.jnum];
        usleep(1000000);
    }
    if (v)
        puts("while3");
    while (abs(sj - (0x20000 / 4)) > 0x400)
    {
        inc_j = inc_j + (angle_to_ref(j.jnum,-2) * pol); //make small jumps of -3 DEG
        sendRef(inc_j, j);
        usleep(2000000);
        old_sj = sj;
        sj = this->bram_ptr[j.jnum];
        if ((abs(sj - (0x20000 / 4)) < 0x400) && abs(old_sj - sj) > 0x300)
        {
            configureInitJoint(j);
        }
    }
    data = 0x00000000 | 0xF7 << 16 | 0x00 << 8 | 0x00; // disable counter reset when microswitch is hit
    this->bram_ptr[0] = data;
}

void EDScorbot::configureInitJoint(EDScorbotJoint j) // configure init pero por joint
{
    // Comprobar que hace que los contadores se reseteen
    // EDScorbotJoint *joints[6] = {&j1, &j2, &j3, &j4, &j5, &j6};
    int data, base;

    base = 0x00000000 | ((j.jnum - 1) * JOINT_STEP) << 16;

    data = base | PI_FD_ENABLE_ADDR << 16 | 0x0f; //|0x00 << 8
    this->bram_ptr[0] = data;

#ifdef EDS_VERBOSE
    printf("J%d PI_FD: %08x\n", j.jnum, data);
#endif

    data = base | PI_FD_ENABLE_ADDR << 16 | 0x03; //|0x00 << 8
    this->bram_ptr[0] = data;

#ifdef EDS_VERBOSE
    printf("J%d PI_FD: %08x\n", j.jnum, data);
#endif

    data = base | SPIKE_GEN_FREQ_DIVIDER << 16 | 0x01; // | 0x00 <<8
    this->bram_ptr[0] = data;
    data = base | PI_FD_ADDR << 16 | ((j.controller["PI_FD_bank3_18bits"] >> 8) & 0xFF) << 8 | (j.controller["PI_FD_bank3_18bits"] & 0xFF);
    this->bram_ptr[0] = data;
#ifdef EDS_VERBOSE
    printf("J%d PI_FD: %08x\n", j.jnum, data);
#endif
    data = base | PD_FD_ENABLE_ADDR << 16 | 0x0f; //|0x00 << 8
    this->bram_ptr[0] = data;
    data = base | PD_FD_ENABLE_ADDR << 16 | 0x03; //|0x00 << 8
    this->bram_ptr[0] = data;

    data = base | PD_FD_ADDR << 16 | ((j.controller["PD_FD_bank3_22bits"] >> 8) & 0xFF) << 8 | (j.controller["PD_FD_bank3_22bits"] & 0xFF);
    this->bram_ptr[0] = data;
#ifdef EDS_VERBOSE
    printf("J%d PD_FD: %08x\n", j.jnum, data);
#endif

    data = base | EI_FD_ENABLE_ADDR << 16 | 0x0f; //|0x00 << 8
    this->bram_ptr[0] = data;

    data = base | EI_FD_ENABLE_ADDR << 16 | 0x03; //|0x00 << 8
    this->bram_ptr[0] = data;

    data = base | EI_FD_ADDR << 16 | ((j.controller["EI_FD_bank3_18bits"] >> 8) & 0xFF) << 8 | (j.controller["EI_FD_bank3_18bits"] & 0xFF);
    this->bram_ptr[0] = data;

#ifdef EDS_VERBOSE
    printf("J%d EI_FD: %08x\n", j.jnum, data);
#endif
    data = base | SPIKE_EXPANSOR_ADDR << 16 | ((j.controller["spike_expansor"] >> 8) & 0xFF) << 8 | (j.controller["spike_expansor"] & 0xFF);
    this->bram_ptr[0] = data;

#ifdef EDS_VERBOSE
    printf("J%d spike expansor: %08x\n", j.jnum, data);

#endif
    sendRef(0, j);
}

void EDScorbot::resetJPos(EDScorbotJoint j)
{
    int address = 0xF0 | j.jnum;
    sendCommand16(address, 0x00, address, this->bram_ptr);
}