#include "handeye_park.h"


int main(int argc, char* argv[])
{
    std::string path = argv[1];

    // load data
    HandEyePark handeye_park;
    handeye_park.loadData(path + "/matched_sim_evolving_state.txt", 
    path + "/matched_real_body_evolving_state.txt");

    Sophus::SE3d T_HE; // T_simreal
    T_HE = handeye_park.run();

    std::cout << "T_HE: \n" << T_HE.matrix3x4() << "\n";

    return 0;
}