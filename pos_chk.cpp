#include <iostream>
#include <cmath>
#include <vector>
// using namespace std;

const double acc = 20;
const double initial_pos = 0;
const double final_pos = 20;
const double vel = 60;
double ta;
double tb;
double time_duration;

double traj_path_point_to_point(std::vector<double> ini_pos, std::vector<double> final_pos)
{
    int num_joints = ini_pos.size();

    std::vector<double> desired_joint_acc, desired_joint_vel;
    desired_joint_acc.resize(num_joints);
    desired_joint_vel.resize(num_joints);

    std::vector<double> acc_time, cruise_time, deacc_time;
    acc_time.resize(num_joints);
    cruise_time.resize(num_joints);
    deacc_time.resize(num_joints);

    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        if (fabs((final_pos[jnt_ctr] - ini_pos[jnt_ctr])) > 1e-5)
        {
            desired_joint_acc[jnt_ctr] = (final_pos[jnt_ctr] - ini_pos[jnt_ctr])/fabs((final_pos[jnt_ctr] - ini_pos[jnt_ctr]))*acc;
        }
        else{
            desired_joint_acc[jnt_ctr] = 0;
        }
    }

    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        if (fabs((final_pos[jnt_ctr] - ini_pos[jnt_ctr])) > 1e-5)
        {
            desired_joint_vel[jnt_ctr] = (final_pos[jnt_ctr] - ini_pos[jnt_ctr])/fabs((final_pos[jnt_ctr] - ini_pos[jnt_ctr]))*vel;
        }
        else{
            desired_joint_vel[jnt_ctr] = 0;
        }
    }

    for (unsigned int jnt_ctr = 0; jnt_ctr < num_joints; jnt_ctr++)
    {
        double min_pos_travel = 0;

        min_pos_travel = desired_joint_vel[jnt_ctr]*desired_joint_vel[jnt_ctr]/desired_joint_acc[jnt_ctr];

        if (fabs(final_pos[jnt_ctr] - ini_pos[jnt_ctr]) < fabs(min_pos_travel) ){
            acc_time[jnt_ctr] = sqrt((final_pos[jnt_ctr] - ini_pos[jnt_ctr])/desired_joint_acc[jnt_ctr]);
            cruise_time[jnt_ctr] = 0;
            deacc_time[jnt_ctr] = acc_time[jnt_ctr];
        }

    }



    // de
    
}

double calculate_pos(double t)
{

    double pos;

    if (t < ta)
    {

        pos = 0.5 * acc * t + initial_pos;
    }

    else if (ta < t < tb)
    {
        pos = initial_pos + 0.5 * acc * pow(ta, 2) + acc * ta * (t - ta);
    }
    else if (tb < t < time_duration)
    {
        pos = final_pos - 0.5 * (ta + tb - t) * (time_duration - t);
    }

    return pos;
}

void find_ta_tb()
{

    if (acc != 0)
    {
        ta = vel / acc;
        tb = time_duration - ta;
    }
    else
    {
        std::cout << "acceleration and velocity can't be zero" << std::endl;
    }
}
void find_time_duration()
{

    if (acc != 0 && vel != 0)
    {
        time_duration = 2 * ta + (final_pos - initial_pos) / vel - vel / acc;
    }

    else
    {
        std::cout << "acceleration and velocity can't be zero" << std::endl;
    }
}

int main()
{
    find_ta_tb();
    find_time_duration();

    FILE *fp;
    fp = fopen("data.csv", "w");
    if (fp == NULL)
    {
        perror("Error opening file");
        return 1;
    }

    for (double t = 0.0; t <= time_duration; t += 0.01)
    {
        double position = calculate_pos(t);
        std::cout << "Joint position at time " << t << ": " << position << std::endl;
        if (fprintf(fp, "%f,%f\n", t, position) < 0)
        {
            perror("Error writing to file");
            return 1;
        }
    }
}
