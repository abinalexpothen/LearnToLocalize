#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <random> //c++11 random numbers
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// landmarks
double landmarks[8][2] = {{20.0, 20.0}, {20.0, 80.0}, {20.0, 50.0}, {50.0, 20.0}, {50.0, 80.0}, {80.0, 80.0}, {80.0, 20.0}, {80.0, 50.0}};

// map size
double world_size = 100.0;

// random number generator
std::random_device rd;
std::mt19937 gen(rd());

// global functions
double mod(double first_term, double second_term);
double gen_real_random();

class Robot{
    public:
    Robot()
    {
        //constructor
        x = gen_real_random() * world_size; // x-coordinate
        y = gen_real_random() * world_size; // y-coordinate
        orient = gen_real_random() * 2.0 * M_PI; // orientation

        forward_noise = 0.0; // formard motion noise
        turn_noise = 0.0; // turn noise
        sense_noise = 0.0; // sensing noise
    }

    void set(double new_x, double new_y, double new_orient)
    {
        if (new_x < 0 || new_x >= world_size)
        throw std::invalid_argument("X-coord out of bounds");

        if (new_y < 0 || new_y >= world_size)
        throw std::invalid_argument("Y-coord out of bounds");

        if (new_orient < 0 || new_orient >= 2*M_PI)
        throw std::invalid_argument("Orientation out of bounds");

        x = new_x;
        y = new_y;
        orient = new_orient;
    }

    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // simulate noise, often found in particle filters
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    std::vector<double> sense()
    {
        //measure distances from robot towards landmark
        std::vector<double> z(sizeof(landmarks)/sizeof(landmarks[0]));
        double dist;

        for (int i = 0; i < sizeof(landmarks)/sizeof(landmarks[0]); i++)
        {
            dist = sqrt(pow((x-landmarks[i][0]),2)+pow((y-landmarks[i][1]),2));
            dist += gen_gauss_random(0.0, sense_noise);
            z[i] = dist;
        }

        return z;
    }

    Robot move(double turn, double forward)
    {
        if (forward < 0)
        throw std::invalid_argument("Robot cannot move backward");

        // turn and add randomness to turn command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2*M_PI);

        //move and add randomness to motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + cos(orient)*dist;
        y = y + sin(orient)*dist;

        // cyclic truncate
        x = mod(x, world_size);
        y = mod(y, world_size);

        // set particle
        Robot res;
        res.set(x, y, orient);
        res.set_noise(forward_noise, turn_noise, sense_noise);

        return res;
    }

    // returns robot current position and orientation as a string
    std::string show_pose()
    {
        return "[x=" + std::to_string(x) + " y=" + std::to_string(y) + " orient=" + std::to_string(orient) + "]";
    }

    // returns all distances from the robot towards landmark
    std::string read_sensors()
    {
        std::vector<double> z  = sense();
        std::string readings = "[";
        for (int i = 0; i < z.size(); i++)
        {
            readings += std::to_string(z[i]) + " ";
        }
        readings[readings.size()-1] = ']';

        return readings;
    }

    // calculates how likely a measurement should be
    double measurement_probability(std::vector<double> measurement)
    {
        double prob = 1.0;
        double dist;

        for (int i = 0; i < sizeof(landmarks)/sizeof(landmarks[0]); i++)
        {
            dist = sqrt(pow((x-landmarks[i][0]),2)+pow((y-landmarks[i][1]),2));
            prob *= gaussian(dist, sense_noise, measurement[i]);

            return prob;
        }

    }

    double x, y, orient; // robot poses
    double forward_noise, turn_noise, sense_noise; // robot noises

    private:
    double gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        std::normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    // probability of x for 1-D gaussian with mean mu and variance sigma
    double gaussian(double mu, double sigma, double x)
    {
        return exp(-(pow((mu-x),2))/pow(sigma,2)/2.0)/sqrt(2.0*M_PI*(pow(sigma,2)));
    }
};

//functions

// generate real numbers between 0 and 1
double gen_real_random()
{
    std::uniform_real_distribution<double> real_dist(0.0, 1.0); //real
    return real_dist(gen);
}

// compute modulus
double mod(double first_term, double second_term)
{
    return first_term - second_term * floor(first_term/second_term);

}

// compute mean error of the system
double evaluation(Robot r, Robot p[], int n)
{
    double sum = 0.0;
    for (int i = 0; i < n;  i++)
    {
        double dx = mod((p[i].x -r.x + world_size/2.0), world_size) - world_size/2.0;
        double dy = mod((p[i].y-r.y+world_size/2.0), world_size) - world_size/2.0;
        double err = sqrt(pow(dx,2)+pow(dy,2));
        sum += err;
    }
    return sum/n;
}

// identify max elements in an array
double max(double arr[], int n)
{
    double max = 0;
    for (int i = 0; i < n; i++)
    {
        if (arr[i] > max)
        max = arr[i];
    }
    return max;
}

void visualization(int n, Robot robot, int step, Robot p[], Robot pr[])
{
    //Draw the robot, landmarks, particles and resampled particles on a graph

    //Graph Format
    plt::title("MCL, step " + std::to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);

    //Draw particles in green
    for (int i = 0; i < n; i++) {
        std::vector<double> x = {p[i].x};
        std::vector<double> y = {p[i].y};
        plt::plot(x, y, "go");
    }

    //Draw resampled particles in yellow
    for (int i = 0; i < n; i++) {
        std::vector<double> x = {pr[i].x};
        std::vector<double> y = {pr[i].y};
        plt::plot(x, y, "yo");
    }

    //Draw landmarks in red
    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
        std::vector<double> x = {landmarks[i][0]};
        std::vector<double> y = {landmarks[i][1]};
        plt::plot(x, y, "ro");
    }

    //Draw robot position in blue
    std::vector<double> x = {robot.x};
    std::vector<double> y = {robot.y};
    plt::plot(x, y, "bo");
    plt::show();

    //Save the image and close the plot
    //plt::savefig("./Images/Step" + std::to_string(step) + ".png");
    //plt::clf();
}


int main()
{
    // Test interfacing with robot class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI/2.0);
    myrobot.move(-M_PI/2.0, 15.0);
    // std::cout << myrobot.read_sensors() << std::endl;
    myrobot.move(-M_PI/2.0, 10.0);
    // std::cout << myrobot.read_sensors() << std::endl;    
    
    // Instantiate 100 particles, each with a random positon and orientation
    int n = 1000;
    Robot p[n];

    for (int i = 0; i < n;i++)
    {
        p[i].set_noise(0.05, 0.05, 5.0);
    }

    myrobot = Robot();
    std::vector<double> z;

    int steps = 50;
    for (int t = 0; t < steps; t++) {

        //Move the robot and sense the environment afterwards
        myrobot = myrobot.move(0.1, 5.0);
        z = myrobot.sense();

        Robot p2[n];

        // iterate over each particle
        for (int i = 0; i < n;i++)
        {
            p2[i] = p[i].move(0.1, 5.0);
            p[i] = p2[i];
            // std::cout << p[i].show_pose() << std::endl;
        }

        // weights
        double w[n];

        for (int i=0; i < n; i++)
        {
            w[i] = p[i].measurement_probability(z);
            //std::cout << w[i] << std::endl;
        }

        // resample wheel
        Robot p3[n];

        int index = gen_real_random() * n;

        double beta = 0.0;
        double wmax = max(w,n);

        for (int i = 0;i < n; i++)
        {
            beta += gen_real_random()*2*wmax;
            while (w[index] < beta)
            {
                beta -= w[index];
                index = mod((index + 1), n);
            }
            p3[i] = p[index];
        }

        for (int k = 0; k < n;k++)
        {
            p[k] = p3[k];
            //std::cout << p[k].show_pose() << std::endl;
        }

        std::cout << "Step = " << t << ", Evaluation = " << evaluation(myrobot, p, n) << std::endl;

        visualization(n, myrobot, t, p2, p3);
    }
    



    return 0;
}