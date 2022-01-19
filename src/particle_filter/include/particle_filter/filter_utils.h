#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <stdexcept> 
#include <random> 
using namespace std;
random_device rd;
mt19937 gen(rd());
// Global Functions
inline double mod(double first_term, double second_term)
{
   // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}
inline double gen_real_random()
{
    // Generate real random between 0 and 1
    uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}
inline double gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

inline double gaussian(double mu, double sigma, double x)
{
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
}
inline double max(double arr[], int n)
{
    // Identify the max element in an array
    double max = 0;
    for (int i = 0; i < n; i++) {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}
