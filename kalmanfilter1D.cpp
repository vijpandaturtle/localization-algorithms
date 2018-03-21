#include<iostream>
#include<math.h>
#include<tuple>

using namespace std;

double updated_mean, updated_variance;

//Finding the posterior mean and variance after a sensor measurement
tuple<double, double> measurement_update(double mean1, double var1, double mean2, double var2) {
    updated_mean = (var2*mean1 + var1*mean2)/(var1 + var2);
    updated_variance = 1/(1/var1 + 1/var2);
    return make_tuple(updated_mean, updated_variance);
}

//Finding the posterior mean and variance after a motion update
tuple<double, double> state_prediction(double mean1, double var1, double mean2, double var2) {
    updated_mean = mean1 + mean2;
    updated_variance = var1 + var2;
    return make_tuple(updated_mean, updated_variance);
}

int main(){
    //Measurement and measurement variance
    double measurements[5] = {1,2,3,4,5};
    double measurement_var = 4;

    //Motion and motion variance
    double motion[5] = {1,1,2,1,1};
    double motion_var = 2;

    ///Initial state
    double mu = 0;
    double sig = 1000;

    //Performing approximation using kalman filter
    for(int i=0; i<sizeof(measurements)/sizeof(measurements[0]); i++) {
        tie(mu,sig) = measurement_update(mu, sig, measurements[i], measurement_var);
        cout<<"Updated mean and variance after measurement update "<<mu<<sig<<endl;
        tie(mu, sig) = state_prediction(mu, sig, motion[i], motion_var);
        cout<<"Updated mean and variance after state prediction "<<mu<<sig<<endl;
    }

    return 0;
}
