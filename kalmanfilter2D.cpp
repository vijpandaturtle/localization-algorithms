#include<iostream>
#include<math.h>
#include<tuple>
#include "LU" //Eigen library is used for performing matrix operations. Check it out here https://eigen.tuxfamily.org/dox/GettingStarted.html
#include "Core" //Eigen library

using namespace std;
using namespace eigen;

float measurements[3] = {1,2,3};

tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf F, MatrixXf H, MatrixXf R, MatrixXf I, MatrixXf u){

   for(int i=0; i<sizeof(measurements)/sizeof(measurements); i++) {
     //Measurement update
     MatrixXf Z(1,1);
     Z << measurements[i];

     MatrixXf y(1, 1);
     y << Z - (H * x);

      MatrixXf S(1, 1);
      S << H * P * H.transpose() + R;

      MatrixXf K(2, 1);
      K << P * H.transpose() * S.inverse();

      x << x + (K * y);

      P << (I - (K * H)) * P;

      //State Prediction
      x << (F * x) + u;
      P << F * P * F.transpose();
      }

      return make_tuple(x, P);
}

int main() {

    MatrixXf x(2, 1);// Initial state (location and velocity)
    x << 0,
    0;
    MatrixXf P(2, 2);//Initial Uncertainty
    P << 100, 0,
    0, 100;
    MatrixXf u(2, 1);// External Motion
    u << 0,
    0;
    MatrixXf F(2, 2);//Next State Function
    F << 1, 1,
    0, 1;
    MatrixXf H(1, 2);//Measurement Function
    H << 1,
    0;
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1, 0,
    0, 1;

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
    cout << "x= " << x << endl;
    cout << "P= " << P << endl;

    return 0;
}
