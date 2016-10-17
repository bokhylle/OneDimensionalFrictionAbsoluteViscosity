#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include "node.h"
#include "friction.h"
#include "integrator.h"

using namespace std;

int main(int argc, char *argv[])
{
    //This code is only valid for positive velocities.

    if(argc!=9){
        cout << "Wrong number of command line arguments!" << endl;
        cout << "Command line arguments: int N, double etaBar, double tauBar, double alphaBar, double dt, int numberOfTimeSteps, int outputFrequency, string foldername" << endl;
        cout << endl;
        cout << "The code is valid for positive velocities. It reproduces parts of Amundsen PRE 2015 when alpha = 0, and the new paper for alpha > 0." << endl;
        return 1;
    }

    double N = strtol(argv[1],nullptr,10);
    double eta_bar = strtod(argv[2],nullptr);
    double tau_bar = strtod(argv[3],nullptr);
    double alpha_bar = strtod(argv[4],nullptr);
    double dt = strtod(argv[5],nullptr);
    int numberOfTimeSteps = strtol(argv[6],nullptr,10);
    int outputFrequency = strtol(argv[7],nullptr,10);
    string outputFoldername = argv[8];
    double Ft = 1-tau_bar;

    outputFoldername.append("/N=");
    outputFoldername.append(to_string(N));
    outputFoldername.append("_eta=");
    outputFoldername.append(to_string(eta_bar));
    outputFoldername.append("_tau=");
    outputFoldername.append(to_string(tau_bar));
    outputFoldername.append("_alpha=");
    outputFoldername.append(to_string(alpha_bar));
    outputFoldername.append("_dt=");
    outputFoldername.append(to_string(dt));

    //Set up nodes
    vector<shared_ptr<Node>> node(N);
    for(int i = 0; i<N; i++){
        node[i] = make_shared<Node>(eta_bar,tau_bar);
    }
    node[0]->setDrivingForce(Ft); //First node has driving spring

    //Connect nodes:
    for(int i = 1; i<N; i++)    { node[i]->setConnectionLeft(node[i-1]); }
    for(int i = 0; i<N-1; i++)  { node[i]->setConnectionRight(node[i+1]); }

    //Add friction law to nodes
    for(int i = 0; i<N; i++){
        shared_ptr<Friction> friction = make_shared<Friction>(alpha_bar);
        node[i]->setFrictionLaw(friction);
    }

    //Set up integrator
    Integrator integrator(node, dt, outputFrequency, outputFoldername);
    integrator.integrate(numberOfTimeSteps);

    return 0;
}
