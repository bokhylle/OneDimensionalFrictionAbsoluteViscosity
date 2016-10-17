#include "integrator.h"
#include "node.h"
#include "friction.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
using namespace std;

Integrator::Integrator(vector<shared_ptr<Node>> nodes, double dt, int outputFrequency, string outputFoldername)
{
    m_nodes = nodes;
    m_dt = dt;
    m_outputFrequency = outputFrequency;
    m_outputFoldername = outputFoldername;

    //mkdir foldername
    std::string command = "mkdir -p ";
    command.append(m_outputFoldername);
    cout << command << endl;
    system(command.c_str());
}

bool Integrator::velocityVerlet(){
    vector<double> forceAtInitialTime(m_nodes.size());
    vector<bool> nodeIsStuck(m_nodes.size());
    for(unsigned int i = 0; i<m_nodes.size(); i++){
        forceAtInitialTime[i] = m_nodes[i]->tangentialForce();
        nodeIsStuck[i] = m_nodes[i]->frictionLaw()->stuck();
        m_nodes[i]->setX(m_nodes[i]->x() + m_nodes[i]->v()*m_dt + 0.5*m_nodes[i]->tangentialForce()*m_dt*m_dt);
    }
    for(unsigned int i = 0; i<m_nodes.size(); i++){
        m_nodes[i]->calculateForces();
    }
    for(unsigned int i = 0; i<m_nodes.size(); i++){
        m_nodes[i]->setVPrevious(m_nodes[i]->v());
        m_nodes[i]->setV(m_nodes[i]->v() + 0.5*(forceAtInitialTime[i] + m_nodes[i]->tangentialForce())*m_dt);
    }
    for(unsigned int i = 0; i<m_nodes.size(); i++){
        m_nodes[i]->calculateForces();
    }
    bool nodeStoppedOrStarted = false;
    for(unsigned int i = 0; i<m_nodes.size(); i++){
        if(nodeIsStuck[i] != m_nodes[i]->frictionLaw()->stuck()){
            nodeStoppedOrStarted = true;
        }
    }
    return nodeStoppedOrStarted;
}

double Integrator::t0() const
{
    return m_t0;
}

void Integrator::setT0(double t0)
{
    m_t0 = t0;
}


void Integrator::setNodes(const vector<shared_ptr<Node> > &nodes)
{
    m_nodes = nodes;
}

vector<shared_ptr<Node> > Integrator::nodes() const
{
    return m_nodes;
}

void Integrator::setDt(double dt)
{
    m_dt = dt;
}

double Integrator::dt() const
{
    return m_dt;
}

double Integrator::outputFrequency() const
{
    return m_outputFrequency;
}

void Integrator::setOutputFrequency(double outputFrequency)
{
    m_outputFrequency = outputFrequency;
}

std::string Integrator::outputFoldername() const
{
    return m_outputFoldername;
}

void Integrator::setOutputFoldername(const string &outputFoldername)
{
    m_outputFoldername = outputFoldername;
}

void Integrator::integrate(unsigned int numberOfTimeSteps){

    for(unsigned int i = 0; i<m_nodes.size(); i++){
        m_nodes[i]->calculateForces();
    }

    bool nodeStoppedOrStarted = false;
    outputOpen();
    for(unsigned int i = 0; i<numberOfTimeSteps; i++){
        if((i% m_outputFrequency) == 0 || nodeStoppedOrStarted){
            output(i*m_dt + m_t0);
        }
        nodeStoppedOrStarted = velocityVerlet();
    }
    outputClose();
}

void Integrator::output(double t){

    cout << "outputting, t = " << t << endl;
    m_tFile << setprecision(10) << t << endl;
    for(unsigned int i = 0; i<m_nodes.size(); i++){
        m_xFile << setprecision(10) << m_nodes[i]->x() << " ";
        m_vFile << setprecision(10) << m_nodes[i]->v() << " ";
        m_springForceFile << setprecision(10) << m_nodes[i]->springForce() << " ";
        m_frictionForceFile << setprecision(10) << m_nodes[i]->frictionForce() << " ";
        m_dampingForceFile << setprecision(10) << m_nodes[i]->dampingForce() << " ";
        m_drivingForceFile << setprecision(10) << m_nodes[i]->drivingForce() << " ";
        m_stuckFile << m_nodes[i]->frictionLaw()->stuck() << " ";
    }

    m_xFile << endl;
    m_vFile << endl;
    m_springForceFile << endl;
    m_frictionForceFile << endl;
    m_dampingForceFile << endl;
    m_drivingForceFile << endl;
    m_stuckFile << endl;
}


void Integrator::outputOpen(){
    m_tFile.open(m_outputFoldername+"/t.txt");
    m_xFile.open(m_outputFoldername+"/x.txt");
    m_vFile.open(m_outputFoldername+"/v.txt");
    m_springForceFile.open(m_outputFoldername+"/springForce.txt");
    m_frictionForceFile.open(m_outputFoldername+"/frictionForce.txt");
    m_dampingForceFile.open(m_outputFoldername+"/dampingForce.txt");
    m_drivingForceFile.open(m_outputFoldername+"/drivingForce.txt");
    m_stuckFile.open(m_outputFoldername+"/stuck.txt");
}

void Integrator::outputClose(){
    m_tFile.close();
    m_xFile.close();
    m_vFile.close();
    m_springForceFile.close();
    m_frictionForceFile.close();
    m_dampingForceFile.close();
    m_drivingForceFile.close();
    m_stuckFile.close();
}
