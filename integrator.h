#include <vector>
#include <memory>
#include <string>
#include <fstream>

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

class Integrator
{
public:
    Integrator(std::vector<std::shared_ptr<class Node>> nodes, double dt, int outputFrequency, std::string outputFoldername);
    void integrate(unsigned int numberOfTimeSteps);
    void output(double t);
    void outputOpen();
    void outputClose();

    std::string outputFoldername() const;
    void setOutputFoldername(const std::string &outputFoldername);

    double outputFrequency() const;
    void setOutputFrequency(double outputFrequency);

    double dt() const;
    void setDt(double dt);

    std::vector<std::shared_ptr<class Node> > nodes() const;
    void setNodes(const std::vector<std::shared_ptr<class Node> > &nodes);

    double t0() const;
    void setT0(double t0);

private:
    bool velocityVerlet();
    std::vector<std::shared_ptr<class Node>> m_nodes;
    double m_dt;
    double m_t0;
    int m_outputFrequency;

    std::string m_outputFoldername;
    std::ofstream m_tFile;
    std::ofstream m_xFile;
    std::ofstream m_vFile;
    std::ofstream m_springForceFile;
    std::ofstream m_frictionForceFile;
    std::ofstream m_dampingForceFile;
    std::ofstream m_drivingForceFile;
    std::ofstream m_stuckFile;

};

#endif // INTEGRATOR_H
