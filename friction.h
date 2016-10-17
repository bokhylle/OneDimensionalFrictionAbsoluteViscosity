#include <memory>
#ifndef FRICTION_H
#define FRICTION_H

class Friction
{
public:
    Friction(double alpha);
    double calculateFrictionForce();

    class Node *parent() const;
    void setParent(std::shared_ptr<class Node> node);

    bool stuck() const;
    void setStuck(bool stuck);

private:
    double m_alpha;
    double m_frictionForce;
    class Node* m_parent;
    bool m_stuck = true;
    int sign(double a);
};

#endif // FRICTION_H
