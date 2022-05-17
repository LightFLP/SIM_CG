#include "constraints/Constraint.h"

std::vector<Constraint*> Constraint::_constraints;

void Constraint::addConstraint(Constraint* constraint) {
    constraint->m_c_index = Constraint::_constraints.size();
    Constraint::_constraints.push_back(constraint);
};

Constraint::~Constraint() {
    for (Constraint *c : Constraint::_constraints) {
        c->iVector.clear();
        delete c;
    }
    Constraint::_constraints.clear();
}