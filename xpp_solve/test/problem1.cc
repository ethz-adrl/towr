
/** C++ Example NLP for interfacing a problem with IPOPT.
 *  MyNLP implements a C++ example showing how to interface with IPOPT
 *  through the TNLP interface. This example is designed to go along with
 *  the tutorial document (see Examples/CppTutorial/).
 *  This class implements the following NLP.
 *
 * min_x f(x) = -(x1-2)^2
 *  s.t.
 *       0 = x0^2 + x1 - 1
 *       -1 <= x0 <= 1
 *
 */

#include <xpp_solve/leaves.h>
#include <xpp_solve/solvers/ipopt_adapter.h>

using namespace opt;
using Eigen::Vector2d;


class ExVariables : public Variables {
public:
  ExVariables() : Variables(2, "var_set1")
  {
    // initial values
    x0_ = 0.0;
    x1_ = 0.0;
  }

  virtual void SetValues(const VectorXd& x)
  {
    x0_ = x(0);
    x1_ = x(1);
  };

  virtual VectorXd GetValues() const
  {
    return Vector2d(x0_, x1_);
  };

  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = NoBound;
    bounds.at(1) = NLPBound(-1.0, 1.0);
    return bounds;
  }

private:
  double x0_, x1_;
};


class ExConstraint : public Constraint {
public:
  ExConstraint(const Composite::Ptr& variables) : Constraint(variables, 1, "constraint1"){}

  virtual VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
    g(0) = std::pow(x(0),2) + x(1);
    return g;
  };

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = NLPBound(1.0, +inf); // between 1 and inifinity
    return b;
  }

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {

      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
      jac.coeffRef(0, 1) = 1.0;      // derivative of first constraint w.r.t x1
    }
  }
};


class ExCost: public Cost {
public:
  ExCost(const Composite::Ptr& variables) : Cost(variables,  "cost_term1") {}

  virtual VectorXd GetValues() const override
  {
    VectorXd J(GetRows());
    Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

    J(0) = -std::pow(x(1)-2,2);
    return J;
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {

      Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 0.0;             // derivative of cost w.r.t x0
      jac.coeffRef(0, 1) = -2.0*(x(1)-2.0); // derivative of cost w.r.t x1
    }
  }
};



int main()
{
  auto variables = std::make_shared<Composite>("all_variables", false);
  variables->AddComponent(std::make_shared<ExVariables>());

  auto constraints = std::make_unique<Composite>("all_constraints", false);
  constraints->AddComponent(std::make_shared<ExConstraint>(variables));

  auto costs = std::make_unique<Composite>("all_costs", true);
  costs->AddComponent(std::make_shared<ExCost>(variables));

  NLP nlp;
  nlp.SetVariables(variables);
  nlp.SetConstraints(std::move(constraints));
  nlp.SetCosts(std::move(costs));

  IpoptAdapter::Solve(nlp);

  nlp.PrintCurrent();

  std::cout << "\n\nx: " << variables->GetValues().transpose() << std::endl;;
}

