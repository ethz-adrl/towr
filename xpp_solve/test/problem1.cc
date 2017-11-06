
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

#include <cmath>
#include <xpp_solve/solvers/ipopt_adapter.h>

using namespace xpp;
using Eigen::Vector2d;


class ExVariables : public Component {
public:
  ExVariables() : Component(2, "var_set1")
  {
    x0_ = 0.0;
    x1_ = 0.0;

    bounds_.resize(GetRows());
    bounds_.at(0) = NoBound;
    bounds_.at(1) = NLPBound(-1.0, 1.0);
  }

  virtual VectorXd GetValues() const
  {
    return Vector2d(x0_, x1_);
  };

  virtual void SetValues(const VectorXd& x)
  {
    x0_ = x(0);
    x1_ = x(1);
  };

private:
  double x0_;
  double x1_;

  VecBound bounds_;
};


class ExConstraint : public Constraint {
public:
  ExConstraint(const OptVarsPtr& vars)
  {
    SetName("quadratic_constraint");
    AddOptimizationVariables(vars);
    int constraint_count = 1; // only one constraint row
    SetRows(constraint_count);
  }

  virtual VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    Vector2d x = GetOptVars()->GetComponent("var_set1")->GetValues(); // could be multiple sets
    g(0) = std::pow(x(0),2) + x(1);
    return g;
  };

  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = NLPBound(1.0, +inf); // between 1 and inifinity
    return b;
  }

  void FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {

      Vector2d x = GetOptVars()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
      jac.coeffRef(0, 1) = 1.0; // derivative of first constraint w.r.t x1
    }
  }
};


class ExCost: public Cost {
public:
  ExCost(const OptVarsPtr& vars)
  {
    SetName("quadratic_cost");
    AddOptimizationVariables(vars);
    SetRows(1);  // cost always has only 1 row
  }

  virtual VectorXd GetValues() const override
  {
    VectorXd J(GetRows());
    Vector2d x = GetOptVars()->GetComponent("var_set1")->GetValues();

    J(0) = -std::pow(x(1)-2,2);
    return J;
  };

  void FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {

      Vector2d x = GetOptVars()->GetComponent("var_set1")->GetValues();

      jac.coeffRef(0, 0) = 0.0;             // derivative of cost w.r.t x0
      jac.coeffRef(0, 1) = -2.0*(x(1)-2.0); // derivative of cost w.r.t x1
    }
  }
};



int main(int argc, char *argv[])
{
  auto variables  = std::make_shared<Composite>("all_variables", true);
  variables->AddComponent(std::make_shared<ExVariables>());

  std::unique_ptr<ExConstraint> constraint(new ExConstraint(variables));
  std::unique_ptr<ExCost> cost(new ExCost(variables));


  NLP nlp;
  nlp.Init(variables);
  nlp.AddConstraint(std::move(constraint));
  nlp.AddCost(std::move(cost));

  variables->Print();
  nlp.PrintCurrent();

  IpoptAdapter::Solve(nlp);

  nlp.PrintCurrent();
  variables->Print();

  VectorXd x = variables->GetValues();

  return 1;
}

