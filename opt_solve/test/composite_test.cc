/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>
#include <opt_solve/composite.h>


namespace opt {

class ExComponent : public Component {
public:
  ExComponent(int n_var, const std::string& name) : Component(n_var, name) {};

  virtual VectorXd GetValues() const {};
  virtual VecBound GetBounds() const {};
  virtual void SetVariables(const VectorXd& x) {};
  virtual Jacobian GetJacobian() const {};
};

} // namespace opt


using namespace opt;

TEST(ComponentTest, Getters)
{
  ExComponent c(2, "ex_component");

  EXPECT_EQ(2, c.GetRows());
  EXPECT_STREQ("ex_component", c.GetName().c_str());

  c.SetRows(4);
  EXPECT_EQ(4, c.GetRows());
}


TEST(CompositeTest, CostOrConstraint)
{
  auto c1 = std::make_shared<ExComponent>(0, "component1");
  auto c2 = std::make_shared<ExComponent>(1, "component2");
  auto c3 = std::make_shared<ExComponent>(2, "component3");

  Composite cost("cost", true);
  cost.AddComponent(c1);
  cost.AddComponent(c2);
  cost.AddComponent(c3);
  EXPECT_EQ(1, cost.GetRows());

  Composite constraint("constraint", false);
  constraint.AddComponent(c1);
  constraint.AddComponent(c2);
  constraint.AddComponent(c3);
  EXPECT_EQ(0+1+2, constraint.GetRows());
}


TEST(CompositeTest, GetComponent)
{
  auto c1 = std::make_shared<ExComponent>(0, "component1");
  auto c2 = std::make_shared<ExComponent>(1, "component2");
  auto c3 = std::make_shared<ExComponent>(2, "component3");

  Composite comp("composite", false);
  comp.AddComponent(c1);
  comp.AddComponent(c2);
  comp.AddComponent(c3);

  auto c1_new = comp.GetComponent("component1");
  EXPECT_EQ(c1->GetRows(), c1_new->GetRows());

  auto c2_new = comp.GetComponent<ExComponent>("component2");
  EXPECT_EQ(c2->GetRows(), c2_new->GetRows());

  auto c3_new = comp.GetComponent<ExComponent>("component3");
  EXPECT_NE(c1->GetRows(), c3_new->GetRows());
}


TEST(CompositeTest, GetRowsNonzeroClear)
{
  auto c1 = std::make_shared<ExComponent>(0, "component1");
  auto c2 = std::make_shared<ExComponent>(1, "component2");
  auto c3 = std::make_shared<ExComponent>(2, "component3");

  Composite comp("composite", false);
  comp.AddComponent(c1);
  comp.AddComponent(c2);
  comp.AddComponent(c3);

  EXPECT_EQ(0+1+2, comp.GetRows());

  EXPECT_EQ(2, comp.GetNonzeroComponents().size());

  comp.ClearComponents();
  EXPECT_EQ(0, comp.GetRows());
}
