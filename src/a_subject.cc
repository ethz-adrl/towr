/*
 * a_subject.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/a_subject.h>

namespace xpp
{
namespace zmp
{

ASubject::ASubject ()
{
  // TODO Auto-generated constructor stub

}

ASubject::~ASubject ()
{
  // TODO Auto-generated destructor stub
}

int
ASubject::GetObserverCount () const
{
  return observers_.size();
}

} /* namespace zmp */
} /* namespace xpp */
