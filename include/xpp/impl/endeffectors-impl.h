/**
 @file    endeffectors.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Brief description
 */

namespace xpp {

template<typename T>
Endeffectors<T>::Endeffectors (int n_ee)
{
  SetCount(n_ee);
}

template<typename T>
void
Endeffectors<T>::SetCount (int n_ee)
{
  ee_.resize(n_ee);
}

template<typename T>
Endeffectors<T>::~Endeffectors ()
{
}

template<typename T>
void
Endeffectors<T>::SetAll (const T& value)
{
  std::fill(ee_.begin(), ee_.end(), value);
}

template<typename T>
T&
Endeffectors<T>::At (EndeffectorID idx)
{
  return ee_.at(idx);
}

template<typename T>
const T&
Endeffectors<T>::At (EndeffectorID idx) const
{
  return ee_.at(idx);
}

template<typename T>
int
Endeffectors<T>::GetCount () const
{
  return ee_.size();
}

template<typename T>
const typename Endeffectors<T>::Container
Endeffectors<T>::ToImpl () const
{
  return ee_;
}

template<typename T>
const typename Endeffectors<T>::EndeffectorsT
Endeffectors<T>::operator - (const EndeffectorsT& rhs) const
{
  EndeffectorsT result(ee_.size());
  for (auto i : GetEEsOrdered())
    result.At(i) = ee_.at(i) - rhs.At(i);

  return result;
}

template<typename T>
const typename Endeffectors<T>::EndeffectorsT
Endeffectors<T>::operator / (double scalar) const
{
  EndeffectorsT result(ee_.size());
  for (auto i : GetEEsOrdered())
    result.At(i) = ee_.at(i)/scalar;

  return result;
}

template<typename T>
std::vector<EndeffectorID>
Endeffectors<T>::GetEEsOrdered () const
{
  return std::vector<EndeffectorID>(kEEOrder.begin(), kEEOrder.begin()+ee_.size());
}

template<typename T>
bool
Endeffectors<T>::operator!=(const Endeffectors& other) const
{
  for (auto ee : GetEEsOrdered()) {
    if (ee_.at(ee) != other.At(ee))
      return true;
  }
  return false;
}

} /* namespace xpp */
