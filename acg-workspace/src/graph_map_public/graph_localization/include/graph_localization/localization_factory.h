#ifndef LOCALISATIONFACTORY_H
#define LOCALISATIONFACTORY_H
#include "stdio.h"
#include <string>
#include <boost/algorithm/string.hpp>
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"
#include "string.h"
#include "iostream"


typedef Eigen::Matrix<double, 6, 1> Vector6d;
using namespace  std;
namespace perception_oru{
namespace graph_localization{

/*!
 * \brief templateMapType implements the map or acts as a wrapper to your existing map classes
 * \brief templateMapTypePtr is the general way the map type is passed around, based om shared pointers to ensure memory no memory losses
 */
class TemplateLocalisationType;
typedef boost::shared_ptr<TemplateLocalisationType> TemplateLocalisationTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
class TemplateLocalisationParam;
typedef boost::shared_ptr<TemplateLocalisationParam> TemplateLocalisationParamPtr;


class MCLNDTType;
typedef boost::shared_ptr<MCLNDTType> MCLNDTTypePtr;

class MCLNDTParam;
typedef boost::shared_ptr<MCLNDTParam> MCLNDTParamPtr;

class RegLocalisationType;
typedef boost::shared_ptr<RegLocalisationType> RegLocalisationTypePtr;

class RegLocalisationParam;
typedef boost::shared_ptr<RegLocalisationParam> RegLocalisationParamPtr;

class LocalizationType;
typedef boost::shared_ptr<LocalizationType> LocalisationTypePtr;

class LocalisationParam;
typedef boost::shared_ptr<LocalisationParam> LocalisationParamPtr;

class LocalisationFactory{
public:
  static LocalisationParamPtr      CreateLocalisationParam(string localisationType);
  static LocalisationTypePtr       CreateLocalisationType(LocalisationParamPtr param);


private:
  LocalisationFactory(){}
};

}
}
#endif // LOCALISATIONFACTORY_H
