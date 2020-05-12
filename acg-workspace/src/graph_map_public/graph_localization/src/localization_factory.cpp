#include "graph_localization/localization_factory.h"//must be included first
#include "graph_localization/localization_type.h"
#include "graph_localization/mcl_ndt/mcl_ndt.h"
#include "graph_localization/reg_localization_type/reg_localization_type.h"
string ndt_mcl_name="mcl_ndt";
string reg_localisation_name="reg_localisation_type";
using namespace std;
namespace perception_oru{
namespace graph_localization{
LocalisationParamPtr LocalisationFactory::CreateLocalisationParam(string localisationType){

  if(localisationType.compare(ndt_mcl_name)==0){
    cout<<"LocalisationFactory: : \""<<ndt_mcl_name<<"\"";
    return MCLNDTParamPtr (new MCLNDTParam());
  }
  else if(localisationType.compare(reg_localisation_name)==0){
    cout<<"LocalisationFactory: Creating parameters for localisation type: \""<<reg_localisation_name<<"\""<<endl;
    return RegLocalisationParamPtr (new RegLocalisationParam());
  }else{
    std::cerr<<"No localisation parameter type exists with name: \""<<localisationType<<"\""<<endl;
    exit(0);
    return NULL;
  }
}

LocalisationTypePtr  LocalisationFactory::CreateLocalisationType(LocalisationParamPtr param){
  if(MCLNDTParamPtr mcl_ndt_par=boost::dynamic_pointer_cast<MCLNDTParam>(param)){
    cout<<"LocalisationFactory: creating object of type: \""<<ndt_mcl_name<<"\""<<endl;
    return  MCLNDTTypePtr(new MCLNDTType(mcl_ndt_par));
  }
  else if(RegLocalisationParamPtr localisation_reg_ptr=boost::dynamic_pointer_cast<RegLocalisationParam>(param)){
    cout<<"LocalisationFactory: Creating object of type: \""<<reg_localisation_name<<"\""<<endl;
    return  RegLocalisationTypePtr(new RegLocalisationType(localisation_reg_ptr));
  }
  else{
    std::cerr<<"LocalisationFactory: No localisation type exists for parameters"<<endl;
    exit(0);
    return NULL;
  }
}
}
}
