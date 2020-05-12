#ifndef NDT_VECTORMAPS_DISPLAY_H_04042017
#define NDT_VECTORMAPS_DISPLAY_H_04042017

#include <boost/circular_buffer.hpp>

#include <ndt_map/NDTMapMsg.h>
#include <rviz/message_filter_display.h>
#include "auto_complete_graph/ACGMaps.h"
#include "ndt_map/NDTVectorMapMsg.h"

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
  class ColorProperty;
  class FloatProperty;
  class IntProperty;
}

namespace perception_oru{
class NDTLineVisual;	
}

namespace AASS{
namespace acg{

  

  class ACGMapsDisplay : public rviz::MessageFilterDisplay<auto_complete_graph::ACGMaps>{
    Q_OBJECT
    public:

    ACGMapsDisplay();
    virtual ~ACGMapsDisplay();

  protected:
    virtual void onInitialize();

    virtual void reset();

  private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();

  private:
    void processMessage(const auto_complete_graph::ACGMaps::ConstPtr& msg);
	
	void processVectorMaps(const ndt_map::NDTVectorMapMsg& msg);

    std::deque<boost::shared_ptr<perception_oru::NDTLineVisual> > visuals_;

    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::IntProperty* history_length_property_;
  };
}
}

#endif 

