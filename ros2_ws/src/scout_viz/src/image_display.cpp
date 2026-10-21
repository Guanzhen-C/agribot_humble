#include <message_filters/connection.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/displays/image/image_display.hpp>

namespace scout_viz
{
class ImageDisplay : public rviz_default_plugins::displays::ImageDisplay
{
public:
  ~ImageDisplay() override { unsubscribe(); }

protected:
  void unsubscribe() override
  {
    // Humble's Connection retains a callback into the filter after disconnect().
    // Clear that callback while the filter still exists, including on disable.
    if (subscription_) {
      subscription_callback_.disconnect();
    }
    subscription_callback_ = message_filters::Connection();
    subscription_.reset();
  }

  void onDisable() override
  {
    unsubscribe();
    rviz_default_plugins::displays::ImageDisplay::onDisable();
  }

  void onEnable() override
  {
    unsubscribe();
    rviz_default_plugins::displays::ImageDisplay::onEnable();
  }
};
}  // namespace scout_viz

PLUGINLIB_EXPORT_CLASS(scout_viz::ImageDisplay, rviz_common::Display)
