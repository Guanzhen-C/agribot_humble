#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main()
{
  using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;
  Cloud::Ptr input(new Cloud);
  for (int i = 0; i < 500; ++i) {
    pcl::PointXYZINormal point{};
    point.x = i * 0.03f;
    point.y = (i % 7) * 0.03f;
    input->push_back(point);
  }
  // Exercise allocation in system PCL and destruction in this executable.
  Cloud output;
  pcl::VoxelGrid<pcl::PointXYZINormal> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(0.2f, 0.2f, 0.2f);
  filter.filter(output);
  return output.empty() || output.size() >= input->size() ? 1 : 0;
}
