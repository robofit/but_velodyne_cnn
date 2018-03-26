#include "ringfiller.h"

#include <pcl/point_types.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;
using namespace velodyne_pointcloud;

void RingFiller::floodAnnotation(const vector<int> &newly_marked_ids,
                                 AnnotationArea::MODE mode, vector<int> &annotation) const {
    for(vector<int>::const_iterator marked_id = newly_marked_ids.begin(); marked_id < newly_marked_ids.end(); marked_id++) {
        int ring = cloud_to_ring_indices[*marked_id] % VelodynePointCloud::VELODYNE_RINGS_COUNT;
        int in_ring_id = cloud_to_ring_indices[*marked_id] / VelodynePointCloud::VELODYNE_RINGS_COUNT;
        for(int i = in_ring_id-1; i >= 0; i--) {
            if(annotation[ring_to_cloud_indices[ring][i]] == annotation[*marked_id] ||
                    !checkAndFillNext(ring, i, i+1, in_ring_id, mode, annotation)) {
                break;
            }
        }
        for(int i = in_ring_id+1; i < rings[ring].size(); i++) {
            if(annotation[ring_to_cloud_indices[ring][i]] == annotation[*marked_id] ||
                    !checkAndFillNext(ring, i, i-1, in_ring_id, mode, annotation)) {
                break;
            }
        }
    }
}

bool RingFiller::checkAndFillNext(int ring, int pt_id, int prev_id, int seed_id,
                                  AnnotationArea::MODE mode, vector<int> &annotation) const {
  PointXYZIR pt = rings[ring][pt_id];
  PointXYZIR prev = rings[ring][prev_id];
  PointXYZIR seed = rings[ring][seed_id];
  if(pt.y-prev.y < properties.max_neighbour_height_diff &&
  pt.y - seed.y < properties.max_total_height_diff &&
  computeRange(pt - prev) < properties.max_neighbour_dist) {
    annotation[ring_to_cloud_indices[ring][pt_id]] = (mode == AnnotationArea::ADD) ? 1 : 0;
    return true;
  } else {
    return false;
  }
}

