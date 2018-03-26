#ifndef RINGFILLER_H
#define RINGFILLER_H

#include <but_velodyne/VelodynePointCloud.h>

#include "annotationarea.h"

class RingFiller {
public:
    class Properties {
    public:
      Properties(
        float max_neighbour_height_diff_ = 0.03,
        float max_neighbour_dist_ = 0.5,
        float max_total_height_diff_ = 0.07) :
            max_neighbour_height_diff(max_neighbour_height_diff_),
            max_neighbour_dist(max_neighbour_dist_),
            max_total_height_diff(max_total_height_diff_) {
      }
      float max_neighbour_height_diff;
      float max_neighbour_dist;
      float max_total_height_diff;
    } properties;

    RingFiller(const but_velodyne::VelodynePointCloud &cloud_) :
        cloud(cloud_) {
        cloud.getRings(rings, ring_to_cloud_indices, cloud_to_ring_indices);
    }

    void floodAnnotation(const std::vector<int> &newly_marked_ids, AnnotationArea::MODE mode, std::vector<int> &annotation) const;

protected:
    bool checkAndFillNext(int ring, int pt_id, int prev_id, int seed_id, AnnotationArea::MODE mode, std::vector<int> &annotation) const;

private:
    const but_velodyne::VelodynePointCloud &cloud;
    std::vector< std::vector<velodyne_pointcloud::PointXYZIR> > rings;
    std::vector< std::vector<int> > ring_to_cloud_indices;
    std::vector<int> cloud_to_ring_indices;
};

#endif // RINGFILLER_H
