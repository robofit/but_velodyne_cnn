#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <but_velodyne/VelodynePointCloud.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "annotationarea.h"
#include "ringfiller.h"
#include "labeling.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:

    explicit PCLViewer (QWidget *parent, const but_velodyne::VelodynePointCloud &cloud_,
                        std::vector<int> &init_annotation_, const RingFiller &ringFiller_);
    ~PCLViewer ();

    void setAnotator(AnnotationArea *annotator_) {
      annotator = annotator_;
    }

    void discardAnnotation() {
        discard_annotation = true;
    }

    void showAnnotator(AnnotationArea::MODE inMode);

    void processAnnotation(const QImage &mask);

    void displayAnnotation();

    std::vector<Eigen::Vector2f> get2dPoints();

    void saveAnnotationIfNotDiscarted(const std::string &out_filename);

    void setTitle(std::string title, std::string subtitle);

    void setTitleFor(AnnotationContext context);

public slots:

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  const but_velodyne::VelodynePointCloud &cloud;
  AnnotationArea *annotator;
  std::vector<int> annotation;
  bool discard_annotation;
  const RingFiller &ringFiller;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
