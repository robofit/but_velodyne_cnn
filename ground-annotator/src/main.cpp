#include "pclviewer.h"
#include "annotationarea.h"
#include "ringfiller.h"
#include "labeling.h"

#include <QApplication>
#include <QMainWindow>
#include <boost/program_options.hpp>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace but_velodyne;
using namespace std;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     string &input, string &output,
                     bool &use_input_annotation, bool &do_estimation) {
  po::options_description desc("Annotation tool for ground detection.\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("input,i", po::value<string>(&input)->required(), "Input pointcloud to annotate")
      ("output,o", po::value<string>(&output)->required(), "Output annotation file.")
      ("load-ann,l", po::value<bool>(&use_input_annotation)->default_value(true), "Use content of output file (if available) for initialization.")
      ("estimate,e", po::bool_switch(&do_estimation), "Run ground estimation for initialization.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line (argc, argv, desc);
  po::store (parsed, vm);

  if (vm.count ("help")) {
    std::cerr << desc << std::endl;
    return false;
  }

  try {
    po::notify (vm);
  }
  catch (std::exception& e) {
    std::cerr << "Error: " << e.what () << std::endl << std::endl << desc
    << std::endl;
    return false;
  }

  return true;
}

bool isWrittable(const string &filename) {
    ofstream test_file(filename.c_str(), fstream::out | fstream::app);
    return test_file.is_open();
}

vector<int> estimateGround(const VelodynePointCloud &cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(cloud.getXYZCloudPtr());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }
    return inliers->indices;
}

int main (int argc, char *argv[])
{
    string cloud_filename, out_filename;
    bool use_input_annotation;
    bool do_estimation;
    if(!parse_arguments(argc, argv, cloud_filename, out_filename, use_input_annotation, do_estimation)) {
        return EXIT_FAILURE;
    }

    VelodynePointCloud cloud;
    but_velodyne::VelodynePointCloud::fromFile(cloud_filename, cloud);

    RingFiller ring_filler(cloud);
    vector<int> init_annotation(cloud.size(), 0);

    if(do_estimation) {
        vector<int> inliers = estimateGround(cloud);
        for(vector<int>::iterator in = inliers.begin(); in < inliers.end(); in++) {
            init_annotation[*in] = 1;
        }
        ring_filler.floodAnnotation(inliers, AnnotationArea::ADD, init_annotation);
        use_input_annotation = false;
    }

    if(use_input_annotation) {
        ifstream ann_file(out_filename.c_str());
        if(ann_file.is_open()) {
            for(int i = 0; i < cloud.size(); i++) {
                ann_file >> init_annotation[i];
            }
        } else {
            perror(("Unable to read: '" + out_filename + "'").c_str());
        }
    }

    if(!isWrittable(out_filename)) {
        perror(("Unable to write to: '" + out_filename + "'").c_str());
        return EXIT_FAILURE;
    }

    QApplication a (argc, argv);
    PCLViewer w(NULL, cloud, init_annotation, ring_filler);
    AnnotationArea sa(&w);
    w.setAnotator(&sa);
    w.show ();
    sa.hide();

    w.setTitleFor(MOVE);

    int retval = a.exec();

    w.saveAnnotationIfNotDiscarted(out_filename);

    return retval;
}
