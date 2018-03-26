#include "pclviewer.h"
#include "ringfiller.h"
#include "labeling.h"
#include "../build/ui_pclviewer.h"

#include <but_velodyne/Visualizer3D.h>

#include <QMouseEvent>

using namespace but_velodyne;
using namespace velodyne_pointcloud;
using namespace pcl;
using namespace std;

void visualizerMouseCallback(const pcl::visualization::MouseEvent &event, void *cookie) {
    if(event.getButton() == pcl::visualization::MouseEvent::RightButton &&
            event.getType() == pcl::visualization::MouseEvent::MouseButtonPress) {
        PCLViewer *viewer = (PCLViewer*)(cookie);
        viewer->setTitleFor(MARK_GROUND);
        viewer->showAnnotator(AnnotationArea::ADD);
    }
}

void visualizerKeyCallback(const pcl::visualization::KeyboardEvent &event, void *cookie) {
    PCLViewer *viewer = (PCLViewer*)(cookie);

    if(event.keyUp()) {
        if(event.getKeySym() == "b") {
            viewer->setTitleFor(CORRECTION);
            viewer->showAnnotator(AnnotationArea::REMOVE);
        } else if(event.getKeySym() == "q" || event.getKeySym() == "e") {
            QApplication::quit();
        } else if(event.getKeySym() == "d") {
            viewer->discardAnnotation();
            QApplication::quit();
        }
    }
}

PCLViewer::PCLViewer (QWidget *parent, const VelodynePointCloud &cloud_, std::vector<int> &init_annotation_,
                      const RingFiller &ringFiller_) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer),
  annotator(NULL),
  cloud(cloud_),
  annotation(init_annotation_),
  discard_annotation(false),
  ringFiller(ringFiller_)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    displayAnnotation();
    viewer->registerMouseCallback(visualizerMouseCallback, (void*)this);
    viewer->registerKeyboardCallback(visualizerKeyCallback, (void*)this);
    viewer->getInteractorStyle();
    viewer->resetCamera ();
    viewer->setCameraPosition(50, -250, 300, -0.15, -0.77, -0.62);
    ui->qvtkWidget->update ();
}

void PCLViewer::showAnnotator(AnnotationArea::MODE inMode)
{
    QPixmap pixmap(size());
    this->render(&pixmap);
    QImage screen = pixmap.toImage();

    if (annotator != NULL) {
        annotator->setMode(inMode);
        annotator->setImage(screen);
        annotator->show();
    }
}

void PCLViewer::processAnnotation(const QImage &mask)
{
    vector<Eigen::Vector2f> points_2d = get2dPoints();
    vector<int> newly_marked_ids;
    for(int i = 0; i < points_2d.size(); i++) {
        Eigen::Vector2f &pt2D = points_2d[i];
        if(pt2D.x() >= 0 && pt2D.x() < this->geometry().width() &&
                pt2D.y() >= 0 && pt2D.y() < this->geometry().height()) {
            QRgb pixel_val = mask.pixel(pt2D.x(), pt2D.y());
            if(qRed(pixel_val) != 0 && qGreen(pixel_val) != 0 && qBlue(pixel_val) != 0) {
                annotation[i] = annotator->getMode() == AnnotationArea::ADD ? 1 : 0;
                newly_marked_ids.push_back(i);
            }
        }
    }
    ringFiller.floodAnnotation(newly_marked_ids, annotator->getMode(), annotation);
    displayAnnotation();
    setTitleFor(MOVE);
}

void PCLViewer::displayAnnotation() {
    string cloud_name = "cloud";

    viewer->removePointCloud(cloud_name);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0; i < cloud.size(); i++) {
        pcl::PointXYZRGB colored_pt;
        but_velodyne::copyXYZ(cloud[i], colored_pt);
        colored_pt.r = colored_pt.g = colored_pt.b = 0;
        switch(annotation[i]) {
        case 1:
            colored_pt.r = 255; break;
        case 2:
            colored_pt.g = 255; break;
        default:
            colored_pt.b = colored_pt.g = colored_pt.r = 200;
        }
        colored_cloud->push_back(colored_pt);
    }
    viewer->addPointCloud(colored_cloud, cloud_name);
}

std::vector<Eigen::Vector2f> PCLViewer::get2dPoints() {
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    Eigen::Matrix4d projection;
    camera.computeProjectionMatrix(projection);
    Eigen::Matrix4d view;
    camera.computeViewMatrix(view);

    but_velodyne::VelodynePointCloud cloud_transformed;
    pcl::transformPointCloud(cloud, cloud_transformed, view.cast<float>());

    std::vector<Eigen::Vector2f> points_2d;
    for(int i = 0; i < cloud.size(); i++) {
        Eigen::Vector4d pt = cloud_transformed[i].getVector4fMap().cast<double>();
        Eigen::Vector4d projected = projection*pt;
        projected /= projected(3);
        Eigen::Vector2f pt2D((projected.x() + 1.0) / 2.0 * this->geometry().width(),
                             (-projected.y() + 1.0) / 2.0 * this->geometry().height());
        points_2d.push_back(pt2D);
    }
    return points_2d;
}

void PCLViewer::saveAnnotationIfNotDiscarted(const string &out_filename) {
    if(!discard_annotation) {
        ofstream out_file(out_filename.c_str());
        for(vector<int>::const_iterator a = annotation.begin(); a < annotation.end(); a++) {
          out_file << *a << endl;
        }
    }
}

void PCLViewer::setTitle(string title, string subtitle) {
    static int margin = 10;
    static int font_title = 20;
    static int font_subtitle = 15;
    static int x = margin;
    static int y = this->size().height() - margin - font_title;
    static double red = 1.0;
    static double green = 1.0;
    static double blue = 1.0;
    string title_id = "title";
    string subtitle_id = "subtitle";
    viewer->removeShape(title_id);
    viewer->removeShape(subtitle_id);
    viewer->addText(title, x, y, font_title, red, green, blue, title_id);
    viewer->addText(subtitle, x, y-font_subtitle-margin, font_subtitle, red, green, blue, subtitle_id);
    ui->qvtkWidget->update();
}

void PCLViewer::setTitleFor(AnnotationContext context) {
    switch(context) {
    case MOVE:
        setTitle("Move/rotate the scene using left/middle-click or mouse wheel.",
                 "Use right-click to start marking the ground or \"B\" to revert the incorrect annotation.");
        break;
    case MARK_GROUND:
        setTitle("Mark the ground area by left-click",
                 "After button release the info will spread to the neighbourhood");
        break;
    case CORRECTION:
        setTitle("Correct the non-ground area using left-click",
                 "After button release the info will spread to the neighbourhood");
        break;
    }
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
